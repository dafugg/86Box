/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Automation control socket.
 *
 *          A listener thread accepts one client at a time on a UNIX domain
 *          socket. Each request is one line of space-separated words; each
 *          response is one line of JSON. Requests are handed to the CPU
 *          thread, which executes them between pc_run() calls, so every
 *          command observes the machine at a quantum boundary. Commands
 *          that consume emulated time (step, wait) complete on a later
 *          quantum. See doc/automation.md for the protocol.
 *
 * Authors: 86box-mcp contributors
 *
 *          Copyright 2026 86box-mcp contributors.
 */
#include <stdarg.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <png.h>
#ifndef _WIN32
#    include <errno.h>
#    include <sys/socket.h>
#    include <sys/un.h>
#    include <unistd.h>
#endif
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/fdd.h>
#include <86box/cdrom.h>
#include <86box/keyboard.h>
#include <86box/mem.h>
#include <86box/mouse.h>
#include <86box/plat.h>
#include <86box/thread.h>
#include <86box/ui.h>
#include <86box/video.h>
#include <86box/version.h>
#include <86box/automation.h>

#define AUTOMATION_PROTOCOL 1
#define REQ_MAX             4096
#define MEM_MAX             65536
#define KEY_QUEUE_MAX       64
/* Emulated ms between injected key events. One event can be up to six bytes
   in scan code set 2 (fake shifts around extended keys); delivering many at
   the same instant overflows the keyboard's 16-byte FIFO and loses keys. */
#define KEY_GAP_MS          2

#ifdef ENABLE_AUTOMATION_LOG
int automation_do_log = ENABLE_AUTOMATION_LOG;

static void
automation_log(const char *fmt, ...)
{
    va_list ap;

    if (automation_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define automation_log(fmt, ...)
#endif

enum {
    CMD_NONE = 0,
    CMD_STEP,
    CMD_WAIT_TEXT,
    CMD_WAIT_MEM,
    CMD_KEYS
};

typedef struct {
    char  *buf;
    size_t len;
    size_t size;
} strbuf_t;

char automation_path[1024] = "";
int  automation_start_held = 0;

static int       enabled;
static thread_t *listen_thread;
static int       listen_fd = -1;
static int       client_fd = -1;
static event_t  *req_done;

/* Mailbox between the listener thread and the CPU thread. */
static char       req_line[REQ_MAX];
static atomic_int req_pending;
static strbuf_t   resp;

/* CPU thread state. */
static int      held;
static int      cmd;              /* long-running command in progress */
static int      cmd_was_held;     /* hold again once the command finishes */
static uint64_t cmd_deadline;     /* emulated ms at which step/wait ends */
static char     wait_text[REQ_MAX];
static uint32_t wait_addr;
static uint8_t  wait_bytes[256];
static int      wait_len;
static uint64_t emu_ms;           /* emulated ms executed since startup */

static struct {
    uint16_t scan;
    uint8_t  down;
} key_queue[KEY_QUEUE_MAX];
static int      key_head;
static int      key_count;
static uint64_t key_next_ms;      /* emulated ms at which the next key event is due */

/* Most recent frame handed to the renderer for monitor 0. */
static mutex_t  *frame_mutex;
static uint32_t *frame_buf;
static int       frame_w;
static int       frame_h;
static int       frame_alloc;
static uint64_t  frame_count;

static void
sb_printf(strbuf_t *sb, const char *fmt, ...)
{
    va_list ap;
    int     n;

    for (;;) {
        va_start(ap, fmt);
        n = vsnprintf(sb->buf + sb->len, sb->size - sb->len, fmt, ap);
        va_end(ap);
        if ((n >= 0) && ((size_t) n < (sb->size - sb->len))) {
            sb->len += n;
            return;
        }
        sb->size = (sb->size + n + 1) * 2;
        sb->buf  = realloc(sb->buf, sb->size);
    }
}

static void
sb_hex(strbuf_t *sb, const uint8_t *data, int len)
{
    static const char digits[] = "0123456789abcdef";

    if ((sb->len + (len * 2) + 1) > sb->size) {
        sb->size = (sb->len + (len * 2) + 1) * 2;
        sb->buf  = realloc(sb->buf, sb->size);
    }
    for (int i = 0; i < len; i++) {
        sb->buf[sb->len++] = digits[data[i] >> 4];
        sb->buf[sb->len++] = digits[data[i] & 0x0f];
    }
    sb->buf[sb->len] = '\0';
}

static void
sb_json_string(strbuf_t *sb, const char *s)
{
    sb_printf(sb, "\"");
    for (; *s; s++) {
        if ((*s == '"') || (*s == '\\'))
            sb_printf(sb, "\\%c", *s);
        else if ((uint8_t) *s < 0x20)
            sb_printf(sb, "\\u%04x", (uint8_t) *s);
        else
            sb_printf(sb, "%c", *s);
    }
    sb_printf(sb, "\"");
}

static void
respond_error(const char *msg)
{
    resp.len = 0;
    sb_printf(&resp, "{\"ok\":false,\"error\":");
    sb_json_string(&resp, msg);
    sb_printf(&resp, "}");
}

/* Finish the request in the mailbox and wake the listener thread. */
static void
complete(void)
{
    atomic_store(&req_pending, 0);
    thread_set_event(req_done);
}

static int
parse_hex_bytes(const char *s, uint8_t *out, int max)
{
    int n = 0;

    while (s[0] && s[1] && (n < max)) {
        unsigned int v;
        if (sscanf(s, "%2x", &v) != 1)
            return -1;
        out[n++] = v;
        s += 2;
    }
    return s[0] ? -1 : n;
}

/* ---- Text screen, via the BIOS data area ---- */

typedef struct {
    int      mode;
    int      cols;
    int      rows;
    int      cur_row;
    int      cur_col;
    uint32_t base;
} text_info_t;

static int
text_info(text_info_t *ti)
{
    int page;

    ti->mode = mem_readb_phys(0x449) & 0x7f;
    if ((ti->mode > 3) && (ti->mode != 7))
        return 0;
    ti->cols = mem_readb_phys(0x44a) | (mem_readb_phys(0x44b) << 8);
    ti->rows = mem_readb_phys(0x484) + 1;
    if ((ti->cols < 40) || (ti->cols > 132))
        ti->cols = 80;
    if ((ti->rows < 25) || (ti->rows > 60))
        ti->rows = 25;
    page        = mem_readb_phys(0x462) & 7;
    ti->cur_col = mem_readb_phys(0x450 + (page * 2));
    ti->cur_row = mem_readb_phys(0x451 + (page * 2));
    ti->base    = ((ti->mode == 7) ? 0xb0000 : 0xb8000) + (mem_readb_phys(0x44e) | (mem_readb_phys(0x44f) << 8));
    return 1;
}

static int
text_contains(const char *needle)
{
    text_info_t ti;
    char        line[133];
    size_t      nlen = strlen(needle);

    if (!text_info(&ti))
        return 0;
    /* Matches within a single row only. */
    for (int r = 0; r < ti.rows; r++) {
        for (int c = 0; c < ti.cols; c++)
            line[c] = mem_readb_phys(ti.base + (((r * ti.cols) + c) * 2));
        line[ti.cols] = '\0';
        for (size_t c = 0; (c + nlen) <= (size_t) ti.cols; c++) {
            if (!memcmp(line + c, needle, nlen))
                return 1;
        }
    }
    return 0;
}

static void
cmd_text(void)
{
    text_info_t ti;
    uint8_t     chars[132];
    uint8_t     attrs[132];

    if (!text_info(&ti)) {
        resp.len = 0;
        sb_printf(&resp, "{\"ok\":false,\"error\":\"not a text mode\",\"mode\":%d}", ti.mode);
        return;
    }
    resp.len = 0;
    sb_printf(&resp, "{\"ok\":true,\"mode\":%d,\"cols\":%d,\"rows\":%d,\"cursor\":[%d,%d],\"chars\":[",
              ti.mode, ti.cols, ti.rows, ti.cur_row, ti.cur_col);
    for (int r = 0; r < ti.rows; r++) {
        for (int c = 0; c < ti.cols; c++)
            chars[c] = mem_readb_phys(ti.base + (((r * ti.cols) + c) * 2));
        sb_printf(&resp, r ? ",\"" : "\"");
        sb_hex(&resp, chars, ti.cols);
        sb_printf(&resp, "\"");
    }
    sb_printf(&resp, "],\"attrs\":[");
    for (int r = 0; r < ti.rows; r++) {
        for (int c = 0; c < ti.cols; c++)
            attrs[c] = mem_readb_phys(ti.base + (((r * ti.cols) + c) * 2) + 1);
        sb_printf(&resp, r ? ",\"" : "\"");
        sb_hex(&resp, attrs, ti.cols);
        sb_printf(&resp, "\"");
    }
    sb_printf(&resp, "]}");
}

/* ---- Screenshots ---- */

static int
write_png(const char *fn)
{
    png_structp png;
    png_infop   info;
    png_bytep   row;
    FILE       *fp;

    if (!frame_buf || !frame_w || !frame_h)
        return 0;

    fp = plat_fopen(fn, "wb");
    if (!fp)
        return 0;

    png  = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    info = png ? png_create_info_struct(png) : NULL;
    if (!info || setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, info ? &info : NULL);
        fclose(fp);
        return 0;
    }

    png_init_io(png, fp);
    /* Speed matters more than size for a local socket consumer. */
    png_set_compression_level(png, 1);
    png_set_IHDR(png, info, frame_w, frame_h, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
    png_write_info(png, info);

    row = malloc(frame_w * 3);
    for (int y = 0; y < frame_h; y++) {
        const uint32_t *src = &frame_buf[y * frame_w];
        for (int x = 0; x < frame_w; x++) {
            row[(x * 3)]     = (src[x] >> 16) & 0xff;
            row[(x * 3) + 1] = (src[x] >> 8) & 0xff;
            row[(x * 3) + 2] = src[x] & 0xff;
        }
        png_write_row(png, row);
    }
    free(row);

    png_write_end(png, NULL);
    png_destroy_write_struct(&png, &info);
    fclose(fp);
    return 1;
}

void
automation_frame(int x, int y, int w, int h, int monitor_index)
{
    bitmap_t *target;

    if (!enabled || (monitor_index != 0) || (w <= 0) || (h <= 0))
        return;

    target = monitors[0].target_buffer;
    if (!target)
        return;

    thread_wait_mutex(frame_mutex);
    if ((w * h) > frame_alloc) {
        frame_alloc = w * h;
        frame_buf   = realloc(frame_buf, frame_alloc * sizeof(uint32_t));
    }
    for (int row = 0; row < h; row++)
        memcpy(&frame_buf[row * w], &target->line[y + row][x], w * sizeof(uint32_t));
    frame_w = w;
    frame_h = h;
    frame_count++;
    thread_release_mutex(frame_mutex);
}

/* ---- Request handling (CPU thread) ---- */

static void
respond_ok_time(void)
{
    resp.len = 0;
    sb_printf(&resp, "{\"ok\":true,\"ms\":%llu,\"frames\":%llu,\"held\":%s}",
              (unsigned long long) emu_ms, (unsigned long long) frame_count, held ? "true" : "false");
}

/* Start a command that consumes emulated time. */
static void
begin_timed(int which, uint64_t ms)
{
    cmd          = which;
    cmd_was_held = held;
    cmd_deadline = emu_ms + ms;
    held         = 0;
}

static void
end_timed(int matched)
{
    cmd  = CMD_NONE;
    held = cmd_was_held;
    resp.len = 0;
    sb_printf(&resp, "{\"ok\":true,\"ms\":%llu,\"frames\":%llu,\"held\":%s",
              (unsigned long long) emu_ms, (unsigned long long) frame_count, held ? "true" : "false");
    if (matched >= 0)
        sb_printf(&resp, ",\"matched\":%s", matched ? "true" : "false");
    sb_printf(&resp, "}");
    complete();
}

/* Returns 1 when the condition of a wait command is satisfied. */
static int
wait_satisfied(void)
{
    if (cmd == CMD_WAIT_TEXT)
        return text_contains(wait_text);

    if (cmd == CMD_WAIT_MEM) {
        for (int i = 0; i < wait_len; i++) {
            if (mem_readb_phys(wait_addr + i) != wait_bytes[i])
                return 0;
        }
        return 1;
    }

    return 0;
}

static void
handle_request(char *line)
{
    static char copy[REQ_MAX];
    char       *argv[16];
    char       *rest[16]; /* rest[i] = unsplit text from argv[i] onwards */
    int         argc = 0;
    char       *p    = line;

    strcpy(copy, line);
    while (*p && (argc < 16)) {
        while (*p == ' ')
            p++;
        if (!*p)
            break;
        argv[argc] = p;
        rest[argc] = copy + (p - line);
        argc++;
        while (*p && (*p != ' '))
            p++;
        if (*p)
            *p++ = '\0';
    }

    automation_log("Automation: %s\n", copy);

    if (argc == 0) {
        respond_error("empty request");
        complete();
        return;
    }

    if (!strcmp(argv[0], "status")) {
        int mode = mem_readb_phys(0x449);
        resp.len = 0;
        sb_printf(&resp, "{\"ok\":true,\"protocol\":%d,\"version\":", AUTOMATION_PROTOCOL);
        sb_json_string(&resp, EMU_VERSION_FULL);
        sb_printf(&resp, ",\"ms\":%llu,\"frames\":%llu,\"held\":%s,\"paused\":%s,\"w\":%d,\"h\":%d,\"bios_mode\":%d,\"quantum_ms\":%d}",
                  (unsigned long long) emu_ms, (unsigned long long) frame_count, held ? "true" : "false",
                  dopause ? "true" : "false", frame_w, frame_h, mode, force_10ms ? 10 : 1);
    } else if (!strcmp(argv[0], "hold")) {
        held = 1;
        respond_ok_time();
    } else if (!strcmp(argv[0], "release")) {
        held = 0;
        respond_ok_time();
    } else if (!strcmp(argv[0], "step") && (argc == 2)) {
        uint64_t ms = strtoull(argv[1], NULL, 0);
        if (!ms) {
            respond_ok_time();
        } else {
            begin_timed(CMD_STEP, ms);
            return; /* completed by automation_quantum() */
        }
    } else if (!strcmp(argv[0], "wait") && (argc >= 4) && !strcmp(argv[1], "text")) {
        /* wait text <timeout_ms> <text...> */
        strncpy(wait_text, rest[3], sizeof(wait_text) - 1);
        begin_timed(CMD_WAIT_TEXT, strtoull(argv[2], NULL, 0));
        if (wait_satisfied())
            end_timed(1);
        return;
    } else if (!strcmp(argv[0], "wait") && (argc == 5) && !strcmp(argv[1], "mem")) {
        /* wait mem <timeout_ms> <addr> <hexbytes> */
        wait_addr = strtoul(argv[3], NULL, 0);
        wait_len  = parse_hex_bytes(argv[4], wait_bytes, sizeof(wait_bytes));
        if (wait_len <= 0) {
            respond_error("bad hex bytes");
        } else {
            begin_timed(CMD_WAIT_MEM, strtoull(argv[2], NULL, 0));
            if (wait_satisfied())
                end_timed(1);
            return;
        }
    } else if (!strcmp(argv[0], "key") && (argc >= 3)) {
        /* key down|up <scancode>... ; scancodes in 86Box form, 0x1xx = E0 prefix.
           Events are delivered KEY_GAP_MS apart; the reply comes once the last
           one has had its gap, so back-to-back requests cannot flood the FIFO. */
        int down = !strcmp(argv[1], "down");
        if (!down && strcmp(argv[1], "up")) {
            respond_error("expected down or up");
        } else if ((argc - 2) > KEY_QUEUE_MAX) {
            respond_error("too many keys in one request");
        } else {
            key_head  = 0;
            key_count = 0;
            for (int i = 2; i < argc; i++) {
                key_queue[key_count].scan   = (uint16_t) strtoul(argv[i], NULL, 0);
                key_queue[key_count++].down = down;
            }
            begin_timed(CMD_KEYS, UINT64_MAX - emu_ms);
            keyboard_input(key_queue[0].down, key_queue[0].scan);
            key_head    = 1;
            key_next_ms = emu_ms + KEY_GAP_MS;
            return; /* completed by automation_quantum() */
        }
    } else if (!strcmp(argv[0], "mouse") && (argc == 4)) {
        /* mouse <dx> <dy> <buttons> */
        /* Mouse devices discard input unless the host has captured the mouse.
           Set the flag directly: plat_mouse_capture() would grab the host
           pointer, and is not safe to call from the CPU thread. */
        mouse_capture = 1;
        mouse_scale(atoi(argv[1]), atoi(argv[2]));
        mouse_set_buttons_ex(atoi(argv[3]));
        respond_ok_time();
    } else if (!strcmp(argv[0], "screenshot") && (argc >= 2)) {
        int ok;
        thread_wait_mutex(frame_mutex);
        ok = write_png(rest[1]);
        resp.len = 0;
        if (ok)
            sb_printf(&resp, "{\"ok\":true,\"w\":%d,\"h\":%d,\"ms\":%llu,\"frames\":%llu}",
                      frame_w, frame_h, (unsigned long long) emu_ms, (unsigned long long) frame_count);
        thread_release_mutex(frame_mutex);
        if (!ok)
            respond_error(frame_buf ? "could not write file" : "no frame rendered yet");
    } else if (!strcmp(argv[0], "text")) {
        cmd_text();
    } else if (!strcmp(argv[0], "memr") && (argc == 3)) {
        uint32_t addr = strtoul(argv[1], NULL, 0);
        int      len  = atoi(argv[2]);
        uint8_t *buf;
        if ((len <= 0) || (len > MEM_MAX)) {
            respond_error("length out of range");
        } else {
            buf = malloc(len);
            for (int i = 0; i < len; i++)
                buf[i] = mem_readb_phys(addr + i);
            resp.len = 0;
            sb_printf(&resp, "{\"ok\":true,\"addr\":%u,\"data\":\"", addr);
            sb_hex(&resp, buf, len);
            sb_printf(&resp, "\"}");
            free(buf);
        }
    } else if (!strcmp(argv[0], "memw") && (argc == 3)) {
        uint32_t addr = strtoul(argv[1], NULL, 0);
        uint8_t *buf  = malloc(strlen(argv[2]) / 2 + 1);
        int      len  = parse_hex_bytes(argv[2], buf, strlen(argv[2]) / 2 + 1);
        if (len <= 0) {
            respond_error("bad hex bytes");
        } else {
            for (int i = 0; i < len; i++)
                mem_writeb_phys(addr + i, buf[i]);
            respond_ok_time();
        }
        free(buf);
    } else if (!strcmp(argv[0], "regs")) {
        static const char *names[8] = { "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi" };
        resp.len = 0;
        sb_printf(&resp, "{\"ok\":true");
        for (int i = 0; i < 8; i++)
            sb_printf(&resp, ",\"%s\":%u", names[i], cpu_state.regs[i].l);
        sb_printf(&resp, ",\"eip\":%u,\"eflags\":%u,\"cs\":%u,\"ds\":%u,\"es\":%u,\"ss\":%u,\"fs\":%u,\"gs\":%u,\"cs_base\":%u,\"cr0\":%u}",
                  cpu_state.pc, ((uint32_t) cpu_state.eflags << 16) | cpu_state.flags,
                  cpu_state.seg_cs.seg, cpu_state.seg_ds.seg, cpu_state.seg_es.seg,
                  cpu_state.seg_ss.seg, cpu_state.seg_fs.seg, cpu_state.seg_gs.seg,
                  cpu_state.seg_cs.base, cr0);
    } else if (!strcmp(argv[0], "fdd") && (argc >= 3)) {
        /* fdd load <drive> <path> | fdd eject <drive> */
        int id = atoi(argv[2]);
        if ((id < 0) || (id >= FDD_NUM)) {
            respond_error("bad drive");
        } else if (!strcmp(argv[1], "eject")) {
            fdd_close(&drives[id]);
            ui_sb_update_icon_state(SB_FLOPPY | id, 1);
            respond_ok_time();
        } else if (!strcmp(argv[1], "load") && (argc >= 4)) {
            fdd_close(&drives[id]);
            drives[id].read_only = 0;
            fdd_load(&drives[id], rest[3]);
            ui_sb_update_icon_state(SB_FLOPPY | id, drives[id].image_path[0] ? 0 : 1);
            respond_ok_time();
        } else
            respond_error("usage: fdd load <drive> <path> | fdd eject <drive>");
    } else if (!strcmp(argv[0], "cd") && (argc >= 3)) {
        /* cd load <drive> <path> | cd eject <drive> */
        int id = atoi(argv[2]);
        if ((id < 0) || (id >= CDROM_NUM)) {
            respond_error("bad drive");
        } else if (!strcmp(argv[1], "eject")) {
            cdrom_eject(id);
            respond_ok_time();
        } else if (!strcmp(argv[1], "load") && (argc >= 4)) {
            int was_empty = cdrom_is_empty(id);
            cdrom_exit(id);
            memset(cdrom[id].image_path, 0, sizeof(cdrom[id].image_path));
            cdrom_load(&cdrom[id], rest[3], 1);
            /* Signal media change to the emulated machine. */
            if (cdrom[id].insert) {
                cdrom[id].insert(cdrom[id].priv);
                if (was_empty)
                    cdrom[id].insert(cdrom[id].priv);
            }
            ui_sb_update_icon_state(SB_CDROM | id, cdrom[id].image_path[0] ? 0 : 1);
            respond_ok_time();
        } else
            respond_error("usage: cd load <drive> <path> | cd eject <drive>");
    } else if (!strcmp(argv[0], "reset") && (argc == 2) && !strcmp(argv[1], "hard")) {
        pc_reset_hard();
        respond_ok_time();
    } else if (!strcmp(argv[0], "reset") && (argc == 2) && !strcmp(argv[1], "cad")) {
        pc_send_cad();
        respond_ok_time();
    } else if (!strcmp(argv[0], "quit")) {
        respond_ok_time();
        complete();
        plat_power_off();
        return;
    } else {
        respond_error("unknown command or wrong arguments");
    }

    complete();
}

void
automation_poll(void)
{
    if (!enabled || (cmd != CMD_NONE) || !atomic_load(&req_pending))
        return;

    handle_request(req_line);
}

int
automation_run_state(void)
{
    if (!enabled)
        return AUTOMATION_FREE;
    if (cmd != CMD_NONE)
        return cmd_was_held ? AUTOMATION_RUN : AUTOMATION_FREE;
    return held ? AUTOMATION_HOLD : AUTOMATION_FREE;
}

void
automation_quantum(void)
{
    if (!enabled)
        return;

    emu_ms += force_10ms ? 10 : 1;

    if (cmd == CMD_NONE)
        return;

    if (cmd == CMD_KEYS) {
        if (emu_ms < key_next_ms)
            return;
        if (key_head == key_count) {
            end_timed(-1);
            return;
        }
        keyboard_input(key_queue[key_head].down, key_queue[key_head].scan);
        key_head++;
        key_next_ms = emu_ms + KEY_GAP_MS;
    } else if ((cmd == CMD_WAIT_TEXT) || (cmd == CMD_WAIT_MEM)) {
        if (wait_satisfied())
            end_timed(1);
        else if (emu_ms >= cmd_deadline)
            end_timed(0);
    } else if (emu_ms >= cmd_deadline)
        end_timed(-1);
}

/* ---- Listener thread ---- */

#ifndef _WIN32
static int
write_all(int fd, const char *buf, size_t len)
{
    while (len) {
        ssize_t n = write(fd, buf, len);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return 0;
        }
        buf += n;
        len -= n;
    }
    return 1;
}

static void
automation_thread(UNUSED(void *priv))
{
    static char inbuf[REQ_MAX];
    size_t      inlen;

    while (enabled) {
        client_fd = accept(listen_fd, NULL, NULL);
        if (client_fd < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        automation_log("Automation: client connected\n");
        inlen = 0;

        while (enabled) {
            char   *nl;
            ssize_t n = read(client_fd, inbuf + inlen, sizeof(inbuf) - 1 - inlen);
            if (n <= 0) {
                if ((n < 0) && (errno == EINTR))
                    continue;
                break;
            }
            inlen += n;
            inbuf[inlen] = '\0';

            while ((nl = memchr(inbuf, '\n', inlen))) {
                size_t linelen = nl - inbuf;
                *nl = '\0';
                if (linelen && (inbuf[linelen - 1] == '\r'))
                    inbuf[linelen - 1] = '\0';

                strncpy(req_line, inbuf, sizeof(req_line) - 1);
                req_line[sizeof(req_line) - 1] = '\0';
                thread_reset_event(req_done);
                atomic_store(&req_pending, 1);
                thread_wait_event(req_done, -1);

                if (!write_all(client_fd, resp.buf, resp.len) || !write_all(client_fd, "\n", 1))
                    goto disconnect;

                inlen -= linelen + 1;
                memmove(inbuf, nl + 1, inlen);
            }

            if (inlen >= (sizeof(inbuf) - 1)) {
                /* Overlong request: drop it. */
                inlen = 0;
            }
        }
disconnect:
        automation_log("Automation: client disconnected\n");
        close(client_fd);
        client_fd = -1;
    }
}
#endif

void
automation_init(void)
{
#ifdef _WIN32
    if (automation_path[0])
        pclog("Automation: not supported on this platform yet\n");
#else
    struct sockaddr_un addr;

    if (!automation_path[0] || enabled)
        return;

    if (strlen(automation_path) >= sizeof(addr.sun_path)) {
        pclog("Automation: socket path too long: %s\n", automation_path);
        return;
    }

    listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        pclog("Automation: socket() failed: %s\n", strerror(errno));
        return;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, automation_path);
    unlink(automation_path);

    if ((bind(listen_fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) || (listen(listen_fd, 1) < 0)) {
        pclog("Automation: cannot listen on %s: %s\n", automation_path, strerror(errno));
        close(listen_fd);
        listen_fd = -1;
        return;
    }

    resp.size   = 4096;
    resp.buf    = malloc(resp.size);
    resp.len    = 0;
    req_done    = thread_create_event();
    frame_mutex = thread_create_mutex();
    held        = automation_start_held;
    enabled     = 1;

    listen_thread = thread_create_named(automation_thread, NULL, "automation");
    pclog("Automation: listening on %s%s\n", automation_path, held ? " (held)" : "");
#endif
}

void
automation_close(void)
{
#ifndef _WIN32
    if (!enabled)
        return;

    enabled = 0;
    shutdown(listen_fd, SHUT_RDWR);
    close(listen_fd);
    listen_fd = -1;
    if (client_fd >= 0)
        shutdown(client_fd, SHUT_RDWR);
    /* Release a listener that is waiting on the CPU thread. */
    thread_set_event(req_done);
    unlink(automation_path);
#endif
}
