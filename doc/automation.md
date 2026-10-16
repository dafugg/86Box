# Automation socket

Build with `-D AUTOMATION=ON`, then start 86Box with:

```
86Box --automation /path/to/socket [--automation-hold] ...
```

86Box listens on that UNIX domain socket and accepts one client at a time.
`--automation-hold` starts the machine **held**: no emulated time passes until
a client asks for it. That makes runs repeatable and lets a slow client (a
test harness, a script, an AI agent) take as long as it needs between
moves.

The socket is not supported on Windows yet.

## Protocol (version 1)

Each request is one line of space-separated words. Each response is one line
of JSON with `"ok": true` or `"ok": false, "error": "..."`. Requests run on
the emulation thread between 1 ms quanta (`pc_run()` calls), so each request
sees the machine at a consistent point. Requests are handled one at a time,
in order.

Responses to most commands include `ms` (emulated milliseconds since
startup), `frames` (frames rendered) and `held`.

| Request | Effect |
|---|---|
| `status` | Protocol and emulator version, `ms`, `frames`, `held`, `paused` (UI pause), last frame size `w`/`h`, `bios_mode` (BIOS data area 0x449), `quantum_ms`. |
| `hold` | Stop running the CPU between requests. |
| `release` | Resume normal real-time execution. |
| `step <ms>` | Run exactly `ms` of emulated time, then reply. If held, runs without real-time pacing (as fast as the host allows), then holds again. |
| `wait text <timeout_ms> <text>` | Run until `<text>` (rest of line, matched against raw screen bytes) appears in a single row of the text screen, or until the timeout. Reply has `matched`. |
| `wait mem <timeout_ms> <addr> <hex>` | Run until physical memory at `addr` equals the hex bytes, or until the timeout. |
| `key down\|up <scancode>...` | Press or release keys. Scancodes are set-1 make codes; add `0x100` for E0-prefixed keys (e.g. `0x148` = Up). Events are delivered 2 ms of emulated time apart (a burst would overflow the keyboard FIFO); the reply comes after the last event plus its 2 ms, so a request consumes 2 ms per key. |
| `mouse <dx> <dy> <buttons>` | Relative motion; buttons bitmask 1=left, 2=right, 4=middle. |
| `screenshot <path>` | Write the most recent frame of monitor 0 as a PNG file. Reply has `w`, `h`. |
| `text` | Text-mode screen via the BIOS data area: `mode`, `cols`, `rows`, `cursor` [row, col], and `chars`/`attrs` as one hex string per row (CP437). Fails in graphics modes. |
| `memr <addr> <len>` | Read up to 65536 bytes of physical memory; `data` is hex. |
| `memw <addr> <hex>` | Write bytes to physical memory. |
| `regs` | General registers, `eip`, `eflags`, segment selectors, `cs_base`, `cr0`. |
| `fdd load <drive> <path>` / `fdd eject <drive>` | Change floppy media (drive 0 = A:). |
| `cd load <drive> <path>` / `cd eject <drive>` | Change CD-ROM media. |
| `reset hard` / `reset cad` | Hard reset, or send Ctrl+Alt+Del. |
| `quit` | Power off and exit 86Box. |

Numbers accept C syntax (`0x` hex, decimal).

`wait`, `memr` and `text` read memory through the normal physical memory map,
so a read in a memory-mapped device range may have side effects (for example,
VGA latches).

Try it by hand:

```
$ socat - UNIX-CONNECT:/tmp/86box.sock
status
step 1000
screenshot /tmp/frame.png
```

## Determinism notes

- While held, emulated time is fully under the client's control. Disable
  "Synchronize time with host" (`time_sync = disabled`) so the guest RTC does
  not follow the host clock.
- Devices that run their own host threads or use host timers (some sound
  devices, networking, passthrough) can still introduce variation.
