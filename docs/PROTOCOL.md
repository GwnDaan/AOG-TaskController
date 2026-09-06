# AOG-TaskController — Protocol Reference

This document describes the interfaces AOG-TaskController exposes to the rest of the system. The intended audience is anyone writing a client — AgOpenGPS (AgIO), AgValoniaGPS, or a generic ISOBUS controller — that needs to talk to the TC.

For Linux deployment of the TC itself (systemd, CAN setup, the daemon footprint), see [LINUX_DAEMON.md](LINUX_DAEMON.md). For the TC's threading model — which callbacks run on which thread, and what that means for any state you touch — see [CONCURRENCY.md](CONCURRENCY.md).

---

## 1. Overview

AOG-TaskController sits between two worlds:

```
   AgIO / AgValonia                 ISOBUS bus (CAN @ 250 kbps)
       (LAN/UDP)                    (sprayers, planters, TECU)
            │                                  │
            ▼                                  ▼
    ┌───────────────────────────────────────────────┐
    │           AOG-TaskController                  │
    │                                               │
    │  • UDP framing on subnet:8888 / .255:9999     │
    │  • ISO 11783 TaskController (Section Control) │
    │  • Optional ISO 11783-9 Tractor ECU (TECU)    │
    └───────────────────────────────────────────────┘
```

Three things to understand before reading the rest:

1. **All client traffic is UDP.** The TC speaks an AgIO-style framed UDP protocol on the LAN. There is no TCP, REST, or gRPC interface.
2. **The TC is the AgIO/AgValonia *peer*, not its client.** AgIO/AgValonia opens its sockets; the TC also opens its sockets; they exchange framed packets. Either can come up first.
3. **ISO 11783 is the source of truth for the CAN side.** The TC implements the ISOBUS Task Controller (Section Control, Generation 1, ISO 11783-10) using the [AgIsoStack++](https://github.com/Open-Agriculture/AgIsoStack-plus-plus) library. You do not need to know ISOBUS to *talk to the TC* — only to understand what it does on the bus.

---

## 2. UDP wire protocol

### 2.1 Packet framing

Every packet — both directions — uses the same frame:

```
 Offset  Size  Field        Notes
 0       2     Start         0x80 0x81 (big-endian sentinel)
 2       1     Source        Sender's logical address (see 2.2)
 3       1     PGN           Logical message id (see 2.5/2.6)
 4       1     Length        Number of payload bytes (N)
 5       N     Payload       N bytes, depends on PGN
 5+N     1     Checksum      Sum of bytes [Source .. last payload byte], mod 256
```

Total wire size is `N + 6` bytes. The maximum payload is currently 250 bytes (limited by the TC's 512-byte receive buffer). Most PGNs are ≤10 bytes; the outlier is `0xD6` (GPS/IMU data, see §2.5), which needs at least 39 bytes today.

**Checksum**: the TC currently does **not** validate inbound checksums (the verification code is present but commented out in `udp_connections.cpp`). Clients **should still compute and include a correct checksum** so that future TC versions, or third-party listeners, can validate.

### 2.2 Source addresses

Source byte identifies the logical sender of a frame. The conventions used today:

| Source | Logical sender |
|---|---|
| `0x7F` (127) | **AgIO / AgValonia** (the GUI/host application) |
| `0x80` (128) | **AOG-TaskController** itself |
| `0x7C` (124) | **AOG's GPS/IMU submodule** — sends PGN `0xD6` only (see §2.5). |

Other addresses appear on the ISOBUS side but are not used in UDP frames.

### 2.3 Ports and binding

The TC opens two UDP sockets:

| Socket | Bind | Purpose |
|---|---|---|
| Main | `<LAN-IP>:8888` | All operational traffic to/from AgIO/AgValonia |
| Address detection | `0.0.0.0:8888` | Listens for the subnet-detection PGN so AgIO can tell the TC what subnet to use |

Both sockets have `SO_REUSEADDR` set (needed on Linux so the same port can host the specific-IP bind and the wildcard bind simultaneously).

The TC sends to **`<subnet>.255:9999`** as a broadcast. The "subnet" is configured via `settings.json` and can be overridden at runtime by the subnet-detection PGN.

### 2.4 NIC selection

On startup the TC enumerates network interfaces and picks the first IPv4 address whose first three octets match `settings.subnet`:

- **Linux/macOS**: via `getifaddrs(3)`.
- **Windows**: via the hostname-based Boost.Asio resolver.

If no NIC matches, the TC falls back to loopback (`127.0.0.1`) — useful for local testing but means broadcast to LAN won't work.

### 2.5 PGNs inbound (client → TC)

All PGNs sent **by AgIO/AgValonia to the TC** use source `0x7F`, except `0xD6` (GPS/IMU data), which comes from AOG's GPS submodule at source `0x7C`.

| PGN | Name | Length | Payload |
|---|---|---|---|
| `0xC9` (201) | Subnet detection | 5 | `[0xC9, 0xC9, IP0, IP1, IP2]` |
| `0xD6` (214) | GPS/IMU data | variable (≥39 used) | Only byte 38 (fix quality) is parsed today; rest of frame is currently unused. Source `0x7C`. |
| `0xE5` (229) | Section states (64 sections) | 8 | Bitfield: bit `8·j + i` of byte `j` is section `(8j + i)` ON/OFF |
| `0xEF` (239) | Machine data | variable | Only used as an AOG-liveness signal today; payload not parsed (see comment in `app.cpp` for the historical byte layout — tram markers there were superseded by PGN `0xF4`). |
| `0xF1` (241) | Section control mode | 1 | `[mode]` where `1` = enabled, `0` = disabled |
| `0xF2` (242) | Process data | 6 | `[DDI_lo, DDI_hi, val0, val1, val2, val3]` — DDI is little-endian `uint16`; value is little-endian `int32` |
| `0xF4` (244) | Guidance track context | 10 | Real-time AB-line/track guidance state, feeding ISOBUS TRACK (Tramline Control) — see §5.7. |

#### `0xC9` — Subnet detection

Tells the TC which `/24` subnet AgIO/AgValonia lives on. The first two payload bytes are `0xC9 0xC9` (a magic to disambiguate from other PGNs that share the source). The next three bytes are the first three octets of AgIO's IP.

On receipt the TC sets `settings.subnet = [IP0, IP1, IP2]`, closes the main socket, re-runs NIC enumeration, and rebinds. Useful for plug-and-play scenarios where the host may move between subnets.

#### `0xD6` — GPS/IMU data

Sent from AOG's GPS submodule, source `0x7C` (not `0x7F`). The TC only reads byte 38: AOG's fix-quality code (`0`=invalid, `1`=GPS, `2`=DGPS, `3`=PPS, `4`=RTK Fixed, `5`=RTK Float, `6+`=Estimated/Manual/Simulated). This value feeds DDI 514 (GNSSQuality) — see §5.7. Two fallback cases both resolve to `1` (weakest real GNSS fix), not `0` (No GNSS): AOG reporting `6+`, and no fresh `0xD6` (AOG doesn't always send this PGN at all — e.g. Simulator mode — and if none has arrived within 2 s the last value is treated as stale, independent of whether AOG overall is still connected). `0` is deliberately avoided as a fallback because some implements gate TRACK/section control on GNSS quality being non-zero.

#### `0xE5` — Section states

Reports the *actual* state of up to 64 sections. 8 bytes = 64 bits, one bit per section. The TC forwards these to the connected ISOBUS implement via the appropriate condensed work-state DDIs (DDI 160/161/290).

#### `0xEF` — Machine data

Historically carried uturn speed, hydraulic lift, geo-stop, and tram-marker bits, consumed by a synthetic guidance-track fallback. That fallback has been removed — guidance now comes exclusively from PGN `0xF4` — so today `0xEF` is only used to update the AOG-liveness timestamp; its payload is not parsed.

#### `0xF1` — Section control mode

`1` = automatic (TC drives the implement). `0` = manual (operator drives). The TC logs and propagates this to the implement.

#### `0xF2` — Process data

Wraps a single ISO 11783 DDI/value pair. The TC currently dispatches on these DDIs:

| DDI (decimal) | Name | TC behavior |
|---|---|---|
| `156` | Actual speed (mm/s) | Stored. If TECU enabled, broadcast as Ground/Wheel/Machine-selected speed (PGN 65256) + NMEA2000 SOG. Drives forward/reverse direction. Also produces J1939 PGN 65256 every 100 ms. |
| `597` | Total distance (mm) | Stored and displayed on the VT Status page. If TECU is enabled, also populated into Speed Messages distance fields. |
| Guidance line deviation | XTE (mm) | Converted to metres. Broadcast as NMEA2000 XTE (PGN 0x1F903) at 1 Hz. Also feeds DDI 0x0201 (GuidanceLineDeviation) — see §5.7. |

Unknown DDIs are silently ignored (PGN 0xF2 is the generic process-data channel — the TC will gain more DDIs over time).

#### `0xF4` — Guidance track context

AOG's real-time AB-line/track guidance state, driving ISOBUS TRACK (Tramline Control). 10-byte payload:

```
 Byte 0    Sequence counter (0–255, wraps)
 Byte 1    Flags: bit0=valid, bit1=heading same way, bit2=curve mode
 Bytes 2-3 Guidance Reference Line ID (uint16 LE) — 0 = no active track
 Bytes 4-5 Actual Track Number (int16 LE, signed — can be negative and jump by more than 1)
 Bytes 6-7 Track Number Left (int16 LE, signed)
 Bytes 8-9 Track Number Right (int16 LE, signed)
```

AOG sends this **only when the guidance state actually changes** — there is no heartbeat. The TC rejects any packet whose sequence number isn't strictly ahead of the last accepted one (catches duplicates, freezes, and reordered/stale UDP delivery).

**Track-number offset:** the TC adds `+1` to all three track numbers (current/left/right) before they reach `GuidanceTrackContext` — confirmed by field testing, not documented anywhere on AOG's side. As sent raw by AOG, the tramline implement's own on-board tram-pattern phase (which pass of N is "on") was consistently one pass out of sync with AOG's own intended tram on/off state, for both left and right passes; a uniform `+1` (independent of sign) brought them into agreement. See `GuidanceTrackProvider::parse()` (`AOG_TRACK_NUMBER_OFFSET`) in `guidance_track_context.hpp`.

See §5.7 for how this maps onto the outbound ISOBUS DDIs.

### 2.6 PGNs outbound (TC → client)

All PGNs sent **by the TC to AgIO/AgValonia** use source `0x80`.

| PGN | Name | Length | Frequency | Payload |
|---|---|---|---|---|
| `0xDD` (221) | Hardware message | 2 + T | On event | `[duration, colour, text0, text1, ...]` — `T` = UTF-8 text bytes |
| `0xF0` (240) | Section heartbeat / state | 2 + ⌈N/8⌉ | 100 ms | `[mode, num_sections, byte0, byte1, ...]` |

#### `0xDD` — Hardware message

Displays a text banner across the top of the AgOpenGPS map so the operator can see TC state that would otherwise only reach the console log. Payload:

- byte 0: `duration` — see the caveat below
- byte 1: `colour` — `0` renders a Salmon background (alert), any other value renders Bisque (info). The TC sends `1` for info.
- bytes 2..: the message text, UTF-8, **not** NUL-terminated

The frame's `Length` field is therefore `text_bytes + 2`, and AOG reads exactly `Length - 2` bytes starting at wire offset 7. The TC caps text at 60 bytes and truncates on a UTF-8 character boundary — splitting a multi-byte sequence would render as U+FFFD, and the display label clips long text anyway.

**`duration` is frames, not seconds.** AOG stores `duration × 10` into a counter that it decrements once per OpenGL render tick, so the banner lasts `duration × 10` redraws — roughly `duration` seconds at AOG's nominal 10 Hz, but it stretches or shrinks with frame rate. **A condition that persists must be re-sent**; do not send once with a large duration and expect the banner to hold. The operator can also dismiss a banner early by clicking it.

**AgOpenGPS ignores the source byte for this PGN** — it dispatches on the PGN byte alone. The TC still sends `0x80` per §2.2.

**Display is opt-in on the AOG side.** AgOpenGPS drops these frames unless *Config → Data → Hardware Messages* is enabled; the setting defaults to **off**. A correct implementation looks like a no-op until it is switched on, which is the first thing to check when testing. There is no corresponding TC-side setting — the TC always sends.

Messages the TC currently emits:

| Text | Colour | Duration | Trigger |
|---|---|---|---|
| `Implement: <name> (<n> sections, <w> m)` | Info | 5 | An implement with sections registers. The `, <w> m` clause is omitted when the DDOP has no usable geometry. |
| `Implement lost: <name>` | Alert | 10 | A previously seen implement disappears from the client list. |
| `TC address conflict: preferred address in use` | Alert | 20 | Another control function claiming `NAME::Function::TaskController` holds address `0xF7`. Re-sent every 15 s while true. |
| `TC address conflict resolved` | Info | 5 | The above clears. |
| `TECU failed to claim address 240` | Alert | 10 | Once at startup, when the TECU is enabled but could not claim its fixed address. |

The conflict messages surface on the operator's screen what the TC already writes to its console log; the console warning remains and keeps its own 30 s throttle.

#### `0xF0` — Section heartbeat / state

Sent every 100 ms for each connected ISOBUS implement that has sections. The payload is:

- byte 0: `1` if section control is enabled (auto), `0` if disabled (manual)
- byte 1: `num_sections` (the implement's section count)
- bytes 2..: bitfield of *actual* section ON/OFF states (1 bit per section, LSB-first within each byte)

If no implement is currently connected (or none have sections), and `aogHeartbeatEnabled` is `true` in `settings.json`, the TC still sends `0xF0` with `num_sections = 0` and no bitfield — a pure "I'm alive" beacon so AgIO can light up its ISOBUS indicator.

> **Compatibility note:** AgOpenGPS releases **before v6.8.2 beta 5** do not handle the `num_sections = 0` heartbeat correctly. Set `aogHeartbeatEnabled: false` in `settings.json` if pairing with an older AOG. AgValonia and AOG ≥ v6.8.2 beta 5 are fine.

---

## 3. Settings reference

The TC reads `settings.json` from a per-user config directory:

| OS | Path |
|---|---|
| Windows | `%APPDATA%\AOG-TaskController\settings.json` |
| Linux | `$XDG_CONFIG_HOME/AOG-TaskController/settings.json` (or `~/.config/AOG-TaskController/settings.json` if `XDG_CONFIG_HOME` is unset) |
| macOS | `~/Library/Application Support/AOG-TaskController/settings.json` |

```json
{
    "subnet": [192, 168, 5],
    "tecuEnabled": true,
    "nmeaSendEnabled": true,
    "aogHeartbeatEnabled": true,
    "vtEnabled": true,
    "tcVersion": 3,
    "languageCode": "en",
    "countryCode": "US"
}
```

| Key | Type | Default | Description |
|---|---|---|---|
| `subnet` | `int[3]` | `[192, 168, 5]` | First three octets of the LAN AgIO/AgValonia lives on. Used for NIC selection and broadcast destination. |
| `tecuEnabled` | `bool` | `true` | If `true`, the TC also impersonates a Tractor ECU on the CAN bus (claims address 240, broadcasts Speed Messages/NMEA2000, announces Class 1 BasicTractorECUServer). Set `false` when the tractor already has a TECU. |
| `nmeaSendEnabled` | `bool` | `true` | Enable cyclic NMEA2000 COG/SOG transmission. Requires `tecuEnabled`; the VT disables this control when no TECU interface exists. |
| `aogHeartbeatEnabled` | `bool` | `true` | Send `0xF0` heartbeat to AgIO/AgValonia every 100 ms even with no implement. Disable for AOG < v6.8.2 beta 5. |
| `vtEnabled` | `bool` | `true` | Register the Virtual Terminal client and display the TC UI when a VT is present. |
| `tcVersion` | `integer` | `3` | Task Controller version code (`0`=DIS, `1`=FDIS.1, `2`=First Edition, `3`=Second Edition Draft, `4`=Second Published Edition). |
| `languageCode` | `string` | `"en"` | Two-character language code advertised through the ISOBUS language interface. |
| `countryCode` | `string` | `"US"` | Two-character country code advertised through the ISOBUS language interface. |

Unknown keys are ignored. Settings updates are written to a temporary file and atomically renamed over `settings.json`; the previous file is preserved if writing fails.

---

## 4. Command-line reference

```
AOG-TaskController [options]
```

| Flag | Default | Description |
|---|---|---|
| `--help` | — | Print usage and exit. |
| `--version` | — | Print git-describe version (with `-dirty` suffix on a dirty tree) and exit. |
| `--can_adapter=<name>` | none | CAN driver. One of: `peak-pcan`, `innomaker-usb2can`, `rusoku-toucan`, `sys-tec-usb2can` (all Windows), `socketcan` (Linux). **Required** — the TC will not start without a driver. |
| `--can_channel=<id>` | — | Driver-specific channel. Numeric (`1`, `2`, ...) for Windows USB adapters; interface name (`can0`, `vcan0`) for SocketCAN. |
| `--log_level=<lvl>` | — | One of `debug`, `info`, `warning`, `error`, `critical`. Filters AgIsoStack log output. |
| `--log2file` | off | Also write all output to `<config>/logs/AOG-TaskController_YYYY-M-D_H-M.log`. |

---

## 5. ISOBUS / CAN side overview

This section is for context. The TC implements the bus side according to ISO 11783 — that standard, plus the AgIsoStack documentation, is the authoritative reference.

### 5.1 What the TC presents on the bus

Two control functions, both claiming addresses via standard J1939-81 address claim (with the 250 ms post-claim quiet period enforced):

| CF | NAME function | Address | Notes |
|---|---|---|---|
| Task Controller | `TaskController` (function code 61) | Preferred `247` (ISO 11783-10 MappingComputer). Walks if claimed. | Always present. |
| Tractor ECU | `TractorECU` (function code 132) | Fixed `240` (non-arbitrary-address-capable per ISO 11783-9). | Only present if `tecuEnabled: true`. |

Common NAME fields: Industry Group `2` (Agricultural), Device Class `0`, Manufacturer Code `1407`, Identity `20`. Override these in `app.cpp` if you fork.

### 5.2 What the TC sends on the bus

| PGN | Cadence | Producer | Purpose |
|---|---|---|---|
| `0xCB00` (Process Data) | 2 s | TC | ISO 11783-10 B.8.1 Task Controller Status. Status byte bit 1 = task totals active. |
| `0x1F903` (NMEA2000 XTE) | 1 Hz | TC | Cross-track error, derived from AOG's guidance-line deviation PGN. |
| `0xFEE8` (PGN 65256 Speed/Direction) | 100 ms | TECU | Ground/Wheel/Machine-selected speed + machine direction, J1939 format. Only when TECU enabled. |
| `0xFC8E` (Control Function Functionalities) | At claim + periodic | TC | Announces TaskControllerBasicServer (v1), TaskControllerSectionControlServer (v1, 1 boom / 64 sections), and functionality 27 — Tramline/TRACK Server (v1, Level 1). See §5.7. |
| `0xFC8E` (Control Function Functionalities) | At claim + periodic | TECU | Announces Class 1 BasicTractorECUServer (no options). |
| `0xFE09` (PGN 65033 Tractor Facilities) | Power-up + on request | TECU | 8-byte facility bitmask advertising which PGNs the TECU actually broadcasts. See §5.6. |
| `0xFEE6` (PGN 65254 Time/Date) | 10 s, suppressed if another provider is detected | TECU | Wall-clock UTC + local offset, from `TimeDateInterface`. Also answers PGN-request for `0xFEE6`. |
| Tramline/TRACK process data (DDI 507-511, 514) | 250 ms, once a client completes negotiation | TC | See §5.7. |
| NMEA2000 COG/SOG | Periodic | TECU | Optional course/speed over ground. |

The TC also receives all ISOBUS Process Data (PGN 0xCB00) and Section Control commands from connected implements.

### 5.3 What the TC handles inbound from implements

- **Device Descriptor Object Pool (DDOP)** uploads from clients (stored per client).
- **Condensed actual work-state DDIs** (160, 161, 290, plus the extended range 16001–16016 per the standard): mapped into the per-client section model and forwarded to AgIO/AgValonia as PGN `0xF0`.
- **Section control state DDI**: tracked per client.
- **Process data acknowledges (PDACK)**: logged.
- **PGN 65033 requests**: answered with the Tractor Facilities response (§5.6). An implement may also send PGN 65032 (Required Tractor Facilities) to advertise what it needs; the TC logs this at debug level but does not change its response.
- **Tramline/TRACK DDIs (505, 506, 507, 508-511, 515)**: negotiated and tracked per client — see §5.7.

### 5.4 ISOBUS feature scope

| Capability | Value |
|---|---|
| ISO 11783-10 version | 2 (Second Edition) |
| Generation | 1 (TC-SC), plus TRACK (Tramline Control) Level 1 |
| Max booms | 1 |
| Max sections | 64 |
| Supported DDIs | 160 / 161 / 290 (condensed section setpoint and actual states); 505 / 506 / 507 / 508 / 509 / 510 / 511 / 514 / 515 (Tramline/TRACK, Level 1 — see §5.7); plus speed/distance/guidance DDIs from the tractor side |

### 5.5 Virtual Terminal UI

The roughly 12 KB VT object pool is embedded in the executable, so deployment does not require a separate `AOG_TC.iop` file. The committed `src/AOG_TC.iop` remains its build-time source of truth, while `src/AOG_TC.iop.h` contains the ISO-Designer-generated object IDs and authored geometry.

The pool was authored for a 480-pixel data mask and an 80-pixel softkey designator. The client asks AgIsoStack to scale both before initialization, and includes that scaling contract in the VT cache version so a terminal cannot reuse a pool cached by an older unscaled build. The UI defines five virtual navigation softkeys; terminals with fewer than five physical softkeys must support paging.

At connection, the TC logs the VT version, screen dimensions, softkey dimensions, and virtual/physical softkey counts. It warns when fewer than five virtual softkeys are available. If a VT address is detected but the client has not connected after 30 seconds, it logs the reported capabilities and recovery guidance. Clear the terminal's stored/cached object pools first when diagnosing an upload failure, because stale pools and full non-volatile pool storage can prevent an otherwise compatible upload.

### 5.6 Tractor Facilities (PGN 65033)

When the TECU is enabled, the TC responds to PGN 65033 requests (ISO 11783-7 B.24.3) and broadcasts the response once on power-up. The 8-byte payload is a bitfield where each bit signals that the TECU actually transmits the corresponding PGN at its defined repetition rate.

**Facilities advertised (bits set to 1):**

| Byte | Bit(s) | Facility | Condition |
|---|---|---|---|
| 1 | 8,7 | TECU class | Always `00` (Class 1). |
| 1 | 2 | Ground-based speed (PGN 65097) | `speedMessagesInterface` exists (always true when TECU is enabled). |
| 1 | 3 | Wheel-based speed (PGN 65096) | `speedMessagesInterface` exists (always true when TECU is enabled). |
| 3 | 8 | Time/date (PGN 65254) | `timeDateActive` — set whenever the TC's `TimeDateInterface` is actively broadcasting FEE6 (i.e. no duplicate Time/Date provider has been detected on the bus). Cleared if another ECU's FEE6 is seen. |
| 3 | 7,6 | Ground-based distance + direction | Same as ground-based speed. |
| 3 | 5,4 | Wheel-based distance + direction | Same as wheel-based speed. |

**Facilities NOT advertised (bits always 0):**

- Engine speed — no engine CAN access.
- Power management — no key switch or power timer signals.
- Hitch position / in-work / draft — the hydraulic lift output is a command we issue, not measured feedback; implements would trust it for work-state logic.
- PTO shaft speed / engagement — no PTO sensor.
- Lighting — no lighting controller.
- Language command storage (PGN 65039) — not broadcast by the TECU.
- Auxiliary valve commands / status — no valve interface.
- Selected speed (PGN 65265) — not broadcast.
- Navigation position data / high-output position — NMEA 2000 position PGNs are not forwarded over Fast Packet.
- Front hitch / PTO — no front hitch or PTO sensors.
- All reserved bits (byte 2 bits 2–1, byte 4 bits 3–1, byte 5 bit 5, byte 7, byte 8 including the reserved-bit indicator at bit 1).

**Default payload** (TECU enabled, speed broadcasts active, TC is the sole Time/Date provider on the bus): `[0x06, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00]`. Byte 3 drops to `0x78` (Time/date bit cleared) if another ECU's FEE6 is detected and the TC suppresses its own broadcast.

**PGN 65032 (Required Tractor Facilities):** When an implement broadcasts what it needs, the TC logs the request at debug level. The response is not modified based on the implement's requirements — a facility bit is set to 1 only when backed by a live broadcast.

### 5.7 Tramline / TRACK Control

The TC implements AEF/ISO 11783 Task Controller TRACK (Tramline Control), **Level 1 only**: it reports track/guidance-line info to the implement. It does not compute tramline valve states itself (Level 3) or handle the extended Level 2 setup DDIs beyond negotiation.

**Capability announcement**: at TC control-function claim, PGN 64654 (Control Function Functionalities, source = the TC's own address — see §5.2) advertises `TaskControllerBasicServer` v1, `TaskControllerSectionControlServer` v1 (1 boom / 64 sections), and functionality 27 (TC-TRAM) v1, telling the implement this TC supports tramline.

**Negotiation (DDI 505/506 handshake)**: when a client's DDOP is registered, the TC scans it for DDI 505 (`TramlineControlLevel`), 506 (`SetpointTramlineControlLevel`), 515 (`TramlineControlState`), and the track DDIs (508-511).

1. The implement reports DDI 505 as a **bitmask** (bit 0 = Level 1, bit 1 = Level 2, bit 2 = Level 3).
2. If the implement also has DDI 506, the TC immediately writes DDI 506 back as an **enum** (`0`=no common level, `1`=Level 1, `2`=Level 2, `3`=Level 3) — currently always `1`, since only Level 1 is implemented, even if the implement also advertises Level 2/3.
3. The implement's DDI 506 echo marks negotiation complete for that client. Only then does the TC start sending live track data to it.

**Live data** (every 250 ms per negotiated client, `MyTCServer::send_tramline_track_data()`):

| DDI | Name | Sent when |
|---|---|---|
| 507 | TramlineSequenceNumber | Track-valid only. Increments whenever `ActualTrackNumber` or the reference line ID changes — not on section-control toggles. |
| 508 | UniqueABGuidanceReferenceLineID | Track-valid only. |
| 509 | ActualTrackNumber | Track-valid only. Signed; can be negative and can jump by more than 1 in a single update (e.g. skipping several tracks on a headland turn). |
| 510 / 511 | TrackNumberToTheRight / TrackNumberToTheLeft | Track-valid only. |
| 0x0200 / 0x0201 | GuidanceLineSwathWidth / GuidanceLineDeviation | Track-valid only. Swath is currently hardcoded to 6000 mm (TODO: derive from DDOP geometry); deviation comes from AOG's XTE (PGN 0xF2, see §2.5). |
| 514 | GNSSQuality | Always, independent of track validity — it's a positioning signal, not a track signal. Sourced from AOG PGN `0xD6` (see §2.5). |
| 515 | TramlineControlState | **Not** sent from here. Owned solely by section-control-mode handling (PGN `0xF1`, see §2.5) — do not write it elsewhere. |

"Track-valid" means the TC has an accepted PGN `0xF4` payload with the valid flag set and a non-zero reference line ID (0 = AOG's "no active track" convention). Since AOG only sends `0xF4` on change — there's no heartbeat — track validity is *not* cleared just because no new `0xF4` has arrived; it's only cleared by an explicit "guidance off" packet from AOG, or by AOG disconnecting entirely (no packets of any kind for 3 s).

---

## 6. Example clients

These examples talk to a TC listening on subnet `192.168.5.x`. Adjust the broadcast address to your LAN.

### 6.1 Python — minimal heartbeat sender + section reader

```python
"""
Sends a subnet-detection packet, then prints every PGN 0xF0 heartbeat
that the TC broadcasts. Useful for confirming the TC is alive on your LAN.
"""
import socket
import struct
import threading

TC_HOST_BCAST = ("192.168.5.255", 8888)
LISTEN_PORT = 9999
SRC_AGIO = 0x7F
PGN_SUBNET_DETECT = 0xC9
PGN_SECTION_HEARTBEAT = 0xF0

def frame(src: int, pgn: int, payload: bytes) -> bytes:
    header = bytes([0x80, 0x81, src, pgn, len(payload)])
    crc = sum(header[2:] + payload) & 0xFF
    return header + payload + bytes([crc])

def announce_subnet():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    # IP0 IP1 IP2 = first three octets of OUR (AgIO/AgValonia) IP
    payload = bytes([0xC9, 0xC9, 192, 168, 5])
    s.sendto(frame(SRC_AGIO, PGN_SUBNET_DETECT, payload), TC_HOST_BCAST)
    s.close()

def listen():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", LISTEN_PORT))
    while True:
        data, addr = s.recvfrom(512)
        if len(data) < 6 or data[0] != 0x80 or data[1] != 0x81:
            continue
        src, pgn, length = data[2], data[3], data[4]
        payload = data[5:5 + length]
        if pgn == PGN_SECTION_HEARTBEAT:
            mode = "AUTO" if payload[0] == 1 else "MANUAL"
            n = payload[1]
            states = []
            for i in range(n):
                byte = payload[2 + (i // 8)]
                states.append("1" if byte & (1 << (i % 8)) else "0")
            print(f"{addr[0]}  heartbeat  mode={mode}  sections={n}  bits={''.join(states) or '(none)'}")

threading.Thread(target=listen, daemon=True).start()
announce_subnet()
import time; time.sleep(60)
```

### 6.2 C# — frame builder for AgValonia / .NET

```csharp
// Reusable AgIO/AgValonia frame builder. Targets .NET 8+.
// Drop into a System.Net.Sockets.UdpClient and you're done.

using System;

public static class TaskControllerFrame
{
    public const byte SrcAgio = 0x7F;
    public const byte SrcTc   = 0x80;

    public const byte PgnSubnetDetect      = 0xC9;
    public const byte PgnSectionStates     = 0xE5;
    public const byte PgnSectionControl    = 0xF1;
    public const byte PgnProcessData       = 0xF2;
    public const byte PgnSectionHeartbeat  = 0xF0;

    /// <summary>Build a complete framed packet ready for UdpClient.Send.</summary>
    public static byte[] Build(byte src, byte pgn, ReadOnlySpan<byte> payload)
    {
        if (payload.Length > 250)
            throw new ArgumentException("payload too large");

        var buf = new byte[6 + payload.Length];
        buf[0] = 0x80;
        buf[1] = 0x81;
        buf[2] = src;
        buf[3] = pgn;
        buf[4] = (byte)payload.Length;
        payload.CopyTo(buf.AsSpan(5));

        int sum = 0;
        for (int i = 2; i < 5 + payload.Length; i++) sum += buf[i];
        buf[5 + payload.Length] = (byte)(sum & 0xFF);
        return buf;
    }

    /// <summary>Tell the TC which subnet AgValonia lives on.</summary>
    public static byte[] SubnetDetect(byte ip0, byte ip1, byte ip2) =>
        Build(SrcAgio, PgnSubnetDetect, new byte[] { 0xC9, 0xC9, ip0, ip1, ip2 });

    /// <summary>Enable (true) or disable (false) automatic section control.</summary>
    public static byte[] SectionControlMode(bool enabled) =>
        Build(SrcAgio, PgnSectionControl, new byte[] { (byte)(enabled ? 1 : 0) });

    /// <summary>Set the actual ON/OFF state of up to 64 sections.</summary>
    public static byte[] SectionStates(ulong bitmap)
    {
        var b = new byte[8];
        for (int i = 0; i < 8; i++) b[i] = (byte)(bitmap >> (i * 8));
        return Build(SrcAgio, PgnSectionStates, b);
    }

    /// <summary>Send a single process-data (DDI, value) pair to the TC.</summary>
    public static byte[] ProcessData(ushort ddi, int value)
    {
        var p = new byte[6];
        p[0] = (byte)(ddi & 0xFF);
        p[1] = (byte)(ddi >> 8);
        p[2] = (byte)(value);
        p[3] = (byte)(value >> 8);
        p[4] = (byte)(value >> 16);
        p[5] = (byte)(value >> 24);
        return Build(SrcAgio, PgnProcessData, p);
    }
}
```

### 6.3 Quick command-line sanity check

```bash
# Listen for TC heartbeats with a one-liner (Linux/macOS):
nc -ul 9999 | xxd | head

# Send a subnet-detect packet with socat:
printf '\x80\x81\x7f\xc9\x05\xc9\xc9\xc0\xa8\x05\x4e' | socat - UDP-DATAGRAM:192.168.5.255:8888,broadcast
```

(The CRC byte `0x4e` is `0x7F + 0xC9 + 0x05 + 0xC9 + 0xC9 + 0xC0 + 0xA8 + 0x05` mod 256.)

---

## 7. Future direction

- The CRC verification path is in `udp_connections.cpp` but commented out. Clients are encouraged to send a correct checksum even though it is not enforced today.
- New DDIs are routinely added to PGN `0xF2`. The set in §2.5 is a snapshot — check `src/app.cpp` for the current dispatcher.
- macOS builds are not in CI yet, but the source compiles cleanly on macOS (with `CAN_DRIVER=MacCANPCAN`) and the config paths follow Apple conventions.
