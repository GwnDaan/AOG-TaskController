# AOG-TaskController — Concurrency Model

This document exists because the threading model here is easy to get wrong by analogy with the
main loop, and a wrong assumption here previously produced a crash with **zero trace** — no
exception, no log line, no console output. Read this before adding a new callback (a
`TaskControllerServer` override, an event-dispatcher listener, a raw
`add_global_parameter_group_number_callback`) or before touching state that such a callback reads
or writes.

## The two threads

`isobus::CANHardwareInterface::start()` (called once, in `Application::setup_can_hardware()`)
spawns its own background thread by default — this is standard AgIsoStack behavior, not something
this repo opted into. That thread calls `CANNetworkManager::CANNetwork.update()` internally, which
is what actually parses incoming CAN frames, handles address claims, and dispatches every PGN to
whatever is registered to receive it.

Meanwhile, `Application::update()` — the loop driven from `main.cpp`, doing UDP handling and
periodic sends — runs on the **main thread**. It never calls `CANHardwareInterface::update()`
itself; it relies entirely on that background thread.

So: **two threads exist for the lifetime of the process**, and the question for any new code is
which one it runs on.

## Which callbacks run on which thread — checked, not assumed

Don't guess this from a class's "manual update mode" comment (that pattern is unrelated to this
question and led to a wrong conclusion once already). Check how the *specific* callback is
registered:

- **Runs on the CAN stack background thread by default** (CANHardwareInterface's updateThread): `TaskControllerServer`
  (`MyTCServer`). Treat every `TaskControllerServer` override (`activate_object_pool`,
  `on_value_command`, ...) as background-thread code unless you have traced the library
  implementation and proven it is deferred to `TaskControllerServer::update()`.
- **Runs directly on the background thread, synchronously, as the frame is processed** — not
  deferred to any `update()` call: `VirtualTerminalClient` (confirmed: `process_rx_message` is
  registered via `add_global_parameter_group_number_callback` and invokes
  `softKeyEventDispatcher`/`buttonEventDispatcher`/`changeNumericValueEventDispatcher`/etc.
  synchronously). Any raw
  `add_global_parameter_group_number_callback`/`add_any_control_function_parameter_group_number_callback`
  registered directly in `app.cpp` is in this category too, unless proven otherwise the same way:
  read the registration call, not the surrounding comments.

If you're not sure which category a new callback falls into, trace it the way this file's history
did: find where it's registered with `CANNetworkManager`/`ControlFunction` and check whether that
registration point itself defers to an `update()` call, or invokes the listener/dispatcher inline.

## What's protected today

- **`MyTCServer::clients` / `uploadedPools`** (`task_controller.hpp`/`.cpp`) — guarded by
  `MyTCServer::clientsMutex` (a `std::recursive_mutex`, since several methods call each other:
  e.g. `update_section_states()` → `send_section_setpoint_states()`). Locked at the top of every
  `TaskControllerServer` override and every method `Application` calls into. `get_clients()`
  returns a **copy**, not a reference — call sites hold the result across multiple operations or
  take a second `get_clients()` call, either of which would leave a dangling reference to a
  temporary if it returned by reference. If you add a new method that reads or writes `clients`,
  take `clientsMutex` at its top, even though (per the note above) the `TaskControllerServer`
  overrides may turn out to already be main-thread-only — the lock is cheap insurance and keeps
  every entry point consistent regardless of how the library's internals might change.

## What's *not* protected yet — known gap

Calls into `vtClient` (`isobus::VirtualTerminalClient`) happen from both the main thread
(`Application::update_vt_client()` and everything it calls: `send_vt_string_if_changed`,
`vtUpdateHelper`, etc.) **and** from the VT event-dispatcher lambdas registered in
`setup_vt_client()` (soft-key, button, numeric-value-change), which — per the confirmed
registration pattern above — run on the background thread. Whether `VirtualTerminalClient`'s own
internal state is safe under that concurrent access is not verified; no external mutex wraps
`vtClient` usage today. If a crash dump ever points into `isobus::VirtualTerminalClient`, or a
crash correlates with VT/AUX-N activity rather than DDOP upload, this is the first place to look.

## Also worth knowing: `get_object_by_index()` can return null

Unrelated to threading, but found while investigating a real crash report ("usually happens right
after DDOP loading"): `isobus::DeviceDescriptorObjectPool::get_object_by_index()` can return
`nullptr` for an index within `[0, size())` — most likely a symptom of a partially- or
incorrectly-parsed pool (e.g. a multi-chunk DDOP transfer). Two call sites in this file already
null-checked it; the rest didn't and were fixed alongside the mutex work above. **Any new call to
`get_object_by_index()` must null-check the result before dereferencing.**
