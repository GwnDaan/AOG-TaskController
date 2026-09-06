/**
 * @file crash_handler.hpp
 * @brief Last-resort crash diagnostics: writes a crash log (and, on Windows, a
 *        minidump) to the config "logs" directory before the process dies.
 *
 * This exists because AOG-TaskController normally runs with no visible console
 * (launched by AgIO/AOG) and without --log2file, so a hard crash (access
 * violation, uncaught exception, std::terminate) otherwise leaves zero trace.
 * install_crash_handlers() should be called once, as early as possible in
 * main/WinMain.
 */
#pragma once

#include <string>

/// @brief Install process-wide crash handlers (SEH filter + minidump on
/// Windows, signal handlers on POSIX; std::terminate handler on both).
void install_crash_handlers();

/// @brief Append a line to the same crash log file the handlers above write to
/// (config dir's logs/crash_<timestamp>.log), regardless of whether --log2file
/// was passed. For fatal-but-caught conditions (e.g. an exception caught at the
/// top of main()) that should still leave a trace on disk.
void log_crash(const std::string &reason);
