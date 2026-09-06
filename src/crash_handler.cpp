#include "crash_handler.hpp"

#include "logging_utils.hpp"
#include "settings.hpp"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace
{
	std::string crash_file_path(const std::string &extension)
	{
		std::time_t now = std::time(nullptr);
		std::tm localTime;
#if defined(_WIN32)
		localtime_s(&localTime, &now);
#else
		localtime_r(&now, &localTime);
#endif
		char stamp[32];
		std::snprintf(stamp, sizeof(stamp), "%04d-%02d-%02d_%02d-%02d-%02d", localTime.tm_year + 1900, localTime.tm_mon + 1, localTime.tm_mday, localTime.tm_hour, localTime.tm_min, localTime.tm_sec);
		// Settings::get_filename_path() throws if it can't create the directory; a crash
		// handler must never throw, so fall back to the current directory on failure.
		try
		{
			return Settings::get_filename_path(std::string("logs/crash_") + stamp + extension);
		}
		catch (...)
		{
			return std::string("crash_") + stamp + extension;
		}
	}

	void log_terminate_reason()
	{
		std::string reason = "std::terminate called";
		if (auto currentException = std::current_exception())
		{
			try
			{
				std::rethrow_exception(currentException);
			}
			catch (const std::exception &e)
			{
				reason += std::string(" due to unhandled exception: ") + e.what();
			}
			catch (...)
			{
				reason += " due to an unhandled exception of unknown type";
			}
		}
		else
		{
			reason += " with no active exception (direct std::terminate()/std::abort(), a noexcept "
			          "violation, or a failure during stack unwinding)";
		}
		log_crash(reason);
	}
} // namespace

void log_crash(const std::string &reason)
{
	std::ofstream out(crash_file_path(".log"), std::ios::app);
	std::ostream &sink = out.is_open() ? static_cast<std::ostream &>(out) : std::cout;
	sink << "[" << get_timestamp() << "] [Crash] " << reason << std::endl;
}

#if defined(_WIN32)

#include <windows.h>

#include <dbghelp.h>

#pragma comment(lib, "dbghelp.lib")

namespace
{
	LONG WINAPI unhandled_exception_filter(EXCEPTION_POINTERS *exceptionPointers)
	{
		const std::string dumpPath = crash_file_path(".dmp");

		{
			std::ostringstream reason;
			reason << "Unhandled SEH exception 0x" << std::hex << exceptionPointers->ExceptionRecord->ExceptionCode
			       << std::dec << " at address " << exceptionPointers->ExceptionRecord->ExceptionAddress
			       << ". Writing minidump to " << dumpPath;
			log_crash(reason.str());
		}

		HANDLE dumpFile = CreateFileA(dumpPath.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
		if (dumpFile != INVALID_HANDLE_VALUE)
		{
			MINIDUMP_EXCEPTION_INFORMATION mdInfo{};
			mdInfo.ThreadId = GetCurrentThreadId();
			mdInfo.ExceptionPointers = exceptionPointers;
			mdInfo.ClientPointers = FALSE;

			MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), dumpFile, MiniDumpWithIndirectlyReferencedMemory, &mdInfo, nullptr, nullptr);
			CloseHandle(dumpFile);
		}

		// Let Windows terminate the process normally after we've captured diagnostics.
		return EXCEPTION_EXECUTE_HANDLER;
	}
} // namespace

void install_crash_handlers()
{
	SetUnhandledExceptionFilter(unhandled_exception_filter);
	std::set_terminate(log_terminate_reason);
}

#else // POSIX

#include <unistd.h>
#include <csignal>

namespace
{
	void fatal_signal_handler(int signalNumber)
	{
		const char *name = "unknown signal";
		switch (signalNumber)
		{
			case SIGSEGV:
				name = "SIGSEGV (segmentation fault)";
				break;
			case SIGABRT:
				name = "SIGABRT (abort)";
				break;
			case SIGFPE:
				name = "SIGFPE (arithmetic error)";
				break;
			case SIGILL:
				name = "SIGILL (illegal instruction)";
				break;
			case SIGBUS:
				name = "SIGBUS (bus error)";
				break;
			default:
				break;
		}

		// Signal-handler-safe I/O only: no iostreams, no dynamic allocation, no
		// get_timestamp(), and no strlen() — it's not on POSIX's async-signal-safe
		// function list, so a hand-rolled length count is used instead.
		const int fd = 2; // stderr
		auto writeRaw = [fd](const char *text) {
			std::size_t len = 0;
			while (text[len] != '\0')
			{
				++len;
			}
			(void)write(fd, text, len);
		};
		writeRaw("[Crash] Fatal signal: ");
		writeRaw(name);
		writeRaw("\n");

		// Re-trigger the signal with default disposition so the process terminates normally
		// (and may produce a core dump), without calling non-async-signal-safe functions.
		::kill(::getpid(), signalNumber);
		::_exit(128 + signalNumber);
	}
} // namespace

void install_crash_handlers()
{
	struct sigaction sa
	{
	};
	sa.sa_handler = fatal_signal_handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESETHAND;
	const int fatalSignals[] = { SIGSEGV, SIGABRT, SIGFPE, SIGILL, SIGBUS };
	for (int sig : fatalSignals)
	{
		(void)sigaction(sig, &sa, nullptr);
	}
	std::set_terminate(log_terminate_reason);
}

#endif
