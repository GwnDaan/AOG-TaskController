#include "app.hpp"
#include "crash_handler.hpp"
#include "logging.cpp"
#include "settings.hpp"

#include "isobus/hardware_integration/available_can_drivers.hpp"
#include "isobus/isobus/can_stack_logger.hpp"

#include "git.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#if defined(_WIN32)
#include <shellapi.h>
#include <windows.h>
#define TRAY_ICON_ID 1
#else
#include <csignal>
#endif

static std::atomic_bool running = { true };

#if defined(_WIN32)
// Window procedure to handle messages
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
		case WM_CLOSE:
			running = false;
			break;

		default:
			return DefWindowProc(hwnd, message, wParam, lParam);
	}
	return 0;
}

std::vector<std::string> ParseCommandLine(LPSTR /*lpCmdLine*/)
{
	std::vector<std::string> arguments;
	int argc;
	LPWSTR *argv = CommandLineToArgvW(GetCommandLineW(), &argc);
	if (argv == NULL)
	{
		return arguments;
	}

	for (int i = 0; i < argc; i++)
	{
		int requiredSize = WideCharToMultiByte(CP_UTF8, 0, argv[i], -1, NULL, 0, NULL, NULL);
		if (requiredSize == 0)
		{
			continue;
		}

		// Allocate a string with length excluding the null terminator.
		std::string arg(requiredSize - 1, '\0');
		int result = WideCharToMultiByte(CP_UTF8, 0, argv[i], -1, arg.data(), requiredSize, NULL, NULL);
		if (result == 0)
		{
			continue;
		}
		arguments.push_back(arg);
	}

	LocalFree(argv);
	return arguments;
}
#else
static void signal_handler(int /*signum*/)
{
	running = false;
}

static std::vector<std::string> ParseCommandLine(int argc, char **argv)
{
	std::vector<std::string> arguments;
	arguments.reserve(argc);
	for (int i = 0; i < argc; ++i)
	{
		arguments.emplace_back(argv[i]);
	}
	return arguments;
}
#endif

enum class CANAdapter
{
	NONE,
	ADAPTER_PCAN_USB,
	ADAPTER_INNOMAKER_USB2CAN,
	ADAPTER_RUSOKU_TOUCAN,
	ADAPTER_SYS_TEC_USB2CAN,
	ADAPTER_SOCKETCAN,
};

class ArgumentProcessor
{
public:
	ArgumentProcessor(std::vector<std::string> arguments) :
	  arguments(arguments)
	{
	}

	bool process()
	{
		for (std::string arg : arguments)
		{
			std::transform(arg.begin(), arg.end(), arg.begin(), [](unsigned char c) { return std::tolower(c); });
			parse_option(arg);
			parse_parameter(arg);
		}
		return true;
	}

	CANAdapter get_can_adapter() const
	{
		return canAdapter;
	}

	std::string get_can_channel() const
	{
		return canChannel;
	}

	bool is_file_logging() const
	{
		return fileLogging;
	}

private:
	bool parse_option(std::string option)
	{
		if ("--help" == option)
		{
			std::cout << "Usage: AOG-TaskController [options]\n";
			std::cout << "Options:\n";
			std::cout << "  --help\t\tShow this help message\n";
			std::cout << "  --version\t\tShow the version of the application\n";
			std::cout << "  --can_adapter=<driver>\tSelect the CAN driver (peak-pcan, innomaker-usb2can, rusoku-toucan, sys-tec-usb2can, socketcan)\n";
			std::cout << "  --can_channel=<channel>\tSelect the CAN channel (numeric, or interface name e.g. can0 for socketcan)\n";
			std::cout << "  --log_level=<level>\tSet the log level (debug, info, warning, error, critical)\n";
			std::cout << "  --log2file\t\tLog to file\n";
			std::exit(0);
		}
		else if ("--version" == option)
		{
			std::cout << std::string(git::Describe()) + (git::AnyUncommittedChanges() ? "-dirty" : "") << std::endl;
			std::exit(0);
		}
		else if ("--log2file" == option)
		{
			fileLogging = true;
		}
		else
		{
			return false;
		}
		return true;
	}

	bool parse_parameter(std::string parameter)
	{
		size_t pos = parameter.find('=');
		if (pos == std::string::npos)
		{
			return false;
		}
		std::string key = parameter.substr(0, pos);
		std::string value = parameter.substr(pos + 1);

		if ("--can_adapter" == key)
		{
			static const std::unordered_map<std::string, CANAdapter> adapterMap = {
				{ "peak-pcan", CANAdapter::ADAPTER_PCAN_USB },
				{ "innomaker-usb2can", CANAdapter::ADAPTER_INNOMAKER_USB2CAN },
				{ "rusoku-toucan", CANAdapter::ADAPTER_RUSOKU_TOUCAN },
				{ "sys-tec-usb2can", CANAdapter::ADAPTER_SYS_TEC_USB2CAN },
				{ "socketcan", CANAdapter::ADAPTER_SOCKETCAN },
			};

			auto it = adapterMap.find(value);
			if (it != adapterMap.end())
			{
				canAdapter = it->second;
				return true;
			}

			std::cout << "Unknown CAN adapter: " << value.c_str() << std::endl;
			return false;
		}
		else if ("--can_channel" == key)
		{
			canChannel = value;
		}
		else if ("--log_level" == key)
		{
			if ("debug" == value)
			{
				isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Debug);
			}
			else if ("info" == value)
			{
				isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Info);
			}
			else if ("warning" == value)
			{
				isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Warning);
			}
			else if ("error" == value)
			{
				isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Error);
			}
			else if ("critical" == value)
			{
				isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Critical);
			}
			else
			{
				std::cout << "Unknown log level: " << value.c_str() << std::endl;
				return false;
			}
		}
		else
		{
			return false;
		}
		return true;
	}

	std::vector<std::string> arguments;
	CANAdapter canAdapter = CANAdapter::NONE;
	std::string canChannel;
	bool fileLogging = false;
};

static std::shared_ptr<isobus::CANHardwarePlugin> create_can_driver(const ArgumentProcessor &args)
{
	switch (args.get_can_adapter())
	{
#if defined(_WIN32)
		case CANAdapter::ADAPTER_PCAN_USB:
		{
			return std::make_shared<isobus::PCANBasicWindowsPlugin>(PCAN_USBBUS1 - 1 + std::stoi(args.get_can_channel()));
		}
		case CANAdapter::ADAPTER_INNOMAKER_USB2CAN:
		{
			return std::make_shared<isobus::InnoMakerUSB2CANWindowsPlugin>(std::stoi(args.get_can_channel()) - 1);
		}
		case CANAdapter::ADAPTER_RUSOKU_TOUCAN:
		{
			return std::make_shared<isobus::TouCANPlugin>(std::stoi(args.get_can_channel()), std::stoi(args.get_can_channel()));
		}
		case CANAdapter::ADAPTER_SYS_TEC_USB2CAN:
		{
			return std::make_shared<isobus::SysTecWindowsPlugin>(static_cast<std::uint8_t>(std::stoi(args.get_can_channel())));
		}
#endif
#if defined(ISOBUS_SOCKETCAN_AVAILABLE)
		case CANAdapter::ADAPTER_SOCKETCAN:
		{
			std::string channel = args.get_can_channel();
			if (channel.empty())
			{
				channel = "can0";
			}
			return std::make_shared<isobus::SocketCANInterface>(channel);
		}
#endif
		default:
		{
			break;
		}
	}
	std::cout << "No CAN adapter selected (or selected adapter not compiled into this build)." << std::endl;
	return nullptr;
}

// Process command-line arguments, set up logging, and create the CAN driver.
// Returns nullptr on argument-error or driver-creation failure. Note that
// --help / --version cause the process to exit() inside argument parsing.
static std::shared_ptr<isobus::CANHardwarePlugin> prepare_application(const std::vector<std::string> &arguments)
{
	ArgumentProcessor argumentProcessor(arguments);
	bool argumentsProcessed = argumentProcessor.process();

	// The sequence is important here: process the arguments first, then check
	// if file logging is enabled, then log the arguments and version.
	isobus::CANStackLogger::set_can_stack_logger_sink(&logger);
	if (argumentProcessor.is_file_logging())
	{
		setup_file_logging();
	}

	for (const std::string &arg : arguments)
	{
		std::cout << arg.c_str() << " ";
	}
	std::cout << std::endl;
	std::cout << "AOG-TC version: v" << std::string(git::Describe()) + (git::AnyUncommittedChanges() ? "-dirty" : "") << std::endl;

	if (!argumentsProcessed)
	{
		std::cout << "Failed to process arguments, exiting..." << std::endl;
		return nullptr;
	}

	return create_can_driver(argumentProcessor);
}

static int run_application_loop(std::shared_ptr<isobus::CANHardwarePlugin> canDriver)
{
	Application app(canDriver);
	try
	{
		if (!app.initialize())
		{
			std::cout << "Failed to initialize application..." << std::endl;
			return -1;
		}

		std::cout << "[" << get_timestamp() << "] Press Ctrl+C to stop the application..." << std::endl;

		while (running)
		{
#if defined(_WIN32)
			// Pump the (hidden) message queue with a 1 ms timeout, mirroring the
			// previous behavior so AOG can still close us via WM_CLOSE.
			MSG msg;
			DWORD result = MsgWaitForMultipleObjects(0, NULL, FALSE, 1, QS_ALLINPUT);
			while (result == WAIT_OBJECT_0 && PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
#else
			// On POSIX we have no message loop; just sleep briefly between updates.
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif

			if (!app.update())
			{
				std::cout << "Something unexpected happened, stopping application..." << std::endl;
				break;
			}
		}
	}
	// A crash (access violation, SIGSEGV, ...) can't be caught here — that's what
	// install_crash_handlers() is for. This is the safety net for ordinary C++
	// exceptions (e.g. a library call throwing) that would otherwise propagate all
	// the way out and terminate the process with zero trace, since this app usually
	// runs with no visible console and without --log2file.
	catch (const std::exception &e)
	{
		log_crash(std::string("Unhandled exception escaped the main loop: ") + e.what());
	}
	catch (...)
	{
		log_crash("Unhandled exception of unknown type escaped the main loop.");
	}

	std::cout << "[" << get_timestamp() << "] Shutting down..." << std::endl;
	app.stop();
	return 0;
}

#if defined(_WIN32)
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	// Install first: this app usually runs with no visible console and without
	// --log2file, so a crash otherwise leaves zero trace (see crash_handler.hpp).
	install_crash_handlers();

	// Try to attach to the parent process's console if it exists
	if (AttachConsole(ATTACH_PARENT_PROCESS))
	{
		FILE *fp;
		freopen_s(&fp, "CONOUT$", "w", stdout); // Redirect stdout to the console
		std::cout << std::string(100, '\n'); // Clear the console screen
		std::cout << "WARNING: Only start the application if you know what you are doing. It is intended to be started from AgIO!" << std::endl;
		std::cout << "Press Ctrl+C to stop the application..." << std::endl;
		std::cout << std::endl; // White space
	}

	auto arguments = ParseCommandLine(lpCmdLine);
	auto canDriver = prepare_application(arguments);
	if (!canDriver)
	{
		return -1;
	}

	// Create a hidden top-level window so AOG/AgIO can still send us WM_CLOSE.
	// Matches the original ordering: args parsed, CAN driver created, then window.
	WNDCLASS wc = { 0 };
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInstance;
	wc.lpszClassName = TEXT("AOG-TaskController");
	RegisterClass(&wc);

	HWND hwnd = CreateWindowEx(WS_EX_TOOLWINDOW, wc.lpszClassName, TEXT("AOG-TaskController"), WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 300, 200, NULL, NULL, hInstance, NULL);
	if (!hwnd)
	{
		return -1;
	}
	ShowWindow(hwnd, SW_SHOWMINNOACTIVE); // Little hack: Keep the window hidden, but still allows AOG (or other applications) to gracefully close it

	return run_application_loop(canDriver);
}
#else
int main(int argc, char **argv)
{
	install_crash_handlers();

	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);
	std::signal(SIGPIPE, SIG_IGN);

	auto arguments = ParseCommandLine(argc, argv);
	auto canDriver = prepare_application(arguments);
	if (!canDriver)
	{
		return -1;
	}
	return run_application_loop(canDriver);
}
#endif
