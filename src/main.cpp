#include "app.hpp"
#include "logging.cpp"
#include "settings.hpp"

#include "isobus/hardware_integration/available_can_drivers.hpp"
#include "isobus/isobus/can_stack_logger.hpp"

#include <windows.h>

#include <shellapi.h>
#include <fstream>
#include <iostream>
#include <string>

#define TRAY_ICON_ID 1
static std::atomic_bool running = { true };

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

std::vector<std::string> ParseCommandLine(LPSTR lpCmdLine)
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

enum class CANAdapter
{
	NONE,
	ADAPTER_PCAN_USB,
	ADAPTER_INNOMAKER_USB2CAN,
	ADAPTER_RUSOKU_TOUCAN,
	ADAPTER_SYS_TEC_USB2CAN,
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
			std::cout << "Usage: AOG-TaskController.exe [options]\n";
			std::cout << "Options:\n";
			std::cout << "  --help\t\tShow this help message\n";
			std::cout << "  --version\t\tShow the version of the application\n";
			std::cout << "  --adapter=<driver>\tSelect the CAN driver\n";
		}
		else if ("--version" == option)
		{
			std::cout << PROJECT_VERSION << std::endl;
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

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	std::ofstream logFile;
	std::shared_ptr<isobus::CANHardwarePlugin> canDriver;
	auto arguments = ParseCommandLine(lpCmdLine);

	// Print command line string and version to console
	ArgumentProcessor argumentProcessor(arguments);
	bool argumentsProcessed = argumentProcessor.process();

	// The sequence is important here, first we process the arguments, then we check if the file logging is enabled, then we log the arguments and version to the console/file.
	isobus::CANStackLogger::set_can_stack_logger_sink(&logger);
	if (argumentProcessor.is_file_logging())
	{
		setup_file_logging();
	}

	// Sent the cmd-line arguments and app-version to the console
	for (std::string arg : arguments)
	{
		std::cout << arg.c_str() << " ";
	}
	std::cout << std::endl;
	std::cout << "AOG-TaskController v" << PROJECT_VERSION << std::endl;

	if (!argumentsProcessed)
	{
		std::cout << "Failed to process arguments, exiting..." << std::endl;
		return -1;
	}

	switch (argumentProcessor.get_can_adapter())
	{
		case CANAdapter::ADAPTER_PCAN_USB:
		{
			canDriver = std::make_shared<isobus::PCANBasicWindowsPlugin>(PCAN_USBBUS1 - 1 + std::stoi(argumentProcessor.get_can_channel()));
			break;
		}
		case CANAdapter::ADAPTER_INNOMAKER_USB2CAN:
		{
			canDriver = std::make_shared<isobus::InnoMakerUSB2CANWindowsPlugin>(std::stoi(argumentProcessor.get_can_channel()) - 1);
			break;
		}
		case CANAdapter::ADAPTER_RUSOKU_TOUCAN:
		{
			canDriver = std::make_shared<isobus::TouCANPlugin>(std::stoi(argumentProcessor.get_can_channel()), std::stoi(argumentProcessor.get_can_channel()));
			break;
		}
		case CANAdapter::ADAPTER_SYS_TEC_USB2CAN:
		{
			canDriver = std::make_shared<isobus::SysTecWindowsPlugin>(static_cast<std::uint8_t>(std::stoi(argumentProcessor.get_can_channel())));
			break;
		}
		default:
		{
			std::cout << "No CAN adapter selected, exiting..." << std::endl;
			return -1;
		}
	}
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

	Application app(canDriver);
	if (!app.initialize())
	{
		std::cout << "Failed to initialize application..." << std::endl;
		return -1;
	}

	MSG msg;
	while (running)
	{
		while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		if (!app.update())
		{
			std::cout << "Something unexpected happened, stopping application..." << std::endl;
			break;
		}
	}

	// Clean up
	app.stop();
	return 0;
}
