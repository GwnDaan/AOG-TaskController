#include "app.hpp"

#include "isobus/hardware_integration/available_can_drivers.hpp"

#include <windows.h>

#include <shellapi.h>
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

		std::string arg(requiredSize, 0);
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
	ADAPTER_NTCAN,
};

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	std::shared_ptr<isobus::CANHardwarePlugin> canDriver;
	auto arguments = ParseCommandLine(lpCmdLine);

	// Print command line string to console
	for (std::string arg : arguments)
	{
		std::cout << arg.c_str() << " ";
	}
	std::cout << std::endl;

	// Print version
	std::cout << "AOG-TaskController v" << PROJECT_VERSION << std::endl;

	auto canAdapter = CANAdapter::NONE;
	std::string canChannel;

	// Parse command line options
	for (std::string arg : arguments)
	{
		size_t pos = arg.find('=');
		if (pos == std::string::npos)
		{
			continue;
		}
		std::string key = arg.substr(0, pos);
		std::string value = arg.substr(pos + 1);

		if (_stricmp("--help", key.c_str()) == 0)
		{
			std::cout << "Usage: AOG-TaskController.exe [options]\n";
			std::cout << "Options:\n";
			std::cout << "  --help\t\tShow this help message\n";
			std::cout << "  --version\t\tShow the version of the application\n";
			std::cout << "  --adapter=<driver>\tSelect the CAN driver\n";
			return 0;
		}
		else if (_stricmp("--version", key.c_str()) == 0)
		{
			std::cout << PROJECT_VERSION << std::endl;
		}
		else if (_stricmp("--can_adapter", key.c_str()) == 0)
		{
			if (_stricmp("PEAK-PCAN", value.c_str()) == 0)
			{
				canAdapter = CANAdapter::ADAPTER_PCAN_USB;
			}
			else if (_stricmp("InnoMaker-USB2CAN", value.c_str()) == 0)
			{
				canAdapter = CANAdapter::ADAPTER_INNOMAKER_USB2CAN;
			}
			else if (_stricmp("Rusoku-TouCAN", value.c_str()) == 0)
			{
				canAdapter = CANAdapter::ADAPTER_RUSOKU_TOUCAN;
			}
			else if (_stricmp("SYS-TEC-USB2CAN", value.c_str()) == 0)
			{
				canAdapter = CANAdapter::ADAPTER_SYS_TEC_USB2CAN;
			}
			else if (_stricmp("NTCAN", value.c_str()) == 0)
			{
				canAdapter = CANAdapter::ADAPTER_NTCAN;
			}
			else
			{
				std::cout << "Unknown CAN adapter: " << value.c_str() << std::endl;
				return -1;
			}
		}
		else if (_strcmpi("--can_channel", key.c_str()) == 0)
		{
			canChannel = value;
		}
	}

	switch (canAdapter)
	{
		case CANAdapter::ADAPTER_PCAN_USB:
		{
			canDriver = std::make_shared<isobus::PCANBasicWindowsPlugin>(PCAN_USBBUS1 - 1 + std::stoi(canChannel));
			break;
		}
		case CANAdapter::ADAPTER_INNOMAKER_USB2CAN:
		{
			canDriver = std::make_shared<isobus::InnoMakerUSB2CANWindowsPlugin>(std::stoi(canChannel) - 1);
			break;
		}
		case CANAdapter::ADAPTER_RUSOKU_TOUCAN:
		{
			canDriver = std::make_shared<isobus::TouCANPlugin>(std::stoi(canChannel), std::stoi(canChannel));
			break;
		}
		case CANAdapter::ADAPTER_SYS_TEC_USB2CAN:
		{
			canDriver = std::make_shared<isobus::SysTecWindowsPlugin>(static_cast<std::uint8_t>(std::stoi(canChannel)));
			break;
		}
		case CANAdapter::ADAPTER_NTCAN:
		{
			canDriver = std::make_shared<isobus::NTCANPlugin>(std::stoi(canChannel));
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
