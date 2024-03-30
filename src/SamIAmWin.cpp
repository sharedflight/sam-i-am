#if IBM
#include <winsock2.h>
#include <windows.h>

#include "acfutils/glew.h"

//#///pragma comment(lib, "Ws2_32.lib")

BOOL APIENTRY DllMain(HANDLE hModule,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}

	lacf_glew_dllmain_hook(ul_reason_for_call);

	return TRUE;
}
#endif