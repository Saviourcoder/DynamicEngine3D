/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#include "Error.h"
#include <string>
#include <sstream>
#include <mutex>

thread_local ErrorCode g_LastError = ErrorCode::None;
thread_local std::string g_LastStackTrace = "";

#if defined(_WIN32)
#include <windows.h>
#include <dbghelp.h>
#pragma comment(lib, "dbghelp.lib")

static bool s_symInitialized = false;
static std::mutex s_dbgHelpMutex;

static void EnsureSymInit(HANDLE process)
{
    if (s_symInitialized) return;

    SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES);
    s_symInitialized = SymInitialize(process, NULL, TRUE) != FALSE;
}

static void CaptureStackTrace()
{
    std::stringstream ss;
    void* stack[32];

    // Capture up to 32 frames, skipping the current CaptureStackTrace frame.
    USHORT frames = CaptureStackBackTrace(1, 32, stack, NULL);
    HANDLE process = GetCurrentProcess();

    std::lock_guard<std::mutex> lock(s_dbgHelpMutex);
    EnsureSymInit(process);

    char buffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME * sizeof(TCHAR)];
    PSYMBOL_INFO symbol = (PSYMBOL_INFO)buffer;
    symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
    symbol->MaxNameLen = MAX_SYM_NAME;

    IMAGEHLP_LINE64 line;
    line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);
    DWORD displacement = 0;

    for (USHORT i = 0; i < frames; i++)
    {
        DWORD64 address = (DWORD64)(stack[i]);

        if (SymFromAddr(process, address, 0, symbol))
        {
            ss << "  at " << symbol->Name;

            if (SymGetLineFromAddr64(process, address, &displacement, &line))
            {
                ss << " (" << line.FileName << ":" << line.LineNumber << ")";
            }
            ss << " [0x" << std::hex << symbol->Address << "]\n";
        }
        else
        {
            // Symbol lookup failed � usually a missing/mismatched PDB for
            // whatever copy of this module is actually loaded.
            ss << "  at 0x" << std::hex << address << "\n";
        }
    }

    g_LastStackTrace = ss.str();
}

static void ShutdownSym()
{
    std::lock_guard<std::mutex> lock(s_dbgHelpMutex);
    if (!s_symInitialized) return;
    SymCleanup(GetCurrentProcess());
    s_symInitialized = false;
}

#else // !_WIN32

static void CaptureStackTrace()
{
    g_LastStackTrace = "(stack traces are only implemented on Windows)";
}

static void ShutdownSym() {}

#endif

void Error_SetError(ErrorCode code)
{
    g_LastError = code;
    if (code != ErrorCode::None)
    {
        CaptureStackTrace();
    }
    else
    {
        g_LastStackTrace.clear();
    }
}

EXPORT int Error_GetLastAndClear()
{
    int temp = static_cast<int>(g_LastError);
    g_LastError = ErrorCode::None;
    return temp;
}

EXPORT const char* Error_GetLastStackTrace()
{
    return g_LastStackTrace.c_str();
}

EXPORT void Error_Clear()
{
    g_LastError = ErrorCode::None;
    g_LastStackTrace.clear();
}

EXPORT void Error_Shutdown()
{
    ShutdownSym();
}