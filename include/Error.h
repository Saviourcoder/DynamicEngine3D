/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#pragma once
#include "Export.h"

enum class ErrorCode : int
{
    None = 0,
    NullHandle = 1, // non existent body
    UnknownHandle = 2, // non registered body
    InvalidNodeIndex = 3, // node/beam/face/contact index out of range
    InvalidDt = 4, // invalid deltatime
    InvalidSubstep = 5, // invalid substeps
    InvalidArgument = 6, // null buffer / bad count / bad value that isn't an index
    BufferTooSmall = 7  // caller-provided output buffer too small for the data
};

extern thread_local ErrorCode g_LastError; // storage for last error

// Records an error
void Error_SetError(ErrorCode code);

EXPORT int Error_GetLastAndClear();
EXPORT const char* Error_GetLastStackTrace();

// Drops a latched error and its trace without reporting it — session-start hygiene.
EXPORT void Error_Clear();

// once at plugin/DLL shutdown to release DbgHelp resources.
EXPORT void Error_Shutdown();