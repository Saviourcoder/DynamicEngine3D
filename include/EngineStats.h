/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#pragma once

#include "Export.h"

// Layout must match C# DynamicEngine3D.EngineStats exactly; defined here once.
extern "C" {

    struct EngineStats
    {
        int   totalNodes;
        int   totalBeams;
        int   totalFaces;

        float integrateTimeMs;
        float constraintTimeMs;
        float collisionTimeMs;
        float staticCollisionTimeMs;
        float nodeNodeCollisionTimeMs;
        float faceNodeCollisionTimeMs;
        float deformTimeMs;
        float fps;

        // Collision debug/profiling counters
        int sapPairCount;              // dynamic-dynamic broadphase (SAP) pairs found this frame
        int staticCandidateChecks;     // node-vs-static-collider narrowphase tests performed
        int staticContactsResolved;    // nodes that actually resolved a static collision
        int totalStaticColliders;      // registered static colliders (grid + oversized)
        int staticOversizedColliders;  // colliders too large for the static grid
        int staticGridRebuilt;         // 1 if the static broadphase grid was rebuilt this frame
        int nodeNodeContactsResolved;
        int faceNodeCandidatesChecked; // node candidates tested against triangle faces
        int faceNodeContactsResolved;

        // Thread pool debug
        int threadPoolSize;              // configured background worker pool size
        int peakThreadsUsedThisFrame;    // most workers used in any single dispatch this frame
        int parallelDispatchesThisFrame; // dispatches that actually used the worker pool
        int serialDispatchesThisFrame;   // dispatches that ran on the calling thread only

        // Rigid layer timings, accumulated per substep in Collisions_RigidStep.
        float rigidFaceDetectionTimeMs;  // broadphase AABBs + rigid-rigid + rigid-static narrowphase
        float rigidResolutionTimeMs;     // contact setup, positional pushout, pseudo-velocity writeback
        float rigidSolveTimeMs;          // sequential-impulse iterations, restitution, warm cache
    };

    // Defined once, in SoftBodyInstance.cpp.
    extern EngineStats g_Stats;

    EXPORT void Engine_GetStats(EngineStats* outStats);
    EXPORT void Engine_ClearFrameStats();
}
