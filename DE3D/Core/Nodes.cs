/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using System;

namespace DynamicEngine
{
    public class NodeManager : IDisposable
    {
        private readonly List<Vector3> currentPositions;
        private readonly List<Quaternion> currentRotations;

        private readonly List<Vector3> initialNodePositions;
        private readonly List<Quaternion> initialNodeRotations;
        private readonly List<Vector3> previousPositions;
        private readonly List<Vector3> predictedPositions;
        private readonly List<Vector3> postIntegrationPositions;
        private readonly List<bool> isPinned;
        private Transform ownerTransform;

        public NativeArray<float3> nativeInitialPositions;
        public NativeArray<float3> nativePreviousPositions;
        public NativeArray<float3> nativePredictedPositions;
        public NativeArray<quaternion> nativeInitialRotations;
        public NativeArray<bool> nativeIsPinned;

        private bool nativeArraysAllocated;
        private bool disposed;

        public NodeManager(Transform owner = null)
        {
            currentPositions = new List<Vector3>();
            currentRotations = new List<Quaternion>();
            initialNodePositions = new List<Vector3>();
            initialNodeRotations = new List<Quaternion>();
            previousPositions = new List<Vector3>();
            predictedPositions = new List<Vector3>();
            postIntegrationPositions = new List<Vector3>();
            isPinned = new List<bool>();
            ownerTransform = owner;
            nativeArraysAllocated = false;
            disposed = false;
        }

        public int NodeCount => currentPositions.Count;
        public IReadOnlyList<Vector3> CurrentPositions => currentPositions;
        public List<Quaternion> CurrentRotations => currentRotations;
        public IReadOnlyList<Vector3> InitialPositions => initialNodePositions;
        public IReadOnlyList<Quaternion> InitialRotations => initialNodeRotations;
        public List<Vector3> PreviousPositions => previousPositions;
        public List<Vector3> PredictedPositions => predictedPositions;
        public List<Vector3> PostIntegrationPositions => postIntegrationPositions;
        public IReadOnlyList<bool> IsPinned => isPinned;

        public float NodeRadius => PhysicsConstants.MIN_NODE_RADIUS;

        public float GetNodeRadius()
        {
            return PhysicsConstants.MIN_NODE_RADIUS;
        }

        public void AddNode(Vector3 localPosition, Vector3 worldPosition, Quaternion localRotation)
        {
            currentPositions.Add(worldPosition);
            currentRotations.Add(localRotation);
            initialNodePositions.Add(localPosition);
            initialNodeRotations.Add(localRotation);

            isPinned.Add(false);
            previousPositions.Add(worldPosition);
            predictedPositions.Add(worldPosition);
            postIntegrationPositions.Add(worldPosition);
        }

        public Vector3 GetPosition(int index)
        {
            return currentPositions[index];
        }
        public void SetCurrentPosition(int index, Vector3 value)
        {
            currentPositions[index] = value;
        }

        public void SetPreviousPosition(int index, Vector3 value)
        {
            previousPositions[index] = value;
        }

        public void AllocateNativeArrays()
        {
            if (nativeArraysAllocated
                && nativeInitialPositions.IsCreated
                && nativeInitialPositions.Length == NodeCount)
                return;

            DisposeNativeArrays();

            int count = NodeCount;
            nativeInitialPositions = new NativeArray<float3>(count, Allocator.Persistent);
            nativePreviousPositions = new NativeArray<float3>(count, Allocator.Persistent);
            nativePredictedPositions = new NativeArray<float3>(count, Allocator.Persistent);
            nativeInitialRotations = new NativeArray<quaternion>(count, Allocator.Persistent);
            nativeIsPinned = new NativeArray<bool>(count, Allocator.Persistent);

            for (int i = 0; i < count; i++)
            {
                nativeInitialPositions[i] = initialNodePositions[i];
                nativePreviousPositions[i] = previousPositions[i];
                nativePredictedPositions[i] = predictedPositions[i];
                nativeInitialRotations[i] = initialNodeRotations[i];
                nativeIsPinned[i] = isPinned[i];
            }

            nativeArraysAllocated = true;
        }

        public void SyncToNativeArrays()
        {
            if (!nativeArraysAllocated) return;

            for (int i = 0; i < NodeCount; i++)
            {
                nativePreviousPositions[i] = previousPositions[i];
                nativePredictedPositions[i] = predictedPositions[i];
                nativeIsPinned[i] = isPinned[i];
            }
        }

        public void SyncFromNativeArrays()
        {
            if (!nativeArraysAllocated) return;

            for (int i = 0; i < NodeCount; i++)
            {
                previousPositions[i] = nativePreviousPositions[i];
                predictedPositions[i] = nativePredictedPositions[i];
                isPinned[i] = nativeIsPinned[i];
            }
        }

        public void DisposeNativeArrays()
        {
            if (nativeArraysAllocated)
            {
                if (nativeInitialPositions.IsCreated) nativeInitialPositions.Dispose();
                if (nativePreviousPositions.IsCreated) nativePreviousPositions.Dispose();
                if (nativePredictedPositions.IsCreated) nativePredictedPositions.Dispose();
                if (nativeInitialRotations.IsCreated) nativeInitialRotations.Dispose();
                if (nativeIsPinned.IsCreated) nativeIsPinned.Dispose();
                nativeArraysAllocated = false;
            }
        }

        public void ValidateNodes(string context)
        {
            for (int i = 0; i < NodeCount; i++)
            {
                Vector3 p = currentPositions[i];
                Quaternion q = currentRotations[i];

                if (float.IsNaN(p.x) || float.IsNaN(p.y) || float.IsNaN(p.z) || float.IsInfinity(p.magnitude))
                {
                    Debug.LogError($"[NodeManager] Node {i} POSITION corrupted at {context}. Pos: {p}");
                }

                if (float.IsNaN(q.x) || float.IsNaN(q.y) || float.IsNaN(q.z) || float.IsNaN(q.w))
                {
                    Debug.LogError($"[NodeManager] Node {i} ROTATION corrupted at {context}. Rot: {q}");
                    currentRotations[i] = Quaternion.identity;
                }
            }
        }

        public void Initialize(List<Vector3> newPositions, Transform owner)
        {
            Clear();
            ownerTransform = owner;

            foreach (var pos in newPositions)
            {
                AddNode(pos, pos, Quaternion.identity);
            }

            AllocateNativeArrays();
        }

        public void Clear()
        {
            currentPositions.Clear();
            currentRotations.Clear();
            initialNodePositions.Clear();
            initialNodeRotations.Clear();
            previousPositions.Clear();
            predictedPositions.Clear();
            postIntegrationPositions.Clear();
            isPinned.Clear();

            DisposeNativeArrays();
        }

        public int FindIndex(Func<Vector3, bool> predicate)
        {
            if (predicate == null) return -1;
            for (int i = 0; i < currentPositions.Count; i++)
            {
                if (predicate(currentPositions[i]))
                    return i;
            }
            return -1;
        }

        public void SetPinnedNodes(List<bool> pinnedNodes)
        {
            isPinned.Clear();
            isPinned.AddRange(pinnedNodes);

            if (nativeArraysAllocated && nativeIsPinned.IsCreated)
            {
                for (int i = 0; i < isPinned.Count && i < nativeIsPinned.Length; i++)
                {
                    nativeIsPinned[i] = isPinned[i];
                }
            }
        }

        public void SetPinned(int index, bool pinned)
        {
            if (index >= 0 && index < isPinned.Count)
            {
                isPinned[index] = pinned;
                if (nativeArraysAllocated && nativeIsPinned.IsCreated && index < nativeIsPinned.Length)
                {
                    nativeIsPinned[index] = pinned;
                }
            }
        }

        public void Dispose()
        {
            if (disposed) return;

            DisposeNativeArrays();
            Clear();
            disposed = true;
        }

        ~NodeManager()
        {
            if (!disposed)
            {
                DisposeNativeArrays();
            }
        }
    }
}
