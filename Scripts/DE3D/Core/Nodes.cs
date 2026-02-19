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
        private readonly List<Transform> nodes;
        private readonly List<SphereCollider> nodeColliders;
        private readonly List<Vector3> initialNodePositions;
        private readonly List<Quaternion> initialNodeRotations;
        private readonly List<Vector3> previousPositions;
        private readonly List<Vector3> predictedPositions;
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
            nodes = new List<Transform>();
            nodeColliders = new List<SphereCollider>();
            initialNodePositions = new List<Vector3>();
            initialNodeRotations = new List<Quaternion>();
            previousPositions = new List<Vector3>();
            predictedPositions = new List<Vector3>();
            isPinned = new List<bool>();
            ownerTransform = owner;
            nativeArraysAllocated = false;
            disposed = false;
        }

        public IReadOnlyList<Transform> Nodes => nodes;
        public IReadOnlyList<SphereCollider> Colliders => nodeColliders;
        public IReadOnlyList<Vector3> InitialPositions => initialNodePositions;
        public IReadOnlyList<Quaternion> InitialRotations => initialNodeRotations;
        public List<Vector3> PreviousPositions => previousPositions;
        public List<Vector3> PredictedPositions => predictedPositions;
        public IReadOnlyList<bool> IsPinned => isPinned;

        public float NodeRadius => PhysicsConstants.MIN_NODE_RADIUS;

        public float GetNodeRadius()
        {
            return PhysicsConstants.MIN_NODE_RADIUS;
        }

        public void AddNode(Transform transform, Vector3 localPosition)
        {
            if (transform != null)
            {
                nodes.Add(transform);
                initialNodePositions.Add(localPosition);
                initialNodeRotations.Add(transform.localRotation);

                Vector3 worldPosition = transform.position;
                isPinned.Add(false);
                previousPositions.Add(worldPosition);
                predictedPositions.Add(worldPosition);

                SphereCollider collider = transform.GetComponent<SphereCollider>();
                nodeColliders.Add(collider);
            }
            else
            {
                Transform dummyTransform = new GameObject("VirtualNode").transform;
                dummyTransform.gameObject.hideFlags = HideFlags.HideAndDontSave;
                dummyTransform.localPosition = localPosition;

                nodes.Add(dummyTransform);
                initialNodePositions.Add(localPosition);
                initialNodeRotations.Add(Quaternion.identity);

                isPinned.Add(false);
                previousPositions.Add(dummyTransform.position);
                predictedPositions.Add(dummyTransform.position);

                nodeColliders.Add(null);
            }

            if (nativeArraysAllocated)
            {
                AllocateNativeArrays();
            }
        }

        public void AllocateNativeArrays()
        {
            DisposeNativeArrays();

            int count = nodes.Count;
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

            for (int i = 0; i < nodes.Count; i++)
            {
                nativePreviousPositions[i] = previousPositions[i];
                nativePredictedPositions[i] = predictedPositions[i];
                nativeIsPinned[i] = isPinned[i];
            }
        }

        public void SyncFromNativeArrays()
        {
            if (!nativeArraysAllocated) return;

            for (int i = 0; i < nodes.Count; i++)
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
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] == null)
                {
                    Debug.LogWarning($"[NodeManager] Node {i} is null at {context}");
                    continue;
                }

                Vector3 p = nodes[i].localPosition;
                Quaternion q = nodes[i].localRotation;

                if (float.IsNaN(p.x) || float.IsNaN(p.y) || float.IsNaN(p.z) || float.IsInfinity(p.magnitude))
                {
                    Debug.LogError($"[NodeManager] Node {i} POSITION corrupted at {context}. Pos: {p}");
                }

                if (float.IsNaN(q.x) || float.IsNaN(q.y) || float.IsNaN(q.z) || float.IsNaN(q.w))
                {
                    Debug.LogError($"[NodeManager] Node {i} ROTATION corrupted at {context}. Rot: {q}");
                    nodes[i].localRotation = Quaternion.identity;
                }
            }
        }
        public void Initialize(List<Transform> newNodes, Transform owner)
        {
            Clear();
            ownerTransform = owner;

            foreach (var node in newNodes)
            {
                if (node != null)
                {
                    nodes.Add(node);
                    initialNodePositions.Add(node.localPosition);
                    initialNodeRotations.Add(node.localRotation);

                    Vector3 worldPosition = node.position;
                    isPinned.Add(false);
                    previousPositions.Add(worldPosition);
                    predictedPositions.Add(worldPosition);

                    SphereCollider collider = node.GetComponent<SphereCollider>();
                    nodeColliders.Add(collider);
                }
            }

            AllocateNativeArrays();
        }

        public void Clear()
        {
            foreach (var node in nodes)
            {
                if (node != null && node.gameObject != null)
                {
                    if (Application.isEditor && !Application.isPlaying)
                        UnityEngine.Object.DestroyImmediate(node.gameObject);
                    else
                        UnityEngine.Object.Destroy(node.gameObject);
                }
            }
            nodes.Clear();
            nodeColliders.Clear();
            initialNodePositions.Clear();
            initialNodeRotations.Clear();
            previousPositions.Clear();
            predictedPositions.Clear();
            isPinned.Clear();

            DisposeNativeArrays();
        }

        public int FindIndex(System.Func<Transform, bool> predicate)
        {
            if (predicate == null) return -1;
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] != null && predicate(nodes[i]))
                    return i;
            }
            return -1;
        }

        public void SetPinnedNodes(List<bool> pinnedNodes)
        {
            isPinned.Clear();
            isPinned.AddRange(pinnedNodes);

            // Sync to native arrays if allocated
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