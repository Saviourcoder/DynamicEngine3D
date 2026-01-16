/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers */
using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace DynamicEngine
{
    public class NodeManager
    {
        private readonly List<Transform> nodes;
        private readonly List<SphereCollider> nodeColliders;
        private readonly List<Vector3> initialNodePositions;
        private readonly List<Quaternion> initialNodeRotations;
        private readonly List<Vector3> previousPositions;
        private readonly List<Vector3> predictedPositions;
        private readonly List<bool> isPinned;
        private Transform ownerTransform;

        private float baseRadius = 0.001f;

        public NativeArray<float3> nativeInitialPositions;
        public NativeArray<float3> nativePreviousPositions;
        public NativeArray<float3> nativePredictedPositions;
        public NativeArray<quaternion> nativeInitialRotations;
        public NativeArray<bool> nativeIsPinned;
        public NativeArray<float> nativeRadii;

        private bool nativeArraysAllocated;

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
        }

        public IReadOnlyList<Transform> Nodes => nodes;
        public IReadOnlyList<SphereCollider> Colliders => nodeColliders;
        public IReadOnlyList<Vector3> InitialPositions => initialNodePositions;
        public IReadOnlyList<Quaternion> InitialRotations => initialNodeRotations;
        public List<Vector3> PreviousPositions => previousPositions;
        public List<Vector3> PredictedPositions => predictedPositions;
        public IReadOnlyList<bool> IsPinned => isPinned;

        public float BaseRadius
        {
            get => baseRadius;
            set => baseRadius = Mathf.Clamp(value, 0.001f, 1f);
        }

        public float GetNodeRadius(int nodeIndex)
        {
            return baseRadius;
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
                if (ownerTransform != null) dummyTransform.SetParent(ownerTransform, false);
                dummyTransform.localPosition = localPosition;

                nodes.Add(dummyTransform);
                initialNodePositions.Add(localPosition);
                initialNodeRotations.Add(Quaternion.identity);

                isPinned.Add(false);
                previousPositions.Add(dummyTransform.position);
                predictedPositions.Add(dummyTransform.position);

                nodeColliders.Add(null);
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
            nativeRadii = new NativeArray<float>(count, Allocator.Persistent);

            for (int i = 0; i < count; i++)
            {
                nativeInitialPositions[i] = initialNodePositions[i];
                nativePreviousPositions[i] = previousPositions[i];
                nativePredictedPositions[i] = predictedPositions[i];
                nativeInitialRotations[i] = initialNodeRotations[i];
                nativeIsPinned[i] = isPinned[i];
                nativeRadii[i] = baseRadius;
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
            }
        }

        public void SyncFromNativeArrays()
        {
            if (!nativeArraysAllocated) return;

            for (int i = 0; i < nodes.Count; i++)
            {
                previousPositions[i] = nativePreviousPositions[i];
                predictedPositions[i] = nativePredictedPositions[i];
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
                if (nativeRadii.IsCreated) nativeRadii.Dispose();
                nativeArraysAllocated = false;
            }
        }

        public void ValidateNodes(string context)
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] == null) continue;

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

        public void Clear()
        {
            foreach (var node in nodes)
            {
                if (node != null)
                {
                    if (Application.isEditor && !Application.isPlaying)
                        Object.DestroyImmediate(node.gameObject);
                    else
                        Object.Destroy(node.gameObject);
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
        }

        public void SetPinned(int index, bool pinned)
        {
            if (index >= 0 && index < isPinned.Count)
            {
                isPinned[index] = pinned;
                if (nativeArraysAllocated && nativeIsPinned.IsCreated)
                {
                    nativeIsPinned[index] = pinned;
                }
            }
        }
    }
}
