using UnityEngine;
using System.Collections.Generic;

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

        // Simple fixed radius configuration 
        private float baseRadius = 0.01f;

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

        // --- DEBUG ADDITION ---
        public void ValidateNodes(string context)
        {
            for(int i = 0; i < nodes.Count; i++)
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
                    // Reset to identity to prevent explosion visual
                    nodes[i].localRotation = Quaternion.identity;
                }
            }
        }
        // ----------------------

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
            }
        }
    }
}