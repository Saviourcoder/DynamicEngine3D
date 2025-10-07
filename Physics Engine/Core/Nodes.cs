// Filename: Nodes.cs
/* DynamicEngine3D - Node Management
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/

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

        // Simple fixed radius configuration 
        private float baseRadius = 0.01f;

        public NodeManager()
        {
            nodes = new List<Transform>();
            nodeColliders = new List<SphereCollider>();
            initialNodePositions = new List<Vector3>();
            initialNodeRotations = new List<Quaternion>();
            previousPositions = new List<Vector3>();
            predictedPositions = new List<Vector3>();
            isPinned = new List<bool>();
        }

        public IReadOnlyList<Transform> Nodes => nodes;
        public IReadOnlyList<SphereCollider> Colliders => nodeColliders;
        public IReadOnlyList<Vector3> InitialPositions => initialNodePositions;
        public IReadOnlyList<Quaternion> InitialRotations => initialNodeRotations;
        public List<Vector3> PreviousPositions => previousPositions;
        public List<Vector3> PredictedPositions => predictedPositions;
        public IReadOnlyList<bool> IsPinned => isPinned;

        // Simple radius property
        public float BaseRadius
        {
            get => baseRadius;
            set => baseRadius = Mathf.Clamp(value, 0.001f, 1f);
        }

        // Get radius for any node (simplified)
        public float GetNodeRadius(int nodeIndex)
        {
            return baseRadius;
        }

        public void AddNode(Transform transform, Vector3 localPosition)
        {
            // If user provided a real Transform, store it
            if (transform != null)
            {
                nodes.Add(transform);
                initialNodePositions.Add(localPosition);
                initialNodeRotations.Add(transform.localRotation);
                
                // Initialize world space positions for physics solver
                Vector3 worldPosition = transform.position; // Node's current world position
                isPinned.Add(false);
                previousPositions.Add(worldPosition);
                predictedPositions.Add(worldPosition);
                
                // Store the collider component if it exists
                SphereCollider collider = transform.GetComponent<SphereCollider>();
                nodeColliders.Add(collider);
            }
            else
            {
                // Create a "virtual node" – no GameObject, just store position
                GameObject dummy = null;
                Transform dummyTransform = new GameObject().transform;
                //remove this later in full refactor. For now we keep a Transform reference for compatibility,
                // but it will never enter the scene hierarchy or have a collider.

                dummyTransform.gameObject.hideFlags = HideFlags.HideAndDontSave;
                nodes.Add(dummyTransform);
                initialNodePositions.Add(localPosition);
                initialNodeRotations.Add(Quaternion.identity);
                
                isPinned.Add(false);
                previousPositions.Add(localPosition); // For virtual nodes, treat as world position
                predictedPositions.Add(localPosition);
                
                // Virtual nodes don't have colliders
                nodeColliders.Add(null);
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