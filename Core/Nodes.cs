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
            if (transform == null) return;

            nodes.Add(transform);
            initialNodePositions.Add(localPosition);
            initialNodeRotations.Add(transform.localRotation);
            isPinned.Add(false);

            SphereCollider collider = transform.GetComponent<SphereCollider>();
            if (collider == null) collider = transform.gameObject.AddComponent<SphereCollider>();
            collider.radius = baseRadius;
            collider.isTrigger = false;
            nodeColliders.Add(collider);
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
    }
}
