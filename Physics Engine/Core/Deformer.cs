/* DynamicEngine3D - Advanced Mesh Deformation for Complex Geometries
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
using System.Linq;

namespace DynamicEngine
{
    public class MeshDeformer
    {
        public readonly Mesh mesh;
        private readonly Vector3[] originalVertices;
        private readonly List<VertexInfluence>[] vertexInfluences;
        private readonly float influenceRadius;
        private Vector3[] deformedVertices;
        private readonly int maxInfluences = 4;
        private bool useAdvancedSkinning = true;

        // For advanced skinning - we'll create a virtual cage around the mesh
        private List<Vector3> cageNodes;
        private Dictionary<int, List<int>> nodeToVerticesMap;

        public MeshDeformer(Mesh mesh, Vector3[] originalVertices, float influenceRadius)
        {
            this.mesh = mesh ?? new Mesh();
            this.originalVertices = originalVertices ?? new Vector3[0];
            this.influenceRadius = Mathf.Max(0.01f, influenceRadius);
            deformedVertices = new Vector3[this.originalVertices.Length];
            System.Array.Copy(this.originalVertices, deformedVertices, this.originalVertices.Length);

            vertexInfluences = new List<VertexInfluence>[this.originalVertices.Length];
            for (int i = 0; i < vertexInfluences.Length; i++)
            {
                vertexInfluences[i] = new List<VertexInfluence>();
            }

            cageNodes = new List<Vector3>();
            nodeToVerticesMap = new Dictionary<int, List<int>>();

            if (this.mesh != null)
            {
                this.mesh.MarkDynamic();
            }
        }

        public Mesh Mesh => mesh;
        public float InfluenceRadius => influenceRadius;

        [System.Serializable]
        public struct VertexInfluence
        {
            public int nodeIndex;
            public float weight;
            public Vector3 localOffset; // Offset from node to vertex in local space

            public VertexInfluence(int nodeIndex, float weight, Vector3 localOffset)
            {
                this.nodeIndex = nodeIndex;
                this.weight = weight;
                this.localOffset = localOffset;
            }
        }

        public void MapVerticesToNodes(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            if (transform == null || nodes == null || initialPositions == null || nodes.Count == 0)
                return;

            // Clear previous mappings
            for (int i = 0; i < vertexInfluences.Length; i++)
            {
                vertexInfluences[i].Clear();
            }
            nodeToVerticesMap.Clear();
            cageNodes.Clear();

            // Build a bounding volume hierarchy for the nodes
            BuildNodeBVH(nodes, initialPositions);

            if (useAdvancedSkinning && nodes.Count > 8)
            {
                UseAdvancedVolumeSkinning(nodes, initialPositions);
            }
            else
            {
                UseDistanceBasedSkinning(nodes, initialPositions);
            }

        }

        private void BuildNodeBVH(IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            cageNodes.Clear();
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] != null && i < initialPositions.Count)
                {
                    cageNodes.Add(initialPositions[i]);
                }
            }
        }

        private void UseAdvancedVolumeSkinning(IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            // For complex meshes with fewer nodes, use barycentric coordinates within the node convex hull
            for (int vertexIndex = 0; vertexIndex < originalVertices.Length; vertexIndex++)
            {
                Vector3 vertex = originalVertices[vertexIndex];
                var closestNodes = FindClosestNodesWithWeights(vertex, nodes, initialPositions, 4);

                foreach (var (nodeIndex, weight, distance) in closestNodes)
                {
                    if (weight > 0.01f)
                    {
                        Vector3 localOffset = vertex - initialPositions[nodeIndex];
                        vertexInfluences[vertexIndex].Add(new VertexInfluence(nodeIndex, weight, localOffset));
                    }
                }

                // Ensure we have at least one influence
                if (vertexInfluences[vertexIndex].Count == 0 && nodes.Count > 0)
                {
                    int closestNode = FindClosestNode(vertex, initialPositions);
                    Vector3 localOffset = vertex - initialPositions[closestNode];
                    vertexInfluences[vertexIndex].Add(new VertexInfluence(closestNode, 1.0f, localOffset));
                }
            }
        }

        private void UseDistanceBasedSkinning(IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            // Method 2: Traditional distance-based weighting for simpler geometries
            float radiusSquared = influenceRadius * influenceRadius;

            for (int vertexIndex = 0; vertexIndex < originalVertices.Length; vertexIndex++)
            {
                Vector3 vertex = originalVertices[vertexIndex];
                var influences = new List<(int nodeIndex, float weight)>();

                for (int nodeIndex = 0; nodeIndex < nodes.Count; nodeIndex++)
                {
                    if (nodes[nodeIndex] == null || nodeIndex >= initialPositions.Count)
                        continue;

                    Vector3 nodePos = initialPositions[nodeIndex];
                    float distanceSquared = (vertex - nodePos).sqrMagnitude;

                    if (distanceSquared <= radiusSquared)
                    {
                        float distance = Mathf.Sqrt(distanceSquared);
                        float weight = 1.0f - (distance / influenceRadius);
                        influences.Add((nodeIndex, weight));
                    }
                }

                // Sort by weight and take top influences
                influences.Sort((a, b) => b.weight.CompareTo(a.weight));

                float totalWeight = 0f;
                int count = Mathf.Min(influences.Count, maxInfluences);

                for (int i = 0; i < count; i++)
                {
                    totalWeight += influences[i].weight;
                }

                // Normalize and assign
                for (int i = 0; i < count; i++)
                {
                    if (totalWeight > 0)
                    {
                        float normalizedWeight = influences[i].weight / totalWeight;
                        Vector3 localOffset = vertex - initialPositions[influences[i].nodeIndex];
                        vertexInfluences[vertexIndex].Add(new VertexInfluence(influences[i].nodeIndex, normalizedWeight, localOffset));
                    }
                }

                // Fallback to closest node if no influences
                if (vertexInfluences[vertexIndex].Count == 0 && nodes.Count > 0)
                {
                    int closestNode = FindClosestNode(vertex, initialPositions);
                    Vector3 localOffset = vertex - initialPositions[closestNode];
                    vertexInfluences[vertexIndex].Add(new VertexInfluence(closestNode, 1.0f, localOffset));
                }
            }
        }

        private List<(int nodeIndex, float weight, float distance)> FindClosestNodesWithWeights(
            Vector3 vertex, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions, int maxNodes)
        {
            var nodeDistances = new List<(int index, float distance)>();

            // Find distances to all nodes
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] != null && i < initialPositions.Count)
                {
                    float distance = Vector3.Distance(vertex, initialPositions[i]);
                    nodeDistances.Add((i, distance));
                }
            }

            // Sort by distance
            nodeDistances.Sort((a, b) => a.distance.CompareTo(b.distance));

            // Take closest nodes
            var result = new List<(int, float, float)>();
            int takeCount = Mathf.Min(maxNodes, nodeDistances.Count);

            if (takeCount == 0) return result;

            // Calculate weights using inverse distance weighting
            float totalInverseWeight = 0f;
            for (int i = 0; i < takeCount; i++)
            {
                float invWeight = 1.0f / (nodeDistances[i].distance + 0.001f); // Avoid division by zero
                totalInverseWeight += invWeight;
                result.Add((nodeDistances[i].index, invWeight, nodeDistances[i].distance));
            }

            // Normalize weights
            for (int i = 0; i < result.Count; i++)
            {
                var (index, weight, distance) = result[i];
                result[i] = (index, weight / totalInverseWeight, distance);
            }

            return result;
        }

        private int FindClosestNode(Vector3 position, IReadOnlyList<Vector3> initialPositions)
        {
            if (initialPositions == null || initialPositions.Count == 0) return 0;

            int best = 0;
            float minDistSq = float.MaxValue;

            for (int i = 0; i < initialPositions.Count; i++)
            {
                float distSq = (position - initialPositions[i]).sqrMagnitude;
                if (distSq < minDistSq)
                {
                    minDistSq = distSq;
                    best = i;
                }
            }

            return best;
        }

        public void Deform(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions, IReadOnlyList<Quaternion> initialRotations)
        {
            if (transform == null || nodes == null || initialPositions == null || initialRotations == null)
            {
                Debug.LogWarning("Cannot deform mesh: Invalid input data.");
                return;
            }

            // Reset to original positions
            System.Array.Copy(originalVertices, deformedVertices, originalVertices.Length);

            // Apply deformation using pre-computed influences
            for (int vertexIndex = 0; vertexIndex < deformedVertices.Length; vertexIndex++)
            {
                if (vertexInfluences[vertexIndex].Count == 0) continue;

                Vector3 deformedPosition = Vector3.zero;
                float totalWeight = 0f;

                foreach (var influence in vertexInfluences[vertexIndex])
                {
                    if (influence.nodeIndex >= nodes.Count || nodes[influence.nodeIndex] == null ||
                        influence.nodeIndex >= initialPositions.Count || influence.nodeIndex >= initialRotations.Count)
                        continue;

                    // Get current node position and rotation in local space
                    Vector3 currentWorldPos = nodes[influence.nodeIndex].position;
                    Vector3 currentLocalPos = transform.InverseTransformPoint(currentWorldPos);

                    Quaternion currentWorldRot = nodes[influence.nodeIndex].rotation;
                    Quaternion currentLocalRot = Quaternion.Inverse(transform.rotation) * currentWorldRot;

                    // Get initial rotation
                    Quaternion initialRot = initialRotations[influence.nodeIndex];

                    // Calculate the rotation from initial to current
                    Quaternion rotationDelta = currentLocalRot * Quaternion.Inverse(initialRot);

                    Vector3 rotatedOffset = rotationDelta * influence.localOffset;
                    Vector3 targetPosition = currentLocalPos + rotatedOffset;

                    deformedPosition += targetPosition * influence.weight;
                    totalWeight += influence.weight;
                }

                // Apply weighted deformation
                if (totalWeight > 0.001f)
                {
                    deformedVertices[vertexIndex] = deformedPosition;
                }
            }

            // Apply to mesh
            mesh.vertices = deformedVertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            if (mesh.tangents != null && mesh.tangents.Length > 0)
            {
                mesh.RecalculateTangents();
            }

            mesh.UploadMeshData(false);
        }

        public void SetSkinningMethod(bool useAdvanced)
        {
            useAdvancedSkinning = useAdvanced;
        }

        public void DebugInfluences(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            if (!Application.isPlaying) return;

            for (int i = 0; i < Mathf.Min(100, vertexInfluences.Length); i += 10) // Sample every 10th vertex
            {
                if (vertexInfluences[i].Count > 0)
                {
                    Vector3 vertexWorldPos = transform.TransformPoint(originalVertices[i]);
                    Color influenceColor = Color.Lerp(Color.blue, Color.red, vertexInfluences[i][0].weight);

                    foreach (var influence in vertexInfluences[i])
                    {
                        if (influence.nodeIndex < nodes.Count && nodes[influence.nodeIndex] != null)
                        {
                            Vector3 nodeWorldPos = nodes[influence.nodeIndex].position;
                            Debug.DrawLine(vertexWorldPos, nodeWorldPos, influenceColor, 0.1f);
                        }
                    }
                }
            }
        }
    }
}