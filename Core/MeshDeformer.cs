/* DynamicEngine3D - Mesh Deformation
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
    public class MeshDeformer
    {
        public readonly Mesh mesh;
        private readonly Vector3[] originalVertices;
        private readonly Dictionary<int, List<(int vertexIndex, float weight)>> nodeVertexWeights;
        private readonly float influenceRadius;
        private Vector3[] deformedVertices;

        public MeshDeformer(Mesh mesh, Vector3[] originalVertices, float influenceRadius)
        {
            this.mesh = mesh ?? new Mesh();
            this.originalVertices = originalVertices ?? new Vector3[0];
            this.influenceRadius = Mathf.Max(0.01f, influenceRadius);
            deformedVertices = new Vector3[this.originalVertices.Length];
            System.Array.Copy(this.originalVertices, deformedVertices, this.originalVertices.Length);
            nodeVertexWeights = new Dictionary<int, List<(int, float)>>();
        }

        public Mesh Mesh => mesh;

        public void MapVerticesToNodes(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            if (transform == null || nodes == null || initialPositions == null) return;

            nodeVertexWeights.Clear();
            Matrix4x4 localToWorld = transform.localToWorldMatrix;

            for (int i = 0; i < originalVertices.Length; i++)
            {
                Vector3 worldVertex = localToWorld.MultiplyPoint3x4(originalVertices[i]);
                Dictionary<int, float> weights = new Dictionary<int, float>();
                float totalWeight = 0f;

                for (int j = 0; j < nodes.Count; j++)
                {
                    if (nodes[j] == null || j >= initialPositions.Count) continue;
                    Vector3 worldNodePos = transform.TransformPoint(initialPositions[j]);
                    float distance = Vector3.Distance(worldVertex, worldNodePos);
                    if (distance <= influenceRadius)
                    {
                        float weight = 1f - (distance / influenceRadius); // Linear falloff
                        if (weight > 0f)
                        {
                            weights[j] = weight;
                            totalWeight += weight;
                        }
                    }
                }

                if (totalWeight > 0f)
                {
                    foreach (var pair in weights)
                    {
                        int nodeIndex = pair.Key;
                        float weight = pair.Value / totalWeight; // Normalize weights
                        if (!nodeVertexWeights.ContainsKey(nodeIndex))
                            nodeVertexWeights[nodeIndex] = new List<(int, float)>();
                        nodeVertexWeights[nodeIndex].Add((i, weight));
                    }
                }
            }
        }

        public void Deform(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            if (transform == null || nodes == null || initialPositions == null || originalVertices == null || deformedVertices == null || nodeVertexWeights == null)
            {
                Debug.LogWarning("Cannot deform mesh: Invalid input data.");
                return;
            }

            System.Array.Copy(originalVertices, deformedVertices, originalVertices.Length);

            for (int i = 0; i < nodes.Count; i++)
            {
                if (!nodeVertexWeights.ContainsKey(i) || nodes[i] == null || i >= initialPositions.Count) continue;

                Vector3 initialWorldPos = transform.TransformPoint(initialPositions[i]);
                Vector3 currentWorldPos = nodes[i].position;
                Vector3 offsetWorld = currentWorldPos - initialWorldPos;

                foreach (var (vertexIndex, weight) in nodeVertexWeights[i])
                {
                    if (vertexIndex >= originalVertices.Length || vertexIndex >= deformedVertices.Length) continue;
                    Vector3 localOffset = transform.InverseTransformVector(offsetWorld);
                    deformedVertices[vertexIndex] += localOffset * weight;
                }
            }

            mesh.vertices = deformedVertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
        }
    }
}
