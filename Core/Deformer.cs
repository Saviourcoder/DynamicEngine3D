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
        private readonly Dictionary<int, List<(int vertexIndex, float weight, Vector3 localOffset)>> nodeVertexWeights;
        private readonly float influenceRadius;
        private Vector3[] deformedVertices;

        public MeshDeformer(Mesh mesh, Vector3[] originalVertices, float influenceRadius)
        {
            this.mesh = mesh ?? new Mesh();
            this.originalVertices = originalVertices ?? new Vector3[0];
            this.influenceRadius = Mathf.Max(0.01f, influenceRadius);
            deformedVertices = new Vector3[this.originalVertices.Length];
            System.Array.Copy(this.originalVertices, deformedVertices, this.originalVertices.Length);
            nodeVertexWeights = new Dictionary<int, List<(int, float, Vector3)>>();
            
            // Mark mesh as dynamic for better performance with frequent updates
            if (this.mesh != null)
            {
                this.mesh.MarkDynamic();
            }
        }

        public Mesh Mesh => mesh;
        public float InfluenceRadius => influenceRadius;

        public void MapVerticesToNodes(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            if (transform == null || nodes == null || initialPositions == null) return;

            nodeVertexWeights.Clear();

            // Work entirely in local space - no world space conversions needed
            for (int i = 0; i < originalVertices.Length; i++)
            {
                Vector3 localVertex = originalVertices[i];
                Dictionary<int, (float weight, Vector3 localOffset)> weights = new Dictionary<int, (float, Vector3)>();
                float totalWeight = 0f;

                for (int j = 0; j < nodes.Count; j++)
                {
                    if (nodes[j] == null || j >= initialPositions.Count) continue;
                    
                    // Use initial local positions directly
                    Vector3 localNodePos = initialPositions[j];
                    float distance = Vector3.Distance(localVertex, localNodePos);
                    
                    if (distance <= influenceRadius)
                    {
                        float weight = 1f - (distance / influenceRadius); // Linear falloff
                        if (weight > 0f)
                        {
                            // Store the local offset from node to vertex (both already in local space)
                            Vector3 localOffset = localVertex - localNodePos;
                            weights[j] = (weight, localOffset);
                            totalWeight += weight;
                        }
                    }
                }

                if (totalWeight > 0f)
                {
                    foreach (var pair in weights)
                    {
                        int nodeIndex = pair.Key;
                        float weight = pair.Value.weight / totalWeight; // Normalize weights
                        Vector3 localOffset = pair.Value.localOffset;
                        
                        if (!nodeVertexWeights.ContainsKey(nodeIndex))
                            nodeVertexWeights[nodeIndex] = new List<(int, float, Vector3)>();
                        nodeVertexWeights[nodeIndex].Add((i, weight, localOffset));
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

            // Start with original vertices
            System.Array.Copy(originalVertices, deformedVertices, originalVertices.Length);

            // Create a displacement map for each vertex
            Vector3[] vertexDisplacements = new Vector3[originalVertices.Length];

            // Apply deformation based on node movements in local space
            for (int i = 0; i < nodes.Count; i++)
            {
                if (!nodeVertexWeights.ContainsKey(i) || nodes[i] == null || i >= initialPositions.Count) continue;

                // Calculate node displacement in local space only
                Vector3 initialLocalPos = initialPositions[i];
                Vector3 currentLocalPos = nodes[i].localPosition; // Use localPosition directly instead of converting from world
                Vector3 nodeDisplacement = currentLocalPos - initialLocalPos;
                
                // Apply transformation to each affected vertex
                foreach (var (vertexIndex, weight, localOffset) in nodeVertexWeights[i])
                {
                    if (vertexIndex >= originalVertices.Length || vertexIndex >= deformedVertices.Length) continue;

                    // Apply the node displacement to the vertex with proper weighting
                    Vector3 weightedDisplacement = nodeDisplacement * weight;
                    vertexDisplacements[vertexIndex] += weightedDisplacement;
                }
            }

            // Apply accumulated displacements to vertices with rotation compensation
            Matrix4x4 rotationMatrix = Matrix4x4.Rotate(transform.localRotation);
            for (int i = 0; i < deformedVertices.Length; i++)
            {
                // Apply soft-body deformation
                Vector3 deformedVertex = originalVertices[i] + vertexDisplacements[i];
                
                // Apply transform rotation to maintain mesh alignment with truss structure
                // This ensures mesh follows Z-axis (and other) rotations of the truss
                deformedVertices[i] = rotationMatrix.MultiplyPoint3x4(deformedVertex);
            }

            // Apply the deformed vertices to the mesh
            mesh.vertices = deformedVertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            mesh.RecalculateTangents();
            
            // Force Unity to update the mesh renderer
            mesh.UploadMeshData(false);
        }
    }
}