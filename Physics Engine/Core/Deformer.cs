/* DynamicEngine3D - Dual Quaternion Mesh Deformation
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
    // Dual Quaternion structure for skinning
    public class MeshDeformer
    {
        public readonly Mesh mesh;
        private readonly Vector3[] originalVertices;
        private readonly Dictionary<int, List<(int vertexIndex, float weight)>> nodeVertexWeights;
        private readonly float influenceRadius;
        private readonly float influenceRadiusSquared; // Cache squared radius for faster distance checks
        private Vector3[] deformedVertices;
        private readonly int maxNodesPerVertex = 4; // Limit influence to top 4 nodes per vertex

        public MeshDeformer(Mesh mesh, Vector3[] originalVertices, float influenceRadius)
        {
            this.mesh = mesh ?? new Mesh();
            this.originalVertices = originalVertices ?? new Vector3[0];
            this.influenceRadius = Mathf.Max(0.01f, influenceRadius);
            this.influenceRadiusSquared = this.influenceRadius * this.influenceRadius;
            deformedVertices = new Vector3[this.originalVertices.Length];
            System.Array.Copy(this.originalVertices, deformedVertices, this.originalVertices.Length);
            nodeVertexWeights = new Dictionary<int, List<(int, float)>>();

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

            // Pre-allocate arrays to reduce garbage collection
            var tempWeights = new List<(int nodeIndex, float weight)>(maxNodesPerVertex);

            for (int i = 0; i < originalVertices.Length; i++)
            {
                Vector3 localVertex = originalVertices[i];
                tempWeights.Clear();
                float totalWeight = 0f;

                // Fast spatial culling with squared distance
                for (int j = 0; j < nodes.Count; j++)
                {
                    if (nodes[j] == null || j >= initialPositions.Count) continue;

                    Vector3 localNodePos = initialPositions[j];
                    
                    // Use squared distance to avoid expensive square root
                    Vector3 diff = localVertex - localNodePos;
                    float distanceSquared = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;

                    if (distanceSquared <= influenceRadiusSquared)
                    {
                        float distance = Mathf.Sqrt(distanceSquared);
                        float weight = 1f - (distance / influenceRadius);
                        
                        if (weight > 0f)
                        {
                            tempWeights.Add((j, weight));
                            totalWeight += weight;
                        }
                    }
                }

                // Sort by weight and keep only top influences
                if (tempWeights.Count > maxNodesPerVertex)
                {
                    tempWeights.Sort((a, b) => b.weight.CompareTo(a.weight));
                    
                    // Recalculate total weight for top influences only
                    totalWeight = 0f;
                    for (int k = 0; k < maxNodesPerVertex; k++)
                    {
                        totalWeight += tempWeights[k].weight;
                    }
                    
                    // Remove excess influences
                    tempWeights.RemoveRange(maxNodesPerVertex, tempWeights.Count - maxNodesPerVertex);
                }

                if (totalWeight > 0f)
                {
                    // Normalize weights and assign to node mappings
                    foreach (var (nodeIndex, weight) in tempWeights)
                    {
                        float normalizedWeight = weight / totalWeight;

                        if (!nodeVertexWeights.ContainsKey(nodeIndex))
                            nodeVertexWeights[nodeIndex] = new List<(int, float)>();
                        nodeVertexWeights[nodeIndex].Add((i, normalizedWeight));
                    }
                }
                else
                {
                    Debug.LogWarning($"Vertex {i} has no influencing nodes within radius {influenceRadius}");
                }
            }
        }

        public void Deform(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions, IReadOnlyList<Quaternion> initialRotations)
        {
            if (transform == null || nodes == null || initialPositions == null || initialRotations == null)
            {
                Debug.LogWarning("Cannot deform mesh: Invalid input data.");
                return;
            }

            // Create vertex-bone mapping for dual quaternion skinning
            DualQuaternion[] vertexDualQuats = new DualQuaternion[originalVertices.Length];
            bool[] hasInfluence = new bool[originalVertices.Length];

            // Initialize all dual quaternions to identity
            for (int i = 0; i < vertexDualQuats.Length; i++)
            {
                vertexDualQuats[i] = new DualQuaternion(Quaternion.identity, Vector3.zero);
                hasInfluence[i] = false;
            }

            // Calculate dual quaternions for each node and blend them
            for (int nodeIndex = 0; nodeIndex < nodes.Count; nodeIndex++)
            {
                if (!nodeVertexWeights.ContainsKey(nodeIndex) || nodes[nodeIndex] == null ||
                    nodeIndex >= initialPositions.Count || nodeIndex >= initialRotations.Count) continue;

                // Get initial and current positions in local space
                Vector3 initialPos = initialPositions[nodeIndex];
                Quaternion initialRot = initialRotations[nodeIndex];

                // Convert current world position to local space
                Vector3 currentPosWorld = nodes[nodeIndex].position;
                Vector3 currentPosLocal = transform.InverseTransformPoint(currentPosWorld);
                Quaternion currentRotLocal = Quaternion.Inverse(transform.rotation) * nodes[nodeIndex].rotation;

                // Calculate transformation from initial to current state
                Vector3 translation = currentPosLocal - initialPos;
                Quaternion relativeRotation = currentRotLocal * Quaternion.Inverse(initialRot);

                DualQuaternion nodeDQ = new DualQuaternion(relativeRotation, translation);

                // Apply to affected vertices
                foreach (var (vertexIndex, weight) in nodeVertexWeights[nodeIndex])
                {
                    if (vertexIndex >= originalVertices.Length) continue;

                    // Blend dual quaternions
                    vertexDualQuats[vertexIndex] = vertexDualQuats[vertexIndex] + nodeDQ * weight;
                    hasInfluence[vertexIndex] = true;
                }
            }

            // Apply dual quaternion transformations to vertices
            for (int i = 0; i < deformedVertices.Length; i++)
            {
                if (hasInfluence[i])
                {
                    // Normalize the blended dual quaternion
                    vertexDualQuats[i].Normalize();

                    // Transform the original vertex position
                    deformedVertices[i] = vertexDualQuats[i].TransformPoint(originalVertices[i]);
                }
                else
                {
                    // If vertex has no influence, keep it at original position
                    deformedVertices[i] = originalVertices[i];
                }
            }

            // Apply to mesh
            mesh.vertices = deformedVertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            mesh.RecalculateTangents();
            mesh.UploadMeshData(false);
        }
    }
    public struct DualQuaternion
    {
        public Quaternion real;
        public Quaternion dual;
        private float translationScale; // Store the scale of the translation

        public DualQuaternion(Quaternion rotation, Vector3 translation)
        {
            real = rotation;
            // Dual part: 0.5 * translation * rotation
            dual = new Quaternion(
                0.5f * (translation.x * rotation.w + translation.y * rotation.z - translation.z * rotation.y),
                0.5f * (-translation.x * rotation.z + translation.y * rotation.w + translation.z * rotation.x),
                0.5f * (translation.x * rotation.y - translation.y * rotation.x + translation.z * rotation.w),
                -0.5f * (translation.x * rotation.x + translation.y * rotation.y + translation.z * rotation.z)
            );
            translationScale = translation.magnitude; // Store the original translation magnitude
        }

        public static DualQuaternion operator +(DualQuaternion a, DualQuaternion b)
        {
            return new DualQuaternion
            {
                real = new Quaternion(a.real.x + b.real.x, a.real.y + b.real.y, a.real.z + b.real.z, a.real.w + b.real.w),
                dual = new Quaternion(a.dual.x + b.dual.x, a.dual.y + b.dual.y, a.dual.z + b.dual.z, a.dual.w + b.dual.w),
                translationScale = a.translationScale + b.translationScale // Blend translation scales
            };
        }

        public static DualQuaternion operator *(DualQuaternion dq, float scalar)
        {
            return new DualQuaternion
            {
                real = new Quaternion(dq.real.x * scalar, dq.real.y * scalar, dq.real.z * scalar, dq.real.w * scalar),
                dual = new Quaternion(dq.dual.x * scalar, dq.dual.y * scalar, dq.dual.z * scalar, dq.dual.w * scalar),
                translationScale = dq.translationScale * scalar
            };
        }

        public void Normalize()
        {
            float magnitude = Mathf.Sqrt(real.x * real.x + real.y * real.y + real.z * real.z + real.w * real.w);
            if (magnitude > 1e-6f)
            {
                float invMag = 1f / magnitude;
                real.x *= invMag;
                real.y *= invMag;
                real.z *= invMag;
                real.w *= invMag;
                dual.x *= invMag;
                dual.y *= invMag;
                dual.z *= invMag;
                dual.w *= invMag;
            }
            else
            {
                real = Quaternion.identity;
                dual = new Quaternion(0, 0, 0, 0);
                translationScale = 0f;
            }
        }

        public Vector3 TransformPoint(Vector3 point)
        {
            // Extract translation from dual quaternion
            Vector3 translation = new Vector3(
                2f * (-dual.w * real.x + dual.x * real.w - dual.y * real.z + dual.z * real.y),
                2f * (-dual.w * real.y + dual.x * real.z + dual.y * real.w - dual.z * real.x),
                2f * (-dual.w * real.z - dual.x * real.y + dual.y * real.x + dual.z * real.w)
            );

            // Apply rotation then translation, preserving original scale
            return real * point + translation * (translationScale > 0 ? translationScale / translation.magnitude : 1f);
        }
    }
}