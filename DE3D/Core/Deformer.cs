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
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;
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

        private NativeArray<float3> nativeOriginalVertices;
        private NativeArray<float3> nativeDeformedVertices;
        private NativeArray<InfluenceData> nativeInfluenceData;
        private NativeArray<int> nativeInfluenceOffsets;
        private bool nativeArraysAllocated;

        [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential)]
        public struct InfluenceData
        {
            public int nodeIndex;
            public float weight;
            public float3 localOffset;
        }

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
            nativeArraysAllocated = false;

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

        [BurstCompile]
        private struct MeshDeformJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> originalVertices;
            [ReadOnly] public NativeArray<float3> nodeCurrentLocalPositions;
            [ReadOnly] public NativeArray<quaternion> nodeCurrentLocalRotations;
            [ReadOnly] public NativeArray<quaternion> nodeInitialRotations;
            [ReadOnly] public NativeArray<InfluenceData> influenceData;
            [ReadOnly] public NativeArray<int> influenceOffsets;
            [ReadOnly] public quaternion inverseTransformRotation;
            [ReadOnly] public float4x4 worldToLocal;

            [WriteOnly] public NativeArray<float3> deformedVertices;

            public void Execute(int vertexIndex)
            {
                int offset = influenceOffsets[vertexIndex];
                int nextOffset = influenceOffsets[vertexIndex + 1];
                int influenceCount = nextOffset - offset;

                if (influenceCount == 0)
                {
                    deformedVertices[vertexIndex] = originalVertices[vertexIndex];
                    return;
                }

                float3 deformedPosition = float3.zero;
                float totalWeight = 0f;

                for (int i = offset; i < nextOffset; i++)
                {
                    InfluenceData influence = influenceData[i];
                    int nodeIndex = influence.nodeIndex;

                    if (nodeIndex < 0 || nodeIndex >= nodeCurrentLocalPositions.Length)
                        continue;

                    float3 currentLocalPos = nodeCurrentLocalPositions[nodeIndex];
                    quaternion currentLocalRot = nodeCurrentLocalRotations[nodeIndex];
                    quaternion initialRot = nodeInitialRotations[nodeIndex];

                    quaternion rotationDelta = math.mul(currentLocalRot, math.inverse(initialRot));
                    float3 rotatedOffset = math.rotate(rotationDelta, influence.localOffset);
                    float3 targetPosition = currentLocalPos + rotatedOffset;

                    deformedPosition += targetPosition * influence.weight;
                    totalWeight += influence.weight;
                }

                if (totalWeight > 0.001f)
                {
                    deformedVertices[vertexIndex] = deformedPosition;
                }
                else
                {
                    deformedVertices[vertexIndex] = originalVertices[vertexIndex];
                }
            }
        }

        private void AllocateNativeArraysForDeformation()
        {
            if (nativeArraysAllocated)
                return;

            int vertexCount = originalVertices.Length;

            nativeOriginalVertices = new NativeArray<float3>(vertexCount, Allocator.Persistent);
            nativeDeformedVertices = new NativeArray<float3>(vertexCount, Allocator.Persistent);

            for (int i = 0; i < vertexCount; i++)
            {
                nativeOriginalVertices[i] = originalVertices[i];
            }

            int totalInfluences = 0;
            for (int i = 0; i < vertexInfluences.Length; i++)
            {
                totalInfluences += vertexInfluences[i].Count;
            }

            nativeInfluenceData = new NativeArray<InfluenceData>(totalInfluences, Allocator.Persistent);
            nativeInfluenceOffsets = new NativeArray<int>(vertexCount + 1, Allocator.Persistent);

            int currentOffset = 0;
            for (int i = 0; i < vertexCount; i++)
            {
                nativeInfluenceOffsets[i] = currentOffset;
                
                for (int j = 0; j < vertexInfluences[i].Count; j++)
                {
                    var influence = vertexInfluences[i][j];
                    nativeInfluenceData[currentOffset] = new InfluenceData
                    {
                        nodeIndex = influence.nodeIndex,
                        weight = influence.weight,
                        localOffset = influence.localOffset
                    };
                    currentOffset++;
                }
            }
            nativeInfluenceOffsets[vertexCount] = currentOffset;

            nativeArraysAllocated = true;
        }

        private void DisposeNativeArrays()
        {
            if (nativeArraysAllocated)
            {
                if (nativeOriginalVertices.IsCreated) nativeOriginalVertices.Dispose();
                if (nativeDeformedVertices.IsCreated) nativeDeformedVertices.Dispose();
                if (nativeInfluenceData.IsCreated) nativeInfluenceData.Dispose();
                if (nativeInfluenceOffsets.IsCreated) nativeInfluenceOffsets.Dispose();
                nativeArraysAllocated = false;
            }
        }

        public void Deform(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions, IReadOnlyList<Quaternion> initialRotations)
        {
            if (transform == null || nodes == null || initialPositions == null || initialRotations == null)
            {
                Debug.LogWarning("Cannot deform mesh: Invalid input data.");
                return;
            }

            if (!nativeArraysAllocated)
            {
                AllocateNativeArraysForDeformation();
            }

            int nodeCount = nodes.Count;
            var nodeCurrentLocalPos = new NativeArray<float3>(nodeCount, Allocator.TempJob);
            var nodeCurrentLocalRot = new NativeArray<quaternion>(nodeCount, Allocator.TempJob);
            var nodeInitialRot = new NativeArray<quaternion>(nodeCount, Allocator.TempJob);

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodes[i] != null && i < initialPositions.Count && i < initialRotations.Count)
                {
                    Vector3 currentWorldPos = nodes[i].position;
                    Vector3 currentLocalPos = transform.InverseTransformPoint(currentWorldPos);
                    nodeCurrentLocalPos[i] = currentLocalPos;

                    Quaternion currentWorldRot = nodes[i].rotation;
                    Quaternion currentLocalRot = Quaternion.Inverse(transform.rotation) * currentWorldRot;
                    nodeCurrentLocalRot[i] = currentLocalRot;

                    nodeInitialRot[i] = initialRotations[i];
                }
            }

            Quaternion unityRot = transform.rotation;
            quaternion inverseTransformRot = math.inverse(new quaternion(unityRot.x, unityRot.y, unityRot.z, unityRot.w));
            float4x4 worldToLocal = transform.worldToLocalMatrix;

            var deformJob = new MeshDeformJob
            {
                originalVertices = nativeOriginalVertices,
                nodeCurrentLocalPositions = nodeCurrentLocalPos,
                nodeCurrentLocalRotations = nodeCurrentLocalRot,
                nodeInitialRotations = nodeInitialRot,
                influenceData = nativeInfluenceData,
                influenceOffsets = nativeInfluenceOffsets,
                inverseTransformRotation = inverseTransformRot,
                worldToLocal = worldToLocal,
                deformedVertices = nativeDeformedVertices
            };

            JobHandle handle = deformJob.Schedule(originalVertices.Length, 64);
            handle.Complete();

            for (int i = 0; i < originalVertices.Length; i++)
            {
                deformedVertices[i] = nativeDeformedVertices[i];
            }

            nodeCurrentLocalPos.Dispose();
            nodeCurrentLocalRot.Dispose();
            nodeInitialRot.Dispose();

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

        ~MeshDeformer()
        {
            DisposeNativeArrays();
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