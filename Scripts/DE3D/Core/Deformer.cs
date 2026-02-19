/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */

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
        private int _framesSinceNormalUpdate = 0;
        private const int NORMAL_UPDATE_INTERVAL = 3;

        private ComputeShader deformComputeShader;
        private bool useGPU = true;
        private int computeKernel;
        private bool gpuBuffersAllocated = false;

        private ComputeBuffer originalVerticesBuffer;
        private ComputeBuffer deformedVerticesBuffer;
        private ComputeBuffer influenceDataBuffer;
        private ComputeBuffer influenceOffsetsBuffer;

        private ComputeBuffer nodeCurrentPosBuffer;
        private ComputeBuffer nodeCurrentRotBuffer;
        private ComputeBuffer nodeInitialRotBuffer;
        private static int mainThreadId = System.Threading.Thread.CurrentThread.ManagedThreadId;

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

        [System.Serializable]
        public struct VertexInfluence
        {
            public int nodeIndex;
            public float weight;
            public Vector3 localOffset;

            public VertexInfluence(int nodeIndex, float weight, Vector3 localOffset)
            {
                this.nodeIndex = nodeIndex;
                this.weight = weight;
                this.localOffset = localOffset;
            }
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

            LoadComputeShader();
        }

        public Mesh Mesh => mesh;
        public float InfluenceRadius => influenceRadius;
        public bool UseGPU
        {
            get => useGPU && deformComputeShader != null && SystemInfo.supportsComputeShaders;
            set => useGPU = value;
        }

        private void LoadComputeShader()
        {
            deformComputeShader = Resources.Load<ComputeShader>("MeshDeformation");

            if (deformComputeShader == null)
            {
                Debug.LogWarning("[MeshDeformer] Compute shader 'MeshDeformation' not found in Resources. Falling back to CPU.");
                useGPU = false;
            }
            else if (!SystemInfo.supportsComputeShaders)
            {
                Debug.LogWarning("[MeshDeformer] Compute shaders not supported on this platform. Falling back to CPU.");
                useGPU = false;
            }
            else
            {
                computeKernel = deformComputeShader.FindKernel("CSMain");
            }
        }

        public void MapVerticesToNodes(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            if (transform == null || nodes == null || initialPositions == null || nodes.Count == 0)
                return;

            for (int i = 0; i < vertexInfluences.Length; i++)
            {
                vertexInfluences[i].Clear();
            }
            nodeToVerticesMap.Clear();
            cageNodes.Clear();

            // Full reset of resources when mapping changes
            DisposeGPUBuffers();
            DisposeNativeArrays();

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

                influences.Sort((a, b) => b.weight.CompareTo(a.weight));

                float totalWeight = 0f;
                int count = Mathf.Min(influences.Count, maxInfluences);

                for (int i = 0; i < count; i++)
                {
                    totalWeight += influences[i].weight;
                }

                for (int i = 0; i < count; i++)
                {
                    if (totalWeight > 0)
                    {
                        float normalizedWeight = influences[i].weight / totalWeight;
                        Vector3 localOffset = vertex - initialPositions[influences[i].nodeIndex];
                        vertexInfluences[vertexIndex].Add(new VertexInfluence(influences[i].nodeIndex, normalizedWeight, localOffset));
                    }
                }

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

            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] != null && i < initialPositions.Count)
                {
                    float distance = Vector3.Distance(vertex, initialPositions[i]);
                    nodeDistances.Add((i, distance));
                }
            }

            nodeDistances.Sort((a, b) => a.distance.CompareTo(b.distance));

            var result = new List<(int, float, float)>();
            int takeCount = Mathf.Min(maxNodes, nodeDistances.Count);

            if (takeCount == 0) return result;

            float totalInverseWeight = 0f;
            for (int i = 0; i < takeCount; i++)
            {
                float invWeight = 1.0f / (nodeDistances[i].distance + 0.001f);
                totalInverseWeight += invWeight;
                result.Add((nodeDistances[i].index, invWeight, nodeDistances[i].distance));
            }

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

        #region GPU Compute Shader Path

        private void AllocateGPUBuffers(int nodeCount)
        {
            if (gpuBuffersAllocated || deformComputeShader == null)
                return;

            int vertexCount = originalVertices.Length;

            originalVerticesBuffer = new ComputeBuffer(vertexCount, sizeof(float) * 3);
            deformedVerticesBuffer = new ComputeBuffer(vertexCount, sizeof(float) * 3);
            originalVerticesBuffer.SetData(originalVertices);

            int totalInfluences = 0;
            for (int i = 0; i < vertexInfluences.Length; i++) totalInfluences += vertexInfluences[i].Count;

            var influenceDataArray = new InfluenceData[totalInfluences];
            var influenceOffsetsArray = new int[vertexCount + 1];

            int currentOffset = 0;
            for (int i = 0; i < vertexCount; i++)
            {
                influenceOffsetsArray[i] = currentOffset;
                foreach (var influence in vertexInfluences[i])
                {
                    influenceDataArray[currentOffset++] = new InfluenceData
                    {
                        nodeIndex = influence.nodeIndex,
                        weight = influence.weight,
                        localOffset = influence.localOffset
                    };
                }
            }
            influenceOffsetsArray[vertexCount] = currentOffset;

            influenceDataBuffer = new ComputeBuffer(totalInfluences, sizeof(int) + sizeof(float) * 4);
            influenceOffsetsBuffer = new ComputeBuffer(vertexCount + 1, sizeof(int));
            influenceDataBuffer.SetData(influenceDataArray);
            influenceOffsetsBuffer.SetData(influenceOffsetsArray);

            // Pre-allocate persistent node data buffers
            nodeCurrentPosBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
            nodeCurrentRotBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 4);
            nodeInitialRotBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 4);

            gpuBuffersAllocated = true;
        }

        private void DeformGPU(Transform transform, IReadOnlyList<Transform> nodes,
                       IReadOnlyList<Vector3> initialPositions, IReadOnlyList<Quaternion> initialRotations)
        {
            if (!gpuBuffersAllocated) AllocateGPUBuffers(nodes.Count);

            // Resizing logic if node count changed
            if (nodeCurrentPosBuffer == null || nodeCurrentPosBuffer.count != nodes.Count)
            {
                DisposeGPUBuffers();
                AllocateGPUBuffers(nodes.Count);
            }

            int nodeCount = nodes.Count;
            var nodeCurrentPos = new Vector3[nodeCount];
            var nodeCurrentRot = new Vector4[nodeCount];
            var nodeInitialRot = new Vector4[nodeCount];

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodes[i] != null && i < initialPositions.Count && i < initialRotations.Count)
                {
                    nodeCurrentPos[i] = transform.InverseTransformPoint(nodes[i].position);

                    Quaternion currentLocalRot = Quaternion.Inverse(transform.rotation) * nodes[i].rotation;
                    nodeCurrentRot[i] = new Vector4(currentLocalRot.x, currentLocalRot.y, currentLocalRot.z, currentLocalRot.w);

                    Quaternion initRot = initialRotations[i];
                    nodeInitialRot[i] = new Vector4(initRot.x, initRot.y, initRot.z, initRot.w);
                }
            }

            nodeCurrentPosBuffer.SetData(nodeCurrentPos);
            nodeCurrentRotBuffer.SetData(nodeCurrentRot);
            nodeInitialRotBuffer.SetData(nodeInitialRot);

            deformComputeShader.SetBuffer(computeKernel, "_OriginalVertices", originalVerticesBuffer);
            deformComputeShader.SetBuffer(computeKernel, "_NodeCurrentLocalPositions", nodeCurrentPosBuffer);
            deformComputeShader.SetBuffer(computeKernel, "_NodeCurrentLocalRotations", nodeCurrentRotBuffer);
            deformComputeShader.SetBuffer(computeKernel, "_NodeInitialRotations", nodeInitialRotBuffer);
            deformComputeShader.SetBuffer(computeKernel, "_InfluenceData", influenceDataBuffer);
            deformComputeShader.SetBuffer(computeKernel, "_InfluenceOffsets", influenceOffsetsBuffer);
            deformComputeShader.SetBuffer(computeKernel, "_DeformedVertices", deformedVerticesBuffer);

            int threadGroups = Mathf.CeilToInt(originalVertices.Length / 256.0f);
            deformComputeShader.Dispatch(computeKernel, threadGroups, 1, 1);

            deformedVerticesBuffer.GetData(deformedVertices);
            mesh.vertices = deformedVertices;

            if (++_framesSinceNormalUpdate >= NORMAL_UPDATE_INTERVAL)
            {
                mesh.RecalculateNormals();
                _framesSinceNormalUpdate = 0;
            }

            mesh.RecalculateBounds();
            mesh.UploadMeshData(false);
        }

        private void DisposeGPUBuffers()
        {
            if (System.Threading.Thread.CurrentThread.ManagedThreadId != mainThreadId) return;

            originalVerticesBuffer?.Release();
            deformedVerticesBuffer?.Release();
            influenceDataBuffer?.Release();
            influenceOffsetsBuffer?.Release();
            nodeCurrentPosBuffer?.Release();
            nodeCurrentRotBuffer?.Release();
            nodeInitialRotBuffer?.Release();

            originalVerticesBuffer = null;
            deformedVerticesBuffer = null;
            influenceDataBuffer = null;
            influenceOffsetsBuffer = null;
            nodeCurrentPosBuffer = null;
            nodeCurrentRotBuffer = null;
            nodeInitialRotBuffer = null;

            gpuBuffersAllocated = false;
        }

        #endregion

        #region CPU Jobs Path (Fallback)

        [BurstCompile]
        private struct MeshDeformJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> originalVertices;
            [ReadOnly] public NativeArray<float3> nodeCurrentLocalPositions;
            [ReadOnly] public NativeArray<quaternion> nodeCurrentLocalRotations;
            [ReadOnly] public NativeArray<quaternion> nodeInitialRotations;
            [ReadOnly] public NativeArray<InfluenceData> influenceData;
            [ReadOnly] public NativeArray<int> influenceOffsets;
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
                int nodeCount = nodeCurrentLocalPositions.Length;

                for (int i = offset; i < nextOffset; i++)
                {
                    InfluenceData influence = influenceData[i];
                    if (influence.nodeIndex < 0 || influence.nodeIndex >= nodeCount) continue;

                    quaternion rotDelta = math.mul(nodeCurrentLocalRotations[influence.nodeIndex], math.conjugate(nodeInitialRotations[influence.nodeIndex]));
                    float3 targetPos = nodeCurrentLocalPositions[influence.nodeIndex] + math.rotate(rotDelta, influence.localOffset);

                    deformedPosition += targetPos * influence.weight;
                    totalWeight += influence.weight;
                }

                deformedVertices[vertexIndex] = totalWeight > 0.001f ? deformedPosition : originalVertices[vertexIndex];
            }
        }

        private void AllocateNativeArraysForDeformation()
        {
            if (nativeArraysAllocated) return;
            int vertexCount = originalVertices.Length;

            nativeOriginalVertices = new NativeArray<float3>(vertexCount, Allocator.Persistent);
            nativeDeformedVertices = new NativeArray<float3>(vertexCount, Allocator.Persistent);
            for (int i = 0; i < vertexCount; i++) nativeOriginalVertices[i] = originalVertices[i];

            int totalInfluences = 0;
            for (int i = 0; i < vertexInfluences.Length; i++) totalInfluences += vertexInfluences[i].Count;

            nativeInfluenceData = new NativeArray<InfluenceData>(totalInfluences, Allocator.Persistent);
            nativeInfluenceOffsets = new NativeArray<int>(vertexCount + 1, Allocator.Persistent);

            int currentOffset = 0;
            for (int i = 0; i < vertexCount; i++)
            {
                nativeInfluenceOffsets[i] = currentOffset;
                foreach (var influence in vertexInfluences[i])
                {
                    nativeInfluenceData[currentOffset++] = new InfluenceData
                    {
                        nodeIndex = influence.nodeIndex,
                        weight = influence.weight,
                        localOffset = influence.localOffset
                    };
                }
            }
            nativeInfluenceOffsets[vertexCount] = currentOffset;
            nativeArraysAllocated = true;
        }

        private void DeformCPU(Transform transform, IReadOnlyList<Transform> nodes,
                               IReadOnlyList<Vector3> initialPositions, IReadOnlyList<Quaternion> initialRotations)
        {
            if (!nativeArraysAllocated) AllocateNativeArraysForDeformation();

            int nodeCount = nodes.Count;
            var nodeCurrentLocalPos = new NativeArray<float3>(nodeCount, Allocator.TempJob);
            var nodeCurrentLocalRot = new NativeArray<quaternion>(nodeCount, Allocator.TempJob);
            var nodeInitialRotArr = new NativeArray<quaternion>(nodeCount, Allocator.TempJob);

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodes[i] != null && i < initialPositions.Count && i < initialRotations.Count)
                {
                    nodeCurrentLocalPos[i] = transform.InverseTransformPoint(nodes[i].position);
                    nodeCurrentLocalRot[i] = Quaternion.Inverse(transform.rotation) * nodes[i].rotation;
                    nodeInitialRotArr[i] = initialRotations[i];
                }
            }

            var deformJob = new MeshDeformJob
            {
                originalVertices = nativeOriginalVertices,
                nodeCurrentLocalPositions = nodeCurrentLocalPos,
                nodeCurrentLocalRotations = nodeCurrentLocalRot,
                nodeInitialRotations = nodeInitialRotArr,
                influenceData = nativeInfluenceData,
                influenceOffsets = nativeInfluenceOffsets,
                deformedVertices = nativeDeformedVertices
            };

            JobHandle handle = deformJob.Schedule(originalVertices.Length, 64);
            handle.Complete();

            for (int i = 0; i < originalVertices.Length; i++) deformedVertices[i] = nativeDeformedVertices[i];

            nodeCurrentLocalPos.Dispose();
            nodeCurrentLocalRot.Dispose();
            nodeInitialRotArr.Dispose();

            mesh.vertices = deformedVertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            mesh.UploadMeshData(false);
        }

        private void DisposeNativeArrays()
        {
            if (nativeOriginalVertices.IsCreated) nativeOriginalVertices.Dispose();
            if (nativeDeformedVertices.IsCreated) nativeDeformedVertices.Dispose();
            if (nativeInfluenceData.IsCreated) nativeInfluenceData.Dispose();
            if (nativeInfluenceOffsets.IsCreated) nativeInfluenceOffsets.Dispose();
            nativeArraysAllocated = false;
        }

        #endregion

        public void Deform(Transform transform, IReadOnlyList<Transform> nodes,
                          IReadOnlyList<Vector3> initialPositions, IReadOnlyList<Quaternion> initialRotations)
        {
            if (transform == null || nodes == null || initialPositions == null || initialRotations == null) return;
            if (UseGPU) DeformGPU(transform, nodes, initialPositions, initialRotations);
            else DeformCPU(transform, nodes, initialPositions, initialRotations);
        }

        public void SetSkinningMethod(bool useAdvanced) => useAdvancedSkinning = useAdvanced;

        public void Cleanup()
        {
            DisposeGPUBuffers();
            DisposeNativeArrays();
        }

        ~MeshDeformer()
        {
            if (nativeArraysAllocated) try { DisposeNativeArrays(); } catch { }
        }
    }
}