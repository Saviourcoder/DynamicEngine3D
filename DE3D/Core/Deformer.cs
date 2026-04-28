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

namespace DynamicEngine
{
    public class MeshDeformer
    {
        public readonly Mesh mesh;
        private readonly float3[] originalVertices;
        private readonly List<VertexInfluence>[] vertexInfluences;
        private readonly float influenceRadius;
        private readonly int maxInfluences = 4;
        private bool useAdvancedSkinning = true;
        private int _framesSinceNormalUpdate = 0;
        private const int NORMAL_UPDATE_INTERVAL = 1;

        private ComputeShader deformComputeShader;
        private bool useGPU = true;
        private int computeKernel;
        private bool gpuBuffersAllocated = false;

        private ComputeBuffer originalVerticesBuffer;
        private GraphicsBuffer deformedVerticesBuffer;   // writes directly into mesh vertex buffer - no CPU readback
        private ComputeBuffer influenceDataBuffer;
        private ComputeBuffer influenceOffsetsBuffer;

        private ComputeBuffer nodeCurrentPosBuffer;
        private ComputeBuffer nodeCurrentRotBuffer;
        private ComputeBuffer nodeInitialRotBuffer;
        private static int mainThreadId = System.Threading.Thread.CurrentThread.ManagedThreadId;

        private List<float3> cageNodes;
        private Dictionary<int, List<int>> nodeToVerticesMap;

        private NativeArray<float3> nativeOriginalVertices;
        private NativeArray<float3> nativeDeformedVertices;
        private NativeArray<InfluenceData> nativeInfluenceData;
        private NativeArray<int> nativeInfluenceOffsets;
        private bool nativeArraysAllocated;
        private NativeArray<float3> _nodeCurrentLocalPosPersistent;
        private NativeArray<quaternion> _nodeCurrentLocalRotPersistent;
        private NativeArray<quaternion> _nodeInitialRotArrPersistent;
        // _gpuReadbackBuffer removed - compute shader writes directly to mesh GPU vertex buffer

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
            public float3 localOffset;

            public VertexInfluence(int nodeIndex, float weight, float3 localOffset)
            {
                this.nodeIndex = nodeIndex;
                this.weight = weight;
                this.localOffset = localOffset;
            }
        }

        public MeshDeformer(Mesh mesh, float3[] originalVertices, float influenceRadius)
        {
            this.mesh = mesh ?? new Mesh();
            this.originalVertices = originalVertices ?? new float3[0];
            this.influenceRadius = Mathf.Max(0.01f, influenceRadius);

            vertexInfluences = new List<VertexInfluence>[this.originalVertices.Length];
            for (int i = 0; i < vertexInfluences.Length; i++)
            {
                vertexInfluences[i] = new List<VertexInfluence>();
            }

            cageNodes = new List<float3>();
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

        public void MapVerticesToNodes(Transform transform, IReadOnlyList<Vector3> nodes, IReadOnlyList<Vector3> initialPositions)
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

        private void BuildNodeBVH(IReadOnlyList<Vector3> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            cageNodes.Clear();
            for (int i = 0; i < nodes.Count; i++)
            {
                if (i < initialPositions.Count)
                {
                    cageNodes.Add((float3)(Vector3)initialPositions[i]);
                }
            }
        }

        private void UseAdvancedVolumeSkinning(IReadOnlyList<Vector3> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            for (int vertexIndex = 0; vertexIndex < originalVertices.Length; vertexIndex++)
            {
                float3 vertex = originalVertices[vertexIndex];

                var closestNodes = FindClosestNodesWithWeights(vertex, nodes, initialPositions, 4, influenceRadius);

                foreach (var (nodeIndex, weight, distance) in closestNodes)
                {
                    if (weight > 0.001f)
                    {
                        float3 localOffset = vertex - (float3)(Vector3)initialPositions[nodeIndex];
                        vertexInfluences[vertexIndex].Add(new VertexInfluence(nodeIndex, weight, localOffset));
                    }
                }

                if (vertexInfluences[vertexIndex].Count == 0 && nodes.Count > 0)
                {
                    int closestNode = FindClosestNode(vertex, initialPositions);
                    float3 localOffset = vertex - (float3)(Vector3)initialPositions[closestNode];
                    vertexInfluences[vertexIndex].Add(new VertexInfluence(closestNode, 1.0f, localOffset));
                }
            }
        }

        private void UseDistanceBasedSkinning(IReadOnlyList<Vector3> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            float radiusSquared = influenceRadius * influenceRadius;

            for (int vertexIndex = 0; vertexIndex < originalVertices.Length; vertexIndex++)
            {
                float3 vertex = originalVertices[vertexIndex];
                var influences = new List<(int nodeIndex, float weight)>();

                for (int nodeIndex = 0; nodeIndex < nodes.Count; nodeIndex++)
                {
                    if (nodeIndex >= initialPositions.Count)
                        continue;

                    float3 nodePos = (float3)(Vector3)initialPositions[nodeIndex];
                    float distanceSquared = math.lengthsq(vertex - nodePos);

                    if (distanceSquared <= radiusSquared)
                    {
                        float distance = math.sqrt(distanceSquared);
                        float t = distance / influenceRadius;
                        float weight = (1f - t * t) * (1f - t * t);
                        influences.Add((nodeIndex, weight));
                    }
                }

                influences.Sort((a, b) => b.weight.CompareTo(a.weight));

                float totalWeight = 0f;
                int count = Mathf.Min(influences.Count, maxInfluences);

                for (int i = 0; i < count; i++)
                    totalWeight += influences[i].weight;

                for (int i = 0; i < count; i++)
                {
                    if (totalWeight > 0)
                    {
                        float normalizedWeight = influences[i].weight / totalWeight;
                        float3 localOffset = vertex - (float3)(Vector3)initialPositions[influences[i].nodeIndex];
                        vertexInfluences[vertexIndex].Add(new VertexInfluence(influences[i].nodeIndex, normalizedWeight, localOffset));
                    }
                }

                if (vertexInfluences[vertexIndex].Count == 0 && nodes.Count > 0)
                {
                    int closestNode = FindClosestNode(vertex, initialPositions);
                    float3 localOffset = vertex - (float3)(Vector3)initialPositions[closestNode];
                    vertexInfluences[vertexIndex].Add(new VertexInfluence(closestNode, 1.0f, localOffset));
                }
            }
        }

        private List<(int nodeIndex, float weight, float distance)> FindClosestNodesWithWeights(
            float3 vertex, IReadOnlyList<Vector3> nodes, IReadOnlyList<Vector3> initialPositions,
            int maxNodes, float radius)
        {
            var nodeDistances = new List<(int index, float distance)>();

            for (int i = 0; i < nodes.Count; i++)
            {
                if (i >= initialPositions.Count) continue;

                float distance = math.distance(vertex, (float3)(Vector3)initialPositions[i]);
                if (distance > radius) continue;

                nodeDistances.Add((i, distance));
            }

            nodeDistances.Sort((a, b) => a.distance.CompareTo(b.distance));

            var result = new List<(int, float, float)>();
            int takeCount = Mathf.Min(maxNodes, nodeDistances.Count);
            if (takeCount == 0) return result;

            float totalWeight = 0f;
            for (int i = 0; i < takeCount; i++)
            {
                float t = nodeDistances[i].distance / radius;
                float weight = (1f - t * t) * (1f - t * t);
                totalWeight += weight;
                result.Add((nodeDistances[i].index, weight, nodeDistances[i].distance));
            }

            for (int i = 0; i < result.Count; i++)
            {
                var (index, weight, distance) = result[i];
                result[i] = (index, totalWeight > 0f ? weight / totalWeight : 0f, distance);
            }

            return result;
        }

        private int FindClosestNode(float3 position, IReadOnlyList<Vector3> initialPositions)
        {
            if (initialPositions == null || initialPositions.Count == 0) return 0;

            int best = 0;
            float minDistSq = float.MaxValue;

            for (int i = 0; i < initialPositions.Count; i++)
            {
                float distSq = math.lengthsq(position - (float3)(Vector3)initialPositions[i]);
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
            originalVerticesBuffer.SetData(originalVertices);

            // Tell the mesh we own the vertex layout (position only, 12 bytes/vertex)
            mesh.SetVertexBufferParams(vertexCount,
                new UnityEngine.Rendering.VertexAttributeDescriptor(
                    UnityEngine.Rendering.VertexAttribute.Position,
                    UnityEngine.Rendering.VertexAttributeFormat.Float32, 3));

            // Mark the vertex buffer as writable from the GPU so GetVertexBuffer works
            mesh.vertexBufferTarget = GraphicsBuffer.Target.Structured;

            // Grab the mesh's own GPU-side vertex buffer - compute will write here directly
            deformedVerticesBuffer = mesh.GetVertexBuffer(0);

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

            nodeCurrentPosBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
            nodeCurrentRotBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 4);
            nodeInitialRotBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 4);

            gpuBuffersAllocated = true;
        }

        private void DeformGPU(Transform transform, IReadOnlyList<Vector3> currentPositions,
                       IReadOnlyList<Quaternion> currentRotations,
                       IReadOnlyList<Vector3> initialPositions, IReadOnlyList<Quaternion> initialRotations)
        {
            if (!gpuBuffersAllocated) AllocateGPUBuffers(currentPositions.Count);

            if (nodeCurrentPosBuffer == null || nodeCurrentPosBuffer.count != currentPositions.Count)
            {
                DisposeGPUBuffers();
                AllocateGPUBuffers(currentPositions.Count);
            }

            int nodeCount = currentPositions.Count;
            var nodeCurrentPos = new float3[nodeCount];
            var nodeCurrentRot = new Vector4[nodeCount];
            var nodeInitialRot = new Vector4[nodeCount];

            for (int i = 0; i < nodeCount; i++)
            {
                if (i < initialPositions.Count && i < initialRotations.Count)
                {
                    nodeCurrentPos[i] = (float3)(Vector3)transform.InverseTransformPoint(currentPositions[i]);

                    // Removing the erroneous Inverse(transform.rotation) that caused spiky meshes on init / rotated objects
                    Quaternion currentLocalRot = i < currentRotations.Count ? currentRotations[i] : Quaternion.identity;
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

            // Data lives entirely on the GPU - no GetData/SetVertices roundtrip.
            // The mesh vertex buffer IS deformedVerticesBuffer, so the dispatch above
            // already updated the mesh. Just keep bounds fresh for culling.
            // NOTE: RecalculateNormals reads from CPU-side vertex data which is stale here.
            //       Move normal computation into the compute shader for a fully GPU path.
            if (++_framesSinceNormalUpdate >= NORMAL_UPDATE_INTERVAL)
            {
                // TODO: replace with a GPU normals compute pass
                _framesSinceNormalUpdate = 0;
            }

            mesh.RecalculateBounds();
        }

        private void DisposeGPUBuffers()
        {
            if (System.Threading.Thread.CurrentThread.ManagedThreadId != mainThreadId) return;

            originalVerticesBuffer?.Release();
            deformedVerticesBuffer?.Release();   // GraphicsBuffer - same Release() API
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
            // _gpuReadbackBuffer removed - no longer needed

            gpuBuffersAllocated = false;
        }

        #endregion

        #region CPU Jobs Path (Fallback)

        [BurstCompile]
        private struct PrepareNodeDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> worldPositions;
            [ReadOnly] public float4x4 worldToLocal;
            [WriteOnly] public NativeArray<float3> localPositions;

            // Rotations are already local - just copy
            [ReadOnly] public NativeArray<quaternion> currentRots;
            [ReadOnly] public NativeArray<quaternion> initialRots;
            [WriteOnly] public NativeArray<quaternion> outCurrentRots;
            [WriteOnly] public NativeArray<quaternion> outInitialRots;

            public void Execute(int i)
            {
                localPositions[i] = math.transform(worldToLocal, worldPositions[i]);
                outCurrentRots[i] = currentRots[i];
                outInitialRots[i] = initialRots[i];
            }
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

        private void AllocateNativeArraysForDeformation(int nodeCount)
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

            _nodeCurrentLocalPosPersistent = new NativeArray<float3>(nodeCount, Allocator.Persistent);
            _nodeCurrentLocalRotPersistent = new NativeArray<quaternion>(nodeCount, Allocator.Persistent);
            _nodeInitialRotArrPersistent = new NativeArray<quaternion>(nodeCount, Allocator.Persistent);

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

        private void DeformCPU(Transform transform, IReadOnlyList<Vector3> currentPositions,
               IReadOnlyList<Quaternion> currentRotations,
               IReadOnlyList<Vector3> initialPositions, IReadOnlyList<Quaternion> initialRotations)
        {
            int nodeCount = currentPositions.Count;
            if (!nativeArraysAllocated) AllocateNativeArraysForDeformation(nodeCount);

            // Stage 0: pack source data into native arrays (one contiguous copy, no Transform calls)
            var worldPosTemp = new NativeArray<float3>(nodeCount, Allocator.TempJob);
            var currentRotTemp = new NativeArray<quaternion>(nodeCount, Allocator.TempJob);
            var initialRotTemp = new NativeArray<quaternion>(nodeCount, Allocator.TempJob);

            for (int i = 0; i < nodeCount; i++)
            {
                worldPosTemp[i] = (float3)(Vector3)currentPositions[i];
                currentRotTemp[i] = i < currentRotations.Count
                    ? (quaternion)currentRotations[i] : quaternion.identity;
                initialRotTemp[i] = (quaternion)initialRotations[i];
            }

            float4x4 w2l = transform.worldToLocalMatrix;

            // Stage 1: world -> local + rot copy in parallel (no managed InverseTransformPoint loop)
            var h0 = new PrepareNodeDataJob
            {
                worldPositions = worldPosTemp,
                worldToLocal = w2l,
                localPositions = _nodeCurrentLocalPosPersistent,
                currentRots = currentRotTemp,
                initialRots = initialRotTemp,
                outCurrentRots = _nodeCurrentLocalRotPersistent,
                outInitialRots = _nodeInitialRotArrPersistent,
            }.Schedule(nodeCount, 32);

            // Stage 2: deform vertices (depends on stage 1)
            var deformJob = new MeshDeformJob
            {
                originalVertices = nativeOriginalVertices,
                nodeCurrentLocalPositions = _nodeCurrentLocalPosPersistent,
                nodeCurrentLocalRotations = _nodeCurrentLocalRotPersistent,
                nodeInitialRotations = _nodeInitialRotArrPersistent,
                influenceData = nativeInfluenceData,
                influenceOffsets = nativeInfluenceOffsets,
                deformedVertices = nativeDeformedVertices,
            };
            deformJob.Schedule(originalVertices.Length, 64, h0).Complete();

            worldPosTemp.Dispose();
            currentRotTemp.Dispose();
            initialRotTemp.Dispose();

            // Zero-copy reinterpret: float3 and Vector3 share identical 12-byte memory layout
            mesh.SetVertices(nativeDeformedVertices.Reinterpret<Vector3>(12));
            _framesSinceNormalUpdate++;
            if (_framesSinceNormalUpdate >= NORMAL_UPDATE_INTERVAL)
            {
                mesh.RecalculateNormals();
                _framesSinceNormalUpdate = 0;
            }
            mesh.RecalculateBounds();
            mesh.UploadMeshData(false);
        }

        private void DisposeNativeArrays()
        {
            if (nativeOriginalVertices.IsCreated) nativeOriginalVertices.Dispose();
            if (nativeDeformedVertices.IsCreated) nativeDeformedVertices.Dispose();
            if (nativeInfluenceData.IsCreated) nativeInfluenceData.Dispose();
            if (nativeInfluenceOffsets.IsCreated) nativeInfluenceOffsets.Dispose();
            if (_nodeCurrentLocalPosPersistent.IsCreated) _nodeCurrentLocalPosPersistent.Dispose();
            if (_nodeCurrentLocalRotPersistent.IsCreated) _nodeCurrentLocalRotPersistent.Dispose();
            if (_nodeInitialRotArrPersistent.IsCreated) _nodeInitialRotArrPersistent.Dispose();
            nativeArraysAllocated = false;
        }

        #endregion

        public void Deform(Transform transform, IReadOnlyList<Vector3> currentPositions,
                          IReadOnlyList<Quaternion> currentRotations,
                          IReadOnlyList<Vector3> initialPositions, IReadOnlyList<Quaternion> initialRotations)
        {
            if (transform == null || currentPositions == null || initialPositions == null || initialRotations == null) return;
            if (UseGPU) DeformGPU(transform, currentPositions, currentRotations, initialPositions, initialRotations);
            else DeformCPU(transform, currentPositions, currentRotations, initialPositions, initialRotations);
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