/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers */
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    public class Solver
    {
        private static bool s_isApplicationQuitting = false;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void InitializeOnLoad()
        {
            s_isApplicationQuitting = false;
            Application.quitting += () => s_isApplicationQuitting = true;
        }

        private readonly Transform owner;
        public readonly NodeManager nodeManager;
        public List<Beam> beams;
        public readonly MeshDeformer meshDeformer;
        public readonly List<Vector3> collisionPoints;
        public bool visualizeForces = false;
        public List<float> nodeMasses = new List<float>();
        public float compliance = 1e-3f;
        public float defaultDamping = 0.3f;
        public float maxDeformation = 1f;
        private Truss trussAsset;

        #region Matter Variables
        private Matter matterAsset;

        #endregion

        #region Collision variables
        private readonly Collider[] overlapResults = new Collider[32];
        private LayerMask collisionLayerMask = ~0;
        #endregion

        #region Pressure Variables
        public float internalPressure = 0f;
        private Vector3 cachedVolumeCenter = Vector3.zero;
        #endregion

        #region Job Structs

        [BurstCompile]
        private struct VerletIntegrationJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> currentWorldPositions;
            [ReadOnly] public NativeArray<float3> previousPositions;
            [ReadOnly] public NativeArray<bool> isPinned;
            [ReadOnly] public float dt;
            [ReadOnly] public float dampingFactor;
            [ReadOnly] public float3 worldGravity;
            [ReadOnly] public float dtSquared;

            public NativeArray<float3> predictedPositions;

            public void Execute(int i)
            {
                if (isPinned[i]) return;

                float3 currentWorldPos = currentWorldPositions[i];
                float3 prevWorldPos = previousPositions[i];
                float3 worldVelocity = (currentWorldPos - prevWorldPos) / dt;

                worldVelocity *= dampingFactor;

                float3 newWorldPosition = currentWorldPos + worldVelocity * dt + worldGravity * dtSquared;
                predictedPositions[i] = newWorldPosition;
            }
        }

        [BurstCompile]
        private struct SolveDistanceConstraintsJob : IJob
        {
            [ReadOnly] public NativeArray<BeamData> beams;
            [ReadOnly] public NativeArray<float3> previousPositions;
            [ReadOnly] public NativeArray<bool> isPinned;
            [ReadOnly] public NativeArray<float> nodeMasses;
            [ReadOnly] public float dt;
            [ReadOnly] public float worldRestLengthScale;

            public NativeArray<float3> predictedPositions;
            public NativeArray<float> beamRestLengths;
            public NativeArray<float> beamLagrangeMultipliers;

            public void Execute()
            {
                for (int beamIndex = 0; beamIndex < beams.Length; beamIndex++)
                {
                    BeamData beam = beams[beamIndex];
                    if (!beam.isActive) continue;

                    int nodeA = beam.nodeA;
                    int nodeB = beam.nodeB;

                    float3 posA = predictedPositions[nodeA];
                    float3 posB = predictedPositions[nodeB];
                    float3 delta = posB - posA;
                    float currentLength = math.length(delta);

                    if (currentLength < 0.001f) continue;

                    float3 gradient = delta / currentLength;

                    float restLength = beamRestLengths[beamIndex];
                    float worldRestLength = restLength * worldRestLengthScale;
                    float constraint = currentLength - worldRestLength;

                    float strain = math.abs(constraint) / worldRestLength;
                    if (strain > beam.plasticityThreshold)
                    {
                        float excessStrain = strain - beam.plasticityThreshold;
                        float plasticDeformation = excessStrain * beam.plasticityRate * dt;

                        float totalDeformation = math.abs(restLength - beam.originalRestLength) / beam.originalRestLength;
                        if (totalDeformation < beam.maxDeformation)
                        {
                            float deformationAmount = plasticDeformation * restLength;
                            if (constraint > 0f)
                                restLength += deformationAmount;
                            else
                                restLength -= deformationAmount;

                            restLength = math.clamp(restLength,
                                beam.originalRestLength * (1f - beam.maxDeformation),
                                beam.originalRestLength * (1f + beam.maxDeformation));

                            beamRestLengths[beamIndex] = restLength;
                            worldRestLength = restLength * worldRestLengthScale;
                            constraint = currentLength - worldRestLength;
                        }
                    }

                    float3 prevA = previousPositions[nodeA];
                    float3 prevB = previousPositions[nodeB];
                    float3 velA = (posA - prevA) / dt;
                    float3 velB = (posB - prevB) / dt;
                    float relVelAlong = math.dot(velB - velA, gradient);
                    float dampingTerm = beam.damping * relVelAlong;
                    float effectiveConstraint = constraint + dampingTerm;

                    bool pinnedA = isPinned[nodeA];
                    bool pinnedB = isPinned[nodeB];

                    float wA = pinnedA ? 0f : 1f / nodeMasses[nodeA];
                    float wB = pinnedB ? 0f : 1f / nodeMasses[nodeB];
                    float wSum = wA + wB;

                    if (wSum <= 0f) continue;

                    float effectiveCompliance = beam.compliance / (dt * dt);
                    float lambda = -effectiveConstraint / (wSum + effectiveCompliance);

                    // For rigid constraints (zero compliance), don't limit lambda or correction
                    if (beam.compliance > 1e-9f)
                    {
                        float maxLambda = worldRestLength * 0.5f;
                        lambda = math.clamp(lambda, -maxLambda, maxLambda);
                    }

                    beamLagrangeMultipliers[beamIndex] += lambda;

                    float3 correction = gradient * lambda;

                    // Only limit corrections for soft constraints
                    if (beam.compliance > 1e-9f)
                    {
                        float maxCorrection = worldRestLength * 0.1f;
                        if (math.length(correction) > maxCorrection)
                            correction = math.normalize(correction) * maxCorrection;
                    }

                    if (!pinnedA)
                        predictedPositions[nodeA] -= wA * correction;
                    if (!pinnedB)
                        predictedPositions[nodeB] += wB * correction;
                }
            }
        }

        public struct BeamData
        {
            public int nodeA;
            public int nodeB;
            public float compliance;
            public float damping;
            public float originalRestLength;
            public float plasticityThreshold;
            public float plasticityRate;
            public float maxDeformation;
            public bool isActive;
        }

        #endregion

        // Runtime Visualization
        public bool showNodesAndLinks = false;
        public bool showNodes = true;
        public bool showLinks = true;
        public bool showNodeIndices = false;
        public bool showLinkForces = false;
        public float nodeDisplaySize = 0.1f;
        public Color nodeColor = Color.green;
        public Color pinnedNodeColor = Color.red;
        public Color linkColor = Color.blue;
        public Color influenceRadiusColor = new Color(1f, 1f, 0f, 0.3f);
        public float forceVisualizationScale = 0.01f;

        public Solver(float influenceRadius, Mesh mesh, Vector3[] originalVertices, Transform owner)
        {
            this.beams = new List<Beam>();
            this.meshDeformer = new MeshDeformer(mesh, originalVertices, influenceRadius);
            this.collisionPoints = new List<Vector3>();
            this.owner = owner;
            this.nodeManager = new NodeManager();
            nodeMasses = new List<float>();
        }

        public void InitializeFromTruss(Truss truss)
        {
            trussAsset = truss;
            if (truss != null)
            {
                nodeMasses = new List<float>(truss.NodeMasses);

                while (nodeMasses.Count < nodeManager.Nodes.Count)
                    nodeMasses.Add(0.5f);
                if (nodeMasses.Count > nodeManager.Nodes.Count)
                    nodeMasses.RemoveRange(nodeManager.Nodes.Count, nodeMasses.Count - nodeManager.Nodes.Count);

                compliance = truss.compliance;
                defaultDamping = truss.DefaultBeamDamping;
            }
            else
            {
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                    nodeMasses.Add(0.5f);
            }
        }

        #region Generate Methods
        public void GenerateNodesAndBeams(Vector3[] positions, float connectionDistance = -1f, Beam[] beamsArray = null, Transform parent = null)
        {
            if (positions == null || positions.Length == 0 ||
                (connectionDistance <= 0f && beamsArray == null))
            {
                Debug.LogWarning("Cannot generate nodes and beams: Invalid input.");
                return;
            }

            if (parent == null)
            {
                Debug.LogWarning("GenerateNodesAndBeams: parent Transform is null.");
                return;
            }

            if (s_isApplicationQuitting || parent.gameObject == null)
            {
                return;
            }

            nodeManager.Clear();
            beams.Clear();
            nodeMasses.Clear();
            DestroyOldNodes(parent);

            for (int i = 0; i < positions.Length; i++)
            {
                GameObject nodeObj = new GameObject($"Node_{i}");
                nodeObj.transform.parent = parent;
                nodeObj.transform.localPosition = positions[i];
                nodeObj.transform.localRotation = Quaternion.identity;
                nodeObj.AddComponent<SphereCollider>().radius = nodeManager.GetNodeRadius(i);

                nodeManager.AddNode(nodeObj.transform, positions[i]);
                nodeMasses.Add(trussAsset != null && i < trussAsset.NodeMasses.Count ? trussAsset.NodeMasses[i] : 0.5f);
            }

            if (beamsArray != null)
            {
                foreach (var b in beamsArray)
                {
                    if (b.nodeA >= 0 && b.nodeA < positions.Length &&
                        b.nodeB >= 0 && b.nodeB < positions.Length &&
                        b.restLength > 0.01f)
                    {
                        beams.Add(new Beam(
                            nodeA: b.nodeA,
                            nodeB: b.nodeB,
                            compliance: b.compliance,
                            damping: b.damping,
                            restLength: b.restLength,
                            plasticityThreshold: b.plasticityThreshold,
                            plasticityRate: b.plasticityRate,
                            maxDeformation: b.maxDeformation
                        ));
                    }
                }
            }
            else
            {
                for (int i = 0; i < positions.Length; i++)
                {
                    for (int j = i + 1; j < positions.Length; j++)
                    {
                        float distance = Vector3.Distance(positions[i], positions[j]);
                        if (distance <= connectionDistance && distance > 0.01f)
                        {
                            beams.Add(new Beam(i, j,
                                               compliance,
                                               defaultDamping,
                                               distance));
                        }
                    }
                }
            }

            meshDeformer.MapVerticesToNodes(parent, nodeManager.Nodes, nodeManager.InitialPositions);
        }

        public void GenerateCubeTest(Transform transform)
        {
            if (transform == null || meshDeformer.Mesh == null) return;

            Bounds bounds = meshDeformer.Mesh.bounds;
            Vector3 min = bounds.min;
            Vector3 max = bounds.max;

            Vector3[] cubeNodes = new Vector3[]
            {
                new Vector3(min.x, min.y, min.z),
                new Vector3(max.x, min.y, min.z),
                new Vector3(min.x, max.y, min.z),
                new Vector3(max.x, max.y, min.z),
                new Vector3(min.x, min.y, max.z),
                new Vector3(max.x, min.y, max.z),
                new Vector3(min.x, max.y, max.z),
                new Vector3(max.x, max.y, max.z)
            };

            GenerateNodesAndBeams(cubeNodes, Vector3.Distance(min, max) * 1.5f, parent: transform);
        }
        #endregion

        public void RestoreNodesAndBeams(Transform transform)
        {
            if (transform == null) return;

            var currentBeams = new List<Beam>(beams);
            var currentInitialPositions = new List<Vector3>(nodeManager.InitialPositions);

            nodeManager.Clear();
            beams.Clear();

            for (int i = 0; i < currentInitialPositions.Count; i++)
            {
                GameObject nodeObj = new GameObject($"Node_{i}");
                nodeObj.transform.parent = transform;
                nodeObj.transform.localPosition = currentInitialPositions[i];
                nodeObj.hideFlags = HideFlags.HideAndDontSave;

                nodeManager.AddNode(nodeObj.transform, currentInitialPositions[i]);
            }

            foreach (var beam in currentBeams)
            {
                if (beam.nodeA < nodeManager.Nodes.Count && beam.nodeB < nodeManager.Nodes.Count &&
                    nodeManager.Nodes[beam.nodeA] != null && nodeManager.Nodes[beam.nodeB] != null &&
                    beam.restLength > 0.01f)
                {
                    Vector3 localPosA = nodeManager.Nodes[beam.nodeA].localPosition;
                    Vector3 localPosB = nodeManager.Nodes[beam.nodeB].localPosition;
                    float distance = Vector3.Distance(localPosA, localPosB);
                    beams.Add(new Beam(beam.nodeA, beam.nodeB, beam.compliance, beam.damping, distance));
                }
            }

            Debug.Log($"Restored {nodeManager.Nodes.Count} nodes, {beams.Count} beams");
        }

        public void SetTrussAsset(Truss truss)
        {
            trussAsset = truss;
        }

        public void SetMatterAsset(Matter matter)
        {
            matterAsset = matter;
            if (matter != null)
            {
                float staticFriction = matter.StaticFriction;
                float dynamicFriction = matter.DynamicFriction;
            }
        }

        public void Solve()
        {
            if (nodeManager == null || beams == null || nodeManager.Nodes == null)
            {
                Debug.LogWarning("Cannot simulate: Invalid state.");
                return;
            }

            int numSubSteps = SceneSettings.Instance != null ?
                Mathf.Clamp(SceneSettings.Instance.BaseSubSteps, SceneSettings.Instance.MinSubSteps, 50) : 1;

            float simScale = SceneSettings.Instance != null ? SceneSettings.Instance.SimulationTimeScale : 1.0f;
            float fullDt = Time.fixedDeltaTime * simScale;
            float subDt = fullDt / numSubSteps;

            for (int step = 0; step < numSubSteps; step++)
            {
                SolveSubStep(subDt);
            }
        }

        #region SolveSubStep
        private void SolveSubStep(float dt)
        {
            const float epsilon = 0.001f;
            float dtSquared = dt * dt;
            int nodeCount = nodeManager.Nodes.Count;
            float damping = matterAsset != null ? matterAsset.CollisionDamping : 0.95f;

            float gravityValue = SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : 9.81f;
            Vector3 worldGravity = Vector3.down * gravityValue;
            int constraintIters = SceneSettings.Instance != null ? SceneSettings.Instance.ConstraintIterations : 20;

            collisionPoints.Clear();

            if (nodeManager.PreviousPositions.Count != nodeCount)
            {
                nodeManager.PreviousPositions.Clear();
                for (int i = 0; i < nodeCount; i++)
                {
                    Vector3 worldPos = nodeManager.Nodes[i] != null ? owner.TransformPoint(nodeManager.Nodes[i].localPosition) : Vector3.zero;
                    nodeManager.PreviousPositions.Add(worldPos);
                }
            }

            if (nodeManager.PredictedPositions.Count != nodeCount)
            {
                nodeManager.PredictedPositions.Clear();
                for (int i = 0; i < nodeCount; i++)
                {
                    Vector3 worldPos = nodeManager.Nodes[i] != null ? owner.TransformPoint(nodeManager.Nodes[i].localPosition) : Vector3.zero;
                    nodeManager.PredictedPositions.Add(worldPos);
                }
            }

            ApplyPressureForces(dt);

            nodeManager.AllocateNativeArrays();
            nodeManager.SyncToNativeArrays();

            var currentWorldPositions = new NativeArray<float3>(nodeCount, Allocator.TempJob);

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodeManager.Nodes[i] != null)
                {
                    currentWorldPositions[i] = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                }
            }

            float dampingFactor = Mathf.Pow(1.0f - (1f - damping), dt);

            var verletJob = new VerletIntegrationJob
            {
                currentWorldPositions = currentWorldPositions,
                previousPositions = nodeManager.nativePreviousPositions,
                isPinned = nodeManager.nativeIsPinned,
                dt = dt,
                dampingFactor = dampingFactor,
                worldGravity = worldGravity,
                dtSquared = dtSquared,
                predictedPositions = nodeManager.nativePredictedPositions
            };

            JobHandle verletHandle = verletJob.Schedule(nodeCount, 64);
            verletHandle.Complete();

            nodeManager.SyncFromNativeArrays();
            currentWorldPositions.Dispose();
            ResolveCollisions(dt, epsilon);

            var nativeBeamData = new NativeArray<BeamData>(beams.Count, Allocator.TempJob);
            var nativeBeamRestLengths = new NativeArray<float>(beams.Count, Allocator.TempJob);
            var nativeBeamLagrange = new NativeArray<float>(beams.Count, Allocator.TempJob);
            var nativeNodeMasses = new NativeArray<float>(nodeMasses.Count, Allocator.TempJob);

            for (int i = 0; i < beams.Count; i++)
            {
                if (!beams[i].IsCrossBody)
                {
                    nativeBeamData[i] = new BeamData
                    {
                        nodeA = beams[i].nodeA,
                        nodeB = beams[i].nodeB,
                        compliance = beams[i].compliance,
                        damping = beams[i].damping,
                        originalRestLength = beams[i].originalRestLength,
                        plasticityThreshold = beams[i].plasticityThreshold,
                        plasticityRate = beams[i].plasticityRate,
                        maxDeformation = beams[i].maxDeformation,
                        isActive = beams[i].isActive
                    };
                    nativeBeamRestLengths[i] = beams[i].restLength;
                    nativeBeamLagrange[i] = beams[i].lagrangeMultiplier;
                }
                else
                {
                    nativeBeamData[i] = new BeamData { isActive = false };
                }
            }

            for (int i = 0; i < nodeMasses.Count; i++)
            {
                nativeNodeMasses[i] = nodeMasses[i];
            }

            nodeManager.SyncToNativeArrays();

            float worldRestLengthScale = owner.TransformDirection(Vector3.right).magnitude;

            for (int iter = 0; iter < constraintIters; iter++)
            {
                var constraintJob = new SolveDistanceConstraintsJob
                {
                    beams = nativeBeamData,
                    previousPositions = nodeManager.nativePreviousPositions,
                    isPinned = nodeManager.nativeIsPinned,
                    nodeMasses = nativeNodeMasses,
                    dt = dt,
                    worldRestLengthScale = worldRestLengthScale,
                    predictedPositions = nodeManager.nativePredictedPositions,
                    beamRestLengths = nativeBeamRestLengths,
                    beamLagrangeMultipliers = nativeBeamLagrange
                };

                JobHandle constraintHandle = constraintJob.Schedule();
                constraintHandle.Complete();
            }

            for (int i = 0; i < beams.Count; i++)
            {
                if (!beams[i].IsCrossBody)
                {
                    beams[i].restLength = nativeBeamRestLengths[i];
                    beams[i].lagrangeMultiplier = nativeBeamLagrange[i];
                }
            }

            nodeManager.SyncFromNativeArrays();

            nativeBeamData.Dispose();
            nativeBeamRestLengths.Dispose();
            nativeBeamLagrange.Dispose();
            nativeNodeMasses.Dispose();

            SolveDistanceConstraintsCrossBody(dt);

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                Vector3 predictedWorldPos = nodeManager.PredictedPositions[i];

                Vector3 worldMotion = predictedWorldPos - currentWorldPos;
                float maxAllowedDisplacement = 1.0f;
                if (worldMotion.magnitude > maxAllowedDisplacement)
                {
                    predictedWorldPos = currentWorldPos + worldMotion.normalized * maxAllowedDisplacement;
                    nodeManager.PredictedPositions[i] = predictedWorldPos;
                }

                Vector3 finalLocalPos = owner.InverseTransformPoint(predictedWorldPos);
                nodeManager.Nodes[i].localPosition = finalLocalPos;
                nodeManager.PreviousPositions[i] = currentWorldPos;
            }
        }

        private void SolveDistanceConstraintsCrossBody(float dt)
        {
            foreach (var beam in beams)
            {
                if (!beam.isActive || !beam.IsCrossBody) continue;

                if (!beam.AreNodesValid()) continue;

                Vector3 posA = beam.GetNodeALocalPosition();
                Vector3 posB = beam.GetNodeBLocalPosition();
                Vector3 delta = posB - posA;
                float currentLength = delta.magnitude;

                if (currentLength < 0.001f) continue;

                Vector3 gradient = delta / currentLength;

                float worldRestLength = beam.bodyA.transform.TransformDirection(Vector3.right * beam.restLength).magnitude;
                float constraint = currentLength - worldRestLength;

                Vector3 prevA = beam.bodyA.solver.nodeManager.PreviousPositions[beam.nodeA];
                Vector3 prevB = beam.bodyB.solver.nodeManager.PreviousPositions[beam.nodeB];
                Vector3 velA = (posA - prevA) / dt;
                Vector3 velB = (posB - prevB) / dt;
                float relVelAlong = Vector3.Dot(velB - velA, gradient);
                float dampingTerm = beam.damping * relVelAlong;
                float effectiveConstraint = constraint + dampingTerm;

                bool pinnedA = beam.IsNodeAPinned();
                bool pinnedB = beam.IsNodeBPinned();

                float massA = beam.bodyA.solver.nodeMasses[beam.nodeA];
                float massB = beam.bodyB.solver.nodeMasses[beam.nodeB];

                float wA = pinnedA ? 0f : 1f / massA;
                float wB = pinnedB ? 0f : 1f / massB;
                float wSum = wA + wB;

                if (wSum <= 0f) continue;

                float effectiveCompliance = beam.compliance / (dt * dt);
                float lambda = -effectiveConstraint / (wSum + effectiveCompliance);

                // For rigid constraints (zero compliance), don't limit lambda or correction
                if (beam.compliance > 1e-9f)
                {
                    float maxLambda = worldRestLength * 0.5f;
                    lambda = Mathf.Clamp(lambda, -maxLambda, maxLambda);
                }

                beam.lagrangeMultiplier += lambda;

                Vector3 correction = gradient * lambda;

                // Only limit corrections for soft constraints
                if (beam.compliance > 1e-9f)
                {
                    float maxCorrection = worldRestLength * 0.1f;
                    if (correction.magnitude > maxCorrection)
                        correction = correction.normalized * maxCorrection;
                }

                beam.bodyA.solver.nodeManager.PredictedPositions[beam.nodeA] -= wA * correction;
                beam.bodyB.solver.nodeManager.PredictedPositions[beam.nodeB] += wB * correction;

                if (showLinkForces)
                {
                    Vector3 mid = (posA + posB) * 0.5f;
                    Color forceCol = constraint > 0f ? Color.red : Color.cyan;
                    Debug.DrawLine(mid, mid + correction * forceVisualizationScale * 100f, forceCol, dt);
                }
            }
        }

        private void ResolveCollisions(float dt, float epsilon)
        {
            int nodeCount = nodeManager.Nodes.Count;

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                Vector3 predictedWorldPos = nodeManager.PredictedPositions[i];
                float radius = nodeManager.GetNodeRadius(i);

                int hitCount = Physics.OverlapSphereNonAlloc(predictedWorldPos, radius, overlapResults, collisionLayerMask);

                for (int j = 0; j < hitCount; j++)
                {
                    Collider col = overlapResults[j];
                    if (IsOwnCollider(col, i)) continue;

                    if (GetCollisionInfo(col, predictedWorldPos, radius, out Vector3 contactPoint, out Vector3 normal, out float penetration))
                    {
                        // Step 1: Correct penetration
                        predictedWorldPos += normal * (penetration + epsilon);

                        // Step 2: Calculate velocity from corrected position
                        Vector3 velocity = (predictedWorldPos - currentWorldPos) / dt;

                        // Step 3: Apply impulse (restitution + friction)
                        Vector3 correctedVelocity = ApplyCollisionImpulse(velocity, normal, 1f / nodeMasses[i]);

                        // Step 4: Update position based on corrected velocity
                        predictedWorldPos = currentWorldPos + correctedVelocity * dt;

                        nodeManager.PredictedPositions[i] = predictedWorldPos;
                        collisionPoints.Add(contactPoint);

                        if (visualizeForces)
                        {
                            Debug.DrawRay(contactPoint, normal * 0.2f, Color.red, dt);
                        }
                    }
                }
            }
        }

        private Vector3 ApplyCollisionImpulse(Vector3 vel, Vector3 normal, float invMass)
        {
            float restitution = matterAsset != null ? matterAsset.Restitution : 0.02f;
            float dynamicFriction = matterAsset != null ? matterAsset.DynamicFriction : 0.6f;
            float staticFriction = matterAsset != null ? matterAsset.StaticFriction : 0.8f;

            float normalVel = Vector3.Dot(vel, normal);

            // Calculate tangential velocity
            Vector3 tangentVel = vel - normal * normalVel;
            float tangentSpeed = tangentVel.magnitude;

            Vector3 newVel = vel;

            // NORMAL IMPULSE: Only apply if moving into surface
            if (normalVel < 0)
            {
                float jn = -(1 + restitution) * normalVel / invMass;
                newVel += normal * jn * invMass;

                // Update normal velocity after collision response
                normalVel = Vector3.Dot(newVel, normal);
            }

            // FRICTION: Apply for both resting and sliding contacts
            if (tangentSpeed > 0.0001f)
            {
                Vector3 tangent = tangentVel / tangentSpeed;

                // For resting contacts, use gravity-based normal force
                // For collision contacts, use the impulse-based force
                float normalForce;
                if (normalVel < -0.01f)
                {
                    // Active collision - use impulse-based normal force
                    float jn = -(1 + restitution) * normalVel / invMass;
                    normalForce = Mathf.Abs(jn);
                }
                else
                {
                    // Resting or slow sliding contact - use weight-based normal force
                    float gravityValue = SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : 9.81f;
                    float mass = 1f / invMass;
                    normalForce = mass * gravityValue * Time.fixedDeltaTime;
                }

                // Calculate friction impulse magnitude
                float tangentImpulse = tangentSpeed / invMass;
                float maxStaticFriction = staticFriction * normalForce;
                float maxDynamicFriction = dynamicFriction * normalForce;

                // Apply static or dynamic friction
                if (tangentImpulse <= maxStaticFriction)
                {
                    // Static friction: stop all tangential motion
                    newVel -= tangentVel;
                }
                else
                {
                    // Dynamic friction: reduce tangential velocity
                    Vector3 frictionForce = -tangent * maxDynamicFriction * invMass;
                    newVel += frictionForce;
                }
            }

            return newVel;
        }

        private bool GetCollisionInfo(Collider col, Vector3 pos, float radius, out Vector3 contactPoint, out Vector3 normal, out float penetration)
        {
            contactPoint = Vector3.zero;
            normal = Vector3.up;
            penetration = 0f;

            if (col is TerrainCollider terrainCol)
            {
                Terrain terrain = terrainCol.GetComponent<Terrain>();
                if (terrain != null)
                {
                    float height = terrain.SampleHeight(pos) + terrain.transform.position.y;

                    if (pos.y < height + radius)
                    {
                        contactPoint = new Vector3(pos.x, height, pos.z);
                        normal = Vector3.up;
                        penetration = (height + radius) - pos.y;
                        return true;
                    }
                }
            }
            else if (col is MeshCollider meshCol && !meshCol.convex)
            {
                Vector3 direction = Vector3.down;
                if (col.Raycast(new Ray(pos + Vector3.up * radius, direction), out RaycastHit meshHit, radius * 2f))
                {
                    contactPoint = meshHit.point;
                    normal = meshHit.normal;
                    penetration = radius - Vector3.Distance(pos, meshHit.point);
                    return penetration > 0;
                }
            }
            else
            {
                contactPoint = col.ClosestPoint(pos);
                float dist = Vector3.Distance(contactPoint, pos);

                if (dist < radius)
                {
                    normal = dist > 0.001f ? (pos - contactPoint).normalized : Vector3.up;
                    penetration = radius - dist;
                    return true;
                }
            }

            return false;
        }

        private bool IsOwnCollider(Collider col, int nodeIndex)
        {
            if (col == nodeManager.Colliders[nodeIndex]) return true;
            if (col.transform == nodeManager.Nodes[nodeIndex]) return true;

            int otherNodeIndex = nodeManager.FindIndex(n => n != null && n == col.transform);
            return otherNodeIndex != -1;
        }
        #endregion

        #region Helper Methods
        public void DeformMesh(Transform transform)
        {
            if (transform == null) return;
            UpdateNodeRotations();
            meshDeformer.Deform(transform, nodeManager.Nodes, nodeManager.InitialPositions, nodeManager.InitialRotations);
        }
        public Vector3 CalculateVolumeCenter()
        {
            if (nodeManager == null || nodeManager.Nodes == null || nodeManager.Nodes.Count == 0)
                return Vector3.zero;

            Vector3 center = Vector3.zero;
            int validNodes = 0;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] != null)
                {
                    center += owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                    validNodes++;
                }
            }

            if (validNodes > 0)
                center /= validNodes;

            cachedVolumeCenter = center;
            return center;
        }
        private void ApplyPressureForces(float dt)
        {
            if (internalPressure <= 0f || nodeManager.Nodes.Count == 0)
                return;

            // Calculate volume center
            Vector3 volumeCenter = CalculateVolumeCenter();

            // Apply outward forces from center to each node
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i])
                    continue;

                // Get node position in world space
                Vector3 nodeWorldPos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);

                // Calculate direction from center to node
                Vector3 direction = nodeWorldPos - volumeCenter;
                float distance = direction.magnitude;

                // Skip if node is at center (shouldn't happen but safety check)
                if (distance < 0.001f)
                    continue;

                direction /= distance;

                float pressureForce = internalPressure * 10f; // Scale factor for good behavior

                // Apply distance falloff (nodes closer to center get more force)
                // This creates a more natural pressure distribution
                float distanceFactor = 1f / (distance + 0.1f);
                Vector3 force = direction * pressureForce * distanceFactor;

                // Apply the force to the node
                ApplyWorldForceToNode(i, force, null, dt);
            }
        }
        public void ApplyForceToNode(int nodeIndex, Vector3 localForce)
        {
            if (nodeIndex < 0 || nodeIndex >= nodeMasses.Count || nodeManager.IsPinned[nodeIndex]) return;

            float mass = nodeMasses[nodeIndex];
            if (mass <= 0f) return;

            float simScale = SceneSettings.Instance != null ? SceneSettings.Instance.SimulationTimeScale : 1.0f;
            float fullDt = Time.fixedDeltaTime * simScale;

            int subSteps = SceneSettings.Instance != null ? Mathf.Clamp(SceneSettings.Instance.BaseSubSteps, SceneSettings.Instance.MinSubSteps, 50) : 1;
            float effectiveDt = fullDt / Mathf.Max(1, subSteps);

            Vector3 worldForce = owner.TransformDirection(localForce);
            ApplyWorldForceToNode(nodeIndex, worldForce, mass, effectiveDt);
        }

        public void ApplyWorldForceToNode(int nodeIndex, Vector3 worldForce, float? customMass = null, float? customDt = null)
        {
            if (nodeIndex < 0 || nodeIndex >= nodeMasses.Count || nodeManager.IsPinned[nodeIndex]) return;

            float mass = customMass ?? nodeMasses[nodeIndex];
            if (mass <= 0f) return;

            float dt = customDt ?? Time.fixedDeltaTime;
            Vector3 accel = worldForce / mass;

            Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[nodeIndex].localPosition);
            Vector3 currentVelocity = (currentWorldPos - nodeManager.PreviousPositions[nodeIndex]) / dt;
            Vector3 newVelocity = currentVelocity + accel * dt;
            nodeManager.PreviousPositions[nodeIndex] = currentWorldPos - newVelocity * dt;
        }

        public int FindClosestNodeToRay(Ray ray, float maxDistance)
        {
            int closestNode = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null) continue;
                Vector3 nodePos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                float distance = Vector3.Cross(ray.direction, nodePos - ray.origin).magnitude;
                if (distance < nodeManager.GetNodeRadius(i) && distance < minDistance)
                {
                    float t = Vector3.Dot(nodePos - ray.origin, ray.direction);
                    if (t > 0 && t < maxDistance)
                    {
                        minDistance = distance;
                        closestNode = i;
                    }
                }
            }

            return closestNode;
        }

        public void ApplyImpulse(int nodeIndex, Vector3 targetPosition, float strength)
        {
            if (nodeIndex >= 0 && nodeIndex < nodeManager.Nodes.Count && nodeMasses.Count > nodeIndex && nodeManager.Nodes[nodeIndex] != null && !nodeManager.IsPinned[nodeIndex])
            {
                float simScale = SceneSettings.Instance != null ? SceneSettings.Instance.SimulationTimeScale : 1.0f;
                float effectiveDt = Time.fixedDeltaTime * simScale;

                Vector3 currentPos = nodeManager.Nodes[nodeIndex].localPosition;
                Vector3 targetLocalPos = owner.InverseTransformPoint(targetPosition);
                Vector3 delta = targetLocalPos - currentPos;
                Vector3 impulse = delta * strength * nodeMasses[nodeIndex] / effectiveDt;
                ApplyForceToNode(nodeIndex, impulse);
            }
        }

        private void DestroyOldNodes(Transform parent)
        {
            var toKill = new List<GameObject>();
            foreach (Transform child in parent)
            {
                if (child.name.StartsWith("Node_"))
                    toKill.Add(child.gameObject);
            }
            foreach (var go in toKill)
            {
                if (Application.isEditor && !Application.isPlaying)
                    Object.DestroyImmediate(go);
                else
                    Object.Destroy(go);
            }
        }
        #endregion

        #region Node Rotation
        private void UpdateNodeRotations()
        {
            int nodeCount = nodeManager.Nodes.Count;
            List<List<int>> neighbors = new List<List<int>>(nodeCount);
            for (int i = 0; i < nodeCount; i++)
            {
                neighbors.Add(new List<int>());
            }

            foreach (var beam in beams)
            {
                neighbors[beam.nodeA].Add(beam.nodeB);
                neighbors[beam.nodeB].Add(beam.nodeA);
            }

            for (int i = 0; i < nodeCount; i++)
            {
                List<int> cluster = new List<int>(neighbors[i]);
                cluster.Add(i);
                if (cluster.Count < 2) continue;

                Vector3 com_rest = Vector3.zero;
                Vector3 com_current = Vector3.zero;
                float totalMass = 0f;

                for (int k = 0; k < cluster.Count; k++)
                {
                    int idx = cluster[k];
                    float m = nodeMasses[idx];
                    com_rest += m * nodeManager.InitialPositions[idx];
                    com_current += m * nodeManager.Nodes[idx].localPosition;
                    totalMass += m;
                }

                if (totalMass < 1e-6f) continue;

                com_rest /= totalMass;
                com_current /= totalMass;

                Matrix4x4 A = Matrix4x4.zero;
                for (int k = 0; k < cluster.Count; k++)
                {
                    int idx = cluster[k];
                    Vector3 p_rest = nodeManager.InitialPositions[idx] - com_rest;
                    Vector3 p_current = nodeManager.Nodes[idx].localPosition - com_current;
                    float m = nodeMasses[idx];
                    A[0, 0] += m * p_current.x * p_rest.x; A[0, 1] += m * p_current.x * p_rest.y; A[0, 2] += m * p_current.x * p_rest.z;
                    A[1, 0] += m * p_current.y * p_rest.x; A[1, 1] += m * p_current.y * p_rest.y; A[1, 2] += m * p_current.y * p_rest.z;
                    A[2, 0] += m * p_current.z * p_rest.x; A[2, 1] += m * p_current.z * p_rest.y; A[2, 2] += m * p_current.z * p_rest.z;
                }

                Quaternion q = nodeManager.Nodes[i].localRotation;
                Matrix4x4 R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);

                int maxIters = 5;
                float eps = 1e-9f;
                for (int iter = 0; iter < maxIters; iter++)
                {
                    Vector3 r0 = R.GetColumn(0);
                    Vector3 r1 = R.GetColumn(1);
                    Vector3 r2 = R.GetColumn(2);
                    Vector3 a0 = A.GetColumn(0);
                    Vector3 a1 = A.GetColumn(1);
                    Vector3 a2 = A.GetColumn(2);
                    Vector3 tau = Vector3.Cross(r0, a0) + Vector3.Cross(r1, a1) + Vector3.Cross(r2, a2);
                    float denom = Mathf.Abs(Vector3.Dot(r0, a0) + Vector3.Dot(r1, a1) + Vector3.Dot(r2, a2)) + eps;
                    Vector3 omega = tau / denom;
                    float w = omega.magnitude;
                    if (w < 1e-6f) break;

                    Vector3 axis = omega / w;
                    Quaternion deltaQ = Quaternion.AngleAxis(w * Mathf.Rad2Deg, axis);
                    q = deltaQ * q;
                    q = q.normalized;
                    R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);
                }

                nodeManager.Nodes[i].localRotation = q;
            }
        }
        #endregion

        #region Cleanup
        ~Solver()
        {
            Cleanup();
        }

        public void Cleanup()
        {
            nodeManager?.DisposeNativeArrays();
        }
        #endregion
    }
}