/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
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
        #region Fields

        private readonly Transform _owner;
        public readonly NodeManager nodeManager;
        public List<Beam> beams;
        public readonly MeshDeformer meshDeformer;
        public CollisionHandler collisionHandler;

        public List<float> nodeMasses = new List<float>();

        private Truss _trussAsset;
        private Matter _matterAsset;

        private NativeArray<float3> _reusableWorldPositions;
        private readonly List<int> _reusableNeighborsList = new List<int>();
        private readonly List<GameObject> _reusableDestroyList = new List<GameObject>();

        #endregion

        #region Properties
        public bool visualizeForces { get; set; } = false;
        public float compliance { get; set; } = 1e-3f;
        public float defaultDamping { get; set; } = 0.3f;
        public float maxDeformation { get; set; } = 1f;
        public float internalPressure { get; set; } = 0f;
        private Vector3 _cachedVolumeCenter = Vector3.zero;
        private int _currentFrameIndex = 0;
        #endregion

        #region Constructor

        public Solver(float influenceRadius, Mesh mesh, Vector3[] originalVertices, Transform owner)
        {
            this.beams = new List<Beam>();
            this.meshDeformer = new MeshDeformer(mesh, originalVertices, influenceRadius);
            this.collisionHandler = new CollisionHandler();
            this._owner = owner;
            this.nodeManager = new NodeManager();
            nodeMasses = new List<float>();
        }

        #endregion

        #region Initialization Methods


        public void InitializeFromTruss(Truss truss)
        {
            _trussAsset = truss;

            if (truss != null)
            {
                nodeMasses = new List<float>(truss.NodeMasses);

                while (nodeMasses.Count < nodeManager.Nodes.Count)
                    nodeMasses.Add(PhysicsConstants.DEFAULT_NODE_MASS);

                if (nodeMasses.Count > nodeManager.Nodes.Count)
                    nodeMasses.RemoveRange(nodeManager.Nodes.Count,
                                          nodeMasses.Count - nodeManager.Nodes.Count);

                compliance = truss.compliance;
                defaultDamping = truss.DefaultBeamDamping;
            }
            else
            {
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                    nodeMasses.Add(PhysicsConstants.DEFAULT_NODE_MASS);
            }
        }


        public void SetTrussAsset(Truss truss)
        {
            _trussAsset = truss;
        }


        public void SetMatterAsset(Matter matter)
        {
            _matterAsset = matter;
        }

        #endregion

        #region Node Generation
        public void GenerateNodesAndBeams(Vector3[] positions, float connectionDistance = -1f,
                                         Beam[] beamsArray = null, Transform parent = null)
        {
            if (!ValidateGenerationParameters(positions, connectionDistance, beamsArray, parent))
                return;

            if (parent.gameObject == null)
                return;

            ClearExistingNodes(parent);
            GenerateNodes(positions, parent);

            if (beamsArray != null)
                GenerateBeamsFromArray(beamsArray, positions);
            else
                GenerateBeamsFromDistance(positions, connectionDistance);

            meshDeformer.MapVerticesToNodes(parent, nodeManager.Nodes,
                                          nodeManager.InitialPositions);
        }
        private bool ValidateGenerationParameters(Vector3[] positions, float connectionDistance,
                                                 Beam[] beamsArray, Transform parent)
        {
            if (positions == null || positions.Length == 0 ||
                (connectionDistance <= 0f && beamsArray == null))
            {
                Debug.LogWarning("[Solver] Cannot generate nodes and beams: Invalid input.");
                return false;
            }

            if (parent == null)
            {
                Debug.LogWarning("[Solver] GenerateNodesAndBeams: parent Transform is null.");
                return false;
            }

            return true;
        }
        private void ClearExistingNodes(Transform parent)
        {
            nodeManager.Clear();
            beams.Clear();
            nodeMasses.Clear();
            DestroyOldNodes(parent);
        }
        private void GenerateNodes(Vector3[] positions, Transform parent)
        {
            for (int i = 0; i < positions.Length; i++)
            {
                GameObject nodeObj = new GameObject($"Node_{i}");
                nodeObj.transform.position = parent.TransformPoint(positions[i]);
                nodeObj.transform.localRotation = Quaternion.identity;
                nodeObj.hideFlags = HideFlags.HideAndDontSave;

                nodeManager.AddNode(nodeObj.transform, positions[i]);

                float mass = (_trussAsset != null && i < _trussAsset.NodeMasses.Count)
                    ? _trussAsset.NodeMasses[i]
                    : PhysicsConstants.DEFAULT_NODE_MASS;
                nodeMasses.Add(mass);
            }
        }
        private void GenerateBeamsFromArray(Beam[] beamsArray, Vector3[] positions)
        {
            foreach (var b in beamsArray)
            {
                if (IsValidBeam(b, positions))
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
        private bool IsValidBeam(Beam beam, Vector3[] positions)
        {
            return beam.nodeA >= 0 && beam.nodeA < positions.Length &&
                   beam.nodeB >= 0 && beam.nodeB < positions.Length &&
                   beam.restLength > PhysicsConstants.MIN_BEAM_LENGTH;
        }
        private void GenerateBeamsFromDistance(Vector3[] positions, float connectionDistance)
        {
            for (int i = 0; i < positions.Length; i++)
            {
                for (int j = i + 1; j < positions.Length; j++)
                {
                    float distance = Vector3.Distance(positions[i], positions[j]);

                    if (distance <= connectionDistance &&
                        distance > PhysicsConstants.MIN_BEAM_LENGTH)
                    {
                        beams.Add(new Beam(i, j, compliance, defaultDamping, distance));
                    }
                }
            }
        }
        public void GenerateCubeTest(Transform transform)
        {
            if (transform == null || meshDeformer.Mesh == null)
                return;

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

            float connectionDistance = Vector3.Distance(min, max) *
                PhysicsConstants.CUBE_CONNECTION_DISTANCE_MULTIPLIER;

            GenerateNodesAndBeams(cubeNodes, connectionDistance, parent: transform);
        }

        #endregion

        #region Node Query Methods


        public Vector3 GetNodeWorldPosition(int index)
        {
            if (index < 0 || index >= nodeManager.Nodes.Count)
                return Vector3.zero;

            return nodeManager.Nodes[index].position;
        }

        #endregion
        #region Synchronized Simulation Methods
        public void SubStep_PrepareAndIntegrate(float dt)
        {
            if (!ValidateSimulationState("Integrate")) return;

            foreach (var beam in beams)
                beam.lagrangeMultiplier = 0f;

            int nodeCount = nodeManager.Nodes.Count;
            float damping = 0.1f;
            float gravityValue = SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : PhysicsConstants.STANDARD_GRAVITY;
            Vector3 worldGravity = Vector3.down * gravityValue;

            EnsureBufferSizes(nodeCount);

            ApplyPressureForces(dt);

            PerformVerletIntegration(dt, dt * dt, nodeCount, damping, worldGravity);
            _currentFrameIndex++;
        }

        public void SubStep_ResolveConstraints(float dt)
        {
            if (nodeManager.Nodes.Count == 0) return;

            int constraintIters = SceneSettings.Instance != null
                ? SceneSettings.Instance.ConstraintIterations : 20;

            SolveConstraints(dt, constraintIters, nodeManager.Nodes.Count);
            CheckAndBreakConstraints();
            ResolveCollisions(dt, nodeManager.Nodes.Count);
        }

        public void SubStep_Finalize()
        {
            UpdateNodePositions(nodeManager.Nodes.Count);
        }

        #endregion

        #region Main Simulation Methods


        public void Solve()
        {
            if (!ValidateSimulationState("Solve")) return;

            int numSubSteps = GetSimulationSubSteps();
            float fullDt = Time.fixedDeltaTime * GetSimulationTimeScale();
            float subDt = fullDt / numSubSteps;

            for (int step = 0; step < numSubSteps; step++)
            {
                SubStep_PrepareAndIntegrate(subDt);
                SubStep_ResolveConstraints(subDt);
            }

            SubStep_Finalize();
        }


        private bool ValidateSimulationState(string context)
        {
            if (nodeManager == null)
            {
                Debug.LogError($"[{context}] NodeManager is null");
                return false;
            }

            if (beams == null)
            {
                Debug.LogError($"[{context}] Beams list is null");
                return false;
            }

            if (nodeManager.Nodes == null || nodeManager.Nodes.Count == 0)
            {
                Debug.LogWarning($"[{context}] No nodes to simulate");
                return false;
            }

            if (_owner == null)
            {
                Debug.LogError($"[{context}] Owner transform is null");
                return false;
            }

            return true;
        }

        public int GetSimulationSubSteps()
        {
            if (SceneSettings.Instance == null)
                return 1;

            return SceneSettings.Instance.SubstepCount;
        }


        private float GetSimulationTimeScale()
        {
            return SceneSettings.Instance != null
                ? SceneSettings.Instance.TimeScale
                : 1.0f;
        }

        #endregion

        #region Sub-Step Simulation


        private void EnsureBufferSizes(int nodeCount)
        {
            EnsureBufferSize(nodeManager.PreviousPositions, nodeCount, true);
            EnsureBufferSize(nodeManager.PredictedPositions, nodeCount, false);
        }

        private void EnsureBufferSize(List<Vector3> buffer, int requiredSize, bool isPrevious)
        {
            if (buffer.Count == requiredSize) return;
            buffer.Clear();

            for (int i = 0; i < requiredSize; i++)
            {
                // Use .position directly
                Vector3 worldPos = nodeManager.Nodes[i] != null
                    ? nodeManager.Nodes[i].position
                    : Vector3.zero;
                buffer.Add(worldPos);
            }
        }

        #endregion

        #region Verlet Integration

        private NativeArray<float3> CreateCurrentWorldPositions(int nodeCount)
        {
            var currentWorldPositions = new NativeArray<float3>(nodeCount, Allocator.TempJob);

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodeManager.Nodes[i] != null)
                {
                    currentWorldPositions[i] = nodeManager.Nodes[i].position;
                }
                else
                {
                    currentWorldPositions[i] = float3.zero;
                }
            }

            return currentWorldPositions;
        }

        private void PerformVerletIntegration(float dt, float dtSquared, int nodeCount,
                                             float damping, Vector3 worldGravity)
        {
            nodeManager.AllocateNativeArrays();
            nodeManager.SyncToNativeArrays();

            NativeArray<float3> currentWorldPositions = CreateCurrentWorldPositions(nodeCount);

            float dampingFactor = Mathf.Exp(-damping * dt);

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

            JobHandle verletHandle = verletJob.Schedule(nodeCount,
                                                        PhysicsConstants.VERLET_JOB_BATCH_SIZE);
            verletHandle.Complete();

            currentWorldPositions.Dispose();

            nodeManager.SyncFromNativeArrays();
        }

        #endregion

        #region Collision Resolution
        private void ResolveCollisions(float dt, int nodeCount)
        {
            collisionHandler.ResolveCollisions(nodeManager, _owner, _matterAsset,
                                              nodeMasses, dt,
                                              PhysicsConstants.DEFAULT_COLLISION_EPSILON,
                                              visualizeForces);
        }

        #endregion

        #region Constraint Solving

        private void SolveConstraints(float dt, int constraintIters, int nodeCount)
        {
            for (int i = 0; i < beams.Count; i++)
            {
                beams[i].lagrangeMultiplier = 0f;
            }

            float worldRestLengthScale = _owner.TransformDirection(Vector3.right).magnitude;

            for (int iter = 0; iter < constraintIters; iter++)
            {
                for (int beamIndex = 0; beamIndex < beams.Count; beamIndex++)
                {
                    Beam beam = beams[beamIndex];
                    if (!beam.isActive || beam.IsCrossBody)
                        continue;

                    int nodeA = beam.nodeA;
                    int nodeB = beam.nodeB;

                    Vector3 posA = nodeManager.PredictedPositions[nodeA];
                    Vector3 posB = nodeManager.PredictedPositions[nodeB];
                    Vector3 delta = posA - posB;
                    float currentLength = delta.magnitude;

                    if (currentLength < 1e-7f) continue;

                    float restLength = beam.restLength * worldRestLengthScale;
                    float constraint = currentLength - restLength;

                    float wA = nodeManager.IsPinned[nodeA] ? 0f : 1f / nodeMasses[nodeA];
                    float wB = nodeManager.IsPinned[nodeB] ? 0f : 1f / nodeMasses[nodeB];
                    float wSum = wA + wB;
                    if (wSum <= 0f) continue;

                    float invDtSq = 1f / (dt * dt);
                    float alphaTilde = beam.compliance * invDtSq;

                    float currentLambda = beam.lagrangeMultiplier;
                    float deltaLambda = -(constraint + alphaTilde * currentLambda) / (wSum + alphaTilde);

                    beam.lagrangeMultiplier = currentLambda + deltaLambda;

                    Vector3 gradient = delta / currentLength;
                    Vector3 correction = deltaLambda * gradient;

                    if (!nodeManager.IsPinned[nodeA])
                        nodeManager.PredictedPositions[nodeA] += wA * correction;
                    if (!nodeManager.IsPinned[nodeB])
                        nodeManager.PredictedPositions[nodeB] -= wB * correction;
                }

                ApplyPlasticityDuringIteration(dt, worldRestLengthScale);
            }

            SolveDistanceConstraintsCrossBody(dt);
        }

        #endregion
        #region Plasticity
        private void ApplyPlasticityDuringIteration(float dt, float worldRestLengthScale)
        {
            for (int beamIndex = 0; beamIndex < beams.Count; beamIndex++)
            {
                Beam beam = beams[beamIndex];
                if (!beam.isActive || beam.IsCrossBody) continue;

                Vector3 posA = nodeManager.PredictedPositions[beam.nodeA];
                Vector3 posB = nodeManager.PredictedPositions[beam.nodeB];
                float currentLength = Vector3.Distance(posA, posB);

                if (currentLength < PhysicsConstants.MIN_BEAM_LENGTH)
                    continue;

                float restLength = beam.restLength;
                float worldRestLength = restLength * worldRestLengthScale;
                float constraint = currentLength - worldRestLength;
                float strain = constraint / worldRestLength;
                if (strain <= beam.plasticityThreshold)
                    continue;

                float excessStrain = strain - beam.plasticityThreshold;
                float plasticDeformation = excessStrain * beam.plasticityRate * dt;

                float lengthDiff = Mathf.Abs(restLength - beam.originalRestLength);
                float totalDeformation = lengthDiff / beam.originalRestLength;

                if (totalDeformation >= beam.maxDeformation)
                    continue;

                float deformationAmount = plasticDeformation * restLength;

                float sign = constraint > 0f ? 1f : -1f;
                restLength += sign * deformationAmount;

                float minLength = beam.originalRestLength * (1f - beam.maxDeformation);
                float maxLength = beam.originalRestLength * (1f + beam.maxDeformation);
                restLength = Mathf.Clamp(restLength, minLength, maxLength);

                beam.restLength = restLength;
            }
        }

        #endregion

        #region Cross-Body Constraint Solving


        private void SolveDistanceConstraintsCrossBody(float dt)
        {
            foreach (var beam in beams)
            {
                if (!beam.isActive || !beam.IsCrossBody)
                    continue;

                if (!beam.AreNodesValid())
                    continue;
                if (beam.lastSolvedFrame == _currentFrameIndex)
                    continue;

                beam.lastSolvedFrame = _currentFrameIndex;

                SolveCrossBodyBeamConstraint(beam, dt);
            }
        }


        private void SolveCrossBodyBeamConstraint(Beam beam, float dt)
        {
            if (beam.isEdgeSliding)
            {
                SolveEdgeSlidingConstraint(beam, dt);
                return;
            }

            Vector3 posA = beam.GetNodeALocalPosition();
            Vector3 posB = beam.GetNodeBLocalPosition();
            Vector3 delta = posB - posA;
            float currentLength = delta.magnitude;

            if (currentLength < PhysicsConstants.MIN_BEAM_LENGTH)
                return;

            Vector3 gradient = delta / currentLength;

            bool hasMinLimit = beam.minLength > 0f;
            bool hasMaxLimit = !float.IsPositiveInfinity(beam.maxLength);

            // FIX: Use local-space lengths directly instead of converting to world space each frame
            // This prevents jitter from transform changes during physics updates
            float targetLength = beam.restLength;

            if (hasMinLimit && currentLength < beam.minLength)
            {
                targetLength = beam.minLength;
            }
            else if (hasMaxLimit && currentLength > beam.maxLength)
            {
                targetLength = beam.maxLength;
            }

            float constraint = currentLength - targetLength;

            // FIX: Increase damping for motors to prevent oscillation
            const float MOTOR_DAMPING_MULTIPLIER = 2.5f;
            float motorDamping = (hasMinLimit || hasMaxLimit) ? beam.damping * MOTOR_DAMPING_MULTIPLIER : beam.damping;
            float dampedCompliance = beam.compliance + (motorDamping + PhysicsConstants.CROSS_BODY_CONSTRAINT_DAMPING) * dt;

            bool pinnedA = beam.IsNodeAPinned();
            bool pinnedB = beam.IsNodeBPinned();
            float wA = pinnedA ? 0f : 1f / beam.bodyA.solver.nodeMasses[beam.nodeA];
            float wB = pinnedB ? 0f : 1f / beam.bodyB.solver.nodeMasses[beam.nodeB];
            float wSum = wA + wB;

            if (wSum <= 0f) return;

            float effectiveCompliance = dampedCompliance / (dt * dt);
            float deltaLambda = -constraint / (wSum + effectiveCompliance);

            if (!float.IsFinite(deltaLambda))
                return;

            if (hasMinLimit && !hasMaxLimit)
            {
                float newLambda = beam.lagrangeMultiplier + deltaLambda;
                if (newLambda < 0f)
                    deltaLambda = -beam.lagrangeMultiplier;
            }

            beam.lagrangeMultiplier += deltaLambda;

            Vector3 correction = gradient * deltaLambda;

            if (correction.magnitude > PhysicsConstants.MAX_ALLOWED_DISPLACEMENT)
            {
                correction = correction.normalized * PhysicsConstants.MAX_ALLOWED_DISPLACEMENT;
            }

            if (!pinnedA)
                beam.bodyA.solver.nodeManager.PredictedPositions[beam.nodeA] -= wA * correction;
            if (!pinnedB)
                beam.bodyB.solver.nodeManager.PredictedPositions[beam.nodeB] += wB * correction;
        }
        private void SolveEdgeSlidingConstraint(Beam beam, float dt)
        {
            int slidingNode = beam.slidingNode;
            int edgeNodeA = beam.edgeNodeA;
            int edgeNodeB = beam.edgeNodeB;

            SoftBody slidingBody = beam.bodyA;
            SoftBody edgeBody = beam.bodyB;

            Vector3 nodePos = slidingBody.solver.nodeManager.PredictedPositions[slidingNode];
            Vector3 edgePosA = edgeBody.solver.nodeManager.PredictedPositions[edgeNodeA];
            Vector3 edgePosB = edgeBody.solver.nodeManager.PredictedPositions[edgeNodeB];

            Vector3 edgeVector = edgePosB - edgePosA;
            float edgeLength = edgeVector.magnitude;

            if (edgeLength < PhysicsConstants.MIN_BEAM_LENGTH)
            {
                Debug.LogWarning($"[EdgeSliding] Edge too short: {edgeLength}");
                return;
            }

            Vector3 edgeDir = edgeVector / edgeLength;

            Vector3 nodeToEdgeStart = nodePos - edgePosA;
            float t = Vector3.Dot(nodeToEdgeStart, edgeDir);

            Vector3 closestPoint = edgePosA + edgeDir * t;
            Vector3 perpVector = nodePos - closestPoint;
            float perpDistance = perpVector.magnitude;

            if (perpDistance < PhysicsConstants.MIN_BEAM_LENGTH)
            {
                return;
            }

            Vector3 perpDir = perpVector / perpDistance;

            float targetPerpDistance = beam.targetPerpDistance;
            float constraint = perpDistance - targetPerpDistance;

            float dampedCompliance = beam.compliance + (beam.damping + PhysicsConstants.CROSS_BODY_CONSTRAINT_DAMPING) * dt;

            bool pinnedNode = slidingBody.solver.nodeManager.IsPinned[slidingNode];
            bool pinnedEdgeA = edgeBody.solver.nodeManager.IsPinned[edgeNodeA];
            bool pinnedEdgeB = edgeBody.solver.nodeManager.IsPinned[edgeNodeB];

            if (pinnedNode && pinnedEdgeA && pinnedEdgeB)
            {
                Debug.LogWarning("[EdgeSliding] All nodes pinned!");
                return;
            }

            float massNode = slidingBody.solver.nodeMasses[slidingNode];
            float massEdgeA = edgeBody.solver.nodeMasses[edgeNodeA];
            float massEdgeB = edgeBody.solver.nodeMasses[edgeNodeB];

            float weightA = 1.0f - (t / edgeLength);
            float weightB = t / edgeLength;

            float wNode = pinnedNode ? 0f : 1f / massNode;
            float wEdgeA = pinnedEdgeA ? 0f : (1f / massEdgeA) * weightA * weightA;
            float wEdgeB = pinnedEdgeB ? 0f : (1f / massEdgeB) * weightB * weightB;
            float wSum = wNode + wEdgeA + wEdgeB;

            if (wSum <= 0f)
            {
                Debug.LogWarning("[EdgeSliding] Zero mass sum!");
                return;
            }

            float effectiveCompliance = dampedCompliance / (dt * dt);
            float deltaLambda = -constraint / (wSum + effectiveCompliance);

            if (!float.IsFinite(deltaLambda))
                return;

            beam.lagrangeMultiplier += deltaLambda;

            float maxCorrection = Mathf.Max(0.1f, perpDistance * 0.5f);
            float correctionMagnitude = Mathf.Abs(deltaLambda);
            if (correctionMagnitude > maxCorrection)
            {
                deltaLambda = Mathf.Sign(deltaLambda) * maxCorrection;
            }

            if (!pinnedNode)
            {
                Vector3 nodeCorrection = (1f / massNode) * deltaLambda * perpDir;
                slidingBody.solver.nodeManager.PredictedPositions[slidingNode] += nodeCorrection;
            }

            if (!pinnedEdgeA)
            {
                Vector3 edgeACorrection = (1f / massEdgeA) * deltaLambda * (-perpDir * weightA);
                edgeBody.solver.nodeManager.PredictedPositions[edgeNodeA] += edgeACorrection;
            }

            if (!pinnedEdgeB)
            {
                Vector3 edgeBCorrection = (1f / massEdgeB) * deltaLambda * (-perpDir * weightB);
                edgeBody.solver.nodeManager.PredictedPositions[edgeNodeB] += edgeBCorrection;
            }
        }
        private void CheckAndBreakConstraints()
        {
            float dt = Time.fixedDeltaTime / GetSimulationSubSteps();
            float dtSquared = dt * dt;

            foreach (var beam in beams)
            {
                if (!beam.isActive || !beam.IsCrossBody || beam.isBroken)
                    continue;

                float forceMagnitude = Mathf.Abs(beam.lagrangeMultiplier) / dtSquared;

                if (beam.ShouldBreak(forceMagnitude, beam.strength))
                {
                    beam.isBroken = true;
                    beam.isActive = false;

                    Debug.Log($"[Breakage] Constraint broken! Force: {forceMagnitude:F2}N exceeded strength: {beam.strength:F2}N");
                }
            }
        }

        #endregion

        #region Position Update

        public void UpdateNodePositions(int nodeCount)
        {
            for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
            {
                if (nodeManager.Nodes[nodeIndex] == null) continue;

                // CHANGE: Use .position directly
                Vector3 currentWorldPos = nodeManager.Nodes[nodeIndex].position;
                Vector3 predictedWorldPos = nodeManager.PredictedPositions[nodeIndex];

                nodeManager.Nodes[nodeIndex].position = predictedWorldPos;
                nodeManager.PreviousPositions[nodeIndex] = currentWorldPos;
            }
        }

        #endregion

        #region Pressure Forces


        public Vector3 CalculateVolumeCenter()
        {
            if (nodeManager?.Nodes == null || nodeManager.Nodes.Count == 0)
                return Vector3.zero;

            Vector3 center = Vector3.zero;
            int validNodes = 0;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] != null)
                {
                    center += nodeManager.Nodes[i].position;
                    validNodes++;
                }
            }

            if (validNodes > 0)
                center /= validNodes;

            _cachedVolumeCenter = center;
            return center;
        }


        private void ApplyPressureForces(float dt)
        {
            if (internalPressure <= 0f || nodeManager.Nodes.Count == 0)
                return;

            Vector3 volumeCenter = CalculateVolumeCenter();

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i])
                    continue;

                ApplyPressureForceToNode(i, volumeCenter, dt);
            }
        }


        private void ApplyPressureForceToNode(int nodeIndex, Vector3 volumeCenter, float dt)
        {
            Vector3 nodeWorldPos = nodeManager.Nodes[nodeIndex].position;
            Vector3 direction = nodeWorldPos - volumeCenter;
            float distance = direction.magnitude;

            if (distance < PhysicsConstants.MIN_CENTER_DISTANCE)
                return;

            direction /= distance;

            float pressureForce = internalPressure * PhysicsConstants.PRESSURE_SCALE_FACTOR;
            float distanceFactor = 1f / (distance + PhysicsConstants.PRESSURE_DISTANCE_OFFSET);
            Vector3 force = direction * pressureForce * distanceFactor;

            ApplyWorldForceToNode(nodeIndex, force, null, dt);
        }

        #endregion

        #region Force Application

        public void ApplyForceToNode(int nodeIndex, Vector3 localForce)
        {
            if (!IsValidNodeForForce(nodeIndex))
                return;

            float mass = nodeMasses[nodeIndex];
            if (mass <= 0f)
                return;

            float simScale = GetSimulationTimeScale();
            float fullDt = Time.fixedDeltaTime * simScale;
            int subSteps = GetSimulationSubSteps();
            float effectiveDt = fullDt / Mathf.Max(1, subSteps);

            Vector3 worldForce = _owner.TransformDirection(localForce);
            ApplyWorldForceToNode(nodeIndex, worldForce, mass, effectiveDt);
        }


        public void ApplyWorldForceToNode(int nodeIndex, Vector3 worldForce,
                                         float? customMass = null, float? customDt = null)
        {
            if (!IsValidNodeForForce(nodeIndex))
                return;

            float mass = customMass ?? nodeMasses[nodeIndex];
            if (mass <= 0f)
                return;

            float dt = customDt ?? Time.fixedDeltaTime;
            Vector3 accel = worldForce / mass;

            Vector3 currentWorldPos = _owner.TransformPoint(
                nodeManager.Nodes[nodeIndex].localPosition);
            Vector3 currentVelocity = (currentWorldPos -
                nodeManager.PreviousPositions[nodeIndex]) / dt;
            Vector3 newVelocity = currentVelocity + accel * dt;

            nodeManager.PreviousPositions[nodeIndex] = currentWorldPos - newVelocity * dt;
        }


        private bool IsValidNodeForForce(int nodeIndex)
        {
            return nodeIndex >= 0 &&
                   nodeIndex < nodeMasses.Count &&
                   !nodeManager.IsPinned[nodeIndex];
        }


        public void ApplyImpulse(int nodeIndex, Vector3 targetPosition, float strength)
        {
            if (nodeIndex < 0 || nodeIndex >= nodeManager.Nodes.Count ||
                nodeMasses.Count <= nodeIndex ||
                nodeManager.Nodes[nodeIndex] == null ||
                nodeManager.IsPinned[nodeIndex])
                return;

            float simScale = GetSimulationTimeScale();
            float effectiveDt = Time.fixedDeltaTime * simScale;

            Vector3 currentPos = nodeManager.Nodes[nodeIndex].localPosition;
            Vector3 targetLocalPos = _owner.InverseTransformPoint(targetPosition);
            Vector3 delta = targetLocalPos - currentPos;
            Vector3 impulse = delta * strength * nodeMasses[nodeIndex] / effectiveDt;

            ApplyForceToNode(nodeIndex, impulse);
        }

        #endregion

        #region Node Search


        public int FindClosestNodeToRay(Ray ray, float maxDistance)
        {
            int closestNode = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null)
                    continue;

                Vector3 nodePos = nodeManager.Nodes[i].localPosition;
                float distance = Vector3.Cross(ray.direction, nodePos - ray.origin).magnitude;

                if (distance < nodeManager.GetNodeRadius() && distance < minDistance)
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

        #endregion

        #region Mesh Deformation


        public void DeformMesh(Transform transform)
        {
            if (transform == null)
                return;

            UpdateNodeRotations();
            meshDeformer.Deform(transform, nodeManager.Nodes,
                              nodeManager.InitialPositions,
                              nodeManager.InitialRotations);
        }

        #endregion

        #region Node Rotation


        private void UpdateNodeRotations()
        {
            int nodeCount = nodeManager.Nodes.Count;
            List<List<int>> neighbors = BuildNeighborLists(nodeCount);

            for (int i = 0; i < nodeCount; i++)
            {
                UpdateSingleNodeRotation(i, neighbors[i]);
            }
        }


        private List<List<int>> BuildNeighborLists(int nodeCount)
        {
            List<List<int>> neighbors = new List<List<int>>(nodeCount);

            for (int i = 0; i < nodeCount; i++)
            {
                neighbors.Add(new List<int>());
            }

            foreach (var beam in beams)
            {
                if (beam.IsCrossBody) continue;
                if (beam.nodeA < 0 || beam.nodeA >= nodeCount ||
                    beam.nodeB < 0 || beam.nodeB >= nodeCount)
                    continue;

                neighbors[beam.nodeA].Add(beam.nodeB);
                neighbors[beam.nodeB].Add(beam.nodeA);
            }

            return neighbors;
        }

        private void UpdateSingleNodeRotation(int nodeIndex, List<int> neighborIndices)
        {
            List<int> cluster = new List<int>(neighborIndices) { nodeIndex };

            if (cluster.Count < 2)
                return;

            CalculateCentersOfMass(cluster, out Vector3 comRest,
                                  out Vector3 comCurrent, out float totalMass);

            if (totalMass < PhysicsConstants.MIN_TOTAL_MASS)
                return;

            comRest /= totalMass;
            comCurrent /= totalMass;

            Matrix4x4 covarianceMatrix = CalculateCovarianceMatrix(cluster, comRest,
                                                                   comCurrent);

            Quaternion rotation = ExtractRotationFromCovariance(nodeIndex, covarianceMatrix);

            nodeManager.Nodes[nodeIndex].localRotation = rotation;
        }
        private void CalculateCentersOfMass(List<int> cluster, out Vector3 comRest,
                                           out Vector3 comCurrent, out float totalMass)
        {
            comRest = Vector3.zero;
            comCurrent = Vector3.zero;
            totalMass = 0f;

            foreach (int idx in cluster)
            {
                float m = nodeMasses[idx];
                comRest += m * nodeManager.InitialPositions[idx];
                comCurrent += m * nodeManager.Nodes[idx].localPosition;
                totalMass += m;
            }
        }
        private Matrix4x4 CalculateCovarianceMatrix(List<int> cluster, Vector3 comRest,
                                                   Vector3 comCurrent)
        {
            Matrix4x4 A = Matrix4x4.zero;

            foreach (int idx in cluster)
            {
                Vector3 pRest = nodeManager.InitialPositions[idx] - comRest;
                Vector3 pCurrent = nodeManager.Nodes[idx].localPosition - comCurrent;
                float m = nodeMasses[idx];

                A[0, 0] += m * pCurrent.x * pRest.x;
                A[0, 1] += m * pCurrent.x * pRest.y;
                A[0, 2] += m * pCurrent.x * pRest.z;
                A[1, 0] += m * pCurrent.y * pRest.x;
                A[1, 1] += m * pCurrent.y * pRest.y;
                A[1, 2] += m * pCurrent.y * pRest.z;
                A[2, 0] += m * pCurrent.z * pRest.x;
                A[2, 1] += m * pCurrent.z * pRest.y;
                A[2, 2] += m * pCurrent.z * pRest.z;
            }

            return A;
        }

        private Quaternion ExtractRotationFromCovariance(int nodeIndex, Matrix4x4 A)
        {
            Quaternion q = nodeManager.Nodes[nodeIndex].localRotation;
            Matrix4x4 R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);

            for (int iter = 0; iter < PhysicsConstants.MAX_ROTATION_ITERATIONS; iter++)
            {
                Vector3 tau = CalculateRotationAxisAngle(R, A);
                float denom = Mathf.Abs(CalculateRotationDenominator(R, A)) +
                             PhysicsConstants.ROTATION_CONVERGENCE_EPSILON;
                Vector3 omega = tau / denom;
                float w = omega.magnitude;

                if (w < PhysicsConstants.MIN_OMEGA_MAGNITUDE)
                    break;

                Vector3 axis = omega / w;
                Quaternion deltaQ = Quaternion.AngleAxis(w * Mathf.Rad2Deg, axis);
                q = deltaQ * q;
                q = q.normalized;
                R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);
            }

            return q;
        }
        private Vector3 CalculateRotationAxisAngle(Matrix4x4 R, Matrix4x4 A)
        {
            Vector3 r0 = R.GetColumn(0);
            Vector3 r1 = R.GetColumn(1);
            Vector3 r2 = R.GetColumn(2);
            Vector3 a0 = A.GetColumn(0);
            Vector3 a1 = A.GetColumn(1);
            Vector3 a2 = A.GetColumn(2);

            return Vector3.Cross(r0, a0) + Vector3.Cross(r1, a1) + Vector3.Cross(r2, a2);
        }
        private float CalculateRotationDenominator(Matrix4x4 R, Matrix4x4 A)
        {
            Vector3 r0 = R.GetColumn(0);
            Vector3 r1 = R.GetColumn(1);
            Vector3 r2 = R.GetColumn(2);
            Vector3 a0 = A.GetColumn(0);
            Vector3 a1 = A.GetColumn(1);
            Vector3 a2 = A.GetColumn(2);

            return Vector3.Dot(r0, a0) + Vector3.Dot(r1, a1) + Vector3.Dot(r2, a2);
        }

        #endregion

        #region Node Restoration

        public void RestoreNodesAndBeams(Transform transform)
        {
            if (transform == null)
                return;

            var currentBeams = new List<Beam>(beams);
            var currentInitialPositions = new List<Vector3>(nodeManager.InitialPositions);

            nodeManager.Clear();
            beams.Clear();

            RestoreNodes(currentInitialPositions, transform);
            RestoreBeams(currentBeams);

            Debug.Log($"[Solver] Restored {nodeManager.Nodes.Count} nodes, {beams.Count} beams");
        }
        private void RestoreNodes(List<Vector3> initialPositions, Transform transform)
        {
            for (int i = 0; i < initialPositions.Count; i++)
            {
                GameObject nodeObj = new GameObject($"Node_{i}");


                nodeObj.transform.position = transform.TransformPoint(initialPositions[i]);
                nodeObj.hideFlags = HideFlags.HideAndDontSave;

                nodeManager.AddNode(nodeObj.transform, initialPositions[i]);
            }
        }
        private void RestoreBeams(List<Beam> savedBeams)
        {
            foreach (var beam in savedBeams)
            {
                if (IsRestoredBeamValid(beam))
                {
                    Vector3 localPosA = nodeManager.Nodes[beam.nodeA].localPosition;
                    Vector3 localPosB = nodeManager.Nodes[beam.nodeB].localPosition;
                    float distance = Vector3.Distance(localPosA, localPosB);

                    beams.Add(new Beam(beam.nodeA, beam.nodeB, beam.compliance,
                                     beam.damping, distance));
                }
            }
        }
        private bool IsRestoredBeamValid(Beam beam)
        {
            return beam.nodeA < nodeManager.Nodes.Count &&
                   beam.nodeB < nodeManager.Nodes.Count &&
                   nodeManager.Nodes[beam.nodeA] != null &&
                   nodeManager.Nodes[beam.nodeB] != null &&
                   beam.restLength > PhysicsConstants.MIN_BEAM_LENGTH;
        }

        #endregion

        #region Job Structs

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
            public float minLength;
            public float maxLength;
            public bool isActive;
        }

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
        private struct SolveDistanceConstraintsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<BeamData> beams;
            [ReadOnly] public NativeArray<float3> previousPositions;
            [ReadOnly] public NativeArray<bool> isPinned;
            [ReadOnly] public NativeArray<float> nodeMasses;
            [ReadOnly] public float dt;
            [ReadOnly] public float worldRestLengthScale;

            [NativeDisableParallelForRestriction]
            public NativeArray<float3> predictedPositions;

            [NativeDisableParallelForRestriction]
            public NativeArray<float> beamRestLengths;

            [NativeDisableParallelForRestriction]
            public NativeArray<float> beamLagrangeMultipliers;

            public void Execute(int beamIndex)
            {
                BeamData beam = beams[beamIndex];
                if (!beam.isActive) return;

                int nodeA = beam.nodeA;
                int nodeB = beam.nodeB;

                float3 posA = predictedPositions[nodeA];
                float3 posB = predictedPositions[nodeB];

                float3 delta = posA - posB;
                float currentLength = math.length(delta);

                if (currentLength < 1e-7f) return;

                float restLength = beamRestLengths[beamIndex] * worldRestLengthScale;
                float constraint = currentLength - restLength;


                float wA = isPinned[nodeA] ? 0f : math.rcp(nodeMasses[nodeA]);
                float wB = isPinned[nodeB] ? 0f : math.rcp(nodeMasses[nodeB]);
                float wSum = wA + wB;
                if (wSum <= 0f) return;

                float invDtSq = 1f / (dt * dt);
                float alphaTilde = beam.compliance * invDtSq;

                float currentLambda = beamLagrangeMultipliers[beamIndex];
                float deltaLambda = -(constraint + alphaTilde * currentLambda) / (wSum + alphaTilde);

                beamLagrangeMultipliers[beamIndex] = currentLambda + deltaLambda;

                float3 gradient = delta / currentLength;
                float3 correction = deltaLambda * gradient;

                if (!isPinned[nodeA])
                    predictedPositions[nodeA] += wA * correction;
                if (!isPinned[nodeB])
                    predictedPositions[nodeB] -= wB * correction;
            }
        }

        #endregion

        #region Cleanup
        private void DestroyOldNodes(Transform parent)
        {
            _reusableDestroyList.Clear();

            foreach (Transform child in parent)
            {
                if (child.name.StartsWith("Node_"))
                    _reusableDestroyList.Add(child.gameObject);
            }

            foreach (var go in _reusableDestroyList)
            {
                if (Application.isEditor && !Application.isPlaying)
                    Object.DestroyImmediate(go);
                else
                    Object.Destroy(go);
            }
        }
        void OnDestroy()
        {
            Cleanup();
        }

        public void Cleanup()
        {
            nodeManager?.DisposeNativeArrays();

            if (_reusableWorldPositions.IsCreated)
                _reusableWorldPositions.Dispose();
        }

        #endregion
    }
}