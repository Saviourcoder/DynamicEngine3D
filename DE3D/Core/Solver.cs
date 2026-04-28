using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Profiling;

namespace DynamicEngine
{
    public class Solver : System.IDisposable
    {
        private readonly Transform _owner;
        public readonly NodeManager nodeManager;
        public List<Beam> beams;
        public readonly MeshDeformer meshDeformer;
        public CollisionHandler collisionHandler;

        public List<float> nodeMasses = new List<float>();

        private Truss _trussAsset;
        private Matter _matterAsset;

        private NativeArray<float3> _reusableWorldPositions;

        private NativeArray<BeamData> _beamDataNative;
        private NativeArray<float> _beamRestLengthsNative;
        private NativeArray<float> _beamLagrangeMultipliersNative;
        private NativeArray<float> _nodeMassesNative;
        private int _lastBeamCount = -1;

        private bool _disposed = false;

        private readonly List<GameObject> _reusableDestroyList = new List<GameObject>();

        public bool visualizeForces { get; set; } = false;
        public float compliance { get; set; } = 1e-3f;
        public float defaultDamping { get; set; } = 0.3f;
        public float maxDeformation { get; set; } = 1f;
        public float internalPressure { get; set; } = 0f;
        public float velocityDamping { get; set; } = 0.1f;
        private Vector3 _cachedVolumeCenter = Vector3.zero;
        private float perpDistance;

        private int WorkerThreads => SceneSettings.Instance != null ? Mathf.Max(1, SceneSettings.Instance.WorkerThreads) : 1;

        private NativeArray<int> _nbrOffsets;
        private NativeArray<int> _nbrData;
        private NativeArray<float3> _nbrInitialPos;
        private NativeArray<float> _nbrMasses;
        private NativeArray<float3> _nbrCurrentLocal;
        private NativeArray<quaternion> _nbrRotOut;
        private bool _nbrDirty = true;
        private int _nbrCachedCount = -1;

        public Solver(float influenceRadius, Mesh mesh, float3[] originalVertices, Transform owner)
        {
            beams = new List<Beam>();
            meshDeformer = new MeshDeformer(mesh, originalVertices, influenceRadius);
            collisionHandler = new CollisionHandler();
            _owner = owner;
            nodeManager = new NodeManager();
            nodeMasses = new List<float>();
        }

        public void Dispose()
        {
            if (_disposed) return;
            Cleanup();
            _disposed = true;
        }

        public void MarkNeighborsDirty() => _nbrDirty = true;

        public void InitializeFromTruss(Truss truss)
        {
            _trussAsset = truss;
            if (truss != null)
            {
                nodeMasses = new List<float>(truss.NodeMasses);
                while (nodeMasses.Count < nodeManager.NodeCount)
                    nodeMasses.Add(PhysicsConstants.DEFAULT_NODE_MASS);
                if (nodeMasses.Count > nodeManager.NodeCount)
                    nodeMasses.RemoveRange(nodeManager.NodeCount, nodeMasses.Count - nodeManager.NodeCount);
                compliance = truss.compliance;
                defaultDamping = truss.DefaultBeamDamping;
            }
            else
            {
                for (int i = 0; i < nodeManager.NodeCount; i++)
                    nodeMasses.Add(PhysicsConstants.DEFAULT_NODE_MASS);
            }
        }

        public void SetTrussAsset(Truss truss) => _trussAsset = truss;
        public void SetMatterAsset(Matter matter) => _matterAsset = matter;

        public void GenerateNodesAndBeams(Vector3[] positions, float connectionDistance = -1f, Beam[] beamsArray = null, Transform parent = null)
        {
            if (positions == null || positions.Length == 0 || (connectionDistance <= 0f && beamsArray == null) || parent == null)
            {
                Debug.LogWarning("[Solver] Invalid generation parameters.");
                return;
            }

            ClearExistingNodes(parent);
            GenerateNodes(positions, parent);

            if (beamsArray != null)
                GenerateBeamsFromArray(beamsArray, positions);
            else
                GenerateBeamsFromDistance(positions, connectionDistance);

            meshDeformer.MapVerticesToNodes(parent, nodeManager.CurrentPositions, nodeManager.InitialPositions);
        }

        private void ClearExistingNodes(Transform parent)
        {
            nodeManager.Clear();
            beams.Clear();
            nodeMasses.Clear();
            DisposeBeamNativeArrays();
            DestroyOldNodes(parent);
            MarkNeighborsDirty();
        }

        private void GenerateNodes(Vector3[] positions, Transform parent)
        {
            for (int i = 0; i < positions.Length; i++)
            {
                Vector3 worldPos = parent.TransformPoint(positions[i]);
                nodeManager.AddNode(positions[i], worldPos, Quaternion.identity);
                float mass = (_trussAsset != null && i < _trussAsset.NodeMasses.Count) ? _trussAsset.NodeMasses[i] : PhysicsConstants.DEFAULT_NODE_MASS;
                nodeMasses.Add(mass);
            }
            nodeManager.AllocateNativeArrays(); // Call once after bulk insert
        }

        private void GenerateBeamsFromArray(Beam[] beamsArray, Vector3[] positions)
        {
            foreach (var b in beamsArray)
            {
                if (b.nodeA >= 0 && b.nodeA < positions.Length && b.nodeB >= 0 && b.nodeB < positions.Length && b.restLength > PhysicsConstants.MIN_BEAM_LENGTH)
                {
                    beams.Add(new Beam(b.nodeA, b.nodeB, b.compliance, b.damping, b.restLength, b.maxDeformation, b.plasticityThreshold, b.plasticityRate));
                }
            }
            MarkNeighborsDirty();
        }

        private void GenerateBeamsFromDistance(Vector3[] positions, float connectionDistance)
        {
            for (int i = 0; i < positions.Length; i++)
            {
                for (int j = i + 1; j < positions.Length; j++)
                {
                    float distance = Vector3.Distance(positions[i], positions[j]);
                    if (distance <= connectionDistance && distance > PhysicsConstants.MIN_BEAM_LENGTH)
                    {
                        beams.Add(new Beam(i, j, compliance, defaultDamping, distance));
                    }
                }
            }
            MarkNeighborsDirty();
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

            float connectionDistance = Vector3.Distance(min, max) * PhysicsConstants.CUBE_CONNECTION_DISTANCE_MULTIPLIER;
            GenerateNodesAndBeams(cubeNodes, connectionDistance, parent: transform);
        }

        public Vector3 GetNodeWorldPosition(int index)
        {
            if (index < 0 || index >= nodeManager.NodeCount) return Vector3.zero;
            return nodeManager.GetPosition(index);
        }

        internal void Integrate(float dt)
        {
            int nodeCount = nodeManager.NodeCount;

            // Fetch gravity dynamically from SceneSettings and apply it downwards
            float gravityMag = SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : 9.81f;
            Vector3 worldGravity = Vector3.down * gravityMag;

            nodeManager.AllocateNativeArrays();
            nodeManager.SyncToNativeArrays();
            EnsureReusableWorldPositionsBuffer(nodeCount);

            for (int i = 0; i < nodeCount; i++)
                _reusableWorldPositions[i] = (float3)nodeManager.GetPosition(i);

            float globalDamping = SceneSettings.Instance != null ? SceneSettings.Instance.GlobalDamping : 0f;
            float dampingFactor = Mathf.Exp(-(velocityDamping + globalDamping) * dt);

            var job = new VerletJob
            {
                currentWorldPositions = _reusableWorldPositions,
                previousPositions = nodeManager.nativePreviousPositions,
                isPinned = nodeManager.nativeIsPinned,
                dt = dt,
                dampingFactor = dampingFactor,
                worldGravity = worldGravity,
                dtSquared = dt * dt,
                predictedPositions = nodeManager.nativePredictedPositions
            };

            job.Schedule(nodeCount, 64).Complete();
            nodeManager.SyncFromNativeArrays();

            for (int i = 0; i < nodeCount; i++)
                nodeManager.PostIntegrationPositions[i] = nodeManager.PredictedPositions[i];
        }


        private void EnsureReusableWorldPositionsBuffer(int nodeCount)
        {
            if (_reusableWorldPositions.IsCreated && _reusableWorldPositions.Length == nodeCount) return;
            if (_reusableWorldPositions.IsCreated) _reusableWorldPositions.Dispose();
            _reusableWorldPositions = new NativeArray<float3>(nodeCount, Allocator.Persistent);
        }

        internal void ApplyBeamDampingForces(float dt)
        {
            for (int i = 0; i < beams.Count; i++)
            {
                Beam b = beams[i];
                if (!b.isActive || b.damping <= 0) continue;

                // Ensure we are working with float3 for math consistency
                float3 posA = nodeManager.PredictedPositions[b.nodeA];
                float3 posB = nodeManager.PredictedPositions[b.nodeB];

                // Calculate velocity using the positions stored in nodeManager
                float3 velA = (posA - (float3)nodeManager.PreviousPositions[b.nodeA]) / dt;
                float3 velB = (posB - (float3)nodeManager.PreviousPositions[b.nodeB]) / dt;

                float3 relVel = velA - velB;
                float3 delta = posA - posB;
                float dist = math.length(delta);

                if (dist < 1e-7f) continue;

                float3 dir = delta / dist;

                // Calculate 1D damping force along the beam axis
                float dampingForce = math.dot(relVel, dir) * b.damping;
                float3 impulse = dir * dampingForce * dt;

                if (!nodeManager.IsPinned[b.nodeA])
                    nodeManager.PredictedPositions[b.nodeA] -= (Vector3)(impulse / nodeMasses[b.nodeA]);

                if (!nodeManager.IsPinned[b.nodeB])
                    nodeManager.PredictedPositions[b.nodeB] += (Vector3)(impulse / nodeMasses[b.nodeB]);
            }
        }

        internal void SolveConstraints(float dt)
        {
            int nodeCount = nodeManager.NodeCount;
            if (nodeCount == 0 || beams.Count == 0) return;

            int constraintIters = SceneSettings.Instance != null
                ? Mathf.Max(1, SceneSettings.Instance.ConstraintIterations)
                : 20;
            float worldRestLengthScale = _owner.TransformVector(Vector3.right).magnitude;

            ResetLagrangeMultipliers();

            // Prepare native beam/mass arrays once before the master loop.
            EnsureBeamNativeArrays();
            SyncBeamDataToNative();
            SyncNodeMassesToNative();

            float invDtSq = 1f / (dt * dt);

            for (int iter = 0; iter < constraintIters; iter++)
            {
                // ── Cross-Body step (one pass, reads/writes managed PredictedPositions) ──
                SolveCrossBodyStep(dt);

                // Propagate cross-body corrections into the native array so the
                // beam job sees them this iteration.
                nodeManager.SyncToNativeArrays();

                // ── Beam step (one pass, reads/writes native PredictedPositions) ──
                new ConstraintsJob
                {
                    allBeamData = _beamDataNative,
                    beamRestLengths = _beamRestLengthsNative,
                    nodeMasses = _nodeMassesNative,
                    isPinned = nodeManager.nativeIsPinned,
                    predictedPositions = nodeManager.nativePredictedPositions,
                    beamLagrangeMultipliers = _beamLagrangeMultipliersNative,
                    invDtSq = invDtSq,
                    worldRestLengthScale = worldRestLengthScale,
                }.Schedule().Complete();

                // Propagate beam corrections back to managed so the next
                // cross-body step sees them.
                nodeManager.SyncFromNativeArrays();
            }

            CopyLagrangeBack();
        }

        private void ResetLagrangeMultipliers()
        {
            for (int i = 0; i < beams.Count; i++)
                beams[i].lagrangeMultiplier = 0f;
        }

        private void SyncBeamDataToNative()
        {
            for (int i = 0; i < beams.Count; i++)
            {
                Beam b = beams[i];
                var data = _beamDataNative[i];
                data.isActive = b.isActive;
                data.damping = b.damping;
                data.compliance = b.compliance;
                _beamDataNative[i] = data;
                _beamRestLengthsNative[i] = b.restLength;
                _beamLagrangeMultipliersNative[i] = 0f;
            }
        }

        private void SyncNodeMassesToNative()
        {
            int count = nodeMasses.Count;
            if (!_nodeMassesNative.IsCreated || _nodeMassesNative.Length != count)
            {
                if (_nodeMassesNative.IsCreated) _nodeMassesNative.Dispose();
                _nodeMassesNative = new NativeArray<float>(count, Allocator.Persistent);
            }
            for (int i = 0; i < count; i++)
                _nodeMassesNative[i] = nodeMasses[i];
        }

        private void CopyLagrangeBack()
        {
            for (int i = 0; i < beams.Count; i++)
                beams[i].lagrangeMultiplier = _beamLagrangeMultipliersNative[i];
        }

        private void SolveCrossBodyStep(float dt)
        {
            foreach (var beam in beams)
            {
                if (!beam.isActive || !beam.IsCrossBody) continue;
                if (!beam.AreNodesValid()) continue;
                if (beam.bodyA == null || beam.bodyA.solver != this) continue;

                if (beam.isEdgeSliding)
                    SolveEdgeSliding(beam, dt);
                else
                    SolveCrossBodyBeam(beam, dt);
            }
        }

        private void SolveCrossBodyBeam(Beam beam, float dt)
        {
            Vector3 posA = beam.GetNodeALocalPosition();
            Vector3 posB = beam.GetNodeBLocalPosition();
            Vector3 delta = posB - posA;
            float currentLength = delta.magnitude;

            if (currentLength < PhysicsConstants.MIN_BEAM_LENGTH) return;

            Vector3 gradient = delta / currentLength;

            bool hasMinLimit = beam.minLength > 0f;
            bool hasMaxLimit = !float.IsPositiveInfinity(beam.maxLength);

            float targetLength = beam.restLength;

            if (hasMinLimit && currentLength < beam.minLength)
            {
                targetLength = beam.minLength;
            }
            else if (hasMaxLimit && currentLength > beam.maxLength)
            {
                targetLength = beam.maxLength;
            }
            else if (hasMinLimit || hasMaxLimit)
            {
                // We are safely inside the min/max radius. 
                // Return immediately to let it stretch freely!
                return;
            }

            float constraint = currentLength - targetLength;
            float dampedCompliance = beam.compliance;

            bool pinnedA = beam.IsNodeAPinned();
            bool pinnedB = beam.IsNodeBPinned();
            float wA = pinnedA ? 0f : 1f / beam.bodyA.solver.nodeMasses[beam.nodeA];
            float wB = pinnedB ? 0f : 1f / beam.bodyB.solver.nodeMasses[beam.nodeB];
            float wSum = wA + wB;

            if (wSum <= 0f) return;

            float effectiveCompliance = dampedCompliance / (dt * dt);
            float deltaLambda = -constraint / (wSum + effectiveCompliance);

            if (!float.IsFinite(deltaLambda)) return;

            float maxCorrection = Mathf.Max(0.1f, perpDistance * 0.5f);
            if (Mathf.Abs(deltaLambda) > maxCorrection)
            {
                deltaLambda = Mathf.Sign(deltaLambda) * maxCorrection;
            }

            if (hasMinLimit && !hasMaxLimit)
            {
                float newLambda = beam.lagrangeMultiplier + deltaLambda;
                if (newLambda < 0f) deltaLambda = -beam.lagrangeMultiplier;
            }

            beam.lagrangeMultiplier += deltaLambda;

            Vector3 correction = gradient * deltaLambda;
            if (correction.magnitude > PhysicsConstants.MAX_ALLOWED_DISPLACEMENT)
                correction = correction.normalized * PhysicsConstants.MAX_ALLOWED_DISPLACEMENT;

            if (!pinnedA) beam.bodyA.solver.nodeManager.PredictedPositions[beam.nodeA] -= wA * correction;
            if (!pinnedB) beam.bodyB.solver.nodeManager.PredictedPositions[beam.nodeB] += wB * correction;
        }

        private void SolveEdgeSliding(Beam beam, float dt)
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

            if (edgeLength < PhysicsConstants.MIN_BEAM_LENGTH) return;

            Vector3 edgeDir = edgeVector / edgeLength;
            Vector3 nodeToEdgeStart = nodePos - edgePosA;
            float t = Vector3.Dot(nodeToEdgeStart, edgeDir);

            Vector3 closestPoint = edgePosA + edgeDir * t;
            Vector3 perpVector = nodePos - closestPoint;
            float perpDistance = perpVector.magnitude;

            if (perpDistance < PhysicsConstants.MIN_BEAM_LENGTH) return;

            Vector3 perpDir = perpVector / perpDistance;

            float targetPerpDistance = beam.targetPerpDistance;
            float constraint = perpDistance - targetPerpDistance;

            float dampedCompliance = beam.compliance + (beam.damping + PhysicsConstants.CROSS_BODY_CONSTRAINT_DAMPING) * dt;

            bool pinnedNode = slidingBody.solver.nodeManager.IsPinned[slidingNode];
            bool pinnedEdgeA = edgeBody.solver.nodeManager.IsPinned[edgeNodeA];
            bool pinnedEdgeB = edgeBody.solver.nodeManager.IsPinned[edgeNodeB];

            if (pinnedNode && pinnedEdgeA && pinnedEdgeB) return;

            float massNode = slidingBody.solver.nodeMasses[slidingNode];
            float massEdgeA = edgeBody.solver.nodeMasses[edgeNodeA];
            float massEdgeB = edgeBody.solver.nodeMasses[edgeNodeB];

            float weightA = 1.0f - (t / edgeLength);
            float weightB = t / edgeLength;

            float wNode = pinnedNode ? 0f : 1f / massNode;
            float wEdgeA = pinnedEdgeA ? 0f : (1f / massEdgeA) * weightA * weightA;
            float wEdgeB = pinnedEdgeB ? 0f : (1f / massEdgeB) * weightB * weightB;
            float wSum = wNode + wEdgeA + wEdgeB;

            if (wSum <= 0f) return;

            float effectiveCompliance = dampedCompliance / (dt * dt);
            float deltaLambda = -constraint / (wSum + effectiveCompliance);

            if (!float.IsFinite(deltaLambda)) return;

            beam.lagrangeMultiplier += deltaLambda;

            float maxCorrection = Mathf.Max(0.1f, perpDistance * 0.5f);
            if (Mathf.Abs(deltaLambda) > maxCorrection)
                deltaLambda = Mathf.Sign(deltaLambda) * maxCorrection;

            if (!pinnedNode)
                slidingBody.solver.nodeManager.PredictedPositions[slidingNode] += (1f / massNode) * deltaLambda * perpDir;

            if (!pinnedEdgeA)
                edgeBody.solver.nodeManager.PredictedPositions[edgeNodeA] += (1f / massEdgeA) * deltaLambda * (-perpDir * weightA);

            if (!pinnedEdgeB)
                edgeBody.solver.nodeManager.PredictedPositions[edgeNodeB] += (1f / massEdgeB) * deltaLambda * (-perpDir * weightB);
        }

        internal void CheckAndBreakConstraints(float dt)
        {
            float dtSquared = dt * dt;

            foreach (var beam in beams)
            {
                if (!beam.isActive || !beam.IsCrossBody || beam.isBroken) continue;

                float forceMagnitude = Mathf.Abs(beam.lagrangeMultiplier) / dtSquared;

                if (beam.ShouldBreak(forceMagnitude, beam.strength))
                {
                    beam.isBroken = true;
                    beam.isActive = false;
                }
            }
        }

        internal void ResolveCollisions(float dt)
        {
            collisionHandler.ResolveCollisions(nodeManager, _owner, _matterAsset, nodeMasses, dt, PhysicsConstants.DEFAULT_COLLISION_EPSILON, visualizeForces);
        }

        internal void FinalizePositions()
        {
            for (int i = 0; i < nodeManager.NodeCount; i++)
            {
                nodeManager.SetPreviousPosition(i, nodeManager.CurrentPositions[i]);
                nodeManager.SetCurrentPosition(i, nodeManager.PredictedPositions[i]);
            }
        }

        internal void ApplyPressure(float dt)
        {
            if (internalPressure <= 0f || nodeManager.NodeCount == 0) return;

            Vector3 volumeCenter = CalculateVolumeCenter();

            int activeNodes = 0;
            for (int i = 0; i < nodeManager.NodeCount; i++)
                if (!nodeManager.IsPinned[i]) activeNodes++;

            if (activeNodes == 0) return;

            float forcePerNode = internalPressure * PhysicsConstants.PRESSURE_SCALE_FACTOR / activeNodes;

            for (int i = 0; i < nodeManager.NodeCount; i++)
            {
                if (nodeManager.IsPinned[i]) continue;
                ApplyPressureToNode(i, volumeCenter, forcePerNode, dt);
            }
        }

        private void ApplyPressureToNode(int nodeIndex, Vector3 volumeCenter, float forcePerNode, float dt)
        {
            Vector3 nodeWorldPos = nodeManager.GetPosition(nodeIndex);
            Vector3 direction = nodeWorldPos - volumeCenter;
            float distance = direction.magnitude;

            if (distance < PhysicsConstants.MIN_CENTER_DISTANCE) return;

            direction /= distance;

            float mass = nodeMasses.Count > nodeIndex ? nodeMasses[nodeIndex] : PhysicsConstants.DEFAULT_NODE_MASS;
            if (mass <= 0f) return;

            Vector3 acceleration = direction * forcePerNode / mass;
            nodeManager.PreviousPositions[nodeIndex] -= acceleration * dt * dt;
        }

        private Vector3 CalculateVolumeCenter()
        {
            if (nodeManager.NodeCount == 0) return Vector3.zero;

            Vector3 center = Vector3.zero;
            int validNodes = 0;

            for (int i = 0; i < nodeManager.NodeCount; i++)
            {
                center += nodeManager.GetPosition(i);
                validNodes++;
            }

            if (validNodes > 0) center /= validNodes;
            _cachedVolumeCenter = center;
            return center;
        }

        public void ApplyForceToNode(int nodeIndex, Vector3 localForce)
        {
            if (!IsValidNodeForForce(nodeIndex)) return;

            float mass = nodeMasses[nodeIndex];
            if (mass <= 0f) return;

            float simScale = SceneSettings.Instance != null ? SceneSettings.Instance.TimeScale : 1.0f;
            float fullDt = Time.fixedDeltaTime * simScale;
            int subSteps = SceneSettings.Instance != null ? SceneSettings.Instance.SubstepCount : 1;
            float effectiveDt = fullDt / Mathf.Max(1, subSteps);

            Vector3 worldForce = _owner.TransformDirection(localForce);
            ApplyWorldForceToNode(nodeIndex, worldForce, mass, effectiveDt);
        }

        public void ApplyWorldForceToNode(int nodeIndex, Vector3 worldForce, float? customMass = null, float? customDt = null)
        {
            if (!IsValidNodeForForce(nodeIndex)) return;

            float mass = customMass ?? nodeMasses[nodeIndex];
            if (mass <= 0f) return;

            float dt = customDt ?? Time.fixedDeltaTime;
            Vector3 accel = worldForce / mass;

            Vector3 currentWorldPos = nodeManager.GetPosition(nodeIndex);
            Vector3 currentVelocity = (currentWorldPos - nodeManager.PreviousPositions[nodeIndex]) / dt;
            Vector3 newVelocity = currentVelocity + accel * dt;

            nodeManager.PreviousPositions[nodeIndex] = currentWorldPos - newVelocity * dt;
        }

        private bool IsValidNodeForForce(int nodeIndex)
        {
            return nodeIndex >= 0 && nodeIndex < nodeMasses.Count && !nodeManager.IsPinned[nodeIndex];
        }

        public void ApplyImpulse(int nodeIndex, Vector3 targetPosition, float strength)
        {
            if (nodeIndex < 0 || nodeIndex >= nodeManager.NodeCount || nodeMasses.Count <= nodeIndex || nodeManager.IsPinned[nodeIndex]) return;

            float simScale = SceneSettings.Instance != null ? SceneSettings.Instance.TimeScale : 1.0f;
            float effectiveDt = Time.fixedDeltaTime * simScale;
            Vector3 currentPos = nodeManager.GetPosition(nodeIndex);
            Vector3 targetLocal = _owner.InverseTransformPoint(targetPosition);
            Vector3 delta = targetLocal - currentPos;
            Vector3 impulse = delta * strength * nodeMasses[nodeIndex] / effectiveDt;

            ApplyForceToNode(nodeIndex, impulse);
        }

        public int FindClosestNodeToRay(Ray ray, float maxDistance)
        {
            int closestNode = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < nodeManager.NodeCount; i++)
            {
                Vector3 nodePos = nodeManager.GetPosition(i);
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

        public void DeformMesh(Transform transform)
        {
            if (transform == null) return;

            UpdateNodeRotations();
            meshDeformer.Deform(transform, nodeManager.CurrentPositions, nodeManager.CurrentRotations, nodeManager.InitialPositions, nodeManager.InitialRotations);
        }

        private List<List<int>> BuildNeighborLists(int nodeCount)
        {
            var neighbors = new List<List<int>>(nodeCount);
            for (int i = 0; i < nodeCount; i++) neighbors.Add(new List<int>());

            foreach (var beam in beams)
            {
                if (beam.IsCrossBody || !beam.isActive) continue;
                neighbors[beam.nodeA].Add(beam.nodeB);
                neighbors[beam.nodeB].Add(beam.nodeA);
            }
            return neighbors;
        }

        private void UpdateSingleNodeRotation(int nodeIndex, List<int> neighborIndices)
        {
            var cluster = new List<int>(neighborIndices) { nodeIndex };
            if (cluster.Count < 2) return;

            CalculateCentersOfMass(cluster, out Vector3 comRest, out Vector3 comCurrent, out float totalMass);

            if (totalMass < PhysicsConstants.MIN_TOTAL_MASS) return;

            comRest /= totalMass;
            comCurrent /= totalMass;

            Matrix4x4 covarianceMatrix = CalculateCovarianceMatrix(cluster, comRest, comCurrent);
            Quaternion rotation = ExtractRotationFromCovariance(nodeIndex, covarianceMatrix);

            nodeManager.CurrentRotations[nodeIndex] = rotation;
        }

        private void CalculateCentersOfMass(List<int> cluster, out Vector3 comRest, out Vector3 comCurrent, out float totalMass)
        {
            comRest = Vector3.zero;
            comCurrent = Vector3.zero;
            totalMass = 0f;

            foreach (int idx in cluster)
            {
                float m = nodeMasses[idx];
                comRest += m * nodeManager.InitialPositions[idx];
                comCurrent += m * _owner.InverseTransformPoint(nodeManager.GetPosition(idx));
                totalMass += m;
            }
        }

        private Matrix4x4 CalculateCovarianceMatrix(List<int> cluster, Vector3 comRest, Vector3 comCurrent)
        {
            Matrix4x4 A = Matrix4x4.zero;

            foreach (int idx in cluster)
            {
                Vector3 pRest = nodeManager.InitialPositions[idx] - comRest;
                Vector3 pCurrent = _owner.InverseTransformPoint(nodeManager.GetPosition(idx)) - comCurrent;
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
            Quaternion q = nodeManager.CurrentRotations[nodeIndex];
            Matrix4x4 R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);

            for (int iter = 0; iter < PhysicsConstants.MAX_ROTATION_ITERATIONS; iter++)
            {
                Vector3 tau = CalculateRotationAxisAngle(R, A);
                float denom = Mathf.Abs(CalculateRotationDenominator(R, A)) + PhysicsConstants.ROTATION_CONVERGENCE_EPSILON;
                Vector3 omega = tau / denom;
                float w = omega.magnitude;

                if (w < PhysicsConstants.MIN_OMEGA_MAGNITUDE) break;

                Vector3 axis = omega / w;
                Quaternion deltaQ = Quaternion.AngleAxis(w * Mathf.Rad2Deg, axis);
                q = (deltaQ * q).normalized;
                R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);
            }

            return q;
        }

        private Vector3 CalculateRotationAxisAngle(Matrix4x4 R, Matrix4x4 A)
        {
            Vector3 r0 = R.GetColumn(0), r1 = R.GetColumn(1), r2 = R.GetColumn(2);
            Vector3 a0 = A.GetColumn(0), a1 = A.GetColumn(1), a2 = A.GetColumn(2);
            return Vector3.Cross(r0, a0) + Vector3.Cross(r1, a1) + Vector3.Cross(r2, a2);
        }

        private float CalculateRotationDenominator(Matrix4x4 R, Matrix4x4 A)
        {
            Vector3 r0 = R.GetColumn(0), r1 = R.GetColumn(1), r2 = R.GetColumn(2);
            Vector3 a0 = A.GetColumn(0), a1 = A.GetColumn(1), a2 = A.GetColumn(2);
            return Vector3.Dot(r0, a0) + Vector3.Dot(r1, a1) + Vector3.Dot(r2, a2);
        }

        public void RestoreNodesAndBeams(Transform transform)
        {
            if (transform == null) return;

            var currentBeams = new List<Beam>(beams);
            var currentInitialPositions = new List<Vector3>(nodeManager.InitialPositions);

            nodeManager.Clear();
            beams.Clear();

            RestoreNodes(currentInitialPositions, transform);
            RestoreBeams(currentBeams);
        }

        private void RestoreNodes(List<Vector3> initialPositions, Transform transform)
        {
            for (int i = 0; i < initialPositions.Count; i++)
            {
                Vector3 worldPos = transform.TransformPoint(initialPositions[i]);
                nodeManager.AddNode(initialPositions[i], worldPos, Quaternion.identity);
            }
            nodeManager.AllocateNativeArrays(); // Call once after bulk insert
        }

        private void RestoreBeams(List<Beam> savedBeams)
        {
            foreach (var beam in savedBeams)
            {
                if (beam.nodeA >= 0 && beam.nodeA < nodeManager.NodeCount && beam.nodeB >= 0 && beam.nodeB < nodeManager.NodeCount && beam.restLength > PhysicsConstants.MIN_BEAM_LENGTH)
                {
                    Vector3 localPosA = nodeManager.InitialPositions[beam.nodeA];
                    Vector3 localPosB = nodeManager.InitialPositions[beam.nodeB];
                    float distance = Vector3.Distance(localPosA, localPosB);
                    beams.Add(new Beam(beam.nodeA, beam.nodeB, beam.compliance, beam.damping, distance));
                }
            }
        }

        private void EnsureBeamNativeArrays()
        {
            if (_beamDataNative.IsCreated && _lastBeamCount == beams.Count) return;
            DisposeBeamNativeArrays();

            int count = beams.Count;
            _beamDataNative = new NativeArray<BeamData>(count, Allocator.Persistent);
            _beamRestLengthsNative = new NativeArray<float>(count, Allocator.Persistent);
            _beamLagrangeMultipliersNative = new NativeArray<float>(count, Allocator.Persistent);

            for (int i = 0; i < count; i++)
            {
                Beam b = beams[i];
                _beamDataNative[i] = new BeamData { nodeA = b.nodeA, nodeB = b.nodeB, compliance = b.compliance, isActive = b.isActive, isCrossBody = b.IsCrossBody };
                _beamRestLengthsNative[i] = b.restLength;
            }
            _lastBeamCount = count;
        }

        private void DisposeBeamNativeArrays()
        {
            if (_beamDataNative.IsCreated) _beamDataNative.Dispose();
            if (_beamRestLengthsNative.IsCreated) _beamRestLengthsNative.Dispose();
            if (_beamLagrangeMultipliersNative.IsCreated) _beamLagrangeMultipliersNative.Dispose();
            if (_nodeMassesNative.IsCreated) _nodeMassesNative.Dispose();
            _lastBeamCount = -1;
        }

        public void ApplyPlasticityStep(float dt)
        {
            float worldRestLengthScale = _owner.TransformVector(Vector3.right).magnitude;
            ApplyPlasticity(dt, worldRestLengthScale);
        }

        private void ApplyPlasticity(float dt, float worldRestLengthScale)
        {
            for (int beamIndex = 0; beamIndex < beams.Count; beamIndex++)
            {
                Beam beam = beams[beamIndex];
                if (!beam.isActive || beam.IsCrossBody) continue;

                Vector3 posA = nodeManager.PredictedPositions[beam.nodeA];
                Vector3 posB = nodeManager.PredictedPositions[beam.nodeB];
                float currentLength = Vector3.Distance(posA, posB);

                if (currentLength < PhysicsConstants.MIN_BEAM_LENGTH) continue;

                float restLength = beam.restLength;
                float worldRestLength = restLength * worldRestLengthScale;
                float constraint = currentLength - worldRestLength;
                float strain = constraint / worldRestLength;

                if (Mathf.Abs(strain) <= beam.plasticityThreshold) continue;

                // Calculate magnitude of excess strain and preserve the direction (sign)
                float excessStrain = (Mathf.Abs(strain) - beam.plasticityThreshold) * Mathf.Sign(strain);
                float plasticDeformation = excessStrain * beam.plasticityRate * dt;

                float lengthDiff = Mathf.Abs(restLength - beam.originalRestLength);
                float totalDeformation = lengthDiff / beam.originalRestLength;

                if (totalDeformation >= beam.maxDeformation) continue;

                // Update rest length based on the signed deformation
                restLength += plasticDeformation * restLength;

                float minLength = beam.originalRestLength * (1f - beam.maxDeformation);
                float maxLength = beam.originalRestLength * (1f + beam.maxDeformation);
                beam.restLength = Mathf.Clamp(restLength, minLength, maxLength);
            }
        }

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

        public void Cleanup()
        {
            nodeManager?.DisposeNativeArrays();

            if (_reusableWorldPositions.IsCreated)
                _reusableWorldPositions.Dispose();

            DisposeBeamNativeArrays();
            DisposeNeighborArrays();
        }

        private void RebuildNeighborCache(int nodeCount)
        {
            DisposeNeighborArrays();

            var counts = new int[nodeCount];
            foreach (var b in beams)
            {
                if (b.IsCrossBody || !b.isActive) continue;
                counts[b.nodeA]++;
                counts[b.nodeB]++;
            }

            _nbrOffsets = new NativeArray<int>(nodeCount + 1, Allocator.Persistent);
            int total = 0;
            for (int i = 0; i < nodeCount; i++) { _nbrOffsets[i] = total; total += counts[i]; }
            _nbrOffsets[nodeCount] = total;

            _nbrData = new NativeArray<int>(total, Allocator.Persistent);
            _nbrInitialPos = new NativeArray<float3>(nodeCount, Allocator.Persistent);
            _nbrMasses = new NativeArray<float>(nodeCount, Allocator.Persistent);
            _nbrCurrentLocal = new NativeArray<float3>(nodeCount, Allocator.Persistent);
            _nbrRotOut = new NativeArray<quaternion>(nodeCount, Allocator.Persistent);

            var cursors = new int[nodeCount];
            foreach (var b in beams)
            {
                if (b.IsCrossBody || !b.isActive) continue;
                _nbrData[_nbrOffsets[b.nodeA] + cursors[b.nodeA]++] = b.nodeB;
                _nbrData[_nbrOffsets[b.nodeB] + cursors[b.nodeB]++] = b.nodeA;
            }

            for (int i = 0; i < nodeCount; i++)
            {
                _nbrInitialPos[i] = nodeManager.InitialPositions[i];
                _nbrMasses[i] = i < nodeMasses.Count ? nodeMasses[i] : PhysicsConstants.DEFAULT_NODE_MASS;
            }

            _nbrCachedCount = nodeCount;
            _nbrDirty = false;
        }

        private void DisposeNeighborArrays()
        {
            if (_nbrOffsets.IsCreated) _nbrOffsets.Dispose();
            if (_nbrData.IsCreated) _nbrData.Dispose();
            if (_nbrInitialPos.IsCreated) _nbrInitialPos.Dispose();
            if (_nbrMasses.IsCreated) _nbrMasses.Dispose();
            if (_nbrCurrentLocal.IsCreated) _nbrCurrentLocal.Dispose();
            if (_nbrRotOut.IsCreated) _nbrRotOut.Dispose();
        }

        private void UpdateNodeRotations()
        {
            int nodeCount = nodeManager.NodeCount;
            if (nodeCount == 0) return;

            if (_nbrDirty || _nbrCachedCount != nodeCount)
                RebuildNeighborCache(nodeCount);

            EnsureReusableWorldPositionsBuffer(nodeCount);
            for (int i = 0; i < nodeCount; i++)
            {
                _reusableWorldPositions[i] = nodeManager.GetPosition(i);
                _nbrRotOut[i] = nodeManager.CurrentRotations[i];
            }

            float4x4 w2l = _owner.worldToLocalMatrix;

            // Stage 1: transform all world positions to local space in parallel
            JobHandle h0 = new WorldToLocalJob
            {
                worldPos = _reusableWorldPositions,
                w2l = w2l,
                localPos = _nbrCurrentLocal,
            }.Schedule(nodeCount, 32);

            // Stage 2: compute rotations in parallel (no inter-node writes, safe)
            new UpdateNodeRotationsJob
            {
                nbrOffsets = _nbrOffsets,
                nbrData = _nbrData,
                localPos = _nbrCurrentLocal,
                initialPos = _nbrInitialPos,
                masses = _nbrMasses,
                rotOut = _nbrRotOut,
                maxIter = PhysicsConstants.MAX_ROTATION_ITERATIONS,
                convEps = PhysicsConstants.ROTATION_CONVERGENCE_EPSILON,
                minOmega = PhysicsConstants.MIN_OMEGA_MAGNITUDE,
                minMass = PhysicsConstants.MIN_TOTAL_MASS,
            }.Schedule(nodeCount, 8, h0).Complete();

            for (int i = 0; i < nodeCount; i++)
                nodeManager.CurrentRotations[i] = _nbrRotOut[i];
        }

        [BurstCompile]
        private struct WorldToLocalJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> worldPos;
            [ReadOnly] public float4x4 w2l;
            [WriteOnly] public NativeArray<float3> localPos;
            public void Execute(int i) => localPos[i] = math.transform(w2l, worldPos[i]);
        }

        [BurstCompile]
        private struct UpdateNodeRotationsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> nbrOffsets;
            [ReadOnly] public NativeArray<int> nbrData;
            [ReadOnly] public NativeArray<float3> localPos;
            [ReadOnly] public NativeArray<float3> initialPos;
            [ReadOnly] public NativeArray<float> masses;
            // Each index writes only its own slot — no parallel conflict
            [NativeDisableParallelForRestriction]
            public NativeArray<quaternion> rotOut;

            public int maxIter;
            public float convEps, minOmega, minMass;

            public void Execute(int ni)
            {
                int s = nbrOffsets[ni];
                int e = nbrOffsets[ni + 1];
                if (e == s) return;

                float mSelf = masses[ni];
                float3 comR = mSelf * initialPos[ni];
                float3 comC = mSelf * localPos[ni];
                float mSum = mSelf;

                for (int k = s; k < e; k++)
                {
                    int j = nbrData[k]; float m = masses[j];
                    comR += m * initialPos[j];
                    comC += m * localPos[j];
                    mSum += m;
                }
                if (mSum < minMass) return;

                float inv = math.rcp(mSum);
                comR *= inv; comC *= inv;

                float3x3 A = float3x3.zero;
                {
                    float3 pR = initialPos[ni] - comR;
                    float3 pC = localPos[ni] - comC;
                    A.c0 += mSelf * pC * pR.x;
                    A.c1 += mSelf * pC * pR.y;
                    A.c2 += mSelf * pC * pR.z;
                }
                for (int k = s; k < e; k++)
                {
                    int j = nbrData[k]; float m = masses[j];
                    float3 pR = initialPos[j] - comR;
                    float3 pC = localPos[j] - comC;
                    A.c0 += m * pC * pR.x;
                    A.c1 += m * pC * pR.y;
                    A.c2 += m * pC * pR.z;
                }

                // ── Muller polar decomposition ───────────────────────────
                // float3x3(quaternion) is the SIMD rotation matrix — no Matrix4x4 boxing
                quaternion q = rotOut[ni];
                for (int iter = 0; iter < maxIter; iter++)
                {
                    float3x3 R = new float3x3(q);
                    float3 tau = math.cross(R.c0, A.c0)
                               + math.cross(R.c1, A.c1)
                               + math.cross(R.c2, A.c2);
                    float denom = math.dot(R.c0, A.c0)
                                + math.dot(R.c1, A.c1)
                                + math.dot(R.c2, A.c2);
                    float3 omega = tau / (math.abs(denom) + convEps);
                    float w = math.length(omega);
                    if (w < minOmega) break;
                    q = math.normalize(math.mul(quaternion.AxisAngle(omega / w, w), q));
                }
                rotOut[ni] = q;
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
            public float minLength;
            public float maxLength;
            public bool isActive;
            public bool isCrossBody;
        }
        [BurstCompile]
        private struct VerletJob : IJobParallelFor
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
                float3 velocity = (currentWorldPositions[i] - previousPositions[i]) / dt * dampingFactor;
                predictedPositions[i] = currentWorldPositions[i] + velocity * dt + worldGravity * dtSquared;
            }
        }

        [BurstCompile]
        private struct ConstraintsJob : IJob
        {
            [ReadOnly] public NativeArray<BeamData> allBeamData;
            [ReadOnly] public NativeArray<float> beamRestLengths;
            [ReadOnly] public NativeArray<float> nodeMasses;
            [ReadOnly] public NativeArray<bool> isPinned;
            [ReadOnly] public float invDtSq;
            [ReadOnly] public float worldRestLengthScale;

            public NativeArray<float3> predictedPositions;
            public NativeArray<float> beamLagrangeMultipliers;

            public void Execute()
            {
                for (int i = 0; i < allBeamData.Length; i++)
                {
                    BeamData beam = allBeamData[i];
                    if (!beam.isActive || beam.isCrossBody) continue;

                    float3 posA = predictedPositions[beam.nodeA];
                    float3 posB = predictedPositions[beam.nodeB];
                    float3 delta = posA - posB;
                    float len = math.length(delta);

                    if (len < 1e-7f) continue;

                    float restLength = beamRestLengths[i] * worldRestLengthScale;
                    float constraintVal = len - restLength;

                    float wA = isPinned[beam.nodeA] ? 0f : math.rcp(nodeMasses[beam.nodeA]);
                    float wB = isPinned[beam.nodeB] ? 0f : math.rcp(nodeMasses[beam.nodeB]);
                    float wSum = wA + wB;
                    if (wSum <= 0f) continue;

                    float alphaTilde = beam.compliance * invDtSq;
                    float curLambda = beamLagrangeMultipliers[i];
                    float deltaLambda = -(constraintVal + alphaTilde * curLambda) / (wSum + alphaTilde);

                    beamLagrangeMultipliers[i] = curLambda + deltaLambda;
                    float3 correction = deltaLambda * (delta / len);

                    if (!isPinned[beam.nodeA]) predictedPositions[beam.nodeA] += wA * correction;
                    if (!isPinned[beam.nodeB]) predictedPositions[beam.nodeB] -= wB * correction;
                }
            }
        }
    }
}