/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    public class SolverComponent : MonoBehaviour
    {
        public Solver solver;

        private void OnDestroy()
        {
            if (solver != null)
            {
                // Clean up if needed
            }
        }
    }
    public struct CollisionForce
    {
        public int nodeIndex;
        public Vector3 force;
        public Vector3 contactPoint;

        public CollisionForce(int nodeIndex, Vector3 force, Vector3 contactPoint)
        {
            this.nodeIndex = nodeIndex;
            this.force = force;
            this.contactPoint = contactPoint;
        }
    }

    public class SoftbodyCollisionManager
    {
        private static readonly Dictionary<int, List<CollisionForce>> pendingCollisionForces =
            new Dictionary<int, List<CollisionForce>>();
        private static readonly HashSet<int> activeSolvers = new HashSet<int>();

        public static void RegisterSolver(int solverID)
        {
            activeSolvers.Add(solverID);
            if (!pendingCollisionForces.ContainsKey(solverID))
                pendingCollisionForces[solverID] = new List<CollisionForce>();
        }

        public static void UnregisterSolver(int solverID)
        {
            activeSolvers.Remove(solverID);
            pendingCollisionForces.Remove(solverID);
        }

        public static void AddCollisionForce(int targetSolverID, CollisionForce force)
        {
            if (pendingCollisionForces.ContainsKey(targetSolverID))
            {
                pendingCollisionForces[targetSolverID].Add(force);
            }
        }

        public static List<CollisionForce> GetAndClearCollisionForces(int solverID)
        {
            if (!pendingCollisionForces.ContainsKey(solverID))
                return new List<CollisionForce>();

            var forces = new List<CollisionForce>(pendingCollisionForces[solverID]);
            pendingCollisionForces[solverID].Clear();
            return forces;
        }

        public static bool AllSolversFinishedDetection()
        {
            // Simple synchronization - you might need a more sophisticated approach
            return activeSolvers.Count > 0;
        }
    }

    public class Solver
    {
        private static bool s_isApplicationQuitting = false;

        // Softbody collision system
        private int solverID;
        private static int nextSolverID = 0;

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
        public  float maxStretchFactor = 1.05f;
        public float minStretchFactor = 0.95f;
        public bool visualizeForces = false;
        public List<float> nodeMasses = new List<float>();
        public float compliance = 1e-3f;
        public float defaultDamping = 0.3f;
        public float deformationScale = 0.1f;
        public float maxDeformation = 0.5f;
        public float plasticityThreshold = 0.1f;
        public float plasticityRate = 0.01f;
        private float restThreshold = 0.01f;
        private Truss trussAsset; // Reference to TrussAsset for face data

        #region Pressure variables
        public float internalPressure = 0f;
        public float pressureForceMultiplier = 1f;
        public float pressureDamping = 0.3f;
        #endregion

        #region Collision variables
        private readonly Collider[] overlapResults = new Collider[32];
        private LayerMask collisionLayerMask = ~0;
        #endregion

        // Runtime Visualization
        public bool showNodesAndLinks = false;
        public bool showNodes = true;
        public bool showLinks = true;
        public bool showNodeIndices = false;
        public bool showLinkForces = false;
        public bool showInfluenceRadius = false;
        public float nodeDisplaySize = 0.1f;
        public Color nodeColor = Color.green;
        public Color pinnedNodeColor = Color.red;
        public Color linkColor = Color.blue;
        public Color stretchedLinkColor = Color.red;
        public Color compressedLinkColor = Color.yellow;
        public Color influenceRadiusColor = new Color(1f, 1f, 0f, 0.3f);
        public float forceVisualizationScale = 0.01f;

        [Header("Adaptive Sub-stepping")]
        public bool enableAdaptiveSubstepping = true;
        public int minSubSteps = 1;
        public int maxSubSteps = 10;
        public float stabilityThreshold = 0.1f;  // Max displacement per frame for stability
        public float energyThreshold = 0.05f;   // Energy change threshold
        public float adaptationRate = 0.1f;     // How quickly to adapt (0-1)

        // Performance tracking removed to eliminate warnings
        private Queue<float> displacementHistory = new Queue<float>();
        private const int historySize = 5;

        public Solver(float influenceRadius, Mesh mesh, Vector3[] originalVertices, Transform owner)
        {
            this.beams = new List<Beam>();
            this.meshDeformer = new MeshDeformer(mesh, originalVertices, influenceRadius);
            this.collisionPoints = new List<Vector3>();
            this.owner = owner;
            this.nodeManager = new NodeManager();
            nodeMasses = new List<float>();

            // Register for softbody collision system
            solverID = nextSolverID++;
            SoftbodyCollisionManager.RegisterSolver(solverID);
        }

        ~Solver()
        {
            SoftbodyCollisionManager.UnregisterSolver(solverID);
        }

        public void InitializeFromTruss(Truss truss)
        {
            trussAsset = truss;
            if (truss != null)
            {
                // Copy per-node masses from Truss
                nodeMasses = new List<float>(truss.NodeMasses);

                // Ensure size matches nodes (pad with default if mismatch)
                while (nodeMasses.Count < nodeManager.Nodes.Count)
                    nodeMasses.Add(0.5f);
                if (nodeMasses.Count > nodeManager.Nodes.Count)
                    nodeMasses.RemoveRange(nodeManager.Nodes.Count, nodeMasses.Count - nodeManager.Nodes.Count);

                // Apply other Truss properties
                maxStretchFactor = truss.MaxStretchFactor;
                minStretchFactor = truss.MinStretchFactor;
                compliance = truss.compliance;
                defaultDamping = truss.DefaultBeamDamping;
            }
            else
            {
                // Default uniform if no Truss
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                    nodeMasses.Add(0.5f);
            }
        }

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

            // Don't create nodes during application quit or when parent GameObject is being destroyed
            if (s_isApplicationQuitting || parent.gameObject == null)
            {
                return;
            }

            nodeManager.Clear();
            beams.Clear();
            nodeMasses.Clear();  // Clear masses
            DestroyOldNodes(parent);

            for (int i = 0; i < positions.Length; i++)
            {
                GameObject nodeObj = new GameObject($"Node_{i}");
                nodeObj.transform.parent = parent;
                nodeObj.transform.localPosition = positions[i]; // Use local position to maintain relative positioning
                nodeObj.hideFlags = HideFlags.HideAndDontSave;
                nodeObj.AddComponent<SphereCollider>().radius = nodeManager.GetNodeRadius(i);

                nodeManager.AddNode(nodeObj.transform, positions[i]);
                nodeMasses.Add(trussAsset != null && i < trussAsset.NodeMasses.Count ? trussAsset.NodeMasses[i] : 0.5f);  // Use Truss mass or default
            }

            if (beamsArray != null)
            {
                foreach (var b in beamsArray)
                {
                    if (b.nodeA >= 0 && b.nodeA < positions.Length &&
                        b.nodeB >= 0 && b.nodeB < positions.Length &&
                        b.restLength > 0.01f)
                    {
                        beams.Add(new Beam(b.nodeA, b.nodeB,
                                           b.compliance, b.damping, b.restLength));
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

            // Tell the deformer how to skin
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

        public void Solve()
        {
            if (nodeManager == null || beams == null || nodeManager.Nodes == null)
            {
                Debug.LogWarning("Cannot simulate plastic deformation: Invalid state.");
                return;
            }

            // Get sub-stepping parameters from SceneSettings
            int numSubSteps = SceneSettings.Instance != null ?
                Mathf.Clamp(SceneSettings.Instance.BaseSubSteps, SceneSettings.Instance.MinSubSteps, 50) : 1;

            float fullDt = Time.fixedDeltaTime;
            float subDt = fullDt / numSubSteps;

            // Perform sub-stepping for better stability
            for (int step = 0; step < numSubSteps; step++)
            {
                SolveSubStep(subDt);
            }

            DrawNodesAndLinks();
        }

        private void SolveSubStep(float dt)
        {
            const float restitution = 0.02f;
            const float friction = 0.9f;
            const float velocityDamping = 0.95f;
            const float epsilon = 0.001f;
            float dtSquared = dt * dt;

            collisionPoints.Clear();

            if (nodeManager.PreviousPositions.Count == 0 || nodeManager.PreviousPositions.Count != nodeManager.Nodes.Count)
            {
                nodeManager.PreviousPositions.Clear();
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                {
                    Vector3 worldPos = nodeManager.Nodes[i] != null ? owner.TransformPoint(nodeManager.Nodes[i].localPosition) : Vector3.zero;
                    nodeManager.PreviousPositions.Add(worldPos);
                }
            }

            if (nodeManager.PredictedPositions.Count == 0 || nodeManager.PredictedPositions.Count != nodeManager.Nodes.Count)
            {
                nodeManager.PredictedPositions.Clear();
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                {
                    Vector3 worldPos = nodeManager.Nodes[i] != null ? owner.TransformPoint(nodeManager.Nodes[i].localPosition) : Vector3.zero;
                    nodeManager.PredictedPositions.Add(worldPos);
                }
            }

            // Step 1: Verlet integration with rest detection
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                // Get current world position from local position
                Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                Vector3 prevWorldPos = nodeManager.PreviousPositions[i];
                Vector3 worldVelocity = (currentWorldPos - prevWorldPos) / dt;

                // Rest detection based on world velocity
                float velocityMagnitude = worldVelocity.magnitude;

                // Apply gravity directly in world space
                float gravityValue = SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : 9.81f;
                Vector3 worldGravity = Vector3.down * gravityValue;
                Vector3 worldAcceleration = worldGravity;

                // Only apply light damping to prevent drift, not enough to stop falling acceleration
                worldVelocity *= 0.995f; // Very light damping to preserve momentum

                // Only apply rest detection for very slow horizontal movement, not vertical falling
                if (velocityMagnitude < restThreshold * 0.05f && Mathf.Abs(worldVelocity.y) < 0.1f) // Only horizontal rest
                {
                    worldVelocity.x *= 0.8f; // Dampen horizontal drift
                    worldVelocity.z *= 0.8f; // Dampen horizontal drift
                    // Leave vertical velocity (y) unchanged to preserve falling
                }

                // Apply minimal global velocity damping only for collision scenarios
                if (velocityMagnitude > 10f) // Only dampen very high velocities
                {
                    worldVelocity *= velocityDamping;
                }

                Vector3 newWorldPosition = currentWorldPos + worldVelocity * dt + worldAcceleration * dtSquared;

                if (float.IsNaN(newWorldPosition.x) || float.IsNaN(newWorldPosition.y) || float.IsNaN(newWorldPosition.z))
                {
                    newWorldPosition = currentWorldPos;
                    Debug.LogWarning($"NaN detected for node {i}, keeping at {currentWorldPos}");
                }
                nodeManager.PredictedPositions[i] = newWorldPosition;
            }

            // Apply pressure forces after Verlet integration
            if (internalPressure > 0)
            {
                ApplyPressureForces(dt);
            }

            // PHASE 1: Detect softbody-to-softbody collisions
            DetectSoftbodyCollisions(dt);

            // PHASE 2: Apply collision forces from other softbodies
            ApplyPendingCollisionForces();

            // Step 2: Environment collision detection (working in world space)
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                Vector3 predictedWorldPos = nodeManager.PredictedPositions[i];
                Vector3 worldMotion = predictedWorldPos - currentWorldPos;
                float motionDistance = worldMotion.magnitude;
                if (motionDistance < epsilon) continue; // Skip if negligible movement

                Vector3 worldDirection = worldMotion.normalized;

                // Handle initial overlaps (safety net for nodes starting inside colliders)
                int hitCount = Physics.OverlapSphereNonAlloc(currentWorldPos, nodeManager.GetNodeRadius(i), overlapResults, collisionLayerMask);
                for (int j = 0; j < hitCount; j++)
                {
                    Collider col = overlapResults[j];
                    if (col == nodeManager.Colliders[i] || col.transform == nodeManager.Nodes[i]) continue;

                    // Skip softbody nodes - they're handled by the collision detection system
                    var solverComp = GetSolverComponentFromCollider(col);
                    var otherSolver = solverComp != null ? solverComp.solver : null;
                    if (otherSolver != null) continue;

                    // Terrain-specific depenetration
                    if (col is TerrainCollider terrainCol)
                    {
                        Terrain terrain = terrainCol.GetComponent<Terrain>();
                        if (terrain != null)
                        {
                            float height = terrain.SampleHeight(currentWorldPos) + terrain.transform.position.y;
                            if (currentWorldPos.y < height + nodeManager.GetNodeRadius(i))
                            {
                                Vector3 worldNewPos = currentWorldPos;
                                worldNewPos.y = height + nodeManager.GetNodeRadius(i) + epsilon;
                                predictedWorldPos = worldNewPos;
                                collisionPoints.Add(currentWorldPos);
                            }
                        }
                    }
                    // For other colliders, use ClosestPoint as fallback
                    else
                    {
                        Vector3 closestPoint = col.ClosestPoint(currentWorldPos);
                        if (closestPoint != currentWorldPos) // Not inside collider
                        {
                            Vector3 worldDirectionToSurface = (closestPoint - currentWorldPos).normalized;
                            predictedWorldPos += worldDirectionToSurface * (nodeManager.GetNodeRadius(i) + epsilon);
                            collisionPoints.Add(currentWorldPos);
                        }
                    }
                }

                // Sweep-based environment collision
                if (Physics.SphereCast(currentWorldPos, nodeManager.GetNodeRadius(i), worldDirection, out RaycastHit hit, motionDistance, collisionLayerMask))
                {
                    // Skip softbody nodes - they're handled by the collision detection system
                    var solverComp = GetSolverComponentFromCollider(hit.collider);
                    var otherSolver = solverComp != null ? solverComp.solver : null;
                    if (otherSolver != null) continue;

                    if (hit.distance < motionDistance)
                    {
                        Vector3 worldHitPos = currentWorldPos + worldDirection * hit.distance;
                        Vector3 worldNewPos = hit.point + hit.normal * (nodeManager.GetNodeRadius(i) + epsilon);

                        // Terrain-specific normal and height adjustment
                        if (hit.collider is TerrainCollider terrainCol)
                        {
                            Terrain terrain = terrainCol.GetComponent<Terrain>();
                            if (terrain != null)
                            {
                                float height = terrain.SampleHeight(worldHitPos) + terrain.transform.position.y;
                                worldNewPos.y = Mathf.Max(worldNewPos.y, height + nodeManager.GetNodeRadius(i) + epsilon);
                                float normX = (worldHitPos.x - terrain.transform.position.x) / terrain.terrainData.size.x;
                                float normZ = (worldHitPos.z - terrain.transform.position.z) / terrain.terrainData.size.z;
                                hit.normal = terrain.terrainData.GetInterpolatedNormal(normX, normZ);
                            }
                        }

                        predictedWorldPos = worldNewPos;

                        // Velocity response in world space
                        Vector3 worldVelocity = worldMotion / dt;
                        Vector3 worldNormalVelocity = Vector3.Project(worldVelocity, hit.normal);
                        Vector3 worldTangentVelocity = worldVelocity - worldNormalVelocity;
                        worldTangentVelocity *= (1f - friction);
                        worldNormalVelocity *= restitution;
                        Vector3 worldNewVelocity = (worldTangentVelocity + worldNormalVelocity) * velocityDamping;
                        nodeManager.PreviousPositions[i] = currentWorldPos - worldNewVelocity * dt;

                        collisionPoints.Add(hit.point);
                        Debug.DrawLine(worldHitPos, worldNewPos, Color.green, 0.1f);
                        Debug.DrawRay(worldNewPos, hit.normal * 0.2f, Color.yellow, 0.1f);
                    }
                }
                nodeManager.PredictedPositions[i] = predictedWorldPos;

                // Face-node collisions (working in world space)
                if (trussAsset != null)
                {
                    foreach (var face in trussAsset.GetTrussFaces())
                    {
                        if (face.nodeA >= nodeManager.Nodes.Count || face.nodeB >= nodeManager.Nodes.Count || face.nodeC >= nodeManager.Nodes.Count ||
                            nodeManager.Nodes[face.nodeA] == null || nodeManager.Nodes[face.nodeB] == null || nodeManager.Nodes[face.nodeC] == null)
                            continue;

                        if (i == face.nodeA || i == face.nodeB || i == face.nodeC) continue;

                        // Convert local rest positions to world space for face collision
                        Vector3 worldPosA = owner.TransformPoint(nodeManager.InitialPositions[face.nodeA]);
                        Vector3 worldPosB = owner.TransformPoint(nodeManager.InitialPositions[face.nodeB]);
                        Vector3 worldPosC = owner.TransformPoint(nodeManager.InitialPositions[face.nodeC]);
                        Vector3 worldNodePos = nodeManager.PredictedPositions[i];

                        Vector3 worldNormal = Vector3.Cross(worldPosB - worldPosA, worldPosC - worldPosA).normalized;
                        float distanceToPlane = Vector3.Dot(worldNodePos - worldPosA, worldNormal);

                        if (Mathf.Abs(distanceToPlane) < nodeManager.GetNodeRadius(i))
                        {
                            Vector3 projectedPoint = worldNodePos - distanceToPlane * worldNormal;

                            // Check if point is in triangle (in world space)
                            if (IsPointInTriangle(projectedPoint, worldPosA, worldPosB, worldPosC))
                            {
                                float w = nodeManager.IsPinned[i] ? 0f : 1f / nodeMasses[i];  // Per-node mass
                                float constraintCompliance = compliance / dtSquared;
                                float lambda = -distanceToPlane / (w + constraintCompliance);
                                Vector3 worldCorrection = worldNormal * lambda;

                                if (float.IsNaN(worldCorrection.x) || float.IsNaN(worldCorrection.y) || float.IsNaN(worldCorrection.z))
                                {
                                    worldCorrection = Vector3.zero;
                                    Debug.LogWarning($"NaN correction in face-node collision for node {i}, skipping");
                                }
                                else
                                {
                                    nodeManager.PredictedPositions[i] += worldCorrection;
                                    Vector3 worldCollisionPoint = nodeManager.PredictedPositions[i] - worldCorrection;
                                    collisionPoints.Add(worldCollisionPoint);
                                    Debug.DrawLine(worldCollisionPoint, nodeManager.PredictedPositions[i], Color.cyan, 0.1f);
                                    Debug.DrawRay(nodeManager.PredictedPositions[i], worldNormal * 0.2f, Color.magenta, 0.1f);
                                }
                            }
                        }
                    }
                }
            }

            // Step 3: XPBD constraint solving (all in world space)
            for (int iter = 0; iter < (SceneSettings.Instance != null ? SceneSettings.Instance.ConstraintIterations : 10); iter++)
            {
                for (int i = 0; i < beams.Count; i++)
                {
                    Beam beam = beams[i];

                    // Use per-node masses in beam constraint
                    float wA = nodeManager.IsPinned[beam.nodeA] ? 0f : 1f / nodeMasses[beam.nodeA];
                    float wB = nodeManager.IsPinned[beam.nodeB] ? 0f : 1f / nodeMasses[beam.nodeB];

                    // Pass per-node masses to SolveConstraint
                    beam.SolveConstraint(dt, nodeMasses[beam.nodeA], nodeMasses[beam.nodeB]);

                    // Regular beam processing for same-body constraints
                    if (beam.nodeA >= nodeManager.Nodes.Count || beam.nodeB >= nodeManager.Nodes.Count ||
                        nodeManager.Nodes[beam.nodeA] == null || nodeManager.Nodes[beam.nodeB] == null)
                    {
                        Debug.LogWarning($"Invalid beam {i}: nodeA={beam.nodeA}, nodeB={beam.nodeB}");
                        continue;
                    }

                    if (nodeManager.IsPinned[beam.nodeA] && nodeManager.IsPinned[beam.nodeB]) continue;

                    // Use world space positions for constraint solving
                    Vector3 worldPosA = nodeManager.PredictedPositions[beam.nodeA];
                    Vector3 worldPosB = nodeManager.PredictedPositions[beam.nodeB];
                    Vector3 delta = worldPosB - worldPosA;
                    float currentLength = delta.magnitude;
                    if (currentLength < 0.0001f)
                    {
                        Debug.LogWarning($"Beam {i} has near-zero length: {currentLength}, skipping correction");
                        continue;
                    }

                    // Convert rest length from local to world space
                    Vector3 localRestVector = Vector3.right * beam.restLength; // Arbitrary direction in local space
                    Vector3 worldRestVector = owner.TransformDirection(localRestVector);
                    float worldRestLength = worldRestVector.magnitude;

                    float constraint = currentLength - worldRestLength;
                    Vector3 gradient = delta / currentLength;

                    float wSum = wA + wB;

                    if (wSum == 0f) continue;

                    float compliance = beam.compliance / dtSquared;
                    float lambda = -constraint / (wSum + compliance);
                    beam.lagrangeMultiplier += lambda;

                    Vector3 correction = gradient * lambda;
                    if (float.IsNaN(correction.x) || float.IsNaN(correction.y) || float.IsNaN(correction.z))
                    {
                        correction = Vector3.zero;
                        Debug.LogWarning($"NaN correction for beam {i}, resetting to zero");
                    }

                    nodeManager.PredictedPositions[beam.nodeA] += wA * correction;
                    nodeManager.PredictedPositions[beam.nodeB] -= wB * correction;

                    // Apply stretch limits in world space
                    currentLength = Vector3.Distance(nodeManager.PredictedPositions[beam.nodeA], nodeManager.PredictedPositions[beam.nodeB]);
                    if (currentLength > worldRestLength * maxStretchFactor)
                    {
                        float scale = worldRestLength * maxStretchFactor / currentLength;
                        Vector3 center = (nodeManager.PredictedPositions[beam.nodeA] + nodeManager.PredictedPositions[beam.nodeB]) * 0.5f;
                        if (!nodeManager.IsPinned[beam.nodeA]) nodeManager.PredictedPositions[beam.nodeA] = center + (nodeManager.PredictedPositions[beam.nodeA] - center) * scale;
                        if (!nodeManager.IsPinned[beam.nodeB]) nodeManager.PredictedPositions[beam.nodeB] = center + (nodeManager.PredictedPositions[beam.nodeB] - center) * scale;
                    }
                    else if (currentLength < worldRestLength * minStretchFactor)
                    {
                        float scale = worldRestLength * minStretchFactor / currentLength;
                        Vector3 center = (nodeManager.PredictedPositions[beam.nodeA] + nodeManager.PredictedPositions[beam.nodeB]) * 0.5f;
                        if (!nodeManager.IsPinned[beam.nodeA]) nodeManager.PredictedPositions[beam.nodeA] = center + (nodeManager.PredictedPositions[beam.nodeA] - center) * scale;
                        if (!nodeManager.IsPinned[beam.nodeB]) nodeManager.PredictedPositions[beam.nodeB] = center + (nodeManager.PredictedPositions[beam.nodeB] - center) * scale;
                    }

                    beams[i] = beam;
                    if (visualizeForces && constraint != 0)
                    {
                        float forceMagnitude = Mathf.Abs(beam.lagrangeMultiplier) / dtSquared;
                        Debug.DrawLine(worldPosA, worldPosB, forceMagnitude > 0 ? Color.red : Color.blue, 0.1f);
                    }
                }
            }

            // Step 4: Finalize positions and convert back to local space
            float sumDisp = 0f;
            float maxDisp = 0f;
            int movableCount = 0;
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                Vector3 predictedWorldPos = nodeManager.PredictedPositions[i];
                Vector3 worldMotion = predictedWorldPos - currentWorldPos;
                sumDisp += worldMotion.magnitude;
                if (!nodeManager.IsPinned[i]) movableCount++;
                if (worldMotion.magnitude > maxDisp) maxDisp = worldMotion.magnitude;

                // Final collision detection in world space
                Collider nodeCollider = nodeManager.Colliders[i];
                
                // Skip collision detection if node has no collider
                if (nodeCollider == null) continue;
                
                Collider[] colliders = Physics.OverlapSphere(predictedWorldPos, nodeManager.GetNodeRadius(i));
                bool inCollision = false;
                Vector3 worldNormal = Vector3.zero;
                foreach (var collider in colliders)
                {
                    if (collider == nodeCollider || collider.transform == nodeManager.Nodes[i]) continue;
                    int otherNodeIndex = nodeManager.FindIndex(n => n != null && n == collider.transform);
                    if (otherNodeIndex != -1) continue;

                    // Skip softbody nodes - they're handled by the collision detection system
                    var solverComp = GetSolverComponentFromCollider(collider);
                    var otherSolver = solverComp != null ? solverComp.solver : null;
                    if (otherSolver != null) continue;

                    Vector3 direction;
                    float distance;
                    if (Physics.ComputePenetration(nodeCollider, predictedWorldPos, owner.rotation,
                                                  collider, collider.transform.position, collider.transform.rotation,
                                                  out direction, out distance))
                    {
                        inCollision = true;
                        worldNormal = direction;
                        // Apply correction in world space
                        predictedWorldPos += direction * distance;
                        nodeManager.PredictedPositions[i] = predictedWorldPos; // Update the array

                        collisionPoints.Add(predictedWorldPos);
                        break;
                    }
                }

                // Convert final world position back to local space
                Vector3 finalLocalPos = owner.InverseTransformPoint(predictedWorldPos);
                nodeManager.Nodes[i].localPosition = finalLocalPos;

                // Update previous positions for next frame (in world space)
                if (inCollision)
                {
                    Vector3 worldVelocity = worldMotion / dt;
                    Vector3 worldVelocityNormal = Vector3.Project(worldVelocity, worldNormal);
                    Vector3 worldVelocityTangent = worldVelocity - worldVelocityNormal;
                    Vector3 worldNewVelocity = worldVelocityTangent * (1f - friction) - worldVelocityNormal * restitution;

                    if (worldNewVelocity.magnitude < restThreshold)
                    {
                        worldNewVelocity = Vector3.zero;
                    }
                    nodeManager.PreviousPositions[i] = predictedWorldPos - worldNewVelocity * dt;
                }
                else
                {
                    nodeManager.PreviousPositions[i] = currentWorldPos;
                }
            }
        }

        private void DetectSoftbodyCollisions(float dt)
        {
            if (nodeManager?.Nodes == null) return;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 worldPosA = nodeManager.PredictedPositions[i]; // Already in world space
                int hitCount = Physics.OverlapSphereNonAlloc(worldPosA, nodeManager.GetNodeRadius(i) * 1.5f, overlapResults, collisionLayerMask);

                for (int j = 0; j < hitCount; j++)
                {
                    Collider otherCollider = overlapResults[j];
                    if (otherCollider.transform == nodeManager.Nodes[i]) continue;

                    var solverComp = GetSolverComponentFromCollider(otherCollider);
                    var otherSolver = solverComp != null ? solverComp.solver : null;
                    if (otherSolver == null || otherSolver == this) continue; // Not a softbody node or self

                    Vector3 worldPosB = otherCollider.transform.position;
                    Vector3 delta = worldPosB - worldPosA;
                    float distance = delta.magnitude;
                    // Get radii for both nodes
                    float nodeARadius = nodeManager.GetNodeRadius(i);
                    float nodeBRadius = nodeARadius; // Default to same, but try to get actual radius if possible

                    // Try to get the other node's radius if it's from the same solver
                    int otherNodeIdx = otherSolver?.nodeManager.FindIndex(t => t == otherCollider.transform) ?? -1;
                    if (otherNodeIdx >= 0 && otherSolver != null)
                        nodeBRadius = otherSolver.nodeManager.GetNodeRadius(otherNodeIdx);

                    float collisionRadius = nodeARadius + nodeBRadius;

                    if (distance < collisionRadius && distance > 0.001f)
                    {
                        // Calculate collision response
                        Vector3 normal = delta.normalized;
                        float penetration = collisionRadius - distance;

                        // Calculate relative velocity
                        int otherNodeIndex = otherSolver.nodeManager.FindIndex(n => n != null && n == otherCollider.transform);
                        Vector3 velocityA = (nodeManager.PredictedPositions[i] - nodeManager.PreviousPositions[i]) / dt;
                        Vector3 velocityB = Vector3.zero;

                        if (otherNodeIndex != -1 && !otherSolver.nodeManager.IsPinned[otherNodeIndex])
                        {
                            velocityB = (otherSolver.nodeManager.PredictedPositions[otherNodeIndex] -
                                        otherSolver.nodeManager.PreviousPositions[otherNodeIndex]) / dt;
                        }

                        Vector3 relativeVelocity = velocityA - velocityB;
                        float normalVelocity = Vector3.Dot(relativeVelocity, normal);

                        // Apply impulse-based collision response
                        float restitution = 0.2f;
                        float friction = 0.4f;

                        // Normal impulse
                        float invMassA = nodeManager.IsPinned[i] ? 0f : 1f / nodeMasses[i];
                        float invMassB = (otherNodeIndex != -1 && !otherSolver.nodeManager.IsPinned[otherNodeIndex]) ? 1f / otherSolver.nodeMasses[otherNodeIndex] : 0f;
                        float jn = -(1 + restitution) * normalVelocity / (invMassA + invMassB);

                        // Friction impulse
                        Vector3 tangent = relativeVelocity - normal * normalVelocity;
                        if (tangent.magnitude > 0.001f) tangent.Normalize();

                        float jt = -Vector3.Dot(relativeVelocity, tangent) / (invMassA + invMassB);
                        jt = Mathf.Clamp(jt, -friction * jn, friction * jn);

                        // Total impulse
                        Vector3 impulse = normal * jn + tangent * jt;

                        // Add collision forces to both solvers
                        Vector3 contactPoint = (worldPosA + worldPosB) * 0.5f;
                        SoftbodyCollisionManager.AddCollisionForce(solverID,
                            new CollisionForce(i, impulse / dt, contactPoint));
                        SoftbodyCollisionManager.AddCollisionForce(otherSolver.solverID,
                            new CollisionForce(otherNodeIndex, -impulse / dt, contactPoint));

                        if (visualizeForces)
                        {
                            Debug.DrawLine(worldPosA, worldPosB, Color.magenta, dt);
                            Debug.DrawRay(contactPoint, impulse.normalized * 0.1f, Color.cyan, dt);
                        }
                    }
                }
            }
        }

        private void ApplyPendingCollisionForces()
        {
            var collisionForces = SoftbodyCollisionManager.GetAndClearCollisionForces(solverID);

            foreach (var force in collisionForces)
            {
                if (force.nodeIndex >= 0 && force.nodeIndex < nodeManager.Nodes.Count &&
                    !nodeManager.IsPinned[force.nodeIndex])
                {
                    // Convert force to local space and apply
                    Vector3 localForce = owner.InverseTransformDirection(force.force);
                    ApplyForceToNode(force.nodeIndex, localForce);

                    if (visualizeForces)
                    {
                        Vector3 worldPos = nodeManager.PredictedPositions[force.nodeIndex]; // Already in world space
                        Debug.DrawRay(worldPos, force.force.normalized * 0.2f, Color.yellow, Time.fixedDeltaTime);
                    }
                }
            }
        }

        private bool IsPointInTriangle(Vector3 point, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 v0 = c - a;
            Vector3 v1 = b - a;
            Vector3 v2 = point - a;

            float dot00 = Vector3.Dot(v0, v0);
            float dot01 = Vector3.Dot(v0, v1);
            float dot02 = Vector3.Dot(v0, v2);
            float dot11 = Vector3.Dot(v1, v1);
            float dot12 = Vector3.Dot(v1, v2);

            float invDenom = 1f / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            return u >= 0 && v >= 0 && u + v < 1;
        }

        private void UpdateBeamRestLengths(int nodeIndex, Vector3 newLocalPosition)
        {
            for (int i = 0; i < beams.Count; i++)
            {
                Beam beam = beams[i];
                if (beam.nodeA != nodeIndex && beam.nodeB != nodeIndex) continue;

                if (beam.nodeA >= nodeManager.Nodes.Count || beam.nodeB >= nodeManager.Nodes.Count ||
                    nodeManager.Nodes[beam.nodeA] == null || nodeManager.Nodes[beam.nodeB] == null)
                {
                    continue;
                }

                Vector3 posA = nodeManager.Nodes[beam.nodeA].localPosition; // Use local positions
                Vector3 posB = nodeManager.Nodes[beam.nodeB].localPosition;
                float newLength = Vector3.Distance(posA, posB);
                float strain = Mathf.Abs(newLength - beam.restLength) / beam.restLength;

                if (strain > plasticityThreshold)
                {
                    beam.restLength = Mathf.Lerp(beam.restLength, newLength, plasticityRate);
                    beam.restLength = Mathf.Clamp(beam.restLength, minStretchFactor * beam.originalRestLength, maxStretchFactor * beam.originalRestLength);
                    beam.lagrangeMultiplier = 0f;
                    beams[i] = beam;
                }
            }
        }

        public void DeformMesh(Transform transform)
        {
            if (transform == null) return;

            // Simply deform the mesh without any transform modifications
            meshDeformer.Deform(transform, nodeManager.Nodes, nodeManager.InitialPositions, nodeManager.InitialRotations);
        }

        public void ApplyForceToNode(int nodeIndex, Vector3 force)
        {
            if (nodeIndex < 0 || nodeIndex >= nodeMasses.Count || nodeManager.IsPinned[nodeIndex]) return;
            {
                float mass = nodeMasses[nodeIndex];
                if (mass <= 0f) return;  // Avoid div by zero

                Vector3 worldForce = owner.TransformDirection(force); 
                Vector3 accel = worldForce / mass; 

                Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[nodeIndex].localPosition);

                nodeManager.PreviousPositions[nodeIndex] = currentWorldPos - accel * Time.fixedDeltaTime * Time.fixedDeltaTime;
            }
        }

        public int FindClosestNodeToRay(Ray ray, float maxDistance)
        {
            int closestNode = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null) continue;
                // Convert local position to world position for ray casting
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
                Vector3 currentPos = nodeManager.Nodes[nodeIndex].localPosition; // Use local position
                Vector3 targetLocalPos = owner.InverseTransformPoint(targetPosition); // Convert target to local space
                Vector3 delta = targetLocalPos - currentPos;
                Vector3 impulse = delta * strength * nodeMasses[nodeIndex] / Time.fixedDeltaTime; // Use per-node mass
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

        private static void DelayedDestroyNodes(Transform parent)
        {
            // Schedule the destruction on the next idle frame
            EditorApplication.delayCall += () =>
            {
                if (parent == null) return;   // safety: object was deleted

                Undo.RegisterCompleteObjectUndo(parent, "Clear old soft-body nodes");

                for (int i = parent.childCount - 1; i >= 0; i--)
                {
                    Transform child = parent.GetChild(i);
                    if (child.name.StartsWith("Node_"))
                        Undo.DestroyObjectImmediate(child.gameObject);
                }
            };
        }

        internal void GenerateNodesAndBeams(Vector3[] vector3s, Beam[] beams, Transform transform)
        {
            throw new System.NotImplementedException();
        }

        // Runtime Visualization Methods
        public void DrawNodesAndLinks()
        {
            if (!showNodesAndLinks) return;

            if (showNodes)
                DrawNodes();

            if (showLinks)
                DrawLinks();

            if (showInfluenceRadius)
                DrawInfluenceRadius();
        }

        private void DrawNodes()
        {
            if (nodeManager?.Nodes == null) return;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null) continue;

                Vector3 position = nodeManager.Nodes[i].localPosition; // Use local position for calculations
                bool isPinned = i < nodeManager.IsPinned.Count && nodeManager.IsPinned[i];
                Color color = isPinned ? pinnedNodeColor : nodeColor;

                // Convert to world position for visualization
                Vector3 worldPosition = owner.TransformPoint(position);

                // Draw node as a wireframe sphere
                DrawWireSphere(worldPosition, nodeDisplaySize, color);

                // Draw node index if enabled
                if (showNodeIndices)
                {
                    DrawLabel(worldPosition + Vector3.up * (nodeDisplaySize + 0.1f), i.ToString(), color);
                }
            }
        }

        private void DrawLinks()
        {
            if (beams == null || nodeManager?.Nodes == null) return;

            for (int i = 0; i < beams.Count; i++)
            {
                Beam beam = beams[i];

                if (beam.nodeA >= nodeManager.Nodes.Count || beam.nodeB >= nodeManager.Nodes.Count ||
                    nodeManager.Nodes[beam.nodeA] == null || nodeManager.Nodes[beam.nodeB] == null)
                    continue;

                Vector3 posA = nodeManager.Nodes[beam.nodeA].localPosition; // Use local positions
                Vector3 posB = nodeManager.Nodes[beam.nodeB].localPosition;

                // Convert to world positions for visualization
                Vector3 worldPosA = owner.TransformPoint(posA);
                Vector3 worldPosB = owner.TransformPoint(posB);

                float currentLength = Vector3.Distance(posA, posB); // Calculate in local space

                // Determine link color based on stretch/compression
                Color color = linkColor;
                if (currentLength > beam.restLength * 1.01f)
                    color = stretchedLinkColor; // Stretched
                else if (currentLength < beam.restLength * 0.99f)
                    color = compressedLinkColor; // Compressed

                // Draw the link in world space
                Debug.DrawLine(worldPosA, worldPosB, color, Time.fixedDeltaTime);

                // Draw force visualization if enabled
                if (showLinkForces && beam.lagrangeMultiplier != 0)
                {
                    Vector3 midPoint = (worldPosA + worldPosB) * 0.5f;
                    Vector3 direction = (worldPosB - worldPosA).normalized;
                    float forceMagnitude = beam.lagrangeMultiplier * forceVisualizationScale;

                    if (forceMagnitude > 0)
                    {
                        // Tension (pulling apart)
                        Debug.DrawRay(midPoint, direction * forceMagnitude, Color.red, Time.fixedDeltaTime);
                        Debug.DrawRay(midPoint, -direction * forceMagnitude, Color.red, Time.fixedDeltaTime);
                    }
                    else
                    {
                        // Compression (pushing together)
                        Debug.DrawRay(midPoint, direction * Mathf.Abs(forceMagnitude), Color.blue, Time.fixedDeltaTime);
                        Debug.DrawRay(midPoint, -direction * Mathf.Abs(forceMagnitude), Color.blue, Time.fixedDeltaTime);
                    }
                }
            }
        }

        private void DrawWireSphere(Vector3 center, float radius, Color color)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = color;

            // Draw three circles to form a wireframe sphere
            DrawCircle(center, radius, Vector3.up);
            DrawCircle(center, radius, Vector3.right);
            DrawCircle(center, radius, Vector3.forward);

            Gizmos.color = oldColor;
        }

        private void DrawCircle(Vector3 center, float radius, Vector3 normal)
        {
            Vector3 forward = Vector3.Cross(normal, Vector3.up);
            if (forward.magnitude < 0.1f)
                forward = Vector3.Cross(normal, Vector3.right);

            Vector3 right = Vector3.Cross(normal, forward).normalized;
            forward = forward.normalized;

            int segments = 32;
            Vector3 prevPos = center + right * radius;

            for (int i = 1; i <= segments; i++)
            {
                float angle = (float)i / segments * Mathf.PI * 2f;
                Vector3 newPos = center + (right * Mathf.Cos(angle) + forward * Mathf.Sin(angle)) * radius;
                Gizmos.DrawLine(prevPos, newPos);
                prevPos = newPos;
            }
        }

        private void DrawLabel(Vector3 position, string text, Color color)
        {
#if UNITY_EDITOR
            UnityEditor.Handles.color = color;
            UnityEditor.Handles.Label(position, text);
#endif
        }

        // Method to toggle visualization at runtime
        public void ToggleVisualization()
        {
            showNodesAndLinks = !showNodesAndLinks;
        }

        public void SetVisualizationOptions(bool nodes, bool links, bool indices, bool forces)
        {
            showNodes = nodes;
            showLinks = links;
            showNodeIndices = indices;
            showLinkForces = forces;
        }

        public void SetVisualizationOptions(bool nodes, bool links, bool indices, bool forces, bool influenceRadius)
        {
            showNodes = nodes;
            showLinks = links;
            showNodeIndices = indices;
            showLinkForces = forces;
            showInfluenceRadius = influenceRadius;
        }

        private void DrawInfluenceRadius()
        {
            if (nodeManager?.Nodes == null || meshDeformer == null) return;

            float influenceRadius = meshDeformer.InfluenceRadius;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null) continue;

                Vector3 localPosition = nodeManager.Nodes[i].localPosition;
                Vector3 worldPosition = owner.TransformPoint(localPosition);

                // Draw influence radius as wireframe sphere with transparent color
                DrawWireSphere(worldPosition, influenceRadius, influenceRadiusColor);
            }
        }

        // Get visualization statistics
        public string GetVisualizationStats()
        {
            int nodeCount = nodeManager?.Nodes?.Count ?? 0;
            int linkCount = beams?.Count ?? 0;
            int pinnedNodes = 0;

            if (nodeManager?.IsPinned != null)
            {
                for (int i = 0; i < nodeManager.IsPinned.Count; i++)
                {
                    if (nodeManager.IsPinned[i]) pinnedNodes++;
                }
            }

            return $"Nodes: {nodeCount} (Pinned: {pinnedNodes}) | Links: {linkCount}";
        }

        #region Internal Pressure Simulation
        public Vector3 GetPressureCenter()
        {
            if (nodeManager?.Nodes == null || nodeManager.Nodes.Count == 0)
                return owner.position; // Return world space center

            Vector3 worldCenter = Vector3.zero;
            int count = 0;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] != null)
                {
                    worldCenter += owner.TransformPoint(nodeManager.Nodes[i].localPosition); // Convert to world space
                    count++;
                }
            }

            return count > 0 ? worldCenter / count : owner.position;
        }

        public void SetInternalPressure(bool enabled, float pressureInPascals, float forceMultiplier)
        {
            internalPressure = enabled ? pressureInPascals : 0f;
            pressureForceMultiplier = forceMultiplier;
        }

        public void SetShowPressureForces(bool show)
        {
            visualizeForces = show;
        }

        private void ApplyPressureForces(float deltaTime)
        {
            if (internalPressure <= 0 || nodeManager?.Nodes == null)
                return;

            // Center (world space) for radial direction  
            Vector3 worldCenter = GetPressureCenter();
            float pressurePa = internalPressure * pressureForceMultiplier; // Pa = N/m^2

            // Damping attenuates but never zeroes
            float dampingFactor = 1f / (1f + Mathf.Max(0f, pressureDamping));

            // Estimate an effective body radius and per-node surface area using a bounding sphere (in world space)
            int n = Mathf.Max(1, nodeManager.Nodes.Count);
            float sumR = 0f;
            int rCount = 0;
            for (int i = 0; i < n; i++)
            {
                if (nodeManager.Nodes[i] == null) continue;
                Vector3 worldNodePos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                float d = Vector3.Distance(worldNodePos, worldCenter); // World space distance
                if (!float.IsNaN(d) && !float.IsInfinity(d)) { sumR += d; rCount++; }
            }
            float radius = Mathf.Max(1e-3f, sumR / Mathf.Max(1, rCount));
            float surfaceArea = 4f * Mathf.PI * radius * radius; // m^2
            float perNodeArea = surfaceArea / n;                  // m^2 per node

            // Per-step stability clamp: limit displacement due to pressure to avoid exploding
            float dt = Mathf.Max(1e-6f, deltaTime);
            float maxDisp = Mathf.Clamp(radius * 0.01f, 0.0005f, 0.02f); // 1% of radius per step (0.5mm..2cm)
            float aMax = maxDisp / (dt * dt);                            // m/s^2

            float accumForce = 0f;
            float minForce = float.MaxValue;
            float maxForce = 0f;
            int forceCount = 0;

            for (int i = 0; i < n; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 worldNodePos = owner.TransformPoint(nodeManager.Nodes[i].localPosition); // World space
                Vector3 worldDir = (worldNodePos - worldCenter).normalized; // Outward direction in world space

                // Base physical force: P * area
                float baseForceMag = pressurePa * perNodeArea * dampingFactor; // N
                // Clamp to stability cap
                float mNode = Mathf.Max(1e-6f, nodeMasses[i]); // Per-node mass
                float Fmax = aMax * mNode;                    // N
                float forceMag = Mathf.Min(baseForceMag, Fmax);

                Vector3 worldForce = worldDir * forceMag; // Force in world space
                Vector3 localForce = owner.InverseTransformDirection(worldForce); // Convert to local for apply method
                ApplyForceToNode(i, localForce); // Apply local force (will be converted back in method)

                accumForce += forceMag;
                forceCount++;
                if (forceMag < minForce) minForce = forceMag;
                if (forceMag > maxForce) maxForce = forceMag;

                if (visualizeForces)
                {
                    // Visualization in world space
                    Debug.DrawRay(worldNodePos, worldDir * Mathf.Min(0.1f, forceMag * 1e-5f), Color.cyan, dt);
                }
            }
        }

        public void SetInternalPressure(bool enabled, float pressureInPascals, float forceMultiplier, float damping = 0.3f)
        {
            internalPressure = enabled ? pressureInPascals : 0f;
            pressureForceMultiplier = forceMultiplier;
            pressureDamping = damping;
        }
        #endregion
        private SolverComponent GetSolverComponentFromCollider(Collider collider)
        {
            // Check if the collider has a SolverComponent directly
            SolverComponent solverComp = collider.GetComponent<SolverComponent>();
            if (solverComp != null) return solverComp;

            // Check in parent
            solverComp = collider.GetComponentInParent<SolverComponent>();
            return solverComp;
        }
    }
}