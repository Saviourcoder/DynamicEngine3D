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
        public bool visualizeForces = false;
        public List<float> nodeMasses = new List<float>();
        public float compliance = 1e-3f;
        public float defaultDamping = 0.3f;
        public float deformationScale = 1f;
        public float maxDeformation = 1f;
        private float restThreshold = 0.05f;
        private Truss trussAsset;
        private Matter matterAsset;

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

        #region Genrerate Methods
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
                nodeObj.transform.localRotation = Quaternion.identity; // Add this line
                //nodeObj.hideFlags = HideFlags.HideAndDontSave;
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

            float simScale = SceneSettings.Instance != null ? SceneSettings.Instance.SimulationTimeScale : 1.0f;
            float fullDt = Time.fixedDeltaTime * simScale;
            float subDt = fullDt / numSubSteps;

            for (int step = 0; step < numSubSteps; step++)
            {
                SolveSubStep(subDt);
            }

            DrawNodesAndLinks();
        }

        #region SolveSubStep
        private void SolveSubStep(float dt)
        {
            float restitutionValue = 0.02f;
            float dynamicFrictionValue = matterAsset != null ? matterAsset.DynamicFriction : 0.4f;
            float staticFrictionValue = matterAsset != null ? matterAsset.StaticFriction : 0.5f;
            float dampingValue = 0.95f;

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

                Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                Vector3 prevWorldPos = nodeManager.PreviousPositions[i];
                Vector3 worldVelocity = (currentWorldPos - prevWorldPos) / dt;

                float velocityMagnitude = worldVelocity.magnitude;

                float gravityValue = SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : 9.81f;
                Vector3 worldGravity = Vector3.down * gravityValue;
                Vector3 worldAcceleration = worldGravity;

                float dampingPerSecond = 1f - dampingValue;
                float dampingFactor = Mathf.Pow(1.0f - dampingPerSecond, dt);
                worldVelocity *= dampingFactor;

                // Only apply rest detection for very slow movement
                if (velocityMagnitude < restThreshold)
                {
                    worldVelocity *= 0.5f;
                }

                Vector3 newWorldPosition = currentWorldPos + worldVelocity * dt + worldAcceleration * dtSquared;

                if (float.IsNaN(newWorldPosition.x) || float.IsNaN(newWorldPosition.y) || float.IsNaN(newWorldPosition.z))
                {
                    newWorldPosition = currentWorldPos;
                    Debug.LogWarning($"NaN detected for node {i}, keeping at {currentWorldPos}");
                }
                nodeManager.PredictedPositions[i] = newWorldPosition;
            }

            if (internalPressure > 0)
            {

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
                if (motionDistance < epsilon) continue;

                Vector3 worldDirection = worldMotion.normalized;

                // Ensure a worldAcceleration is available for collision response calculations
                float gravityValue = SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : 9.81f;
                Vector3 worldAcceleration = Vector3.down * gravityValue;

                int hitCount = Physics.OverlapSphereNonAlloc(currentWorldPos, nodeManager.GetNodeRadius(i), overlapResults, collisionLayerMask);
                for (int j = 0; j < hitCount; j++)
                {
                    Collider col = overlapResults[j];
                    if (col == nodeManager.Colliders[i] || col.transform == nodeManager.Nodes[i]) continue;

                    var solverComp = GetSolverComponentFromCollider(col);
                    var otherSolver = solverComp != null ? solverComp.solver : null;
                    if (otherSolver != null) continue;

                    if (col is TerrainCollider terrainCol)
                    {
                        Terrain terrain = terrainCol.GetComponent<Terrain>();
                        if (terrain != null)
                        {
                            float height = terrain.SampleHeight(currentWorldPos) + terrain.transform.position.y;
                            if (currentWorldPos.y < height + nodeManager.GetNodeRadius(i))
                            {
                                // Position correction
                                Vector3 worldNewPos = currentWorldPos;
                                worldNewPos.y = height + nodeManager.GetNodeRadius(i) + epsilon;
                                predictedWorldPos = worldNewPos;

                                // Add friction for terrain
                                Vector3 velocity = (predictedWorldPos - currentWorldPos) / dt;
                                Vector3 normal = Vector3.up; // Assuming flat terrain; for accurate, use terrain normal
                                float normalVelocity = Vector3.Dot(velocity, normal);
                                if (normalVelocity >= 0) continue;

                                Vector3 tangentVelocity = velocity - normal * normalVelocity;
                                Vector3 tangent = (tangentVelocity.magnitude > 0.001f) ? tangentVelocity.normalized : Vector3.zero;
                                float tangentMagnitude = tangentVelocity.magnitude;

                                float invMassA = nodeManager.IsPinned[i] ? 0f : 1f / nodeMasses[i];
                                float jn = -(1 + restitutionValue) * normalVelocity / invMassA;

                                float jtRequired = -Vector3.Dot(velocity, tangent) / invMassA;
                                float maxStatic = staticFrictionValue * Mathf.Abs(jn);
                                float jt;
                                if (Mathf.Abs(jtRequired) <= maxStatic)
                                {
                                    jt = jtRequired; // Static friction
                                }
                                else
                                {
                                    jt = Mathf.Sign(jtRequired) * dynamicFrictionValue * Mathf.Abs(jn); // Dynamic friction
                                }

                                Vector3 impulse = normal * jn + tangent * jt;
                                Vector3 postCollisionVelocity = velocity + impulse * invMassA;
                                postCollisionVelocity *= dampingValue;
                                predictedWorldPos = currentWorldPos + postCollisionVelocity * dt + worldAcceleration * dtSquared;

                                nodeManager.PredictedPositions[i] = predictedWorldPos;
                                collisionPoints.Add(currentWorldPos);

                                if (visualizeForces)
                                {
                                    Debug.DrawRay(currentWorldPos, impulse.normalized * 0.2f, Color.red, dt);
                                }
                            }
                        }
                    }
                    else
                    {
                        if (col is BoxCollider || col is SphereCollider || col is CapsuleCollider || col is MeshCollider)
                        {
                            // For non-convex mesh colliders, use raycast instead of ClosestPoint
                            if (col is MeshCollider meshCol && !meshCol.convex)
                            {
                                Vector3 direction = (predictedWorldPos - currentWorldPos).normalized;
                                if (col.Raycast(new Ray(currentWorldPos - direction * nodeManager.GetNodeRadius(i), direction),
                                    out RaycastHit meshHit, nodeManager.GetNodeRadius(i) * 2f))
                                {
                                    // Position correction
                                    predictedWorldPos = meshHit.point + meshHit.normal * (nodeManager.GetNodeRadius(i) + epsilon);

                                    // Add friction
                                    Vector3 velocity = (predictedWorldPos - currentWorldPos) / dt;
                                    Vector3 normal = meshHit.normal;
                                    float normalVelocity = Vector3.Dot(velocity, normal);
                                    if (normalVelocity >= 0) continue;

                                    Vector3 tangentVelocity = velocity - normal * normalVelocity;
                                    Vector3 tangent = (tangentVelocity.magnitude > 0.001f) ? tangentVelocity.normalized : Vector3.zero;
                                    float tangentMagnitude = tangentVelocity.magnitude;

                                    float invMassA = nodeManager.IsPinned[i] ? 0f : 1f / nodeMasses[i];
                                    float jn = -(1 + restitutionValue) * normalVelocity / invMassA;

                                    float jtRequired = -Vector3.Dot(velocity, tangent) / invMassA;
                                    float maxStatic = staticFrictionValue * Mathf.Abs(jn);
                                    float jt;
                                    if (Mathf.Abs(jtRequired) <= maxStatic)
                                    {
                                        jt = jtRequired; // Static friction
                                    }
                                    else
                                    {
                                        jt = Mathf.Sign(jtRequired) * dynamicFrictionValue * Mathf.Abs(jn); // Dynamic friction
                                    }

                                    Vector3 impulse = normal * jn + tangent * jt;
                                    Vector3 postCollisionVelocity = velocity + impulse * invMassA;
                                    postCollisionVelocity *= dampingValue;
                                    predictedWorldPos = currentWorldPos + postCollisionVelocity * dt + worldAcceleration * dtSquared;

                                    nodeManager.PredictedPositions[i] = predictedWorldPos;
                                    collisionPoints.Add(meshHit.point);

                                    if (visualizeForces)
                                    {
                                        Debug.DrawRay(meshHit.point, impulse.normalized * 0.2f, Color.red, dt);
                                    }
                                }
                            }
                            else
                            {
                                Vector3 closestPoint = col.ClosestPoint(currentWorldPos);
                                if (closestPoint != currentWorldPos)
                                {
                                    Vector3 worldDirectionToSurface = (closestPoint - currentWorldPos).normalized;
                                    float penetration = nodeManager.GetNodeRadius(i) - Vector3.Distance(closestPoint, currentWorldPos);
                                    predictedWorldPos += worldDirectionToSurface * (penetration + epsilon);

                                    // Normal points outward from collider
                                    Vector3 normal = worldDirectionToSurface; // Direction to push out

                                    // Add friction
                                    Vector3 velocity = (predictedWorldPos - currentWorldPos) / dt;
                                    float normalVelocity = Vector3.Dot(velocity, normal);
                                    if (normalVelocity >= 0) continue;

                                    Vector3 tangentVelocity = velocity - normal * normalVelocity;
                                    Vector3 tangent = (tangentVelocity.magnitude > 0.001f) ? tangentVelocity.normalized : Vector3.zero;
                                    float tangentMagnitude = tangentVelocity.magnitude;

                                    float invMassA = nodeManager.IsPinned[i] ? 0f : 1f / nodeMasses[i];
                                    float jn = -(1 + restitutionValue) * normalVelocity / invMassA;

                                    float jtRequired = -Vector3.Dot(velocity, tangent) / invMassA;
                                    float maxStatic = staticFrictionValue * Mathf.Abs(jn);
                                    float jt;
                                    if (Mathf.Abs(jtRequired) <= maxStatic)
                                    {
                                        jt = jtRequired; // Static friction
                                    }
                                    else
                                    {
                                        jt = Mathf.Sign(jtRequired) * dynamicFrictionValue * Mathf.Abs(jn); // Dynamic friction
                                    }

                                    Vector3 impulse = normal * jn + tangent * jt;
                                    Vector3 postCollisionVelocity = velocity + impulse * invMassA;
                                    postCollisionVelocity *= dampingValue;
                                    predictedWorldPos = currentWorldPos + postCollisionVelocity * dt + worldAcceleration * dtSquared;

                                    nodeManager.PredictedPositions[i] = predictedWorldPos;
                                    collisionPoints.Add(currentWorldPos);

                                    if (visualizeForces)
                                    {
                                        Debug.DrawRay(currentWorldPos, impulse.normalized * 0.2f, Color.red, dt);
                                    }
                                }
                            }
                        }
                    }
                }

                // Sweep-based environment collision
                if (Physics.SphereCast(currentWorldPos, nodeManager.GetNodeRadius(i), worldDirection, out RaycastHit hit, motionDistance, collisionLayerMask))
                {
                    var solverComp = GetSolverComponentFromCollider(hit.collider);
                    var otherSolver = solverComp != null ? solverComp.solver : null;
                    if (otherSolver != null) continue;

                    if (hit.distance < motionDistance)
                    {
                        Vector3 worldHitPos = currentWorldPos + worldDirection * hit.distance;
                        Vector3 worldNewPos = hit.point + hit.normal * (nodeManager.GetNodeRadius(i) + epsilon);

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

                        // Compute incoming velocity
                        Vector3 velocity = worldMotion / dt;
                        Vector3 normal = hit.normal;
                        float normalVelocity = Vector3.Dot(velocity, normal);
                        if (normalVelocity >= 0) continue;

                        Vector3 tangentVelocity = velocity - normal * normalVelocity;
                        Vector3 tangent = (tangentVelocity.magnitude > 0.001f) ? tangentVelocity.normalized : Vector3.zero;
                        float tangentMagnitude = tangentVelocity.magnitude;

                        float invMassA = nodeManager.IsPinned[i] ? 0f : 1f / nodeMasses[i];
                        float jn = -(1 + restitutionValue) * normalVelocity / invMassA;

                        float jtRequired = -Vector3.Dot(velocity, tangent) / invMassA;
                        float maxStatic = staticFrictionValue * Mathf.Abs(jn);
                        float jt;
                        if (Mathf.Abs(jtRequired) <= maxStatic)
                        {
                            jt = jtRequired; // Static friction: stick
                        }
                        else
                        {
                            jt = Mathf.Sign(jtRequired) * dynamicFrictionValue * Mathf.Abs(jn); // Dynamic friction: slide
                        }

                        Vector3 impulse = normal * jn + tangent * jt;
                        Vector3 postCollisionVelocity = velocity + impulse * invMassA;
                        postCollisionVelocity *= dampingValue;

                        nodeManager.PreviousPositions[i] = currentWorldPos - postCollisionVelocity * dt;

                        collisionPoints.Add(hit.point);

                        if (visualizeForces)
                        {
                            Debug.DrawRay(hit.point, impulse.normalized * 0.2f, Color.red, dt);
                        }
                    }
                }
                nodeManager.PredictedPositions[i] = predictedWorldPos;

                // Face-node collisions
                if (trussAsset != null)
                {
                    foreach (var face in trussAsset.GetTrussFaces())
                    {
                        if (face.nodeA >= nodeManager.Nodes.Count || face.nodeB >= nodeManager.Nodes.Count || face.nodeC >= nodeManager.Nodes.Count ||
                            nodeManager.Nodes[face.nodeA] == null || nodeManager.Nodes[face.nodeB] == null || nodeManager.Nodes[face.nodeC] == null)
                            continue;

                        if (i == face.nodeA || i == face.nodeB || i == face.nodeC) continue;

                        Vector3 worldPosA = owner.TransformPoint(nodeManager.InitialPositions[face.nodeA]);
                        Vector3 worldPosB = owner.TransformPoint(nodeManager.InitialPositions[face.nodeB]);
                        Vector3 worldPosC = owner.TransformPoint(nodeManager.InitialPositions[face.nodeC]);
                        Vector3 worldNodePos = nodeManager.PredictedPositions[i];

                        Vector3 worldNormal = Vector3.Cross(worldPosB - worldPosA, worldPosC - worldPosA).normalized;
                        float distanceToPlane = Vector3.Dot(worldNodePos - worldPosA, worldNormal);

                        if (Mathf.Abs(distanceToPlane) < nodeManager.GetNodeRadius(i))
                        {
                            Vector3 projectedPoint = worldNodePos - distanceToPlane * worldNormal;

                            if (IsPointInTriangle(projectedPoint, worldPosA, worldPosB, worldPosC))
                            {
                                float w = nodeManager.IsPinned[i] ? 0f : 1f / nodeMasses[i];
                                float constraintCompliance = compliance / dtSquared;
                                float lambda = -distanceToPlane / (w + constraintCompliance);
                                Vector3 worldCorrection = worldNormal * lambda;

                                if (float.IsNaN(worldCorrection.x) || float.IsNaN(worldCorrection.y) || float.IsNaN(worldCorrection.z))
                                {
                                    worldCorrection = Vector3.zero;
                                }
                                else
                                {
                                    nodeManager.PredictedPositions[i] += worldCorrection;
                                    Vector3 worldCollisionPoint = nodeManager.PredictedPositions[i] - worldCorrection;
                                    collisionPoints.Add(worldCollisionPoint);
                                }
                            }
                        }
                    }
                }
            }

            // Step 3: XPBD constraint solving with plastic deformation
            int constraintIterations = SceneSettings.Instance != null ? SceneSettings.Instance.ConstraintIterations : 10;
            for (int iter = 0; iter < constraintIterations; iter++)
            {
                for (int i = 0; i < beams.Count; i++)
                {
                    Beam beam = beams[i];

                    if (!beam.isActive) continue;

                    // Get the correct managers and transforms based on whether this is a cross-body beam
                    NodeManager managerA, managerB;
                    Transform ownerA, ownerB;
                    List<float> massesA, massesB;

                    if (beam.IsCrossBody)
                    {
                        // Cross-body constraint - nodes are in different bodies
                        if (beam.bodyA?.solver?.nodeManager == null || beam.bodyB?.solver?.nodeManager == null)
                            continue;

                        managerA = beam.bodyA.solver.nodeManager;
                        managerB = beam.bodyB.solver.nodeManager;
                        ownerA = beam.bodyA.transform;
                        ownerB = beam.bodyB.transform;
                        massesA = beam.bodyA.solver.nodeMasses;
                        massesB = beam.bodyB.solver.nodeMasses;

                        if (beam.nodeA >= managerA.Nodes.Count || beam.nodeB >= managerB.Nodes.Count ||
                            managerA.Nodes[beam.nodeA] == null || managerB.Nodes[beam.nodeB] == null)
                            continue;

                        if (managerA.IsPinned[beam.nodeA] && managerB.IsPinned[beam.nodeB]) continue;
                    }
                    else
                    {
                        // Same-body beam
                        managerA = nodeManager;
                        managerB = nodeManager;
                        ownerA = owner;
                        ownerB = owner;
                        massesA = nodeMasses;
                        massesB = nodeMasses;

                        if (beam.nodeA >= nodeManager.Nodes.Count || beam.nodeB >= nodeManager.Nodes.Count ||
                            nodeManager.Nodes[beam.nodeA] == null || nodeManager.Nodes[beam.nodeB] == null)
                            continue;

                        if (nodeManager.IsPinned[beam.nodeA] && nodeManager.IsPinned[beam.nodeB]) continue;
                    }

                    Vector3 worldPosA = managerA.PredictedPositions[beam.nodeA];
                    Vector3 worldPosB = managerB.PredictedPositions[beam.nodeB];
                    Vector3 delta = worldPosB - worldPosA;
                    float currentLength = delta.magnitude;

                    // Skip near-zero length beams
                    if (currentLength < 0.0001f)
                    {
                        continue;
                    }

                    // Convert rest length from local to world space
                    float worldRestLength;
                    if (beam.IsCrossBody)
                    {
                        // Cross-body beams store rest length in world units already
                        worldRestLength = beam.restLength;
                    }
                    else
                    {
                        // Same-body beams need local-to-world conversion
                        Vector3 localRestVector = Vector3.right * beam.restLength;
                        Vector3 worldRestVector = owner.TransformDirection(localRestVector);
                        worldRestLength = worldRestVector.magnitude;
                    }

                    float constraint = currentLength - worldRestLength;
                    float strain = Mathf.Abs(constraint) / worldRestLength;

                    // If strain exceeds plasticity threshold, permanently deform the beam (only for same-body beams)
                    if (!beam.IsCrossBody && strain > beam.plasticityThreshold)
                    {
                        // Calculate how much plastic deformation to apply
                        float excessStrain = strain - beam.plasticityThreshold;
                        float plasticDeformation = excessStrain * beam.plasticityRate * dt;

                        // Clamp to maximum deformation
                        float totalDeformation = Mathf.Abs(beam.restLength - beam.originalRestLength) / beam.originalRestLength;

                        if (totalDeformation < beam.maxDeformation)
                        {
                            // Apply plastic deformation by permanently changing rest length
                            float deformationAmount = plasticDeformation * beam.restLength;

                            // Deform in the direction of the strain
                            if (constraint > 0) // Stretching
                            {
                                beam.restLength += deformationAmount;
                            }
                            else // Compression
                            {
                                beam.restLength -= deformationAmount;
                            }

                            // Clamp rest length to reasonable bounds
                            float minRestLength = beam.originalRestLength * (1f - beam.maxDeformation);
                            float maxRestLength = beam.originalRestLength * (1f + beam.maxDeformation);
                            beam.restLength = Mathf.Clamp(beam.restLength, minRestLength, maxRestLength);

                            // Update world rest length for current iteration
                            Vector3 localRestVector = Vector3.right * beam.restLength;
                            Vector3 worldRestVector = owner.TransformDirection(localRestVector);
                            worldRestLength = worldRestVector.magnitude;

                            // Recalculate constraint with new rest length
                            constraint = currentLength - worldRestLength;
                        }
                    }
                    // ===== END PLASTIC DEFORMATION LOGIC =====

                    Vector3 gradient = delta / currentLength;

                    bool pinnedA = beam.IsCrossBody ? managerA.IsPinned[beam.nodeA] : nodeManager.IsPinned[beam.nodeA];
                    bool pinnedB = beam.IsCrossBody ? managerB.IsPinned[beam.nodeB] : nodeManager.IsPinned[beam.nodeB];

                    float wA = pinnedA ? 0f : 1f / massesA[beam.nodeA];
                    float wB = pinnedB ? 0f : 1f / massesB[beam.nodeB];
                    float wSum = wA + wB;

                    if (wSum == 0f) continue;

                    float effectiveCompliance = beam.compliance / dtSquared;
                    float lambda = -constraint / (wSum + effectiveCompliance);

                    float maxLambda = worldRestLength * 0.5f;
                    lambda = Mathf.Clamp(lambda, -maxLambda, maxLambda);

                    beam.lagrangeMultiplier += lambda;

                    Vector3 correction = gradient * lambda;

                    float maxCorrection = worldRestLength * 0.1f;
                    if (correction.magnitude > maxCorrection)
                    {
                        correction = correction.normalized * maxCorrection;
                    }

                    if (float.IsNaN(correction.x) || float.IsNaN(correction.y) || float.IsNaN(correction.z))
                    {
                        correction = Vector3.zero;
                    }

                    // Apply corrections to the appropriate managers
                    if (beam.IsCrossBody)
                    {
                        managerA.PredictedPositions[beam.nodeA] -= wA * correction;
                        managerB.PredictedPositions[beam.nodeB] += wB * correction;
                    }
                    else
                    {
                        nodeManager.PredictedPositions[beam.nodeA] -= wA * correction;
                        nodeManager.PredictedPositions[beam.nodeB] += wB * correction;
                    }

                    if (visualizeForces && constraint != 0)
                    {
                        float forceMagnitude = Mathf.Abs(beam.lagrangeMultiplier) / dtSquared;
                        Color forceColor = beam.IsCrossBody ? Color.cyan :
                                          (strain > beam.plasticityThreshold ? Color.magenta :
                                          (forceMagnitude > 0 ? Color.red : Color.blue));
                        Debug.DrawLine(worldPosA, worldPosB, forceColor, 0.1f);
                    }
                }
            }

            // Step 4: Finalize positions and apply additional stabilization
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
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

                // Final collision detection
                Collider nodeCollider = nodeManager.Colliders[i];

                if (nodeCollider == null) continue;

                Collider[] colliders = Physics.OverlapSphere(predictedWorldPos, nodeManager.GetNodeRadius(i));
                bool inCollision = false;
                Vector3 worldNormal = Vector3.zero;

                foreach (var collider in colliders)
                {
                    if (collider == nodeCollider || collider.transform == nodeManager.Nodes[i]) continue;
                    int otherNodeIndex = nodeManager.FindIndex(n => n != null && n == collider.transform);
                    if (otherNodeIndex != -1) continue;

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
                        predictedWorldPos += direction * distance;
                        nodeManager.PredictedPositions[i] = predictedWorldPos;
                        collisionPoints.Add(predictedWorldPos);

                        // Add friction for final collision
                        Vector3 velocity = worldMotion / dt;
                        float normalVelocity = Vector3.Dot(velocity, worldNormal);
                        if (normalVelocity >= 0) break;

                        Vector3 tangentVelocity = velocity - worldNormal * normalVelocity;
                        Vector3 tangent = (tangentVelocity.magnitude > 0.001f) ? tangentVelocity.normalized : Vector3.zero;

                        float invMassA = nodeManager.IsPinned[i] ? 0f : 1f / nodeMasses[i];
                        float jn = -(1 + restitutionValue) * normalVelocity / invMassA;

                        float jtRequired = -Vector3.Dot(velocity, tangent) / invMassA;
                        float maxStatic = staticFrictionValue * Mathf.Abs(jn);
                        float jt;
                        if (Mathf.Abs(jtRequired) <= maxStatic)
                        {
                            jt = jtRequired; // Static friction
                        }
                        else
                        {
                            jt = Mathf.Sign(jtRequired) * dynamicFrictionValue * Mathf.Abs(jn); // Dynamic friction
                        }

                        Vector3 impulse = worldNormal * jn + tangent * jt;
                        Vector3 postCollisionVelocity = velocity + impulse * invMassA;
                        postCollisionVelocity *= dampingValue;

                        if (postCollisionVelocity.magnitude < restThreshold)
                        {
                            postCollisionVelocity = Vector3.zero;
                        }
                        nodeManager.PreviousPositions[i] = predictedWorldPos - postCollisionVelocity * dt;

                        if (visualizeForces)
                        {
                            Debug.DrawRay(predictedWorldPos, impulse.normalized * 0.2f, Color.red, dt);
                        }

                        break;
                    }
                }

                Vector3 finalLocalPos = owner.InverseTransformPoint(predictedWorldPos);
                nodeManager.Nodes[i].localPosition = finalLocalPos;

                if (!inCollision)
                {
                    nodeManager.PreviousPositions[i] = currentWorldPos;
                }
            }
        }
        #endregion
        #region Collision Methods

        private void DetectSoftbodyCollisions(float dt)
        {
            if (nodeManager?.Nodes == null) return;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 worldPosA = nodeManager.PredictedPositions[i];
                int hitCount = Physics.OverlapSphereNonAlloc(worldPosA, nodeManager.GetNodeRadius(i) * 1.5f, overlapResults, collisionLayerMask);

                for (int j = 0; j < hitCount; j++)
                {
                    Collider otherCollider = overlapResults[j];
                    if (otherCollider.transform == nodeManager.Nodes[i]) continue;

                    var solverComp = GetSolverComponentFromCollider(otherCollider);
                    var otherSolver = solverComp != null ? solverComp.solver : null;
                    if (otherSolver == null || otherSolver == this) continue;

                    Vector3 worldPosB = otherCollider.transform.position;
                    Vector3 delta = worldPosB - worldPosA;
                    float distance = delta.magnitude;
                    float nodeARadius = nodeManager.GetNodeRadius(i);
                    float nodeBRadius = nodeARadius;

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
        #endregion
        #region Helper Methods

        public void DeformMesh(Transform transform)
        {
            if (transform == null) return;
            UpdateNodeRotations();
            meshDeformer.Deform(transform, nodeManager.Nodes, nodeManager.InitialPositions, nodeManager.InitialRotations);
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

            // Convert local force to world space
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

            // Get current world position
            Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[nodeIndex].localPosition);

            // Apply velocity change by modifying previous position
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
                float effectiveDt = Time.fixedDeltaTime * simScale;  // Scale for consistency with solver

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

        internal void GenerateNodesAndBeams(Vector3[] vector3s, Beam[] beams, Transform transform)
        {
            throw new System.NotImplementedException();
        }
        #endregion

        #region   Runtime Visualization
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
        #region Node Rotation

        private void UpdateNodeRotations()
        {
            int nodeCount = nodeManager.Nodes.Count;
            List<List<int>> neighbors = new List<List<int>>(nodeCount);
            for (int i = 0; i < nodeCount; i++)
            {
                neighbors.Add(new List<int>());
            }
            // Build neighbor list from beams
            foreach (var beam in beams)
            {
                neighbors[beam.nodeA].Add(beam.nodeB);
                neighbors[beam.nodeB].Add(beam.nodeA);
            }
            for (int i = 0; i < nodeCount; i++)
            {
                List<int> cluster = new List<int>(neighbors[i]);
                cluster.Add(i);
                if (cluster.Count < 2)
                {
                    Debug.LogWarning($"Node {i} has insufficient cluster size ({cluster.Count}), skipping rotation update.");
                    continue;
                }
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
                if (totalMass < 1e-6f)
                {
                    Debug.LogWarning($"Node {i} cluster has zero total mass, skipping.");
                    continue;
                }
                com_rest /= totalMass;
                com_current /= totalMass;
                // Covariance matrix A (3x3)
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
                // Warm start from previous rotation
                Quaternion q = nodeManager.Nodes[i].localRotation;
                Matrix4x4 R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);
                // Iterative update (Müller 2016)
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
                    if (w < 1e-6f)
                    {
                        break;
                    }
                    Vector3 axis = omega / w;
                    Quaternion deltaQ = Quaternion.AngleAxis(w * Mathf.Rad2Deg, axis);
                    q = deltaQ * q;
                    q = q.normalized;
                    R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);
                }
                // Set local rotation
                nodeManager.Nodes[i].localRotation = q;
            }
        }
    }
}
#endregion