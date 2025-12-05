/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers */
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    public class Solver
    {
        private static bool s_isApplicationQuitting = false;
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
        public float surfaceCollisionThickness = 0.05f;
        #endregion

        // Runtime Visualization
        public bool showNodesAndLinks = false;
        public bool showNodes = true;
        public bool showLinks = true;
        public bool showNodeIndices = false;
        public bool showLinkForces = true;
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
        private static Vector3 GetClosestPointOnTriangleToPoint(Vector3 point, Vector3 a, Vector3 b, Vector3 c, out Vector3 barycentric)
        {
            barycentric = Vector3.zero;

            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 bc = c - b;

            Vector3 ap = point - a;
            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f)
            {
                barycentric = new Vector3(1f, 0f, 0f);
                return a;
            }

            Vector3 bp = point - b;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3)
            {
                barycentric = new Vector3(0f, 1f, 0f);
                return b;
            }

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float vEdge = d1 / (d1 - d3);
                barycentric = new Vector3(1f - vEdge, vEdge, 0f);
                return a + vEdge * ab;
            }

            Vector3 cp = point - c;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6)
            {
                barycentric = new Vector3(0f, 0f, 1f);
                return c;
            }

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w2 = d2 / (d2 - d6);
                barycentric = new Vector3(1f - w2, 0f, w2);
                return a + w2 * ac;
            }

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w2 = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                barycentric = new Vector3(0f, 1f - w2, w2);
                return b + w2 * bc;
            }

            float denom = 1f / (va + vb + vc);
            float v = vb * denom;
            float w = vc * denom;
            barycentric = new Vector3(1f - v - w, v, w);
            return a + ab * v + ac * w;
        }
        private void HandleSelfTriangleNodeCollisions(float dt)
        {
            if (trussAsset == null || trussAsset.GetTrussFaces().Count == 0) return;

            var faces = trussAsset.GetTrussFaces();
            float thickness = surfaceCollisionThickness;
            float restitution = 0.02f;
            float friction = 0.4f;
            float eps = 0.001f;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.IsPinned[i]) continue;

                Vector3 point = nodeManager.PredictedPositions[i];
                Vector3 velNode = (point - nodeManager.PreviousPositions[i]) / dt;
                float invMassNode = 1f / nodeMasses[i];

                foreach (var face in faces)
                {
                    // Skip faces that contain this node (prevents self-sticking)
                    if (face.nodeA == i || face.nodeB == i || face.nodeC == i) continue;

                    Vector3 ta = nodeManager.PredictedPositions[face.nodeA];
                    Vector3 tb = nodeManager.PredictedPositions[face.nodeB];
                    Vector3 tc = nodeManager.PredictedPositions[face.nodeC];

                    Vector3 closest = GetClosestPointOnTriangleToPoint(point, ta, tb, tc, out Vector3 bary);

                    Vector3 delta = point - closest;
                    float dist = delta.magnitude;
                    if (dist >= thickness || dist < 0.001f) continue;

                    Vector3 normal = delta.normalized;

                    // Position correction
                    nodeManager.PredictedPositions[i] = closest + normal * (thickness + eps);

                    // Update velocity after correction
                    point = nodeManager.PredictedPositions[i];
                    velNode = (point - nodeManager.PreviousPositions[i]) / dt;

                    // Triangle velocity
                    Vector3 velTri = bary.x * (ta - nodeManager.PreviousPositions[face.nodeA]) / dt
                                   + bary.y * (tb - nodeManager.PreviousPositions[face.nodeB]) / dt
                                   + bary.z * (tc - nodeManager.PreviousPositions[face.nodeC]) / dt;

                    Vector3 relVel = velNode - velTri;
                    float relNormal = Vector3.Dot(relVel, normal);
                    if (relNormal > 0f) continue;

                    float invMassTri = 0f;
                    if (bary.x > 0.001f) invMassTri += bary.x * bary.x * (nodeManager.IsPinned[face.nodeA] ? 0f : 1f / nodeMasses[face.nodeA]);
                    if (bary.y > 0.001f) invMassTri += bary.y * bary.y * (nodeManager.IsPinned[face.nodeB] ? 0f : 1f / nodeMasses[face.nodeB]);
                    if (bary.z > 0.001f) invMassTri += bary.z * bary.z * (nodeManager.IsPinned[face.nodeC] ? 0f : 1f / nodeMasses[face.nodeC]);

                    float denom = invMassNode + invMassTri;
                    if (denom <= 0.0001f) continue;

                    float jn = -(1f + restitution) * relNormal / denom;

                    Vector3 tangent = relVel - normal * relNormal;
                    float jt = 0f;
                    if (tangent.sqrMagnitude > 0.0001f)
                    {
                        tangent.Normalize();
                        jt = -Vector3.Dot(relVel, tangent) / denom;
                        float maxJt = friction * Mathf.Abs(jn);
                        jt = Mathf.Clamp(jt, -maxJt, maxJt);
                    }

                    Vector3 impulse = normal * jn + tangent * jt;

                    // Direct apply (self)
                    ApplyWorldForceToNode(i, impulse / dt);

                    if (bary.x > 0.001f) ApplyWorldForceToNode(face.nodeA, -impulse * bary.x / dt);
                    if (bary.y > 0.001f) ApplyWorldForceToNode(face.nodeB, -impulse * bary.y / dt);
                    if (bary.z > 0.001f) ApplyWorldForceToNode(face.nodeC, -impulse * bary.z / dt);
                }
            }
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

            // PHASE 1: Detect softbody-to-softbody collisions
            DetectSoftbodyCollisions(dt);

            // PHASE 2: Apply collision forces from other softbodies
            ApplyPendingCollisionForces();
            HandleSelfTriangleNodeCollisions(dt);

            // Step 2: Environment collision detection 
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
                                Vector3 normal = Vector3.up;
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

            // Step 3: Beam constraints (XPBD) - FIXED PER-BEAM DAMPING (sign was inverted before!)
            // Now correctly damps instead of amplifying → no more rapid expansion

            int constraintIters = SceneSettings.Instance != null
                ? SceneSettings.Instance.ConstraintIterations // use whatever your scene settings expose, or hardcode e.g. 16-32
                : 20;

            for (int iter = 0; iter < constraintIters; iter++)
            {
                foreach (var beam in beams)
                {
                    if (!beam.isActive) continue;

                    // Resolve managers / masses / positions for same-body or cross-body beams
                    NodeManager managerA = beam.IsCrossBody ? beam.bodyA.solver.nodeManager : nodeManager;
                    NodeManager managerB = beam.IsCrossBody ? beam.bodyB.solver.nodeManager : nodeManager;
                    List<float> massesA = beam.IsCrossBody ? beam.bodyA.solver.nodeMasses : nodeMasses;
                    List<float> massesB = beam.IsCrossBody ? beam.bodyB.solver.nodeMasses : nodeMasses;

                    Vector3 posA = managerA.PredictedPositions[beam.nodeA];
                    Vector3 posB = managerB.PredictedPositions[beam.nodeB];
                    Vector3 delta = posB - posA;
                    float currentLength = delta.magnitude;
                    if (currentLength < 0.001f) continue;

                    Vector3 gradient = delta / currentLength;

                    // World-space rest length (handles non-uniform scale for same-body beams)
                    float worldRestLength = beam.IsCrossBody
                        ? beam.restLength
                        : owner.TransformDirection(Vector3.right * beam.restLength).magnitude;

                    float constraint = currentLength - worldRestLength;

                    // === PLASTIC DEFORMATION (unchanged - only for same-body beams) ===
                    float strain = Mathf.Abs(constraint) / worldRestLength;
                    if (!beam.IsCrossBody && strain > beam.plasticityThreshold)
                    {
                        float excessStrain = strain - beam.plasticityThreshold;
                        float plasticDeformation = excessStrain * beam.plasticityRate * dt;

                        float totalDeformation = Mathf.Abs(beam.restLength - beam.originalRestLength) / beam.originalRestLength;
                        if (totalDeformation < beam.maxDeformation)
                        {
                            float deformationAmount = plasticDeformation * beam.restLength;
                            if (constraint > 0f) // stretching
                                beam.restLength += deformationAmount;
                            else // compression
                                beam.restLength -= deformationAmount;

                            beam.restLength = Mathf.Clamp(beam.restLength,
                                beam.originalRestLength * (1f - beam.maxDeformation),
                                beam.originalRestLength * (1f + beam.maxDeformation));

                            // Recompute world rest length after plastic flow
                            worldRestLength = beam.IsCrossBody
                                ? beam.restLength
                                : owner.TransformDirection(Vector3.right * beam.restLength).magnitude;

                            constraint = currentLength - worldRestLength;
                        }
                    }
                    // === END PLASTIC DEFORMATION ===

                    // === PER-BEAM STRUCTURAL DAMPING (CORRECTED SIGN) ===
                    Vector3 prevA = managerA.PreviousPositions[beam.nodeA];
                    Vector3 prevB = managerB.PreviousPositions[beam.nodeB];
                    Vector3 velA = (posA - prevA) / dt;
                    Vector3 velB = (posB - prevB) / dt;

                    // Critical fix: velB - velA (positive = separating / length increasing)
                    float relVelAlong = Vector3.Dot(velB - velA, gradient);

                    // Classic PBD/XPBD artificial damping term - adds velocity-dependent bias
                    // No * dt here because most production PBD/XPBD implementations (Obi, etc.) tune it this way
                    // and it works beautifully with 0–0.5 range. If you want strict frame-rate independence,
                    // multiply by dt and increase the damping value ~50-100×.
                    float dampingTerm = beam.damping * relVelAlong;
                    float effectiveConstraint = constraint + dampingTerm;
                    // === END DAMPING ===

                    // Standard XPBD distance constraint
                    bool pinnedA = beam.IsCrossBody ? managerA.IsPinned[beam.nodeA] : nodeManager.IsPinned[beam.nodeA];
                    bool pinnedB = beam.IsCrossBody ? managerB.IsPinned[beam.nodeB] : nodeManager.IsPinned[beam.nodeB];

                    float wA = pinnedA ? 0f : 1f / massesA[beam.nodeA];
                    float wB = pinnedB ? 0f : 1f / massesB[beam.nodeB];
                    float wSum = wA + wB;
                    if (wSum <= 0f) continue;

                    float effectiveCompliance = beam.compliance / (dt * dt);
                    float lambda = -effectiveConstraint / (wSum + effectiveCompliance);

                    // Safety clamps (already in your original code)
                    float maxLambda = worldRestLength * 0.5f;
                    lambda = Mathf.Clamp(lambda, -maxLambda, maxLambda);

                    beam.lagrangeMultiplier += lambda;

                    Vector3 correction = gradient * lambda;
                    float maxCorrection = worldRestLength * 0.1f;
                    if (correction.magnitude > maxCorrection)
                        correction = correction.normalized * maxCorrection;

                    // Apply correction
                    managerA.PredictedPositions[beam.nodeA] -= wA * correction;
                    managerB.PredictedPositions[beam.nodeB] += wB * correction;

                    // Optional link force visualization
                    if (showLinkForces)
                    {
                        Vector3 mid = (posA + posB) * 0.5f;
                        Color forceCol = constraint > 0f ? Color.red : Color.cyan;
                        Debug.DrawLine(mid, mid + correction * forceVisualizationScale * 100f, forceCol, dt);
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
            float restitution = 0.02f;
            float friction = 0.4f;
            float eps = 0.001f;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.IsPinned[i]) continue;

                Vector3 worldPos = nodeManager.PredictedPositions[i];
                float nodeRadius = nodeManager.GetNodeRadius(i);

                float broadphaseRadius = nodeRadius + surfaceCollisionThickness * 2f + 0.1f;

                int hitCount = Physics.OverlapSphereNonAlloc(worldPos, broadphaseRadius, overlapResults, collisionLayerMask);

                for (int j = 0; j < hitCount; j++)
                {
                    Collider col = overlapResults[j];
                    if (col.transform.IsChildOf(owner)) continue; // skip own body

                    SolverComponent solverComp = GetSolverComponentFromCollider(col);
                    Solver otherSolver = solverComp?.solver;

                    if (otherSolver == null) continue;

                    // === CRITICAL FIX: prevent double-application of forces (both sphere and triangle) ===
                    if (solverID > otherSolver.solverID) continue;

                    int otherNodeIndex = otherSolver.nodeManager.FindIndex(t => t == col.transform);

                    if (otherNodeIndex >= 0)
                    {
                        Vector3 otherPos = otherSolver.nodeManager.PredictedPositions[otherNodeIndex];
                        float otherRadius = otherSolver.nodeManager.GetNodeRadius(otherNodeIndex);
                        float dist = Vector3.Distance(worldPos, otherPos);

                        if (dist < nodeRadius + otherRadius && dist > 0.001f)
                        {
                            Vector3 delta = worldPos - otherPos;
                            Vector3 normal = delta.normalized;

                            Vector3 velA = (worldPos - nodeManager.PreviousPositions[i]) / dt;
                            Vector3 velB = !otherSolver.nodeManager.IsPinned[otherNodeIndex]
                                ? (otherPos - otherSolver.nodeManager.PreviousPositions[otherNodeIndex]) / dt
                                : Vector3.zero;

                            Vector3 relVel = velA - velB;
                            float relNormalVel = Vector3.Dot(relVel, normal);

                            float invMassA = 1f / nodeMasses[i];
                            float invMassB = !otherSolver.nodeManager.IsPinned[otherNodeIndex] ? 1f / otherSolver.nodeMasses[otherNodeIndex] : 0f;

                            float jn = -(1 + restitution) * relNormalVel / (invMassA + invMassB);

                            Vector3 tangent = relVel - normal * relNormalVel;
                            float jt = 0f;
                            if (tangent.sqrMagnitude > 0.001f)
                            {
                                tangent.Normalize();
                                jt = -Vector3.Dot(relVel, tangent) / (invMassA + invMassB);
                                jt = Mathf.Clamp(jt, -friction * Mathf.Abs(jn), friction * Mathf.Abs(jn));
                            }

                            Vector3 impulse = normal * jn + tangent * jt;
                            Vector3 contactPoint = (worldPos + otherPos) * 0.5f;

                            SoftbodyCollisionManager.AddCollisionForce(solverID, new CollisionForce(i, impulse / dt, contactPoint));
                            SoftbodyCollisionManager.AddCollisionForce(otherSolver.solverID, new CollisionForce(otherNodeIndex, -impulse / dt, contactPoint));

                            if (visualizeForces)
                            {
                                Debug.DrawLine(worldPos, otherPos, Color.magenta, dt);
                                Debug.DrawRay(contactPoint, impulse.normalized * 0.1f, Color.cyan, dt);
                            }
                        }
                    }

                    // ================================================================
                    // 2. NEW: Node → Triangle surface collision (inter-body)
                    // ================================================================
                    if (otherSolver.trussAsset != null && otherSolver.trussAsset.GetTrussFaces().Count > 0)
                    {
                        var faces = otherSolver.trussAsset.GetTrussFaces();

                        Vector3 velNode = (worldPos - nodeManager.PreviousPositions[i]) / dt;
                        float invMassNode = 1f / nodeMasses[i];

                        foreach (var face in faces)
                        {
                            Vector3 ta = otherSolver.nodeManager.PredictedPositions[face.nodeA];
                            Vector3 tb = otherSolver.nodeManager.PredictedPositions[face.nodeB];
                            Vector3 tc = otherSolver.nodeManager.PredictedPositions[face.nodeC];

                            Vector3 closest = GetClosestPointOnTriangleToPoint(worldPos, ta, tb, tc, out Vector3 bary);

                            Vector3 delta = worldPos - closest;
                            float dist = delta.magnitude;
                            if (dist >= surfaceCollisionThickness || dist < 0.001f) continue;

                            Vector3 normal = delta.normalized; // always points from triangle → node (two-sided)

                            // Position correction – push only the penetrating node out
                            nodeManager.PredictedPositions[i] = closest + normal * (surfaceCollisionThickness + eps);

                            // Refresh after correction
                            worldPos = nodeManager.PredictedPositions[i];
                            velNode = (worldPos - nodeManager.PreviousPositions[i]) / dt;

                            // Interpolated triangle velocity
                            Vector3 velTri =
                                bary.x * (ta - otherSolver.nodeManager.PreviousPositions[face.nodeA]) / dt +
                                bary.y * (tb - otherSolver.nodeManager.PreviousPositions[face.nodeB]) / dt +
                                bary.z * (tc - otherSolver.nodeManager.PreviousPositions[face.nodeC]) / dt;

                            Vector3 relVel = velNode - velTri;
                            float relNormalVel = Vector3.Dot(relVel, normal);
                            if (relNormalVel > 0f) continue; // separating

                            // Effective inverse mass of triangle (barycentric weighting)
                            float invMassTri = 0f;
                            if (bary.x > 0.001f) invMassTri += bary.x * bary.x * (otherSolver.nodeManager.IsPinned[face.nodeA] ? 0f : 1f / otherSolver.nodeMasses[face.nodeA]);
                            if (bary.y > 0.001f) invMassTri += bary.y * bary.y * (otherSolver.nodeManager.IsPinned[face.nodeB] ? 0f : 1f / otherSolver.nodeMasses[face.nodeB]);
                            if (bary.z > 0.001f) invMassTri += bary.z * bary.z * (otherSolver.nodeManager.IsPinned[face.nodeC] ? 0f : 1f / otherSolver.nodeMasses[face.nodeC]);

                            float denom = invMassNode + invMassTri;
                            if (denom <= 0.0001f) continue;

                            float jn = -(1 + restitution) * relNormalVel / denom;

                            // Friction
                            Vector3 tangentVel = relVel - normal * relNormalVel;
                            float jt = 0f;
                            if (tangentVel.sqrMagnitude > 0.001f)
                            {
                                Vector3 tangent = tangentVel.normalized;
                                jt = -Vector3.Dot(relVel, tangent) / denom;
                                jt = Mathf.Clamp(jt, -friction * Mathf.Abs(jn), friction * Mathf.Abs(jn));
                            }

                            Vector3 impulse = normal * jn + tangentVel.normalized * jt;

                            // Apply to this node
                            SoftbodyCollisionManager.AddCollisionForce(solverID, new CollisionForce(i, impulse / dt, closest));

                            // Apply to triangle vertices (barycentric)
                            if (bary.x > 0.001f) SoftbodyCollisionManager.AddCollisionForce(otherSolver.solverID, new CollisionForce(face.nodeA, -impulse * bary.x / dt, closest));
                            if (bary.y > 0.001f) SoftbodyCollisionManager.AddCollisionForce(otherSolver.solverID, new CollisionForce(face.nodeB, -impulse * bary.y / dt, closest));
                            if (bary.z > 0.001f) SoftbodyCollisionManager.AddCollisionForce(otherSolver.solverID, new CollisionForce(face.nodeC, -impulse * bary.z / dt, closest));

                            if (visualizeForces)
                                Debug.DrawRay(closest, impulse.normalized * 0.2f, Color.cyan, dt);
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
        #endregion

        #region   Runtime Visualization
        public void DrawNodesAndLinks()
        {
            if (!showNodesAndLinks) return;

            if (showNodes)
                DrawNodes();

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