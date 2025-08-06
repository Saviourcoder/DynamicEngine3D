/* DynamicEngine3D - Soft Body Simulation
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/
using System.Collections.Generic;
using UnityEngine;
using DynamicEngine;

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
        public MaterialProps materialProps;
        public readonly List<Vector3> collisionPoints;
        private float maxStretchFactor = 1.05f;
        private float minStretchFactor = 0.95f;
        public bool visualizeForces = false;
        private TrussAsset trussAsset; // Reference to TrussAsset for face data
        
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

        public Solver(float nodeRadius, float influenceRadius, MaterialProps materialProps, Mesh mesh, Vector3[] originalVertices, Transform owner)
        {
            this.nodeManager = new NodeManager(nodeRadius);
            this.beams = new List<Beam>();
            this.meshDeformer = new MeshDeformer(mesh, originalVertices, influenceRadius);
            this.materialProps = materialProps ?? MaterialProps.GetDefault(MaterialType.Custom);
            this.collisionPoints = new List<Vector3>();
            this.owner = owner;
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

            // 1) Clear old data
            nodeManager.Clear();
            beams.Clear();
            DestroyOldNodes(parent);

            // 2) Build the *mesh-local* node positions
            for (int i = 0; i < positions.Length; i++)
            {
                GameObject nodeObj = new GameObject($"Node_{i}");
                nodeObj.transform.parent = parent;
                nodeObj.transform.position = parent.TransformPoint(positions[i]); // world pos for GO
                nodeObj.hideFlags = HideFlags.HideAndDontSave;

                // FIXED: Store local position instead of world position for mesh mapping
                nodeManager.AddNode(nodeObj.transform, positions[i]);
            }

            /* 3) Build beams – unchanged except we now use the local positions
                  to compute restLength (distance in *local* space)                */
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
                                               materialProps.Springforce,
                                               materialProps.Damping,
                                               distance));
                        }
                    }
                }
            }

            // 4) Tell the deformer how to skin
            meshDeformer.MapVerticesToNodes(parent, nodeManager.Nodes, nodeManager.InitialPositions);
        }
        // ------------------------------------------------------------------
        //  REST OF FILE – unchanged
        // ------------------------------------------------------------------
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

        public void SetTrussAsset(TrussAsset truss)
        {
            trussAsset = truss;
        }

        public void Solve()
        {
            // Keep settings in sync with the Designer
            var editor = owner?.GetComponent<Designer>();
            if (editor != null)
            {
                materialProps.nodeMass = editor.nodeMass;
                maxStretchFactor = editor.maxStretchFactor;
                minStretchFactor = editor.minStretchFactor;

                // Live-link properties
                for (int i = 0; i < beams.Count && i < editor.links.Count; i++)
                {
                    beams[i].compliance = 1f / editor.links[i].springForce;
                    beams[i].damping = editor.links[i].damping;
                }
            }
            if (nodeManager == null || beams == null || materialProps == null || nodeManager.Nodes == null)
            {
                Debug.LogWarning("Cannot simulate plastic deformation: Invalid state.");
                return;
            }

            const float restitution = 0.05f;
            const float friction = 0.7f;
            const float restThreshold = 0.01f;
            float dt = Time.fixedDeltaTime;
            float dtSquared = dt * dt;
            const int maxSubSteps = 5;

            collisionPoints.Clear();

            if (nodeManager.PreviousPositions.Count != nodeManager.Nodes.Count)
            {
                nodeManager.PreviousPositions.Clear();
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                {
                    nodeManager.PreviousPositions.Add(nodeManager.Nodes[i] != null ? nodeManager.Nodes[i].localPosition : Vector3.zero);
                }
            }

            if (nodeManager.PredictedPositions.Count != nodeManager.Nodes.Count)
            {
                nodeManager.PredictedPositions.Clear();
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                {
                    nodeManager.PredictedPositions.Add(nodeManager.Nodes[i] != null ? nodeManager.Nodes[i].localPosition : Vector3.zero);
                }
            }

            // Step 1: Verlet integration
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentPos = nodeManager.Nodes[i].localPosition; // Use local position
                Vector3 prevPos = nodeManager.PreviousPositions[i];
                Vector3 velocity = (currentPos - prevPos) / dt;
                
                // Apply gravity in local space (transform gravity to local space)
                Vector3 localGravity = owner.InverseTransformDirection(Vector3.down * (SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : 9.81f));
                Vector3 acceleration = localGravity;
                Vector3 newPosition = currentPos + velocity * dt + acceleration * dtSquared;

                if (float.IsNaN(newPosition.x) || float.IsNaN(newPosition.y) || float.IsNaN(newPosition.z))
                {
                    newPosition = currentPos;
                    Debug.LogWarning($"NaN detected for node {i}, keeping at {currentPos}");
                }
                nodeManager.PredictedPositions[i] = newPosition;
            }

            // Step 2: Handle collisions (node-to-node, environment, and face-node)
            float collisionRadius = nodeManager.NodeRadius * 2f;
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentPos = nodeManager.Nodes[i].localPosition; // Use local position
                Vector3 predictedPos = nodeManager.PredictedPositions[i];
                Vector3 motion = predictedPos - currentPos;
                float motionDistance = motion.magnitude;
                int subSteps = Mathf.Min(maxSubSteps, Mathf.CeilToInt(motionDistance / (nodeManager.NodeRadius * 0.5f)));
                float subDt = dt / (subSteps > 0 ? subSteps : 1);
                Vector3 subMotion = motion / (subSteps > 0 ? subSteps : 1);

                Vector3 tempPos = currentPos;
                for (int step = 0; step < subSteps; step++)
                {
                    tempPos += subMotion;

                    // Environment collisions - convert to world space for collision detection
                    Vector3 worldTempPos = owner.TransformPoint(tempPos);
                    Collider nodeCollider = nodeManager.Colliders[i];
                    Collider[] colliders = Physics.OverlapSphere(worldTempPos, nodeManager.NodeRadius);
                    foreach (var collider in colliders)
                    {
                        if (collider == nodeCollider || collider.transform == nodeManager.Nodes[i]) continue;

                        int otherNodeIndex = nodeManager.FindIndex(n => n != null && n == collider.transform);
                        if (otherNodeIndex != -1) continue;

                        Vector3 direction;
                        float distance;
                        if (Physics.ComputePenetration(nodeCollider, worldTempPos, owner.rotation,
                                                      collider, collider.transform.position, collider.transform.rotation,
                                                      out direction, out distance))
                        {
                            // Convert correction back to local space
                            Vector3 worldCorrection = direction * distance;
                            Vector3 localDirection = owner.InverseTransformDirection(direction);
                            Vector3 localCorrection = localDirection * distance;
                            
                            float w = 1f / materialProps.nodeMass;
                            float compliance = materialProps.Springforce / dtSquared;
                            float lambda = -distance / (w + compliance);
                            tempPos += localCorrection * lambda;

                            Vector3 velocity = subMotion / subDt;
                            float impactMagnitude = velocity.magnitude * materialProps.nodeMass / subDt;
                            if (impactMagnitude > 200)
                            {
                                Vector3 deformationOffset = -localDirection * materialProps.deformationScale;
                                deformationOffset = Vector3.ClampMagnitude(deformationOffset, materialProps.maxDeformation);
                                tempPos += deformationOffset;
                                UpdateBeamRestLengths(i, tempPos);
                            }

                            // Convert collision point to world space for debug drawing
                            Vector3 worldCollisionPoint = owner.TransformPoint(tempPos - localCorrection);
                            collisionPoints.Add(worldCollisionPoint);
                            Debug.DrawLine(worldCollisionPoint, owner.TransformPoint(tempPos), Color.green, 0.1f);
                            Debug.DrawRay(owner.TransformPoint(tempPos), direction * 0.2f, Color.yellow, 0.1f);
                        }
                    }

                    // Face-node collisions
                    if (trussAsset != null)
                    {
                        foreach (var face in trussAsset.GetTrussFaces())
                        {
                            if (face.nodeA >= nodeManager.Nodes.Count || face.nodeB >= nodeManager.Nodes.Count || face.nodeC >= nodeManager.Nodes.Count ||
                                nodeManager.Nodes[face.nodeA] == null || nodeManager.Nodes[face.nodeB] == null || nodeManager.Nodes[face.nodeC] == null)
                                continue;

                            if (i == face.nodeA || i == face.nodeB || i == face.nodeC) continue;

                            // Convert local positions to world space for face collision calculations
                            Vector3 posA = owner.TransformPoint(nodeManager.InitialPositions[face.nodeA]);
                            Vector3 posB = owner.TransformPoint(nodeManager.InitialPositions[face.nodeB]);
                            Vector3 posC = owner.TransformPoint(nodeManager.InitialPositions[face.nodeC]);
                            Vector3 worldTempPos2 = owner.TransformPoint(tempPos);

                            Vector3 normal = Vector3.Cross(posB - posA, posC - posA).normalized;
                            float distanceToPlane = Vector3.Dot(worldTempPos2 - posA, normal);

                            if (Mathf.Abs(distanceToPlane) < nodeManager.NodeRadius)
                            {
                                Vector3 projectedPoint = worldTempPos2 - distanceToPlane * normal;

                                if (IsPointInTriangle(projectedPoint, posA, posB, posC))
                                {
                                    float w = 1f / materialProps.nodeMass;
                                    float compliance = materialProps.Springforce / dtSquared;
                                    float lambda = -distanceToPlane / (w + compliance);
                                    Vector3 worldCorrection = normal * lambda;
                                    Vector3 localCorrection = owner.InverseTransformVector(worldCorrection);

                                    if (float.IsNaN(localCorrection.x) || float.IsNaN(localCorrection.y) || float.IsNaN(localCorrection.z))
                                    {
                                        localCorrection = Vector3.zero;
                                        Debug.LogWarning($"NaN correction in face-node collision for node {i}, skipping");
                                    }
                                    else
                                    {
                                        tempPos += localCorrection;
                                        Vector3 worldCollisionPoint = owner.TransformPoint(tempPos - localCorrection);
                                        collisionPoints.Add(worldCollisionPoint);
                                        Debug.DrawLine(worldCollisionPoint, owner.TransformPoint(tempPos), Color.cyan, 0.1f);
                                        Debug.DrawRay(owner.TransformPoint(tempPos), normal * 0.2f, Color.magenta, 0.1f);
                                    }
                                }
                            }
                        }
                    }
                }
                nodeManager.PredictedPositions[i] = tempPos;

                // Node-to-node collisions - convert to world space for collision detection
                Vector3 worldPredictedPos = owner.TransformPoint(nodeManager.PredictedPositions[i]);
                Collider[] overlaps = Physics.OverlapSphere(worldPredictedPos, nodeManager.NodeRadius);
                foreach (var collider in overlaps)
                {
                    if (collider.transform == nodeManager.Nodes[i]) continue;
                    int j = nodeManager.FindIndex(n => n != null && n == collider.transform);
                    if (j == -1 || j == i || nodeManager.IsPinned[j]) continue;

                    bool areConnected = beams.Exists(b =>
                        (b.nodeA == i && b.nodeB == j) || (b.nodeA == j && b.nodeB == i));
                    if (areConnected) continue;

                    Vector3 posA = nodeManager.PredictedPositions[i];
                    Vector3 posB = nodeManager.PredictedPositions[j];
                    Vector3 delta = posB - posA;
                    float distance = delta.magnitude;
                    if (distance < collisionRadius && distance > 0.001f)
                    {
                        float constraint = distance - collisionRadius;
                        Vector3 gradient = delta / distance;

                        float wA = 1f / materialProps.nodeMass;
                        float wB = 1f / materialProps.nodeMass;
                        float wSum = wA + wB;

                        float compliance = materialProps.Springforce / dtSquared;
                        float lambda = -constraint / (wSum + compliance);

                        Vector3 correction = gradient * lambda;
                        nodeManager.PredictedPositions[i] -= wA * correction;
                        nodeManager.PredictedPositions[j] += wB * correction;
                        
                        // Convert collision point to world space for debugging
                        Vector3 worldCollisionPoint = owner.TransformPoint((posA + posB) * 0.5f);
                        collisionPoints.Add(worldCollisionPoint);
                    }
                }
            }

            // Step 3: XPBD constraint solving
            for (int iter = 0; iter < (SceneSettings.Instance != null ? SceneSettings.Instance.ConstraintIterations : 10); iter++)
            {
                for (int i = 0; i < beams.Count; i++)
                {
                    Beam beam = beams[i];
                    if (beam.nodeA >= nodeManager.Nodes.Count || beam.nodeB >= nodeManager.Nodes.Count ||
                        nodeManager.Nodes[beam.nodeA] == null || nodeManager.Nodes[beam.nodeB] == null)
                    {
                        Debug.LogWarning($"Invalid beam {i}: nodeA={beam.nodeA}, nodeB={beam.nodeB}");
                        continue;
                    }

                    if (nodeManager.IsPinned[beam.nodeA] && nodeManager.IsPinned[beam.nodeB]) continue;

                    Vector3 posA = nodeManager.PredictedPositions[beam.nodeA];
                    Vector3 posB = nodeManager.PredictedPositions[beam.nodeB];
                    Vector3 delta = posB - posA;
                    float currentLength = delta.magnitude;
                    if (currentLength < 0.0001f)
                    {
                        Debug.LogWarning($"Beam {i} has near-zero length: {currentLength}, skipping correction");
                        continue;
                    }

                    float constraint = currentLength - beam.restLength;
                    Vector3 gradient = delta / currentLength;

                    float wA = nodeManager.IsPinned[beam.nodeA] ? 0f : 1f / materialProps.nodeMass;
                    float wB = nodeManager.IsPinned[beam.nodeB] ? 0f : 1f / materialProps.nodeMass;
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

                    currentLength = Vector3.Distance(nodeManager.PredictedPositions[beam.nodeA], nodeManager.PredictedPositions[beam.nodeB]);
                    if (currentLength > beam.restLength * maxStretchFactor)
                    {
                        float scale = beam.restLength * maxStretchFactor / currentLength;
                        Vector3 center = (nodeManager.PredictedPositions[beam.nodeA] + nodeManager.PredictedPositions[beam.nodeB]) * 0.5f;
                        if (!nodeManager.IsPinned[beam.nodeA]) nodeManager.PredictedPositions[beam.nodeA] = center + (nodeManager.PredictedPositions[beam.nodeA] - center) * scale;
                        if (!nodeManager.IsPinned[beam.nodeB]) nodeManager.PredictedPositions[beam.nodeB] = center + (nodeManager.PredictedPositions[beam.nodeB] - center) * scale;
                    }
                    else if (currentLength < beam.restLength * minStretchFactor)
                    {
                        float scale = beam.restLength * minStretchFactor / currentLength;
                        Vector3 center = (nodeManager.PredictedPositions[beam.nodeA] + nodeManager.PredictedPositions[beam.nodeB]) * 0.5f;
                        if (!nodeManager.IsPinned[beam.nodeA]) nodeManager.PredictedPositions[beam.nodeA] = center + (nodeManager.PredictedPositions[beam.nodeA] - center) * scale;
                        if (!nodeManager.IsPinned[beam.nodeB]) nodeManager.PredictedPositions[beam.nodeB] = center + (nodeManager.PredictedPositions[beam.nodeB] - center) * scale;
                    }

                    beams[i] = beam;
                    if (visualizeForces && constraint != 0)
                    {
                        float forceMagnitude = Mathf.Abs(beam.lagrangeMultiplier) / dtSquared;
                        Debug.DrawLine(posA, posB, forceMagnitude > 0 ? Color.red : Color.blue, 0.1f);
                    }
                }
            }

            // Step 4: Finalize positions and apply velocity corrections
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentPos = nodeManager.Nodes[i].localPosition; // Use local position
                Vector3 predictedPos = nodeManager.PredictedPositions[i];
                Vector3 motion = predictedPos - currentPos;

                // Convert to world space for collision detection
                Vector3 worldPredictedPos = owner.TransformPoint(predictedPos);
                Collider nodeCollider = nodeManager.Colliders[i];
                Collider[] colliders = Physics.OverlapSphere(worldPredictedPos, nodeManager.NodeRadius);
                bool inCollision = false;
                Vector3 normal = Vector3.zero;
                foreach (var collider in colliders)
                {
                    if (collider == nodeCollider || collider.transform == nodeManager.Nodes[i]) continue;
                    int otherNodeIndex = nodeManager.FindIndex(n => n != null && n == collider.transform);
                    if (otherNodeIndex != -1) continue;

                    Vector3 direction;
                    float distance;
                    if (Physics.ComputePenetration(nodeCollider, worldPredictedPos, owner.rotation,
                                                  collider, collider.transform.position, collider.transform.rotation,
                                                  out direction, out distance))
                    {
                        inCollision = true;
                        normal = direction;
                        // Convert correction back to local space
                        Vector3 localDirection = owner.InverseTransformDirection(direction);
                        predictedPos += localDirection * distance;
                        
                        Vector3 worldCollisionPoint = owner.TransformPoint(predictedPos);
                        collisionPoints.Add(worldCollisionPoint);
                        break;
                    }
                }

                nodeManager.Nodes[i].localPosition = predictedPos; // Set local position

                if (inCollision)
                {
                    Vector3 velocity = motion / dt;
                    Vector3 localNormal = owner.InverseTransformDirection(normal);
                    Vector3 velocityNormal = Vector3.Project(velocity, localNormal);
                    Vector3 velocityTangent = velocity - velocityNormal;
                    Vector3 newVelocity = velocityTangent * (1f - friction) - velocityNormal * restitution;

                    if (newVelocity.magnitude < restThreshold)
                    {
                        newVelocity = Vector3.zero;
                    }
                    nodeManager.PreviousPositions[i] = nodeManager.Nodes[i].localPosition - newVelocity * dt;
                }
                else
                {
                    nodeManager.PreviousPositions[i] = currentPos;
                }
            }
            
            // Draw visualization if enabled
            DrawNodesAndLinks();
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

                if (strain > materialProps.plasticityThreshold)
                {
                    beam.restLength = Mathf.Lerp(beam.restLength, newLength, materialProps.plasticityRate);
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
            meshDeformer.Deform(transform, nodeManager.Nodes, nodeManager.InitialPositions);
        }

        public void ApplyForceToNode(int nodeIndex, Vector3 force)
        {
            if (nodeIndex >= 0 && nodeIndex < nodeManager.Nodes.Count && nodeManager.Nodes[nodeIndex] != null && !nodeManager.IsPinned[nodeIndex])
            {
                Vector3 acceleration = force / materialProps.nodeMass;
                nodeManager.PreviousPositions[nodeIndex] = nodeManager.Nodes[nodeIndex].localPosition - acceleration * Time.fixedDeltaTime * Time.fixedDeltaTime;
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
                if (distance < nodeManager.NodeRadius && distance < minDistance)
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
            if (nodeIndex >= 0 && nodeIndex < nodeManager.Nodes.Count && nodeManager.Nodes[nodeIndex] != null && !nodeManager.IsPinned[nodeIndex])
            {
                Vector3 currentPos = nodeManager.Nodes[nodeIndex].localPosition; // Use local position
                Vector3 targetLocalPos = owner.InverseTransformPoint(targetPosition); // Convert target to local space
                Vector3 delta = targetLocalPos - currentPos;
                Vector3 impulse = delta * strength * materialProps.nodeMass / Time.fixedDeltaTime;
                ApplyForceToNode(nodeIndex, impulse);
            }
        }

        public void SetMaterialProperties(MaterialProps props)
        {
            this.materialProps = props ?? MaterialProps.GetDefault(MaterialType.Custom);
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
    }
}
