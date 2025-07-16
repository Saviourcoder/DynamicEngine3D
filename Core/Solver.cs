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

namespace DynamicEngine
{
    public class SoftBodyCore
    {
        public readonly NodeManager nodeManager;
        public List<Beam> beams;
        private readonly MeshDeformer meshDeformer;
        private MaterialProperties materialProps;
        public readonly List<Vector3> collisionPoints;
        private float maxStretchFactor = 1.05f;
        private float minStretchFactor = 0.95f;
        public bool visualizeForces = false;
        private TrussAsset trussAsset; // Reference to TrussAsset for face data

        public SoftBodyCore(
            float nodeRadius,
            float influenceRadius,
            MaterialProperties materialProps,
            Mesh mesh,
            Vector3[] originalVertices)
        {
            this.nodeManager = new NodeManager(nodeRadius);
            this.beams = new List<Beam>();
            this.meshDeformer = new MeshDeformer(mesh, originalVertices, influenceRadius);
            this.materialProps = materialProps ?? MaterialProperties.GetDefault(MaterialType.Custom);
            this.collisionPoints = new List<Vector3>();
        }

        // ------------------------------------------------------------------
        //  SINGLE METHOD  – replaces the two old overloads
        // ------------------------------------------------------------------
        public void GenerateNodesAndBeams(
            Vector3[] positions,
            float     connectionDistance = -1f,
            Beam[]    beamsArray         = null,
            Transform parent             = null)
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
            // 1) Destroy any existing “Node_*” children
            for (int i = parent.childCount - 1; i >= 0; i--)
            {
                Transform child = parent.GetChild(i);
                if (child.name.StartsWith("Node_"))
                    Object.DestroyImmediate(child.gameObject);
            }

            // 2) Clear lists
            nodeManager.Clear();
            beams.Clear();

            // 3) Create hidden nodes
            for (int i = 0; i < positions.Length; i++)
            {
                GameObject nodeObj = new GameObject($"Node_{i}");
                nodeObj.transform.parent = parent;
                nodeObj.transform.localPosition = positions[i];
                nodeObj.hideFlags = HideFlags.HideAndDontSave;

                nodeManager.AddNode(nodeObj.transform, positions[i]);
            }

            // 4) Build beams
            if (beamsArray != null)                    // explicit list
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
            else                                       // auto-links
            {
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                {
                    for (int j = i + 1; j < nodeManager.Nodes.Count; j++)
                    {
                        if (nodeManager.Nodes[i] == null || nodeManager.Nodes[j] == null) continue;

                        Vector3 posA = nodeManager.Nodes[i].localPosition;
                        Vector3 posB = nodeManager.Nodes[j].localPosition;
                        float distance = Vector3.Distance(posA, posB);
                        if (distance <= connectionDistance && distance > 0.01f)
                        {
                            beams.Add(new Beam(i, j,
                                               materialProps.defaultCompliance,
                                               materialProps.defaultDamping,
                                               distance));
                        }
                    }
                }
            }

            meshDeformer.MapVerticesToNodes(parent, nodeManager.Nodes, nodeManager.InitialPositions);
            Debug.Log($"Generated {nodeManager.Nodes.Count} nodes, {beams.Count} beams");
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
                    nodeManager.PreviousPositions.Add(nodeManager.Nodes[i] != null ? nodeManager.Nodes[i].position : Vector3.zero);
                }
            }

            if (nodeManager.PredictedPositions.Count != nodeManager.Nodes.Count)
            {
                nodeManager.PredictedPositions.Clear();
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                {
                    nodeManager.PredictedPositions.Add(nodeManager.Nodes[i] != null ? nodeManager.Nodes[i].position : Vector3.zero);
                }
            }

            // Step 1: Verlet integration
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentPos = nodeManager.Nodes[i].position;
                Vector3 prevPos = nodeManager.PreviousPositions[i];
                Vector3 velocity = (currentPos - prevPos) / dt;
                Vector3 acceleration = Vector3.down * (SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : 9.81f);
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

                Vector3 currentPos = nodeManager.Nodes[i].position;
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

                    // Environment collisions
                    Collider nodeCollider = nodeManager.Colliders[i];
                    Collider[] colliders = Physics.OverlapSphere(tempPos, nodeManager.NodeRadius);
                    foreach (var collider in colliders)
                    {
                        if (collider == nodeCollider || collider.transform == nodeManager.Nodes[i]) continue;

                        int otherNodeIndex = nodeManager.FindIndex(n => n != null && n == collider.transform);
                        if (otherNodeIndex != -1) continue;

                        Vector3 direction;
                        float distance;
                        if (Physics.ComputePenetration(nodeCollider, tempPos, Quaternion.identity,
                                                      collider, collider.transform.position, collider.transform.rotation,
                                                      out direction, out distance))
                        {
                            Vector3 correction = direction * distance;
                            float w = 1f / materialProps.nodeMass;
                            float compliance = materialProps.defaultCompliance / dtSquared;
                            float lambda = -distance / (w + compliance);
                            tempPos += correction * lambda;

                            Vector3 velocity = subMotion / subDt;
                            float impactMagnitude = velocity.magnitude * materialProps.nodeMass / subDt;
                            if (impactMagnitude > 200)
                            {
                                Vector3 deformationOffset = -direction * materialProps.deformationScale;
                                deformationOffset = Vector3.ClampMagnitude(deformationOffset, materialProps.maxDeformation);
                                tempPos += deformationOffset;
                                UpdateBeamRestLengths(i, tempPos);
                            }

                            collisionPoints.Add(tempPos - correction);
                            Debug.DrawLine(tempPos - correction, tempPos, Color.green, 0.1f);
                            Debug.DrawRay(tempPos, direction * 0.2f, Color.yellow, 0.1f);
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

                            Vector3 posA = nodeManager.Nodes[face.nodeA].position;
                            Vector3 posB = nodeManager.Nodes[face.nodeB].position;
                            Vector3 posC = nodeManager.Nodes[face.nodeC].position;

                            Vector3 normal = Vector3.Cross(posB - posA, posC - posA).normalized;
                            float distanceToPlane = Vector3.Dot(tempPos - posA, normal);

                            if (Mathf.Abs(distanceToPlane) < nodeManager.NodeRadius)
                            {
                                Vector3 projectedPoint = tempPos - distanceToPlane * normal;

                                if (IsPointInTriangle(projectedPoint, posA, posB, posC))
                                {
                                    float w = 1f / materialProps.nodeMass;
                                    float compliance = materialProps.defaultCompliance / dtSquared;
                                    float lambda = -distanceToPlane / (w + compliance);
                                    Vector3 correction = normal * lambda;

                                    if (float.IsNaN(correction.x) || float.IsNaN(correction.y) || float.IsNaN(correction.z))
                                    {
                                        correction = Vector3.zero;
                                        Debug.LogWarning($"NaN correction in face-node collision for node {i}, skipping");
                                    }
                                    else
                                    {
                                        tempPos += correction;
                                        collisionPoints.Add(tempPos - correction);
                                        Debug.DrawLine(tempPos - correction, tempPos, Color.cyan, 0.1f);
                                        Debug.DrawRay(tempPos, normal * 0.2f, Color.magenta, 0.1f);
                                    }
                                }
                            }
                        }
                    }
                }
                nodeManager.PredictedPositions[i] = tempPos;

                Collider[] overlaps = Physics.OverlapSphere(nodeManager.PredictedPositions[i], nodeManager.NodeRadius);
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

                        float compliance = materialProps.defaultCompliance / dtSquared;
                        float lambda = -constraint / (wSum + compliance);

                        Vector3 correction = gradient * lambda;
                        nodeManager.PredictedPositions[i] -= wA * correction;
                        nodeManager.PredictedPositions[j] += wB * correction;
                        collisionPoints.Add((posA + posB) * 0.5f);
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

                Vector3 currentPos = nodeManager.Nodes[i].position;
                Vector3 predictedPos = nodeManager.PredictedPositions[i];
                Vector3 motion = predictedPos - currentPos;

                Collider nodeCollider = nodeManager.Colliders[i];
                Collider[] colliders = Physics.OverlapSphere(predictedPos, nodeManager.NodeRadius);
                bool inCollision = false;
                Vector3 normal = Vector3.zero;
                foreach (var collider in colliders)
                {
                    if (collider == nodeCollider || collider.transform == nodeManager.Nodes[i]) continue;
                    int otherNodeIndex = nodeManager.FindIndex(n => n != null && n == collider.transform);
                    if (otherNodeIndex != -1) continue;

                    Vector3 direction;
                    float distance;
                    if (Physics.ComputePenetration(nodeCollider, predictedPos, Quaternion.identity,
                                                  collider, collider.transform.position, collider.transform.rotation,
                                                  out direction, out distance))
                    {
                        inCollision = true;
                        normal = direction;
                        predictedPos += direction * distance;
                        collisionPoints.Add(predictedPos);
                        break;
                    }
                }

                nodeManager.Nodes[i].position = predictedPos;

                if (inCollision)
                {
                    Vector3 velocity = motion / dt;
                    Vector3 velocityNormal = Vector3.Project(velocity, normal);
                    Vector3 velocityTangent = velocity - velocityNormal;
                    Vector3 newVelocity = velocityTangent * (1f - friction) - velocityNormal * restitution;

                    if (newVelocity.magnitude < restThreshold)
                    {
                        newVelocity = Vector3.zero;
                    }
                    nodeManager.PreviousPositions[i] = nodeManager.Nodes[i].position - newVelocity * dt;
                }
                else
                {
                    nodeManager.PreviousPositions[i] = currentPos;
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

        private void UpdateBeamRestLengths(int nodeIndex, Vector3 newPosition)
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

                Vector3 posA = nodeManager.Nodes[beam.nodeA].position;
                Vector3 posB = nodeManager.Nodes[beam.nodeB].position;
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
            meshDeformer.Deform(transform, nodeManager.Nodes, nodeManager.InitialPositions);
        }

        public void ApplyForceToNode(int nodeIndex, Vector3 force)
        {
            if (nodeIndex >= 0 && nodeIndex < nodeManager.Nodes.Count && nodeManager.Nodes[nodeIndex] != null && !nodeManager.IsPinned[nodeIndex])
            {
                Vector3 acceleration = force / materialProps.nodeMass;
                nodeManager.PreviousPositions[nodeIndex] = nodeManager.Nodes[nodeIndex].position - acceleration * Time.fixedDeltaTime * Time.fixedDeltaTime;
            }
        }

        public void PinNode(int nodeIndex)
        {
            nodeManager.PinNode(nodeIndex);
        }

        public void UnpinNode(int nodeIndex)
        {
            nodeManager.UnpinNode(nodeIndex);
        }

        public int FindClosestNodeToRay(Ray ray, float maxDistance)
        {
            int closestNode = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null) continue;
                Vector3 nodePos = nodeManager.Nodes[i].position;
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
                Vector3 currentPos = nodeManager.Nodes[nodeIndex].position;
                Vector3 delta = targetPosition - currentPos;
                Vector3 impulse = delta * strength * materialProps.nodeMass / Time.fixedDeltaTime;
                ApplyForceToNode(nodeIndex, impulse);
            }
        }

        public void SetMaterialProperties(MaterialProperties props)
        {
            this.materialProps = props ?? MaterialProperties.GetDefault(MaterialType.Custom);
        }
    }
}
