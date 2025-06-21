using System.Collections.Generic;
using UnityEngine;
using DynamicEngine; // Use DynamicEngine for Beam and MaterialProperties

namespace DynamicEngine
{
    public class SceneSettings : MonoBehaviour
    {
        public static SceneSettings Instance { get; private set; } [SerializeField, Range(0f, 20f)] public float Gravity = 9.81f; [SerializeField, Range(1, 20)] public int ConstraintIterations = 10;


        private void Awake()
        {
            if (Instance == null)
            {
                Instance = this;
                DontDestroyOnLoad(gameObject);
            }
            else
            {
                Destroy(gameObject);
            }
        }
    }

    public class NodeManager
    {
        private readonly List<Transform> nodes;
        private readonly List<SphereCollider> nodeColliders;
        private readonly List<Vector3> initialNodePositions;
        private readonly List<Vector3> previousPositions;
        private readonly List<Vector3> predictedPositions;
        private readonly List<bool> isPinned;
        private readonly float nodeRadius;

        public NodeManager(float nodeRadius)
        {
            this.nodeRadius = Mathf.Max(0.01f, nodeRadius);
            nodes = new List<Transform>();
            nodeColliders = new List<SphereCollider>();
            initialNodePositions = new List<Vector3>();
            previousPositions = new List<Vector3>();
            predictedPositions = new List<Vector3>();
            isPinned = new List<bool>();
        }

        public IReadOnlyList<Transform> Nodes => nodes;
        public IReadOnlyList<SphereCollider> Colliders => nodeColliders;
        public IReadOnlyList<Vector3> InitialPositions => initialNodePositions;
        public List<Vector3> PreviousPositions => previousPositions;
        public List<Vector3> PredictedPositions => predictedPositions;
        public IReadOnlyList<bool> IsPinned => isPinned;
        public float NodeRadius => nodeRadius;

        public void AddNode(Transform transform, Vector3 position)
        {
            if (transform == null) return;

            nodes.Add(transform);
            initialNodePositions.Add(position);
            previousPositions.Add(transform.position);
            predictedPositions.Add(transform.position);
            isPinned.Add(false);

            SphereCollider collider = transform.GetComponent<SphereCollider>();
            if (collider == null) collider = transform.gameObject.AddComponent<SphereCollider>();
            collider.radius = nodeRadius;
            collider.isTrigger = false;
            nodeColliders.Add(collider);
        }

        public void PinNode(int index)
        {
            if (index >= 0 && index < isPinned.Count)
            {
                isPinned[index] = true;
            }
        }

        public void UnpinNode(int index)
        {
            if (index >= 0 && index < isPinned.Count)
            {
                isPinned[index] = false;
            }
        }

        public void Clear()
        {
            foreach (var node in nodes)
            {
                if (node != null)
                {
                    if (Application.isEditor && !Application.isPlaying)
                        Object.DestroyImmediate(node.gameObject);
                    else
                        Object.Destroy(node.gameObject);
                }
            }
            nodes.Clear();
            nodeColliders.Clear();
            initialNodePositions.Clear();
            previousPositions.Clear();
            predictedPositions.Clear();
            isPinned.Clear();
        }

        public int FindIndex(System.Func<Transform, bool> predicate)
        {
            if (predicate == null) return -1;
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] != null && predicate(nodes[i]))
                    return i;
            }
            return -1;
        }
    }

    public class MeshDeformer
    {
        private readonly Mesh mesh;
        private readonly Vector3[] originalVertices;
        private readonly Dictionary<int, List<int>> nodeVertexMap;
        private readonly float influenceRadius;
        private Vector3[] deformedVertices;

        public MeshDeformer(Mesh mesh, Vector3[] originalVertices, float influenceRadius)
        {
            this.mesh = mesh ?? new Mesh();
            this.originalVertices = originalVertices ?? new Vector3[0];
            this.influenceRadius = Mathf.Max(0.01f, influenceRadius);
            deformedVertices = new Vector3[this.originalVertices.Length];
            System.Array.Copy(this.originalVertices, deformedVertices, this.originalVertices.Length);
            nodeVertexMap = new Dictionary<int, List<int>>();
        }

        public Mesh Mesh => mesh;

        public void MapVerticesToNodes(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            if (transform == null || nodes == null || initialPositions == null) return;

            nodeVertexMap.Clear();
            Matrix4x4 localToWorld = transform.localToWorldMatrix;

            for (int i = 0; i < originalVertices.Length; i++)
            {
                Vector3 worldVertex = localToWorld.MultiplyPoint3x4(originalVertices[i]);
                int closestNode = -1;
                float minDistance = float.MaxValue;

                for (int j = 0; j < nodes.Count; j++)
                {
                    if (nodes[j] == null || j >= initialPositions.Count) continue;
                    Vector3 worldNodePos = nodes[j].position;
                    float distance = Vector3.Distance(worldVertex, worldNodePos);
                    if (distance < minDistance && distance <= influenceRadius)
                    {
                        minDistance = distance;
                        closestNode = j;
                    }
                }

                if (closestNode != -1)
                {
                    if (!nodeVertexMap.ContainsKey(closestNode))
                        nodeVertexMap[closestNode] = new List<int>();
                    nodeVertexMap[closestNode].Add(i);
                }
            }
        }

        public void Deform(Transform transform, IReadOnlyList<Transform> nodes, IReadOnlyList<Vector3> initialPositions)
        {
            if (transform == null || nodes == null || initialPositions == null || originalVertices == null || deformedVertices == null || nodeVertexMap == null)
            {
                Debug.LogWarning("Cannot deform mesh: Invalid input data.");
                return;
            }

            System.Array.Copy(originalVertices, deformedVertices, originalVertices.Length);

            for (int i = 0; i < nodes.Count; i++)
            {
                if (!nodeVertexMap.ContainsKey(i) || nodes[i] == null || i >= initialPositions.Count) continue;

                Vector3 initialPos = transform.TransformPoint(initialPositions[i]);
                Vector3 currentPos = nodes[i].position;
                Vector3 offsetWorld = currentPos - initialPos;
                Vector3 offsetLocal = transform.InverseTransformVector(offsetWorld);

                foreach (int vertexIndex in nodeVertexMap[i])
                {
                    if (vertexIndex >= originalVertices.Length || vertexIndex >= deformedVertices.Length) continue;

                    Vector3 originalPos = originalVertices[vertexIndex];
                    float distance = Vector3.Distance(originalPos, initialPositions[i]);
                    float influence = Mathf.Clamp01(1f - (distance / influenceRadius));
                    deformedVertices[vertexIndex] = originalPos + offsetLocal * influence;
                }
            }

            mesh.vertices = deformedVertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
        }
    }

    public class SoftBodyCore
    {
        public readonly NodeManager nodeManager;
        public List<Beam> beams; // Removed readonly to allow updates
        private readonly MeshDeformer meshDeformer;
        private MaterialProperties materialProps;
        public readonly List<Vector3> collisionPoints;
        private float maxStretchFactor = 1.05f;
        private float minStretchFactor = 0.95f;
        public bool visualizeForces = false;

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

        public void GenerateNodesAndBeams(Vector3[] positions, float connectionDistance, Transform transform)
        {
            if (positions == null || positions.Length == 0 || transform == null)
            {
                Debug.LogWarning("Cannot generate nodes and beams: Invalid input.");
                return;
            }

            nodeManager.Clear();
            beams.Clear();

            for (int i = 0; i < positions.Length; i++)
            {
                GameObject nodeObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                nodeObj.name = $"Node_{i}";
                nodeObj.transform.parent = transform;
                nodeObj.transform.localPosition = positions[i];
                nodeObj.transform.localScale = Vector3.one * nodeManager.NodeRadius * 2f;

                Renderer renderer = nodeObj.GetComponent<Renderer>();
                if (renderer != null)
                    renderer.material = new Material(Shader.Find("Standard"));

                nodeManager.AddNode(nodeObj.transform, positions[i]);
            }

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
                        beams.Add(new Beam(i, j, materialProps.defaultCompliance, materialProps.defaultDamping, distance));
                    }
                }
            }

            meshDeformer.MapVerticesToNodes(transform, nodeManager.Nodes, nodeManager.InitialPositions);
            Debug.Log($"Generated {nodeManager.Nodes.Count} nodes, {beams.Count} beams");
        }

        public void GenerateNodesAndBeams(Vector3[] positions, Beam[] beamsArray, Transform transform)
        {
            if (positions == null || positions.Length == 0 || beamsArray == null || transform == null)
            {
                Debug.LogWarning("Cannot generate nodes and beams: Invalid input.");
                return;
            }

            nodeManager.Clear();
            beams.Clear();

            for (int i = 0; i < positions.Length; i++)
            {
                GameObject nodeObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                nodeObj.name = $"Node_{i}";
                nodeObj.transform.parent = transform;
                nodeObj.transform.localPosition = positions[i];
                nodeObj.transform.localScale = Vector3.one * nodeManager.NodeRadius * 2f;

                Renderer renderer = nodeObj.GetComponent<Renderer>();
                if (renderer != null)
                    renderer.material = new Material(Shader.Find("Standard"));

                nodeManager.AddNode(nodeObj.transform, positions[i]);
            }

            foreach (var beam in beamsArray)
            {
                if (beam.nodeA >= 0 && beam.nodeA < positions.Length && beam.nodeB >= 0 && beam.nodeB < positions.Length &&
                    beam.restLength > 0.01f)
                {
                    beams.Add(new Beam(beam.nodeA, beam.nodeB, beam.compliance, beam.damping, beam.restLength));
                }
            }

            meshDeformer.MapVerticesToNodes(transform, nodeManager.Nodes, nodeManager.InitialPositions);
            Debug.Log($"Generated {nodeManager.Nodes.Count} nodes, {beams.Count} beams");
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

            GenerateNodesAndBeams(cubeNodes, Vector3.Distance(min, max) * 1.5f, transform);
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
                GameObject nodeObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                nodeObj.name = $"Node_{i}";
                nodeObj.transform.parent = transform;
                nodeObj.transform.localPosition = currentInitialPositions[i];
                nodeObj.transform.localScale = Vector3.one * nodeManager.NodeRadius * 2f;

                Renderer renderer = nodeObj.GetComponent<Renderer>();
                if (renderer != null)
                    renderer.material = new Material(Shader.Find("Standard"));

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

        public void SimulatePlastic()
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

            // Step 2: XPBD constraint solving
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

                    // Apply stretch limits
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

                    beams[i] = beam; // Update beam in the list
                    if (visualizeForces && constraint != 0)
                    {
                        float forceMagnitude = Mathf.Abs(beam.lagrangeMultiplier) / dtSquared;
                        Debug.DrawLine(posA, posB, forceMagnitude > 0 ? Color.red : Color.blue, 0.1f);
                    }
                }
            }

            // Step 3: Node-to-node collisions
            float collisionRadius = nodeManager.NodeRadius * 2f;
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

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

            // Step 4: Update positions and ground collisions
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentPos = nodeManager.Nodes[i].position;
                Vector3 predictedPos = nodeManager.PredictedPositions[i];

                float rayDistance = Mathf.Max(Vector3.Distance(predictedPos, currentPos) + nodeManager.NodeRadius, 0.01f);
                if (Physics.Raycast(currentPos, Vector3.down, out RaycastHit hit, rayDistance))
                {
                    Vector3 velocity = (predictedPos - currentPos) / dt;
                    float impactMagnitude = velocity.magnitude * materialProps.nodeMass / dt;
                    Vector3 normal = hit.normal;
                    Vector3 contactPoint = hit.point + normal * nodeManager.NodeRadius;

                    if (impactMagnitude > 200)
                    {
                        Vector3 deformationOffset = -normal * materialProps.deformationScale;
                        deformationOffset = Vector3.ClampMagnitude(deformationOffset, materialProps.maxDeformation);
                        nodeManager.Nodes[i].position = contactPoint + deformationOffset;
                        UpdateBeamRestLengths(i, nodeManager.Nodes[i].position);
                    }
                    else
                    {
                        float constraint = Vector3.Dot(predictedPos - contactPoint, normal);
                        if (constraint < 0)
                        {
                            float w = 1f / materialProps.nodeMass;
                            float compliance = materialProps.defaultCompliance / dtSquared;
                            float lambda = -constraint / (w + compliance);

                            nodeManager.Nodes[i].position = predictedPos + normal * lambda;
                        }
                        else
                        {
                            nodeManager.Nodes[i].position = predictedPos;
                        }

                        Vector3 newVelocity = (nodeManager.Nodes[i].position - currentPos) / dt;
                        Vector3 velocityNormal = Vector3.Project(newVelocity, normal);
                        Vector3 velocityTangent = newVelocity - velocityNormal;
                        newVelocity = velocityTangent * (1f - friction) - velocityNormal * restitution;

                        if (newVelocity.magnitude < restThreshold)
                        {
                            newVelocity = Vector3.zero;
                        }
                        nodeManager.PreviousPositions[i] = nodeManager.Nodes[i].position - newVelocity * dt;
                    }
                }
                else
                {
                    nodeManager.Nodes[i].position = predictedPos;
                    nodeManager.PreviousPositions[i] = currentPos;
                }
            }
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