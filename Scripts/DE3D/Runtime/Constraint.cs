/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using UnityEngine;
using System.Collections.Generic;

namespace DynamicEngine
{
    public enum AttachmentType
    {
        World,
        Node,
        Edge
    }

    [System.Serializable]
    public struct LinkConfig
    {
        public string name;
        public AttachmentType attachmentTypeA;
        public string attachmentSetA;
        public AttachmentType attachmentTypeB;
        public string attachmentSetB;

        [Tooltip("Set to -1 to auto-calculate from initial scene distance. Set to 0 to snap perfectly.")]
        public float restLength;
        public float minLength;
        public float maxLength;

        [Tooltip("Maximum force the joint can withstand before breaking. Use 'Infinity' for unbreakable.")]
        public float strength;
        public bool master;
        public bool show;
    }

    public class Constraint : MonoBehaviour
    {
        [Header("Base Body")]
        [SerializeField] private SoftBody m_baseBody;
        [SerializeField] private bool m_disableCollision = true;
        [SerializeField] private bool m_showLinks = false;

        [Header("Links")]
        [SerializeField] private List<LinkConfig> m_links = new List<LinkConfig>();
        [HideInInspector] public List<bool> linkVisibility = new List<bool>();
        private bool m_beamsInitialized = false;
        private List<(Collider, Collider)> m_disabledCollisionPairs = new List<(Collider, Collider)>();
        private Dictionary<int, List<Beam>> m_linkToBeamsMap = new Dictionary<int, List<Beam>>();

        [Header("Motor")]
        [SerializeField] private bool m_enableMotor = false;
        [SerializeField] private string m_axisNodeSet = "";
        [SerializeField] private float m_targetRate = 90f;
        [SerializeField] private float m_maxTorque = 100f;
        [SerializeField] private float m_motorProportionalGain = 100f;
        private Dictionary<int, float> m_lastNodeAngles = new Dictionary<int, float>();
        private float GetCurrentDt() => m_attachedBody?.solver != null ?
            (Time.fixedDeltaTime / Mathf.Max(1, m_attachedBody.solver.GetSimulationSubSteps())) : Time.fixedDeltaTime;
        private float m_currentAngularVelocity = 0f;

        private SoftBody m_attachedBody;
        private List<Beam> m_addedBeams = new List<Beam>();

        public SoftBody baseBody => m_baseBody;
        public bool disableCollision => m_disableCollision;
        public bool showLinks => m_showLinks;
        public List<LinkConfig> links => m_links;
        public bool enableMotor => m_enableMotor;
        public string axisNodeSet => m_axisNodeSet;
        public float targetRate => m_targetRate;
        public float maxTorque => m_maxTorque;
        public SoftBody attachedBody => m_attachedBody;
        public int[] GetAxisNodeIndices()
        {
            if (string.IsNullOrEmpty(m_axisNodeSet) || m_attachedBody?.truss == null) return null;
            return m_attachedBody.truss.GetNodeSetIndices(m_axisNodeSet);
        }

        private Vector3 GetAxisCenter()
        {
            if (m_baseBody == null || m_baseBody.solver == null) return Vector3.zero;

            int[] axisIndices = GetAxisNodeIndices();
            if (axisIndices == null || axisIndices.Length == 0) return Vector3.zero;

            Vector3 center = Vector3.zero;
            foreach (int idx in axisIndices)
            {
                center += m_baseBody.solver.nodeManager.PredictedPositions[idx];
            }
            return center / axisIndices.Length;
        }

        private Vector3 GetAxisDirection()
        {
            if (m_baseBody == null || m_baseBody.solver == null) return Vector3.up;

            int[] axisIndices = GetAxisNodeIndices();
            if (axisIndices == null || axisIndices.Length < 2) return Vector3.up;

            Vector3 posA = m_baseBody.solver.nodeManager.PredictedPositions[axisIndices[0]];
            Vector3 posB = m_baseBody.solver.nodeManager.PredictedPositions[axisIndices[1]];

            Vector3 axis = (posB - posA).normalized;
            return axis.magnitude > 0.001f ? axis : Vector3.up;
        }

        private void ApplyMotorTorque(float dt)
        {
            if (!m_enableMotor || m_attachedBody?.solver == null || m_baseBody?.solver == null)
                return;

            int[] axisIndices = GetAxisNodeIndices();
            if (axisIndices == null || axisIndices.Length < 2) return;

            Vector3 axisCenter = GetAxisCenter();
            Vector3 axisDirection = GetAxisDirection();
            float totalAngularVelocity = 0f;
            int validNodes = 0;
            int attachedNodeCount = m_attachedBody.solver.nodeManager.Nodes.Count;

            // Velocity Tracking
            for (int i = 0; i < attachedNodeCount; i++)
            {
                if (m_attachedBody.solver.nodeManager.IsPinned[i]) continue;

                Vector3 nodePos = m_attachedBody.solver.nodeManager.PredictedPositions[i];
                Vector3 radius = nodePos - axisCenter;
                Vector3 radialComponent = radius - Vector3.Project(radius, axisDirection);
                float distance = radialComponent.magnitude;

                if (distance < 0.001f) continue;

                Vector3 referenceDir = Vector3.Cross(axisDirection, Vector3.up).normalized;
                if (referenceDir.magnitude < 0.001f) referenceDir = Vector3.Cross(axisDirection, Vector3.right).normalized;
                Vector3 perpDir = Vector3.Cross(axisDirection, referenceDir).normalized;

                float angle = Mathf.Atan2(Vector3.Dot(radialComponent, perpDir), Vector3.Dot(radialComponent, referenceDir));

                if (m_lastNodeAngles.ContainsKey(i))
                {
                    float deltaAngle = Mathf.DeltaAngle(m_lastNodeAngles[i] * Mathf.Rad2Deg, angle * Mathf.Rad2Deg) * Mathf.Deg2Rad;
                    totalAngularVelocity += deltaAngle / dt;
                    validNodes++;
                }
                m_lastNodeAngles[i] = angle;
            }

            m_currentAngularVelocity = validNodes > 0 ? totalAngularVelocity / validNodes : 0f;
            float targetAngularVelocity = m_targetRate * Mathf.Deg2Rad;
            float angularError = targetAngularVelocity - m_currentAngularVelocity;
            float appliedTorque = Mathf.Clamp(angularError * m_motorProportionalGain, -m_maxTorque, m_maxTorque);

            // Apply Forces
            for (int i = 0; i < attachedNodeCount; i++)
            {
                if (m_attachedBody.solver.nodeManager.IsPinned[i]) continue;

                Vector3 nodePos = m_attachedBody.solver.nodeManager.PredictedPositions[i];
                Vector3 radialComponent = (nodePos - axisCenter) - Vector3.Project(nodePos - axisCenter, axisDirection);
                float distance = radialComponent.magnitude;

                if (distance < 0.001f) continue;

                Vector3 tangent = Vector3.Cross(axisDirection, radialComponent).normalized;
                float forceMag = appliedTorque / (distance * attachedNodeCount);
                Vector3 force = tangent * forceMag;

                m_attachedBody.solver.ApplyWorldForceToNode(i, force, customDt: dt);

                foreach (int axisIdx in axisIndices)
                {
                    m_baseBody.solver.ApplyWorldForceToNode(axisIdx, -force / axisIndices.Length, customDt: dt);
                }
            }
        }

        private void Awake()
        {
            m_attachedBody = GetComponent<SoftBody>();
            if (m_attachedBody == null)
            {
                Debug.LogError("Constraint must be attached to a GameObject with SoftBody component.", this);
                enabled = false;
                return;
            }
        }

        void LateUpdate()
        {
            if (!m_beamsInitialized && Application.isPlaying)
            {
                InitializeBeams();
                m_beamsInitialized = true;
            }
#if UNITY_EDITOR
            if (Application.isPlaying)
                UnityEditor.SceneView.RepaintAll();
#endif
        }

        private void FixedUpdate()
        {
            if (!Application.isPlaying) return;

            // Update beam properties from inspector changes
            UpdateBeamPropertiesFromLinks();

            if (m_enableMotor)
            {
                ApplyMotorTorque(GetCurrentDt());
            }

            if (m_disableCollision && m_disabledCollisionPairs.Count == 0 && m_baseBody != null)
            {
                SetupCollisionDisabling();
            }
        }

        private void SetupCollisionDisabling()
        {
            if (!m_disableCollision || m_baseBody == null) return;

            Collider[] collidersA = m_attachedBody.GetComponentsInChildren<Collider>();
            Collider[] collidersB = m_baseBody.GetComponentsInChildren<Collider>();

            if (collidersA.Length == 0 || collidersB.Length == 0) return;

            RestoreCollisionPairs();

            foreach (Collider colA in collidersA)
            {
                foreach (Collider colB in collidersB)
                {
                    if (colA != null && colB != null)
                    {
                        Physics.IgnoreCollision(colA, colB, true);
                        m_disabledCollisionPairs.Add((colA, colB));
                    }
                }
            }

            if (m_attachedBody.solver?.collisionHandler != null)
            {
                m_attachedBody.solver.collisionHandler.SetIgnoreCollision(
                    m_attachedBody.transform, m_baseBody.transform, true);
            }

            if (m_baseBody.solver?.collisionHandler != null)
            {
                m_baseBody.solver.collisionHandler.SetIgnoreCollision(
                    m_attachedBody.transform, m_baseBody.transform, true);
            }
        }

        private void RestoreCollisionPairs()
        {
            foreach (var (colA, colB) in m_disabledCollisionPairs)
            {
                if (colA != null && colB != null)
                {
                    Physics.IgnoreCollision(colA, colB, false);
                }
            }
            m_disabledCollisionPairs.Clear();

            if (m_baseBody != null)
            {
                if (m_attachedBody?.solver?.collisionHandler != null)
                    m_attachedBody.solver.collisionHandler.SetIgnoreCollision(
                        m_attachedBody.transform, m_baseBody.transform, false);

                if (m_baseBody.solver?.collisionHandler != null)
                    m_baseBody.solver.collisionHandler.SetIgnoreCollision(
                        m_attachedBody.transform, m_baseBody.transform, false);
            }
        }

        private void InitializeBeams()
        {
            if (m_baseBody == null)
            {
                InitializeWorldConstraints();
                return;
            }

            if (m_attachedBody.truss == null || m_baseBody.truss == null)
            {
                Debug.LogError("Truss asset must be assigned to both soft bodies.", this);
                return;
            }

            SetupCollisionDisabling();

            if (m_attachedBody.solver?.nodeManager?.Nodes == null || m_baseBody.solver?.nodeManager?.Nodes == null)
            {
                Debug.LogError("Solver or node manager not properly initialized.", this);
                return;
            }

            m_linkToBeamsMap.Clear();

            int successfulLinks = 0;
            for (int i = 0; i < m_links.Count; i++)
            {
                var config = m_links[i];

                int[] indicesA = m_attachedBody.truss.GetNodeSetIndices(config.attachmentSetA);
                if (!ValidateAttachment(indicesA, config.attachmentTypeA, "A", i, m_attachedBody))
                    continue;

                int[] indicesB = m_baseBody.truss.GetNodeSetIndices(config.attachmentSetB);
                if (!ValidateAttachment(indicesB, config.attachmentTypeB, "B", i, m_baseBody))
                    continue;

                m_linkToBeamsMap[i] = new List<Beam>();

                if (config.attachmentTypeA == AttachmentType.Node && config.attachmentTypeB == AttachmentType.Node)
                {
                    CreateNodeToNodeBeam(indicesA[0], indicesB[0], config, ref successfulLinks, i);
                }
                else if (config.attachmentTypeA == AttachmentType.Node && config.attachmentTypeB == AttachmentType.Edge)
                {
                    CreateNodeToEdgeBeam(indicesA[0], indicesB, config, ref successfulLinks, true, i);
                }
                else if (config.attachmentTypeA == AttachmentType.Edge && config.attachmentTypeB == AttachmentType.Node)
                {
                    CreateEdgeToNodeBeam(indicesA, indicesB[0], config, ref successfulLinks, true, i);
                }
                else if (config.attachmentTypeA == AttachmentType.Edge && config.attachmentTypeB == AttachmentType.Edge)
                {
                    CreateEdgeToEdgeBeam(indicesA, indicesB, config, ref successfulLinks, i);
                }
            }
        }

        private float DetermineTargetRestLength(LinkConfig config, Vector3 posA, Vector3 posB)
        {
            // If restLength is 0 or positive, use it directly (0 = snap perfectly)
            if (config.restLength >= 0f) return config.restLength;

            // If restLength is -1 (Auto), prioritize parameters over scene distance
            if (config.minLength > 0f || config.maxLength > 0f || (config.minLength == 0 && config.maxLength == 0))
            {
                // If min and max are specified, use their average as the target rest length
                // This ensures 0,0 results in 0 rest length instead of the scene distance
                float max = config.maxLength > 0f ? config.maxLength : (config.minLength > 0 ? float.PositiveInfinity : 0f);
                if (float.IsPositiveInfinity(max)) return config.minLength;
                return (config.minLength + max) * 0.5f;
            }

            return Vector3.Distance(posA, posB);
        }

        private void UpdateBeamPropertiesFromLinks()
        {
            if (!m_beamsInitialized) return;
            if (m_linkToBeamsMap == null || m_linkToBeamsMap.Count == 0) return;

            for (int i = 0; i < m_links.Count; i++)
            {
                if (!m_linkToBeamsMap.ContainsKey(i)) continue;

                var config = m_links[i];
                var beams = m_linkToBeamsMap[i];
                if (beams == null) continue;

                foreach (var beam in beams)
                {
                    if (beam == null) continue;

                    beam.minLength = config.minLength;
                    beam.maxLength = config.maxLength;
                    beam.strength = config.strength;

                    // NEW LOGIC: If restLength >= 0, use it. If negative, it was auto-calc (do nothing, preserve current)
                    if (config.restLength >= 0f)
                    {
                        beam.restLength = config.restLength;
                        if (beam.isEdgeSliding) beam.targetPerpDistance = config.restLength;
                    }
                    else if (config.minLength >= 0f || config.maxLength >= 0f)
                    {
                        // Handle range clamping for auto-calc
                        float min = config.minLength;
                        float max = config.maxLength > 0f ? config.maxLength : float.PositiveInfinity;

                        if (beam.isEdgeSliding)
                        {
                            Vector3 nodePos = m_attachedBody.solver.nodeManager.PredictedPositions[beam.slidingNode];
                            Vector3 proj = GetProjectedPointOnEdge(m_baseBody, new int[] { beam.edgeNodeA, beam.edgeNodeB }, nodePos);
                            beam.targetPerpDistance = Mathf.Clamp(Vector3.Distance(nodePos, proj), min, max);
                        }
                        else
                        {
                            beam.restLength = Mathf.Clamp(beam.restLength, min, max);
                        }
                    }
                }
            }
        }

        private void CreateNodeToNodeBeam(int nodeA, int nodeB, LinkConfig config, ref int successfulLinks, int linkIndex)
        {
            float compliance = 0f;
            float damping = 0;

            Vector3 posA = GetAttachmentPoint(m_attachedBody, new int[] { nodeA }, AttachmentType.Node);
            Vector3 posB = GetAttachmentPoint(m_baseBody, new int[] { nodeB }, AttachmentType.Node);
            float targetRestLength = DetermineTargetRestLength(config, posA, posB);

            Beam newBeam = new Beam(
                nodeA: nodeA,
                nodeB: nodeB,
                compliance: compliance,
                damping: damping,
                restLength: targetRestLength,
                maxDeformation: m_attachedBody.solver.maxDeformation,
                bodyA: m_attachedBody,
                bodyB: m_baseBody
            );

            newBeam.minLength = config.minLength;
            newBeam.maxLength = config.maxLength;
            newBeam.strength = config.strength;

            m_attachedBody.solver.beams.Add(newBeam);
            m_baseBody.solver.beams.Add(newBeam);
            m_addedBeams.Add(newBeam);
            m_linkToBeamsMap[linkIndex].Add(newBeam);
            successfulLinks++;
        }

        private void CreateNodeToEdgeBeam(int nodeA, int[] edgeNodes, LinkConfig config, ref int successfulLinks, bool isNodeA, int linkIndex)
        {
            float compliance = 0f;
            float damping = 0;

            float targetPerpDistance;

            // NEW LOGIC: Use >= 0 for explicit settings
            if (config.restLength >= 0f)
            {
                targetPerpDistance = config.restLength;
            }
            else if (config.minLength > 0f || config.maxLength > 0f)
            {
                if (config.minLength <= 0f) targetPerpDistance = config.maxLength;
                else if (config.maxLength <= 0f) targetPerpDistance = config.minLength;
                else targetPerpDistance = (config.minLength + config.maxLength) * 0.5f;
            }
            else
            {
                // Auto-calculate
                Vector3 nodePos = GetAttachmentPoint(m_attachedBody, new int[] { nodeA }, AttachmentType.Node);
                Vector3 edgePosA = GetAttachmentPoint(m_baseBody, new int[] { edgeNodes[0] }, AttachmentType.Node);
                Vector3 edgePosB = GetAttachmentPoint(m_baseBody, new int[] { edgeNodes[1] }, AttachmentType.Node);

                Vector3 edgeVector = edgePosB - edgePosA;
                float edgeLength = edgeVector.magnitude;

                if (edgeLength > 0.001f)
                {
                    Vector3 edgeDir = edgeVector / edgeLength;
                    Vector3 nodeToEdgeStart = nodePos - edgePosA;
                    float t = Vector3.Dot(nodeToEdgeStart, edgeDir);

                    // UNCLAMPED
                    Vector3 closestPoint = edgePosA + edgeDir * t;
                    targetPerpDistance = Vector3.Distance(nodePos, closestPoint);
                }
                else
                {
                    targetPerpDistance = 0.1f;
                }
            }

            for (int i = 0; i < 2; i++)
            {
                int edgeNode = edgeNodes[i];
                Vector3 nodePos = GetAttachmentPoint(m_attachedBody, new int[] { nodeA }, AttachmentType.Node);
                Vector3 pointB = GetAttachmentPoint(m_baseBody, new int[] { edgeNode }, AttachmentType.Node);
                float actualRestLength = Vector3.Distance(nodePos, pointB);

                Beam newBeam = new Beam(
                    nodeA: isNodeA ? nodeA : edgeNode,
                    nodeB: isNodeA ? edgeNode : nodeA,
                    compliance: compliance,
                    damping: damping,
                    restLength: actualRestLength,
                    maxDeformation: m_attachedBody.solver.maxDeformation,
                    bodyA: m_attachedBody,
                    bodyB: m_baseBody
                );

                newBeam.isEdgeSliding = true;
                newBeam.edgeNodeA = edgeNodes[0];
                newBeam.edgeNodeB = edgeNodes[1];
                newBeam.slidingNode = nodeA;
                newBeam.targetPerpDistance = targetPerpDistance;
                newBeam.minLength = config.minLength;
                newBeam.maxLength = config.maxLength;
                newBeam.strength = config.strength;

                m_attachedBody.solver.beams.Add(newBeam);
                m_baseBody.solver.beams.Add(newBeam);
                m_addedBeams.Add(newBeam);
                m_linkToBeamsMap[linkIndex].Add(newBeam);
                successfulLinks++;
            }
        }

        private void CreateEdgeToNodeBeam(int[] edgeNodes, int nodeB, LinkConfig config, ref int successfulLinks, bool isNodeB, int linkIndex)
        {
            CreateNodeToEdgeBeam(nodeB, edgeNodes, config, ref successfulLinks, !isNodeB, linkIndex);
        }

        private void CreateEdgeToEdgeBeam(int[] edgeNodesA, int[] edgeNodesB, LinkConfig config, ref int successfulLinks, int linkIndex)
        {
            float compliance = 0f;
            float damping = 0;

            Vector3 centerA = (GetAttachmentPoint(m_attachedBody, new int[] { edgeNodesA[0] }, AttachmentType.Node) +
                              GetAttachmentPoint(m_attachedBody, new int[] { edgeNodesA[1] }, AttachmentType.Node)) * 0.5f;
            Vector3 centerB = (GetAttachmentPoint(m_baseBody, new int[] { edgeNodesB[0] }, AttachmentType.Node) +
                              GetAttachmentPoint(m_baseBody, new int[] { edgeNodesB[1] }, AttachmentType.Node)) * 0.5f;

            float targetRestLength = DetermineTargetRestLength(config, centerA, centerB);

            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    Beam newBeam = new Beam(
                        nodeA: edgeNodesA[i],
                        nodeB: edgeNodesB[j],
                        compliance: compliance,
                        damping: damping,
                        restLength: targetRestLength,
                        maxDeformation: m_attachedBody.solver.maxDeformation,
                        bodyA: m_attachedBody,
                        bodyB: m_baseBody
                    );

                    newBeam.minLength = config.minLength;
                    newBeam.maxLength = config.maxLength;
                    newBeam.strength = config.strength;

                    m_attachedBody.solver.beams.Add(newBeam);
                    m_baseBody.solver.beams.Add(newBeam);
                    m_addedBeams.Add(newBeam);
                    m_linkToBeamsMap[linkIndex].Add(newBeam);
                    successfulLinks++;
                }
            }
        }

        private void InitializeWorldConstraints()
        {
            if (m_attachedBody.truss == null) return;

            int successfulLinks = 0;
            for (int i = 0; i < m_links.Count; i++)
            {
                var config = m_links[i];
                int[] indicesA = m_attachedBody.truss.GetNodeSetIndices(config.attachmentSetA);
                if (!ValidateAttachment(indicesA, config.attachmentTypeA, "A", i, m_attachedBody)) continue;

                m_linkToBeamsMap[i] = new List<Beam>();

                foreach (int nodeIndex in indicesA)
                {
                    Vector3 worldAnchor = m_attachedBody.solver.nodeManager.PredictedPositions[nodeIndex];

                    Beam worldBeam = new Beam(
                        nodeA: nodeIndex,
                        nodeB: -1,
                        compliance: 0f,
                        damping: 0f,
                        restLength: config.restLength >= 0 ? config.restLength : 0f,
                        maxDeformation: m_attachedBody.solver.maxDeformation,
                        bodyA: m_attachedBody,
                        bodyB: null
                    );

                    worldBeam.minLength = config.minLength;
                    worldBeam.maxLength = config.maxLength;
                    worldBeam.strength = config.strength;

                    m_attachedBody.solver.beams.Add(worldBeam);
                    m_addedBeams.Add(worldBeam);
                    m_linkToBeamsMap[i].Add(worldBeam);
                    successfulLinks++;
                }
            }
            Debug.Log($"[Constraint] Created {successfulLinks} World Constraints.");
        }

        private bool ValidateAttachment(int[] indices, AttachmentType type, string bodyLabel, int linkIndex, SoftBody body)
        {
            if (indices == null)
            {
                Debug.LogWarning($"Invalid attachment set for link {linkIndex} body {bodyLabel}. Set not found.", this);
                return false;
            }
            int requiredNodes = type == AttachmentType.Node ? 1 : 2;
            if (indices.Length != requiredNodes)
            {
                Debug.LogWarning($"Invalid attachment set for link {linkIndex} body {bodyLabel}. Expected {requiredNodes} nodes, got {indices.Length}.", this);
                return false;
            }
            foreach (int index in indices)
            {
                if (index < 0 || index >= body.solver.nodeManager.Nodes.Count)
                {
                    Debug.LogWarning($"Invalid node index {index} for link {linkIndex} body {bodyLabel}. Nodes={body.solver.nodeManager.Nodes.Count}. Skipping.", this);
                    return false;
                }
            }
            return true;
        }

        private Vector3 GetAttachmentPoint(SoftBody body, int[] indices, AttachmentType type)
        {
            if (body == null || body.solver == null || body.solver.nodeManager == null)
                return Vector3.zero;

            var predicted = body.solver.nodeManager.PredictedPositions;

            if (type == AttachmentType.Node)
            {
                int index = indices[0];
                if (index < 0 || index >= predicted.Count) return Vector3.zero;
                return predicted[index];
            }
            else
            {
                int indexA = indices[0];
                int indexB = indices[1];

                if (indexA < 0 || indexA >= predicted.Count) return Vector3.zero;
                if (indexB < 0 || indexB >= predicted.Count) return Vector3.zero;

                return (predicted[indexA] + predicted[indexB]) * 0.5f;
            }
        }

        private void OnDestroy()
        {
            RestoreCollisionPairs();

            if (m_baseBody != null && m_baseBody.solver != null)
            {
                foreach (var beam in m_addedBeams)
                {
                    m_baseBody.solver.beams.Remove(beam);
                }
            }
        }

        private void OnValidate()
        {
            if (m_links != null)
            {
                for (int i = 0; i < m_links.Count; i++)
                {
                    var config = m_links[i];

                    // Note: Removed the Mathf.Max(0f) check to allow -1 (auto)

                    if (!float.IsInfinity(config.strength))
                    {
                        config.strength = Mathf.Max(1f, config.strength);
                    }
                    m_links[i] = config;
                }
            }

            linkVisibility.Clear();
            for (int i = 0; i < m_links.Count; i++)
            {
                linkVisibility.Add(m_links[i].show);
            }

            if (Application.isPlaying)
            {
                UpdateBeamPropertiesFromLinks();
            }
        }

        private bool IsLinkBroken(int linkIndex)
        {
            if (!m_linkToBeamsMap.ContainsKey(linkIndex))
                return false;

            var beams = m_linkToBeamsMap[linkIndex];
            if (beams == null || beams.Count == 0)
                return false;

            foreach (var beam in beams)
            {
                if (!beam.isBroken)
                    return false;
            }

            return true;
        }

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;

            if (!m_showLinks) return;
            if (linkVisibility == null || linkVisibility.Count != m_links.Count) return;

            SoftBody bodyA = GetComponent<SoftBody>();
            SoftBody bodyB = m_baseBody;

            if (bodyA == null || bodyA.truss == null) return;
            if (bodyB != null && bodyB.truss == null) return;

            for (int i = 0; i < m_links.Count; i++)
            {
                if (!linkVisibility[i]) continue;

                if (Application.isPlaying && IsLinkBroken(i))
                    continue;

                var link = m_links[i];

                int[] indicesA = bodyA?.truss?.GetNodeSetIndices(link.attachmentSetA);
                if (indicesA == null) continue;

                Vector3 posA = GetAttachmentPoint(bodyA, indicesA, link.attachmentTypeA);
                if (posA == Vector3.zero && bodyA.transform.position.magnitude > 0.1f) continue;

                if (bodyB != null)
                {
                    int[] indicesB = bodyB.truss.GetNodeSetIndices(link.attachmentSetB);
                    if (indicesB == null) continue;

                    Vector3 posB;
                    if (link.attachmentTypeB == AttachmentType.Edge && indicesB.Length == 2)
                    {
                        posB = GetProjectedPointOnEdge(bodyB, indicesB, posA);
                        if (posB == Vector3.zero && bodyB.transform.position.magnitude > 0.1f) continue;

                        Vector3 edgeStart, edgeEnd;
                        if (Application.isPlaying && bodyB.solver != null && bodyB.solver.nodeManager != null)
                        {
                            edgeStart = bodyB.solver.nodeManager.PredictedPositions[indicesB[0]];
                            edgeEnd = bodyB.solver.nodeManager.PredictedPositions[indicesB[1]];
                        }
                        else
                        {
                            edgeStart = bodyB.transform.TransformPoint(bodyB.truss.NodePositions[indicesB[0]]);
                            edgeEnd = bodyB.transform.TransformPoint(bodyB.truss.NodePositions[indicesB[1]]);
                        }

                        Gizmos.color = new Color(0.7f, 0.7f, 0.7f, 0.5f);

                        Vector3 edgeDir = (edgeEnd - edgeStart).normalized;
                        if (edgeDir.sqrMagnitude > 0f)
                        {
                            Gizmos.DrawLine(edgeStart - edgeDir * 1000f, edgeEnd + edgeDir * 1000f);
                        }
                        else
                        {
                            Gizmos.DrawLine(edgeStart, edgeEnd);
                        }

                        Gizmos.DrawSphere(edgeStart, 0.015f);
                        Gizmos.DrawSphere(edgeEnd, 0.015f);
                    }
                    else if (link.attachmentTypeA == AttachmentType.Edge && indicesA.Length == 2)
                    {
                        posB = GetAttachmentPoint(bodyB, indicesB, link.attachmentTypeB);
                        if (posB == Vector3.zero && bodyB.transform.position.magnitude > 0.1f) continue;

                        posA = GetProjectedPointOnEdge(bodyA, indicesA, posB);
                        if (posA == Vector3.zero && bodyA.transform.position.magnitude > 0.1f) continue;

                        Vector3 edgeStart, edgeEnd;
                        if (Application.isPlaying && bodyA.solver != null && bodyA.solver.nodeManager != null)
                        {
                            edgeStart = bodyA.solver.nodeManager.PredictedPositions[indicesA[0]];
                            edgeEnd = bodyA.solver.nodeManager.PredictedPositions[indicesA[1]];
                        }
                        else
                        {
                            edgeStart = bodyA.transform.TransformPoint(bodyA.truss.NodePositions[indicesA[0]]);
                            edgeEnd = bodyA.transform.TransformPoint(bodyA.truss.NodePositions[indicesA[1]]);
                        }

                        Gizmos.color = new Color(0.7f, 0.7f, 0.7f, 0.5f);

                        Vector3 edgeDir = (edgeEnd - edgeStart).normalized;
                        if (edgeDir.sqrMagnitude > 0f)
                        {
                            Gizmos.DrawLine(edgeStart - edgeDir * 1000f, edgeEnd + edgeDir * 1000f);
                        }
                        else
                        {
                            Gizmos.DrawLine(edgeStart, edgeEnd);
                        }

                        Gizmos.DrawSphere(edgeStart, 0.015f);
                        Gizmos.DrawSphere(edgeEnd, 0.015f);
                    }
                    else
                    {
                        posB = GetAttachmentPoint(bodyB, indicesB, link.attachmentTypeB);
                        if (posB == Vector3.zero && bodyB.transform.position.magnitude > 0.1f) continue;
                    }

                    Gizmos.color = link.master ? Color.red : Color.cyan;
                    Gizmos.DrawLine(posA, posB);
                    Gizmos.color = link.master ? new Color(1f, 0f, 0f, 0.6f) : new Color(0f, 1f, 1f, 0.6f);
                    Gizmos.DrawSphere(posA, 0.02f);
                    Gizmos.DrawSphere(posB, 0.02f);

                    if (link.minLength > 0)
                    {
                        Gizmos.color = new Color(1f, 0f, 0f, 0.4f);
                        DrawWireCircle(posA, Vector3.up, link.minLength, 32);
                        DrawWireCircle(posA, Vector3.right, link.minLength, 32);
                        DrawWireCircle(posA, Vector3.forward, link.minLength, 32);
                    }

                    if (link.maxLength > 0)
                    {
                        Gizmos.color = new Color(0f, 1f, 0f, 0.4f);
                        DrawWireCircle(posA, Vector3.up, link.maxLength, 32);
                        DrawWireCircle(posA, Vector3.right, link.maxLength, 32);
                        DrawWireCircle(posA, Vector3.forward, link.maxLength, 32);
                    }
                }
                else
                {
                    Gizmos.color = link.master ? Color.red : Color.green;
                    Gizmos.DrawSphere(posA, 0.025f);

                    if (link.maxLength > 0)
                    {
                        Gizmos.color = new Color(0f, 1f, 0f, 0.3f);
                        DrawWireCircle(posA, Vector3.up, link.maxLength, 32);
                        DrawWireCircle(posA, Vector3.right, link.maxLength, 32);
                        DrawWireCircle(posA, Vector3.forward, link.maxLength, 32);
                    }
                }
            }
        }

        private void DrawWireCircle(Vector3 center, Vector3 normal, float radius, int segments)
        {
            Vector3 forward = Vector3.Slerp(normal, -normal, 0.5f);
            Vector3 right = Vector3.Cross(normal, forward).normalized * radius;
            forward = Vector3.Cross(right, normal).normalized * radius;

            Vector3 prevPoint = center + right;
            for (int i = 1; i <= segments; i++)
            {
                float angle = (i / (float)segments) * Mathf.PI * 2;
                Vector3 nextPoint = center + right * Mathf.Cos(angle) + forward * Mathf.Sin(angle);
                Gizmos.DrawLine(prevPoint, nextPoint);
                prevPoint = nextPoint;
            }
        }

        private Vector3 GetProjectedPointOnEdge(SoftBody body, int[] edgeIndices, Vector3 point)
        {
            if (body == null || body.truss == null || edgeIndices == null || edgeIndices.Length != 2)
                return Vector3.zero;

            bool useRuntime = Application.isPlaying && body.solver != null && body.solver.nodeManager != null;
            int maxIndex = useRuntime ? body.solver.nodeManager.Nodes.Count : body.truss.NodePositions.Length;
            if (edgeIndices[0] < 0 || edgeIndices[0] >= maxIndex || edgeIndices[1] < 0 || edgeIndices[1] >= maxIndex)
                return Vector3.zero;

            Vector3 edgeStart, edgeEnd;
            if (useRuntime)
            {
                edgeStart = body.solver.nodeManager.PredictedPositions[edgeIndices[0]];
                edgeEnd = body.solver.nodeManager.PredictedPositions[edgeIndices[1]];
            }
            else
            {
                edgeStart = body.transform.TransformPoint(body.truss.NodePositions[edgeIndices[0]]);
                edgeEnd = body.transform.TransformPoint(body.truss.NodePositions[edgeIndices[1]]);
            }

            Vector3 edgeVector = edgeEnd - edgeStart;
            float edgeLength = edgeVector.magnitude;

            if (edgeLength < 0.001f) return edgeStart;

            Vector3 edgeDir = edgeVector / edgeLength;
            Vector3 pointToEdgeStart = point - edgeStart;
            float t = Vector3.Dot(pointToEdgeStart, edgeDir);

            // UNCLAMPED FOR INFINITE EDGE
            return edgeStart + edgeDir * t;
        }
    }
}