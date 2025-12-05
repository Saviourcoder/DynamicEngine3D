/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers */
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
        public AttachmentType attachmentTypeA;
        public string attachmentSetA;
        public AttachmentType attachmentTypeB;
        public string attachmentSetB;
        public float restLength;
        public float minLength;
        public float maxLength;
        public float stiffness;      // Now used as spring constant (k), higher = stiffer
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

        [Header("Motor")]
        [SerializeField] private bool m_enableMotor = false;
        [SerializeField] private string m_axisNodeSet = "";
        [SerializeField] private float m_targetRate = 90f;
        [SerializeField] private float m_maxTorque = 100f;

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
            InitializeBeams();

            if (Application.isPlaying)
                UnityEditor.SceneView.RepaintAll();

            for (int i = 0; i < m_links.Count; i++)
            {
                var link = m_links[i];

                var bodyA = m_attachedBody;
                var bodyB = m_baseBody;
                if (bodyA == null || bodyA.solver == null) continue;

                int[] a = bodyA.truss.GetNodeSetIndices(link.attachmentSetA);
                if (a == null || a.Length == 0) continue;

                Vector3 posA = bodyA.solver.nodeManager.PredictedPositions[a[0]];

                Vector3 posB;
                if (bodyB != null)
                {
                    int[] b = bodyB.truss.GetNodeSetIndices(link.attachmentSetB);
                    if (b == null || b.Length == 0) continue;
                    posB = bodyB.solver.nodeManager.PredictedPositions[b[0]];
                }
                else continue;

                float dist = Vector3.Distance(posA, posB);

                Debug.Log(
                    $"[RigidDebug] Link {i}\n" +
                    $"CurrentDistance = {dist}\n" +
                    $"RestLength = {link.restLength}\n" +
                    $"MinLength = {link.minLength}\n" +
                    $"MaxLength = {link.maxLength}"
                );
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

            if (m_attachedBody.solver?.nodeManager?.Nodes == null || m_baseBody.solver?.nodeManager?.Nodes == null ||
                m_attachedBody.solver?.nodeManager?.PredictedPositions == null || m_baseBody.solver?.nodeManager?.PredictedPositions == null)
            {
                Debug.LogError($"Solver or node manager not properly initialized. " +
                $"attachedBody: Nodes={m_attachedBody.solver?.nodeManager?.Nodes?.Count ?? 0}, " +
                $"baseBody: Nodes={m_baseBody.solver?.nodeManager?.Nodes?.Count ?? 0}", this);
                return;
            }

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

                float restLength = config.restLength > 0 ? config.restLength : CalculateRestLength(indicesA, config.attachmentTypeA, indicesB, config.attachmentTypeB);

                if (config.attachmentTypeA == AttachmentType.Node && config.attachmentTypeB == AttachmentType.Node)
                {
                    CreateNodeToNodeBeam(indicesA[0], indicesB[0], config, restLength, ref successfulLinks, i);
                }
                else if (config.attachmentTypeA == AttachmentType.Node && config.attachmentTypeB == AttachmentType.Edge)
                {
                    CreateNodeToEdgeBeam(indicesA[0], indicesB, config, restLength, ref successfulLinks, i, true);
                }
                else if (config.attachmentTypeA == AttachmentType.Edge && config.attachmentTypeB == AttachmentType.Node)
                {
                    CreateEdgeToNodeBeam(indicesA, indicesB[0], config, restLength, ref successfulLinks, i, true);
                }
                else if (config.attachmentTypeA == AttachmentType.Edge && config.attachmentTypeB == AttachmentType.Edge)
                {
                    CreateEdgeToEdgeBeam(indicesA, indicesB, config, restLength, ref successfulLinks, i);
                }
            }
        }

        private void InitializeWorldConstraints()
        {
            if (m_attachedBody.truss == null)
            {
                Debug.LogError("Truss asset must be assigned to the soft body.", this);
                return;
            }

            int successfulLinks = 0;
            for (int i = 0; i < m_links.Count; i++)
            {
                var config = m_links[i];
                int[] indicesA = m_attachedBody.truss.GetNodeSetIndices(config.attachmentSetA);
                if (!ValidateAttachment(indicesA, config.attachmentTypeA, "A", i, m_attachedBody))
                    continue;

                foreach (int nodeIndex in indicesA)
                {
                    Debug.Log($"[Constraint] World constraint: Node {nodeIndex} with max distance {config.restLength}");
                    successfulLinks++;
                }
            }
            Debug.Log($"[Constraint] World constraints complete: {successfulLinks} constraints created");
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

        private float CalculateRestLength(int[] indicesA, AttachmentType typeA, int[] indicesB, AttachmentType typeB)
        {
            Vector3 pointA = GetAttachmentPoint(m_attachedBody, indicesA, typeA);
            Vector3 pointB = GetAttachmentPoint(m_baseBody, indicesB, typeB);
            return Vector3.Distance(pointA, pointB);
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
            else // Edge
            {
                int indexA = indices[0];
                int indexB = indices[1];

                if (indexA < 0 || indexA >= predicted.Count) return Vector3.zero;
                if (indexB < 0 || indexB >= predicted.Count) return Vector3.zero;

                return (predicted[indexA] + predicted[indexB]) * 0.5f;
            }
        }

        // ███████ UPDATED METHODS BELOW ███████

        private void CreateNodeToNodeBeam(int nodeA, int nodeB, LinkConfig config, float restLength, ref int successfulLinks, int linkIndex)
        {
            bool isRigid = Mathf.Approximately(config.minLength, config.maxLength);
            float baseCompliance = isRigid ? 0f : (config.stiffness > 0f ? 1f / config.stiffness : 1e-6f);
            int parallelCount = 1;
            float compliance = baseCompliance * parallelCount;

            float rangeFactor = Mathf.Clamp01((config.maxLength - config.minLength) / Mathf.Max(0.001f, restLength));
            float damping = Mathf.Lerp(0.1f, 0.5f, rangeFactor);

            Beam newBeam = new Beam(
                nodeA: nodeA,
                nodeB: nodeB,
                compliance: compliance,
                damping: damping,
                restLength: restLength,
                maxDeformation: m_attachedBody.solver.maxDeformation,
                bodyA: m_attachedBody,
                bodyB: m_baseBody
            );

            m_attachedBody.solver.beams.Add(newBeam);
            m_addedBeams.Add(newBeam);
            successfulLinks++;
        }

        private void CreateNodeToEdgeBeam(int nodeA, int[] edgeNodes, LinkConfig config, float restLength, ref int successfulLinks, int linkIndex, bool isNodeA)
        {
            bool isRigid = Mathf.Approximately(config.minLength, config.maxLength);
            float baseCompliance = isRigid ? 0f : (config.stiffness > 0f ? 1f / config.stiffness : 1e-6f);
            int parallelCount = 2;
            float compliance = baseCompliance * parallelCount;

            float rangeFactor = Mathf.Clamp01((config.maxLength - config.minLength) / Mathf.Max(0.001f, restLength));
            float damping = Mathf.Lerp(0.1f, 0.5f, rangeFactor);

            for (int i = 0; i < 2; i++)
            {
                int edgeNode = edgeNodes[i];

                Vector3 pointA = GetAttachmentPoint(m_attachedBody, new int[] { nodeA }, AttachmentType.Node);
                Vector3 pointB = GetAttachmentPoint(m_baseBody, new int[] { edgeNode }, AttachmentType.Node);
                float actualRestLength = Vector3.Distance(pointA, pointB);

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

                m_attachedBody.solver.beams.Add(newBeam);
                m_addedBeams.Add(newBeam);
                successfulLinks++;
            }
        }

        private void CreateEdgeToNodeBeam(int[] edgeNodes, int nodeB, LinkConfig config, float restLength, ref int successfulLinks, int linkIndex, bool isNodeB)
        {
            CreateNodeToEdgeBeam(nodeB, edgeNodes, config, restLength, ref successfulLinks, linkIndex, !isNodeB);
        }

        private void CreateEdgeToEdgeBeam(int[] edgeNodesA, int[] edgeNodesB, LinkConfig config, float restLength, ref int successfulLinks, int linkIndex)
        {
            bool isRigid = Mathf.Approximately(config.minLength, config.maxLength);
            float baseCompliance = isRigid ? 0f : (config.stiffness > 0f ? 1f / config.stiffness : 1e-6f);
            int parallelCount = 4;
            float compliance = baseCompliance * parallelCount;

            float rangeFactor = Mathf.Clamp01((config.maxLength - config.minLength) / Mathf.Max(0.001f, restLength));
            float damping = Mathf.Lerp(0.1f, 0.5f, rangeFactor);

            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    Vector3 pointA = GetAttachmentPoint(m_attachedBody, new int[] { edgeNodesA[i] }, AttachmentType.Node);
                    Vector3 pointB = GetAttachmentPoint(m_baseBody, new int[] { edgeNodesB[j] }, AttachmentType.Node);
                    float actualRestLength = Vector3.Distance(pointA, pointB);

                    Beam newBeam = new Beam(
                        nodeA: edgeNodesA[i],
                        nodeB: edgeNodesB[j],
                        compliance: compliance,
                        damping: damping,
                        restLength: actualRestLength,
                        maxDeformation: m_attachedBody.solver.maxDeformation,
                        bodyA: m_attachedBody,
                        bodyB: m_baseBody
                    );

                    m_attachedBody.solver.beams.Add(newBeam);
                    m_addedBeams.Add(newBeam);
                    successfulLinks++;
                }
            }
        }

        private void OnDestroy()
        {
            if (m_attachedBody != null && m_attachedBody.solver != null)
            {
                foreach (var beam in m_addedBeams)
                {
                    m_attachedBody.solver.beams.Remove(beam);
                }
                m_addedBeams.Clear();
            }
        }

        private void OnValidate()
        {
            if (m_links != null)
            {
                for (int i = 0; i < m_links.Count; i++)
                {
                    var config = m_links[i];

                    if (config.restLength > 0f)
                    {
                        config.minLength = config.restLength;
                        config.maxLength = config.restLength;
                    }

                    config.restLength = Mathf.Max(0f, config.restLength);
                    config.stiffness = Mathf.Max(1f, config.stiffness);

                    m_links[i] = config;
                }
            }

            linkVisibility.Clear();
            for (int i = 0; i < m_links.Count; i++)
            {
                linkVisibility.Add(m_links[i].show);
            }
        }

        private void OnDrawGizmos()
        {
            if (!m_showLinks) return;
            if (linkVisibility == null || linkVisibility.Count != m_links.Count) return;

            SoftBody bodyA = GetComponent<SoftBody>();
            SoftBody bodyB = m_baseBody;

            for (int i = 0; i < m_links.Count; i++)
            {
                if (!linkVisibility[i]) continue;

                var link = m_links[i];

                int[] indicesA = bodyA?.truss?.GetNodeSetIndices(link.attachmentSetA);
                if (indicesA == null) continue;

                Vector3 posA = GetAttachmentPoint(bodyA, indicesA, link.attachmentTypeA);

                if (bodyB != null)
                {
                    int[] indicesB = bodyB.truss.GetNodeSetIndices(link.attachmentSetB);
                    if (indicesB == null) continue;

                    Vector3 posB = GetAttachmentPoint(bodyB, indicesB, link.attachmentTypeB);

                    Gizmos.color = link.master ? Color.red : Color.red;
                    Gizmos.DrawLine(posA, posB);
                }
                else
                {
                    Gizmos.color = link.master ? Color.red : Color.green;
                    Gizmos.DrawSphere(posA, 0.02f);
                }
            }
        }
    }
}