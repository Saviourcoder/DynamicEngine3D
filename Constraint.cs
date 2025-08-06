/* DynamicEngine3D - Soft Body Constraint
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/

using System;
using UnityEngine;

namespace DynamicEngine
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(SoftBody))]
    public class Constraint : MonoBehaviour
    {
        public delegate void OnBreakFn();

        [SerializeField] private SoftBody baseBody = null;
        [SerializeField] private ConstraintData constraintData = null;
        [SerializeField] private bool showSnaps = false;

        private int jointID = -1;
        private int jointInstanceID = -1;
        private int motorID = -1;
        private int motorInstanceID = -1;
        private SoftBody attachedBody = null;
        private bool wasBroken = false;
        private Matrix4x4 startMatrix = Matrix4x4.identity;

        // Properties
        public bool EnableMotor => constraintData?.EnableMotor ?? false;
        public string motorAxis => constraintData?.AxisNodeSet ?? "";
        public SoftBody AttachedBody => attachedBody;
        public SoftBody BaseBody => baseBody;
        public Snap[] Snaps => constraintData?.Snaps ?? new Snap[0];
        public Matrix4x4 StartMatrix => startMatrix;
        public ConstraintData ConstraintData => constraintData;
        
        // Editor-accessible properties
        public bool ShowSnaps => showSnaps;

        public float motorRate
        {
            get => constraintData?.TargetRate ?? 0f;
            set
            {
                if (constraintData != null)
                {
                    constraintData.SetMotorConfiguration(constraintData.EnableMotor, constraintData.AxisNodeSet, value, constraintData.MaxTorque);
                }
                // Motor rate update would go here when implementing native backend
            }
        }

        public float motorTorque
        {
            get => constraintData?.MaxTorque ?? 0f;
            set
            {
                if (constraintData != null)
                {
                    constraintData.SetMotorConfiguration(constraintData.EnableMotor, constraintData.AxisNodeSet, constraintData.TargetRate, value);
                }
                // Motor torque update would go here when implementing native backend
            }
        }

        public bool isBroken
        {
            get
            {
                // Only check solver validity during play mode
                if (!Application.isPlaying)
                    return false;
                    
                // For now, check if constraint components still exist
                return attachedBody == null || baseBody == null;
            }
        }

        public event OnBreakFn onBreak;

        void Start()
        {
            if (Application.isPlaying)
            {
                CreateConstraint();
            }
        }

        void OnDestroy()
        {
            if (Application.isPlaying)
            {
                DestroyConstraint();
            }
        }

        void OnValidate()
        {
            // Validate constraint data snaps if available
            if (constraintData?.Snaps != null)
            {
                foreach (var snap in constraintData.Snaps)
                {
                    if (snap != null)
                    {
                        snap.minLimit = Mathf.Max(0f, snap.minLimit);
                        snap.maxLimit = Mathf.Max(snap.minLimit, snap.maxLimit);
                    }
                }
            }
        }

        void Update()
        {
            if (Application.isPlaying && !wasBroken && isBroken)
            {
                onBreak?.Invoke();
                wasBroken = true;
            }
        }

        private void CreateConstraint()
        {
            attachedBody = GetComponent<SoftBody>();
            if (attachedBody == null)
            {
                Debug.LogError($"SoftBodyConstraint: No SoftBody component found on {gameObject.name}");
                return;
            }

            // Store initial transform relationship
            startMatrix = baseBody ? 
                (baseBody.transform.worldToLocalMatrix * transform.localToWorldMatrix) : 
                transform.localToWorldMatrix;

            // Generate unique constraint ID
            jointID = GetHashCode();
            jointInstanceID = GetInstanceID();

            // Process each snap constraint from constraint data
            if (constraintData?.Snaps != null)
            {
                foreach (var snap in constraintData.Snaps)
                {
                    if (snap?.IsValid() == true)
                    {
                        ProcessSnap(snap);
                    }
                }
            }

            // Setup motor if enabled
            if (EnableMotor)
            {
                SetupMotor();
            }

            int snapCount = constraintData?.Snaps?.Length ?? 0;
            Debug.Log($"SoftBodyConstraint: Created constraint on {gameObject.name} with {snapCount} snaps from ConstraintData: {constraintData?.name}");
        }

        private void ProcessSnap(Snap snap)
        {
            var nodeIndices = FindNodeSet(snap.node);
            if (nodeIndices == null || nodeIndices.Length == 0)
            {
                Debug.LogWarning($"SoftBodyConstraint: Node set '{snap.node}' not found on {gameObject.name}");
                return;
            }

            switch (snap.type)
            {
                case SnapType.Point:
                    ProcessPointSnap(snap, nodeIndices);
                    break;
                case SnapType.Node:
                    ProcessNodeSnap(snap, nodeIndices);
                    break;
                case SnapType.Edge:
                    ProcessEdgeSnap(snap, nodeIndices);
                    break;
            }
        }

        private void ProcessPointSnap(Snap snap, int[] nodeIndices)
        {
            foreach (int nodeIndex in nodeIndices)
            {
                var node = GetNodeTransform(nodeIndex);
                if (node == null) continue;

                Vector3 targetPos = snap.targetPosition;
                if (baseBody != null)
                {
                    targetPos = baseBody.transform.InverseTransformPoint(targetPos);
                }

                CreatePointConstraint(node, targetPos, snap);
            }
        }

        private void ProcessNodeSnap(Snap snap, int[] nodeIndices)
        {
            if (!(baseBody is SoftBody baseSoftBody))
            {
                Debug.LogError($"SoftBodyConstraint: Base body must be a SoftBody for Node snap type on {gameObject.name}");
                return;
            }

            var baseNodeIndices = FindNodeSetOnSoftBody(baseSoftBody, snap.targetNode);
            if (baseNodeIndices == null || baseNodeIndices.Length != 1)
            {
                Debug.LogError($"SoftBodyConstraint: Target node set '{snap.targetNode}' should contain exactly one node on {baseBody.name}");
                return;
            }

            foreach (int nodeIndex in nodeIndices)
            {
                var nodeA = GetNodeTransform(nodeIndex);
                var nodeB = GetNodeTransformFromSoftBody(baseSoftBody, baseNodeIndices[0]);
                
                if (nodeA != null && nodeB != null)
                {
                    CreateNodeConstraint(nodeA, nodeB, snap);
                }
            }
        }

        private void ProcessEdgeSnap(Snap snap, int[] nodeIndices)
        {
            if (!(baseBody is SoftBody baseSoftBody))
            {
                Debug.LogError($"SoftBodyConstraint: Base body must be a SoftBody for Edge snap type on {gameObject.name}");
                return;
            }

            var baseNodeIndices = FindNodeSetOnSoftBody(baseSoftBody, snap.targetNode);
            if (baseNodeIndices == null || baseNodeIndices.Length != 2)
            {
                Debug.LogError($"SoftBodyConstraint: Target node set '{snap.targetNode}' should contain exactly two nodes for edge snap on {baseBody.name}");
                return;
            }

            foreach (int nodeIndex in nodeIndices)
            {
                var node = GetNodeTransform(nodeIndex);
                var edgeNodeA = GetNodeTransformFromSoftBody(baseSoftBody, baseNodeIndices[0]);
                var edgeNodeB = GetNodeTransformFromSoftBody(baseSoftBody, baseNodeIndices[1]);
                
                if (node != null && edgeNodeA != null && edgeNodeB != null)
                {
                    CreateEdgeConstraint(node, edgeNodeA, edgeNodeB, snap);
                }
            }
        }

        private void CreatePointConstraint(Transform node, Vector3 targetPosition, Snap snap)
        {
            // Create a virtual anchor node at the target position for point constraints
            int nodeIndex = GetNodeIndex(node);
            if (nodeIndex == -1) return;

            // For point constraints, we'll create a fixed anchor point in the solver
            var solver = attachedBody?.solver;
            if (solver != null)
            {
                // Create a virtual anchor at the target position
                CreateVirtualAnchor(nodeIndex, targetPosition, snap);
            }
            
            Debug.Log($"Created point constraint from node {nodeIndex} to position {targetPosition}");
        }

        private void CreateNodeConstraint(Transform nodeA, Transform nodeB, Snap snap)
        {
            // Use your custom Link system to create node-to-node constraints
            int nodeIndexA = GetNodeIndex(nodeA);
            int nodeIndexB = GetNodeIndexFromSoftBody(baseBody as SoftBody, nodeB);
            
            if (nodeIndexA == -1 || nodeIndexB == -1) return;

            // Create custom Link between the two nodes
            CreateCustomLink(nodeIndexA, nodeIndexB, snap);
            
            Debug.Log($"Created node constraint between node {nodeIndexA} and target node {nodeIndexB}");
        }

        private void CreateEdgeConstraint(Transform node, Transform edgeNodeA, Transform edgeNodeB, Snap snap)
        {
            // For edge constraints, create links to both edge nodes with weighted strength
            int nodeIndex = GetNodeIndex(node);
            int edgeNodeIndexA = GetNodeIndexFromSoftBody(baseBody as SoftBody, edgeNodeA);
            int edgeNodeIndexB = GetNodeIndexFromSoftBody(baseBody as SoftBody, edgeNodeB);
            
            if (nodeIndex == -1 || edgeNodeIndexA == -1 || edgeNodeIndexB == -1) return;

            // Create weighted links to both edge nodes
            float strengthA = snap.strength * 0.5f; // Split strength between both edge nodes
            float strengthB = snap.strength * 0.5f;
            
            var edgeSnapA = new Snap { 
                type = SnapType.Node, 
                strength = strengthA, 
                minLimit = snap.minLimit, 
                maxLimit = snap.maxLimit 
            };
            var edgeSnapB = new Snap { 
                type = SnapType.Node, 
                strength = strengthB, 
                minLimit = snap.minLimit, 
                maxLimit = snap.maxLimit 
            };
            
            CreateCustomLink(nodeIndex, edgeNodeIndexA, edgeSnapA);
            CreateCustomLink(nodeIndex, edgeNodeIndexB, edgeSnapB);
            
            Debug.Log($"Created edge constraint from node {nodeIndex} to edge nodes {edgeNodeIndexA}-{edgeNodeIndexB}");
        }

        private void CreateCustomLink(int nodeA, int nodeB, Snap snap)
        {
            // Get the Designer to add custom links
            var nodeLinkEditor = attachedBody?.GetComponent<Designer>();
            if (nodeLinkEditor == null)
            {
                Debug.LogWarning($"No Designer found on {attachedBody.name}. Cannot create custom link.");
                return;
            }

            // Calculate rest length between nodes
            var nodeATransform = GetNodeTransform(nodeA);
            var nodeBTransform = GetNodeTransformFromSoftBody(baseBody as SoftBody, nodeB);
            
            if (nodeATransform == null || nodeBTransform == null)
            {
                Debug.LogWarning($"Could not get transforms for nodes {nodeA} and {nodeB}");
                return;
            }

            float restLength = Vector3.Distance(nodeATransform.position, nodeBTransform.position);
            
            // Clamp rest length to snap limits if specified
            if (snap.minLimit > 0) restLength = Mathf.Max(restLength, snap.minLimit);
            if (snap.maxLimit > 0) restLength = Mathf.Min(restLength, snap.maxLimit);

            // Create custom Link using your DynamicEngine Link system
            var customLink = new Link
            {
                nodeA = nodeA,
                nodeB = nodeB,
                springForce = snap.strength == float.PositiveInfinity ? 10000f : snap.strength,
                damping = CalculateDamping(snap.strength),
                restLength = restLength,
                deformationScale = 0.1f,
                maxDeformation = snap.maxLimit > 0 ? snap.maxLimit : 1.0f,
                plasticityThreshold = 0.8f,
                plasticityRate = 0.1f
            };

            // Add the link to the Designer
            nodeLinkEditor.links.Add(customLink);
            
            // Save to truss asset to persist the link
            nodeLinkEditor.SaveToTrussAsset();
            
            Debug.Log($"Created custom Link: nodeA={nodeA}, nodeB={nodeB}, springForce={customLink.springForce}, restLength={customLink.restLength}");
        }

        private void CreateVirtualAnchor(int nodeIndex, Vector3 anchorPosition, Snap snap)
        {
            // For point constraints, we need to create a constraint to a fixed world position
            // This might require adding support for fixed anchors in your solver
            var solver = attachedBody?.solver;
            if (solver == null) return;

            // You might need to implement this in your Solver class
            // For now, log what would be created
            Debug.Log($"Virtual anchor needed: node {nodeIndex} -> position {anchorPosition}, strength {snap.strength}");
            
            // TODO: Implement virtual anchor support in your Solver
            // This could be done by:
            // 1. Adding a fixed anchor node at the target position
            // 2. Creating a link between the node and the anchor
            // 3. Marking the anchor as immovable in the physics simulation
        }

        private float CalculateDamping(float strength)
        {
            // Calculate appropriate damping based on strength
            if (strength == float.PositiveInfinity) return 0.005f; // Very rigid
            if (strength > 5000f) return 0.01f;  // Rigid
            if (strength > 1000f) return 0.05f;  // Medium
            return 0.1f; // Soft
        }

        private int GetNodeIndex(Transform nodeTransform)
        {
            // Find the index of a node transform in the attached body
            if (attachedBody?.solver?.nodeManager?.Nodes == null) return -1;
            
            var nodes = attachedBody.solver.nodeManager.Nodes;
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] == nodeTransform) return i;
            }
            return -1;
        }

        private int GetNodeIndexFromSoftBody(SoftBody softBody, Transform nodeTransform)
        {
            // Find the index of a node transform in the specified soft body
            if (softBody?.solver?.nodeManager?.Nodes == null) return -1;
            
            var nodes = softBody.solver.nodeManager.Nodes;
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] == nodeTransform) return i;
            }
            return -1;
        }

        private void SetupMotor()
        {
            string axisNodeSet = constraintData?.AxisNodeSet ?? "";
            if (string.IsNullOrEmpty(axisNodeSet)) return;

            var axisNodes = FindNodeSet(axisNodeSet);
            if (axisNodes == null || axisNodes.Length != 2)
            {
                Debug.LogError($"SoftBodyConstraint: Motor axis node set '{axisNodeSet}' should contain exactly two nodes on {gameObject.name}");
                return;
            }

            motorID = GetHashCode() + 1000;
            motorInstanceID = GetInstanceID() + 1000;

            float targetRate = constraintData?.TargetRate ?? 0f;
            float maxTorque = constraintData?.MaxTorque ?? 0f;

            // Motor implementation would go here
            Debug.Log($"SoftBodyConstraint: Motor setup between nodes {axisNodes[0]} and {axisNodes[1]} with rate {targetRate} and torque {maxTorque}");
        }

        private void DestroyConstraint()
        {
            // Clean up custom links created by this constraint
            var nodeLinkEditor = attachedBody?.GetComponent<Designer>();
            if (nodeLinkEditor != null)
            {
                // Remove links that were created by this constraint
                // You might want to track which links belong to this constraint
                // For now, this is a placeholder for proper cleanup
                Debug.Log($"Constraint destroyed on {gameObject.name}. Custom link cleanup needed.");
                
                // TODO: Implement proper link tracking and cleanup
                // This could involve:
                // 1. Storing references to created links when they're made
                // 2. Removing only those specific links on destruction
                // 3. Updating the truss asset after cleanup
            }
            
            // Also clean up any remaining Unity joints as fallback
            var joints = GetComponentsInChildren<Joint>();
            foreach (var joint in joints)
            {
                if (Application.isPlaying)
                    Destroy(joint);
                else
                    DestroyImmediate(joint);
            }
        }

        private int[] FindNodeSet(string nodeSetName)
        {
            return NodeSetHelper.FindNodeSet(attachedBody, nodeSetName);
        }

        private int[] FindNodeSetOnSoftBody(SoftBody softBody, string nodeSetName)
        {
            return NodeSetHelper.FindNodeSet(softBody, nodeSetName);
        }

        private Transform GetNodeTransform(int nodeIndex)
        {
            // Only access solver during play mode
            if (!Application.isPlaying || attachedBody?.solver?.nodeManager?.Nodes == null) 
                return null;
            if (nodeIndex < 0 || nodeIndex >= attachedBody.solver.nodeManager.Nodes.Count) return null;
            
            return attachedBody.solver.nodeManager.Nodes[nodeIndex];
        }

        private Transform GetNodeTransformFromSoftBody(SoftBody softBody, int nodeIndex)
        {
            // Only access solver during play mode
            if (!Application.isPlaying || softBody?.solver?.nodeManager?.Nodes == null) 
                return null;
            if (nodeIndex < 0 || nodeIndex >= softBody.solver.nodeManager.Nodes.Count) return null;
            
            return softBody.solver.nodeManager.Nodes[nodeIndex];
        }

        private Vector3 GetClosestPointOnLineSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
        {
            Vector3 line = lineEnd - lineStart;
            float lineLength = line.magnitude;
            
            if (lineLength < 0.001f)
                return lineStart;
                
            Vector3 lineDirection = line / lineLength;
            Vector3 toPoint = point - lineStart;
            
            float projectionLength = Vector3.Dot(toPoint, lineDirection);
            projectionLength = Mathf.Clamp(projectionLength, 0f, lineLength);
            
            return lineStart + lineDirection * projectionLength;
        }

        void OnDrawGizmos()
        {
            // Only draw gizmos with solver access during play mode
            if (!showSnaps || !Application.isPlaying || constraintData?.Snaps == null) return;

            foreach (var snap in constraintData.Snaps)
            {
                if (snap?.show == true)
                {
                    DrawSnapGizmo(snap);
                }
            }
        }

        private void DrawSnapGizmo(Snap snap)
        {
            var nodeIndices = FindNodeSet(snap.node);
            if (nodeIndices == null) return;

            Gizmos.color = snap.master ? Color.red : Color.yellow;
            
            foreach (int nodeIndex in nodeIndices)
            {
                var node = GetNodeTransform(nodeIndex);
                if (node == null) continue;

                switch (snap.type)
                {
                    case SnapType.Point:
                        Gizmos.DrawLine(node.position, snap.targetPosition);
                        Gizmos.DrawWireSphere(snap.targetPosition, 0.1f);
                        break;
                        
                    case SnapType.Node:
                        if (baseBody is SoftBody baseSoftBody)
                        {
                            var baseIndices = FindNodeSetOnSoftBody(baseSoftBody, snap.targetNode);
                            if (baseIndices != null && baseIndices.Length > 0)
                            {
                                var targetNode = GetNodeTransformFromSoftBody(baseSoftBody, baseIndices[0]);
                                if (targetNode != null)
                                {
                                    Gizmos.DrawLine(node.position, targetNode.position);
                                }
                            }
                        }
                        break;
                        
                    case SnapType.Edge:
                        if (baseBody is SoftBody baseSoftBody2)
                        {
                            var baseIndices = FindNodeSetOnSoftBody(baseSoftBody2, snap.targetNode);
                            if (baseIndices != null && baseIndices.Length == 2)
                            {
                                var nodeA = GetNodeTransformFromSoftBody(baseSoftBody2, baseIndices[0]);
                                var nodeB = GetNodeTransformFromSoftBody(baseSoftBody2, baseIndices[1]);
                                if (nodeA != null && nodeB != null)
                                {
                                    Vector3 closestPoint = GetClosestPointOnLineSegment(node.position, nodeA.position, nodeB.position);
                                    Gizmos.DrawLine(node.position, closestPoint);
                                    Gizmos.DrawLine(nodeA.position, nodeB.position);
                                }
                            }
                        }
                        break;
                }
                
                // Draw distance limits
                if (snap.maxLimit > 0)
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawWireSphere(node.position, snap.maxLimit);
                }
                if (snap.minLimit > 0)
                {
                    Gizmos.color = Color.green;
                    Gizmos.DrawWireSphere(node.position, snap.minLimit);
                }
            }
        }
    }
}