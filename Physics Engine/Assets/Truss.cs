/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */

using UnityEngine;
using System.Collections.Generic;
using System.Linq;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    #region Truss
    [CreateAssetMenu(fileName = "Truss", menuName = "DynamicEngine/Truss")]
    public class Truss : ScriptableObject
    {
        [SerializeField] private Vector3[] nodePositions;
        [SerializeField] private List<Beam> beams = new List<Beam>();
        [SerializeField] private List<int> pinnedNodes = new List<int>();
        [SerializeField] private List<float> nodeMasses = new List<float>();
        [SerializeField] private float maxStretchFactor = 1.05f;
        [SerializeField] private float minStretchFactor = 0.95f;
        [SerializeField] private List<Face> faces = new List<Face>();
        [SerializeField] private List<NodeSet> nodeSets = new List<NodeSet>();
        [SerializeField] public float compliance = 0.01f;
        [SerializeField] public float DefaultBeamDamping = 0.3f;
        [SerializeField, HideInInspector] private float uniformMassEditorValue = 0.5f;


        public Vector3[] NodePositions => nodePositions;
        public List<int> PinnedNodes => pinnedNodes;
        public List<float> NodeMasses => nodeMasses;
        public float MaxStretchFactor => maxStretchFactor;
        public float MinStretchFactor => minStretchFactor;
        public List<Face> GetTrussFaces() => faces;
        public List<NodeSet> GetNodeSets() => nodeSets;
        public List<Beam> GetTrussBeams() => beams;

        public int[] GetNodeSetIndices(string nodeSetName)
        {
            NodeSet nodeSet = nodeSets.Find(ns => ns.name == nodeSetName);
            return nodeSet != null ? nodeSet.nodeIndices.ToArray() : null;
        }

        public void SetNodePositions(Vector3[] positions)
        {
            nodePositions = positions != null ? new Vector3[positions.Length] : new Vector3[0];
            if (positions != null)
                System.Array.Copy(positions, nodePositions, positions.Length);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetBeams(List<Beam> newBeams)
        {
            beams = newBeams != null ? new List<Beam>(newBeams) : new List<Beam>();
            foreach (var beam in beams)
            {
                beam.originalRestLength = beam.restLength;
            }
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetPinnedNodes(List<int> newPinnedNodes)
        {
            pinnedNodes = newPinnedNodes != null ? new List<int>(newPinnedNodes) : new List<int>();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetNodeMasses(List<float> masses)
        {
            nodeMasses = masses != null ? new List<float>(masses) : new List<float>();

            // Ensure size matches nodePositions
            while (nodeMasses.Count < nodePositions.Length)
                nodeMasses.Add(0.5f); // default mass
            while (nodeMasses.Count > nodePositions.Length)
                nodeMasses.RemoveAt(nodeMasses.Count - 1);

            for (int i = 0; i < nodeMasses.Count; i++)
            {
                nodeMasses[i] = Mathf.Max(0.1f, nodeMasses[i]);
            }
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetUniformNodeMass(float mass)  // Helper for backward compatibility or uniform set
        {
            float clampedMass = Mathf.Max(0.1f, mass);
            for (int i = 0; i < nodePositions.Length; i++)
            {
                if (i < nodeMasses.Count)
                    nodeMasses[i] = clampedMass;
                else
                    nodeMasses.Add(clampedMass);
            }
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetMaxStretchFactor(float maxStretch)
        {
            maxStretchFactor = Mathf.Max(1.0f, maxStretch);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetMinStretchFactor(float minStretch)
        {
            minStretchFactor = Mathf.Clamp(minStretch, 0.5f, 1.0f);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetPhysicsProperties(float mass, float maxStretch, float minStretch)
        {
            // Set uniform mass for all nodes
            float clampedMass = Mathf.Max(0.1f, mass);
            for (int i = 0; i < nodeMasses.Count; i++)
            {
                nodeMasses[i] = clampedMass;
            }
            // If nodeMasses is empty or doesn't match nodePositions, sync it
            if (nodeMasses.Count == 0 || nodeMasses.Count != nodePositions.Length)
            {
                nodeMasses.Clear();
                for (int i = 0; i < nodePositions.Length; i++)
                {
                    nodeMasses.Add(clampedMass);
                }
            }

            maxStretchFactor = Mathf.Max(1.0f, maxStretch);
            minStretchFactor = Mathf.Clamp(minStretch, 0.5f, 1.0f);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetNodeSets(List<NodeSet> newNodeSets)
        {
            nodeSets = newNodeSets != null ? new List<NodeSet>(newNodeSets) : new List<NodeSet>();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void AddNodeSet(NodeSet nodeSet)
        {
            if (nodeSet != null && !nodeSets.Exists(ns => ns.name == nodeSet.name))
            {
                nodeSets.Add(new NodeSet(nodeSet.name, nodeSet.nodeIndices)
                {
                    color = nodeSet.color,
                    isVisible = nodeSet.isVisible
                });
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }

        public void RemoveNodeSet(string name)
        {
            nodeSets.RemoveAll(ns => ns.name == name);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void ValidateNodeSets()
        {
            for (int i = nodeSets.Count - 1; i >= 0; i--)
            {
                var nodeSet = nodeSets[i];
                nodeSet.nodeIndices.RemoveAll(idx => idx < 0 || idx >= nodePositions.Length);
                if (nodeSet.nodeIndices.Count == 0)
                {
                    nodeSets.RemoveAt(i);
                }
            }
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void AddFace(Face face)
        {
            if (!faces.Exists(f => f.nodeA == face.nodeA && f.nodeB == face.nodeB && f.nodeC == face.nodeC))
            {
                faces.Add(face);
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }

        public void RemoveFace(int index)
        {
            if (index >= 0 && index < faces.Count)
            {
                faces.RemoveAt(index);
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }
    }
    #endregion
    #region NodeSet
    [System.Serializable]
    public class NodeSet
    {
        public string name;
        public List<int> nodeIndices = new List<int>();
        public Color color = Color.cyan;
        public bool isVisible = true;
        public NodeSet()
        {
            nodeIndices = new List<int>();
        }

        public NodeSet(string name, List<int> indices)
        {
            this.name = name;
            this.nodeIndices = new List<int>(indices);
        }

        public bool IsValid()
        {
            return !string.IsNullOrEmpty(name) && nodeIndices != null && nodeIndices.Count > 0;
        }

        public bool ContainsNode(int nodeIndex)
        {
            return nodeIndices.Contains(nodeIndex);
        }
    }
    #endregion
    #region Beam

    [System.Serializable]
    public class Beam
    {
        public int nodeA;
        public int nodeB;
        public float compliance = 1e-3f;
        public float damping = 0.3f;
        public float restLength;
        public float originalRestLength;
        public float lagrangeMultiplier;
        public float deformationScale = 0.1f;
        public float maxDeformation = 0.5f;
        public float plasticityThreshold = 0.05f;
        public float plasticityRate = 0.1f;
        [System.NonSerialized]
        public SoftBody bodyA;
        [System.NonSerialized]
        public SoftBody bodyB;
        public bool isActive = true;

        public Beam(int nodeA, int nodeB, float compliance, float damping, float restLength,
                    float deformationScale = 0.1f, float maxDeformation = 0.5f,
                    float plasticityThreshold = 0.05f, float plasticityRate = 0.1f,
                    SoftBody bodyA = null, SoftBody bodyB = null)
        {
            this.nodeA = nodeA;
            this.nodeB = nodeB;
            this.compliance = compliance;
            this.damping = damping;
            this.restLength = restLength;
            this.originalRestLength = restLength;
            this.lagrangeMultiplier = 0f;
            this.deformationScale = deformationScale;
            this.maxDeformation = maxDeformation;
            this.plasticityThreshold = plasticityThreshold;
            this.plasticityRate = plasticityRate;
            this.bodyA = bodyA;
            this.bodyB = bodyB ?? bodyA;
            this.isActive = true;
        }

        public void SetSoftBodyReferences(SoftBody bodyA, SoftBody bodyB = null)
        {
            this.bodyA = bodyA;
            this.bodyB = bodyB ?? bodyA;
        }

        public void SolveConstraint(float deltaTime, float massA = 1f, float massB = 1f)
        {
            if (!isActive) return;

            bool isCrossBody = bodyA != bodyB && bodyA != null && bodyB != null;

            Vector3 posA = isCrossBody ? GetNodeAWorldPosition() : GetNodeALocalPosition();
            Vector3 posB = isCrossBody ? GetNodeBWorldPosition() : GetNodeBLocalPosition();

            Vector3 delta = posB - posA;
            float currentLength = delta.magnitude;
            if (currentLength < 0.0001f) return;

            Vector3 gradient = delta.normalized;
            float constraint = currentLength - restLength;

            float strain = Mathf.Abs(constraint / restLength);
            if (strain > plasticityThreshold)
            {
                float plasticDeformation = strain * plasticityRate * deltaTime;
                restLength += Mathf.Clamp(plasticDeformation, -maxDeformation * originalRestLength, maxDeformation * originalRestLength);
                restLength = Mathf.Clamp(restLength, originalRestLength * 0.5f, originalRestLength * 2f);
            }

            Vector3 velA = GetNodeAVelocity(deltaTime);
            Vector3 velB = GetNodeBVelocity(deltaTime);
            float dampingTerm = damping * Vector3.Dot(velB - velA, gradient);

            float wA = IsNodeAPinned() ? 0f : 1f / massA;
            float wB = IsNodeBPinned() ? 0f : 1f / massB;
            float wSum = wA + wB;
            if (wSum == 0f) return;

            float dtSquared = deltaTime * deltaTime;
            float complianceValue = compliance / dtSquared;
            float lambda = -(constraint + dampingTerm * deltaTime) / (wSum + complianceValue);

            float maxLambda = 1000f;
            lambda = Mathf.Clamp(lambda, -maxLambda, maxLambda);
            lagrangeMultiplier += lambda;

            Vector3 correction = gradient * lambda;

            if (!IsNodeAPinned())
            {
                Vector3 corrA = wA * correction;
                if (isCrossBody) corrA = bodyA.transform.InverseTransformVector(corrA);
                if (corrA.magnitude > 0.1f) corrA = corrA.normalized * 0.1f;
                UpdateNodeAPosition(corrA);
            }

            if (!IsNodeBPinned())
            {
                Vector3 corrB = -wB * correction;
                if (isCrossBody) corrB = bodyB.transform.InverseTransformVector(corrB);
                if (corrB.magnitude > 0.1f) corrB = corrB.normalized * 0.1f;
                UpdateNodeBPosition(corrB);
            }

            Debug.DrawLine(posA, posB, Color.red, 0.1f);
        }

        public Vector3 GetNodeAWorldPosition()
        {
            return bodyA?.solver?.nodeManager?.Nodes?[nodeA]?.position ?? Vector3.zero;
        }

        public Vector3 GetNodeBWorldPosition()
        {
            return bodyB?.solver?.nodeManager?.Nodes?[nodeB]?.position ?? Vector3.zero;
        }

        public Vector3 GetNodeALocalPosition()
        {
            return bodyA?.solver?.nodeManager?.PredictedPositions?[nodeA] ?? Vector3.zero;
        }

        public Vector3 GetNodeBLocalPosition()
        {
            return bodyB?.solver?.nodeManager?.PredictedPositions?[nodeB] ?? Vector3.zero;
        }

        public bool AreNodesValid()
        {
            bool validA = bodyA?.solver?.nodeManager?.Nodes != null && nodeA >= 0 && nodeA < bodyA.solver.nodeManager.Nodes.Count && bodyA.solver.nodeManager.Nodes[nodeA] != null;
            bool validB = bodyB?.solver?.nodeManager?.Nodes != null && nodeB >= 0 && nodeB < bodyB.solver.nodeManager.Nodes.Count && bodyB.solver.nodeManager.Nodes[nodeB] != null;
            return validA && validB;
        }

        public bool IsNodeAPinned()
        {
            return bodyA?.solver?.nodeManager?.IsPinned?[nodeA] ?? false;
        }

        public bool IsNodeBPinned()
        {
            return bodyB?.solver?.nodeManager?.IsPinned?[nodeB] ?? false;
        }

        private Vector3 GetNodeAVelocity(float deltaTime)
        {
            Vector3 curr = GetNodeALocalPosition();
            Vector3 prev = bodyA?.solver?.nodeManager?.PreviousPositions?[nodeA] ?? curr;
            return (curr - prev) / deltaTime;
        }

        private Vector3 GetNodeBVelocity(float deltaTime)
        {
            Vector3 curr = GetNodeBLocalPosition();
            Vector3 prev = bodyB?.solver?.nodeManager?.PreviousPositions?[nodeB] ?? curr;
            return (curr - prev) / deltaTime;
        }

        private void UpdateNodeAPosition(Vector3 correction)
        {
            if (bodyA?.solver?.nodeManager?.PredictedPositions != null && nodeA < bodyA.solver.nodeManager.PredictedPositions.Count)
                bodyA.solver.nodeManager.PredictedPositions[nodeA] += correction;
        }

        private void UpdateNodeBPosition(Vector3 correction)
        {
            if (bodyB?.solver?.nodeManager?.PredictedPositions != null && nodeB < bodyB.solver.nodeManager.PredictedPositions.Count)
                bodyB.solver.nodeManager.PredictedPositions[nodeB] += correction;
        }
    }
    #endregion
    #region Face

    [System.Serializable]
    public struct Face
    {
        public int nodeA;
        public int nodeB;
        public int nodeC;

        public Face(int nodeA, int nodeB, int nodeC)
        {
            this.nodeA = nodeA;
            this.nodeB = nodeB;
            this.nodeC = nodeC;
        }
    }
    #endregion
}