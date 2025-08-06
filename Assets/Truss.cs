/* DynamicEngine3D - Soft Body Simulation
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/
using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    [CreateAssetMenu(fileName = "TrussAsset", menuName = "SoftBody/TrussAsset", order = 1)]
    public class TrussAsset : ScriptableObject
    {
        [System.Serializable]
        public class TrussBeam
        {
            [Tooltip("Index of the first node (must be valid in Node Positions)")]
            public int nodeA;
            [Tooltip("Index of the second node (must be valid in Node Positions)")]
            public int nodeB;
            [Tooltip("Compliance (inverse stiffness) for the beam (lower = stiffer)")]
            public float compliance = 1e-3f;
            [Tooltip("Damping factor for the beam")]
            public float damping = 0.3f;
            [Tooltip("Rest length of the beam (set automatically if 0)")]
            public float restLength;

            public TrussBeam(int nodeA, int nodeB, float compliance, float damping, float restLength)
            {
                this.nodeA = nodeA;
                this.nodeB = nodeB;
                this.compliance = compliance;
                this.damping = damping;
                this.restLength = restLength;
            }

            public Beam ToBeam(Vector3[] nodePositions)
            {
                if (nodeA < 0 || nodeA >= nodePositions.Length || nodeB < 0 || nodeB >= nodePositions.Length)
                {
                    Debug.LogWarning($"Invalid TrussBeam: nodeA={nodeA}, nodeB={nodeB}, nodePositions.Length={nodePositions.Length}");
                    return new Beam(0, 0, 1e-3f, 0.3f, 0.01f);
                }
                float length = restLength > 0 ? restLength : Vector3.Distance(nodePositions[nodeA], nodePositions[nodeB]);
                if (length < 0.01f)
                {
                    Debug.LogWarning($"TrussBeam rest length too small: {length}");
                    length = 0.01f;
                }
                return new Beam(nodeA, nodeB, compliance, damping, length);
            }
        }

        [System.Serializable]
        public class TrussFace
        {
            [Tooltip("Index of the first node of the triangle")]
            public int nodeA;
            [Tooltip("Index of the second node of the triangle")]
            public int nodeB;
            [Tooltip("Index of the third node of the triangle")]
            public int nodeC;

            public TrussFace(int a, int b, int c)
            {
                nodeA = a;
                nodeB = b;
                nodeC = c;
            }

            public bool IsValid(Vector3[] nodePositions)
            {
                return nodeA >= 0 && nodeA < nodePositions.Length &&
                       nodeB >= 0 && nodeB < nodePositions.Length &&
                       nodeC >= 0 && nodeC < nodePositions.Length &&
                       nodeA != nodeB && nodeB != nodeC && nodeA != nodeC;
            }
        }

        [SerializeField, Tooltip("Positions of nodes in local space")]
        private Vector3[] nodePositions = new Vector3[0];

        [SerializeField, Tooltip("Beams connecting nodes")]
        private List<TrussBeam> beams = new List<TrussBeam>();

        [SerializeField, Tooltip("Triangular faces defined by nodes")]
        private List<TrussFace> faces = new List<TrussFace>();

        [Header("Material Properties")]
        [SerializeField, Tooltip("Material properties for this truss")]
        private MaterialProps materialProperties = MaterialProps.GetDefault(MaterialType.Custom);

        [SerializeField, Tooltip("Node mass for physics simulation")]
        private float nodeMass = 0.5f;

        [SerializeField, Tooltip("Maximum stretch factor for beams")]
        private float maxStretchFactor = 1.05f;

        [SerializeField, Tooltip("Minimum stretch factor for beams")]
        private float minStretchFactor = 0.95f;

        [SerializeField, Tooltip("Indices of pinned nodes")]
        private List<int> pinnedNodes = new List<int>();

        [SerializeField, Tooltip("Named node sets for easy constraint setup")]
        private List<NodeSet> nodeSets = new List<NodeSet>();

        [Header("Pressure System")]
        [SerializeField, Tooltip("Center of mass point for pressure calculations")]
        private Vector3 centerOfMass = Vector3.zero;

        public Vector3[] NodePositions => nodePositions;
        public Vector3 CenterOfMass => centerOfMass;
        public MaterialProps MaterialProperties => materialProperties;
        public float NodeMass => nodeMass;
        public float MaxStretchFactor => maxStretchFactor;
        public float MinStretchFactor => minStretchFactor;
        public IReadOnlyList<int> PinnedNodes => pinnedNodes.AsReadOnly();
        public IReadOnlyList<NodeSet> NodeSets => nodeSets.AsReadOnly();

        public IReadOnlyList<TrussBeam> GetTrussBeams() => beams.AsReadOnly();

        public IReadOnlyList<TrussFace> GetTrussFaces() => faces.AsReadOnly();

        public Beam[] GetBeams()
        {
            Beam[] result = new Beam[beams.Count];
            for (int i = 0; i < beams.Count; i++)
            {
                result[i] = beams[i].ToBeam(nodePositions);
            }
            return result;
        }

        public void SetNodePositions(Vector3[] positions)
        {
            if (positions == null)
                return;
            nodePositions = positions;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetMaterialProperties(MaterialProps props)
        {
            if (props == null)
                return;
            materialProperties = props;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetPhysicsProperties(float mass, float maxStretch, float minStretch, List<int> pinned)
        {
            nodeMass = mass;
            maxStretchFactor = maxStretch;
            minStretchFactor = minStretch;
            pinnedNodes = pinned ?? new List<int>();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetCenterOfMass(Vector3 centerOfMass)
        {
            this.centerOfMass = centerOfMass;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void CalculateAndSetCenterOfMass()
        {
            if (nodePositions == null || nodePositions.Length == 0)
            {
                centerOfMass = Vector3.zero;
                return;
            }

            Vector3 sum = Vector3.zero;
            foreach (var pos in nodePositions)
            {
                sum += pos;
            }
            centerOfMass = sum / nodePositions.Length;
            
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetBeams(List<TrussBeam> newBeams)
        {
            if (newBeams == null)
                return;
            beams = newBeams;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetFaces(List<TrussFace> newFaces)
        {
            if (newFaces == null)
                return;
            faces = newFaces;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetNodeSets(List<NodeSet> newNodeSets)
        {
            if (newNodeSets == null)
                return;
            nodeSets = newNodeSets;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public NodeSet GetNodeSet(string name)
        {
            foreach (var nodeSet in nodeSets)
            {
                if (nodeSet.name == name)
                    return nodeSet;
            }
            return null;
        }

        public void AddNodeSet(NodeSet nodeSet)
        {
            if (nodeSet != null && nodeSet.IsValid() && GetNodeSet(nodeSet.name) == null)
            {
                nodeSets.Add(nodeSet);
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }

        public void RemoveNodeSet(string name)
        {
            for (int i = nodeSets.Count - 1; i >= 0; i--)
            {
                if (nodeSets[i].name == name)
                {
                    nodeSets.RemoveAt(i);
#if UNITY_EDITOR
                    EditorUtility.SetDirty(this);
#endif
                    break;
                }
            }
        }

        public void ValidateNodeSets()
        {
            // Remove or update node sets with invalid indices
            for (int i = nodeSets.Count - 1; i >= 0; i--)
            {
                var nodeSet = nodeSets[i];
                var validIndices = new List<int>();
                
                foreach (int idx in nodeSet.nodeIndices)
                {
                    if (idx >= 0 && idx < nodePositions.Length)
                        validIndices.Add(idx);
                }
                
                if (validIndices.Count == 0)
                {
                    // Remove empty node set
                    nodeSets.RemoveAt(i);
                }
                else if (validIndices.Count != nodeSet.nodeIndices.Count)
                {
                    // Update node set with valid indices only
                    nodeSet.nodeIndices = validIndices;
                }
            }
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void AddFace(TrussFace face)
        {
            if (face != null && face.IsValid(nodePositions))
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

#if UNITY_EDITOR
    [CustomEditor(typeof(TrussAsset))]
    public class TrussAssetEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            TrussAsset truss = (TrussAsset)target;
            SerializedProperty nodePositionsProp = serializedObject.FindProperty("nodePositions");
            SerializedProperty beamsProp = serializedObject.FindProperty("beams");
            SerializedProperty facesProp = serializedObject.FindProperty("faces");

            EditorGUILayout.Space();

            EditorGUILayout.LabelField("Node Positions", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(nodePositionsProp, new GUIContent("Nodes"), true);

            EditorGUILayout.Space();

            EditorGUILayout.LabelField("Beams", EditorStyles.boldLabel);
            for (int i = 0; i < beamsProp.arraySize; i++)
            {
                EditorGUILayout.BeginVertical(GUI.skin.box);
                SerializedProperty beamProp = beamsProp.GetArrayElementAtIndex(i);
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("nodeA"), new GUIContent("Node A"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("nodeB"), new GUIContent("Node B"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("compliance"), new GUIContent("Compliance"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("damping"), new GUIContent("Damping"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("restLength"), new GUIContent("Rest Length"));
                if (GUILayout.Button("Remove Beam"))
                {
                    beamsProp.DeleteArrayElementAtIndex(i);
                    serializedObject.ApplyModifiedProperties();
                    return;
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();

            if (GUILayout.Button("Add New Beam", GUILayout.Height(30)))
            {
                beamsProp.arraySize++;
                SerializedProperty newBeam = beamsProp.GetArrayElementAtIndex(beamsProp.arraySize - 1);
                newBeam.FindPropertyRelative("nodeA").intValue = 0;
                newBeam.FindPropertyRelative("nodeB").intValue = 1;
                newBeam.FindPropertyRelative("compliance").floatValue = 1e-3f;
                newBeam.FindPropertyRelative("damping").floatValue = 0.3f;
                newBeam.FindPropertyRelative("restLength").floatValue = 0f;
                serializedObject.ApplyModifiedProperties();
            }

            EditorGUILayout.Space();

            EditorGUILayout.LabelField("Faces", EditorStyles.boldLabel);
            for (int i = 0; i < facesProp.arraySize; i++)
            {
                EditorGUILayout.BeginVertical(GUI.skin.box);
                SerializedProperty faceProp = facesProp.GetArrayElementAtIndex(i);
                EditorGUILayout.PropertyField(faceProp.FindPropertyRelative("nodeA"), new GUIContent("Node A"));
                EditorGUILayout.PropertyField(faceProp.FindPropertyRelative("nodeB"), new GUIContent("Node B"));
                EditorGUILayout.PropertyField(faceProp.FindPropertyRelative("nodeC"), new GUIContent("Node C"));
                if (GUILayout.Button("Remove Face"))
                {
                    facesProp.DeleteArrayElementAtIndex(i);
                    serializedObject.ApplyModifiedProperties();
                    return;
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();

            if (GUILayout.Button("Add New Face", GUILayout.Height(30)))
            {
                facesProp.arraySize++;
                SerializedProperty newFace = facesProp.GetArrayElementAtIndex(facesProp.arraySize - 1);
                newFace.FindPropertyRelative("nodeA").intValue = 0;
                newFace.FindPropertyRelative("nodeB").intValue = 1;
                newFace.FindPropertyRelative("nodeC").intValue = 2;
                serializedObject.ApplyModifiedProperties();
            }

            EditorGUILayout.Space();

            EditorGUILayout.LabelField("Assign to SoftBody", EditorStyles.boldLabel);
            SoftBody targetSoftBody = (SoftBody)EditorGUILayout.ObjectField(
                "SoftBody",
                null,
                typeof(SoftBody),
                true);
            if (targetSoftBody != null && GUILayout.Button("Apply to SoftBody", GUILayout.Height(30)))
            {
                targetSoftBody.ApplyTrussAsset(truss);
                EditorUtility.SetDirty(targetSoftBody);
                Debug.Log("Applied TrussAsset to SoftBody: " + targetSoftBody.name, targetSoftBody);
            }

            serializedObject.ApplyModifiedProperties();
        }
    }
#endif
}
