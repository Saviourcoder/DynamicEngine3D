/* DynamicEngine3D - Constraint Data Asset
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/

using System.Collections.Generic;
using System.Linq;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    [CreateAssetMenu(fileName = "ConstraintData", menuName = "SoftBody/Constraint Data", order = 2)]
    public class ConstraintData : ScriptableObject
    {
        [Header("Constraint Configuration")]
        [SerializeField, Tooltip("Description of this constraint setup")]
        private string description = "";
        
        [SerializeField, Tooltip("Disable collision between constrained bodies")]
        private bool disableCollision = true;
        
        [Header("Constraint Snaps")]
        [SerializeField, Tooltip("Array of constraint snaps")]
        private Snap[] snaps = new Snap[0];
        
        [Header("Motor Configuration")]
        [SerializeField, Tooltip("Enable motor functionality")]
        private bool enableMotor = false;
        
        [SerializeField, Tooltip("Node set that defines the motor axis")]
        private string axisNodeSet = "";
        
        [SerializeField, Tooltip("Target rotation rate for the motor")]
        private float targetRate = 0f;
        
        [SerializeField, Tooltip("Maximum torque the motor can apply")]
        private float maxTorque = 0f;

        // Public properties
        public string Description => description;
        public bool DisableCollision => disableCollision;
        public Snap[] Snaps => snaps;
        public bool EnableMotor => enableMotor;
        public string AxisNodeSet => axisNodeSet;
        public float TargetRate => targetRate;
        public float MaxTorque => maxTorque;

        public void SetSnaps(Snap[] newSnaps)
        {
            if (newSnaps == null)
                snaps = new Snap[0];
            else
                snaps = newSnaps;
                
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void AddSnap(Snap snap)
        {
            if (snap == null || !snap.IsValid()) return;
            
            var snapList = new List<Snap>(snaps) { snap };
            snaps = snapList.ToArray();
            
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void RemoveSnap(int index)
        {
            if (index < 0 || index >= snaps.Length) return;
            
            var snapList = new List<Snap>(snaps);
            snapList.RemoveAt(index);
            snaps = snapList.ToArray();
            
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetMotorConfiguration(bool enable, string nodeSet, float rate, float torque)
        {
            enableMotor = enable;
            axisNodeSet = nodeSet ?? "";
            targetRate = rate;
            maxTorque = torque;
            
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetConstraintSettings(string desc, bool disableCol)
        {
            description = desc ?? "";
            disableCollision = disableCol;
            
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        private void OnValidate()
        {
            // Validate snap limits
            if (snaps != null)
            {
                foreach (var snap in snaps)
                {
                    if (snap != null)
                    {
                        snap.minLimit = Mathf.Max(0f, snap.minLimit);
                        snap.maxLimit = Mathf.Max(snap.minLimit, snap.maxLimit);
                    }
                }
            }
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(ConstraintData))]
    public class ConstraintDataEditor : Editor
    {
        private bool showSnapsSection = true;
        private bool showMotorSection = true;

        public override void OnInspectorGUI()
        {
            ConstraintData constraintData = (ConstraintData)target;
            
            serializedObject.Update();

            // Description and basic settings
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Constraint Configuration", EditorStyles.boldLabel);
            
            var descriptionProp = serializedObject.FindProperty("description");
            EditorGUILayout.PropertyField(descriptionProp);
            
            var disableCollisionProp = serializedObject.FindProperty("disableCollision");
            EditorGUILayout.PropertyField(disableCollisionProp);

            EditorGUILayout.Space();

            // Constraint Snaps Section
            showSnapsSection = EditorGUILayout.Foldout(showSnapsSection, "Constraint Snaps", true, EditorStyles.foldoutHeader);
            
            if (showSnapsSection)
            {
                EditorGUI.indentLevel++;
                DrawConstraintSnaps();
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();

            // Motor Configuration Section
            showMotorSection = EditorGUILayout.Foldout(showMotorSection, "Motor Configuration", true, EditorStyles.foldoutHeader);
            
            if (showMotorSection)
            {
                EditorGUI.indentLevel++;
                
                var enableMotorProp = serializedObject.FindProperty("enableMotor");
                EditorGUILayout.PropertyField(enableMotorProp);
                
                if (enableMotorProp.boolValue)
                {
                    var axisNodeSetProp = serializedObject.FindProperty("axisNodeSet");
                    var targetRateProp = serializedObject.FindProperty("targetRate");
                    var maxTorqueProp = serializedObject.FindProperty("maxTorque");
                    
                    DrawNodeSetDropdown(axisNodeSetProp, "Axis Node Set", "Node set that defines the motor axis");
                    EditorGUILayout.PropertyField(targetRateProp);
                    EditorGUILayout.PropertyField(maxTorqueProp);
                }
                
                EditorGUI.indentLevel--;
            }

            serializedObject.ApplyModifiedProperties();
        }

        private void DrawConstraintSnaps()
        {
            var snapsProp = serializedObject.FindProperty("snaps");
            
            EditorGUILayout.LabelField($"Total Snaps: {snapsProp.arraySize}", EditorStyles.miniLabel);
            
            for (int i = 0; i < snapsProp.arraySize; i++)
            {
                DrawSnapElement(snapsProp, i);
            }

            EditorGUILayout.Space(5f);
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Add Snap", GUILayout.Height(25)))
            {
                snapsProp.arraySize++;
                var newSnap = snapsProp.GetArrayElementAtIndex(snapsProp.arraySize - 1);
                InitializeNewSnap(newSnap);
                serializedObject.ApplyModifiedProperties();
            }
            EditorGUILayout.EndHorizontal();
        }

        private void DrawSnapElement(SerializedProperty snapsProp, int index)
        {
            var snapProp = snapsProp.GetArrayElementAtIndex(index);
            
            EditorGUILayout.BeginVertical(GUI.skin.box);
            
            // Header with fold-out
            EditorGUILayout.BeginHorizontal();
            var showProp = snapProp.FindPropertyRelative("show");
            showProp.boolValue = EditorGUILayout.Foldout(showProp.boolValue, $"Snap {index}", true);
            
            if (GUILayout.Button("×", GUILayout.Width(20), GUILayout.Height(16)))
            {
                snapsProp.DeleteArrayElementAtIndex(index);
                serializedObject.ApplyModifiedProperties();
                EditorGUILayout.EndHorizontal();
                EditorGUILayout.EndVertical();
                return;
            }
            EditorGUILayout.EndHorizontal();
            
            if (showProp.boolValue)
            {
                EditorGUI.indentLevel++;
                
                // Snap type
                var typeProp = snapProp.FindPropertyRelative("type");
                EditorGUILayout.PropertyField(typeProp);
                
                // Node set with dropdown
                var nodeProp = snapProp.FindPropertyRelative("node");
                DrawNodeSetDropdown(nodeProp, "Node Set", "Source soft body node sets");
                
                // Target based on type
                SnapType snapType = (SnapType)typeProp.enumValueIndex;
                switch (snapType)
                {
                    case SnapType.Point:
                        var targetPosProp = snapProp.FindPropertyRelative("targetPosition");
                        EditorGUILayout.PropertyField(targetPosProp, new GUIContent("Target Position"));
                        break;
                        
                    case SnapType.Node:
                    case SnapType.Edge:
                        var targetNodeProp = snapProp.FindPropertyRelative("targetNode");
                        DrawNodeSetDropdown(targetNodeProp, "Target Node Set", "Target soft body node sets");
                        break;
                }
                
                // Constraint properties
                var strengthProp = snapProp.FindPropertyRelative("strength");
                var minLimitProp = snapProp.FindPropertyRelative("minLimit");
                var maxLimitProp = snapProp.FindPropertyRelative("maxLimit");
                var masterProp = snapProp.FindPropertyRelative("master");
                
                EditorGUILayout.PropertyField(strengthProp);
                EditorGUILayout.PropertyField(minLimitProp);
                EditorGUILayout.PropertyField(maxLimitProp);
                EditorGUILayout.PropertyField(masterProp);
                
                EditorGUI.indentLevel--;
            }
            
            EditorGUILayout.EndVertical();
        }

        private void InitializeNewSnap(SerializedProperty snapProp)
        {
            snapProp.FindPropertyRelative("type").enumValueIndex = 0;
            snapProp.FindPropertyRelative("node").stringValue = "";
            snapProp.FindPropertyRelative("targetNode").stringValue = "";
            snapProp.FindPropertyRelative("minLimit").floatValue = 0f;
            snapProp.FindPropertyRelative("maxLimit").floatValue = 0f;
            snapProp.FindPropertyRelative("strength").floatValue = float.PositiveInfinity;
            snapProp.FindPropertyRelative("master").boolValue = true;
            snapProp.FindPropertyRelative("show").boolValue = true;
            snapProp.FindPropertyRelative("targetPosition").vector3Value = Vector3.zero;
        }

        private void DrawNodeSetDropdown(SerializedProperty nodeProp, string label, string tooltip)
        {
            if (nodeProp == null)
            {
                EditorGUILayout.LabelField(label, "Property not found");
                return;
            }

            string[] availableNodeSets = GetAvailableNodeSets();
            string currentValue = nodeProp.stringValue ?? "";

            EditorGUILayout.BeginHorizontal();
            
            // Add icon based on label type
            string icon = label.Contains("Target") ? "🎯" : "📍";
            EditorGUILayout.LabelField(new GUIContent($"{icon} {label}", tooltip), GUILayout.Width(130));

            if (availableNodeSets == null || availableNodeSets.Length == 0)
            {
                // Fallback to text field if no node sets found
                EditorGUI.BeginChangeCheck();
                string newValue = EditorGUILayout.TextField(currentValue);
                if (EditorGUI.EndChangeCheck())
                {
                    nodeProp.stringValue = newValue;
                }
                
                if (GUILayout.Button("🔍", GUILayout.Width(20)))
                {
                    ShowNodeSetHelp();
                }
            }
            else
            {
                // Enhanced dropdown with node sets - include empty option
                var dropdownOptions = new string[availableNodeSets.Length + 1];
                dropdownOptions[0] = "<None>";
                System.Array.Copy(availableNodeSets, 0, dropdownOptions, 1, availableNodeSets.Length);
                
                int selectedIndex = System.Array.IndexOf(availableNodeSets, currentValue);
                selectedIndex = selectedIndex == -1 ? 0 : selectedIndex + 1; // +1 for the <None> option

                EditorGUI.BeginChangeCheck();
                selectedIndex = EditorGUILayout.Popup(selectedIndex, dropdownOptions);
                if (EditorGUI.EndChangeCheck())
                {
                    nodeProp.stringValue = selectedIndex == 0 ? "" : availableNodeSets[selectedIndex - 1];
                }

                if (GUILayout.Button("ℹ", GUILayout.Width(20)))
                {
                    if (selectedIndex > 0)
                        ShowNodeSetInfo(availableNodeSets[selectedIndex - 1]);
                    else
                        ShowNodeSetHelp();
                }
            }
            
            EditorGUILayout.EndHorizontal();

            // Show helpful message if no node sets available
            if (availableNodeSets == null || availableNodeSets.Length == 0)
            {
                EditorGUILayout.HelpBox("💡 No node sets found in TrussAssets. Create named node sets in your TrussAssets for easier selection.", MessageType.Info);
            }
            else if (availableNodeSets.Length > 0 && string.IsNullOrEmpty(currentValue))
            {
                EditorGUILayout.LabelField($"📋 {availableNodeSets.Length} node sets available", EditorStyles.miniLabel);
            }
        }

        private string[] GetAvailableNodeSets()
        {
            var nodeSetList = new List<string>();
            
            // Find all TrussAssets in the project and scene
            TrussAsset[] allTrussAssets = FindObjectsByType<TrussAsset>(FindObjectsSortMode.None);
            
            // Also search for TrussAssets in the project assets
            string[] guids = AssetDatabase.FindAssets("t:TrussAsset");
            foreach (string guid in guids)
            {
                string assetPath = AssetDatabase.GUIDToAssetPath(guid);
                TrussAsset trussAsset = AssetDatabase.LoadAssetAtPath<TrussAsset>(assetPath);
                if (trussAsset != null)
                {
                    var nodeSets = trussAsset.NodeSets;
                    if (nodeSets != null)
                    {
                        foreach (var nodeSet in nodeSets)
                        {
                            if (!string.IsNullOrEmpty(nodeSet.name) && !nodeSetList.Contains(nodeSet.name))
                            {
                                nodeSetList.Add(nodeSet.name);
                            }
                        }
                    }
                }
            }

            // Add runtime TrussAssets from SoftBodies in scene
            SoftBody[] softBodies = FindObjectsByType<SoftBody>(FindObjectsSortMode.None);
            foreach (var softBody in softBodies)
            {
                var trussAsset = softBody.GetTrussAsset();
                if (trussAsset != null)
                {
                    var nodeSets = trussAsset.NodeSets;
                    if (nodeSets != null)
                    {
                        foreach (var nodeSet in nodeSets)
                        {
                            if (!string.IsNullOrEmpty(nodeSet.name) && !nodeSetList.Contains(nodeSet.name))
                            {
                                nodeSetList.Add(nodeSet.name);
                            }
                        }
                    }
                }
            }

            // Add common node naming patterns as fallbacks
            if (nodeSetList.Count == 0)
            {
                nodeSetList.AddRange(new string[] { "all", "chassis", "wheel", "suspension", "body" });
            }

            nodeSetList.Sort();
            return nodeSetList.ToArray();
        }

        private void ShowNodeSetInfo(string nodeSetName)
        {
            if (string.IsNullOrEmpty(nodeSetName))
                return;

            string message = $"Node Set: {nodeSetName}\n\n";
            int totalNodes = 0;
            var foundInAssets = new List<string>();

            // Search in TrussAssets for this node set
            string[] guids = AssetDatabase.FindAssets("t:TrussAsset");
            foreach (string guid in guids)
            {
                string assetPath = AssetDatabase.GUIDToAssetPath(guid);
                TrussAsset trussAsset = AssetDatabase.LoadAssetAtPath<TrussAsset>(assetPath);
                if (trussAsset != null)
                {
                    var nodeSet = trussAsset.NodeSets?.FirstOrDefault(ns => ns.name == nodeSetName);
                    if (nodeSet != null)
                    {
                        foundInAssets.Add(System.IO.Path.GetFileName(assetPath));
                        totalNodes += nodeSet.nodeIndices?.Count ?? 0;
                    }
                }
            }

            if (foundInAssets.Count > 0)
            {
                message += $"Found in {foundInAssets.Count} TrussAsset(s):\n";
                foreach (var asset in foundInAssets)
                {
                    message += $"  • {asset}\n";
                }
                message += $"\nTotal nodes in this set: {totalNodes}";
            }
            else
            {
                message += "This node set was not found in any TrussAssets.\n";
                message += "You may need to create it or check the spelling.";
            }

            EditorUtility.DisplayDialog("Node Set Information", message, "OK");
        }

        private void ShowNodeSetHelp()
        {
            string message = "Node Set Help\n\n";
            message += "Node sets are named groups of nodes defined in TrussAssets.\n\n";
            message += "To create node sets:\n";
            message += "1. Open a TrussAsset in the inspector\n";
            message += "2. Add named node sets with specific node indices\n";
            message += "3. Use these names in constraints\n\n";
            message += "Common patterns:\n";
            message += "• 'chassis' - main body nodes\n";
            message += "• 'wheel' - wheel connection points\n";
            message += "• 'suspension' - suspension attachment points";

            EditorUtility.DisplayDialog("Node Set Help", message, "OK");
        }
    }
#endif
}