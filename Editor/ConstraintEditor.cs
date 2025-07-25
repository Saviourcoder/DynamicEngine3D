/* DynamicEngine3D - Soft Body Constraint Editor
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/

using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

namespace DynamicEngine
{
    public enum BaseBodyValidationStatus
    {
        Valid,
        Warning,
        Error,
        NotAssigned
    }

    public struct BaseBodyValidationResult
    {
        public BaseBodyValidationStatus status;
        public string message;
    }

    [CustomEditor(typeof(Constraint))]
    public class ConstraintEditor : Editor
    {
        private Constraint constraint;
        private bool showSnapsSection = true;
        private bool showMotorSection = true;
        
        private SerializedProperty baseBody;
        private SerializedProperty disableCollision;
        private SerializedProperty showSnaps;
        private SerializedProperty snaps;
        private SerializedProperty enableMotor;
        private SerializedProperty axisNodeSet;
        private SerializedProperty targetRate;
        private SerializedProperty maxTorque;

        private const float SECTION_SPACING = 10f;
        private const float BUTTON_HEIGHT = 25f;

        void OnEnable()
        {
            constraint = (Constraint)target;
            
            if (constraint == null)
            {
                Debug.LogError("ConstraintEditor: Target is not a valid Constraint component");
                return;
            }
            
            CacheSerializedProperties();
            EnableNodeVisualizationForConstraintSetup();
        }

        void OnDisable()
        {
            // Optionally disable node visualization when constraint is deselected
            // DisableNodeVisualizationAfterConstraintSetup();
        }

        private void CacheSerializedProperties()
        {
            baseBody = serializedObject.FindProperty("baseBody");
            disableCollision = serializedObject.FindProperty("disableCollision");
            showSnaps = serializedObject.FindProperty("showSnaps");
            snaps = serializedObject.FindProperty("snaps");
            enableMotor = serializedObject.FindProperty("enableMotor");
            axisNodeSet = serializedObject.FindProperty("axisNodeSet");
            targetRate = serializedObject.FindProperty("targetRate");
            maxTorque = serializedObject.FindProperty("maxTorque");
            
            if (snaps == null)
            {
                Debug.LogWarning("ConstraintEditor: 'snaps' property not found. Component may not be properly serialized.");
            }
        }

        public override void OnInspectorGUI()
        {
            if (constraint == null) return;
            
            serializedObject.Update();
            
            // Track if any properties changed for validation
            EditorGUI.BeginChangeCheck();
            
            DrawHeader();
            DrawBasicConfiguration();
            DrawRuntimeStatus();
            DrawQuickActions();
            DrawConstraintSnaps();
            DrawMotorConfiguration();
            
            // Apply changes and validate if anything changed
            if (EditorGUI.EndChangeCheck())
            {
                serializedObject.ApplyModifiedProperties();
                ValidateAndClampSnapValues();
            }
            else
            {
                serializedObject.ApplyModifiedProperties();
            }
        }
        
        private void ValidateAndClampSnapValues()
        {
            // Validate all snap constraints
            if (snaps != null)
            {
                bool needsUpdate = false;
                
                for (int i = 0; i < snaps.arraySize; i++)
                {
                    var snap = snaps.GetArrayElementAtIndex(i);
                    var minLimitProp = snap?.FindPropertyRelative("minLimit");
                    var maxLimitProp = snap?.FindPropertyRelative("maxLimit");
                    var strengthProp = snap?.FindPropertyRelative("strength");
                    
                    if (minLimitProp != null && maxLimitProp != null)
                    {
                        float minValue = minLimitProp.floatValue;
                        float maxValue = maxLimitProp.floatValue;
                        
                        // Clamp and validate
                        float clampedMin = Mathf.Max(0f, minValue);
                        float clampedMax = Mathf.Max(clampedMin, maxValue);
                        
                        if (Mathf.Abs(clampedMin - minValue) > 0.001f || Mathf.Abs(clampedMax - maxValue) > 0.001f)
                        {
                            minLimitProp.floatValue = clampedMin;
                            maxLimitProp.floatValue = clampedMax;
                            needsUpdate = true;
                        }
                    }
                    
                    // Validate strength
                    if (strengthProp != null && !float.IsPositiveInfinity(strengthProp.floatValue))
                    {
                        float strengthValue = strengthProp.floatValue;
                        float clampedStrength = Mathf.Max(0f, strengthValue);
                        
                        if (Mathf.Abs(clampedStrength - strengthValue) > 0.001f)
                        {
                            strengthProp.floatValue = clampedStrength;
                            needsUpdate = true;
                        }
                    }
                }
                
                if (needsUpdate)
                {
                    serializedObject.ApplyModifiedProperties();
                    EditorUtility.SetDirty(constraint);
                }
            }
        }

        private void DrawHeader()
        {
            EditorGUILayout.LabelField("Constraint Editor", EditorStyles.largeLabel);
            DrawBaseBodyValidation();
            EditorGUILayout.Space(SECTION_SPACING);
        }

        private void DrawBasicConfiguration()
        {
            EditorGUILayout.LabelField("Basic Configuration", EditorStyles.boldLabel);
            
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(baseBody, new GUIContent("Base Body", "The target soft body or rigidbody to constrain to"));
            if (EditorGUI.EndChangeCheck())
            {
                serializedObject.ApplyModifiedProperties();
                EnableNodeVisualizationForConstraintSetup();
            }
            
            EditorGUILayout.PropertyField(disableCollision, new GUIContent("Disable Collision", "Disable collision between constrained bodies"));
            EditorGUILayout.PropertyField(showSnaps, new GUIContent("Show Gizmos", "Show constraint visualization in scene view"));
            
            EditorGUILayout.Space(SECTION_SPACING);
        }

        private void DrawRuntimeStatus()
        {
            if (!Application.isPlaying) return;
            
            EditorGUILayout.LabelField("Runtime Status", EditorStyles.boldLabel);
            
            EditorGUI.BeginDisabledGroup(true);
            EditorGUILayout.Toggle("Is Broken", constraint.isBroken);
            if (constraint.AttachedBody != null)
            {
                EditorGUILayout.ObjectField("Attached Body", constraint.AttachedBody, typeof(SoftBody), true);
            }
            EditorGUI.EndDisabledGroup();
            
            EditorGUILayout.Space(SECTION_SPACING);
        }

        private void DrawQuickActions()
        {
            EditorGUILayout.LabelField("Quick Actions", EditorStyles.boldLabel);
            
            if (constraint.BaseBody != null && constraint.AttachedBody != null)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("📍 Node-to-Node", GUILayout.Height(BUTTON_HEIGHT)))
                {
                    CreateNodeToNodeSnap();
                }
                if (GUILayout.Button("⚓ Anchor Point", GUILayout.Height(BUTTON_HEIGHT)))
                {
                    CreatePointSnap();
                }
                GUILayout.EndHorizontal();
                
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("🔗 Edge Constraint", GUILayout.Height(BUTTON_HEIGHT)))
                {
                    CreateEdgeSnap();
                }
                if (GUILayout.Button("🎛️ Refresh Nodes", GUILayout.Height(BUTTON_HEIGHT)))
                {
                    EnableNodeVisualizationForConstraintSetup();
                }
                GUILayout.EndHorizontal();
                
                EditorGUILayout.HelpBox("💡 Nodes are visible in Scene View for easy selection!", MessageType.Info);
            }
            else
            {
                EditorGUILayout.HelpBox("Assign a Base Body to enable quick actions.", MessageType.Warning);
            }
            
            EditorGUILayout.Space(SECTION_SPACING);
        }

        private void DrawConstraintSnaps()
        {
            showSnapsSection = EditorGUILayout.Foldout(showSnapsSection, "Constraint Snaps", true, EditorStyles.foldoutHeader);
            if (!showSnapsSection) return;
            
            if (snaps == null)
            {
                EditorGUILayout.HelpBox("Snaps property not found. Component may not be properly serialized.", MessageType.Error);
                return;
            }

            EditorGUILayout.Space(5);
            
            // Draw existing snaps
            for (int i = snaps.arraySize - 1; i >= 0; i--)
            {
                var snapProp = snaps.GetArrayElementAtIndex(i);
                if (snapProp == null) continue;
                
                Rect rect = EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                
                // Header with delete button
                GUILayout.BeginHorizontal();
                EditorGUILayout.LabelField($"Snap {i}", EditorStyles.boldLabel);
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("✕", GUILayout.Width(20), GUILayout.Height(20)))
                {
                    snaps.DeleteArrayElementAtIndex(i);
                    EditorGUILayout.EndVertical();
                    GUILayout.EndHorizontal();
                    continue;
                }
                GUILayout.EndHorizontal();
                
                EditorGUILayout.Space(3);
                DrawSnapProperties(snapProp);
                
                EditorGUILayout.EndVertical();
                EditorGUILayout.Space(5);
            }
            
            // Add new snap button
            if (GUILayout.Button("+ Add New Snap", GUILayout.Height(30)))
            {
                AddNewSnap(SnapType.Node);
            }
            
            EditorGUILayout.Space(SECTION_SPACING);
        }

        private void DrawSnapProperties(SerializedProperty snapProp)
        {
            if (snapProp == null)
            {
                EditorGUILayout.HelpBox("Invalid snap element", MessageType.Error);
                return;
            }

            var typeProp = snapProp.FindPropertyRelative("type");
            var nodeProp = snapProp.FindPropertyRelative("node");
            var targetNodeProp = snapProp.FindPropertyRelative("targetNode");
            var minLimitProp = snapProp.FindPropertyRelative("minLimit");
            var maxLimitProp = snapProp.FindPropertyRelative("maxLimit");
            var strengthProp = snapProp.FindPropertyRelative("strength");
            var masterProp = snapProp.FindPropertyRelative("master");
            var showProp = snapProp.FindPropertyRelative("show");
            var targetPositionProp = snapProp.FindPropertyRelative("targetPosition");

            // Snap type
            EditorGUILayout.PropertyField(typeProp, new GUIContent("Type"));
            SnapType snapType = (SnapType)typeProp.enumValueIndex;

            // Node selection
            DrawNodeField(nodeProp, "Node", GetAttachedBodyNodes());

            // Feature B
            if (snapType == SnapType.Node && targetNodeProp != null)
                DrawNodeField(targetNodeProp, "Target Node", GetBaseBodyNodes());
            else if (snapType == SnapType.Edge && targetNodeProp != null)
                DrawEdgeField(targetNodeProp);
            else if (snapType == SnapType.Point && targetPositionProp != null)
                EditorGUILayout.PropertyField(targetPositionProp, new GUIContent("Target Position"));

            if (minLimitProp != null && maxLimitProp != null)
            {
                EditorGUI.BeginChangeCheck();
                float minValue = EditorGUILayout.FloatField("Min Limit", minLimitProp.floatValue);
                float maxValue = EditorGUILayout.FloatField("Max Limit", maxLimitProp.floatValue);

                if (EditorGUI.EndChangeCheck())
                {
                    minValue = Mathf.Max(0f, minValue);
                    maxValue = Mathf.Max(minValue, maxValue);

                    minLimitProp.floatValue = minValue;
                    maxLimitProp.floatValue = maxValue;
                }

                if (minLimitProp.floatValue < 0f || maxLimitProp.floatValue < minLimitProp.floatValue)
                {
                    EditorGUILayout.HelpBox("Min limit must be >= 0, Max limit must be >= Min limit", MessageType.Warning);
                }
            }
            else
            {
                EditorGUILayout.HelpBox("Limit properties not found", MessageType.Warning);
            }

            // Strength field (no space between limits and strength)
            if (strengthProp != null)
            {
                EditorGUI.BeginChangeCheck();
                float strengthValue = strengthProp.floatValue;

                if (float.IsPositiveInfinity(strengthValue))
                {
                    bool isInfinite = EditorGUILayout.Toggle("Infinite Strength", true);
                    if (!isInfinite)
                        strengthProp.floatValue = 1000f;
                }
                else
                {
                    bool isInfinite = EditorGUILayout.Toggle("Infinite Strength", false);
                    if (isInfinite)
                        strengthProp.floatValue = float.PositiveInfinity;
                    else
                    {
                        float newStrength = EditorGUILayout.FloatField("Strength", strengthValue);
                        if (newStrength != strengthValue)
                            strengthProp.floatValue = Mathf.Max(0f, newStrength);
                    }
                }
            }

            EditorGUILayout.Space(3);

            // Options (VERTICAL FIX)
            EditorGUILayout.LabelField("Options", EditorStyles.boldLabel);
            if (masterProp != null)
                masterProp.boolValue = EditorGUILayout.Toggle("Master", masterProp.boolValue);
            if (showProp != null)
                showProp.boolValue = EditorGUILayout.Toggle("Show Gizmo", showProp.boolValue);
        }


        private void DrawNodeField(SerializedProperty nodeProp, string label, string[] availableNodes)
        {
            if (nodeProp == null)
            {
                EditorGUILayout.LabelField(label, "Property not found");
                return;
            }

            if (availableNodes == null || availableNodes.Length == 0)
            {
                EditorGUILayout.LabelField(label, "No nodes available");
                return;
            }
            
            string currentValue = nodeProp.stringValue ?? "";
            int selectedIndex = System.Array.IndexOf(availableNodes, currentValue);
            if (selectedIndex == -1) selectedIndex = 0;
            
            EditorGUI.BeginChangeCheck();
            selectedIndex = EditorGUILayout.Popup(label, selectedIndex, availableNodes);
            if (EditorGUI.EndChangeCheck())
            {
                nodeProp.stringValue = availableNodes[selectedIndex];
            }
        }

        private void DrawEdgeField(SerializedProperty targetNodeProp)
        {
            if (targetNodeProp == null)
            {
                EditorGUILayout.LabelField("Feature property not found");
                return;
            }

            string[] baseNodes = GetBaseBodyNodes();
            if (baseNodes == null || baseNodes.Length < 2)
            {
                EditorGUILayout.LabelField("Need at least 2 nodes for edge selection");
                return;
            }
            
            EditorGUILayout.LabelField("Target Edge (Base Body)");
            
            // Parse current edge string
            string currentValue = targetNodeProp.stringValue ?? "";
            int node1Index = 0, node2Index = 1;
            
            if (!string.IsNullOrEmpty(currentValue))
            {
                string nodesPart = currentValue.Replace("nodes", "");
                string[] parts = nodesPart.Split(',');
                if (parts.Length >= 2)
                {
                    if (int.TryParse(parts[0], out int n1)) node1Index = Mathf.Clamp(n1, 0, baseNodes.Length - 1);
                    if (int.TryParse(parts[1], out int n2)) node2Index = Mathf.Clamp(n2, 0, baseNodes.Length - 1);
                }
            }
            
            GUILayout.BeginHorizontal();
            EditorGUI.BeginChangeCheck();
            node1Index = EditorGUILayout.Popup("Node 1", node1Index, baseNodes);
            node2Index = EditorGUILayout.Popup("Node 2", node2Index, baseNodes);
            GUILayout.EndHorizontal();
            
            if (EditorGUI.EndChangeCheck())
            {
                targetNodeProp.stringValue = $"nodes{node1Index},{node2Index}";
            }
            
            EditorGUILayout.LabelField("Edge String", targetNodeProp.stringValue ?? "", EditorStyles.miniLabel);
        }

        private void DrawMotorConfiguration()
        {
            showMotorSection = EditorGUILayout.Foldout(showMotorSection, "Motor Configuration", true, EditorStyles.foldoutHeader);
            if (!showMotorSection) return;

            if (enableMotor == null)
            {
                EditorGUILayout.HelpBox("Motor properties not found.", MessageType.Error);
                return;
            }

            EditorGUILayout.Space(5);
            EditorGUILayout.PropertyField(enableMotor, new GUIContent("Enable Motor", "Enable motor functionality"));
            
            if (enableMotor.boolValue)
            {
                EditorGUILayout.Space(5);
                
                if (axisNodeSet != null)
                    EditorGUILayout.PropertyField(axisNodeSet, new GUIContent("Axis Node Set", "Node set defining the motor axis (requires 2 nodes)"));
                
                if (targetRate != null)
                    EditorGUILayout.PropertyField(targetRate, new GUIContent("Target Rate", "Target rotation rate in radians per second"));
                
                if (maxTorque != null)
                    EditorGUILayout.PropertyField(maxTorque, new GUIContent("Max Torque", "Maximum torque the motor can apply"));
                
                // Runtime motor controls
                if (Application.isPlaying && constraint.EnableMotor)
                {
                    EditorGUILayout.Space(5);
                    EditorGUILayout.LabelField("Runtime Control", EditorStyles.boldLabel);
                    
                    float newRate = EditorGUILayout.FloatField("Motor Rate", constraint.motorRate);
                    if (newRate != constraint.motorRate)
                    {
                        constraint.motorRate = newRate;
                    }
                    
                    float newTorque = EditorGUILayout.FloatField("Motor Torque", constraint.motorTorque);
                    if (newTorque != constraint.motorTorque)
                    {
                        constraint.motorTorque = newTorque;
                    }
                }
            }
            
            EditorGUILayout.Space(SECTION_SPACING);
        }

        // Helper methods for node data and validation
        private string[] GetAttachedBodyNodes()
        {
            var attachedSoftBody = constraint?.GetComponent<SoftBody>();
            return GetNodesFromSoftBody(attachedSoftBody);
        }

        private string[] GetBaseBodyNodes()
        {
            if (constraint?.BaseBody is SoftBody baseSoftBody)
            {
                return GetNodesFromSoftBody(baseSoftBody);
            }
            return new string[0];
        }

        private string[] GetNodesFromSoftBody(SoftBody softBody)
        {
            if (softBody == null) return new string[0];
            
            // Try runtime solver first
            if (Application.isPlaying && softBody.solver?.nodeManager?.Nodes != null)
            {
                var runtimeNodes = softBody.solver.nodeManager.Nodes;
                string[] nodeNames = new string[runtimeNodes.Count];
                for (int i = 0; i < runtimeNodes.Count; i++)
                {
                    nodeNames[i] = $"node{i}";
                }
                return nodeNames;
            }
            
            // Fallback to truss asset
            var trussAsset = softBody.GetTrussAsset();
            if (trussAsset != null)
            {
                return GetNodesFromTrussAsset(trussAsset);
            }
            
            return GenerateDefaultNodeNames(8);
        }

        private string[] GetNodesFromTrussAsset(object trussAsset)
        {
            if (trussAsset == null) return GenerateDefaultNodeNames(4);
            
            var trussType = trussAsset.GetType();
            
            // Try node count properties
            string[] nodeCountProps = { "nodeCount", "NodeCount", "numNodes", "NumNodes", "vertices", "Vertices" };
            foreach (string propName in nodeCountProps)
            {
                var prop = trussType.GetProperty(propName);
                if (prop?.PropertyType == typeof(int))
                {
                    int nodeCount = (int)prop.GetValue(trussAsset);
                    if (nodeCount > 0) return GenerateDefaultNodeNames(nodeCount);
                }
                
                var field = trussType.GetField(propName);
                if (field?.FieldType == typeof(int))
                {
                    int nodeCount = (int)field.GetValue(trussAsset);
                    if (nodeCount > 0) return GenerateDefaultNodeNames(nodeCount);
                }
            }
            
            // Try node arrays
            string[] nodeArrayProps = { "nodes", "Nodes", "vertices", "Vertices", "points", "Points" };
            foreach (string propName in nodeArrayProps)
            {
                var prop = trussType.GetProperty(propName);
                if (prop != null)
                {
                    var value = prop.GetValue(trussAsset);
                    if (value is System.Collections.ICollection collection)
                    {
                        return GenerateDefaultNodeNames(collection.Count);
                    }
                }
                
                var field = trussType.GetField(propName);
                if (field != null)
                {
                    var value = field.GetValue(trussAsset);
                    if (value is System.Collections.ICollection collection)
                    {
                        return GenerateDefaultNodeNames(collection.Count);
                    }
                }
            }
            
            return GenerateDefaultNodeNames(8);
        }

        private string[] GenerateDefaultNodeNames(int count)
        {
            string[] nodeNames = new string[count];
            for (int i = 0; i < count; i++)
            {
                nodeNames[i] = $"node{i}";
            }
            return nodeNames;
        }

        private void DrawBaseBodyValidation()
        {
            var result = ValidateBaseBody();
            
            MessageType messageType = result.status switch
            {
                BaseBodyValidationStatus.Valid => MessageType.Info,
                BaseBodyValidationStatus.Warning => MessageType.Warning,
                BaseBodyValidationStatus.Error => MessageType.Error,
                _ => MessageType.Info
            };

            string icon = result.status switch
            {
                BaseBodyValidationStatus.Valid => "✅",
                BaseBodyValidationStatus.Warning => "⚠️", 
                BaseBodyValidationStatus.Error => "❌",
                _ => "📝"
            };

            EditorGUILayout.HelpBox($"{icon} {result.message}", messageType);
            
            if (result.status == BaseBodyValidationStatus.Valid && 
                constraint.BaseBody != null && constraint.AttachedBody != null)
            {
                EditorGUILayout.HelpBox($"🎯 Nodes visible in Scene View for easy constraint setup!", MessageType.Info);
            }
        }

        private BaseBodyValidationResult ValidateBaseBody()
        {
            var baseBody = constraint?.BaseBody;
            
            if (baseBody == null)
            {
                return new BaseBodyValidationResult
                {
                    status = BaseBodyValidationStatus.NotAssigned,
                    message = "Assign a Base Body to enable constraint setup"
                };
            }
            
            if (baseBody.gameObject == constraint.gameObject)
            {
                return new BaseBodyValidationResult
                {
                    status = BaseBodyValidationStatus.Error,
                    message = "Cannot constrain to itself! Choose a different object."
                };
            }
            
            if (!(baseBody is SoftBody baseSoftBody))
            {
                return new BaseBodyValidationResult
                {
                    status = BaseBodyValidationStatus.Warning,
                    message = "Base Body is not a SoftBody. Some constraints require SoftBody targets."
                };
            }
            
            if (Application.isPlaying)
            {
                if (baseSoftBody.GetTrussAsset() == null)
                {
                    return new BaseBodyValidationResult
                    {
                        status = BaseBodyValidationStatus.Warning,
                        message = "Base Body has no truss asset assigned."
                    };
                }
                
                if (baseSoftBody.solver?.nodeManager?.Nodes == null)
                {
                    return new BaseBodyValidationResult
                    {
                        status = BaseBodyValidationStatus.Warning,
                        message = "Base Body solver not initialized."
                    };
                }
                
                int nodeCount = baseSoftBody.solver.nodeManager.Nodes.Count;
                if (nodeCount == 0)
                {
                    return new BaseBodyValidationResult
                    {
                        status = BaseBodyValidationStatus.Warning,
                        message = "Base Body has no nodes available."
                    };
                }
                
                return new BaseBodyValidationResult
                {
                    status = BaseBodyValidationStatus.Valid,
                    message = $"Ready! Base Body '{baseBody.name}' has {nodeCount} nodes available."
                };
            }
            
            return new BaseBodyValidationResult
            {
                status = BaseBodyValidationStatus.Valid,
                message = $"Ready! Base Body '{baseBody.name}' configured. Node count available in play mode."
            };
        }

        // Quick snap creation methods
        private void AddNewSnap(SnapType snapType)
        {
            if (snaps == null) return;
            
            snaps.arraySize++;
            var newSnap = snaps.GetArrayElementAtIndex(snaps.arraySize - 1);
            
            SetSnapDefaults(newSnap, snapType);
            serializedObject.ApplyModifiedProperties();
        }

        private void SetSnapDefaults(SerializedProperty snapProp, SnapType snapType)
        {
            if (snapProp == null) return;
            
            var typeProp = snapProp.FindPropertyRelative("type");
            var nodeProp = snapProp.FindPropertyRelative("node");
            var targetNodeProp = snapProp.FindPropertyRelative("targetNode");
            var minLimitProp = snapProp.FindPropertyRelative("minLimit");
            var maxLimitProp = snapProp.FindPropertyRelative("maxLimit");
            var strengthProp = snapProp.FindPropertyRelative("strength");
            var masterProp = snapProp.FindPropertyRelative("master");
            var showProp = snapProp.FindPropertyRelative("show");
            var targetPositionProp = snapProp.FindPropertyRelative("targetPosition");
            
            // Set type
            if (typeProp != null)
                typeProp.enumValueIndex = (int)snapType;
            
            // Set node defaults
            if (nodeProp != null)
                nodeProp.stringValue = "node0";
            
            // Set validated limits
            if (minLimitProp != null)
                minLimitProp.floatValue = 0f;
            
            if (maxLimitProp != null)
            {
                float defaultMax = snapType == SnapType.Point ? 2f : 0.5f;
                maxLimitProp.floatValue = defaultMax;
            }
            
            // Set strength
            if (strengthProp != null)
            {
                float defaultStrength = snapType == SnapType.Point ? 5000f : float.PositiveInfinity;
                strengthProp.floatValue = defaultStrength;
            }
            
            // Set flags
            if (masterProp != null)
                masterProp.boolValue = true;
            
            if (showProp != null)
                showProp.boolValue = true;
            
            // Set target based on snap type
            if (snapType == SnapType.Point)
            {
                if (targetNodeProp != null)
                    targetNodeProp.stringValue = "";
                    
                if (targetPositionProp != null)
                    targetPositionProp.vector3Value = constraint.transform.position;
            }
            else
            {
                if (targetNodeProp != null)
                {
                    string defaultTarget = snapType == SnapType.Edge ? "nodes0,1" : "node0";
                    targetNodeProp.stringValue = defaultTarget;
                }
                
                if (targetPositionProp != null)
                    targetPositionProp.vector3Value = Vector3.zero;
            }
            
            // Force update
            snapProp.serializedObject.ApplyModifiedProperties();
        }

        private void CreateNodeToNodeSnap()
        {
            AddNewSnap(SnapType.Node);
            Debug.Log("Node-to-Node snap created!");
        }

        private void CreatePointSnap()
        {
            AddNewSnap(SnapType.Point);
            Debug.Log("Point snap created!");
        }

        private void CreateEdgeSnap()
        {
            AddNewSnap(SnapType.Edge);
            Debug.Log("Edge snap created!");
        }

        // Node visualization methods
        private void EnableNodeVisualizationForConstraintSetup()
        {
            var attachedSoftBody = constraint?.GetComponent<SoftBody>();
            if (attachedSoftBody != null)
            {
                EnableSoftBodyNodeVisualization(attachedSoftBody, true);
            }

            if (constraint?.BaseBody is SoftBody baseSoftBody)
            {
                EnableSoftBodyNodeVisualization(baseSoftBody, true);
            }

            SceneView.RepaintAll();
        }

        private void EnableSoftBodyNodeVisualization(SoftBody softBody, bool enable)
        {
            if (softBody == null) return;

            var softBodySO = new SerializedObject(softBody);
            var showNodesProperty = softBodySO.FindProperty("showNodes");
            var showNodeIndicesProperty = softBodySO.FindProperty("showNodeIndices");
            
            if (showNodesProperty != null) showNodesProperty.boolValue = enable;
            if (showNodeIndicesProperty != null) showNodeIndicesProperty.boolValue = enable;
            
            softBodySO.ApplyModifiedProperties();
            EditorUtility.SetDirty(softBody);
        }

        // Scene GUI for handles
        void OnSceneGUI()
        {
            if (!constraint?.ShowSnaps == true) return;

            Handles.color = Color.cyan;
            
            foreach (var snap in constraint.SnapsArray)
            {
                if (!snap.show) continue;
                DrawSnapHandle(snap);
            }
        }

        private void DrawSnapHandle(Snap snap)
        {
            if (snap.type == SnapType.Point)
            {
                EditorGUI.BeginChangeCheck();
                Vector3 newPos = Handles.PositionHandle(snap.targetPosition, Quaternion.identity);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(constraint, "Move Point Snap");
                    snap.targetPosition = newPos;
                    EditorUtility.SetDirty(constraint);
                }
                
                Handles.Label(snap.targetPosition + Vector3.up * 0.2f, $"Point: {snap.node}");
            }
        }
    }

    // Extension methods for cleaner property setting
    public static class SerializedPropertyExtensions
    {
        public static void SetValue(this SerializedProperty prop, object value)
        {
            if (prop == null) return;
            
            switch (value)
            {
                case bool boolValue:
                    prop.boolValue = boolValue;
                    break;
                case int intValue:
                    prop.intValue = intValue;
                    break;
                case float floatValue:
                    prop.floatValue = floatValue;
                    break;
                case string stringValue:
                    prop.stringValue = stringValue;
                    break;
                case Vector3 vector3Value:
                    prop.vector3Value = vector3Value;
                    break;
            }
        }
        
        public static void SetEnumValue<T>(this SerializedProperty prop, T enumValue) where T : System.Enum
        {
            if (prop != null)
            {
                prop.enumValueIndex = System.Convert.ToInt32(enumValue);
            }
        }
    }
}