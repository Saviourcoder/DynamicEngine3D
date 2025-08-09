/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */
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

        private SerializedProperty baseBody;
        private SerializedProperty constraintData;
        private SerializedProperty showSnaps;
        private SerializedProperty snaps;
        private SerializedProperty disableCollision;
        private SerializedProperty enableMotor;
        private SerializedProperty axisNodeSet;
        private SerializedProperty targetRate;
        private SerializedProperty maxTorque;

        private const float SECTION_SPACING = 10f;
        private const float BUTTON_HEIGHT = 25f;

        // State tracking fields
        private static GameObject s_lastSelectedObject;
        private static bool s_wasInPlayMode;
        private bool m_needsVisualizationRefresh;
        private bool m_needsDataRefresh;
        private bool showSnapsSection = true;
        private bool showMotorSection = true;
        private bool[] individualSnapFoldouts;

        void OnEnable()
        {
            constraint = (Constraint)target;
            CacheSerializedProperties();
            
            // Subscribe to selection change events
            Selection.selectionChanged += OnSelectionChanged;
        }

        public override void OnInspectorGUI()
        {
            if (constraint == null)
            {
                EditorGUILayout.HelpBox("Constraint component is null", MessageType.Error);
                return;
            }

            serializedObject.Update();

            DrawRequiredComponents();
            DrawConstraintDataSection();
            DrawVisualizationSection();
            DrawConstraintDataPreview();

            serializedObject.ApplyModifiedProperties();
        }

        private void DrawRequiredComponents()
        {
            EditorGUILayout.LabelField("Required Components", EditorStyles.boldLabel);

            // SoftBody validation
            var softBody = constraint.GetComponent<SoftBody>();
            if (softBody == null)
            {
                EditorGUILayout.HelpBox("Constraint requires a SoftBody component on this GameObject.", MessageType.Error);
                if (GUILayout.Button("Add SoftBody Component"))
                {
                    constraint.gameObject.AddComponent<SoftBody>();
                }
            }
            else
            {
                EditorGUILayout.HelpBox("✓ SoftBody component found", MessageType.Info);
            }

            EditorGUILayout.Space(SECTION_SPACING);
        }

        private void DrawConstraintDataSection()
        {
            EditorGUILayout.LabelField("Constraint Configuration", EditorStyles.boldLabel);

            // Base Body
            EditorGUILayout.PropertyField(baseBody, new GUIContent("Base Body", "The SoftBody to constrain against"));

            var validationResult = ValidateBaseBody();
            if (validationResult.status != BaseBodyValidationStatus.Valid)
            {
                MessageType messageType = validationResult.status == BaseBodyValidationStatus.Error ? MessageType.Error : MessageType.Warning;
                EditorGUILayout.HelpBox(validationResult.message, messageType);
            }

            EditorGUILayout.Space(5f);

            // Constraint Data Asset
            EditorGUILayout.PropertyField(constraintData, new GUIContent("Constraint Data", "ScriptableObject containing constraint snap configurations"));

            if (constraintData.objectReferenceValue == null)
            {
                EditorGUILayout.HelpBox("No Constraint Data assigned. Create or assign a ConstraintData asset to configure constraint snaps.", MessageType.Warning);

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Create New ConstraintData"))
                {
                    CreateNewConstraintData();
                }
                if (GUILayout.Button("Find Existing ConstraintData"))
                {
                    ShowConstraintDataSelector();
                }
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Edit ConstraintData"))
                {
                    Selection.activeObject = constraintData.objectReferenceValue;
                }
                if (GUILayout.Button("Ping in Project"))
                {
                    EditorGUIUtility.PingObject(constraintData.objectReferenceValue);
                }
                EditorGUILayout.EndHorizontal();
            }

            EditorGUILayout.Space(SECTION_SPACING);
        }

        private void DrawVisualizationSection()
        {
            EditorGUILayout.LabelField("Visualization", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(showSnaps, new GUIContent("Show Snaps", "Show constraint snap gizmos in Scene view"));

            if (showSnaps.boolValue && constraintData.objectReferenceValue == null)
            {
                EditorGUILayout.HelpBox("Snap visualization requires a ConstraintData asset to be assigned.", MessageType.Info);
            }

            EditorGUILayout.Space(SECTION_SPACING);
        }

        private void DrawConstraintDataPreview()
        {
            if (constraintData.objectReferenceValue == null) return;

            ConstraintData data = constraintData.objectReferenceValue as ConstraintData;
            if (data == null) return;

            EditorGUILayout.LabelField("Constraint Data Preview", EditorStyles.boldLabel);

            EditorGUI.BeginDisabledGroup(true);

            EditorGUILayout.TextField("Description", data.Description);
            EditorGUILayout.Toggle("Disable Collision", data.DisableCollision);

            if (data.Snaps != null)
            {
                EditorGUILayout.LabelField($"Constraint Snaps: {data.Snaps.Length}");

                if (data.Snaps.Length > 0)
                {
                    EditorGUI.indentLevel++;
                    for (int i = 0; i < Mathf.Min(data.Snaps.Length, 3); i++)
                    {
                        var snap = data.Snaps[i];
                        EditorGUILayout.LabelField($"  {i}: {snap.type} - Node: {snap.node}");
                    }
                    if (data.Snaps.Length > 3)
                    {
                        EditorGUILayout.LabelField($"  ... and {data.Snaps.Length - 3} more");
                    }
                    EditorGUI.indentLevel--;
                }
            }

            EditorGUILayout.Toggle("Enable Motor", data.EnableMotor);
            if (data.EnableMotor)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.TextField("Axis Node Set", data.AxisNodeSet);
                EditorGUILayout.FloatField("Target Rate", data.TargetRate);
                EditorGUILayout.FloatField("Max Torque", data.MaxTorque);
                EditorGUI.indentLevel--;
            }

            EditorGUI.EndDisabledGroup();
        }

        private void CreateNewConstraintData()
        {
            string path = EditorUtility.SaveFilePanelInProject(
                "Create ConstraintData",
                "NewConstraintData",
                "asset",
                "Choose where to save the new ConstraintData asset");

            if (!string.IsNullOrEmpty(path))
            {
                ConstraintData newData = CreateInstance<ConstraintData>();
                newData.SetConstraintSettings($"Constraint for {constraint.name}", true);

                AssetDatabase.CreateAsset(newData, path);
                AssetDatabase.SaveAssets();

                constraintData.objectReferenceValue = newData;
                serializedObject.ApplyModifiedProperties();

                Selection.activeObject = newData;
            }
        }

        private void ShowConstraintDataSelector()
        {
            string[] guids = AssetDatabase.FindAssets("t:ConstraintData");
            if (guids.Length == 0)
            {
                EditorUtility.DisplayDialog("No ConstraintData Found", "No ConstraintData assets found in project.", "OK");
                return;
            }

            GenericMenu menu = new GenericMenu();
            foreach (string guid in guids)
            {
                string assetPath = AssetDatabase.GUIDToAssetPath(guid);
                ConstraintData data = AssetDatabase.LoadAssetAtPath<ConstraintData>(assetPath);
                if (data != null)
                {
                    menu.AddItem(new GUIContent(data.name), false, () =>
                    {
                        constraintData.objectReferenceValue = data;
                        serializedObject.ApplyModifiedProperties();
                    });
                }
            }
            menu.ShowAsContext();
        }

        private BaseBodyValidationResult ValidateBaseBody()
        {
            if (baseBody.objectReferenceValue == null)
            {
                return new BaseBodyValidationResult
                {
                    status = BaseBodyValidationStatus.NotAssigned,
                    message = "Base Body is not assigned. This constraint will have no effect."
                };
            }

            var baseSoftBody = baseBody.objectReferenceValue as SoftBody;
            if (baseSoftBody == null)
            {
                return new BaseBodyValidationResult
                {
                    status = BaseBodyValidationStatus.Error,
                    message = "Base Body must be a SoftBody component."
                };
            }

            if (baseSoftBody == constraint.GetComponent<SoftBody>())
            {
                return new BaseBodyValidationResult
                {
                    status = BaseBodyValidationStatus.Warning,
                    message = "Base Body is the same as this constraint's SoftBody. This may cause unexpected behavior."
                };
            }

            return new BaseBodyValidationResult
            {
                status = BaseBodyValidationStatus.Valid,
                message = "Base Body configuration is valid."
            };
        }

        private void CacheSerializedProperties()
        {
            baseBody = serializedObject.FindProperty("baseBody");
            constraintData = serializedObject.FindProperty("constraintData");
            showSnaps = serializedObject.FindProperty("showSnaps");
            snaps = serializedObject.FindProperty("snaps");
            disableCollision = serializedObject.FindProperty("disableCollision");
            enableMotor = serializedObject.FindProperty("enableMotor");
            axisNodeSet = serializedObject.FindProperty("axisNodeSet");
            targetRate = serializedObject.FindProperty("targetRate");
            maxTorque = serializedObject.FindProperty("maxTorque");

            if (constraintData == null)
            {
                return;
            }
        }

        void OnDisable()
        {
            // Unsubscribe from selection change events
            Selection.selectionChanged -= OnSelectionChanged;

            // Force scene repaint to clear any lingering visualizations
            SceneView.RepaintAll();
        }

        private void OnSelectionChanged()
        {
            // Check if selection changed from/to a constraint-related object
            GameObject currentSelection = Selection.activeGameObject;

            if (currentSelection != s_lastSelectedObject)
            {
                // Clear any cached visualization data and force refresh
                if (s_lastSelectedObject != null)
                {
                    // Previous selection might have had constraint visualizations
                    m_needsVisualizationRefresh = true;
                }

                if (currentSelection != null && constraint != null)
                {
                    // Check if new selection is constraint-related
                    bool isConstraintRelated =
                        currentSelection == constraint.gameObject ||
                        currentSelection == constraint.BaseBody?.gameObject ||
                        (currentSelection.GetComponent<Constraint>() != null) ||
                        (currentSelection.GetComponent<SoftBody>() != null);

                    if (isConstraintRelated)
                    {
                        m_needsVisualizationRefresh = true;
                    }
                }

                s_lastSelectedObject = currentSelection;

                // Force immediate scene refresh if needed
                if (m_needsVisualizationRefresh)
                {
                    RefreshConstraintVisualization();
                    m_needsVisualizationRefresh = false;
                }
            }
        }

        private void ForceSceneRefreshOnSelectionChange()
        {
            // Clear any stale visualizations immediately
            SceneView.RepaintAll();

            // Mark that we need a fresh render of constraint data
            m_needsVisualizationRefresh = true;
        }

        private void RefreshConstraintVisualization()
        {
            // Force refresh of all cached data
            RefreshAllNodeData();

            // Clear and repaint scene views immediately
            SceneView.RepaintAll();

            // Mark scene as dirty to force complete refresh
            EditorApplication.QueuePlayerLoopUpdate();
        }

        private void HandlePlayModeStateChanges()
        {
            // Detect play mode state changes
            bool currentlyInPlayMode = Application.isPlaying;

            // If we just exited play mode or data refresh is needed
            if (m_needsDataRefresh || (s_wasInPlayMode && !currentlyInPlayMode))
            {
                RefreshAllNodeData();
                m_needsDataRefresh = false;
                s_wasInPlayMode = false;
            }
            else if (currentlyInPlayMode && !s_wasInPlayMode)
            {
                s_wasInPlayMode = true;
            }
        }

        private void RefreshAllNodeData()
        {
            // Force refresh of all node visualization and data caches
            EnableNodeVisualizationForConstraintSetup();

            // Clear any cached position data by forcing a repaint
            SceneView.RepaintAll();

            // Force repaint to update dropdowns and UI
            Repaint();
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

        private new void DrawHeader()
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

        // Quick actions removed for cleaner interface

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

            // Initialize individual snap foldout states if needed
            if (individualSnapFoldouts == null || individualSnapFoldouts.Length != snaps.arraySize)
            {
                individualSnapFoldouts = new bool[snaps.arraySize];
                for (int i = 0; i < individualSnapFoldouts.Length; i++)
                {
                    individualSnapFoldouts[i] = true; // Default to expanded
                }
            }

            // Draw existing snaps
            for (int i = snaps.arraySize - 1; i >= 0; i--)
            {
                var snapProp = snaps.GetArrayElementAtIndex(i);
                if (snapProp == null) continue;

                EditorGUILayout.BeginVertical(EditorStyles.helpBox);

                // Header with collapsible foldout and delete button
                EditorGUILayout.BeginHorizontal();

                // Ensure we have a valid index in our foldouts array
                if (i >= individualSnapFoldouts.Length)
                {
                    System.Array.Resize(ref individualSnapFoldouts, i + 1);
                    individualSnapFoldouts[i] = true;
                }

                // Collapsible foldout for this snap
                individualSnapFoldouts[i] = EditorGUILayout.Foldout(individualSnapFoldouts[i], $"Snap {i}", true, EditorStyles.foldout);

                GUILayout.FlexibleSpace();
                if (GUILayout.Button("✕", GUILayout.Width(20), GUILayout.Height(20)))
                {
                    snaps.DeleteArrayElementAtIndex(i);
                    // Resize the foldouts array to match
                    if (individualSnapFoldouts.Length > snaps.arraySize)
                    {
                        System.Array.Resize(ref individualSnapFoldouts, snaps.arraySize);
                    }
                    EditorGUILayout.EndHorizontal();
                    EditorGUILayout.EndVertical();
                    continue;
                }
                EditorGUILayout.EndHorizontal();

                // Only draw snap properties if expanded
                if (individualSnapFoldouts[i])
                {
                    EditorGUILayout.Space(3);
                    DrawSnapProperties(snapProp);
                }

                EditorGUILayout.EndVertical();
                EditorGUILayout.Space(5);
            }

            // Add new snap button
            if (GUILayout.Button("+ Add New Snap", GUILayout.Height(30)))
            {
                AddNewSnap(SnapType.Node);
                // Resize foldouts array and set new snap as expanded
                System.Array.Resize(ref individualSnapFoldouts, snaps.arraySize);
                if (snaps.arraySize > 0)
                {
                    individualSnapFoldouts[snaps.arraySize - 1] = true;
                }
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

            // Snap type with better visual styling
            EditorGUILayout.Space(5);
            EditorGUILayout.LabelField("🔗 Snap Configuration", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(typeProp, new GUIContent("Snap Type"));
            SnapType snapType = (SnapType)typeProp.enumValueIndex;

            EditorGUILayout.Space(3);
            
            // Source node selection
            EditorGUILayout.LabelField("📍 Source (This SoftBody)", EditorStyles.boldLabel);
            DrawEnhancedNodeField(nodeProp, "Source Node", GetAttachedBodyNodes(), constraint?.GetComponent<SoftBody>());

            EditorGUILayout.Space(3);
            
            // Target selection with appropriate icon based on snap type
            string targetIcon = snapType switch 
            {
                SnapType.Node => "🎯",
                SnapType.Edge => "📏", 
                SnapType.Point => "📌",
                _ => "🎯"
            };
            
            EditorGUILayout.LabelField($"{targetIcon} Target ({snapType})", EditorStyles.boldLabel);
            
            if (snapType == SnapType.Node && targetNodeProp != null)
                DrawEnhancedNodeField(targetNodeProp, "Target Node", GetBaseBodyNodes(), GetBaseSoftBody());
            else if (snapType == SnapType.Edge && targetNodeProp != null)
                DrawEdgeField(targetNodeProp);
            else if (snapType == SnapType.Point && targetPositionProp != null)
                EditorGUILayout.PropertyField(targetPositionProp, new GUIContent("Target Position"));

            EditorGUILayout.Space(5);
            
            // Constraint limits section
            EditorGUILayout.LabelField("⚙️ Constraint Limits", EditorStyles.boldLabel);
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

            EditorGUILayout.Space(5);

            // Enhanced Strength Controls
            if (strengthProp != null)
            {
                EditorGUI.BeginChangeCheck();
                float strengthValue = strengthProp.floatValue;

                EditorGUILayout.LabelField("💪 Constraint Strength", EditorStyles.boldLabel);

                if (float.IsPositiveInfinity(strengthValue))
                {
                    bool isInfinite = EditorGUILayout.Toggle("🔗 Rigid (Infinite)", true);
                    if (!isInfinite)
                        strengthProp.floatValue = 1000f;

                    EditorGUILayout.HelpBox("Rigid constraint - nodes are locked together like welding.", MessageType.Info);
                }
                else
                {
                    bool isInfinite = EditorGUILayout.Toggle("🔗 Rigid (Infinite)", false);
                    if (isInfinite)
                    {
                        strengthProp.floatValue = float.PositiveInfinity;
                    }
                    else
                    {
                        // Strength slider with presets
                        EditorGUILayout.BeginHorizontal();
                        EditorGUILayout.LabelField("🌊 Flexible Strength:", GUILayout.Width(120));

                        float newStrength = EditorGUILayout.FloatField(strengthValue, GUILayout.Width(80));
                        if (newStrength != strengthValue)
                            strengthProp.floatValue = Mathf.Max(1f, newStrength);
                        EditorGUILayout.EndHorizontal();

                        // Visual strength slider
                        float sliderValue = Mathf.Log10(Mathf.Clamp(strengthValue, 1f, 10000f));
                        float minSlider = 0f; // 10^0 = 1
                        float maxSlider = 4f;  // 10^4 = 10000

                        EditorGUI.BeginChangeCheck();
                        sliderValue = EditorGUILayout.Slider("", sliderValue, minSlider, maxSlider);
                        if (EditorGUI.EndChangeCheck())
                        {
                            strengthProp.floatValue = Mathf.Pow(10f, sliderValue);
                        }

                        // Strength presets
                        EditorGUILayout.BeginHorizontal();
                        if (GUILayout.Button("Soft\n(100)", GUILayout.Height(30), GUILayout.Width(60)))
                            strengthProp.floatValue = 100f;
                        if (GUILayout.Button("Medium\n(500)", GUILayout.Height(30), GUILayout.Width(60)))
                            strengthProp.floatValue = 500f;
                        if (GUILayout.Button("Strong\n(2000)", GUILayout.Height(30), GUILayout.Width(60)))
                            strengthProp.floatValue = 2000f;
                        if (GUILayout.Button("Very Strong\n(8000)", GUILayout.Height(30), GUILayout.Width(80)))
                            strengthProp.floatValue = 8000f;
                        EditorGUILayout.EndHorizontal();

                        // Strength description
                        string strengthDesc = strengthValue switch
                        {
                            < 50f => "Very Soft - High flexibility, springy behavior",
                            < 200f => "Soft - Noticeable flexibility, allows movement",
                            < 1000f => "Medium - Balanced between rigid and flexible",
                            < 3000f => "Strong - Low flexibility, mostly stable",
                            < 8000f => "Very Strong - Minimal flexibility, near-rigid",
                            _ => "Extremely Strong - Almost rigid behavior"
                        };

                        EditorGUILayout.HelpBox($"💪 {strengthDesc}", MessageType.None);
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
                EditorGUILayout.HelpBox($"No nodes available for {label}. Make sure the SoftBody has a TrussAsset assigned with nodes defined.", MessageType.Warning);
                return;
            }

            string currentValue = nodeProp.stringValue ?? "";
            
            // Find matching index, accounting for enhanced names
            int selectedIndex = -1;
            for (int i = 0; i < availableNodes.Length; i++)
            {
                string nodeName = availableNodes[i];
                // Check if current value matches the base node name (e.g., "node0")
                if (nodeName.StartsWith(currentValue) || nodeName == currentValue)
                {
                    selectedIndex = i;
                    break;
                }
            }
            
            if (selectedIndex == -1) selectedIndex = 0;

            EditorGUI.BeginChangeCheck();
            
            // Enhanced dropdown with better styling
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField(label, GUILayout.Width(120));
            
            selectedIndex = EditorGUILayout.Popup(selectedIndex, availableNodes);
            
            if (GUILayout.Button("ℹ", GUILayout.Width(20)))
            {
                ShowNodeInfo(availableNodes, selectedIndex);
            }
            EditorGUILayout.EndHorizontal();
            
            if (EditorGUI.EndChangeCheck())
            {
                // Extract base node name from enhanced name (e.g., "node0 (wheelHub)" → "node0")
                string selectedNodeName = availableNodes[selectedIndex];
                string baseNodeName = selectedNodeName.Split(' ')[0]; // Get "node0" part
                nodeProp.stringValue = baseNodeName;
            }
            
            // Show helpful info if node has set information
            if (selectedIndex >= 0 && selectedIndex < availableNodes.Length)
            {
                string selectedNodeName = availableNodes[selectedIndex];
                if (selectedNodeName.Contains("(") && selectedNodeName.Contains(")"))
                {
                    int start = selectedNodeName.IndexOf("(") + 1;
                    int end = selectedNodeName.IndexOf(")");
                    string setName = selectedNodeName.Substring(start, end - start);
                    EditorGUILayout.LabelField("", $"Part of node set: {setName}", EditorStyles.miniLabel);
                }
            }
        }

        private void ShowNodeInfo(string[] availableNodes, int selectedIndex)
        {
            if (selectedIndex < 0 || selectedIndex >= availableNodes.Length)
                return;

            string selectedNodeName = availableNodes[selectedIndex];
            string message = $"Selected Node: {selectedNodeName}\n\n";
            
            // Extract node index
            string baseNodeName = selectedNodeName.Split(' ')[0];
            if (baseNodeName.StartsWith("node") && int.TryParse(baseNodeName.Substring(4), out int nodeIndex))
            {
                message += $"Node Index: {nodeIndex}\n";
                
                // Try to get position information
                var attachedSoftBody = constraint?.GetComponent<SoftBody>();
                if (attachedSoftBody != null)
                {
                    Vector3 nodePos = GetEditModeNodePosition(nodeIndex, attachedSoftBody);
                    message += $"Estimated Position: {nodePos:F2}\n";
                }
                
                // Add node set information if available
                if (selectedNodeName.Contains("(") && selectedNodeName.Contains(")"))
                {
                    int start = selectedNodeName.IndexOf("(") + 1;
                    int end = selectedNodeName.IndexOf(")");
                    string setName = selectedNodeName.Substring(start, end - start);
                    message += $"\nNode Set: {setName}\n";
                    message += "This node is part of a named group in the TrussAsset.";
                }
            }
            
            EditorUtility.DisplayDialog("Node Information", message, "OK");
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
                EditorGUILayout.HelpBox("Need at least 2 nodes for edge selection. Make sure the Base Body has a TrussAsset assigned with multiple nodes.", MessageType.Warning);
                return;
            }

            EditorGUILayout.LabelField("Target Edge (Base Body)", EditorStyles.boldLabel);

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

            // Enhanced edge selection with better UI
            EditorGUI.BeginChangeCheck();
            
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Node 1:", GUILayout.Width(60));
            node1Index = EditorGUILayout.Popup(node1Index, baseNodes);
            if (GUILayout.Button("ℹ", GUILayout.Width(20)))
            {
                ShowNodeInfo(baseNodes, node1Index);
            }
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Node 2:", GUILayout.Width(60));
            node2Index = EditorGUILayout.Popup(node2Index, baseNodes);
            if (GUILayout.Button("ℹ", GUILayout.Width(20)))
            {
                ShowNodeInfo(baseNodes, node2Index);
            }
            EditorGUILayout.EndHorizontal();

            if (EditorGUI.EndChangeCheck())
            {
                // Extract base node names for the edge string
                string baseNode1 = baseNodes[node1Index].Split(' ')[0];
                string baseNode2 = baseNodes[node2Index].Split(' ')[0];
                
                // Extract node indices
                int index1 = int.Parse(baseNode1.Substring(4)); // "node0" -> 0
                int index2 = int.Parse(baseNode2.Substring(4)); // "node1" -> 1
                
                targetNodeProp.stringValue = $"nodes{index1},{index2}";
            }

            // Show current edge information
            EditorGUILayout.Space(5);
            EditorGUILayout.LabelField("Edge Configuration:", EditorStyles.miniLabel);
            EditorGUILayout.LabelField($"  Edge String: {targetNodeProp.stringValue}", EditorStyles.miniLabel);
            
            if (node1Index == node2Index)
            {
                EditorGUILayout.HelpBox("Warning: Both nodes are the same. An edge requires two different nodes.", MessageType.Warning);
            }
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

        private SoftBody GetBaseSoftBody()
        {
            return constraint?.BaseBody as SoftBody;
        }

        private void DrawEnhancedNodeField(SerializedProperty nodeProp, string label, string[] availableNodes, SoftBody softBody)
        {
            // Use the existing DrawNodeField but could be enhanced further for node sets
            DrawNodeField(nodeProp, label, availableNodes);
            
            // Add quick node set selection if available
            if (softBody != null)
            {
                var trussAsset = softBody.GetTrussAsset();
                if (trussAsset?.NodeSets != null && trussAsset.NodeSets.Count > 0)
                {
                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.LabelField("", GUILayout.Width(120));
                    
                    if (GUILayout.Button("📋 Quick Select from Node Set", GUILayout.Height(18)))
                    {
                        ShowNodeSetSelectionMenu(nodeProp, trussAsset);
                    }
                    EditorGUILayout.EndHorizontal();
                }
            }
        }

        private void ShowNodeSetSelectionMenu(SerializedProperty nodeProp, TrussAsset trussAsset)
        {
            GenericMenu menu = new GenericMenu();
            
            var nodeSets = trussAsset.NodeSets;
            for (int i = 0; i < nodeSets.Count; i++)
            {
                var nodeSet = nodeSets[i];
                if (nodeSet.nodeIndices != null && nodeSet.nodeIndices.Count > 0)
                {
                    // Add each node in the set as a menu option
                    foreach (int nodeIndex in nodeSet.nodeIndices)
                    {
                        string menuPath = $"{nodeSet.name}/node{nodeIndex}";
                        menu.AddItem(new GUIContent(menuPath), false, () => {
                            nodeProp.stringValue = $"node{nodeIndex}";
                            nodeProp.serializedObject.ApplyModifiedProperties();
                        });
                    }
                }
            }
            
            if (menu.GetItemCount() == 0)
            {
                menu.AddDisabledItem(new GUIContent("No node sets available"));
            }
            
            menu.ShowAsContext();
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

            // Try runtime solver first (play mode)
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

            // Edit mode: Try TrussAsset first (most reliable source)
            var trussAsset = softBody.GetTrussAsset();
            if (trussAsset != null)
            {
                var enhancedNodes = GetEnhancedNodesFromTrussAsset(trussAsset);
                if (enhancedNodes.Length > 0)
                {
                    return enhancedNodes;
                }
            }

            // Fallback to mesh vertices if no truss asset
            var meshFilter = softBody.GetComponent<MeshFilter>();
            if (meshFilter?.sharedMesh != null)
            {
                var mesh = meshFilter.sharedMesh;
                var vertices = mesh.vertices;
                if (vertices.Length > 0)
                {
                    return GenerateDefaultNodeNames(vertices.Length);
                }
            }

            // Ultimate fallback
            return GenerateDefaultNodeNames(8);
        }

        private string[] GetEnhancedNodesFromTrussAsset(TrussAsset trussAsset)
        {
            if (trussAsset == null) return new string[0];

            try
            {
                var nodePositions = trussAsset.NodePositions;
                if (nodePositions == null || nodePositions.Length == 0)
                    return new string[0];

                string[] nodeNames = new string[nodePositions.Length];
                
                // Create basic node names
                for (int i = 0; i < nodePositions.Length; i++)
                {
                    nodeNames[i] = $"node{i}";
                }

                // Enhance with NodeSet names if available
                var nodeSets = trussAsset.NodeSets;
                if (nodeSets != null && nodeSets.Count > 0)
                {
                    foreach (var nodeSet in nodeSets)
                    {
                        if (nodeSet.nodeIndices != null && !string.IsNullOrEmpty(nodeSet.name))
                        {
                            foreach (int nodeIndex in nodeSet.nodeIndices)
                            {
                                if (nodeIndex >= 0 && nodeIndex < nodeNames.Length)
                                {
                                    // Enhance node name with set information
                                    nodeNames[nodeIndex] = $"node{nodeIndex} ({nodeSet.name})";
                                }
                            }
                        }
                    }
                }

                return nodeNames;
            }
            catch (System.Exception)
            {
                return new string[0];
            }
        }

        private string[] GetNodesFromTrussAsset(TrussAsset trussAsset)
        {
            if (trussAsset == null) return new string[0];

            try
            {
                // Direct access to NodePositions property (most reliable)
                var nodePositions = trussAsset.NodePositions;
                if (nodePositions != null && nodePositions.Length > 0)
                {
                    return GenerateDefaultNodeNames(nodePositions.Length);
                }

                return new string[0];
            }
            catch (System.Exception)
            {
                return new string[0];
            }
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

        // Quick action creation methods removed for cleaner interface

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

        // Scene GUI for handles and enhanced constraint visualization
        void OnSceneGUI()
        {
            if (constraint == null) return;

            // Handle selection change detection in scene GUI
            HandleSceneSelectionChanges();

            // Always draw constraint previews when constraint is selected (both edit and play mode)
            DrawConstraintPreviews();

            // Draw interactive handles when ShowSnaps is enabled
            if (constraint.ShowSnaps)
            {
                DrawInteractiveHandles();
            }
        }

        private void HandleSceneSelectionChanges()
        {
            // Check if we need to refresh visualization due to stale data
            if (m_needsVisualizationRefresh)
            {
                RefreshConstraintVisualization();
                m_needsVisualizationRefresh = false;
            }

            // Force repaint if selection changed recently and we're drawing constraints
            Event currentEvent = Event.current;
            if (currentEvent.type == EventType.Repaint)
            {
                // Clear any stale handles or gizmos by forcing a complete redraw
                if (Selection.activeGameObject != constraint.gameObject)
                {
                    // If constraint is no longer selected, its visualizations should be hidden
                    return;
                }
            }
        }

        private void DrawConstraintPreviews()
        {
            if (constraint.Snaps == null || constraint.Snaps.Length == 0) return;
            if (individualSnapFoldouts == null) return;

            // Only draw previews for expanded (uncollapsed) snaps
            for (int i = 0; i < constraint.Snaps.Length && i < individualSnapFoldouts.Length; i++)
            {
                // Only show gizmos for expanded snap foldouts (acts as toggle)
                if (!individualSnapFoldouts[i]) continue;

                var snap = constraint.Snaps[i];
                if (!snap.show) continue; // Also respect the individual show flag

                DrawSnapPreview(snap, i);
            }
        }

        private void DrawInteractiveHandles()
        {
            if (constraint.Snaps == null) return;
            if (individualSnapFoldouts == null) return;

            // Only draw handles for expanded (uncollapsed) snaps
            for (int i = 0; i < constraint.Snaps.Length && i < individualSnapFoldouts.Length; i++)
            {
                // Only show handles for expanded snap foldouts
                if (!individualSnapFoldouts[i]) continue;

                var snap = constraint.Snaps[i];
                if (!snap.show) continue;

                DrawSnapHandle(snap);
            }
        }

        private void DrawSnapPreview(Snap snap, int snapIndex)
        {
            var nodeIndices = FindNodeSetForPreview(snap.node);
            if (nodeIndices == null || nodeIndices.Length == 0)
            {
                // Debug info for troubleshooting
                Handles.Label(constraint.transform.position + Vector3.up * (0.5f + snapIndex * 0.3f),
                    $"Snap {snapIndex}: No nodes found for '{snap.node}'", EditorStyles.helpBox);
                return;
            }

            foreach (int nodeIndex in nodeIndices)
            {
                Vector3 nodePosition = GetNodePositionForPreview(nodeIndex, constraint.GetComponent<SoftBody>());
                if (nodePosition == Vector3.zero)
                {
                    // Try alternative positioning
                    nodePosition = constraint.transform.position + Vector3.right * nodeIndex * 0.2f;
                }

                // Set colors based on constraint strength
                Color constraintColor = GetConstraintColor(snap.strength);
                Color connectionColor = constraintColor;
                connectionColor.a = 0.8f;

                // Draw the source node marker
                Handles.color = constraintColor;
                Handles.SphereHandleCap(0, nodePosition, Quaternion.identity, 0.05f, EventType.Repaint);

                switch (snap.type)
                {
                    case SnapType.Point:
                        DrawPointConstraintPreview(nodePosition, snap.targetPosition, constraintColor, snap, snapIndex);
                        break;

                    case SnapType.Node:
                        DrawNodeConstraintPreview(nodePosition, snap, constraintColor, snapIndex);
                        break;

                    case SnapType.Edge:
                        DrawEdgeConstraintPreview(nodePosition, snap, constraintColor, snapIndex);
                        break;
                }

                // Constraint info labels removed for cleaner Scene View
            }
        }

        private void DrawPointConstraintPreview(Vector3 nodePosition, Vector3 targetPosition, Color color, Snap snap, int snapIndex)
        {
            Handles.color = color;

            // Draw connection line with thickness based on strength
            float lineThickness = GetLineThickness(snap.strength);
            Handles.DrawLine(nodePosition, targetPosition, lineThickness);

            // Draw target anchor
            Handles.color = Color.cyan;
            Handles.SphereHandleCap(0, targetPosition, Quaternion.identity, 0.08f, EventType.Repaint);

            // Target position label removed for cleaner Scene View

            // Draw enhanced distance limits visualization without center cubes
            DrawConstraintLimitSpheresNoCubes(nodePosition, snap);

            // Draw constraint arrow
            Vector3 direction = (targetPosition - nodePosition).normalized;
            if (direction.magnitude > 0.01f)
            {
                float arrowSize = 0.15f;
                Handles.color = color;
                Vector3 arrowPos = Vector3.Lerp(nodePosition, targetPosition, 0.7f);
                Handles.ArrowHandleCap(0, arrowPos, Quaternion.LookRotation(direction), arrowSize, EventType.Repaint);
            }
        }

        private void DrawNodeConstraintPreview(Vector3 nodePosition, Snap snap, Color color, int snapIndex)
        {
            if (!(constraint.BaseBody is SoftBody baseSoftBody))
            {
                Handles.Label(nodePosition + Vector3.up * 0.3f, "No SoftBody base!", EditorStyles.helpBox);
                return;
            }

            var baseIndices = FindNodeSetOnSoftBodyForPreview(baseSoftBody, snap.targetNode);
            if (baseIndices == null || baseIndices.Length == 0)
            {
                Handles.Label(nodePosition + Vector3.up * 0.3f, $"No target nodes: '{snap.targetNode}'", EditorStyles.helpBox);
                return;
            }

            foreach (int targetIndex in baseIndices)
            {
                Vector3 targetPosition = GetNodePositionForPreview(targetIndex, baseSoftBody);
                if (targetPosition == Vector3.zero)
                {
                    // Fallback positioning for base body nodes
                    targetPosition = baseSoftBody.transform.position + Vector3.forward * targetIndex * 0.3f;
                }

                Handles.color = color;

                // Draw connection line with thickness based on strength
                float lineThickness = GetLineThickness(snap.strength);
                Handles.DrawLine(nodePosition, targetPosition, lineThickness);

                // Draw target node
                Handles.color = Color.green;
                Handles.SphereHandleCap(0, targetPosition, Quaternion.identity, 0.08f, EventType.Repaint);

                // Target node label removed for cleaner Scene View

                // Draw constraint spring visualization for flexible constraints
                if (!float.IsPositiveInfinity(snap.strength))
                {
                    DrawSpringVisualization(nodePosition, targetPosition, color, snap);
                }
            }

            // Draw enhanced distance limits visualization without center cubes
            DrawConstraintLimitSpheresNoCubes(nodePosition, snap);
        }

        private void DrawEdgeConstraintPreview(Vector3 nodePosition, Snap snap, Color color, int snapIndex)
        {
            if (!(constraint.BaseBody is SoftBody baseSoftBody))
            {
                Handles.Label(nodePosition + Vector3.up * 0.3f, "No SoftBody base!", EditorStyles.helpBox);
                return;
            }

            var baseIndices = FindNodeSetOnSoftBodyForPreview(baseSoftBody, snap.targetNode);
            if (baseIndices == null || baseIndices.Length < 2)
            {
                Handles.Label(nodePosition + Vector3.up * 0.3f, $"Edge needs 2 nodes: '{snap.targetNode}'", EditorStyles.helpBox);
                return;
            }

            Vector3 nodeAPosition = GetNodePositionForPreview(baseIndices[0], baseSoftBody);
            Vector3 nodeBPosition = GetNodePositionForPreview(baseIndices[1], baseSoftBody);

            // Fallback positioning if nodes can't be found
            if (nodeAPosition == Vector3.zero)
                nodeAPosition = baseSoftBody.transform.position + Vector3.forward * baseIndices[0] * 0.3f;
            if (nodeBPosition == Vector3.zero)
                nodeBPosition = baseSoftBody.transform.position + Vector3.forward * baseIndices[1] * 0.3f;

            // Draw the edge line
            Handles.color = Color.cyan;
            float edgeThickness = 0.04f;
            Handles.DrawLine(nodeAPosition, nodeBPosition, edgeThickness);

            // Draw edge nodes
            Handles.SphereHandleCap(0, nodeAPosition, Quaternion.identity, 0.06f, EventType.Repaint);
            Handles.SphereHandleCap(0, nodeBPosition, Quaternion.identity, 0.06f, EventType.Repaint);

            // Edge node labels removed for cleaner Scene View

            // Find closest point on edge
            Vector3 closestPoint = GetClosestPointOnLineSegment(nodePosition, nodeAPosition, nodeBPosition);

            // Draw connection to closest point
            Handles.color = color;
            float lineThickness = GetLineThickness(snap.strength);
            Handles.DrawLine(nodePosition, closestPoint, lineThickness);

            // Draw closest point marker
            Handles.color = Color.magenta;
            Handles.SphereHandleCap(0, closestPoint, Quaternion.identity, 0.08f, EventType.Repaint);
            // Closest point label removed for cleaner Scene View

            // Draw constraint spring visualization for flexible constraints
            if (!float.IsPositiveInfinity(snap.strength))
            {
                DrawSpringVisualization(nodePosition, closestPoint, color, snap);
            }

            // Draw enhanced distance limits visualization without center cubes
            DrawConstraintLimitSpheresNoCubes(nodePosition, snap);
        }

        /// <summary>
        /// Draw enhanced min/max limit spheres using Handles for editor preview - WITHOUT center cubes
        /// </summary>
        private void DrawConstraintLimitSpheresNoCubes(Vector3 nodePosition, Snap snap)
        {
            // Draw maximum limit sphere (outer boundary)
            if (snap.maxLimit > 0)
            {
                // Draw solid sphere with transparency
                Handles.color = new Color(1f, 0f, 0f, 0.1f); // Semi-transparent red
                DrawHandlesSphere(nodePosition, snap.maxLimit);

                // Draw wire sphere for clear boundary
                Handles.color = new Color(1f, 0f, 0f, 0.6f); // More opaque red
                Handles.DrawWireDisc(nodePosition, Vector3.up, snap.maxLimit);
                Handles.DrawWireDisc(nodePosition, Vector3.right, snap.maxLimit);
                Handles.DrawWireDisc(nodePosition, Vector3.forward, snap.maxLimit);

                // Draw limit indicator markers - without labels
                DrawLimitIndicatorsNoCubes(nodePosition, snap.maxLimit, new Color(1f, 0f, 0f, 0.8f), "MAX");
            }

            // Draw minimum limit sphere (inner boundary)
            if (snap.minLimit > 0)
            {
                // Draw solid sphere with transparency
                Handles.color = new Color(0f, 1f, 0f, 0.15f); // Semi-transparent green
                DrawHandlesSphere(nodePosition, snap.minLimit);

                // Draw wire sphere for clear boundary
                Handles.color = new Color(0f, 1f, 0f, 0.8f); // More opaque green
                Handles.DrawWireDisc(nodePosition, Vector3.up, snap.minLimit);
                Handles.DrawWireDisc(nodePosition, Vector3.right, snap.minLimit);
                Handles.DrawWireDisc(nodePosition, Vector3.forward, snap.minLimit);

                // Draw limit indicator markers - without labels
                DrawLimitIndicatorsNoCubes(nodePosition, snap.minLimit, new Color(0f, 1f, 0f, 0.8f), "MIN");
            }
        }

        /// <summary>
        /// Draw small indicator markers around limit spheres - without labels or center cubes
        /// </summary>
        private void DrawLimitIndicatorsNoCubes(Vector3 center, float radius, Color color, string limitType)
        {
            // Indicator spheres removed for cleaner Scene View - they looked too much like nodes
            // The wireframe spheres already provide clear boundary visualization
        }

        /// <summary>
        /// Draw enhanced min/max limit spheres using Handles for editor preview
        /// </summary>
        private void DrawConstraintLimitSpheres(Vector3 nodePosition, Snap snap)
        {
            // Draw maximum limit sphere (outer boundary)
            if (snap.maxLimit > 0)
            {
                // Draw solid sphere with transparency
                Handles.color = new Color(1f, 0f, 0f, 0.1f); // Semi-transparent red
                DrawHandlesSphere(nodePosition, snap.maxLimit);

                // Draw wire sphere for clear boundary
                Handles.color = new Color(1f, 0f, 0f, 0.6f); // More opaque red
                Handles.DrawWireDisc(nodePosition, Vector3.up, snap.maxLimit);
                Handles.DrawWireDisc(nodePosition, Vector3.right, snap.maxLimit);
                Handles.DrawWireDisc(nodePosition, Vector3.forward, snap.maxLimit);

                // Draw limit indicator markers
                DrawLimitIndicators(nodePosition, snap.maxLimit, new Color(1f, 0f, 0f, 0.8f), "MAX");
            }

            // Draw minimum limit sphere (inner boundary)
            if (snap.minLimit > 0)
            {
                // Draw solid sphere with transparency
                Handles.color = new Color(0f, 1f, 0f, 0.15f); // Semi-transparent green
                DrawHandlesSphere(nodePosition, snap.minLimit);

                // Draw wire sphere for clear boundary
                Handles.color = new Color(0f, 1f, 0f, 0.8f); // More opaque green
                Handles.DrawWireDisc(nodePosition, Vector3.up, snap.minLimit);
                Handles.DrawWireDisc(nodePosition, Vector3.right, snap.minLimit);
                Handles.DrawWireDisc(nodePosition, Vector3.forward, snap.minLimit);

                // Draw limit indicator markers
                DrawLimitIndicators(nodePosition, snap.minLimit, new Color(0f, 1f, 0f, 0.8f), "MIN");
            }

            // Constraint strength indicators removed for cleaner Scene View
        }

        /// <summary>
        /// Draw a sphere using Handles
        /// </summary>
        private void DrawHandlesSphere(Vector3 center, float radius)
        {
            // Draw wireframe sphere approximation using discs
            int discCount = 8;
            for (int i = 0; i < discCount; i++)
            {
                float angle = (i * 180f / discCount) * Mathf.Deg2Rad;
                float discRadius = Mathf.Sin(angle) * radius;
                float height = Mathf.Cos(angle) * radius;

                Vector3 discCenter = center + Vector3.up * height;
                Handles.DrawWireDisc(discCenter, Vector3.up, discRadius);
            }
        }

        /// <summary>
        /// Draw small indicator markers around limit spheres
        /// </summary>
        private void DrawLimitIndicators(Vector3 center, float radius, Color color, string limitType)
        {
            // Indicator spheres removed for cleaner Scene View - they looked too much like nodes
            // The wireframe spheres already provide clear boundary visualization
        }

        /// <summary>
        /// Draw visual indicator for constraint strength using Handles - WITHOUT center cube indicators
        /// </summary>
        private void DrawStrengthIndicatorHandles(Vector3 nodePosition, Snap snap)
        {
            // Strength indicators removed for cleaner Scene View
            // Line thickness and color already indicate strength in the main drawing
        }

        private void DrawSpringVisualization(Vector3 start, Vector3 end, Color color, Snap snap)
        {
            if (float.IsPositiveInfinity(snap.strength)) return; // Don't draw springs for rigid constraints

            Vector3 direction = end - start;
            float distance = direction.magnitude;
            if (distance < 0.01f) return;

            direction.Normalize();
            Vector3 perpendicular = Vector3.Cross(direction, Vector3.up).normalized;
            if (perpendicular.magnitude < 0.01f)
                perpendicular = Vector3.Cross(direction, Vector3.forward).normalized;

            // Draw spring coils
            Handles.color = new Color(color.r, color.g, color.b, 0.5f);
            int coils = Mathf.Max(3, (int)(distance * 10));

            for (int i = 0; i < coils; i++)
            {
                float t = (float)i / coils;
                float nextT = (float)(i + 1) / coils;

                float angle = t * Mathf.PI * 4; // 2 full rotations
                float nextAngle = nextT * Mathf.PI * 4;

                Vector3 springOffset = perpendicular * Mathf.Sin(angle) * 0.05f;
                Vector3 nextSpringOffset = perpendicular * Mathf.Sin(nextAngle) * 0.05f;

                Vector3 springStart = Vector3.Lerp(start, end, t) + springOffset;
                Vector3 springEnd = Vector3.Lerp(start, end, nextT) + nextSpringOffset;

                Handles.DrawLine(springStart, springEnd);
            }
        }

        private Color GetConstraintColor(float strength)
        {
            if (float.IsPositiveInfinity(strength))
                return Color.red; // Rigid constraints in red
            else if (strength > 5000f)
                return Color.yellow; // Very strong in yellow
            else if (strength > 2000f)
                return Color.green; // Strong in green
            else if (strength > 500f)
                return Color.cyan; // Medium in cyan
            else
                return Color.blue; // Soft in blue
        }

        private float GetLineThickness(float strength)
        {
            if (float.IsPositiveInfinity(strength))
                return 4f; // Thick lines for rigid constraints
            else if (strength > 5000f)
                return 3f;
            else if (strength > 2000f)
                return 2.5f;
            else if (strength > 500f)
                return 2f;
            else
                return 1.5f;
        }

        private string GetConstraintInfo(Snap snap, int nodeIndex, int snapIndex)
        {
            string strengthText = float.IsPositiveInfinity(snap.strength) ? "∞ (Rigid)" : snap.strength.ToString("F0");
            return $"Snap {snapIndex}: {snap.type}\nSource Node: {nodeIndex}\nStrength: {strengthText}";
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

        private int[] FindNodeSetForPreview(string nodeSetName)
        {
            // Enhanced node finding for better preview support
            if (string.IsNullOrEmpty(nodeSetName)) return new int[0];

            // Try standard node set helper first
            var result = NodeSetHelper.FindNodeSet(constraint.GetComponent<SoftBody>(), nodeSetName);
            if (result != null && result.Length > 0)
                return result;

            // Try simple node parsing for "node0", "node1", etc.
            if (nodeSetName.StartsWith("node") && nodeSetName.Length > 4)
            {
                string indexString = nodeSetName.Substring(4);
                if (int.TryParse(indexString, out int nodeIndex))
                {
                    return new int[] { nodeIndex };
                }
            }

            // Try nodes parsing for "nodes0,1", etc.
            if (nodeSetName.StartsWith("nodes"))
            {
                string nodesPart = nodeSetName.Replace("nodes", "");
                string[] parts = nodesPart.Split(',');
                List<int> indices = new List<int>();
                foreach (string part in parts)
                {
                    if (int.TryParse(part.Trim(), out int index))
                    {
                        indices.Add(index);
                    }
                }
                if (indices.Count > 0)
                    return indices.ToArray();
            }

            return new int[0];
        }

        private int[] FindNodeSetOnSoftBodyForPreview(SoftBody softBody, string nodeSetName)
        {
            // Enhanced base body node finding
            if (softBody == null || string.IsNullOrEmpty(nodeSetName)) return new int[0];

            // Try standard node set helper first
            var result = NodeSetHelper.FindNodeSet(softBody, nodeSetName);
            if (result != null && result.Length > 0)
                return result;

            // Try simple node parsing
            if (nodeSetName.StartsWith("node") && nodeSetName.Length > 4)
            {
                string indexString = nodeSetName.Substring(4);
                if (int.TryParse(indexString, out int nodeIndex))
                {
                    return new int[] { nodeIndex };
                }
            }

            // Try nodes parsing for edges
            if (nodeSetName.StartsWith("nodes"))
            {
                string nodesPart = nodeSetName.Replace("nodes", "");
                string[] parts = nodesPart.Split(',');
                List<int> indices = new List<int>();
                foreach (string part in parts)
                {
                    if (int.TryParse(part.Trim(), out int index))
                    {
                        indices.Add(index);
                    }
                }
                if (indices.Count > 0)
                    return indices.ToArray();
            }

            return new int[0];
        }

        private Vector3 GetNodePositionForPreview(int nodeIndex, SoftBody softBody)
        {
            if (softBody == null) return Vector3.zero;

            // CRITICAL FIX: Always use edit-mode data for previews to avoid stale solver data
            // The solver can have stale/incorrect node data especially when switching selections
            // This prevents the visualization from showing incorrect constraint lines
            return GetEditModeNodePosition(nodeIndex, softBody);
        }

        private Vector3 GetEditModeNodePosition(int nodeIndex, SoftBody softBody)
        {
            if (softBody == null) return Vector3.zero;

            // Method 1: Try TrussAsset first (most reliable for edit mode)
            var trussAsset = softBody.GetTrussAsset();
            if (trussAsset != null)
            {
                var nodePositions = trussAsset.NodePositions;
                if (nodePositions != null && nodeIndex >= 0 && nodeIndex < nodePositions.Length)
                {
                    // Transform node position from local space to world space
                    return softBody.transform.TransformPoint(nodePositions[nodeIndex]);
                }
            }

            // Method 2: Look for child GameObjects named "node{index}"
            Transform nodeTransform = softBody.transform.Find($"node{nodeIndex}");
            if (nodeTransform != null)
            {
                return nodeTransform.position;
            }

            // Method 3: Try to find nodes in children with node naming patterns
            foreach (Transform child in softBody.transform)
            {
                if (child.name.ToLower().Contains($"node{nodeIndex}") ||
                    child.name.ToLower().Contains($"node_{nodeIndex}") ||
                    child.name.ToLower().Contains($"node {nodeIndex}"))
                {
                    return child.position;
                }
            }

            // Method 4: Use mesh vertices to estimate node positions
            var meshFilter = softBody.GetComponent<MeshFilter>();
            if (meshFilter?.sharedMesh != null)
            {
                var mesh = meshFilter.sharedMesh;
                var vertices = mesh.vertices;

                if (nodeIndex >= 0 && nodeIndex < vertices.Length)
                {
                    // Transform vertex to world space
                    Vector3 worldVertex = softBody.transform.TransformPoint(vertices[nodeIndex]);
                    return worldVertex;
                }

                // If nodeIndex is beyond vertex count, try to map it intelligently
                if (vertices.Length > 0)
                {
                    int mappedIndex = nodeIndex % vertices.Length;
                    Vector3 worldVertex = softBody.transform.TransformPoint(vertices[mappedIndex]);
                    return worldVertex;
                }
            }

            // Method 5: Generate estimated positions around the object bounds
            var renderer = softBody.GetComponent<Renderer>();
            if (renderer != null)
            {
                var bounds = renderer.bounds;

                // Create a grid or circle of estimated node positions
                int totalEstimatedNodes = Mathf.Max(8, nodeIndex + 1);
                float angle = (nodeIndex * 360f / totalEstimatedNodes) * Mathf.Deg2Rad;
                float radius = Mathf.Max(bounds.size.x, bounds.size.z) * 0.4f;

                Vector3 offset = new Vector3(
                    Mathf.Cos(angle) * radius,
                    0f,
                    Mathf.Sin(angle) * radius
                );

                return bounds.center + offset;
            }

            // Final fallback: offset from the transform position
            float fallbackRadius = 0.5f;
            float fallbackAngle = (nodeIndex * 45f) * Mathf.Deg2Rad;
            Vector3 fallbackOffset = new Vector3(
                Mathf.Cos(fallbackAngle) * fallbackRadius,
                0f,
                Mathf.Sin(fallbackAngle) * fallbackRadius
            );

            return softBody.transform.position + fallbackOffset;
        }

        private void DrawSnapHandle(Snap snap)
        {
            Handles.color = Color.cyan;

            switch (snap.type)
            {
                case SnapType.Point:
                    DrawPointHandle(snap);
                    break;

                case SnapType.Node:
                    DrawNodeHandle(snap);
                    break;

                case SnapType.Edge:
                    DrawEdgeHandle(snap);
                    break;
            }
        }

        private void DrawPointHandle(Snap snap)
        {
            EditorGUI.BeginChangeCheck();
            Vector3 newPos = Handles.PositionHandle(snap.targetPosition, Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(constraint, "Move Point Snap");
                snap.targetPosition = newPos;
                EditorUtility.SetDirty(constraint);
            }

            // Draw handle label
            Handles.Label(snap.targetPosition + Vector3.up * 0.2f, $"Point: {snap.node}");
        }

        private void DrawNodeHandle(Snap snap)
        {
            if (!(constraint.BaseBody is SoftBody baseSoftBody)) return;

            var baseIndices = FindNodeSetOnSoftBodyForPreview(baseSoftBody, snap.targetNode);
            if (baseIndices == null || baseIndices.Length == 0) return;

            foreach (int targetIndex in baseIndices)
            {
                Vector3 targetPosition = GetNodePositionForPreview(targetIndex, baseSoftBody);
                if (targetPosition == Vector3.zero) continue;

                // Draw interactive disc handle for target node
                EditorGUI.BeginChangeCheck();
                Vector3 newPos = Handles.FreeMoveHandle(targetPosition, 0.1f, Vector3.zero, Handles.SphereHandleCap);
                if (EditorGUI.EndChangeCheck())
                {
                    // Node positions can't be directly moved in edit mode
                }

                Handles.Label(targetPosition + Vector3.up * 0.2f, $"Target Node: {targetIndex}");
            }
        }

        private void DrawEdgeHandle(Snap snap)
        {
            if (!(constraint.BaseBody is SoftBody baseSoftBody)) return;

            var baseIndices = FindNodeSetOnSoftBodyForPreview(baseSoftBody, snap.targetNode);
            if (baseIndices == null || baseIndices.Length < 2) return;

            Vector3 nodeAPosition = GetNodePositionForPreview(baseIndices[0], baseSoftBody);
            Vector3 nodeBPosition = GetNodePositionForPreview(baseIndices[1], baseSoftBody);

            if (nodeAPosition == Vector3.zero || nodeBPosition == Vector3.zero) return;

            // Draw interactive handles for edge nodes
            EditorGUI.BeginChangeCheck();
            Vector3 newPosA = Handles.FreeMoveHandle(nodeAPosition, 0.08f, Vector3.zero, Handles.SphereHandleCap);
            Vector3 newPosB = Handles.FreeMoveHandle(nodeBPosition, 0.08f, Vector3.zero, Handles.SphereHandleCap);
            if (EditorGUI.EndChangeCheck())
            {
                // Edge node positions can't be directly moved in edit mode
            }

            // Draw edge midpoint label
            Vector3 midpoint = (nodeAPosition + nodeBPosition) * 0.5f;
            Handles.Label(midpoint + Vector3.up * 0.2f, $"Edge: {baseIndices[0]}-{baseIndices[1]}");
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

