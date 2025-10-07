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
using System.Linq;

namespace DynamicEngine
{

    public class Designer : EditorWindow
    {
        [MenuItem("DynamicEngine/Designer", false, 1)]
        public static void ShowWindow()
        {
            var window = GetWindow<Designer>("Designer");
            window.minSize = new Vector2(350, 600);
            window.Show();
        }
        
        #region Tool State & Configuration
        
        [System.Serializable]
        public enum DesignerMode
        {
            Node,
            Beam,
            Face,
            MeshConvert,
            Optimization
        }
        
        [System.Serializable]
        public enum SelectionMode
        {
            Single,
            Multi,
            Box,
            Sphere
        }
        
        private DesignerMode currentMode = DesignerMode.Node;
        private SelectionMode selectionMode = SelectionMode.Single;
        
        // Target objects
        private SoftBody targetSoftBody;
        private Truss workingTruss;
        private bool autoApplyChanges = true;
        
        // Store target reference for play mode persistence
        private int targetSoftBodyInstanceID = -1;
        
        // Node editing
        private List<int> selectedNodes = new List<int>();
        private Vector3 nodeCreationPosition = Vector3.zero;
        private float nodeCreationMass = 0.5f;
        private bool isCreatingNode = false;
        
        // Beam editing
        private List<int> selectedBeams = new List<int>();
        private int beamStartNode = -1;
        private bool isCreatingBeam = false;
        private float defaultBeamCompliance = 0.01f;
        private float defaultBeamDamping = 0.3f;
        private float defaultMinStretchFactor = 0.5f;
        private float defaultMaxStretchFactor = 2.0f;
        
        // Beam presets
        [System.Serializable]
        public enum BeamPreset
        {
            Custom,
            Metal,
            Rubber
        }
        private BeamPreset selectedBeamPreset = BeamPreset.Custom;
        
        // Node transform
        private bool isTransformingNode = false;
        private int transformingNodeIndex = -1;
        private Vector3 nodeTransformOffset = Vector3.zero;
        
        // Multi-node transform
        private Vector3 selectionCenter = Vector3.zero;
        private bool showTransformHandles = false;
        
        // Face editing
        private List<int> selectedFaces = new List<int>();
        private List<int> faceCreationNodes = new List<int>();
        private bool isCreatingFace = false;
        
        // Mesh conversion
        private MeshFilter sourceMeshFilter;
        private float meshSimplificationFactor = 0.5f;
        private bool generateInternalBeams = true;
        private float internalBeamThreshold = 2.0f;
        private bool preserveBoundaryNodes = true;
        
        // Selection & manipulation
        private Vector3 boxSelectionStart;
        private Vector3 boxSelectionEnd;
        private bool isBoxSelecting = false;
        private float sphereSelectionRadius = 1.0f;
        private Vector3 sphereSelectionCenter;
        private bool isTransformingSelection = false;
        private Vector3 transformStartPosition;
        
        // Visualization settings
        private bool showNodeIndices = true;
        private bool showBeamIndices = false;
        private bool showFaceIndices = false;
        private float nodeDisplaySize = 0.15f;
        private float beamLineThickness = 2.0f;
        private Color nodeColor = Color.green;
        private Color selectedNodeColor = Color.yellow;
        private Color pinnedNodeColor = Color.red;
        private Color beamColor = Color.cyan;
        private Color selectedBeamColor = Color.magenta;
        private Color faceColor = new Color(0.2f, 0.8f, 1.0f, 0.3f);
        private Color selectedFaceColor = new Color(1.0f, 0.8f, 0.2f, 0.5f);
        
        // Optimization settings
        private bool optimizeNodePositions = true;
        private bool mergeCloseNodes = true;
        private float nodeDistanceThreshold = 0.1f;
        private bool removeRedundantBeams = true;
        private bool simplifyBeamNetwork = false;
        private float beamAngleThreshold = 10.0f;
        
        // UI state
        private Vector2 scrollPosition;
        private bool showVisualizationSettings = true;
        private bool showConversionSettings = true;
        private bool showOptimizationSettings = true;

        #endregion

        #region Unity Editor Lifecycle

        private void OnEnable()
        {
            SceneView.duringSceneGui += OnSceneGUI;
            LoadPreferences();

            // Restore target from Instance ID if available
            if (targetSoftBodyInstanceID != -1)
            {
#pragma warning disable CS0618 // Type or member is obsolete
                var obj = EditorUtility.InstanceIDToObject(targetSoftBodyInstanceID);
#pragma warning restore CS0618 // Type or member is obsolete
                if (obj is SoftBody softBody)
                {
                    SetTarget(softBody);
                }
            }

            // Add this line to ensure mouse move events are received for smooth slider dragging
            wantsMouseMove = true;
        }
        private void OnDisable()
        {
            SceneView.duringSceneGui -= OnSceneGUI;
            SavePreferences();
        }
        
        private void OnSelectionChange()
        {
            // Auto-detect target SoftBody from selection
            if (Selection.activeGameObject != null)
            {
                var softBody = Selection.activeGameObject.GetComponent<SoftBody>();
                if (softBody != null && softBody != targetSoftBody)
                {
                    SetTarget(softBody);
                }
            }
            Repaint();
        }
        
        #endregion
        
        #region Main GUI
        
        private void OnGUI()
        {
            scrollPosition = EditorGUILayout.BeginScrollView(scrollPosition);
            
            DrawHeader();
            DrawTargetSelection();
            DrawModeSelection();
            
            if (workingTruss != null)
            {
                switch (currentMode)
                {
                    case DesignerMode.Node:
                        DrawNodeEditPanel();
                        break;
                    case DesignerMode.Beam:
                        DrawBeamEditPanel();
                        break;
                    case DesignerMode.Face:
                        DrawFaceEditPanel();
                        break;
                    case DesignerMode.MeshConvert:
                        DrawMeshConvertPanel();
                        break;
                    case DesignerMode.Optimization:
                        DrawOptimizationPanel();
                        break;
                }
            }
            
            DrawVisualizationSettings();
            DrawStatistics();
            
            EditorGUILayout.EndScrollView();
        }
        
        private void DrawHeader()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Designer", EditorStyles.largeLabel);
            EditorGUILayout.LabelField("Interactive visual design tool for complex truss-based soft body physics structures", EditorStyles.wordWrappedMiniLabel);
            EditorGUILayout.EndVertical();
            EditorGUILayout.Space();
        }
        
        private void DrawTargetSelection()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Target Configuration", EditorStyles.boldLabel);
            
            EditorGUI.BeginChangeCheck();
            targetSoftBody = (SoftBody)EditorGUILayout.ObjectField("Target SoftBody", targetSoftBody, typeof(SoftBody), true);
            if (EditorGUI.EndChangeCheck())
            {
                SetTarget(targetSoftBody);
            }
            
            if (targetSoftBody != null)
            {
                EditorGUI.BeginChangeCheck();
                workingTruss = (Truss)EditorGUILayout.ObjectField("Working Truss", workingTruss, typeof(Truss), false);
                if (EditorGUI.EndChangeCheck() && workingTruss != null)
                {
                    ClearSelections();
                }
                
                autoApplyChanges = EditorGUILayout.Toggle("Auto Apply Changes", autoApplyChanges);
                
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Create New Truss"))
                {
                    CreateNewTruss();
                }
                if (GUILayout.Button("Apply to SoftBody"))
                {
                    ApplyTrussToTarget();
                }
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                EditorGUILayout.HelpBox("Select a GameObject with a SoftBody component to begin designing.", MessageType.Info);
            }
            
            EditorGUILayout.EndVertical();
            EditorGUILayout.Space();
        }
        
        private void DrawModeSelection()
        {
            if (workingTruss == null) return;
            
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Design Mode", EditorStyles.boldLabel);
            
            EditorGUI.BeginChangeCheck();
            currentMode = (DesignerMode)GUILayout.Toolbar((int)currentMode, System.Enum.GetNames(typeof(DesignerMode)));
            if (EditorGUI.EndChangeCheck())
            {
                OnModeChanged();
            }
            
            EditorGUILayout.EndVertical();
            EditorGUILayout.Space();
        }
        
        private void DrawNodeEditPanel()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Node Editing", EditorStyles.boldLabel);
            
            nodeCreationMass = EditorGUILayout.FloatField("New Node Mass", nodeCreationMass);
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button(isCreatingNode ? "Cancel Node Creation" : "Create Node"))
            {
                ToggleNodeCreation();
            }
            if (GUILayout.Button("Delete Selected"))
            {
                DeleteSelectedNodes();
            }
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Select All Nodes"))
            {
                SelectAllNodes();
            }
            if (GUILayout.Button("Clear Selection"))
            {
                selectedNodes.Clear();
                UpdateSelectionCenter();
                Repaint();
            }
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Transform Tools", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox("Hold 'T' + Click to transform nodes in Scene View", MessageType.Info);
            
            EditorGUILayout.BeginHorizontal();
            showTransformHandles = EditorGUILayout.Toggle("Show Transform Handles", showTransformHandles);
            if (selectedNodes.Count > 0)
            {
                EditorGUILayout.LabelField($"({selectedNodes.Count} selected)", EditorStyles.miniLabel);
            }
            EditorGUILayout.EndHorizontal();
            
            if (selectedNodes.Count > 0)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField($"Selected Nodes: {selectedNodes.Count}", EditorStyles.boldLabel);
                
                // Display selected node indices for individual selection
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                EditorGUILayout.LabelField("Individual Node Selection:");
                string nodeIndices = string.Join(", ", selectedNodes);
                EditorGUILayout.SelectableLabel(nodeIndices, EditorStyles.textField, GUILayout.Height(EditorGUIUtility.singleLineHeight));
                EditorGUILayout.EndVertical();
                
                if (selectedNodes.Count == 1)
                {
                    int nodeIndex = selectedNodes[0];
                    if (nodeIndex < workingTruss.NodePositions.Length)
                    {
                        EditorGUI.BeginChangeCheck();
                        Vector3 newPos = EditorGUILayout.Vector3Field("Position", workingTruss.NodePositions[nodeIndex]);
                        if (EditorGUI.EndChangeCheck())
                        {
                            SetNodePosition(nodeIndex, newPos);
                        }
                        
                        bool isPinned = workingTruss.PinnedNodes.Contains(nodeIndex);
                        EditorGUI.BeginChangeCheck();
                        bool newPinned = EditorGUILayout.Toggle("Pinned", isPinned);
                        if (EditorGUI.EndChangeCheck())
                        {
                            SetNodePinned(nodeIndex, newPinned);
                        }
                    }
                }
                else
                {
                    EditorGUILayout.BeginHorizontal();
                    if (GUILayout.Button("Pin Selected"))
                    {
                        PinSelectedNodes(true);
                    }
                    if (GUILayout.Button("Unpin Selected"))
                    {
                        PinSelectedNodes(false);
                    }
                    EditorGUILayout.EndHorizontal();
                }
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawBeamEditPanel()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Beam Editing", EditorStyles.boldLabel);
            
            // Beam Presets
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Beam Presets", EditorStyles.boldLabel);
            EditorGUI.BeginChangeCheck();
            selectedBeamPreset = (BeamPreset)EditorGUILayout.EnumPopup("Preset", selectedBeamPreset);
            if (EditorGUI.EndChangeCheck())
            {
                GetBeamPresetValues(selectedBeamPreset, out defaultBeamCompliance, out defaultBeamDamping, out defaultMinStretchFactor, out defaultMaxStretchFactor);
            }
            
            EditorGUILayout.Space();
            defaultBeamCompliance = EditorGUILayout.FloatField("Default Compliance", defaultBeamCompliance);
            defaultBeamDamping = EditorGUILayout.Slider("Default Damping", defaultBeamDamping, 0f, 1f);
            
            // Min and Max Stretch Factor controls for the Truss
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Truss Stretch Limits", EditorStyles.boldLabel);
            
            if (workingTruss != null)
            {
                EditorGUI.BeginChangeCheck();
                float currentMinStretch = EditorGUILayout.Slider("Min Stretch Factor", workingTruss.MinStretchFactor, 0.1f, 1.0f);
                float currentMaxStretch = EditorGUILayout.Slider("Max Stretch Factor", workingTruss.MaxStretchFactor, 1.0f, 10.0f);
                if (EditorGUI.EndChangeCheck())
                {
                    workingTruss.SetMinStretchFactor(currentMinStretch);
                    workingTruss.SetMaxStretchFactor(currentMaxStretch);
                    EditorUtility.SetDirty(workingTruss);
                    
                    if (autoApplyChanges)
                    {
                        ApplyTrussToTarget();
                    }
                }
            }
            else
            {
                defaultMinStretchFactor = EditorGUILayout.Slider("Default Min Stretch Factor", defaultMinStretchFactor, 0.1f, 1.0f);
                defaultMaxStretchFactor = EditorGUILayout.Slider("Default Max Stretch Factor", defaultMaxStretchFactor, 1.0f, 10.0f);
            }
            
            // Help text for stretch factors
            EditorGUILayout.HelpBox("Min Stretch Factor: How much beams can compress (0.5 = 50% compression)\nMax Stretch Factor: How much beams can extend (2.0 = 200% extension)\nThese apply to all beams in the truss.", MessageType.Info);
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button(isCreatingBeam ? "Cancel Beam Creation" : "Create Beam"))
            {
                ToggleBeamCreation();
            }
            if (GUILayout.Button("Auto Connect"))
            {
                AutoConnectSelectedNodes();
            }
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Delete Selected"))
            {
                DeleteSelectedBeams();
            }
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Select All Beams"))
            {
                SelectAllBeams();
            }
            if (GUILayout.Button("Clear Selection"))
            {
                selectedBeams.Clear();
                Repaint();
            }
            EditorGUILayout.EndHorizontal();
            
            if (selectedBeams.Count > 0)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField($"Selected Beams: {selectedBeams.Count}", EditorStyles.boldLabel);
                
                // Display selected beam indices for individual selection
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                EditorGUILayout.LabelField("Individual Beam Selection:");
                string beamIndices = string.Join(", ", selectedBeams);
                EditorGUILayout.SelectableLabel(beamIndices, EditorStyles.textField, GUILayout.Height(EditorGUIUtility.singleLineHeight));
                EditorGUILayout.EndVertical();
                
                if (selectedBeams.Count == 1)
                {
                    var beams = workingTruss.GetTrussBeams();
                    if (selectedBeams[0] < beams.Count)
                    {
                        var beam = beams[selectedBeams[0]];
                        
                        EditorGUI.BeginChangeCheck();
                        float newCompliance = EditorGUILayout.FloatField("Compliance", beam.compliance);
                        float newDamping = EditorGUILayout.Slider("Damping", beam.damping, 0f, 1f);
                        if (EditorGUI.EndChangeCheck())
                        {
                            SetBeamProperties(selectedBeams[0], newCompliance, newDamping);
                        }
                    }
                }
                else
                {
                    EditorGUI.BeginChangeCheck();
                    float batchCompliance = EditorGUILayout.FloatField("Batch Compliance", defaultBeamCompliance);
                    float batchDamping = EditorGUILayout.Slider("Batch Damping", defaultBeamDamping, 0f, 1f);
                    if (EditorGUI.EndChangeCheck())
                    {
                        ApplyBatchBeamProperties(batchCompliance, batchDamping);
                    }
                }
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawFaceEditPanel()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Face Editing", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button(isCreatingFace ? "Cancel Face Creation" : "Create Face"))
            {
                ToggleFaceCreation();
            }
            if (GUILayout.Button("Auto Generate"))
            {
                AutoGenerateFaces();
            }
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("From Selection"))
            {
                CreateFaceFromSelection();
            }
            if (GUILayout.Button("Delete Selected"))
            {
                DeleteSelectedFaces();
            }
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Select All Faces"))
            {
                SelectAllFaces();
            }
            if (GUILayout.Button("Clear Selection"))
            {
                selectedFaces.Clear();
                Repaint();
            }
            EditorGUILayout.EndHorizontal();
            
            if (selectedFaces.Count > 0)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField($"Selected Faces: {selectedFaces.Count}", EditorStyles.boldLabel);
            }
            
            if (isCreatingFace && faceCreationNodes.Count > 0)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField($"Face Creation: {faceCreationNodes.Count}/3 nodes selected");
                foreach (int nodeIndex in faceCreationNodes)
                {
                    EditorGUILayout.LabelField($"  Node {nodeIndex}");
                }
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawMeshConvertPanel()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Mesh to Truss Conversion", EditorStyles.boldLabel);
            
            sourceMeshFilter = (MeshFilter)EditorGUILayout.ObjectField("Source Mesh", sourceMeshFilter, typeof(MeshFilter), true);
            
            if (sourceMeshFilter != null)
            {
                showConversionSettings = EditorGUILayout.Foldout(showConversionSettings, "Conversion Settings");
                if (showConversionSettings)
                {
                    EditorGUI.indentLevel++;
                    meshSimplificationFactor = EditorGUILayout.Slider("Simplification Factor", meshSimplificationFactor, 0.1f, 1.0f);
                    generateInternalBeams = EditorGUILayout.Toggle("Generate Internal Beams", generateInternalBeams);
                    if (generateInternalBeams)
                    {
                        internalBeamThreshold = EditorGUILayout.FloatField("Internal Beam Threshold", internalBeamThreshold);
                    }
                    preserveBoundaryNodes = EditorGUILayout.Toggle("Preserve Boundary Nodes", preserveBoundaryNodes);
                    EditorGUI.indentLevel--;
                }
                
                EditorGUILayout.Space();
                if (GUILayout.Button("Convert Mesh to Truss", GUILayout.Height(30)))
                {
                    ConvertMeshToTruss();
                }
            }
            else
            {
                EditorGUILayout.HelpBox("Select a MeshFilter to convert its mesh into a truss structure.", MessageType.Info);
            }
            
            EditorGUILayout.EndVertical();
        }
        
        // DrawSelectionPanel method removed as Selection tab is no longer needed
        
        private void DrawOptimizationPanel()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Structure Optimization", EditorStyles.boldLabel);
            
            showOptimizationSettings = EditorGUILayout.Foldout(showOptimizationSettings, "Optimization Settings");
            if (showOptimizationSettings)
            {
                EditorGUI.indentLevel++;
                optimizeNodePositions = EditorGUILayout.Toggle("Optimize Node Positions", optimizeNodePositions);
                mergeCloseNodes = EditorGUILayout.Toggle("Merge Close Nodes", mergeCloseNodes);
                if (mergeCloseNodes)
                {
                    nodeDistanceThreshold = EditorGUILayout.FloatField("Distance Threshold", nodeDistanceThreshold);
                }
                removeRedundantBeams = EditorGUILayout.Toggle("Remove Redundant Beams", removeRedundantBeams);
                simplifyBeamNetwork = EditorGUILayout.Toggle("Simplify Beam Network", simplifyBeamNetwork);
                if (simplifyBeamNetwork)
                {
                    beamAngleThreshold = EditorGUILayout.FloatField("Angle Threshold", beamAngleThreshold);
                }
                EditorGUI.indentLevel--;
            }
            
            EditorGUILayout.Space();
            if (GUILayout.Button("Optimize Structure", GUILayout.Height(30)))
            {
                OptimizeStructure();
            }
            
            EditorGUILayout.Space();
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Center Structure"))
            {
                CenterStructure();
            }
            if (GUILayout.Button("Scale Structure"))
            {
                ScaleStructureDialog();
            }
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawVisualizationSettings()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            showVisualizationSettings = EditorGUILayout.Foldout(showVisualizationSettings, "Visualization Settings");
            if (showVisualizationSettings)
            {
                EditorGUI.indentLevel++;
                
                showNodeIndices = EditorGUILayout.Toggle("Show Node Indices", showNodeIndices);
                showBeamIndices = EditorGUILayout.Toggle("Show Beam Indices", showBeamIndices);
                showFaceIndices = EditorGUILayout.Toggle("Show Face Indices", showFaceIndices);
                
                EditorGUILayout.Space();
                nodeDisplaySize = EditorGUILayout.Slider("Node Display Size", nodeDisplaySize, 0.05f, 0.5f);
                beamLineThickness = EditorGUILayout.Slider("Beam Line Thickness", beamLineThickness, 0.5f, 10.0f);
                
                EditorGUILayout.Space();
                nodeColor = EditorGUILayout.ColorField("Node Color", nodeColor);
                selectedNodeColor = EditorGUILayout.ColorField("Selected Node Color", selectedNodeColor);
                pinnedNodeColor = EditorGUILayout.ColorField("Pinned Node Color", pinnedNodeColor);
                beamColor = EditorGUILayout.ColorField("Beam Color", beamColor);
                selectedBeamColor = EditorGUILayout.ColorField("Selected Beam Color", selectedBeamColor);
                faceColor = EditorGUILayout.ColorField("Face Color", faceColor);
                selectedFaceColor = EditorGUILayout.ColorField("Selected Face Color", selectedFaceColor);
                
                EditorGUI.indentLevel--;
            }
            EditorGUILayout.EndVertical();
            EditorGUILayout.Space();
        }
        
        private void DrawStatistics()
        {
            if (workingTruss == null) return;
            
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Structure Statistics", EditorStyles.boldLabel);
            
            EditorGUILayout.LabelField($"Nodes: {workingTruss.NodePositions?.Length ?? 0}");
            EditorGUILayout.LabelField($"Beams: {workingTruss.GetTrussBeams()?.Count ?? 0}");
            EditorGUILayout.LabelField($"Faces: {workingTruss.GetTrussFaces()?.Count ?? 0}");
            EditorGUILayout.LabelField($"Pinned Nodes: {workingTruss.PinnedNodes?.Count ?? 0}");
            EditorGUILayout.LabelField($"Selected Nodes: {selectedNodes.Count}");
            EditorGUILayout.LabelField($"Selected Beams: {selectedBeams.Count}");
            EditorGUILayout.LabelField($"Selected Faces: {selectedFaces.Count}");
            
            EditorGUILayout.EndVertical();
        }
        
        #endregion
        
        #region Scene View Integration
        
        private void OnSceneGUI(SceneView sceneView)
        {
            if (workingTruss == null || targetSoftBody == null) return;
            
            HandleSceneInput();
            DrawSceneVisualization();
            
            // Force repaint to ensure visualization always updates
            sceneView.Repaint();
        }
        
        private void HandleSceneInput()
        {
            Event e = Event.current;
            int controlID = GUIUtility.GetControlID(FocusType.Passive);
            
            switch (e.type)
            {
                case EventType.MouseDown:
                    if (e.button == 0)
                    {
                        HandleMouseDown(e);
                        GUIUtility.hotControl = controlID;
                        e.Use();
                    }
                    break;
                    
                case EventType.MouseDrag:
                    if (GUIUtility.hotControl == controlID)
                    {
                        HandleMouseDrag(e);
                        e.Use();
                    }
                    break;
                    
                case EventType.MouseUp:
                    if (GUIUtility.hotControl == controlID)
                    {
                        HandleMouseUp(e);
                        GUIUtility.hotControl = 0;
                        e.Use();
                    }
                    break;
                    
                case EventType.KeyDown:
                    HandleKeyDown(e);
                    break;
            }
        }
        
        private void HandleMouseDown(Event e)
        {
            Vector3 worldPosition = GetWorldPositionFromMouse(e.mousePosition);
            
            switch (currentMode)
            {
                case DesignerMode.Node:
                    if (isCreatingNode)
                    {
                        CreateNodeAtPosition(worldPosition);
                    }
                    else if (e.keyCode == KeyCode.T || (e.modifiers & EventModifiers.Shift) != 0)
                    {
                        // Transform mode - find closest node and start transforming
                        int nodeIndex = FindClosestNode(worldPosition);
                        if (nodeIndex >= 0)
                        {
                            StartNodeTransform(nodeIndex);
                            selectedNodes.Clear();
                            selectedNodes.Add(nodeIndex);
                            Repaint();
                        }
                    }
                    else
                    {
                        SelectNodeAtPosition(worldPosition, e.control);
                    }
                    break;
                    
                case DesignerMode.Beam:
                    if (isCreatingBeam)
                    {
                        HandleBeamCreation(worldPosition);
                    }
                    else
                    {
                        SelectBeamAtPosition(worldPosition, e.control);
                    }
                    break;
                    
                case DesignerMode.Face:
                    if (isCreatingFace)
                    {
                        HandleFaceCreation(worldPosition);
                    }
                    else
                    {
                        SelectFaceAtPosition(worldPosition, e.control);
                    }
                    break;
            }
        }
        
        private void HandleMouseDrag(Event e)
        {
            if (isTransformingNode)
            {
                Vector3 mouseWorldPosition = GetWorldPositionFromMouse(e.mousePosition);
                UpdateNodeTransform(mouseWorldPosition);
            }
            else if (selectionMode == SelectionMode.Box && isBoxSelecting)
            {
                boxSelectionEnd = GetWorldPositionFromMouse(e.mousePosition);
                SceneView.RepaintAll();
            }
            else if (isTransformingSelection && selectedNodes.Count > 0)
            {
                Vector3 currentPosition = GetWorldPositionFromMouse(e.mousePosition);
                Vector3 delta = currentPosition - transformStartPosition;
                TransformSelectedNodes(delta);
                transformStartPosition = currentPosition;
            }
        }
        
        private void HandleMouseUp(Event e)
        {
            if (isTransformingNode)
            {
                EndNodeTransform();
            }
            else if (isBoxSelecting)
            {
                PerformBoxSelection();
                isBoxSelecting = false;
            }
            isTransformingSelection = false;
        }
        
        private void HandleKeyDown(Event e)
        {
            switch (e.keyCode)
            {
                case KeyCode.Delete:
                    DeleteSelectedElements();
                    e.Use();
                    break;
                case KeyCode.Escape:
                    CancelCurrentOperation();
                    e.Use();
                    break;
                case KeyCode.A:
                    if (e.control)
                    {
                        SelectAllNodes();
                        e.Use();
                    }
                    break;
            }
        }
        
        #endregion
        
        #region Core Operations
        
        private void SetTarget(SoftBody softBody)
        {
            targetSoftBody = softBody;
            if (targetSoftBody != null)
            {
                // Store instance ID for play mode persistence
                targetSoftBodyInstanceID = targetSoftBody.GetInstanceID();
                
                workingTruss = targetSoftBody.GetTrussAsset();
                if (workingTruss == null)
                {
                    CreateNewTruss();
                }
            }
            else
            {
                targetSoftBodyInstanceID = -1;
            }
            ClearSelections();
            SceneView.RepaintAll();
        }
        
        private void CreateNewTruss()
        {
            string path = EditorUtility.SaveFilePanelInProject(
                "Create New Truss Asset",
                "NewTruss",
                "asset",
                "Create a new truss asset for the soft body structure"
            );
            
            if (!string.IsNullOrEmpty(path))
            {
                workingTruss = CreateInstance<Truss>();
                AssetDatabase.CreateAsset(workingTruss, path);
                AssetDatabase.SaveAssets();
                
                // Initialize with a basic structure if targeting a mesh
                if (targetSoftBody != null)
                {
                    var meshFilter = targetSoftBody.GetComponent<MeshFilter>();
                    if (meshFilter != null && meshFilter.sharedMesh != null)
                    {
                        InitializeTrussFromMesh(meshFilter.sharedMesh);
                    }
                }
                
                EditorUtility.SetDirty(workingTruss);
            }
        }
        
        private void ApplyTrussToTarget()
        {
            if (targetSoftBody != null && workingTruss != null)
            {
                targetSoftBody.ApplyTrussAsset(workingTruss);
                EditorUtility.SetDirty(targetSoftBody);
            }
        }
        
        private void OnModeChanged()
        {
            CancelCurrentOperation();
            ClearSelections();
        }
        
        private void CancelCurrentOperation()
        {
            isCreatingNode = false;
            isCreatingBeam = false;
            isCreatingFace = false;
            isBoxSelecting = false;
            isTransformingSelection = false;
            beamStartNode = -1;
            faceCreationNodes.Clear();
        }
        
        private void ClearSelections()
        {
            selectedNodes.Clear();
            selectedBeams.Clear();
            selectedFaces.Clear();
            SceneView.RepaintAll();
        }
        
        #endregion
        
        #region Node Operations
        
        private void ToggleNodeCreation()
        {
            isCreatingNode = !isCreatingNode;
            if (!isCreatingNode)
            {
                SceneView.RepaintAll();
            }
        }

        public void CreateNodeAtPosition(Vector3 worldPosition)
        {
            Vector3 localPosition = targetSoftBody.transform.InverseTransformPoint(worldPosition);

            var nodePositions = workingTruss.NodePositions?.ToList() ?? new List<Vector3>();
            nodePositions.Add(localPosition);
            workingTruss.SetNodePositions(nodePositions.ToArray());

            List<float> updatedMasses = workingTruss.NodeMasses?.ToList() ?? new List<float>();
            updatedMasses.Add(Mathf.Max(0.1f, nodeCreationMass));
            workingTruss.SetNodeMasses(updatedMasses);

            EditorUtility.SetDirty(workingTruss);

            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }

            isCreatingNode = false;
            SceneView.RepaintAll();
        }
        
        private void SelectNodeAtPosition(Vector3 worldPosition, bool addToSelection)
        {
            int closestNode = FindClosestNode(worldPosition);
            if (closestNode >= 0)
            {
                if (!addToSelection)
                {
                    selectedNodes.Clear();
                }
                
                if (selectedNodes.Contains(closestNode))
                {
                    selectedNodes.Remove(closestNode);
                }
                else
                {
                    selectedNodes.Add(closestNode);
                }
                
                UpdateSelectionCenter(); // Update center when selection changes
                SceneView.RepaintAll();
            }
        }
        
        private void DeleteSelectedNodes()
        {
            if (selectedNodes.Count == 0) return;
            
            var nodePositions = workingTruss.NodePositions.ToList();
            var beams = workingTruss.GetTrussBeams();
            var faces = workingTruss.GetTrussFaces();
            var pinnedNodes = workingTruss.PinnedNodes;
            
            // Sort in descending order to avoid index shifting issues
            selectedNodes.Sort((a, b) => b.CompareTo(a));
            
            foreach (int nodeIndex in selectedNodes)
            {
                if (nodeIndex < nodePositions.Count)
                {
                    nodePositions.RemoveAt(nodeIndex);
                    
                    // Remove beams connected to this node
                    beams.RemoveAll(b => b.nodeA == nodeIndex || b.nodeB == nodeIndex);
                    
                    // Update beam indices
                    for (int i = 0; i < beams.Count; i++)
                    {
                        var beam = beams[i];
                        if (beam.nodeA > nodeIndex) beam.nodeA--;
                        if (beam.nodeB > nodeIndex) beam.nodeB--;
                    }
                    
                    // Remove faces containing this node
                    faces.RemoveAll(f => f.nodeA == nodeIndex || f.nodeB == nodeIndex || f.nodeC == nodeIndex);
                    
                    // Update face indices
                    for (int i = 0; i < faces.Count; i++)
                    {
                        var face = faces[i];
                        if (face.nodeA > nodeIndex) face.nodeA--;
                        if (face.nodeB > nodeIndex) face.nodeB--;
                        if (face.nodeC > nodeIndex) face.nodeC--;
                    }
                    
                    // Update pinned nodes
                    pinnedNodes.RemoveAll(p => p == nodeIndex);
                    for (int i = 0; i < pinnedNodes.Count; i++)
                    {
                        if (pinnedNodes[i] > nodeIndex)
                            pinnedNodes[i]--;
                    }
                }
            }
            
            workingTruss.SetNodePositions(nodePositions.ToArray());
            workingTruss.SetBeams(beams);
            workingTruss.SetPinnedNodes(pinnedNodes);
            
            selectedNodes.Clear();
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            SceneView.RepaintAll();
        }
        
        private void SetNodePosition(int nodeIndex, Vector3 newPosition)
        {
            var nodePositions = workingTruss.NodePositions.ToArray();
            if (nodeIndex < nodePositions.Length)
            {
                nodePositions[nodeIndex] = newPosition;
                workingTruss.SetNodePositions(nodePositions);
                EditorUtility.SetDirty(workingTruss);
                
                if (autoApplyChanges)
                {
                    ApplyTrussToTarget();
                }
            }
        }
        
        private void SetNodePinned(int nodeIndex, bool pinned)
        {
            var pinnedNodes = workingTruss.PinnedNodes.ToList();
            
            if (pinned && !pinnedNodes.Contains(nodeIndex))
            {
                pinnedNodes.Add(nodeIndex);
            }
            else if (!pinned && pinnedNodes.Contains(nodeIndex))
            {
                pinnedNodes.Remove(nodeIndex);
            }
            
            workingTruss.SetPinnedNodes(pinnedNodes);
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
        }
        
        private void PinSelectedNodes(bool pinned)
        {
            var pinnedNodes = workingTruss.PinnedNodes.ToList();
            
            foreach (int nodeIndex in selectedNodes)
            {
                if (pinned && !pinnedNodes.Contains(nodeIndex))
                {
                    pinnedNodes.Add(nodeIndex);
                }
                else if (!pinned && pinnedNodes.Contains(nodeIndex))
                {
                    pinnedNodes.Remove(nodeIndex);
                }
            }
            
            workingTruss.SetPinnedNodes(pinnedNodes);
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
        }
        
        #endregion
        
        #region Beam Operations
        
        private void ToggleBeamCreation()
        {
            isCreatingBeam = !isCreatingBeam;
            beamStartNode = -1;
            SceneView.RepaintAll();
        }
        
        private void HandleBeamCreation(Vector3 worldPosition)
        {
            int nodeIndex = FindClosestNode(worldPosition);
            if (nodeIndex < 0) return;
            
            if (beamStartNode < 0)
            {
                beamStartNode = nodeIndex;
            }
            else if (beamStartNode != nodeIndex)
            {
                CreateBeam(beamStartNode, nodeIndex);
                beamStartNode = -1;
                isCreatingBeam = false;
            }
        }
        
        private void CreateBeam(int nodeA, int nodeB)
        {
            var beams = workingTruss.GetTrussBeams();
            
            // Check if beam already exists
            if (beams.Any(b => (b.nodeA == nodeA && b.nodeB == nodeB) || (b.nodeA == nodeB && b.nodeB == nodeA)))
            {
                Debug.Log("Beam already exists between these nodes.");
                return;
            }
            
            Vector3 posA = workingTruss.NodePositions[nodeA];
            Vector3 posB = workingTruss.NodePositions[nodeB];
            float restLength = Vector3.Distance(posA, posB);
            
            var newBeam = new Beam(nodeA, nodeB, defaultBeamCompliance, defaultBeamDamping, restLength);
            beams.Add(newBeam);
            
            workingTruss.SetBeams(beams);
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            SceneView.RepaintAll();
        }
        
        private void AutoConnectSelectedNodes()
        {
            if (selectedNodes.Count < 2) return;
            
            var beams = workingTruss.GetTrussBeams();
            int initialBeamCount = beams.Count;
            
            for (int i = 0; i < selectedNodes.Count; i++)
            {
                for (int j = i + 1; j < selectedNodes.Count; j++)
                {
                    int nodeA = selectedNodes[i];
                    int nodeB = selectedNodes[j];
                    
                    // Check if beam already exists
                    if (!beams.Any(b => (b.nodeA == nodeA && b.nodeB == nodeB) || (b.nodeA == nodeB && b.nodeB == nodeA)))
                    {
                        Vector3 posA = workingTruss.NodePositions[nodeA];
                        Vector3 posB = workingTruss.NodePositions[nodeB];
                        float restLength = Vector3.Distance(posA, posB);
                        
                        var newBeam = new Beam(nodeA, nodeB, defaultBeamCompliance, defaultBeamDamping, restLength);
                        beams.Add(newBeam);
                    }
                }
            }
            
            workingTruss.SetBeams(beams);
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            Debug.Log($"Created {beams.Count - initialBeamCount} new beams between selected nodes.");
            SceneView.RepaintAll();
        }
        
        private void SelectBeamAtPosition(Vector3 worldPosition, bool addToSelection)
        {
            int closestBeam = FindClosestBeam(worldPosition);
            if (closestBeam >= 0)
            {
                if (!addToSelection)
                {
                    selectedBeams.Clear();
                }
                
                if (selectedBeams.Contains(closestBeam))
                {
                    selectedBeams.Remove(closestBeam);
                }
                else
                {
                    selectedBeams.Add(closestBeam);
                }
                
                SceneView.RepaintAll();
            }
        }
        
        private void DeleteSelectedBeams()
        {
            if (selectedBeams.Count == 0) return;
            
            var beams = workingTruss.GetTrussBeams();
            
            // Sort in descending order to avoid index shifting issues
            selectedBeams.Sort((a, b) => b.CompareTo(a));
            
            foreach (int beamIndex in selectedBeams)
            {
                if (beamIndex < beams.Count)
                {
                    beams.RemoveAt(beamIndex);
                }
            }
            
            workingTruss.SetBeams(beams);
            selectedBeams.Clear();
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            SceneView.RepaintAll();
        }
        
        private void SetBeamProperties(int beamIndex, float compliance, float damping)
        {
            var beams = workingTruss.GetTrussBeams();
            if (beamIndex < beams.Count)
            {
                var beam = beams[beamIndex];
                beam.compliance = compliance;
                beam.damping = damping;
                beams[beamIndex] = beam;
                
                workingTruss.SetBeams(beams);
                EditorUtility.SetDirty(workingTruss);
                
                if (autoApplyChanges)
                {
                    ApplyTrussToTarget();
                }
            }
        }
        
        private void ApplyBatchBeamProperties(float compliance, float damping)
        {
            var beams = workingTruss.GetTrussBeams();
            
            foreach (int beamIndex in selectedBeams)
            {
                if (beamIndex < beams.Count)
                {
                    var beam = beams[beamIndex];
                    beam.compliance = compliance;
                    beam.damping = damping;
                    beams[beamIndex] = beam;
                }
            }
            
            workingTruss.SetBeams(beams);
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
        }
        
        #endregion
        
        #region Face Operations
        
        private void ToggleFaceCreation()
        {
            isCreatingFace = !isCreatingFace;
            faceCreationNodes.Clear();
            SceneView.RepaintAll();
        }
        
        private void HandleFaceCreation(Vector3 worldPosition)
        {
            int nodeIndex = FindClosestNode(worldPosition);
            if (nodeIndex < 0) return;
            
            if (!faceCreationNodes.Contains(nodeIndex))
            {
                faceCreationNodes.Add(nodeIndex);
                
                if (faceCreationNodes.Count >= 3)
                {
                    CreateFace(faceCreationNodes[0], faceCreationNodes[1], faceCreationNodes[2]);
                    faceCreationNodes.Clear();
                    isCreatingFace = false;
                }
            }
        }
        
        private void CreateFace(int nodeA, int nodeB, int nodeC)
        {
            var faces = workingTruss.GetTrussFaces();
            
            var newFace = new Face(nodeA, nodeB, nodeC);
            faces.Add(newFace);
            
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            SceneView.RepaintAll();
        }
        
        private void CreateFaceFromSelection()
        {
            if (selectedNodes.Count >= 3)
            {
                CreateFace(selectedNodes[0], selectedNodes[1], selectedNodes[2]);
            }
        }
        
        private void AutoGenerateFaces()
        {
            // Simple face generation using Delaunay triangulation approach
            if (workingTruss.NodePositions == null || workingTruss.NodePositions.Length < 3) return;
            
            var faces = new List<Face>();
            var nodes = workingTruss.NodePositions;
            
            // Create faces from convex hull first, then add internal faces
            for (int i = 0; i < nodes.Length - 2; i++)
            {
                for (int j = i + 1; j < nodes.Length - 1; j++)
                {
                    for (int k = j + 1; k < nodes.Length; k++)
                    {
                        // Check if this would be a valid face (not too large)
                        Vector3 a = nodes[i];
                        Vector3 b = nodes[j];
                        Vector3 c = nodes[k];
                        
                        float maxEdgeLength = Mathf.Max(
                            Vector3.Distance(a, b),
                            Vector3.Distance(b, c),
                            Vector3.Distance(c, a)
                        );
                        
                        if (maxEdgeLength < internalBeamThreshold)
                        {
                            var newFace = new Face(i, j, k);
                            faces.Add(newFace);
                        }
                    }
                }
            }
            
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            Debug.Log($"Generated {faces.Count} faces.");
            SceneView.RepaintAll();
        }
        
        private void SelectFaceAtPosition(Vector3 worldPosition, bool addToSelection)
        {
            int closestFace = FindClosestFace(worldPosition);
            if (closestFace >= 0)
            {
                if (!addToSelection)
                {
                    selectedFaces.Clear();
                }
                
                if (selectedFaces.Contains(closestFace))
                {
                    selectedFaces.Remove(closestFace);
                }
                else
                {
                    selectedFaces.Add(closestFace);
                }
                
                SceneView.RepaintAll();
            }
        }
        
        private void DeleteSelectedFaces()
        {
            if (selectedFaces.Count == 0) return;
            
            var faces = workingTruss.GetTrussFaces();
            
            // Sort in descending order to avoid index shifting issues
            selectedFaces.Sort((a, b) => b.CompareTo(a));
            
            foreach (int faceIndex in selectedFaces)
            {
                if (faceIndex < faces.Count)
                {
                    faces.RemoveAt(faceIndex);
                }
            }
            
            selectedFaces.Clear();
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            SceneView.RepaintAll();
        }
        #endregion
        
        #region Mesh Conversion Methods
        
        private void ConvertMeshToTruss()
        {
            if (sourceMeshFilter == null || sourceMeshFilter.sharedMesh == null) return;
            
            Mesh mesh = sourceMeshFilter.sharedMesh;
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            
            // Simplify mesh if needed
            if (meshSimplificationFactor < 1.0f)
            {
                SimplifyMesh(ref vertices, ref triangles, meshSimplificationFactor);
            }
            
            // Convert to local space of target SoftBody
            Transform sourceTransform = sourceMeshFilter.transform;
            Transform targetTransform = targetSoftBody.transform;
            
            for (int i = 0; i < vertices.Length; i++)
            {
                Vector3 worldPos = sourceTransform.TransformPoint(vertices[i]);
                vertices[i] = targetTransform.InverseTransformPoint(worldPos);
            }
            
            // Set nodes
            workingTruss.SetNodePositions(vertices);
            workingTruss.SetUniformNodeMass(nodeCreationMass);
            
            // Generate beams from mesh edges
            var beams = GenerateBeamsFromMesh(vertices, triangles);
            
            if (generateInternalBeams)
            {
                beams.AddRange(GenerateInternalBeams(vertices, internalBeamThreshold));
            }
            
            workingTruss.SetBeams(beams);
            
            // Set stretch factors for the truss
            workingTruss.SetMinStretchFactor(defaultMinStretchFactor);
            workingTruss.SetMaxStretchFactor(defaultMaxStretchFactor);
            
            // Generate faces from triangles
            var faces = new List<Face>();
            for (int i = 0; i < triangles.Length; i += 3)
            {
                faces.Add(new Face(triangles[i], triangles[i + 1], triangles[i + 2]));
            }
            
            if (preserveBoundaryNodes)
            {
                var boundaryNodes = FindBoundaryNodes(vertices, triangles);
                workingTruss.SetPinnedNodes(boundaryNodes);
            }
            
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            Debug.Log($"Converted mesh to truss: {vertices.Length} nodes, {beams.Count} beams, {faces.Count} faces");
            SceneView.RepaintAll();
        }
        
        private void InitializeTrussFromMesh(Mesh mesh)
        {
            Vector3[] vertices = mesh.vertices;
            
            // Simplify for basic initialization
            if (vertices.Length > 100)
            {
                int[] triangles = mesh.triangles;
                SimplifyMesh(ref vertices, ref triangles, 0.3f);
            }
            
            workingTruss.SetNodePositions(vertices);
            workingTruss.SetUniformNodeMass(0.5f);
            
            // Create basic beam structure
            var beams = GenerateBasicBeamStructure(vertices);
            workingTruss.SetBeams(beams);
            
            // Set default stretch factors for the truss
            workingTruss.SetMinStretchFactor(defaultMinStretchFactor);
            workingTruss.SetMaxStretchFactor(defaultMaxStretchFactor);
        }
        
        private void SimplifyMesh(ref Vector3[] vertices, ref int[] triangles, float factor)
        {
            // Simple vertex reduction based on distance threshold
            float threshold = CalculateAverageEdgeLength(vertices, triangles) * (1.0f - factor);
            
            var simplifiedVertices = new List<Vector3>();
            var vertexMapping = new Dictionary<int, int>();
            
            for (int i = 0; i < vertices.Length; i++)
            {
                bool tooClose = false;
                for (int j = 0; j < simplifiedVertices.Count; j++)
                {
                    if (Vector3.Distance(vertices[i], simplifiedVertices[j]) < threshold)
                    {
                        vertexMapping[i] = j;
                        tooClose = true;
                        break;
                    }
                }
                
                if (!tooClose)
                {
                    vertexMapping[i] = simplifiedVertices.Count;
                    simplifiedVertices.Add(vertices[i]);
                }
            }
            
            // Update triangles with new indices
            var simplifiedTriangles = new List<int>();
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int a = vertexMapping[triangles[i]];
                int b = vertexMapping[triangles[i + 1]];
                int c = vertexMapping[triangles[i + 2]];
                
                // Only add if it forms a valid triangle
                if (a != b && b != c && c != a)
                {
                    simplifiedTriangles.Add(a);
                    simplifiedTriangles.Add(b);
                    simplifiedTriangles.Add(c);
                }
            }
            
            vertices = simplifiedVertices.ToArray();
            triangles = simplifiedTriangles.ToArray();
        }
        
        private List<Beam> GenerateBeamsFromMesh(Vector3[] vertices, int[] triangles)
        {
            var beams = new List<Beam>();
            var edges = new HashSet<(int, int)>();
            
            // Extract edges from triangles
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int a = triangles[i];
                int b = triangles[i + 1];
                int c = triangles[i + 2];
                
                edges.Add((Mathf.Min(a, b), Mathf.Max(a, b)));
                edges.Add((Mathf.Min(b, c), Mathf.Max(b, c)));
                edges.Add((Mathf.Min(c, a), Mathf.Max(c, a)));
            }
            
            // Create beams from edges
            foreach (var edge in edges)
            {
                Vector3 posA = vertices[edge.Item1];
                Vector3 posB = vertices[edge.Item2];
                float restLength = Vector3.Distance(posA, posB);
                
                var beam = new Beam(edge.Item1, edge.Item2, defaultBeamCompliance, defaultBeamDamping, restLength);
                beams.Add(beam);
            }
            
            return beams;
        }
        
        private List<Beam> GenerateInternalBeams(Vector3[] vertices, float maxDistance)
        {
            var beams = new List<Beam>();
            
            for (int i = 0; i < vertices.Length; i++)
            {
                for (int j = i + 1; j < vertices.Length; j++)
                {
                    float distance = Vector3.Distance(vertices[i], vertices[j]);
                    if (distance <= maxDistance)
                    {
                        var beam = new Beam(i, j, defaultBeamCompliance, defaultBeamDamping, distance);
                        beams.Add(beam);
                    }
                }
            }
            
            return beams;
        }
        
        private List<Beam> GenerateBasicBeamStructure(Vector3[] vertices)
        {
            var beams = new List<Beam>();
            float avgDistance = CalculateAverageVertexDistance(vertices);
            float connectionThreshold = avgDistance * 1.5f;
            
            for (int i = 0; i < vertices.Length; i++)
            {
                for (int j = i + 1; j < vertices.Length; j++)
                {
                    float distance = Vector3.Distance(vertices[i], vertices[j]);
                    if (distance <= connectionThreshold)
                    {
                        var beam = new Beam(i, j, defaultBeamCompliance, defaultBeamDamping, distance);
                        beams.Add(beam);
                    }
                }
            }
            
            return beams;
        }
        
        private List<int> FindBoundaryNodes(Vector3[] vertices, int[] triangles)
        {
            var edgeCount = new Dictionary<(int, int), int>();
            
            // Count occurrences of each edge
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int a = triangles[i];
                int b = triangles[i + 1];
                int c = triangles[i + 2];
                
                var edges = new[]
                {
                    (Mathf.Min(a, b), Mathf.Max(a, b)),
                    (Mathf.Min(b, c), Mathf.Max(b, c)),
                    (Mathf.Min(c, a), Mathf.Max(c, a))
                };
                
                foreach (var edge in edges)
                {
                    edgeCount[edge] = edgeCount.GetValueOrDefault(edge, 0) + 1;
                }
            }
            
            // Find boundary edges (appear only once)
            var boundaryNodes = new HashSet<int>();
            foreach (var kvp in edgeCount)
            {
                if (kvp.Value == 1)
                {
                    boundaryNodes.Add(kvp.Key.Item1);
                    boundaryNodes.Add(kvp.Key.Item2);
                }
            }
            
            return boundaryNodes.ToList();
        }
        
        private float CalculateAverageEdgeLength(Vector3[] vertices, int[] triangles)
        {
            float totalLength = 0f;
            int edgeCount = 0;
            
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 a = vertices[triangles[i]];
                Vector3 b = vertices[triangles[i + 1]];
                Vector3 c = vertices[triangles[i + 2]];
                
                totalLength += Vector3.Distance(a, b);
                totalLength += Vector3.Distance(b, c);
                totalLength += Vector3.Distance(c, a);
                edgeCount += 3;
            }
            
            return edgeCount > 0 ? totalLength / edgeCount : 1f;
        }
        
        #endregion
        
        #region Selection Operations
        
        private void HandleSelectionModeClick(Vector3 worldPosition, Event e)
        {
            switch (selectionMode)
            {
                case SelectionMode.Single:
                case SelectionMode.Multi:
                    SelectNodeAtPosition(worldPosition, selectionMode == SelectionMode.Multi || e.control);
                    break;
                    
                case SelectionMode.Box:
                    if (!isBoxSelecting)
                    {
                        boxSelectionStart = worldPosition;
                        boxSelectionEnd = worldPosition;
                        isBoxSelecting = true;
                    }
                    break;
                    
                case SelectionMode.Sphere:
                    sphereSelectionCenter = worldPosition;
                    PerformSphereSelection();
                    break;
            }
        }
        
        private void PerformBoxSelection()
        {
            if (workingTruss.NodePositions == null) return;
            
            var bounds = new Bounds();
            bounds.SetMinMax(
                Vector3.Min(boxSelectionStart, boxSelectionEnd),
                Vector3.Max(boxSelectionStart, boxSelectionEnd)
            );
            
            selectedNodes.Clear();
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 worldPos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                if (bounds.Contains(worldPos))
                {
                    selectedNodes.Add(i);
                }
            }
            
            SceneView.RepaintAll();
        }
        
        private void PerformSphereSelection()
        {
            if (workingTruss.NodePositions == null) return;
            
            selectedNodes.Clear();
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 worldPos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                if (Vector3.Distance(worldPos, sphereSelectionCenter) <= sphereSelectionRadius)
                {
                    selectedNodes.Add(i);
                }
            }
            
            SceneView.RepaintAll();
        }
        
        private void SelectAllNodes()
        {
            if (workingTruss.NodePositions == null) return;
            
            selectedNodes.Clear();
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                selectedNodes.Add(i);
            }
            
            UpdateSelectionCenter(); // Update center when all nodes are selected
            SceneView.RepaintAll();
        }
        
        private void InvertNodeSelection()
        {
            if (workingTruss.NodePositions == null) return;
            
            var newSelection = new List<int>();
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                if (!selectedNodes.Contains(i))
                {
                    newSelection.Add(i);
                }
            }
            
            selectedNodes = newSelection;
            SceneView.RepaintAll();
        }
        
        private void SelectConnectedNodes()
        {
            if (selectedNodes.Count == 0 || workingTruss.NodePositions == null) return;
            
            var beams = workingTruss.GetTrussBeams();
            var connectedNodes = new HashSet<int>(selectedNodes);
            
            bool foundNew = true;
            while (foundNew)
            {
                foundNew = false;
                foreach (var beam in beams)
                {
                    if (connectedNodes.Contains(beam.nodeA) && !connectedNodes.Contains(beam.nodeB))
                    {
                        connectedNodes.Add(beam.nodeB);
                        foundNew = true;
                    }
                    else if (connectedNodes.Contains(beam.nodeB) && !connectedNodes.Contains(beam.nodeA))
                    {
                        connectedNodes.Add(beam.nodeA);
                        foundNew = true;
                    }
                }
            }
            
            selectedNodes = connectedNodes.ToList();
            SceneView.RepaintAll();
        }
        
        private void TransformSelectedNodes(Vector3 delta)
        {
            if (selectedNodes.Count == 0) return;
            
            var nodePositions = workingTruss.NodePositions.ToArray();
            Vector3 localDelta = targetSoftBody.transform.InverseTransformVector(delta);
            
            foreach (int nodeIndex in selectedNodes)
            {
                if (nodeIndex < nodePositions.Length)
                {
                    nodePositions[nodeIndex] += localDelta;
                }
            }
            
            workingTruss.SetNodePositions(nodePositions);
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
        }
        
        private void DeleteSelectedElements()
        {
            switch (currentMode)
            {
                case DesignerMode.Node:
                    DeleteSelectedNodes();
                    break;
                case DesignerMode.Beam:
                    DeleteSelectedBeams();
                    break;
                case DesignerMode.Face:
                    DeleteSelectedFaces();
                    break;
            }
        }
        
        #endregion
        
        #region Optimization Methods
        
        private void OptimizeStructure()
        {
            if (workingTruss.NodePositions == null) return;
            
            if (mergeCloseNodes)
            {
                MergeCloseNodes();
            }
            
            if (removeRedundantBeams)
            {
                RemoveRedundantBeams();
            }
            
            if (simplifyBeamNetwork)
            {
                SimplifyBeamNetwork();
            }
            
            if (optimizeNodePositions)
            {
                OptimizeNodePositions();
            }
            
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            Debug.Log("Structure optimization completed.");
            SceneView.RepaintAll();
        }
        
        private void MergeCloseNodes()
        {
            var nodePositions = workingTruss.NodePositions.ToList();
            var beams = workingTruss.GetTrussBeams();
            var faces = workingTruss.GetTrussFaces();
            var pinnedNodes = workingTruss.PinnedNodes.ToList();
            
            var mergeMap = new Dictionary<int, int>();
            var toRemove = new List<int>();
            
            for (int i = 0; i < nodePositions.Count; i++)
            {
                if (toRemove.Contains(i)) continue;
                
                for (int j = i + 1; j < nodePositions.Count; j++)
                {
                    if (toRemove.Contains(j)) continue;
                    
                    if (Vector3.Distance(nodePositions[i], nodePositions[j]) < nodeDistanceThreshold)
                    {
                        mergeMap[j] = i;
                        toRemove.Add(j);
                    }
                }
            }
            
            // Remove merged nodes and update indices
            toRemove.Sort((a, b) => b.CompareTo(a));
            foreach (int index in toRemove)
            {
                nodePositions.RemoveAt(index);
            }
            
            // Update all references
            workingTruss.SetNodePositions(nodePositions.ToArray());
            workingTruss.SetBeams(beams);
            workingTruss.SetPinnedNodes(pinnedNodes);
        }
        
        private void RemoveRedundantBeams()
        {
            var beams = workingTruss.GetTrussBeams();
            var uniqueBeams = new List<Beam>();
            var seenPairs = new HashSet<(int, int)>();
            
            foreach (var beam in beams)
            {
                var pair = (Mathf.Min(beam.nodeA, beam.nodeB), Mathf.Max(beam.nodeA, beam.nodeB));
                if (!seenPairs.Contains(pair) && beam.nodeA != beam.nodeB)
                {
                    seenPairs.Add(pair);
                    uniqueBeams.Add(beam);
                }
            }
            
            workingTruss.SetBeams(uniqueBeams);
        }
        
        private void SimplifyBeamNetwork()
        {
            var beams = workingTruss.GetTrussBeams();
            var nodePositions = workingTruss.NodePositions;
            var toRemove = new List<int>();
            
            for (int i = 0; i < beams.Count; i++)
            {
                if (toRemove.Contains(i)) continue;
                
                var beamA = beams[i];
                Vector3 dirA = (nodePositions[beamA.nodeB] - nodePositions[beamA.nodeA]).normalized;
                
                for (int j = i + 1; j < beams.Count; j++)
                {
                    if (toRemove.Contains(j)) continue;
                    
                    var beamB = beams[j];
                    Vector3 dirB = (nodePositions[beamB.nodeB] - nodePositions[beamB.nodeA]).normalized;
                    
                    float angle = Vector3.Angle(dirA, dirB);
                    if (angle < beamAngleThreshold || angle > 180f - beamAngleThreshold)
                    {
                        // Check if beams share a node and are close
                        bool sharesNode = beamA.nodeA == beamB.nodeA || beamA.nodeA == beamB.nodeB ||
                                         beamA.nodeB == beamB.nodeA || beamA.nodeB == beamB.nodeB;
                        
                        if (sharesNode)
                        {
                            // Keep the shorter beam
                            if (beamA.restLength > beamB.restLength)
                            {
                                toRemove.Add(i);
                            }
                            else
                            {
                                toRemove.Add(j);
                            }
                        }
                    }
                }
            }
            
            // Remove beams in reverse order
            toRemove.Sort((a, b) => b.CompareTo(a));
            foreach (int index in toRemove)
            {
                beams.RemoveAt(index);
            }
            
            workingTruss.SetBeams(beams);
        }
        
        private void OptimizeNodePositions()
        {
            // Simple spring relaxation to improve node positions
            var nodePositions = workingTruss.NodePositions.ToArray();
            var beams = workingTruss.GetTrussBeams();
            var pinnedNodes = workingTruss.PinnedNodes;
            
            for (int iteration = 0; iteration < 10; iteration++)
            {
                var forces = new Vector3[nodePositions.Length];
                
                foreach (var beam in beams)
                {
                    if (beam.nodeA >= nodePositions.Length || beam.nodeB >= nodePositions.Length) continue;
                    
                    Vector3 posA = nodePositions[beam.nodeA];
                    Vector3 posB = nodePositions[beam.nodeB];
                    Vector3 delta = posB - posA;
                    float currentLength = delta.magnitude;
                    
                    if (currentLength > 0.001f)
                    {
                        Vector3 direction = delta / currentLength;
                        float force = (currentLength - beam.restLength) * 0.1f;
                        
                        if (!pinnedNodes.Contains(beam.nodeA))
                        {
                            forces[beam.nodeA] += direction * force;
                        }
                        if (!pinnedNodes.Contains(beam.nodeB))
                        {
                            forces[beam.nodeB] -= direction * force;
                        }
                    }
                }
                
                // Apply forces
                for (int i = 0; i < nodePositions.Length; i++)
                {
                    if (!pinnedNodes.Contains(i))
                    {
                        nodePositions[i] += forces[i] * 0.1f;
                    }
                }
            }
            
            workingTruss.SetNodePositions(nodePositions);
        }
        
        private void CenterStructure()
        {
            if (workingTruss.NodePositions == null || workingTruss.NodePositions.Length == 0) return;
            
            Vector3 center = Vector3.zero;
            foreach (var pos in workingTruss.NodePositions)
            {
                center += pos;
            }
            center /= workingTruss.NodePositions.Length;
            
            var nodePositions = workingTruss.NodePositions.ToArray();
            for (int i = 0; i < nodePositions.Length; i++)
            {
                nodePositions[i] -= center;
            }
            
            workingTruss.SetNodePositions(nodePositions);
            EditorUtility.SetDirty(workingTruss);
            
            if (autoApplyChanges)
            {
                ApplyTrussToTarget();
            }
            
            SceneView.RepaintAll();
        }
        
        private void ScaleStructureDialog()
        {
            string scaleStr = EditorUtility.DisplayDialog("Scale Structure", "Enter scale factor:", "1.0") ? "1.0" : "";
            
            if (!string.IsNullOrEmpty(scaleStr) && float.TryParse(scaleStr, out float scale) && scale > 0)
            {
                var nodePositions = workingTruss.NodePositions.ToArray();
                for (int i = 0; i < nodePositions.Length; i++)
                {
                    nodePositions[i] *= scale;
                }
                
                workingTruss.SetNodePositions(nodePositions);
                EditorUtility.SetDirty(workingTruss);
                
                if (autoApplyChanges)
                {
                    ApplyTrussToTarget();
                }
                
                SceneView.RepaintAll();
            }
        }
        
        #endregion
        
        #region Scene Visualization
        
        private void DrawSceneVisualization()
        {
            if (workingTruss == null || targetSoftBody == null) return;
            
            Handles.matrix = targetSoftBody.transform.localToWorldMatrix;
            
            // Draw elements based on current mode for focused visualization
            switch (currentMode)
            {
                case DesignerMode.Node:
                    DrawNodes();
                    break;
                    
                case DesignerMode.Beam:
                    DrawNodes(); // Show nodes as connection points for beams
                    DrawBeams();
                    break;
                    
                case DesignerMode.Face:
                    DrawNodes(); // Show nodes as connection points for faces
                    DrawFaces();
                    break;
                    
                case DesignerMode.MeshConvert:
                case DesignerMode.Optimization:
                    // Show everything for these modes as they work with the complete structure
                    DrawNodes();
                    DrawBeams();
                    DrawFaces();
                    break;
            }
            
            DrawSelectionIndicators();
            DrawCreationPreview();
            DrawTransformHandles();
        }
        
        private void DrawNodes()
        {
            if (workingTruss.NodePositions == null) return;
            
            var pinnedNodes = workingTruss.PinnedNodes;
            
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 position = workingTruss.NodePositions[i];
                
                Color color = nodeColor;
                float size = nodeDisplaySize;
                
                if (isTransformingNode && transformingNodeIndex == i)
                {
                    color = Color.yellow; // Highlight the node being transformed
                    size *= 1.5f; // Make it larger
                }
                else if (selectedNodes.Contains(i))
                    color = selectedNodeColor;
                else if (pinnedNodes.Contains(i))
                    color = pinnedNodeColor;
                
                Handles.color = color;
                Handles.SphereHandleCap(0, position, Quaternion.identity, size, EventType.Repaint);
                
                if (showNodeIndices)
                {
                    Handles.Label(position + Vector3.up * nodeDisplaySize, i.ToString());
                }
            }
        }
        
        private void DrawBeams()
        {
            if (workingTruss.NodePositions == null) return;
            
            var beams = workingTruss.GetTrussBeams();
            var nodePositions = workingTruss.NodePositions;
            
            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                if (beam.nodeA >= nodePositions.Length || beam.nodeB >= nodePositions.Length) continue;
                
                Vector3 posA = nodePositions[beam.nodeA];
                Vector3 posB = nodePositions[beam.nodeB];
                
                Color color = selectedBeams.Contains(i) ? selectedBeamColor : beamColor;
                
                Handles.color = color;
                
                // Use thick lines if thickness > 1, otherwise use regular DrawLine
                if (beamLineThickness > 1.0f)
                {
                    Handles.DrawAAPolyLine(beamLineThickness, posA, posB);
                }
                else
                {
                    Handles.DrawLine(posA, posB);
                }
                
                if (showBeamIndices)
                {
                    Vector3 midPoint = (posA + posB) * 0.5f;
                    Handles.Label(midPoint, i.ToString());
                }
            }
        }
        
        private void DrawFaces()
        {
            if (workingTruss.NodePositions == null) return;
            
            var faces = workingTruss.GetTrussFaces();
            var nodePositions = workingTruss.NodePositions;
            
            for (int i = 0; i < faces.Count; i++)
            {
                var face = faces[i];
                if (face.nodeA >= nodePositions.Length || face.nodeB >= nodePositions.Length || face.nodeC >= nodePositions.Length) continue;
                
                Vector3 posA = nodePositions[face.nodeA];
                Vector3 posB = nodePositions[face.nodeB];
                Vector3 posC = nodePositions[face.nodeC];
                
                Color color = selectedFaces.Contains(i) ? selectedFaceColor : faceColor;
                
                Handles.color = color;
                Handles.DrawAAConvexPolygon(posA, posB, posC);
                
                if (showFaceIndices)
                {
                    Vector3 center = (posA + posB + posC) / 3f;
                    Handles.Label(center, i.ToString());
                }
            }
        }
        
        private void DrawSelectionIndicators()
        {
            if (isBoxSelecting)
            {
                Handles.matrix = Matrix4x4.identity;
                Handles.color = Color.yellow;
                
                Vector3 min = Vector3.Min(boxSelectionStart, boxSelectionEnd);
                Vector3 max = Vector3.Max(boxSelectionStart, boxSelectionEnd);
                Vector3 size = max - min;
                
                Handles.DrawWireCube((min + max) * 0.5f, size);
                Handles.matrix = targetSoftBody.transform.localToWorldMatrix;
            }
        }
        
        private void DrawCreationPreview()
        {
            if (isCreatingBeam && beamStartNode >= 0 && workingTruss.NodePositions != null)
            {
                Vector3 startPos = workingTruss.NodePositions[beamStartNode];
                Vector3 mousePos = GetMouseWorldPosition();
                Vector3 localMousePos = targetSoftBody.transform.InverseTransformPoint(mousePos);
                
                Handles.color = Color.green;
                if (beamLineThickness > 1.0f)
                {
                    Handles.DrawAAPolyLine(beamLineThickness, startPos, localMousePos);
                }
                else
                {
                    Handles.DrawLine(startPos, localMousePos);
                }
                
                Handles.color = Color.yellow;
                Handles.SphereHandleCap(0, startPos, Quaternion.identity, nodeDisplaySize * 1.2f, EventType.Repaint);
            }
        }
        
        #endregion
        
        #region Utility Methods
        
        private Vector3 GetWorldPositionFromMouse(Vector2 mousePosition)
        {
            Ray ray = HandleUtility.GUIPointToWorldRay(mousePosition);
            Plane plane = new Plane(Vector3.up, targetSoftBody.transform.position);
            
            if (plane.Raycast(ray, out float distance))
            {
                return ray.GetPoint(distance);
            }
            
            return targetSoftBody.transform.position;
        }
        
        private Vector3 GetMouseWorldPosition()
        {
            Vector2 mousePosition = Event.current.mousePosition;
            return GetWorldPositionFromMouse(mousePosition);
        }
        
        private int FindClosestNode(Vector3 worldPosition)
        {
            if (workingTruss.NodePositions == null) return -1;
            
            // Create a ray from the mouse position
            Ray ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
            
            int closestIndex = -1;
            float closestDistance = float.MaxValue;
            float selectionRadius = nodeDisplaySize * 1.5f; // Slightly larger than visual size
            
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 nodeWorldPos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                
                // Calculate distance from ray to sphere (node)
                float distanceToRay = DistanceFromRayToSphere(ray, nodeWorldPos, selectionRadius);
                
                if (distanceToRay < closestDistance && distanceToRay <= selectionRadius)
                {
                    closestDistance = distanceToRay;
                    closestIndex = i;
                }
            }
            
            return closestIndex;
        }

        private int FindClosestBeam(Vector3 worldPosition)
        {
            if (workingTruss.NodePositions == null) return -1;

            var beams = workingTruss.GetTrussBeams();
            if (beams == null) return -1;

            int closestIndex = -1;
            float closestDistance = float.MaxValue;
            float selectionThreshold = 10f; // Pixels; adjust if needed (e.g., 5f for finer selection, 15f for looser)

            Vector2 mousePosition = Event.current.mousePosition;

            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                if (beam.nodeA < 0 || beam.nodeA >= workingTruss.NodePositions.Length ||
                    beam.nodeB < 0 || beam.nodeB >= workingTruss.NodePositions.Length) continue;

                Vector3 posA = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeA]);
                Vector3 posB = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeB]);

                // Find the closest 3D point on the beam segment to the mouse ray
                Vector3 closestPoint = HandleUtility.ClosestPointToPolyLine(posA, posB);

                // Project to screen space and calculate pixel distance to mouse
                Vector2 screenPoint = HandleUtility.WorldToGUIPoint(closestPoint);
                float dist = Vector2.Distance(screenPoint, mousePosition);

                if (dist < closestDistance)
                {
                    closestDistance = dist;
                    closestIndex = i;
                }
            }

            // Only return if within threshold to avoid accidental far-away selections
            if (closestIndex >= 0 && closestDistance <= selectionThreshold)
            {
                return closestIndex;
            }

            return -1;
        }
        
        private int FindClosestFace(Vector3 worldPosition)
        {
            if (workingTruss.NodePositions == null) return -1;
            
            // Create a ray from the mouse position
            Ray ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
            
            var faces = workingTruss.GetTrussFaces();
            int closestIndex = -1;
            float closestDistance = float.MaxValue;
            
            for (int i = 0; i < faces.Count; i++)
            {
                var face = faces[i];
                if (face.nodeA >= workingTruss.NodePositions.Length || 
                    face.nodeB >= workingTruss.NodePositions.Length || 
                    face.nodeC >= workingTruss.NodePositions.Length) continue;
                
                Vector3 posA = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[face.nodeA]);
                Vector3 posB = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[face.nodeB]);
                Vector3 posC = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[face.nodeC]);
                
                // Check if ray intersects with triangle
                if (RayIntersectsTriangle(ray, posA, posB, posC, out float distance))
                {
                    if (distance < closestDistance)
                    {
                        closestDistance = distance;
                        closestIndex = i;
                    }
                }
            }
            
            return closestIndex;
        }
        
        private float DistanceFromRayToSphere(Ray ray, Vector3 sphereCenter, float sphereRadius)
        {
            Vector3 rayToSphere = sphereCenter - ray.origin;
            float projectionLength = Vector3.Dot(rayToSphere, ray.direction);
            
            // Find closest point on ray to sphere center
            Vector3 closestPointOnRay = ray.origin + ray.direction * Mathf.Max(0f, projectionLength);
            float distanceToCenter = Vector3.Distance(closestPointOnRay, sphereCenter);
            
            // Return 0 if inside sphere, otherwise distance to sphere surface
            return Mathf.Max(0f, distanceToCenter - sphereRadius);
        }
        
        private float DistanceFromRayToLineSegment(Ray ray, Vector3 lineStart, Vector3 lineEnd)
        {
            Vector3 lineDirection = lineEnd - lineStart;
            float lineLength = lineDirection.magnitude;
            
            if (lineLength < 0.001f)
            {
                // Degenerate line, treat as point
                return DistanceFromRayToSphere(ray, lineStart, nodeDisplaySize * 0.5f);
            }
            
            lineDirection /= lineLength;
            
            // Find the closest points between the ray and line segment
            Vector3 rayToLineStart = lineStart - ray.origin;
            float rayDotLine = Vector3.Dot(ray.direction, lineDirection);
            float rayDotToStart = Vector3.Dot(ray.direction, rayToLineStart);
            float lineDotToStart = Vector3.Dot(lineDirection, rayToLineStart);
            
            float denominator = 1f - rayDotLine * rayDotLine;
            
            float rayParam, lineParam;
            
            if (Mathf.Abs(denominator) > 0.001f)
            {
                // Lines are not parallel
                rayParam = (rayDotLine * lineDotToStart - rayDotToStart) / denominator;
                lineParam = (lineDotToStart - rayDotLine * rayDotToStart) / denominator;
            }
            else
            {
                // Lines are parallel
                rayParam = 0f;
                lineParam = lineDotToStart;
            }
            
            // Clamp line parameter to segment bounds
            lineParam = Mathf.Clamp(lineParam, 0f, lineLength);
            
            // Calculate closest points
            Vector3 closestOnRay = ray.origin + ray.direction * Mathf.Max(0f, rayParam);
            Vector3 closestOnLine = lineStart + lineDirection * lineParam;
            
            return Vector3.Distance(closestOnRay, closestOnLine);
        }
        
        private bool RayIntersectsTriangle(Ray ray, Vector3 vertex0, Vector3 vertex1, Vector3 vertex2, out float distance)
        {
            distance = 0f;
            
            // Calculate triangle edges
            Vector3 edge1 = vertex1 - vertex0;
            Vector3 edge2 = vertex2 - vertex0;
            
            // Calculate triangle normal
            Vector3 triangleNormal = Vector3.Cross(edge1, edge2);
            float normalLength = triangleNormal.magnitude;
            
            if (normalLength < 0.001f)
            {
                // Degenerate triangle
                return false;
            }
            
            triangleNormal /= normalLength;
            
            // Check if ray is parallel to triangle
            float rayDotNormal = Vector3.Dot(ray.direction, triangleNormal);
            if (Mathf.Abs(rayDotNormal) < 0.001f)
            {
                return false; // Ray is parallel to triangle
            }
            
            // Calculate intersection with triangle plane
            Vector3 rayToVertex = vertex0 - ray.origin;
            distance = Vector3.Dot(rayToVertex, triangleNormal) / rayDotNormal;
            
            if (distance < 0f)
            {
                return false; // Triangle is behind ray origin
            }
            
            // Calculate intersection point
            Vector3 intersectionPoint = ray.origin + ray.direction * distance;
            
            // Check if point is inside triangle using barycentric coordinates
            Vector3 pointToVertex0 = intersectionPoint - vertex0;
            
            float dot00 = Vector3.Dot(edge1, edge1);
            float dot01 = Vector3.Dot(edge1, edge2);
            float dot02 = Vector3.Dot(edge1, pointToVertex0);
            float dot11 = Vector3.Dot(edge2, edge2);
            float dot12 = Vector3.Dot(edge2, pointToVertex0);
            
            float invDenom = 1f / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            
            // Check if point is inside triangle
            return (u >= 0f) && (v >= 0f) && (u + v <= 1f);
        }
        
        private float CalculateAverageVertexDistance(Vector3[] vertices)
        {
            if (vertices.Length < 2) return 1f;
            
            float totalDistance = 0f;
            int pairCount = 0;
            
            // Sample only first 10 vertices for performance
            for (int i = 0; i < vertices.Length && i < 10; i++)
            {
                for (int j = i + 1; j < vertices.Length && j < 10; j++)
                {
                    totalDistance += Vector3.Distance(vertices[i], vertices[j]);
                    pairCount++;
                }
            }
            
            return pairCount > 0 ? totalDistance / pairCount : 1f;
        }
        
        private void DrawTransformHandles()
        {
            if (currentMode != DesignerMode.Node || selectedNodes.Count == 0 || !showTransformHandles) return;
            
            // Calculate selection center
            UpdateSelectionCenter();
            
            // Show position handle for multiple selected nodes
            if (selectedNodes.Count > 0)
            {
                EditorGUI.BeginChangeCheck();
                Vector3 newCenter = Handles.PositionHandle(selectionCenter, Quaternion.identity);
                
                if (EditorGUI.EndChangeCheck())
                {
                    // Apply the offset to all selected nodes
                    Vector3 offset = newCenter - selectionCenter;
                    MoveSelectedNodes(offset);
                    UpdateSelectionCenter(); // Update center after move
                }
            }
        }
        
        private void UpdateSelectionCenter()
        {
            if (selectedNodes.Count == 0 || workingTruss?.NodePositions == null)
            {
                selectionCenter = Vector3.zero;
                return;
            }
            
            Vector3 center = Vector3.zero;
            int validNodes = 0;
            
            foreach (int nodeIndex in selectedNodes)
            {
                if (nodeIndex >= 0 && nodeIndex < workingTruss.NodePositions.Length)
                {
                    center += workingTruss.NodePositions[nodeIndex];
                    validNodes++;
                }
            }
            
            if (validNodes > 0)
            {
                selectionCenter = center / validNodes;
            }
        }
        
        private void MoveSelectedNodes(Vector3 offset)
        {
            if (workingTruss?.NodePositions == null || selectedNodes.Count == 0) return;
            
            var nodePositions = workingTruss.NodePositions.ToList();
            bool hasChanges = false;
            
            foreach (int nodeIndex in selectedNodes)
            {
                if (nodeIndex >= 0 && nodeIndex < nodePositions.Count)
                {
                    nodePositions[nodeIndex] += offset;
                    hasChanges = true;
                }
            }
            
            if (hasChanges)
            {
                workingTruss.SetNodePositions(nodePositions.ToArray());
                EditorUtility.SetDirty(workingTruss);
                
                if (autoApplyChanges && targetSoftBody != null)
                {
                    targetSoftBody.ApplyTrussAsset(workingTruss);
                }
                
                SceneView.RepaintAll();
            }
        }
        
        private void SavePreferences()
        {
            EditorPrefs.SetBool("SoftBodyDesigner_ShowVisualization", showVisualizationSettings);
            EditorPrefs.SetBool("SoftBodyDesigner_ShowConversion", showConversionSettings);
            EditorPrefs.SetBool("SoftBodyDesigner_ShowOptimization", showOptimizationSettings);
            EditorPrefs.SetBool("SoftBodyDesigner_AutoApply", autoApplyChanges);
            EditorPrefs.SetFloat("SoftBodyDesigner_NodeSize", nodeDisplaySize);
            EditorPrefs.SetFloat("SoftBodyDesigner_BeamThickness", beamLineThickness);
            EditorPrefs.SetFloat("SoftBodyDesigner_NodeMass", nodeCreationMass);
            EditorPrefs.SetFloat("SoftBodyDesigner_BeamCompliance", defaultBeamCompliance);
            EditorPrefs.SetFloat("SoftBodyDesigner_BeamDamping", defaultBeamDamping);
            EditorPrefs.SetFloat("SoftBodyDesigner_MinStretchFactor", defaultMinStretchFactor);
            EditorPrefs.SetFloat("SoftBodyDesigner_MaxStretchFactor", defaultMaxStretchFactor);
            EditorPrefs.SetInt("SoftBodyDesigner_TargetInstanceID", targetSoftBodyInstanceID);
        }
        
        private void LoadPreferences()
        {
            showVisualizationSettings = EditorPrefs.GetBool("SoftBodyDesigner_ShowVisualization", true);
            showConversionSettings = EditorPrefs.GetBool("SoftBodyDesigner_ShowConversion", true);
            showOptimizationSettings = EditorPrefs.GetBool("SoftBodyDesigner_ShowOptimization", true);
            autoApplyChanges = EditorPrefs.GetBool("SoftBodyDesigner_AutoApply", true);
            nodeDisplaySize = EditorPrefs.GetFloat("SoftBodyDesigner_NodeSize", 0.15f);
            beamLineThickness = EditorPrefs.GetFloat("SoftBodyDesigner_BeamThickness", 2.0f);
            nodeCreationMass = EditorPrefs.GetFloat("SoftBodyDesigner_NodeMass", 0.5f);
            defaultBeamCompliance = EditorPrefs.GetFloat("SoftBodyDesigner_BeamCompliance", 0.01f);
            defaultBeamDamping = EditorPrefs.GetFloat("SoftBodyDesigner_BeamDamping", 0.3f);
            defaultMinStretchFactor = EditorPrefs.GetFloat("SoftBodyDesigner_MinStretchFactor", 0.5f);
            defaultMaxStretchFactor = EditorPrefs.GetFloat("SoftBodyDesigner_MaxStretchFactor", 2.0f);
            targetSoftBodyInstanceID = EditorPrefs.GetInt("SoftBodyDesigner_TargetInstanceID", -1);
        }
        
        #endregion
        
        #region New Features - Transform, Select All, Presets
        
        private void GetBeamPresetValues(BeamPreset preset, out float compliance, out float damping, out float minStretch, out float maxStretch)
        {
            switch (preset)
            {
                case BeamPreset.Metal:
                    compliance = 0.001f;  // Very stiff
                    damping = 0.1f;       // Low damping
                    minStretch = 0.8f;    // Limited compression
                    maxStretch = 1.2f;    // Limited extension
                    break;
                case BeamPreset.Rubber:
                    compliance = 0.1f;    // Very flexible
                    damping = 0.8f;       // High damping
                    minStretch = 0.3f;    // High compression
                    maxStretch = 3.0f;    // High extension
                    break;
                default: // Custom
                    compliance = defaultBeamCompliance;
                    damping = defaultBeamDamping;
                    minStretch = defaultMinStretchFactor;
                    maxStretch = defaultMaxStretchFactor;
                    break;
            }
            
            // Apply stretch factors to the current truss if available
            if (workingTruss != null)
            {
                workingTruss.SetMinStretchFactor(minStretch);
                workingTruss.SetMaxStretchFactor(maxStretch);
                EditorUtility.SetDirty(workingTruss);
                
                if (autoApplyChanges)
                {
                    ApplyTrussToTarget();
                }
            }
        }
        
        private void SelectAllBeams()
        {
            if (workingTruss == null) return;
            
            var beams = workingTruss.GetTrussBeams();
            selectedBeams.Clear();
            for (int i = 0; i < beams.Count; i++)
            {
                selectedBeams.Add(i);
            }
            Repaint();
        }
        
        private void SelectAllFaces()
        {
            if (workingTruss == null) return;
            
            var faces = workingTruss.GetTrussFaces();
            selectedFaces.Clear();
            for (int i = 0; i < faces.Count; i++)
            {
                selectedFaces.Add(i);
            }
            Repaint();
        }
        
        private void StartNodeTransform(int nodeIndex)
        {
            if (workingTruss?.NodePositions == null || nodeIndex < 0 || nodeIndex >= workingTruss.NodePositions.Length)
                return;
                
            isTransformingNode = true;
            transformingNodeIndex = nodeIndex;
            
            Vector3 nodeWorldPos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[nodeIndex]);
            Vector3 mouseWorldPos = GetWorldPositionFromMouse(Event.current.mousePosition);
            nodeTransformOffset = nodeWorldPos - mouseWorldPos;
        }
        
        private void UpdateNodeTransform(Vector3 mouseWorldPosition)
        {
            if (!isTransformingNode || transformingNodeIndex < 0) return;
            
            Vector3 newWorldPosition = mouseWorldPosition + nodeTransformOffset;
            Vector3 newLocalPosition = targetSoftBody.transform.InverseTransformPoint(newWorldPosition);
            
            var nodePositions = workingTruss.NodePositions.ToList();
            if (transformingNodeIndex < nodePositions.Count)
            {
                nodePositions[transformingNodeIndex] = newLocalPosition;
                workingTruss.SetNodePositions(nodePositions.ToArray());
                
                if (autoApplyChanges && targetSoftBody != null)
                {
                    targetSoftBody.ApplyTrussAsset(workingTruss);
                }
                
                SceneView.RepaintAll();
            }
        }
        
        private void EndNodeTransform()
        {
            isTransformingNode = false;
            transformingNodeIndex = -1;
            nodeTransformOffset = Vector3.zero;
        }
        
        #endregion
    }
}