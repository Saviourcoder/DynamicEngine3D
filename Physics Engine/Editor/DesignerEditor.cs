using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

namespace DynamicEngine
{
    [CustomEditor(typeof(SoftBodyDesigner))]
    public class SoftBodyDesignerEditor : UnityEditor.Editor
    {
        // --- Constants ---
        const int TAB_TRUSS = 0;
        const int TAB_NODES = 1;
        const int TAB_BEAMS = 2;
        const int TAB_FACES = 3;


        // --- References ---
        private SoftBodyDesigner designer;
        private SoftBody targetSoftBody;
        private Truss workingTruss;

        // --- Editor State ---
        private int currentTab = 0;

        // --- Tool State (Transient) ---
        private bool isCreatingNode = false;
        private bool isCreatingBeam = false;
        private bool isCreatingFace = false;
        private int beamStartNode = -1;
        private List<int> faceCreationNodes = new List<int>();

        // --- Transform State ---
        private bool isTransformingNode = false;
        private int transformingNodeIndex = -1;
        private Vector3 selectionCenter = Vector3.zero;
        private void OnEnable()
        {
            designer = (SoftBodyDesigner)target;
            targetSoftBody = designer.TargetSoftBody;

            if (targetSoftBody != null)
            {
                workingTruss = targetSoftBody.GetTrussAsset();
            }

            if (designer.selectedNodes == null) designer.selectedNodes = new List<int>();
            if (designer.selectedBeams == null) designer.selectedBeams = new List<int>();
            if (designer.selectedFaces == null) designer.selectedFaces = new List<int>();
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            if (targetSoftBody == null)
            {
                EditorGUILayout.HelpBox("No SoftBody component found on this GameObject.", MessageType.Error);
                return;
            }

            if (workingTruss != targetSoftBody.GetTrussAsset())
            {
                workingTruss = targetSoftBody.GetTrussAsset();
            }

            DrawHeader();

            string[] tabs = { "Truss Info", "Nodes", "Beams", "Faces" };

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            currentTab = GUILayout.Toolbar(currentTab, tabs, GUILayout.Height(25));
            EditorGUILayout.EndVertical();

            UpdateDesignerModeFromTab();

            if (workingTruss != null)
            {
                switch (currentTab)
                {
                    case TAB_TRUSS: DrawTrussTab(); break;
                    case TAB_NODES: DrawNodesTab(); break;
                    case TAB_BEAMS: DrawBeamsTab(); break;
                    case TAB_FACES: DrawFacesTab(); break;
                }
            }
            else
            {
                EditorGUILayout.HelpBox("No Truss Asset assigned.", MessageType.Warning);
                if (currentTab != TAB_TRUSS) currentTab = TAB_TRUSS;
                DrawTrussTab();
            }

            if (GUI.changed)
            {
                serializedObject.ApplyModifiedProperties();
                SceneView.RepaintAll();
            }
        }

        private void UpdateDesignerModeFromTab()
        {
            switch (currentTab)
            {
                case TAB_NODES: designer.currentMode = SoftBodyDesigner.DesignerMode.Node; break;
                case TAB_BEAMS: designer.currentMode = SoftBodyDesigner.DesignerMode.Beam; break;
                case TAB_FACES: designer.currentMode = SoftBodyDesigner.DesignerMode.Face; break;
                default: designer.currentMode = SoftBodyDesigner.DesignerMode.Node; break;
            }
        }

        #region Tab Draw Methods

        private new void DrawHeader()
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Soft Body Designer", EditorStyles.boldLabel);
            EditorGUILayout.Space();
        }

        private void DrawTrussTab()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Truss Information", EditorStyles.boldLabel);
            EditorGUILayout.Separator();

            GUI.enabled = false; // Grey out fields

            EditorGUILayout.ObjectField("Truss File", workingTruss, typeof(Truss), false);

            if (workingTruss != null)
            {
                EditorGUILayout.IntField("Nodes", workingTruss.NodePositions?.Length ?? 0);
                EditorGUILayout.IntField("Links", workingTruss.GetTrussBeams()?.Count ?? 0);
                EditorGUILayout.IntField("Faces", workingTruss.GetTrussFaces()?.Count ?? 0);
            }
            else
            {
                EditorGUILayout.IntField("Nodes", 0);
                EditorGUILayout.IntField("Links", 0);
                EditorGUILayout.IntField("Faces", 0);
            }

            GUI.enabled = true; // Restore enabled state

            EditorGUILayout.EndVertical();
        }

        private void DrawNodesTab()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);

            int count = designer.selectedNodes.Count;
            string selInfo = count == 0 ? "No Nodes Selected" : $"{count} Nodes Selected";
            var style = new GUIStyle(EditorStyles.boldLabel) { alignment = TextAnchor.MiddleRight };
            EditorGUILayout.LabelField(selInfo, style);

            if (count > 0)
            {
                EditorGUILayout.Separator();
                designer.nodeCreationMass = EditorGUILayout.FloatField("Mass (kg)", designer.nodeCreationMass);

                if (count == 1 && designer.selectedNodes[0] < workingTruss.NodePositions.Length)
                {
                    int idx = designer.selectedNodes[0];
                    Vector3 currentPos = workingTruss.NodePositions[idx];
                    EditorGUI.BeginChangeCheck();
                    Vector3 newPos = EditorGUILayout.Vector3Field("Position (Local)", currentPos);
                    if (EditorGUI.EndChangeCheck())
                    {
                        SetNodePosition(idx, newPos);
                    }
                }

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Pin")) PinSelectedNodes(true);
                if (GUILayout.Button("Unpin")) PinSelectedNodes(false);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Set Mass")) SetSelectedNodesMass(designer.nodeCreationMass);
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                EditorGUILayout.HelpBox("Select nodes in Scene View to edit.", MessageType.Info);
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space();

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = true;
            bool createPressed = GUILayout.Button("Create Node", isCreatingNode ? new GUIStyle(GUI.skin.button) { normal = { textColor = Color.green } } : GUI.skin.button);
            if (createPressed) ToggleNodeCreation();

            GUI.enabled = count > 0;
            if (GUILayout.Button("Delete")) DeleteSelectedNodes();

            GUI.enabled = true;
            if (GUILayout.Button("Select All")) SelectAllNodes();

            if (GUILayout.Button("Clear")) { designer.selectedNodes.Clear(); SceneView.RepaintAll(); }
            EditorGUILayout.EndHorizontal();

            if (isCreatingNode)
            {
                EditorGUILayout.HelpBox("Click in scene to place new nodes.", MessageType.None);
            }

            if (count > 0)
            {
                EditorGUILayout.Space();
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                EditorGUILayout.LabelField("Transform Tools", EditorStyles.boldLabel);
                designer.showTransformHandles = EditorGUILayout.Toggle("Show Handles", designer.showTransformHandles);
                EditorGUILayout.HelpBox("Hold 'T' + Click to drag nodes freely.", MessageType.None);
                EditorGUILayout.EndVertical();
            }
        }

        private void DrawBeamsTab()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);

            int count = designer.selectedBeams.Count;
            string selInfo = count == 0 ? "No Beams Selected" : $"{count} Beams Selected";
            var style = new GUIStyle(EditorStyles.boldLabel) { alignment = TextAnchor.MiddleRight };
            EditorGUILayout.LabelField(selInfo, style);

            EditorGUILayout.Separator();

            EditorGUILayout.LabelField("Beam Properties", EditorStyles.miniLabel);

            EditorGUI.BeginChangeCheck();
            designer.selectedBeamPreset = (SoftBodyDesigner.BeamPreset)EditorGUILayout.EnumPopup("Preset", designer.selectedBeamPreset);
            if (EditorGUI.EndChangeCheck())
            {
                if (designer.selectedBeamPreset != SoftBodyDesigner.BeamPreset.Custom)
                {
                    float c, d, thresh, rate, defScale, maxDef;
                    GetBeamPresetValues(designer.selectedBeamPreset, out c, out d, out thresh, out rate, out defScale, out maxDef);
                    designer.defaultBeamCompliance = c;
                    designer.defaultBeamDamping = d;
                    designer.defaultPlasticityThreshold = thresh;
                    designer.defaultPlasticityRate = rate;
                    designer.defaultDeformationScale = defScale;  // â† ADD THIS!
                    designer.defaultMaxDeformation = maxDef;

                    // Apply to selected beams immediately
                    var beams = workingTruss.GetTrussBeams();
                    foreach (int idx in designer.selectedBeams)
                    {
                        if (idx < beams.Count)
                        {
                            var beam = beams[idx];
                            beam.compliance = c;
                            beam.damping = d;
                            beam.plasticityThreshold = thresh;
                            beam.plasticityRate = rate;
                            beam.deformationScale = defScale;  // â† ADD THIS!
                            beam.maxDeformation = maxDef;
                            beams[idx] = beam;
                        }
                    }
                    EditorUtility.SetDirty(workingTruss);
                    if (designer.autoApplyChanges) ApplyTrussToTarget();
                }
            }

            EditorGUI.BeginChangeCheck();
            designer.defaultBeamCompliance = EditorGUILayout.FloatField("Compliance", designer.defaultBeamCompliance);
            designer.defaultBeamDamping = EditorGUILayout.Slider("Damping", designer.defaultBeamDamping, 0f, 1f);
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Plasticity Settings", EditorStyles.miniLabel);
            designer.defaultPlasticityThreshold = EditorGUILayout.Slider("Plastic Threshold", designer.defaultPlasticityThreshold, 0f, 0.5f);
            designer.defaultPlasticityRate = EditorGUILayout.Slider("Plastic Rate", designer.defaultPlasticityRate, 0f, 100f);
            designer.defaultDeformationScale = EditorGUILayout.Slider("Deformation Scale", designer.defaultDeformationScale, 0f, 3f); // â† ADD THIS!
            designer.defaultMaxDeformation = EditorGUILayout.Slider("Max Deformation", designer.defaultMaxDeformation, 0f, 3f);

            if (EditorGUI.EndChangeCheck() && count > 0)
            {
                var beams = workingTruss.GetTrussBeams();
                foreach (int idx in designer.selectedBeams)
                {
                    if (idx < beams.Count)
                    {
                        var beam = beams[idx];
                        beam.compliance = designer.defaultBeamCompliance;
                        beam.damping = designer.defaultBeamDamping;
                        beam.plasticityThreshold = designer.defaultPlasticityThreshold;
                        beam.plasticityRate = designer.defaultPlasticityRate;
                        beam.deformationScale = designer.defaultDeformationScale;  // â† ADD THIS!
                        beam.maxDeformation = designer.defaultMaxDeformation;
                        beams[idx] = beam;
                    }
                }
                EditorUtility.SetDirty(workingTruss);
                if (designer.autoApplyChanges) ApplyTrussToTarget();
            }

            EditorGUILayout.EndVertical();

            EditorGUILayout.Space();

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = true;
            if (GUILayout.Button(isCreatingBeam ? "Cancel Create" : "Create Beam")) ToggleBeamCreation();
            if (GUILayout.Button("Auto Connect")) AutoConnectSelectedNodes();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = count > 0;
            if (GUILayout.Button("Delete")) DeleteSelectedBeams();
            GUI.enabled = true;
            if (GUILayout.Button("Select All")) SelectAllBeams();
            if (GUILayout.Button("Clear")) { designer.selectedBeams.Clear(); SceneView.RepaintAll(); }
            EditorGUILayout.EndHorizontal();

            if (isCreatingBeam)
            {
                EditorGUILayout.HelpBox("Click two nodes to connect them.", MessageType.None);
            }
        }

        private void DrawFacesTab()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);

            int count = designer.selectedFaces.Count;
            string selInfo = count == 0 ? "No Faces Selected" : $"{count} Faces Selected";
            var style = new GUIStyle(EditorStyles.boldLabel) { alignment = TextAnchor.MiddleRight };
            EditorGUILayout.LabelField(selInfo, style);

            if (count == 0)
            {
                EditorGUILayout.HelpBox("Select faces to edit.", MessageType.Info);
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button(isCreatingFace ? "Cancel Create" : "Create Face")) ToggleFaceCreation();
            if (GUILayout.Button("Auto Generate")) AutoGenerateFaces();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = count > 0;
            if (GUILayout.Button("Delete")) DeleteSelectedFaces();
            GUI.enabled = true;
            if (GUILayout.Button("Select All")) SelectAllFaces();
            if (GUILayout.Button("Clear")) { designer.selectedFaces.Clear(); SceneView.RepaintAll(); }
            EditorGUILayout.EndHorizontal();

            if (isCreatingFace)
            {
                string msg = faceCreationNodes.Count > 0 ? $"{faceCreationNodes.Count}/3 Nodes Selected" : "Click 3 nodes to form a face.";
                EditorGUILayout.HelpBox(msg, MessageType.None);
            }
        }

        #endregion

        #region Scene View & Logic

        public void OnSceneGUI()
        {
            if (designer == null || targetSoftBody == null || workingTruss == null || workingTruss.NodePositions == null) return;

            Event e = Event.current;
            int controlID = GUIUtility.GetControlID(FocusType.Passive);
            HandleUtility.AddDefaultControl(controlID); // â† CRITICAL: blocks default tools/hierarchy selection

            Vector3 mouseWorldPos = GetWorldPositionFromMouse(e.mousePosition);

            // Always draw visualization
            DrawSceneVisualization();

            switch (e.type)
            {
                case EventType.MouseDown:
                    if (e.button == 0 && !e.alt) // Left click (ignore alt-camera)
                    {
                        bool multi = e.shift || e.control;

                        if (isCreatingNode)
                        {
                            CreateNodeAtPosition(mouseWorldPos);
                            if (!e.shift) ToggleNodeCreation(); // Hold Shift to keep placing
                        }
                        else if (isCreatingBeam)
                        {
                            HandleBeamCreation(mouseWorldPos);
                        }
                        else if (isCreatingFace)
                        {
                            HandleFaceCreation(mouseWorldPos);
                        }
                        else
                        {
                            // Normal selection
                            if (designer.currentMode == SoftBodyDesigner.DesignerMode.Node)
                                SelectNodeAtPosition(mouseWorldPos, multi);
                            else if (designer.currentMode == SoftBodyDesigner.DesignerMode.Beam)
                                SelectBeamAtPosition(mouseWorldPos, multi);
                            else if (designer.currentMode == SoftBodyDesigner.DesignerMode.Face)
                                SelectFaceAtPosition(mouseWorldPos, multi);
                        }

                        e.Use(); // â† CRITICAL: prevents hierarchy selection / move tool
                    }
                    break;

                case EventType.MouseDrag:
                    if (isTransformingNode && e.button == 0)
                    {
                        UpdateNodeTransform(mouseWorldPos);
                        e.Use();
                    }
                    break;

                case EventType.MouseUp:
                    if (e.button == 0)
                    {
                        EndNodeTransform();
                        e.Use();
                    }
                    break;

                case EventType.KeyDown:
                    if (e.keyCode == KeyCode.Escape)
                    {
                        CancelCurrentOperation();
                        e.Use();
                    }
                    break;
            }

            // Transform handles for selected nodes
            if (designer.showTransformHandles && designer.selectedNodes.Count > 0 && !isCreatingNode && !isCreatingBeam && !isCreatingFace)
            {
                Vector3 center = targetSoftBody.transform.TransformPoint(selectionCenter);
                EditorGUI.BeginChangeCheck();
                center = Handles.PositionHandle(center, Quaternion.identity);
                if (EditorGUI.EndChangeCheck())
                {
                    Vector3 localOffset = targetSoftBody.transform.InverseTransformPoint(center) - selectionCenter;
                    MoveSelectedNodes(localOffset);
                    UpdateSelectionCenter();
                }
            }

            // Force repaint when selection changes
            if (e.type == EventType.MouseDown || e.type == EventType.MouseUp || e.type == EventType.KeyDown)
            {
                SceneView.RepaintAll();
                Repaint();
            }
        }

        private void HandleSceneInput()
        {
            Event e = Event.current;
            int controlID = GUIUtility.GetControlID(FocusType.Passive);

            switch (e.type)
            {
                case EventType.MouseDown:
                    if (e.button == 0 && !e.alt)
                    {
                        HandleMouseDown(e);

                        // Only capture input if we're in a mode that needs it
                        bool shouldCapture = false;
                        Vector3 worldPos = GetWorldPositionFromMouse(e.mousePosition);

                        switch (currentTab)
                        {
                            case TAB_NODES:
                                shouldCapture = isCreatingNode || FindClosestNode(worldPos) != -1;
                                break;
                            case TAB_BEAMS:
                                shouldCapture = isCreatingBeam || (FindClosestNode(worldPos) != -1 && !isCreatingBeam);
                                break;
                            case TAB_FACES:
                                shouldCapture = isCreatingFace || (FindClosestNode(worldPos) != -1 && !isCreatingFace);
                                break;
                        }

                        if (shouldCapture)
                        {
                            GUIUtility.hotControl = controlID;
                            e.Use();
                        }
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
                    if (e.keyCode == KeyCode.Delete) { DeleteSelectedElements(); e.Use(); }
                    if (e.keyCode == KeyCode.Escape) { CancelCurrentOperation(); e.Use(); }
                    if (e.keyCode == KeyCode.A && e.control)
                    {
                        switch (currentTab)
                        {
                            case TAB_NODES: SelectAllNodes(); break;
                            case TAB_BEAMS: SelectAllBeams(); break;
                            case TAB_FACES: SelectAllFaces(); break;
                        }
                        e.Use();
                    }
                    break;
            }
        }

        private void HandleMouseDown(Event e)
        {
            Vector3 worldPos = GetWorldPositionFromMouse(e.mousePosition);

            if (isCreatingNode && currentTab == TAB_NODES)
            {
                CreateNodeAtPosition(worldPos);
            }
            else if (isCreatingBeam && currentTab == TAB_BEAMS)
            {
                HandleBeamCreation(worldPos);
            }
            else if (isCreatingFace && currentTab == TAB_FACES)
            {
                HandleFaceCreation(worldPos);
            }
            else if (e.keyCode == KeyCode.T || (e.modifiers & EventModifiers.Shift) != 0)
            {
                // Transform mode - only available in Nodes tab
                if (currentTab == TAB_NODES)
                {
                    int nodeIndex = FindClosestNode(worldPos);
                    if (nodeIndex >= 0)
                    {
                        StartNodeTransform(nodeIndex);
                        if (!designer.selectedNodes.Contains(nodeIndex))
                        {
                            designer.selectedNodes.Clear();
                            designer.selectedNodes.Add(nodeIndex);
                        }
                    }
                }
            }
            else
            {
                // Regular selection
                switch (currentTab)
                {
                    case TAB_NODES:
                        SelectNodeAtPosition(worldPos, e.control);
                        break;
                    case TAB_BEAMS:
                        if (!isCreatingBeam) SelectBeamAtPosition(worldPos, e.control);
                        break;
                    case TAB_FACES:
                        if (!isCreatingFace) SelectFaceAtPosition(worldPos, e.control);
                        break;
                }
            }
        }

        private void HandleMouseDrag(Event e)
        {
            if (isTransformingNode)
            {
                Vector3 worldPos = GetWorldPositionFromMouse(e.mousePosition);
                UpdateNodeTransform(worldPos);
            }
        }

        private void HandleMouseUp(Event e)
        {
            if (isTransformingNode)
            {
                EndNodeTransform();
            }
        }

        // --- Core Functions ---

        private void CreateNewTruss()
        {
            string path = EditorUtility.SaveFilePanelInProject("Create New Truss Asset", "NewTruss", "asset", "Create a new truss asset");
            if (!string.IsNullOrEmpty(path))
            {
                workingTruss = ScriptableObject.CreateInstance<Truss>();
                AssetDatabase.CreateAsset(workingTruss, path);
                AssetDatabase.SaveAssets();
                targetSoftBody.truss = workingTruss;
                EditorUtility.SetDirty(targetSoftBody);
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

        private void ClearSelections()
        {
            designer.selectedNodes.Clear();
            designer.selectedBeams.Clear();
            designer.selectedFaces.Clear();
            SceneView.RepaintAll();
        }

        private void DrawSceneVisualization()
        {
            Handles.matrix = targetSoftBody.transform.localToWorldMatrix;

            switch (currentTab)
            {
                case TAB_NODES:
                    DrawNodes();
                    break;
                case TAB_BEAMS:
                    DrawNodes();
                    DrawBeams();
                    break;
                case TAB_FACES:
                    DrawNodes();
                    DrawFaces();
                    break;
            }

            DrawTransformHandles();
        }

        private void DrawNodes()
        {
            if (workingTruss.NodePositions == null) return;
            var pinnedNodes = workingTruss.PinnedNodes;

            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 position = workingTruss.NodePositions[i];
                Color color = designer.nodeColor;
                float size = designer.nodeDisplaySize;

                if (isTransformingNode && transformingNodeIndex == i) { color = Color.yellow; size *= 1.2f; }
                else if (designer.selectedNodes.Contains(i)) color = designer.selectedNodeColor;
                else if (pinnedNodes.Contains(i)) color = designer.pinnedNodeColor;

                Handles.color = color;
                Handles.SphereHandleCap(0, position, Quaternion.identity, size, EventType.Repaint);
                if (designer.showNodeIndices) Handles.Label(position + Vector3.up * size, i.ToString());
            }
        }

        private void DrawBeams()
        {
            var beams = workingTruss.GetTrussBeams();
            if (beams == null || workingTruss.NodePositions == null) return;

            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                if (beam.nodeA >= workingTruss.NodePositions.Length || beam.nodeB >= workingTruss.NodePositions.Length) continue;

                Vector3 posA = workingTruss.NodePositions[beam.nodeA];
                Vector3 posB = workingTruss.NodePositions[beam.nodeB];

                Color color = designer.selectedBeams.Contains(i) ? designer.selectedBeamColor : designer.beamColor;
                Handles.color = color;

                if (designer.beamLineThickness > 1.0f) Handles.DrawAAPolyLine(designer.beamLineThickness, posA, posB);
                else Handles.DrawLine(posA, posB);

                if (designer.showBeamIndices) Handles.Label((posA + posB) * 0.5f, i.ToString());
            }
        }

        private void DrawFaces()
        {
            var faces = workingTruss.GetTrussFaces();
            if (faces == null || workingTruss.NodePositions == null) return;

            Handles.color = designer.faceColor;
            for (int i = 0; i < faces.Count; i++)
            {
                var face = faces[i];
                if (face.nodeA >= workingTruss.NodePositions.Length || face.nodeB >= workingTruss.NodePositions.Length || face.nodeC >= workingTruss.NodePositions.Length) continue;

                Vector3 a = workingTruss.NodePositions[face.nodeA];
                Vector3 b = workingTruss.NodePositions[face.nodeB];
                Vector3 c = workingTruss.NodePositions[face.nodeC];

                if (designer.selectedFaces.Contains(i))
                {
                    Handles.color = designer.selectedFaceColor;
                    Handles.DrawAAConvexPolygon(a, b, c);
                    Handles.color = designer.faceColor;
                }
                else
                {
                    Handles.DrawAAConvexPolygon(a, b, c);
                }

                if (designer.showFaceIndices) Handles.Label((a + b + c) / 3f, i.ToString());
            }
        }

        private void DrawTransformHandles()
        {
            if (designer.selectedNodes.Count == 0 || !designer.showTransformHandles) return;
            UpdateSelectionCenter();

            Handles.matrix = Matrix4x4.identity;
            Vector3 worldCenter = targetSoftBody.transform.TransformPoint(selectionCenter);

            EditorGUI.BeginChangeCheck();
            Vector3 newWorldCenter = Handles.PositionHandle(worldCenter, Quaternion.identity);

            if (EditorGUI.EndChangeCheck())
            {
                Vector3 worldOffset = newWorldCenter - worldCenter;
                Vector3 localOffset = targetSoftBody.transform.InverseTransformVector(worldOffset);
                MoveSelectedNodes(localOffset);
                UpdateSelectionCenter();
            }
            Handles.matrix = targetSoftBody.transform.localToWorldMatrix;
        }

        private void AutoConnectSelectedNodes()
        {
            if (designer.selectedNodes.Count < 2) return;
            var beams = workingTruss.GetTrussBeams();
            int created = 0;

            for (int i = 0; i < designer.selectedNodes.Count; i++)
            {
                for (int j = i + 1; j < designer.selectedNodes.Count; j++)
                {
                    int nodeA = designer.selectedNodes[i];
                    int nodeB = designer.selectedNodes[j];
                    bool exists = beams.Any(b => (b.nodeA == nodeA && b.nodeB == nodeB) || (b.nodeA == nodeB && b.nodeB == nodeA));
                    if (!exists)
                    {
                        Vector3 pA = workingTruss.NodePositions[nodeA];
                        Vector3 pB = workingTruss.NodePositions[nodeB];
                        beams.Add(new Beam(nodeA, nodeB, designer.defaultBeamCompliance, designer.defaultBeamDamping, Vector3.Distance(pA, pB)));
                        created++;
                    }
                }
            }
            if (created > 0)
            {
                EditorUtility.SetDirty(workingTruss);
                if (designer.autoApplyChanges) ApplyTrussToTarget();
            }
        }

        private void AutoGenerateFaces()
        {
            Debug.Log("Auto Generate Faces placeholder.");
        }

        private Vector3 GetWorldPositionFromMouse(Vector2 mousePosition)
        {
            Ray ray = HandleUtility.GUIPointToWorldRay(mousePosition);
            Plane plane = new Plane(targetSoftBody.transform.up, targetSoftBody.transform.position);
            if (plane.Raycast(ray, out float dist)) return ray.GetPoint(dist);
            return targetSoftBody.transform.position;
        }

private int FindClosestNode(Vector3 worldPos)
{
    if (workingTruss.NodePositions == null) return -1;
    
    Camera sceneCamera = SceneView.lastActiveSceneView?.camera;
    if (sceneCamera == null) return -1;
    
    Ray ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
    
    int closestNode = -1;
    float minDistance = float.MaxValue;
    float maxDistance = 1000f;
    
    for (int i = 0; i < workingTruss.NodePositions.Length; i++)
    {
        Vector3 nodeWorldPos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
        
        // Distance from ray to node (same logic as solver)
        float distance = Vector3.Cross(ray.direction, nodeWorldPos - ray.origin).magnitude;
        
        if (distance < designer.nodeDisplaySize && distance < minDistance)
        {
            float t = Vector3.Dot(nodeWorldPos - ray.origin, ray.direction);
            if (t > 0 && t < maxDistance)
            {
                minDistance = distance;
                closestNode = i;
            }
        }
    }
    
    return closestNode;
}

        private void CreateNodeAtPosition(Vector3 worldPos)
        {
            Vector3 local = targetSoftBody.transform.InverseTransformPoint(worldPos);
            var list = new List<Vector3>(workingTruss.NodePositions ?? new Vector3[0]);
            list.Add(local);
            workingTruss.SetNodePositions(list.ToArray());

            var masses = new List<float>(workingTruss.NodeMasses);
            masses.Add(designer.nodeCreationMass);
            workingTruss.SetNodeMasses(masses);

            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private void SelectNodeAtPosition(Vector3 worldPos, bool multi)
        {
            int idx = FindClosestNode(worldPos);
            if (idx == -1)
            {
                if (!multi) designer.selectedNodes.Clear();
                return;
            }

            if (!multi) designer.selectedNodes.Clear();

            if (designer.selectedNodes.Contains(idx)) designer.selectedNodes.Remove(idx);
            else designer.selectedNodes.Add(idx);

            UpdateSelectionCenter();
        }

        private void SelectBeamAtPosition(Vector3 worldPos, bool multi)
        {
            var beams = workingTruss.GetTrussBeams();
            float minD = 0.5f;
            int best = -1;

            for (int i = 0; i < beams.Count; i++)
            {
                Vector3 a = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beams[i].nodeA]);
                Vector3 b = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beams[i].nodeB]);
                float d = HandleUtility.DistancePointToLineSegment(worldPos, a, b);
                if (d < minD)
                {
                    minD = d;
                    best = i;
                }
            }

            if (best != -1)
            {
                if (!multi) designer.selectedBeams.Clear();
                if (designer.selectedBeams.Contains(best)) designer.selectedBeams.Remove(best);
                else designer.selectedBeams.Add(best);
            }
            else if (!multi) designer.selectedBeams.Clear();
        }

        private void SelectFaceAtPosition(Vector3 worldPos, bool multi)
        {
            var faces = workingTruss.GetTrussFaces();
            float minD = 0.5f;
            int best = -1;

            for (int i = 0; i < faces.Count; i++)
            {
                Vector3 a = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[faces[i].nodeA]);
                Vector3 b = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[faces[i].nodeB]);
                Vector3 c = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[faces[i].nodeC]);
                Vector3 center = (a + b + c) / 3f;
                float d = Vector3.Distance(worldPos, center);
                if (d < minD)
                {
                    minD = d;
                    best = i;
                }
            }

            if (best != -1)
            {
                if (!multi) designer.selectedFaces.Clear();
                if (designer.selectedFaces.Contains(best)) designer.selectedFaces.Remove(best);
                else designer.selectedFaces.Add(best);
            }
            else if (!multi) designer.selectedFaces.Clear();
        }

        private void HandleBeamCreation(Vector3 worldPos)
        {
            int idx = FindClosestNode(worldPos);
            if (idx == -1) return;

            if (beamStartNode == -1)
            {
                beamStartNode = idx;
            }
            else if (beamStartNode != idx)
            {
                CreateBeam(beamStartNode, idx);
                beamStartNode = -1;
                isCreatingBeam = false;
            }
        }

        private void CreateBeam(int a, int b)
        {
            Vector3 pA = workingTruss.NodePositions[a];
            Vector3 pB = workingTruss.NodePositions[b];
            workingTruss.GetTrussBeams().Add(new Beam(
                a, b,
                designer.defaultBeamCompliance,
                designer.defaultBeamDamping,
                Vector3.Distance(pA, pB),
                deformationScale: designer.defaultDeformationScale,  // â† FIX THIS!
                maxDeformation: designer.defaultMaxDeformation,
                plasticityThreshold: designer.defaultPlasticityThreshold,
                plasticityRate: designer.defaultPlasticityRate
            ));
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private void HandleFaceCreation(Vector3 worldPos)
        {
            int idx = FindClosestNode(worldPos);
            if (idx == -1) return;

            if (!faceCreationNodes.Contains(idx))
            {
                faceCreationNodes.Add(idx);
                if (faceCreationNodes.Count == 3)
                {
                    workingTruss.AddFace(new Face(faceCreationNodes[0], faceCreationNodes[1], faceCreationNodes[2]));
                    faceCreationNodes.Clear();
                    isCreatingFace = false;
                    EditorUtility.SetDirty(workingTruss);
                    if (designer.autoApplyChanges) ApplyTrussToTarget();
                }
            }
        }

        private void SetNodePosition(int idx, Vector3 pos)
        {
            workingTruss.NodePositions[idx] = pos;
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private void StartNodeTransform(int idx)
        {
            isTransformingNode = true;
            transformingNodeIndex = idx;
        }

        private void UpdateNodeTransform(Vector3 worldPos)
        {
            if (transformingNodeIndex != -1)
            {
                workingTruss.NodePositions[transformingNodeIndex] = targetSoftBody.transform.InverseTransformPoint(worldPos);
                EditorUtility.SetDirty(workingTruss);
                if (designer.autoApplyChanges) ApplyTrussToTarget();
            }
        }

        private void EndNodeTransform()
        {
            isTransformingNode = false;
            transformingNodeIndex = -1;
        }

        private void PinSelectedNodes(bool pin)
        {
            var list = new List<int>(workingTruss.PinnedNodes);
            foreach (int i in designer.selectedNodes)
            {
                if (pin && !list.Contains(i)) list.Add(i);
                else if (!pin && list.Contains(i)) list.Remove(i);
            }
            workingTruss.SetPinnedNodes(list);
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private void SetSelectedNodesMass(float m)
        {
            foreach (int i in designer.selectedNodes)
            {
                if (i < workingTruss.NodeMasses.Count) workingTruss.NodeMasses[i] = m;
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void DeleteSelectedNodes()
        {
            Debug.LogWarning("Node deletion requires complex re-indexing. Clear truss and start over or remove manually from asset.");
        }

        private void DeleteSelectedBeams()
        {
            var beams = workingTruss.GetTrussBeams();
            designer.selectedBeams.Sort();
            for (int i = designer.selectedBeams.Count - 1; i >= 0; i--)
            {
                int idx = designer.selectedBeams[i];
                if (idx < beams.Count) beams.RemoveAt(idx);
            }
            designer.selectedBeams.Clear();
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private void DeleteSelectedFaces()
        {
            var faces = workingTruss.GetTrussFaces();
            designer.selectedFaces.Sort();
            for (int i = designer.selectedFaces.Count - 1; i >= 0; i--)
            {
                int idx = designer.selectedFaces[i];
                if (idx < faces.Count) workingTruss.RemoveFace(idx);
            }
            designer.selectedFaces.Clear();
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private void SelectAllNodes()
        {
            designer.selectedNodes.Clear();
            for (int i = 0; i < workingTruss.NodePositions.Length; i++) designer.selectedNodes.Add(i);
        }

        private void SelectAllBeams()
        {
            designer.selectedBeams.Clear();
            for (int i = 0; i < workingTruss.GetTrussBeams().Count; i++) designer.selectedBeams.Add(i);
        }

        private void SelectAllFaces()
        {
            designer.selectedFaces.Clear();
            for (int i = 0; i < workingTruss.GetTrussFaces().Count; i++) designer.selectedFaces.Add(i);
        }

        private void UpdateSelectionCenter()
        {
            if (designer.selectedNodes.Count == 0) return;
            Vector3 avg = Vector3.zero;
            foreach (int i in designer.selectedNodes) avg += workingTruss.NodePositions[i];
            selectionCenter = avg / designer.selectedNodes.Count;
        }

        private void MoveSelectedNodes(Vector3 offset)
        {
            foreach (int i in designer.selectedNodes)
            {
                workingTruss.NodePositions[i] += offset;
            }
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private void GetBeamPresetValues(SoftBodyDesigner.BeamPreset preset, out float c, out float d, out float thresh, out float rate, out float defScale, out float maxDef)
        {
            // Set default values first
            c = 0.01f; d = 0.3f; thresh = 0.02f; rate = 0.5f; defScale = 1.0f; maxDef = 0.8f;

            // Apply preset overrides
            if (preset == SoftBodyDesigner.BeamPreset.Metal)
            {
                c = 0.001f; d = 0.1f; thresh = 0.05f; rate = 0.1f; defScale = 0.5f; maxDef = 0.2f;
            }
            else if (preset == SoftBodyDesigner.BeamPreset.Rubber)
            {
                c = 0.1f; d = 0.5f; thresh = 0.01f; rate = 0.8f; defScale = 2.0f; maxDef = 2.0f;
            }
        }

        private void ToggleNodeCreation() => isCreatingNode = !isCreatingNode;
        private void ToggleBeamCreation() => isCreatingBeam = !isCreatingBeam;
        private void ToggleFaceCreation() => isCreatingFace = !isCreatingFace;
        private void CancelCurrentOperation() { isCreatingNode = false; isCreatingBeam = false; isCreatingFace = false; beamStartNode = -1; faceCreationNodes.Clear(); }
        private void DeleteSelectedElements() { /* Combined delete logic if needed */ }
    }
}
#endregion