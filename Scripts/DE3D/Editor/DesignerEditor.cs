/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

namespace DynamicEngine
{
    [CustomEditor(typeof(Designer))]
    public class DesignerEditor : UnityEditor.Editor
    {
        #region Constants
        const int TAB_TRUSS = 0;
        const int TAB_NODES = 1;
        const int TAB_BEAMS = 2;
        const int TAB_FACES = 3;
        const int TAB_NODESETS = 4;
        const int TAB_LINKSETS = 5;
        #endregion

        #region References
        private Designer designer;
        private SoftBody targetSoftBody;
        private Truss workingTruss;
        #endregion

        #region Editor State
        private int currentTab = 0;
        private GUIStyle roundedButtonStyle;
        private GUIStyle roundedButtonActiveStyle;
        private GUIStyle sectionStyle;
        #endregion

        #region Tool State
        private bool isCreatingNode = false;
        private bool isCreatingBeam = false;
        private bool isCreatingFace = false;
        private int beamStartNode = -1;
        private List<int> faceCreationNodes = new List<int>();
        #endregion

        #region Transform State
        private bool isTransformingNode = false;
        private int transformingNodeIndex = -1;
        private Vector3 selectionCenter = Vector3.zero;
        private Vector3 ghostNodePosition = Vector3.zero;
        private bool showGhostNode = false;
        #endregion

        #region Unity Callbacks
        private void OnEnable()
        {
            designer = (Designer)target;
            targetSoftBody = designer.TargetSoftBody;

            if (targetSoftBody != null)
            {
                workingTruss = targetSoftBody.GetTrussAsset();
                if (workingTruss != null)
                {
                    designer.LoadDefaultsFromTruss(workingTruss);
                }
            }

            if (designer.selectedNodes == null) designer.selectedNodes = new List<int>();
            if (designer.selectedBeams == null) designer.selectedBeams = new List<int>();
            if (designer.selectedFaces == null) designer.selectedFaces = new List<int>();
        }
        private void OnDisable()
        {
            // Restore Unity's transform tools when editor is disabled
            Tools.hidden = false;
        }

        public override void OnInspectorGUI()
        {
            // Ensure styles are initialized before drawing
            InitializeStyles();

            serializedObject.Update();

            if (targetSoftBody == null)
            {
                EditorGUILayout.HelpBox("No SoftBody component found on this GameObject.", MessageType.Error);
                return;
            }

            if (workingTruss != targetSoftBody.GetTrussAsset())
            {
                workingTruss = targetSoftBody.GetTrussAsset();
                if (workingTruss != null)
                {
                    designer.LoadDefaultsFromTruss(workingTruss);
                }
            }

            DrawHeader();

            string[] tabs = { "Truss Info", "Nodes", "Beams", "Faces", "Node Sets", "Link Sets" };
            currentTab = GUILayout.Toolbar(currentTab, tabs, GUILayout.Height(30));

            EditorGUILayout.Space(10);

            UpdateDesignerModeFromTab();

            if (workingTruss != null)
            {
                switch (currentTab)
                {
                    case TAB_TRUSS: DrawTrussTab(); break;
                    case TAB_NODES: DrawNodesTab(); break;
                    case TAB_BEAMS: DrawBeamsTab(); break;
                    case TAB_FACES: DrawFacesTab(); break;
                    case TAB_NODESETS: DrawNodeSetsTab(); break;
                    case TAB_LINKSETS: DrawLinkSetsTab(); break;
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

        public void OnSceneGUI()
        {
            if (designer == null || targetSoftBody == null || workingTruss == null || workingTruss.NodePositions == null) return;

            Event e = Event.current;
            int controlID = GUIUtility.GetControlID(FocusType.Passive);
            HandleUtility.AddDefaultControl(controlID);

            Vector3 mouseWorldPos = GetWorldPositionFromMouse(e.mousePosition);

            if (isCreatingNode)
            {
                ghostNodePosition = mouseWorldPos;
                showGhostNode = true;
                SceneView.RepaintAll();
            }
            else
            {
                showGhostNode = false;
            }

            DrawSceneVisualization();

            bool showHandles = currentTab == TAB_NODES && designer.selectedNodes.Count > 0 &&
                               !isCreatingNode && !isCreatingBeam && !isCreatingFace;

            if (showHandles)
            {
                Tools.hidden = true;
                HandleTransformGizmo();
            }
            else
            {
                Tools.hidden = false;
            }

            HandleSceneInput(e, mouseWorldPos);

            if (e.type == EventType.MouseDown || e.type == EventType.MouseUp || e.type == EventType.KeyDown)
            {
                SceneView.RepaintAll();
                Repaint();
            }
        }
        #endregion

        #region Styles
        private void InitializeStyles()
        {
            // Only initialize if the style is missing or its internal textures were flushed
            if (roundedButtonStyle != null && roundedButtonStyle.normal.background != null)
                return;

            // "miniButton" is the standard Unity style for rounded, smaller buttons
            // We use it as a base to ensure the rounded corner textures are applied
            roundedButtonStyle = new GUIStyle(EditorStyles.miniButton)
            {
                padding = new RectOffset(10, 10, 5, 5),
                margin = new RectOffset(4, 4, 4, 4),
                fontSize = 11,
                fontStyle = FontStyle.Normal,
                alignment = TextAnchor.MiddleCenter,
                fixedHeight = 24 // Standard height for inspector buttons
            };

            // Create the active/toggled version
            roundedButtonActiveStyle = new GUIStyle(roundedButtonStyle)
            {
                fontStyle = FontStyle.Bold,
                // Apply a distinct color to the text when the button/mode is active
                normal = { textColor = new Color(0.3f, 0.8f, 0.3f) }
            };

            // Set the background to the 'active' state of the standard button
            roundedButtonActiveStyle.normal.background = EditorStyles.miniButton.active.background;

            if (sectionStyle == null)
            {
                sectionStyle = new GUIStyle()
                {
                    padding = new RectOffset(8, 8, 8, 8),
                    margin = new RectOffset(0, 0, 4, 4)
                };
            }
        }

        private GUIStyle GetButtonStyle(bool isActive)
        {
            InitializeStyles();
            return isActive ? roundedButtonActiveStyle : roundedButtonStyle;
        }

        private GUIStyle GetRoundedButtonStyle()
        {
            InitializeStyles();
            return roundedButtonStyle;
        }
        #endregion

        #region Tab Drawing
        private new void DrawHeader()
        {
            EditorGUILayout.Space(5);
            GUIStyle headerStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                fontSize = 14,
                alignment = TextAnchor.MiddleCenter
            };
            EditorGUILayout.LabelField("Soft Body Designer", headerStyle);
            EditorGUILayout.Space(5);
        }

        private void DrawTrussTab()
        {
            EditorGUILayout.LabelField("Truss Information", EditorStyles.boldLabel);
            EditorGUILayout.Space(5);

            GUI.enabled = false;
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
            GUI.enabled = true;

            EditorGUILayout.Space(10);
            DrawVisualizationSettings();
        }

        private void DrawVisualizationSettings()
        {
            EditorGUILayout.LabelField("Visualization Settings", EditorStyles.boldLabel);
            EditorGUILayout.Space(5);

            designer.enableDepthCulling = EditorGUILayout.Toggle("Enable Depth Culling", designer.enableDepthCulling);

            if (designer.enableDepthCulling)
            {
                EditorGUI.indentLevel++;
                designer.showOccludedElements = EditorGUILayout.Toggle("Show Occluded Elements", designer.showOccludedElements);
                EditorGUILayout.HelpBox("Shows faded elements behind surfaces", MessageType.Info);

                if (designer.showOccludedElements)
                {
                    designer.occludedAlpha = EditorGUILayout.Slider("Occluded Transparency", designer.occludedAlpha, 0.05f, 0.5f);
                }
                EditorGUI.indentLevel--;
            }
            else
            {
                EditorGUILayout.HelpBox("Elements will show through all geometry (X-ray mode)", MessageType.Info);
            }
        }

        private void DrawNodesTab()
        {
            int count = designer.selectedNodes.Count;

            GUIStyle rightAlignStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                alignment = TextAnchor.MiddleRight,
                normal = { textColor = count > 0 ? new Color(0.3f, 0.8f, 0.3f) : Color.gray }
            };
            EditorGUILayout.LabelField(count == 0 ? "No Nodes Selected" : $"{count} Nodes Selected", rightAlignStyle);
            EditorGUILayout.Space(5);

            if (count > 0)
            {
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

                EditorGUILayout.Space(5);
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Pin", GetRoundedButtonStyle())) PinSelectedNodes(true);
                if (GUILayout.Button("Unpin", GetRoundedButtonStyle())) PinSelectedNodes(false);
                EditorGUILayout.EndHorizontal();

                if (GUILayout.Button("Set Mass", GetRoundedButtonStyle())) SetSelectedNodesMass(designer.nodeCreationMass);

                // NEW: Duplicate button
                EditorGUILayout.Space(5);
                if (GUILayout.Button($"Duplicate ({count} node{(count == 1 ? "" : "s")})", GetRoundedButtonStyle(), GUILayout.Height(30)))
                    DuplicateSelectedNodes();
            }
            else
            {
                EditorGUILayout.HelpBox("Select nodes in Scene View to edit.", MessageType.Info);
            }

            EditorGUILayout.Space(10);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button(isCreatingNode ? "Cancel Create" : "Create Node", GetButtonStyle(isCreatingNode), GUILayout.Height(35)))
                ToggleNodeCreation();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = count > 0;
            if (GUILayout.Button("Delete", GetRoundedButtonStyle())) DeleteSelectedNodes();
            GUI.enabled = true;
            if (GUILayout.Button("Select All", GetRoundedButtonStyle())) SelectAllNodes();
            if (GUILayout.Button("Clear", GetRoundedButtonStyle())) { designer.selectedNodes.Clear(); SceneView.RepaintAll(); }
            EditorGUILayout.EndHorizontal();

            if (isCreatingNode)
            {
                EditorGUILayout.Space(5);
                EditorGUILayout.HelpBox("Click in scene to place new nodes.", MessageType.None);
            }

            // NEW: Info box for transform handles (always shown when nodes selected)
            if (count > 0)
            {
                EditorGUILayout.Space(10);
                EditorGUILayout.HelpBox("Use transform handles in Scene View to move selected nodes.", MessageType.Info);
            }
        }

        private void DrawBeamsTab()
        {
            int count = designer.selectedBeams.Count;

            GUIStyle rightAlignStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                alignment = TextAnchor.MiddleRight,
                normal = { textColor = count > 0 ? new Color(0.3f, 0.8f, 0.3f) : Color.gray }
            };
            EditorGUILayout.LabelField(count == 0 ? "No Beams Selected" : $"{count} Beams Selected", rightAlignStyle);
            EditorGUILayout.Space(5);

            EditorGUILayout.LabelField("Beam Properties", EditorStyles.boldLabel);

            EditorGUI.BeginChangeCheck();
            designer.selectedBeamPreset = (Designer.BeamPreset)EditorGUILayout.EnumPopup("Preset", designer.selectedBeamPreset);
            if (EditorGUI.EndChangeCheck() && designer.selectedBeamPreset != Designer.BeamPreset.Custom)
            {
                ApplyBeamPreset(designer.selectedBeamPreset);
            }

            EditorGUI.BeginChangeCheck();
            designer.defaultBeamCompliance = EditorGUILayout.FloatField("Compliance", designer.defaultBeamCompliance);
            designer.defaultBeamDamping = EditorGUILayout.Slider("Damping", designer.defaultBeamDamping, 0f, 1f);

            EditorGUILayout.Space(5);
            EditorGUILayout.LabelField("Plasticity Settings", EditorStyles.miniLabel);
            designer.defaultPlasticityThreshold = EditorGUILayout.Slider("Plastic Threshold", designer.defaultPlasticityThreshold, 0f, 0.5f);
            designer.defaultPlasticityRate = EditorGUILayout.Slider("Plastic Rate", designer.defaultPlasticityRate, 0f, 100f);
            designer.defaultMaxDeformation = EditorGUILayout.Slider("Max Deformation", designer.defaultMaxDeformation, 0f, 3f);

            if (EditorGUI.EndChangeCheck() && count > 0)
            {
                ApplyBeamPropertiesToSelection();
            }

            EditorGUILayout.Space(10);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button(isCreatingBeam ? "Cancel Create" : "Create Beam", GetButtonStyle(isCreatingBeam), GUILayout.Height(35)))
                ToggleBeamCreation();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Auto Connect", GetRoundedButtonStyle())) AutoConnectSelectedNodes();
            GUI.enabled = count > 0;
            if (GUILayout.Button("Delete", GetRoundedButtonStyle())) DeleteSelectedBeams();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = true;
            if (GUILayout.Button("Select All", GetRoundedButtonStyle())) SelectAllBeams();
            if (GUILayout.Button("Clear", GetRoundedButtonStyle())) { designer.selectedBeams.Clear(); SceneView.RepaintAll(); }
            EditorGUILayout.EndHorizontal();

            if (isCreatingBeam)
            {
                EditorGUILayout.Space(5);
                EditorGUILayout.HelpBox("Click two nodes to connect them.", MessageType.None);
            }
        }

        private void DrawFacesTab()
        {
            int count = designer.selectedFaces.Count;

            GUIStyle rightAlignStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                alignment = TextAnchor.MiddleRight,
                normal = { textColor = count > 0 ? new Color(0.3f, 0.8f, 0.3f) : Color.gray }
            };
            EditorGUILayout.LabelField(count == 0 ? "No Faces Selected" : $"{count} Faces Selected", rightAlignStyle);
            EditorGUILayout.Space(5);

            if (count == 0)
            {
                EditorGUILayout.HelpBox("Select faces to edit.", MessageType.Info);
            }

            EditorGUILayout.Space(10);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button(isCreatingFace ? "Cancel Create" : "Create Face", GetButtonStyle(isCreatingFace), GUILayout.Height(35)))
                ToggleFaceCreation();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = count > 0;
            if (GUILayout.Button("Delete", GetRoundedButtonStyle())) DeleteSelectedFaces();
            GUI.enabled = true;
            if (GUILayout.Button("Select All", GetRoundedButtonStyle())) SelectAllFaces();
            if (GUILayout.Button("Clear", GetRoundedButtonStyle())) { designer.selectedFaces.Clear(); SceneView.RepaintAll(); }
            EditorGUILayout.EndHorizontal();

            if (isCreatingFace)
            {
                EditorGUILayout.Space(5);
                string msg = faceCreationNodes.Count > 0 ?
                    $"{faceCreationNodes.Count}/3 Nodes Selected" : "Click 3 nodes to form a face.";
                EditorGUILayout.HelpBox(msg, MessageType.None);
            }
        }

        private void DrawNodeSetsTab()
        {
            if (workingTruss == null) return;

            var nodeSets = workingTruss.GetNodeSets();

            EditorGUILayout.LabelField("Node Sets", EditorStyles.boldLabel);
            EditorGUILayout.Space(5);

            if (nodeSets.Count == 0)
            {
                EditorGUILayout.HelpBox("No node sets created yet.", MessageType.Info);
            }
            else
            {
                DrawNodeSetList(nodeSets);
            }

            EditorGUILayout.Space(10);
            DrawNodeSetCreation();

            EditorGUILayout.Space(10);
            DrawNodeSetVisualization();
        }

        private void DrawNodeSetList(List<NodeSet> nodeSets)
        {
            for (int i = 0; i < nodeSets.Count; i++)
            {
                var nodeSet = nodeSets[i];

                EditorGUILayout.BeginHorizontal();

                GUI.color = nodeSet.color;
                GUILayout.Box("", GUILayout.Width(20), GUILayout.Height(20));
                GUI.color = Color.white;

                EditorGUILayout.BeginVertical();
                EditorGUILayout.LabelField(nodeSet.name, EditorStyles.boldLabel);
                EditorGUILayout.LabelField($"{nodeSet.nodeIndices.Count} nodes", EditorStyles.miniLabel);
                EditorGUILayout.EndVertical();

                if (GUILayout.Button(designer.selectedNodeSetIndex == i ? "Selected" : "Select", GetRoundedButtonStyle(), GUILayout.Width(80)))
                {
                    SelectNodeSet(i, nodeSet);
                }

                GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
                if (GUILayout.Button("X", GUILayout.Width(30)))
                {
                    DeleteNodeSet(nodeSet);
                }
                GUI.backgroundColor = Color.white;

                EditorGUILayout.EndHorizontal();

                if (designer.selectedNodeSetIndex == i)
                {
                    DrawNodeSetEditor(nodeSet);
                }

                EditorGUILayout.Space(5);
            }
        }

        private void DrawNodeSetEditor(NodeSet nodeSet)
        {
            EditorGUI.indentLevel++;

            EditorGUI.BeginChangeCheck();
            nodeSet.name = EditorGUILayout.TextField("Name", nodeSet.name);
            nodeSet.color = EditorGUILayout.ColorField("Color", nodeSet.color);
            nodeSet.isVisible = EditorGUILayout.Toggle("Visible", nodeSet.isVisible);

            if (EditorGUI.EndChangeCheck())
            {
                EditorUtility.SetDirty(workingTruss);
            }

            EditorGUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Add Selected", GetRoundedButtonStyle()) && designer.selectedNodes.Count > 0)
            {
                AddNodesToSet(nodeSet);
            }

            if (GUILayout.Button("Remove Selected", GetRoundedButtonStyle()) && designer.selectedNodes.Count > 0)
            {
                RemoveNodesFromSet(nodeSet);
            }
            EditorGUILayout.EndHorizontal();

            EditorGUI.indentLevel--;
        }

        private void DrawNodeSetCreation()
        {
            EditorGUILayout.LabelField("Create New Node Set", EditorStyles.boldLabel);
            designer.nodeSetCreationName = EditorGUILayout.TextField("Name", designer.nodeSetCreationName);
            designer.nodeSetCreationColor = EditorGUILayout.ColorField("Color", designer.nodeSetCreationColor);

            GUI.enabled = designer.selectedNodes.Count > 0 && !string.IsNullOrEmpty(designer.nodeSetCreationName);
            if (GUILayout.Button($"Create from Selected ({designer.selectedNodes.Count} nodes)", GetRoundedButtonStyle(), GUILayout.Height(30)))
            {
                CreateNodeSetFromSelection();
            }
            GUI.enabled = true;
        }

        private void DrawNodeSetVisualization()
        {
            EditorGUILayout.LabelField("Visualization", EditorStyles.boldLabel);
            designer.showNodeSetLabels = EditorGUILayout.Toggle("Show Labels", designer.showNodeSetLabels);
            if (designer.showNodeSetLabels)
            {
                designer.nodeSetLabelOffset = EditorGUILayout.Slider("Label Offset", designer.nodeSetLabelOffset, 0f, 1f);
            }
        }

        private void DrawLinkSetsTab()
        {
            if (workingTruss == null) return;

            var linkSets = workingTruss.GetLinkSets();

            EditorGUILayout.LabelField("Link Sets", EditorStyles.boldLabel);
            EditorGUILayout.Space(5);

            if (linkSets.Count == 0)
            {
                EditorGUILayout.HelpBox("No link sets created yet.", MessageType.Info);
            }
            else
            {
                DrawLinkSetList(linkSets);
            }

            EditorGUILayout.Space(10);
            DrawLinkSetCreation();
        }

        private void DrawLinkSetList(List<LinkSet> linkSets)
        {
            for (int i = 0; i < linkSets.Count; i++)
            {
                var linkSet = linkSets[i];

                EditorGUILayout.BeginHorizontal();

                GUI.color = linkSet.color;
                GUILayout.Box("", GUILayout.Width(20), GUILayout.Height(20));
                GUI.color = Color.white;

                EditorGUILayout.BeginVertical();
                EditorGUILayout.LabelField(linkSet.name, EditorStyles.boldLabel);
                EditorGUILayout.LabelField($"{linkSet.linkIndices.Count} links", EditorStyles.miniLabel);
                EditorGUILayout.EndVertical();

                if (GUILayout.Button(designer.selectedLinkSetIndex == i ? "Selected" : "Select", GetRoundedButtonStyle(), GUILayout.Width(80)))
                {
                    SelectLinkSet(i, linkSet);
                }

                GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
                if (GUILayout.Button("X", GUILayout.Width(30)))
                {
                    DeleteLinkSet(linkSet);
                }
                GUI.backgroundColor = Color.white;

                EditorGUILayout.EndHorizontal();

                if (designer.selectedLinkSetIndex == i)
                {
                    DrawLinkSetEditor(linkSet);
                }

                EditorGUILayout.Space(5);
            }
        }

        private void DrawLinkSetEditor(LinkSet linkSet)
        {
            EditorGUI.indentLevel++;

            EditorGUI.BeginChangeCheck();
            linkSet.name = EditorGUILayout.TextField("Name", linkSet.name);
            linkSet.color = EditorGUILayout.ColorField("Color", linkSet.color);
            linkSet.isVisible = EditorGUILayout.Toggle("Visible", linkSet.isVisible);

            if (EditorGUI.EndChangeCheck())
            {
                EditorUtility.SetDirty(workingTruss);
            }

            EditorGUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Add Selected", GetRoundedButtonStyle()) && designer.selectedBeams.Count > 0)
            {
                AddLinksToSet(linkSet);
            }

            if (GUILayout.Button("Remove Selected", GetRoundedButtonStyle()) && designer.selectedBeams.Count > 0)
            {
                RemoveLinksFromSet(linkSet);
            }
            EditorGUILayout.EndHorizontal();

            EditorGUI.indentLevel--;
        }

        private void DrawLinkSetCreation()
        {
            EditorGUILayout.LabelField("Create New Link Set", EditorStyles.boldLabel);
            designer.linkSetCreationName = EditorGUILayout.TextField("Name", designer.linkSetCreationName);
            designer.linkSetCreationColor = EditorGUILayout.ColorField("Color", designer.linkSetCreationColor);

            GUI.enabled = designer.selectedBeams.Count > 0 && !string.IsNullOrEmpty(designer.linkSetCreationName);
            if (GUILayout.Button($"Create from Selected ({designer.selectedBeams.Count} links)", GetRoundedButtonStyle(), GUILayout.Height(30)))
            {
                CreateLinkSetFromSelection();
            }
            GUI.enabled = true;
        }
        #endregion

        #region Scene Visualization
        private void DrawSceneVisualization()
        {
            Handles.matrix = targetSoftBody.transform.localToWorldMatrix;

            switch (currentTab)
            {
                case TAB_NODES:
                    DrawNodes();
                    if (showGhostNode && isCreatingNode) DrawGhostNode(ghostNodePosition);
                    break;
                case TAB_BEAMS:
                    DrawNodes();
                    DrawBeams();
                    break;
                case TAB_FACES:
                    DrawNodes();
                    DrawFaces();
                    break;
                case TAB_NODESETS:
                    DrawNodes();
                    break;
                case TAB_LINKSETS:
                    DrawNodes();
                    DrawBeams();
                    break;
            }
        }

        private void DrawNodes()
        {
            if (workingTruss.NodePositions == null) return;
            var pinnedNodes = workingTruss.PinnedNodes;

            DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.LessEqual, () =>
            {
                for (int i = 0; i < workingTruss.NodePositions.Length; i++)
                {
                    DrawNode(i, pinnedNodes, designer.nodeDisplaySize, 1f);
                }
            });

            if (designer.enableDepthCulling && designer.showOccludedElements)
            {
                DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.Greater, () =>
                {
                    for (int i = 0; i < workingTruss.NodePositions.Length; i++)
                    {
                        DrawNode(i, pinnedNodes, designer.nodeDisplaySize * 0.8f, designer.occludedAlpha);
                    }
                });
            }

            Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
        }

        private void DrawNode(int index, List<int> pinnedNodes, float size, float alpha)
        {
            Vector3 position = workingTruss.NodePositions[index];
            Color color = GetNodeColor(index, pinnedNodes);
            color.a = alpha;

            if (isTransformingNode && transformingNodeIndex == index)
            {
                color = Color.yellow;
                size *= 1.2f;
            }

            Handles.color = color;
            Handles.SphereHandleCap(0, position, Quaternion.identity, size, EventType.Repaint);
        }

        private Color GetNodeColor(int index, List<int> pinnedNodes)
        {
            if (designer.selectedNodes.Contains(index)) return designer.selectedNodeColor;
            if (pinnedNodes.Contains(index)) return designer.pinnedNodeColor;
            return designer.nodeColor;
        }

        private void DrawBeams()
        {
            var beams = workingTruss.GetTrussBeams();
            if (beams == null || workingTruss.NodePositions == null) return;

            DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.LessEqual, () =>
            {
                DrawBeamList(beams, designer.beamLineThickness, 1f);
            });

            if (designer.enableDepthCulling && designer.showOccludedElements)
            {
                DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.Greater, () =>
                {
                    DrawBeamList(beams, designer.beamLineThickness * 0.5f, designer.occludedAlpha);
                });
            }

            Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
        }

        private void DrawBeamList(List<Beam> beams, float thickness, float alpha)
        {
            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                if (beam.nodeA >= workingTruss.NodePositions.Length || beam.nodeB >= workingTruss.NodePositions.Length)
                    continue;

                Vector3 posA = workingTruss.NodePositions[beam.nodeA];
                Vector3 posB = workingTruss.NodePositions[beam.nodeB];

                Color color = designer.selectedBeams.Contains(i) ? designer.selectedBeamColor : designer.beamColor;
                color.a = alpha;
                Handles.color = color;

                if (thickness > 1.0f)
                    Handles.DrawAAPolyLine(thickness, posA, posB);
                else
                    Handles.DrawLine(posA, posB);
            }
        }

        private void DrawFaces()
        {
            var faces = workingTruss.GetTrussFaces();
            if (faces == null || workingTruss.NodePositions == null) return;

            DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.LessEqual, () =>
            {
                DrawFaceList(faces, 1f);
            });

            if (designer.enableDepthCulling && designer.showOccludedElements)
            {
                DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.Greater, () =>
                {
                    DrawFaceList(faces, designer.occludedAlpha * 0.5f);
                });
            }

            Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
        }

        private void DrawFaceList(List<Face> faces, float alpha)
        {
            for (int i = 0; i < faces.Count; i++)
            {
                var face = faces[i];
                if (face.nodeA >= workingTruss.NodePositions.Length ||
                    face.nodeB >= workingTruss.NodePositions.Length ||
                    face.nodeC >= workingTruss.NodePositions.Length)
                    continue;

                Vector3 a = workingTruss.NodePositions[face.nodeA];
                Vector3 b = workingTruss.NodePositions[face.nodeB];
                Vector3 c = workingTruss.NodePositions[face.nodeC];

                Color color = designer.selectedFaces.Contains(i) ? designer.selectedFaceColor : designer.faceColor;
                color.a *= alpha;
                Handles.color = color;
                Handles.DrawAAConvexPolygon(a, b, c);
            }
        }

        private void DrawGhostNode(Vector3 worldPos)
        {
            Vector3 localPos = targetSoftBody.transform.InverseTransformPoint(worldPos);

            Color ghostColor = designer.selectedNodeColor;
            ghostColor.a = 0.5f;
            Handles.color = ghostColor;
            Handles.matrix = targetSoftBody.transform.localToWorldMatrix;

            float pulseSize = designer.nodeDisplaySize * 1.2f;
            float time = (float)EditorApplication.timeSinceStartup;
            float pulse = 1.0f + Mathf.Sin(time * 3f) * 0.1f;

            Handles.SphereHandleCap(0, localPos, Quaternion.identity, pulseSize * pulse, EventType.Repaint);

            Handles.color = new Color(ghostColor.r, ghostColor.g, ghostColor.b, 0.8f);
            float crossSize = designer.nodeDisplaySize * 0.3f;
            Handles.DrawLine(localPos + Vector3.right * crossSize, localPos - Vector3.right * crossSize);
            Handles.DrawLine(localPos + Vector3.up * crossSize, localPos - Vector3.up * crossSize);
            Handles.DrawLine(localPos + Vector3.forward * crossSize, localPos - Vector3.forward * crossSize);

            GUIStyle labelStyle = new GUIStyle(EditorStyles.miniLabel) { normal = { textColor = ghostColor } };
            Handles.Label(localPos + Vector3.up * designer.nodeDisplaySize * 2f, "New Node (Click to place)", labelStyle);
        }

        private void DrawWithDepthTest(UnityEngine.Rendering.CompareFunction test, System.Action drawAction)
        {
            Handles.zTest = designer.enableDepthCulling ? test : UnityEngine.Rendering.CompareFunction.Always;
            drawAction();
        }
        #endregion

        #region Scene Interaction
        private void HandleSceneInput(Event e, Vector3 mouseWorldPos)
        {
            // Don't handle input if transform gizmo is being used
            bool isUsingTransformHandle = currentTab == TAB_NODES && designer.selectedNodes.Count > 0 &&
                                           !isCreatingNode && !isCreatingBeam && !isCreatingFace &&
                                           (e.type == EventType.MouseDown || e.type == EventType.MouseDrag) &&
                                           GUIUtility.hotControl != 0;

            if (isUsingTransformHandle)
                return;

            switch (e.type)
            {
                case EventType.MouseMove:
                    if (isCreatingNode) e.Use();
                    break;

                case EventType.MouseDown:
                    if (e.button == 0 && !e.alt)
                    {
                        HandleMouseDown(e, mouseWorldPos);
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
                    else if (e.keyCode == KeyCode.Delete || e.keyCode == KeyCode.Backspace)
                    {
                        ExecuteDeleteShortcut();
                        e.Use();
                    }
                    break;
            }
        }
        private void ExecuteDeleteShortcut()
        {
            switch (designer.currentMode)
            {
                case Designer.DesignerMode.Node:
                    if (designer.selectedNodes.Count > 0) DeleteSelectedNodes();
                    break;
                case Designer.DesignerMode.Beam:
                    if (designer.selectedBeams.Count > 0) DeleteSelectedBeams();
                    break;
                case Designer.DesignerMode.Face:
                    if (designer.selectedFaces.Count > 0) DeleteSelectedFaces();
                    break;
            }
        }

        private void HandleMouseDown(Event e, Vector3 mouseWorldPos)
        {
            bool multi = e.shift || e.control;

            if (isCreatingNode)
            {
                CreateNodeAtPosition(mouseWorldPos);
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
                HandleSelection(mouseWorldPos, multi);
            }

            e.Use();
        }

        private void HandleSelection(Vector3 mouseWorldPos, bool multi)
        {
            switch (designer.currentMode)
            {
                case Designer.DesignerMode.Node:
                    SelectNodeAtPosition(mouseWorldPos, multi);
                    break;
                case Designer.DesignerMode.Beam:
                    SelectBeamAtPosition(mouseWorldPos, multi);
                    break;
                case Designer.DesignerMode.Face:
                    SelectFaceAtPosition(mouseWorldPos, multi);
                    break;
            }
        }

        private void HandleTransformGizmo()
        {
            Handles.matrix = Matrix4x4.identity;

            Vector3 center = targetSoftBody.transform.TransformPoint(selectionCenter);
            EditorGUI.BeginChangeCheck();
            center = Handles.PositionHandle(center, Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                Vector3 localOffset = targetSoftBody.transform.InverseTransformPoint(center) - selectionCenter;
                MoveSelectedNodes(localOffset);
                UpdateSelectionCenter();
            }
            Handles.matrix = targetSoftBody.transform.localToWorldMatrix;
        }
        #endregion

        #region Core Operations
        private void UpdateDesignerModeFromTab()
        {
            Designer.DesignerMode previousMode = designer.currentMode;

            switch (currentTab)
            {
                case TAB_NODES:
                case TAB_NODESETS:
                    designer.currentMode = Designer.DesignerMode.Node;
                    break;
                case TAB_BEAMS:
                case TAB_LINKSETS:
                    designer.currentMode = Designer.DesignerMode.Beam;
                    break;
                case TAB_FACES:
                    designer.currentMode = Designer.DesignerMode.Face;
                    break;
                default:
                    designer.currentMode = Designer.DesignerMode.Node;
                    break;
            }

            if (previousMode == Designer.DesignerMode.Node && designer.currentMode != Designer.DesignerMode.Node)
            {
                if (isCreatingNode)
                {
                    isCreatingNode = false;
                    designer.SetEditorCollider(false);
                }
            }
        }

        private void ApplyTrussToTarget()
        {
            if (targetSoftBody == null || workingTruss == null) return;

            if (targetSoftBody.truss != workingTruss)
            {
                targetSoftBody.truss = workingTruss;
                EditorUtility.SetDirty(targetSoftBody);
            }
        }

        private void ApplyBeamPreset(Designer.BeamPreset preset)
        {
            var (c, d, thresh, rate, maxDef) = GetBeamPresetValues(preset);

            designer.defaultBeamCompliance = c;
            designer.defaultBeamDamping = d;
            designer.defaultPlasticityThreshold = thresh;
            designer.defaultPlasticityRate = rate;
            designer.defaultMaxDeformation = maxDef;

            if (designer.selectedBeams.Count > 0)
            {
                ApplyBeamPropertiesToSelection();
            }
        }

        private void ApplyBeamPropertiesToSelection()
        {
            var beams = workingTruss.GetTrussBeams();
            foreach (int idx in designer.selectedBeams)
            {
                if (idx < beams.Count)
                {
                    beams[idx].compliance = designer.defaultBeamCompliance;
                    beams[idx].damping = designer.defaultBeamDamping;
                    beams[idx].plasticityThreshold = designer.defaultPlasticityThreshold;
                    beams[idx].plasticityRate = designer.defaultPlasticityRate;
                    beams[idx].maxDeformation = designer.defaultMaxDeformation;
                }
            }
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private (float c, float d, float thresh, float rate, float maxDef) GetBeamPresetValues(Designer.BeamPreset preset)
        {
            switch (preset)
            {
                case Designer.BeamPreset.Metal:
                    return (0.001f, 0.1f, 0.05f, 0.1f, 0.2f);
                case Designer.BeamPreset.Rubber:
                    return (0.1f, 0.5f, 0.01f, 0.8f, 2.0f);
                default:
                    return (0.01f, 0.3f, 0.02f, 0.5f, 0.8f);
            }
        }
        #endregion

        #region Node Operations
        private void DuplicateSelectedNodes()
        {
            if (designer.selectedNodes.Count == 0) return;

            // Store old indices before adding new nodes
            List<int> oldIndices = new List<int>(designer.selectedNodes);
            List<int> newIndices = new List<int>();

            // Duplicate each selected node
            foreach (int oldIndex in oldIndices)
            {
                if (oldIndex >= workingTruss.NodePositions.Length) continue;

                // Get original node data
                Vector3 originalPos = workingTruss.NodePositions[oldIndex];
                float mass = oldIndex < workingTruss.NodeMasses.Count ? workingTruss.NodeMasses[oldIndex] : designer.nodeCreationMass;
                bool isPinned = workingTruss.PinnedNodes.Contains(oldIndex);

                Vector3 newPos = originalPos;

                // Add to truss
                var positions = new List<Vector3>(workingTruss.NodePositions);
                positions.Add(newPos);
                workingTruss.SetNodePositions(positions.ToArray());

                var masses = new List<float>(workingTruss.NodeMasses);
                masses.Add(mass);
                workingTruss.SetNodeMasses(masses);

                // Pin the new node if original was pinned
                if (isPinned)
                {
                    var pinnedNodes = new List<int>(workingTruss.PinnedNodes);
                    pinnedNodes.Add(positions.Count - 1);
                    workingTruss.SetPinnedNodes(pinnedNodes);
                }

                newIndices.Add(positions.Count - 1);
            }

            // Select the newly created nodes
            designer.selectedNodes.Clear();
            designer.selectedNodes.AddRange(newIndices);
            UpdateSelectionCenter();

            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();

            SceneView.RepaintAll();
            Repaint();
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

            if (designer.selectedNodes.Contains(idx))
                designer.selectedNodes.Remove(idx);
            else
                designer.selectedNodes.Add(idx);

            UpdateSelectionCenter();
        }

        private void SetNodePosition(int idx, Vector3 pos)
        {
            workingTruss.NodePositions[idx] = pos;
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
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
                if (pin && !list.Contains(i))
                    list.Add(i);
                else if (!pin && list.Contains(i))
                    list.Remove(i);
            }
            workingTruss.SetPinnedNodes(list);
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private void SetSelectedNodesMass(float mass)
        {
            foreach (int i in designer.selectedNodes)
            {
                if (i < workingTruss.NodeMasses.Count)
                    workingTruss.NodeMasses[i] = mass;
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void DeleteSelectedNodes()
        {
            if (designer.selectedNodes.Count == 0) return;

            if (!EditorUtility.DisplayDialog("Delete Nodes",
                $"Delete {designer.selectedNodes.Count} node(s)?\n\n" +
                "This will also remove:\n" +
                "• All beams connected to these nodes\n" +
                "• All faces using these nodes\n" +
                "• References in node/link sets\n\n" +
                "This operation cannot be undone.",
                "Delete", "Cancel"))
            {
                return;
            }

            var nodesToDelete = new HashSet<int>(designer.selectedNodes);
            Dictionary<int, int> oldToNewIndex = BuildIndexMapping(nodesToDelete);

            RemoveNodesFromTruss(nodesToDelete);
            UpdateNodeMasses(nodesToDelete);
            UpdatePinnedNodes(nodesToDelete, oldToNewIndex);
            var removedBeamIndices = UpdateBeams(nodesToDelete, oldToNewIndex);
            UpdateFaces(nodesToDelete, oldToNewIndex);
            UpdateNodeSetsAfterDeletion(nodesToDelete, oldToNewIndex);
            UpdateLinkSetsAfterDeletion(removedBeamIndices);

            designer.selectedNodes.Clear();
            designer.selectedBeams.Clear();
            designer.selectedFaces.Clear();

            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();

            SceneView.RepaintAll();
            Repaint();
        }

        private Dictionary<int, int> BuildIndexMapping(HashSet<int> nodesToDelete)
        {
            Dictionary<int, int> oldToNewIndex = new Dictionary<int, int>();
            int offset = 0;
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                if (nodesToDelete.Contains(i))
                {
                    offset++;
                }
                else
                {
                    oldToNewIndex[i] = i - offset;
                }
            }
            return oldToNewIndex;
        }

        private void RemoveNodesFromTruss(HashSet<int> nodesToDelete)
        {
            var newPositions = new List<Vector3>();
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                if (!nodesToDelete.Contains(i))
                {
                    newPositions.Add(workingTruss.NodePositions[i]);
                }
            }
            workingTruss.SetNodePositions(newPositions.ToArray());
        }

        private void UpdateNodeMasses(HashSet<int> nodesToDelete)
        {
            var newMasses = new List<float>();
            for (int i = 0; i < workingTruss.NodeMasses.Count; i++)
            {
                if (!nodesToDelete.Contains(i))
                {
                    newMasses.Add(workingTruss.NodeMasses[i]);
                }
            }
            workingTruss.SetNodeMasses(newMasses);
        }

        private void UpdatePinnedNodes(HashSet<int> nodesToDelete, Dictionary<int, int> oldToNewIndex)
        {
            var newPinnedNodes = new List<int>();
            foreach (int pinnedNode in workingTruss.PinnedNodes)
            {
                if (!nodesToDelete.Contains(pinnedNode))
                {
                    newPinnedNodes.Add(oldToNewIndex[pinnedNode]);
                }
            }
            workingTruss.SetPinnedNodes(newPinnedNodes);
        }

        private HashSet<int> UpdateBeams(HashSet<int> nodesToDelete, Dictionary<int, int> oldToNewIndex)
        {
            var beams = workingTruss.GetTrussBeams();
            var validBeams = new List<Beam>();
            var removedBeamIndices = new HashSet<int>();

            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                bool nodeADeleted = nodesToDelete.Contains(beam.nodeA);
                bool nodeBDeleted = nodesToDelete.Contains(beam.nodeB);

                if (!nodeADeleted && !nodeBDeleted)
                {
                    beam.nodeA = oldToNewIndex[beam.nodeA];
                    beam.nodeB = oldToNewIndex[beam.nodeB];
                    validBeams.Add(beam);
                }
                else
                {
                    removedBeamIndices.Add(i);
                }
            }
            workingTruss.SetBeams(validBeams);
            return removedBeamIndices;
        }

        private void UpdateFaces(HashSet<int> nodesToDelete, Dictionary<int, int> oldToNewIndex)
        {
            var faces = workingTruss.GetTrussFaces();
            for (int i = faces.Count - 1; i >= 0; i--)
            {
                var face = faces[i];
                bool anyNodeDeleted = nodesToDelete.Contains(face.nodeA) ||
                                     nodesToDelete.Contains(face.nodeB) ||
                                     nodesToDelete.Contains(face.nodeC);

                if (anyNodeDeleted)
                {
                    faces.RemoveAt(i);
                }
                else
                {
                    face.nodeA = oldToNewIndex[face.nodeA];
                    face.nodeB = oldToNewIndex[face.nodeB];
                    face.nodeC = oldToNewIndex[face.nodeC];
                    faces[i] = face;
                }
            }
        }

        private void UpdateNodeSetsAfterDeletion(HashSet<int> nodesToDelete, Dictionary<int, int> oldToNewIndex)
        {
            var nodeSets = workingTruss.GetNodeSets();
            foreach (var nodeSet in nodeSets)
            {
                var updatedIndices = new List<int>();
                foreach (int nodeIndex in nodeSet.nodeIndices)
                {
                    if (!nodesToDelete.Contains(nodeIndex))
                    {
                        updatedIndices.Add(oldToNewIndex[nodeIndex]);
                    }
                }
                nodeSet.nodeIndices = updatedIndices;
            }
            workingTruss.ValidateNodeSets();
        }

        private void UpdateLinkSetsAfterDeletion(HashSet<int> removedBeamIndices)
        {
            var linkSets = workingTruss.GetLinkSets();
            foreach (var linkSet in linkSets)
            {
                var updatedIndices = new List<int>();
                foreach (int linkIndex in linkSet.linkIndices)
                {
                    if (!removedBeamIndices.Contains(linkIndex))
                    {
                        int newIndex = linkIndex - removedBeamIndices.Count(removed => removed < linkIndex);
                        updatedIndices.Add(newIndex);
                    }
                }
                linkSet.linkIndices = updatedIndices;
            }
            workingTruss.ValidateLinkSets();
        }

        private void SelectAllNodes()
        {
            designer.selectedNodes.Clear();
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
                designer.selectedNodes.Add(i);
        }

        private void UpdateSelectionCenter()
        {
            if (designer.selectedNodes.Count == 0) return;
            Vector3 avg = Vector3.zero;
            foreach (int i in designer.selectedNodes)
                avg += workingTruss.NodePositions[i];
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
        #endregion

        #region Beam Operations
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
                maxDeformation: designer.defaultMaxDeformation,
                plasticityThreshold: designer.defaultPlasticityThreshold,
                plasticityRate: designer.defaultPlasticityRate
            ));
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();
        }

        private void SelectBeamAtPosition(Vector3 worldPos, bool multi)
        {
            var beams = workingTruss.GetTrussBeams();

            Ray ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
            float minDistance = 0.5f;
            int closestBeam = -1;

            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                if (beam.nodeA >= workingTruss.NodePositions.Length || beam.nodeB >= workingTruss.NodePositions.Length)
                    continue;

                Vector3 localA = workingTruss.NodePositions[beam.nodeA];
                Vector3 localB = workingTruss.NodePositions[beam.nodeB];

                Vector3 worldA = targetSoftBody.transform.TransformPoint(localA);
                Vector3 worldB = targetSoftBody.transform.TransformPoint(localB);

                float distance = DistanceFromRayToLineSegment(ray, worldA, worldB);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestBeam = i;
                }
            }

            if (closestBeam != -1)
            {
                if (!multi) designer.selectedBeams.Clear();

                if (designer.selectedBeams.Contains(closestBeam))
                    designer.selectedBeams.Remove(closestBeam);
                else
                    designer.selectedBeams.Add(closestBeam);
            }
            else if (!multi)
            {
                designer.selectedBeams.Clear();
            }
        }

        private float DistanceFromRayToLineSegment(Ray ray, Vector3 lineStart, Vector3 lineEnd)
        {
            Vector3 lineDir = lineEnd - lineStart;
            float lineLength = lineDir.magnitude;

            if (lineLength < 0.0001f)
            {
                return Vector3.Cross(ray.direction, lineStart - ray.origin).magnitude;
            }

            lineDir /= lineLength;

            Vector3 rayToLineStart = lineStart - ray.origin;
            Vector3 rayToLineEnd = lineEnd - ray.origin;

            float rayDotLine = Vector3.Dot(ray.direction, lineDir);
            float t1 = Vector3.Dot(rayToLineStart, ray.direction);
            float t2 = Vector3.Dot(rayToLineStart, lineDir);

            float denom = 1.0f - rayDotLine * rayDotLine;

            float s, t;
            if (Mathf.Abs(denom) < 0.0001f)
            {
                s = 0;
                t = t2;
            }
            else
            {
                s = (t1 - t2 * rayDotLine) / denom;
                t = (t1 * rayDotLine - t2) / denom;
            }

            t = Mathf.Clamp(t, 0, lineLength);

            Vector3 closestPointOnRay = ray.origin + ray.direction * s;
            Vector3 closestPointOnLine = lineStart + lineDir * t;

            return Vector3.Distance(closestPointOnRay, closestPointOnLine);
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
                    bool exists = beams.Any(b => (b.nodeA == nodeA && b.nodeB == nodeB) ||
                                                 (b.nodeA == nodeB && b.nodeB == nodeA));
                    if (!exists)
                    {
                        Vector3 pA = workingTruss.NodePositions[nodeA];
                        Vector3 pB = workingTruss.NodePositions[nodeB];
                        beams.Add(new Beam(nodeA, nodeB, designer.defaultBeamCompliance,
                                         designer.defaultBeamDamping, Vector3.Distance(pA, pB)));
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

        private void SelectAllBeams()
        {
            designer.selectedBeams.Clear();
            for (int i = 0; i < workingTruss.GetTrussBeams().Count; i++)
                designer.selectedBeams.Add(i);
        }
        #endregion

        #region Face Operations
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
                if (designer.selectedFaces.Contains(best))
                    designer.selectedFaces.Remove(best);
                else
                    designer.selectedFaces.Add(best);
            }
            else if (!multi)
            {
                designer.selectedFaces.Clear();
            }
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

        private void SelectAllFaces()
        {
            designer.selectedFaces.Clear();
            for (int i = 0; i < workingTruss.GetTrussFaces().Count; i++)
                designer.selectedFaces.Add(i);
        }
        #endregion

        #region Node Set Operations
        private void CreateNodeSetFromSelection()
        {
            if (designer.selectedNodes.Count == 0 || string.IsNullOrEmpty(designer.nodeSetCreationName))
                return;

            var nodeSet = new NodeSet(designer.nodeSetCreationName, designer.selectedNodes)
            {
                color = designer.nodeSetCreationColor,
                isVisible = true
            };

            workingTruss.AddNodeSet(nodeSet);
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();

            designer.nodeSetCreationName = "New Node Set";
            designer.selectedNodeSetIndex = workingTruss.GetNodeSets().Count - 1;
        }

        private void SelectNodeSet(int index, NodeSet nodeSet)
        {
            designer.selectedNodeSetIndex = index;
            designer.selectedNodes.Clear();
            designer.selectedNodes.AddRange(nodeSet.nodeIndices);
            SceneView.RepaintAll();
        }

        private void DeleteNodeSet(NodeSet nodeSet)
        {
            if (EditorUtility.DisplayDialog("Delete Node Set",
                $"Delete node set '{nodeSet.name}'?", "Delete", "Cancel"))
            {
                workingTruss.RemoveNodeSet(nodeSet.name);
                designer.selectedNodeSetIndex = -1;
                EditorUtility.SetDirty(workingTruss);
                if (designer.autoApplyChanges) ApplyTrussToTarget();
            }
        }

        private void AddNodesToSet(NodeSet nodeSet)
        {
            foreach (int nodeIdx in designer.selectedNodes)
            {
                if (!nodeSet.nodeIndices.Contains(nodeIdx))
                    nodeSet.nodeIndices.Add(nodeIdx);
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void RemoveNodesFromSet(NodeSet nodeSet)
        {
            foreach (int nodeIdx in designer.selectedNodes)
            {
                nodeSet.nodeIndices.Remove(nodeIdx);
            }
            EditorUtility.SetDirty(workingTruss);
        }
        #endregion

        #region Link Set Operations
        private void CreateLinkSetFromSelection()
        {
            if (designer.selectedBeams.Count == 0 || string.IsNullOrEmpty(designer.linkSetCreationName))
                return;

            var linkSet = new LinkSet(designer.linkSetCreationName, designer.selectedBeams)
            {
                color = designer.linkSetCreationColor,
                isVisible = true
            };

            workingTruss.AddLinkSet(linkSet);
            EditorUtility.SetDirty(workingTruss);
            if (designer.autoApplyChanges) ApplyTrussToTarget();

            designer.linkSetCreationName = "New Link Set";
            designer.selectedLinkSetIndex = workingTruss.GetLinkSets().Count - 1;
        }

        private void SelectLinkSet(int index, LinkSet linkSet)
        {
            designer.selectedLinkSetIndex = index;
            designer.selectedBeams.Clear();
            designer.selectedBeams.AddRange(linkSet.linkIndices);
            SceneView.RepaintAll();
        }

        private void DeleteLinkSet(LinkSet linkSet)
        {
            if (EditorUtility.DisplayDialog("Delete Link Set",
                $"Delete link set '{linkSet.name}'?", "Delete", "Cancel"))
            {
                workingTruss.RemoveLinkSet(linkSet.name);
                designer.selectedLinkSetIndex = -1;
                EditorUtility.SetDirty(workingTruss);
                if (designer.autoApplyChanges) ApplyTrussToTarget();
            }
        }

        private void AddLinksToSet(LinkSet linkSet)
        {
            foreach (int linkIdx in designer.selectedBeams)
            {
                if (!linkSet.linkIndices.Contains(linkIdx))
                    linkSet.linkIndices.Add(linkIdx);
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void RemoveLinksFromSet(LinkSet linkSet)
        {
            foreach (int linkIdx in designer.selectedBeams)
            {
                linkSet.linkIndices.Remove(linkIdx);
            }
            EditorUtility.SetDirty(workingTruss);
        }
        #endregion

        #region Utility Methods
        private Vector3 GetWorldPositionFromMouse(Vector2 mousePosition)
        {
            Ray ray = HandleUtility.GUIPointToWorldRay(mousePosition);

            if (isCreatingNode)
            {
                Collider softBodyCollider = targetSoftBody.GetComponent<Collider>();
                if (softBodyCollider != null && softBodyCollider.Raycast(ray, out RaycastHit hit, Mathf.Infinity))
                {
                    return hit.point;
                }
            }

            if (Physics.Raycast(ray, out RaycastHit sceneHit, Mathf.Infinity))
            {
                return sceneHit.point;
            }

            Plane plane = new Plane(targetSoftBody.transform.up, targetSoftBody.transform.position);
            if (plane.Raycast(ray, out float dist))
            {
                return ray.GetPoint(dist);
            }

            return ray.GetPoint(10f);
        }

        private int FindClosestNode(Vector3 worldPos)
        {
            if (workingTruss.NodePositions == null) return -1;

            Ray ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
            int closestNode = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 nodeWorldPos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                float distance = Vector3.Cross(ray.direction, nodeWorldPos - ray.origin).magnitude;

                if (distance < designer.nodeDisplaySize && distance < minDistance)
                {
                    float t = Vector3.Dot(nodeWorldPos - ray.origin, ray.direction);
                    if (t > 0 && t < 1000f)
                    {
                        minDistance = distance;
                        closestNode = i;
                    }
                }
            }

            return closestNode;
        }

        private void ToggleNodeCreation()
        {
            isCreatingNode = !isCreatingNode;
            designer.SetEditorCollider(isCreatingNode);
        }

        private void ToggleBeamCreation()
        {
            isCreatingBeam = !isCreatingBeam;
        }

        private void ToggleFaceCreation()
        {
            isCreatingFace = !isCreatingFace;
        }

        private void CancelCurrentOperation()
        {
            isCreatingNode = false;
            isCreatingBeam = false;
            isCreatingFace = false;
            beamStartNode = -1;
            faceCreationNodes.Clear();
        }
        #endregion
    }
}