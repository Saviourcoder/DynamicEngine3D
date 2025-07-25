#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

namespace DynamicEngine
{
    [CustomEditor(typeof(NodeLinkEditor))]
    public class NodeLinkEditorCustom : Editor
    {
        private NodeLinkEditor editor;
        private Vector3 handlePosition;
        private bool isDragging;
        private bool stretchLimitsFoldout = true;
        private bool visualizationFoldout = true;
        private bool materialPresetsFoldout = true;
        private Dictionary<int, int> nodeConnections = new Dictionary<int, int>();

        private void OnEnable()
        {
            editor = (NodeLinkEditor)target;
            if (editor.softBody == null)
            {
                editor.softBody = editor.GetComponent<SoftBody>();
            }

            if (EditorApplication.isPlayingOrWillChangePlaymode)
                return;

            EditorApplication.delayCall += DelayedInitializeSoftBody;
        }

        public override void OnInspectorGUI()
        {
            // Safety check to prevent GUI errors
            if (editor == null || serializedObject == null)
            {
                EditorGUILayout.HelpBox("NodeLinkEditor not properly initialized. Please select a valid GameObject with the NodeLinkEditor component.", MessageType.Error);
                return;
            }
            
            try
            {
                serializedObject.Update();
                DrawInspectorGUI();
                serializedObject.ApplyModifiedProperties();
                SceneView.RepaintAll();
            }
            catch (System.Exception e)
            {
                EditorGUILayout.HelpBox($"GUI Error: {e.Message}", MessageType.Error);
                // Reset editor state if needed
                if (GUILayout.Button("Reset Editor State"))
                {
                    editor = (NodeLinkEditor)target;
                    EditorUtility.SetDirty(editor);
                }
            }
        }

        private void DrawInspectorGUI()
        {
            if (editor == null) return;
            
            DrawSoftBodyReference();
            EditorGUILayout.Space();
            DrawTabs();
        }


        private void DelayedInitializeSoftBody()
        {
            // if the editor window might have been closed already
            if (editor == null)
                return;

            // get the softBody component if we dont have it yet
            var softBody = editor.GetComponent<SoftBody>();
            if (softBody == null)
                return;

            // make sure we have a trussasset
            if (softBody.truss == null)
            {
                Debug.LogWarning("SoftBody has no TrussAsset assigned.", softBody);
                return;
            }

            softBody.InitializeInEditMode();
            EditorUtility.SetDirty(softBody);
        }


        private void DrawSoftBodyReference()
        {
            EditorGUILayout.BeginHorizontal();
            GUI.enabled = false;
            EditorGUILayout.ObjectField("SoftBody", editor.SoftBody, typeof(SoftBody), true);
            GUI.enabled = true;
            EditorGUILayout.EndHorizontal();

            if (EditorApplication.isPlayingOrWillChangePlaymode) return;

            editor.softBody = editor.GetComponent<SoftBody>();
            if (editor.softBody == null)
            {
                Debug.LogWarning("No SoftBody component found on this GameObject.", editor);
                return;
            }


        }
        private void DrawTabs()
        {
            string[] tabs = { "Info", "Nodes", "Links", "Debug", "Visual", "Stretch", "Node Sets" };
            editor.currentTab = GUILayout.Toolbar(editor.currentTab, tabs);
            EditorGUILayout.Space();

            switch (editor.currentTab)
            {
                case 0: DrawInfoTab(); break;
                case 1: DrawNodesTab(); break;
                case 2: DrawLinksTab(); break;
                case 3: DrawDebuggingTab(); break;
                case 4: DrawVisualizationTab(); break;
                case 5: DrawStretchLimitsTab(); break;
                case 6: DrawNodeSetsTab(); break;
            }
        }

        private void DrawInfoTab()
        {
            EditorGUILayout.LabelField("Asset Information", EditorStyles.boldLabel);
            GUI.enabled = false;
            EditorGUILayout.ObjectField("NodeLinkEditor", editor, typeof(NodeLinkEditor), true);
            EditorGUILayout.IntField("Nodes", editor.nodes.Count);
            EditorGUILayout.IntField("Links", editor.links.Count);
            EditorGUILayout.IntField("Pinned Nodes", editor.pinnedNodes.Count);
            GUI.enabled = true;
            if (GUILayout.Button("Reload from TrussAsset"))
            {
                editor.LoadFromTrussAsset();
                editor.ApplyStretchLimits();
                editor.ApplyPinnedNodes();
                EditorUtility.SetDirty(editor);
            }
            editor.isCreatingNode = false;
            editor.creatingLinkNodeIndex = -1;
        }

        private void DrawNodesTab()
        {
            if (GUILayout.Button(editor.isCreatingNode ? "Cancel Create Node" : "Create Node"))
            {
                editor.isCreatingNode = !editor.isCreatingNode;
                editor.creatingLinkNodeIndex = -1;   // cancel link creation if any
            }
            
            // Camera Occlusion
            EditorGUI.BeginChangeCheck();
            alwaysSeeNodes = EditorGUILayout.Toggle(new GUIContent("Camera Occlusion", "When enabled, nodes show through mesh geometry"), alwaysSeeNodes);
            if (EditorGUI.EndChangeCheck())
            {
                SceneView.RepaintAll();
            }
            
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.LabelField("Node Mass", EditorStyles.boldLabel);
            editor.nodeMass = EditorGUILayout.FloatField(new GUIContent("Mass", "Mass of each node for physics simulation"), Mathf.Max(0.1f, editor.nodeMass));

            if (EditorGUI.EndChangeCheck())
            {
                var core = editor.GetSolver();
                if (core != null) core.materialProps.nodeMass = editor.nodeMass;
            }
            string nodeInfo = editor.selectedNodeIndices.Count switch
            {
                0 => "No Nodes Selected",
                1 => $"Node {editor.selectedNodeIndices[0]} Selected",
                _ => $"{editor.selectedNodeIndices.Count} Nodes Selected"
            };
            EditorGUILayout.LabelField(nodeInfo, new GUIStyle(EditorStyles.label) { alignment = TextAnchor.MiddleRight });

            if (editor.selectedNodeIndices.Count > 0)
            {
                EditorGUILayout.LabelField("Selected Node Details", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                
                // Show summary information
                DrawNodeSelectionSummary();
                
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Delete Selected")) editor.DeleteSelected();
            if (GUILayout.Button("Select All"))
            {
                editor.selectedNodeIndices.Clear();
                for (int i = 0; i < editor.nodes.Count; i++) editor.selectedNodeIndices.Add(i);
            }
            if (GUILayout.Button("Clear Selection")) editor.selectedNodeIndices.Clear();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Pin Selected")) editor.PinSelectedNodes();
            if (GUILayout.Button("Unpin Selected")) editor.UnpinSelectedNodes();
            EditorGUILayout.EndHorizontal();

            editor.creatingLinkNodeIndex = -1;
        }

        private void DrawLinksTab()
        {
            bool isCreatingLink = editor.creatingLinkNodeIndex >= 0;
            editor.isCreatingNode = false;
            
            // Camera Occlusion
            EditorGUI.BeginChangeCheck();
            alwaysSeeLinks = EditorGUILayout.Toggle(new GUIContent("Camera Occlusion", "When enabled, links show through mesh geometry"), alwaysSeeLinks);
            if (EditorGUI.EndChangeCheck())
            {
                SceneView.RepaintAll();
            }
            
            string linkInfo = editor.selectedLinkIndices.Count switch
            {
                0 => "No Links Selected",
                1 => $"Link {editor.selectedLinkIndices[0]} Selected",
                _ => $"{editor.selectedLinkIndices.Count} Links Selected"
            };
            EditorGUILayout.LabelField(linkInfo, new GUIStyle(EditorStyles.label) { alignment = TextAnchor.MiddleRight });

            if (editor.selectedLinkIndices.Count > 0)
            {
                EditorGUILayout.LabelField("Selected Link Details", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                
                // Show summary information
                DrawLinkSelectionSummary();
                
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.BeginHorizontal();
            bool createLinkPressed = GUILayout.Button("Create Link", isCreatingLink ? new GUIStyle(GUI.skin.button) { normal = { background = Texture2D.grayTexture } } : GUI.skin.button);
            if (createLinkPressed)
            {
                editor.creatingLinkNodeIndex = isCreatingLink ? -1 : 0;
            }
            if (GUILayout.Button("Delete Selected")) editor.DeleteSelected();
            if (GUILayout.Button("Select All"))
            {
                editor.selectedLinkIndices.Clear();
                for (int i = 0; i < editor.links.Count; i++) editor.selectedLinkIndices.Add(i);
            }
            if (GUILayout.Button("Clear Selection")) editor.selectedLinkIndices.Clear();
            EditorGUILayout.EndHorizontal();

            materialPresetsFoldout = EditorGUILayout.Foldout(materialPresetsFoldout, "Material Presets", true, EditorStyles.boldLabel);
            if (materialPresetsFoldout)
            {
                EditorGUI.indentLevel++;
                
                // Show information about preset application
                if (editor.selectedLinkIndices.Count > 0)
                {
                    EditorGUILayout.HelpBox($"Presets will be applied to {editor.selectedLinkIndices.Count} selected link{(editor.selectedLinkIndices.Count > 1 ? "s" : "")}.", MessageType.Info);
                }
                else
                {
                    EditorGUILayout.HelpBox("No links selected. Presets will be applied to ALL links.", MessageType.Warning);
                }
                
                EditorGUILayout.Space(5);
                
                // Display presets in a grid layout - safer implementation
                if (editor.materialPresets != null && editor.materialPresets.Count > 0)
                {
                    int presetsPerRow = 3;
                    bool isInHorizontalGroup = false;
                    
                    for (int i = 0; i < editor.materialPresets.Count; i++)
                    {
                        // Start a new row
                        if (i % presetsPerRow == 0)
                        {
                            // End previous row if we were in one
                            if (isInHorizontalGroup)
                            {
                                EditorGUILayout.EndHorizontal();
                                isInHorizontalGroup = false;
                            }
                            
                            // Start new row
                            EditorGUILayout.BeginHorizontal();
                            isInHorizontalGroup = true;
                        }
                        
                        var preset = editor.materialPresets[i];
                        if (preset != null && !string.IsNullOrEmpty(preset.name))
                        {
                            if (GUILayout.Button(preset.name, GUILayout.Height(30)))
                            {
                                editor.ApplyMaterialPreset(preset);
                                EditorUtility.SetDirty(editor);
                            }
                        }
                        else
                        {
                            // Handle null preset gracefully
                            if (GUILayout.Button("Invalid Preset", GUILayout.Height(30)))
                            {
                                Debug.LogWarning($"Invalid material preset at index {i}");
                            }
                        }
                    }
                    
                    // Always close the horizontal group if we're in one
                    if (isInHorizontalGroup)
                    {
                        EditorGUILayout.EndHorizontal();
                    }
                }
                else
                {
                    EditorGUILayout.HelpBox("No material presets available. Click 'Refresh Presets' to initialize them.", MessageType.Warning);
                }
                
                EditorGUILayout.Space(5);
                
                // Add refresh button for presets
                EditorGUILayout.BeginHorizontal();
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("Refresh Presets", GUILayout.Width(120)))
                {
                    editor.RefreshMaterialPresets();
                    EditorUtility.SetDirty(editor);
                }
                EditorGUILayout.EndHorizontal();
                
                // Show current material properties for comparison
                if (editor.selectedLinkIndices.Count > 0)
                {
                    EditorGUILayout.LabelField("Current Selection Properties", EditorStyles.miniBoldLabel);
                    var firstLink = editor.links[editor.selectedLinkIndices[0]];
                    EditorGUILayout.LabelField($"Spring Force: {firstLink.springForce:F0}");
                    EditorGUILayout.LabelField($"Damping: {firstLink.damping:F3}");
                    EditorGUILayout.LabelField($"Node Mass: {editor.nodeMass:F2}");
                }
                
                EditorGUI.indentLevel--;
            }

            if (editor.selectedLinkIndices.Count > 0)
            {
                EditorGUILayout.LabelField("Selected Link Properties", EditorStyles.boldLabel);
                var firstLink = editor.links[editor.selectedLinkIndices[0]];
                float springForce = firstLink.springForce;
                float damping = firstLink.damping;
                bool uniform = true;

                foreach (int idx in editor.selectedLinkIndices)
                {
                    var link = editor.links[idx];
                    if (link.springForce != springForce || link.damping != damping)
                    {
                        uniform = false;
                        break;
                    }
                }

                EditorGUI.BeginChangeCheck();
                springForce = EditorGUILayout.FloatField(new GUIContent("Spring Force", "Force applied to maintain rest length"), Mathf.Clamp(springForce, 100f, 20000f));
                damping = EditorGUILayout.FloatField(new GUIContent("Damping", "Damping factor to reduce oscillations"), Mathf.Clamp(damping, 0.001f, 50f));
                if (EditorGUI.EndChangeCheck() && uniform)
                {
                    Undo.RecordObject(editor, "Modify Link Properties");
                    for (int i = 0; i < editor.selectedLinkIndices.Count; i++)
                    {
                        int idx = editor.selectedLinkIndices[i];
                        var link = editor.links[idx];
                        link.springForce = Mathf.Max(100f, springForce);
                        link.damping = Mathf.Max(0.001f, damping);
                        editor.links[idx] = link;
                    }
                    editor.SaveToTrussAsset();
                }
                else if (!uniform)
                {
                    EditorGUILayout.HelpBox("Multiple links with different properties selected. Values shown are for the first selected link.", MessageType.Info);
                }

                var core = editor.GetSolver();
                if (core != null)
                {
                    foreach (int idx in editor.selectedLinkIndices)
                    {
                        var beam = core.beams[idx];
                        beam.compliance = 1f / editor.links[idx].springForce; // invert because Solver expects “compliance”
                        beam.damping = editor.links[idx].damping;
                        core.beams[idx] = beam;
                    }
                }
            }
        }

        private void DrawDebuggingTab()
        {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("enableDebugLogs"), new GUIContent("Enable Debug Logs", "Log detailed operations like pinning and stretch limit application"));

            if (GUILayout.Button("Validate Node Connectivity"))
            {
                nodeConnections.Clear();
                if (editor.nodes.Count == 0 || editor.links.Count == 0)
                {
                    EditorGUILayout.LabelField("No nodes or links to validate.");
                }
                else
                {
                    for (int i = 0; i < editor.nodes.Count; i++) nodeConnections[i] = 0;
                    foreach (var link in editor.links)
                    {
                        if (link.nodeA >= 0 && link.nodeA < editor.nodes.Count) nodeConnections[link.nodeA]++;
                        if (link.nodeB >= 0 && link.nodeB < editor.nodes.Count) nodeConnections[link.nodeB]++;
                    }
                }
            }

            if (nodeConnections.Count > 0)
            {
                EditorGUILayout.LabelField("Node Connectivity Details", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                foreach (var pair in nodeConnections.OrderBy(kvp => kvp.Key))
                {
                    EditorGUILayout.LabelField($"Node {pair.Key}: {pair.Value} connections");
                }
                EditorGUI.indentLevel--;
            }

            if (GUILayout.Button("Reapply Truss Limits and Pinned Nodes"))
            {
                editor.ApplyStretchLimits();
                editor.ApplyPinnedNodes();
            }
        }

        private void DrawVisualizationTab()
        {
            visualizationFoldout = EditorGUILayout.Foldout(visualizationFoldout, "Visualization Settings", true, EditorStyles.boldLabel);
            if (visualizationFoldout)
            {
                EditorGUI.indentLevel++;
                
                // Display Mode Information
                EditorGUILayout.Space(5);
                EditorGUILayout.LabelField("Automatic Display Mode", EditorStyles.boldLabel);
                
                string currentModeDescription = GetCurrentDisplayMode() switch
                {
                    DisplayMode.NodesOnly => "🔴 Nodes Only - Current tab shows only nodes",
                    DisplayMode.LinksOnly => "🔗 Links Only - Current tab shows only links",
                    DisplayMode.Both => "👁️ Both - Current tab shows both nodes and links",
                    _ => ""
                };
                
                EditorGUILayout.HelpBox($"{currentModeDescription}\n\n• Nodes Tab: Shows only nodes\n• Links Tab: Shows only links (both when creating)\n• Node Sets Tab: Shows only nodes", MessageType.Info);
                
                EditorGUILayout.Space(5);
                
                EditorGUILayout.PropertyField(serializedObject.FindProperty("visualizeForces"), new GUIContent("Visualize Forces", "Show force vectors in simulation (red: tension, blue: compression)"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("nodeSize"), new GUIContent("Node Size", "Size of node spheres in Scene view"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("linkThickness"), new GUIContent("Link Thickness", "Thickness of link lines in Scene view"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("nodeColor"), new GUIContent("Node Color", "Color for unselected nodes"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("selectedNodeColor"), new GUIContent("Selected Node Color"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("pinnedNodeColor"), new GUIContent("Pinned Node Color"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("linkColor"), new GUIContent("Link Color", "Color for unselected links"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("selectedLinkColor"), new GUIContent("Selected Link Color"));
                EditorGUI.indentLevel--;
            }
        }

        private void DrawLinkSelectionSummary()
        {
            // Safety checks to prevent GUI errors
            if (editor == null || editor.links == null || editor.selectedLinkIndices == null)
            {
                EditorGUILayout.HelpBox("Editor data not properly initialized.", MessageType.Warning);
                return;
            }

            var selectedLinks = editor.selectedLinkIndices
                .Where(idx => idx >= 0 && idx < editor.links.Count)
                .Select(idx => editor.links[idx])
                .ToList();

            if (!selectedLinks.Any()) return;

            // Count and overview
            EditorGUILayout.LabelField($"Selection: {selectedLinks.Count} link{(selectedLinks.Count > 1 ? "s" : "")}");

            // Calculate statistics with safety checks
            var springForces = selectedLinks.Select(l => l.springForce).Where(f => !float.IsNaN(f) && !float.IsInfinity(f)).ToList();
            var dampings = selectedLinks.Select(l => l.damping).Where(d => !float.IsNaN(d) && !float.IsInfinity(d)).ToList();
            var restLengths = selectedLinks.Select(l => l.restLength).Where(r => !float.IsNaN(r) && !float.IsInfinity(r)).ToList();

            if (!springForces.Any() || !dampings.Any() || !restLengths.Any())
            {
                EditorGUILayout.HelpBox("Selected links contain invalid data.", MessageType.Warning);
                return;
            }

            // Show aggregate information
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Summary Statistics", EditorStyles.miniBoldLabel);
            
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Spring Force:", GUILayout.Width(80));
            if (springForces.Distinct().Count() == 1)
            {
                EditorGUILayout.LabelField($"{springForces.First():F1}");
            }
            else
            {
                EditorGUILayout.LabelField($"{springForces.Min():F1} - {springForces.Max():F1} (avg: {springForces.Average():F1})");
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Damping:", GUILayout.Width(80));
            if (dampings.Distinct().Count() == 1)
            {
                EditorGUILayout.LabelField($"{dampings.First():F3}");
            }
            else
            {
                EditorGUILayout.LabelField($"{dampings.Min():F3} - {dampings.Max():F3} (avg: {dampings.Average():F3})");
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Rest Length:", GUILayout.Width(80));
            if (restLengths.Distinct().Count() == 1)
            {
                EditorGUILayout.LabelField($"{restLengths.First():F2}");
            }
            else
            {
                EditorGUILayout.LabelField($"{restLengths.Min():F2} - {restLengths.Max():F2} (avg: {restLengths.Average():F2})");
            }
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();

            // Show individual links only if there are few of them
            if (selectedLinks.Count <= 5)
            {
                EditorGUILayout.Space(5);
                EditorGUILayout.LabelField("Individual Links", EditorStyles.miniBoldLabel);
                foreach (var idx in editor.selectedLinkIndices.Take(5))
                {
                    if (idx >= 0 && idx < editor.links.Count)
                    {
                        var link = editor.links[idx];
                        EditorGUILayout.LabelField($"Link {idx}: Nodes {link.nodeA}↔{link.nodeB}");
                    }
                }
            }
            else
            {
                EditorGUILayout.Space(5);
                EditorGUILayout.LabelField($"Individual Links (showing first 3 of {selectedLinks.Count})", EditorStyles.miniBoldLabel);
                foreach (var idx in editor.selectedLinkIndices.Take(3))
                {
                    if (idx >= 0 && idx < editor.links.Count)
                    {
                        var link = editor.links[idx];
                        EditorGUILayout.LabelField($"Link {idx}: Nodes {link.nodeA}↔{link.nodeB}");
                    }
                }
                EditorGUILayout.LabelField($"... and {selectedLinks.Count - 3} more", EditorStyles.miniLabel);
            }
        }

        private void DrawNodeSelectionSummary()
        {
            // Safety checks to prevent GUI errors
            if (editor == null || editor.nodes == null || editor.selectedNodeIndices == null)
            {
                EditorGUILayout.HelpBox("Editor data not properly initialized.", MessageType.Warning);
                return;
            }

            var selectedNodes = editor.selectedNodeIndices
                .Where(idx => idx >= 0 && idx < editor.nodes.Count)
                .Select(idx => new { Index = idx, Position = editor.nodes[idx] })
                .ToList();

            if (!selectedNodes.Any()) return;

            // Count and overview
            EditorGUILayout.LabelField($"Selection: {selectedNodes.Count} node{(selectedNodes.Count > 1 ? "s" : "")}");

            // Calculate statistics with safety checks
            var positions = selectedNodes.Select(n => n.Position).ToList();
            var xPositions = positions.Select(p => p.x).Where(x => !float.IsNaN(x) && !float.IsInfinity(x)).ToList();
            var yPositions = positions.Select(p => p.y).Where(y => !float.IsNaN(y) && !float.IsInfinity(y)).ToList();
            var zPositions = positions.Select(p => p.z).Where(z => !float.IsNaN(z) && !float.IsInfinity(z)).ToList();

            if (!xPositions.Any() || !yPositions.Any() || !zPositions.Any())
            {
                EditorGUILayout.HelpBox("Selected nodes contain invalid position data.", MessageType.Warning);
                return;
            }

            // Show aggregate position information
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Position Statistics", EditorStyles.miniBoldLabel);
            
            // Center point
            Vector3 centerPoint = editor.GetSelectionCenter();
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Center:", GUILayout.Width(60));
            EditorGUILayout.LabelField($"({centerPoint.x:F2}, {centerPoint.y:F2}, {centerPoint.z:F2})");
            EditorGUILayout.EndHorizontal();

            // X Position range
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("X Range:", GUILayout.Width(60));
            if (xPositions.Distinct().Count() == 1)
            {
                EditorGUILayout.LabelField($"{xPositions.First():F2}");
            }
            else
            {
                EditorGUILayout.LabelField($"{xPositions.Min():F2} to {xPositions.Max():F2} (span: {(xPositions.Max() - xPositions.Min()):F2})");
            }
            EditorGUILayout.EndHorizontal();

            // Y Position range
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Y Range:", GUILayout.Width(60));
            if (yPositions.Distinct().Count() == 1)
            {
                EditorGUILayout.LabelField($"{yPositions.First():F2}");
            }
            else
            {
                EditorGUILayout.LabelField($"{yPositions.Min():F2} to {yPositions.Max():F2} (span: {(yPositions.Max() - yPositions.Min()):F2})");
            }
            EditorGUILayout.EndHorizontal();

            // Z Position range
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Z Range:", GUILayout.Width(60));
            if (zPositions.Distinct().Count() == 1)
            {
                EditorGUILayout.LabelField($"{zPositions.First():F2}");
            }
            else
            {
                EditorGUILayout.LabelField($"{zPositions.Min():F2} to {zPositions.Max():F2} (span: {(zPositions.Max() - zPositions.Min()):F2})");
            }
            EditorGUILayout.EndHorizontal();

            // Show pinned status information
            var pinnedCount = selectedNodes.Count(n => editor.pinnedNodes.Contains(n.Index));
            var unpinnedCount = selectedNodes.Count - pinnedCount;
            
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Status:", GUILayout.Width(60));
            if (pinnedCount > 0 && unpinnedCount > 0)
            {
                EditorGUILayout.LabelField($"{pinnedCount} pinned, {unpinnedCount} unpinned");
            }
            else if (pinnedCount > 0)
            {
                EditorGUILayout.LabelField($"All pinned ({pinnedCount})");
            }
            else
            {
                EditorGUILayout.LabelField($"All unpinned ({unpinnedCount})");
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.EndVertical();

            // Show individual nodes only if there are few of them
            if (selectedNodes.Count <= 5)
            {
                EditorGUILayout.Space(5);
                EditorGUILayout.LabelField("Individual Nodes", EditorStyles.miniBoldLabel);
                foreach (var node in selectedNodes.Take(5))
                {
                    string pinnedStatus = editor.pinnedNodes.Contains(node.Index) ? " (Pinned)" : "";
                    EditorGUILayout.LabelField($"Node {node.Index}: ({node.Position.x:F2}, {node.Position.y:F2}, {node.Position.z:F2}){pinnedStatus}");
                }
            }
            else
            {
                EditorGUILayout.Space(5);
                EditorGUILayout.LabelField($"Individual Nodes (showing first 3 of {selectedNodes.Count})", EditorStyles.miniBoldLabel);
                foreach (var node in selectedNodes.Take(3))
                {
                    string pinnedStatus = editor.pinnedNodes.Contains(node.Index) ? " (Pinned)" : "";
                    EditorGUILayout.LabelField($"Node {node.Index}: ({node.Position.x:F2}, {node.Position.y:F2}, {node.Position.z:F2}){pinnedStatus}");
                }
                EditorGUILayout.LabelField($"... and {selectedNodes.Count - 3} more", EditorStyles.miniLabel);
            }
        }

        private void DrawStretchLimitsTab()
        {
            stretchLimitsFoldout = EditorGUILayout.Foldout(stretchLimitsFoldout, "Stretch Limits", true, EditorStyles.boldLabel);
            if (stretchLimitsFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUI.BeginChangeCheck();
                float newMaxFactor = EditorGUILayout.FloatField(new GUIContent("Max Stretch Factor", "Maximum allowable stretch (e.g., 1.05 = 105% of rest length)"), editor.maxStretchFactor);
                float newMinFactor = EditorGUILayout.FloatField(new GUIContent("Min Stretch Factor", "Minimum allowable stretch (e.g., 0.95 = 95% of rest length)"), editor.minStretchFactor);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(editor, "Change Stretch Limits");
                    editor.maxStretchFactor = Mathf.Clamp(newMaxFactor, 1.0f, 1.5f);
                    editor.minStretchFactor = Mathf.Clamp(newMinFactor, 0.5f, 1.0f);
                    editor.ApplyStretchLimits();
                    serializedObject.FindProperty("maxStretchFactor").floatValue = editor.maxStretchFactor;
                    serializedObject.FindProperty("minStretchFactor").floatValue = editor.minStretchFactor;
                }

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Apply Rigid Settings")) editor.ApplyRigidSettings();
                if (GUILayout.Button("Reset to Defaults")) editor.ResetToDefaultSettings();
                EditorGUILayout.EndHorizontal();
                EditorGUI.indentLevel--;
            }
        }

        // Node naming mode state
        private bool isNodeNamingMode = false;
        private string nodeNamingText = "";
        private List<int> nodeNamingSelection = new List<int>();
        
        // Display mode settings - now automatically controlled by tabs
        private enum DisplayMode { Both, NodesOnly, LinksOnly }
        
        // Camera occlusion mode settings
        private bool alwaysSeeNodes = false;
        private bool alwaysSeeLinks = false;

        private DisplayMode GetCurrentDisplayMode()
        {
            // Determine display mode based on current tab
            switch (editor.currentTab)
            {
                case 1: // Nodes tab
                    return DisplayMode.NodesOnly;
                case 2: // Links tab
                    // Show both when creating links, otherwise only links
                    return (editor.creatingLinkNodeIndex >= 0) ? DisplayMode.Both : DisplayMode.LinksOnly;
                case 6: // Node Sets tab
                    return DisplayMode.NodesOnly;
                default:
                    return DisplayMode.Both; // Other tabs show both by default
            }
        }

        private void DrawNodeSetsTab()
        {
            EditorGUILayout.LabelField("Named Node Sets", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox("Create named groups of nodes for easy constraint setup.", MessageType.Info);
            
            // Camera Occlusion
            EditorGUI.BeginChangeCheck();
            alwaysSeeNodes = EditorGUILayout.Toggle(new GUIContent("Camera Occlusion", "When enabled, nodes show through mesh geometry"), alwaysSeeNodes);
            if (EditorGUI.EndChangeCheck())
            {
                SceneView.RepaintAll();
            }
            
            // Node Naming Mode Section
            EditorGUILayout.Space(5);
            EditorGUILayout.LabelField("Visual Node Naming", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            
            // Node naming mode toggle and controls
            EditorGUILayout.BeginHorizontal();
            
            bool newNamingMode = EditorGUILayout.Toggle("Node Naming Mode", isNodeNamingMode, GUILayout.Width(140));
            if (newNamingMode != isNodeNamingMode)
            {
                isNodeNamingMode = newNamingMode;
                if (isNodeNamingMode)
                {
                    // Entering naming mode - clear previous selection
                    nodeNamingSelection.Clear();
                    if (string.IsNullOrEmpty(nodeNamingText))
                        nodeNamingText = $"NodeSet{editor.nodeSets.Count + 1}";
                }
                else
                {
                    // Exiting naming mode - clear selection
                    nodeNamingSelection.Clear();
                }
                SceneView.RepaintAll();
            }
            
            GUI.enabled = isNodeNamingMode;
            nodeNamingText = EditorGUILayout.TextField("Name:", nodeNamingText);
            GUI.enabled = true;
            
            EditorGUILayout.EndHorizontal();
            
            if (isNodeNamingMode)
            {
                EditorGUILayout.HelpBox("🎯 Node Naming Mode Active: Click nodes in the Scene view to select them, then click 'Create Node Set' to assign the name.", MessageType.Info);
                
                // Show current selection in naming mode
                if (nodeNamingSelection.Count > 0)
                {
                    string selectionText = nodeNamingSelection.Count <= 10 ? 
                        string.Join(", ", nodeNamingSelection) : 
                        $"{string.Join(", ", nodeNamingSelection.Take(10))}... (+{nodeNamingSelection.Count - 10} more)";
                    EditorGUILayout.LabelField($"Selected Nodes ({nodeNamingSelection.Count}): {selectionText}", EditorStyles.miniLabel);
                }
                else
                {
                    EditorGUILayout.LabelField("No nodes selected. Click nodes in the Scene view to select them.", EditorStyles.miniLabel);
                }
                
                // Action buttons for naming mode
                EditorGUILayout.BeginHorizontal();
                
                GUI.enabled = nodeNamingSelection.Count > 0 && !string.IsNullOrEmpty(nodeNamingText);
                if (GUILayout.Button("Create Node Set", GUILayout.Height(25)))
                {
                    // Check if name already exists
                    if (editor.nodeSets.Any(ns => ns.name == nodeNamingText))
                    {
                        if (EditorUtility.DisplayDialog("Name Exists", $"A node set named '{nodeNamingText}' already exists. Replace it?", "Replace", "Cancel"))
                        {
                            editor.DeleteNodeSet(nodeNamingText);
                        }
                        else
                        {
                            return;
                        }
                    }
                    
                    editor.CreateNodeSet(nodeNamingText, new List<int>(nodeNamingSelection));
                    nodeNamingSelection.Clear();
                    nodeNamingText = $"NodeSet{editor.nodeSets.Count + 1}";
                    EditorUtility.SetDirty(editor);
                    SceneView.RepaintAll();
                }
                GUI.enabled = true;
                
                if (GUILayout.Button("Clear Selection", GUILayout.Height(25)))
                {
                    nodeNamingSelection.Clear();
                    SceneView.RepaintAll();
                }
                
                if (GUILayout.Button("Exit Naming Mode", GUILayout.Height(25)))
                {
                    isNodeNamingMode = false;
                    nodeNamingSelection.Clear();
                    SceneView.RepaintAll();
                }
                
                EditorGUILayout.EndHorizontal();
            }
            
            EditorGUILayout.EndVertical();
            
            // Create new node set section (traditional method)
            EditorGUILayout.Space(5);
            EditorGUILayout.LabelField("Traditional Creation", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Create from Selection", GUILayout.Height(25)))
            {
                if (editor.selectedNodeIndices.Count > 0)
                {
                    string defaultName = $"NodeSet{editor.nodeSets.Count + 1}";
                    CreateNodeSetDialog(defaultName, new List<int>(editor.selectedNodeIndices));
                }
                else
                {
                    EditorUtility.DisplayDialog("No Selection", "Please select nodes first before creating a node set.", "OK");
                }
            }
            
            if (GUILayout.Button("Create Empty Set", GUILayout.Height(25)))
            {
                string defaultName = $"NodeSet{editor.nodeSets.Count + 1}";
                CreateNodeSetDialog(defaultName, new List<int>());
            }
            EditorGUILayout.EndHorizontal();
            
            // Current selection info
            if (editor.selectedNodeIndices.Count > 0)
            {
                EditorGUILayout.HelpBox($"Current selection: {editor.selectedNodeIndices.Count} nodes ({string.Join(", ", editor.selectedNodeIndices.Take(5))}{(editor.selectedNodeIndices.Count > 5 ? "..." : "")})", MessageType.None);
            }
            
            EditorGUILayout.Space(10);
            
            // Existing node sets
            EditorGUILayout.LabelField("Existing Node Sets", EditorStyles.boldLabel);
            
            if (editor.nodeSets == null || editor.nodeSets.Count == 0)
            {
                EditorGUILayout.HelpBox("No node sets created yet. Create one by selecting nodes and clicking 'Create from Selection'.", MessageType.Info);
                return;
            }
            
            // Draw each node set
            for (int i = editor.nodeSets.Count - 1; i >= 0; i--)
            {
                var nodeSet = editor.nodeSets[i];
                if (nodeSet == null) continue;
                
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                
                // Header with name and actions
                EditorGUILayout.BeginHorizontal();
                
                // Name and edit button
                EditorGUI.BeginChangeCheck();
                string newName = EditorGUILayout.TextField(nodeSet.name, EditorStyles.boldLabel);
                if (EditorGUI.EndChangeCheck() && !string.IsNullOrEmpty(newName) && newName != nodeSet.name)
                {
                    if (!editor.nodeSets.Any(ns => ns.name == newName))
                    {
                        editor.RenameNodeSet(nodeSet.name, newName);
                        EditorUtility.SetDirty(editor);
                    }
                    else
                    {
                        EditorUtility.DisplayDialog("Name Conflict", $"A node set named '{newName}' already exists.", "OK");
                    }
                }
                
                GUILayout.FlexibleSpace();
                
                // Color picker
                EditorGUI.BeginChangeCheck();
                nodeSet.color = EditorGUILayout.ColorField(GUIContent.none, nodeSet.color, false, false, false, GUILayout.Width(40));
                if (EditorGUI.EndChangeCheck())
                {
                    EditorUtility.SetDirty(editor);
                }
                
                // Visibility toggle
                EditorGUI.BeginChangeCheck();
                nodeSet.isVisible = EditorGUILayout.Toggle(nodeSet.isVisible, GUILayout.Width(20));
                if (EditorGUI.EndChangeCheck())
                {
                    EditorUtility.SetDirty(editor);
                }
                
                // Delete button
                if (GUILayout.Button("✕", GUILayout.Width(25), GUILayout.Height(20)))
                {
                    if (EditorUtility.DisplayDialog("Delete Node Set", $"Are you sure you want to delete '{nodeSet.name}'?", "Delete", "Cancel"))
                    {
                        editor.DeleteNodeSet(nodeSet.name);
                        EditorUtility.SetDirty(editor);
                        continue;
                    }
                }
                
                EditorGUILayout.EndHorizontal();
                
                // Node count and indices
                var validIndices = nodeSet.nodeIndices.Where(idx => idx >= 0 && idx < editor.nodes.Count).ToList();
                EditorGUILayout.LabelField($"Nodes: {validIndices.Count} ({string.Join(", ", validIndices.Take(10))}{(validIndices.Count > 10 ? "..." : "")})");
                
                // Action buttons
                EditorGUILayout.BeginHorizontal();
                
                if (GUILayout.Button("Select", GUILayout.Height(20)))
                {
                    editor.SelectNodeSet(nodeSet.name);
                    SceneView.RepaintAll();
                }
                
                if (GUILayout.Button("Add Selected", GUILayout.Height(20)))
                {
                    if (editor.selectedNodeIndices.Count > 0)
                    {
                        var newIndices = new List<int>(nodeSet.nodeIndices);
                        foreach (int idx in editor.selectedNodeIndices)
                        {
                            if (!newIndices.Contains(idx))
                                newIndices.Add(idx);
                        }
                        editor.UpdateNodeSet(nodeSet.name, newIndices);
                        EditorUtility.SetDirty(editor);
                    }
                }
                
                if (GUILayout.Button("Remove Selected", GUILayout.Height(20)))
                {
                    if (editor.selectedNodeIndices.Count > 0)
                    {
                        var newIndices = nodeSet.nodeIndices.Where(idx => !editor.selectedNodeIndices.Contains(idx)).ToList();
                        if (newIndices.Count > 0)
                        {
                            editor.UpdateNodeSet(nodeSet.name, newIndices);
                        }
                        else
                        {
                            if (EditorUtility.DisplayDialog("Empty Node Set", $"Removing selected nodes would make '{nodeSet.name}' empty. Delete the node set?", "Delete", "Cancel"))
                            {
                                editor.DeleteNodeSet(nodeSet.name);
                            }
                        }
                        EditorUtility.SetDirty(editor);
                    }
                }
                
                if (GUILayout.Button("Clear", GUILayout.Height(20)))
                {
                    if (EditorUtility.DisplayDialog("Clear Node Set", $"Are you sure you want to clear all nodes from '{nodeSet.name}'?", "Clear", "Cancel"))
                    {
                        editor.DeleteNodeSet(nodeSet.name);
                        EditorUtility.SetDirty(editor);
                    }
                }
                
                EditorGUILayout.EndHorizontal();
                
                EditorGUILayout.EndVertical();
                EditorGUILayout.Space(3);
            }
            
            // Utility buttons
            EditorGUILayout.Space(10);
            EditorGUILayout.LabelField("Utilities", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Validate All Node Sets"))
            {
                editor.ValidateNodeSets();
                EditorUtility.SetDirty(editor);
            }
            
            if (GUILayout.Button("Clear All Node Sets"))
            {
                if (EditorUtility.DisplayDialog("Clear All Node Sets", "Are you sure you want to delete all node sets? This cannot be undone.", "Delete All", "Cancel"))
                {
                    editor.nodeSets.Clear();
                    editor.SaveToTrussAsset();
                    EditorUtility.SetDirty(editor);
                }
            }
            EditorGUILayout.EndHorizontal();
            
            // Show usage info
            EditorGUILayout.Space(5);
            EditorGUILayout.HelpBox("Node sets can be used in constraints by their name. For example, use 'WheelHub' instead of 'node0,node1,node2'.", MessageType.Info);
        }

        private void CreateNodeSetDialog(string defaultName, List<int> initialNodes)
        {
            string name = defaultName;
            
            // Simple approach - use a text field in the inspector
            // We'll just use the default name and let user rename later
            if (!string.IsNullOrEmpty(name))
            {
                editor.CreateNodeSet(name, initialNodes);
                EditorUtility.SetDirty(editor);
            }
        }

        private void ApplyPresetToSelectedLinks(MaterialProps preset)
        {
            if (preset == null) return;
            
            // Use the new method from NodeLinkEditor
            editor.ApplyMaterialPreset(preset);
            EditorUtility.SetDirty(editor);
        }

        public void OnSceneGUI()
        {
            if (Application.isPlaying) return;       // stop the whole editor

            Event e = Event.current;
            int controlID = GUIUtility.GetControlID(FocusType.Passive);
            HandleUtility.AddDefaultControl(controlID);

            // Always draw nodes and links when in nodes, links, or node sets tab
            if (editor.currentTab == 1 || editor.currentTab == 2 || editor.currentTab == 6)
            {
                DrawNodesAndLinks();
            }

            if (editor.isCreatingNode && editor.currentTab == 1)
            {
                HandleNodeCreation(e);
            }
            else if (editor.creatingLinkNodeIndex >= 0 && editor.currentTab == 2)
            {
                HandleLinkCreation(e);
            }
            else if (e.type == EventType.MouseDown && e.button == 0 && !editor.isCreatingNode && editor.creatingLinkNodeIndex < 0)
            {
                // Handle selection based on current tab and mode
                if (editor.currentTab == 1 || (editor.currentTab == 6 && isNodeNamingMode))
                {
                    HandleNodeSelection(e);
                }
                else if (editor.currentTab == 2)
                {
                    HandleLinkSelection(e);
                }
            }

            // Only allow transform handles in normal node mode, not in naming mode
            if (editor.selectedNodeIndices.Count > 0 && editor.currentTab == 1 && !isNodeNamingMode)
            {
                HandleNodeTransform(e);
            }

            if (GUI.changed) EditorUtility.SetDirty(editor);
            HandleUtility.Repaint();
        }

        private void DrawNodesAndLinks()
        {
            // In Node Naming Mode, always show all nodes prominently
            bool inNamingMode = editor.currentTab == 6 && isNodeNamingMode;
            
            // Get current display mode based on active tab
            DisplayMode currentDisplayMode = GetCurrentDisplayMode();
            
            // Enhanced clearance settings based on display mode
            float nodeSizeMultiplier = currentDisplayMode == DisplayMode.NodesOnly ? 1.4f : 1.0f;
            float linkThicknessMultiplier = currentDisplayMode == DisplayMode.LinksOnly ? 1.5f : 1.0f;
            float labelSpacing = currentDisplayMode == DisplayMode.NodesOnly ? 1.5f : 1.2f;
            
            // Draw node sets first (as colored backgrounds) - but not in naming mode or links-only mode
            if (editor.currentTab == 6 && editor.nodeSets != null && !inNamingMode && currentDisplayMode != DisplayMode.LinksOnly) 
            {
                foreach (var nodeSet in editor.nodeSets)
                {
                    if (nodeSet == null || !nodeSet.isVisible || nodeSet.nodeIndices.Count == 0) continue;
                    
                    // Apply depth test for node sets
                    Handles.zTest = alwaysSeeNodes ? 
                        UnityEngine.Rendering.CompareFunction.Always : 
                        UnityEngine.Rendering.CompareFunction.LessEqual;
                    
                    float nodeSetAlpha = 0.3f;
                    Handles.color = new Color(nodeSet.color.r, nodeSet.color.g, nodeSet.color.b, nodeSetAlpha);
                    
                    // Draw connections between nodes in the set to show grouping
                    for (int i = 0; i < nodeSet.nodeIndices.Count; i++)
                    {
                        for (int j = i + 1; j < nodeSet.nodeIndices.Count; j++)
                        {
                            int nodeA = nodeSet.nodeIndices[i];
                            int nodeB = nodeSet.nodeIndices[j];
                            
                            if (nodeA >= 0 && nodeA < editor.nodes.Count && nodeB >= 0 && nodeB < editor.nodes.Count)
                            {
                                Vector3 posA = editor.transform.TransformPoint(editor.nodes[nodeA]);
                                Vector3 posB = editor.transform.TransformPoint(editor.nodes[nodeB]);
                                Handles.DrawDottedLine(posA, posB, 2f);
                            }
                        }
                    }
                }
            }
            
            // Draw Nodes (if not in LinksOnly mode)
            if (currentDisplayMode != DisplayMode.LinksOnly)
            {
                // Apply depth test based on toggle
                Handles.zTest = alwaysSeeNodes ? 
                    UnityEngine.Rendering.CompareFunction.Always : 
                    UnityEngine.Rendering.CompareFunction.LessEqual;
                
                for (int i = 0; i < editor.nodes.Count; i++)
                {
                    Vector3 pos = editor.transform.TransformPoint(editor.nodes[i]);
                    
                    // Determine node color and size based on mode and state
                    Color nodeColor = editor.nodeColor;
                    float sizeMultiplier = nodeSizeMultiplier;
                    
                    if (inNamingMode)
                    {
                        // In naming mode, make all nodes highly visible
                        sizeMultiplier = 1.8f; // Even larger in naming mode
                        
                        if (nodeNamingSelection.Contains(i))
                        {
                            // Selected for naming - bright green
                            nodeColor = Color.green;
                            sizeMultiplier = 2.2f;
                        }
                        else
                        {
                            // Available for selection - bright cyan
                            nodeColor = Color.cyan;
                        }
                        // Full visibility in naming mode
                        nodeColor.a = 1.0f;
                    }
                    else
                    {
                        // Normal mode coloring
                        if (editor.pinnedNodes.Contains(i))
                            nodeColor = editor.pinnedNodeColor;
                        else if (editor.selectedNodeIndices.Contains(i))
                        {
                            nodeColor = editor.selectedNodeColor;
                            sizeMultiplier *= 1.3f; // Make selected nodes larger
                        }
                        else if (editor.currentTab == 6 && editor.nodeSets != null) // Node Sets tab
                        {
                            // Show node set color
                            var nodeSet = editor.nodeSets.FirstOrDefault(ns => ns.isVisible && ns.nodeIndices.Contains(i));
                            if (nodeSet != null)
                                nodeColor = nodeSet.color;
                        }
                        
                        nodeColor.a = 1.0f;
                    }
                    
                    Handles.color = nodeColor;
                    float size = HandleUtility.GetHandleSize(pos) * editor.nodeSize * sizeMultiplier;
                    Handles.SphereHandleCap(0, pos, Quaternion.identity, size, EventType.Repaint);
                    
                    // Draw node labels with better spacing
                    if (inNamingMode || currentDisplayMode == DisplayMode.NodesOnly)
                    {
                        // Enhanced labels for nodes-only or naming mode
                        var labelStyle = new GUIStyle(GUI.skin.label)
                        {
                            normal = { textColor = nodeColor },
                            fontStyle = FontStyle.Bold,
                            fontSize = currentDisplayMode == DisplayMode.NodesOnly ? 11 : 10
                        };
                        
                        Vector3 labelOffset = Vector3.up * (size * labelSpacing);
                        
                        if (inNamingMode)
                        {
                            Handles.Label(pos + labelOffset, $"Node {i}", labelStyle);
                        }
                        else if (currentDisplayMode == DisplayMode.NodesOnly)
                        {
                            // Show more details in nodes-only mode
                            string nodeInfo = $"N{i}";
                            if (editor.pinnedNodes.Contains(i))
                                nodeInfo += " (P)"; // P for Pinned
                                
                            Handles.Label(pos + labelOffset, nodeInfo, labelStyle);
                        }
                    }
                    else if (editor.currentTab == 6 && editor.nodeSets != null)
                    {
                        // In normal node sets mode, show node set names
                        var nodeSetsForNode = editor.nodeSets.Where(ns => ns.isVisible && ns.nodeIndices.Contains(i)).ToList();
                        if (nodeSetsForNode.Count > 0)
                        {
                            string label = string.Join(", ", nodeSetsForNode.Select(ns => ns.name));
                            var labelStyle = new GUIStyle(GUI.skin.label)
                            {
                                normal = { textColor = nodeColor }
                            };
                            Handles.Label(pos + Vector3.up * (size * 2), $"{i}: {label}", labelStyle);
                        }
                    }
                }
            }

            // Draw Links (if not in NodesOnly mode)
            if (currentDisplayMode != DisplayMode.NodesOnly)
            {
                // Apply depth test based on toggle
                Handles.zTest = alwaysSeeLinks ? 
                    UnityEngine.Rendering.CompareFunction.Always : 
                    UnityEngine.Rendering.CompareFunction.LessEqual;
                
                for (int i = 0; i < editor.links.Count; i++)
                {
                    var link = editor.links[i];
                    if (link.nodeA >= 0 && link.nodeA < editor.nodes.Count && link.nodeB >= 0 && link.nodeB < editor.nodes.Count)
                    {
                        Vector3 posA = editor.transform.TransformPoint(editor.nodes[link.nodeA]);
                        Vector3 posB = editor.transform.TransformPoint(editor.nodes[link.nodeB]);
                        
                        // Enhanced link visualization
                        Color linkColor = editor.selectedLinkIndices.Contains(i) ? editor.selectedLinkColor : editor.linkColor;
                        linkColor.a = 1.0f;
                        
                        // Make selected links more prominent
                        float thickness = editor.linkThickness * linkThicknessMultiplier;
                        if (editor.selectedLinkIndices.Contains(i))
                        {
                            thickness *= 1.5f; // Make selected links thicker
                        }
                        
                        Handles.color = linkColor;
                        Handles.DrawLine(posA, posB, thickness);
                        
                        // Draw link labels in links-only mode
                        if (currentDisplayMode == DisplayMode.LinksOnly)
                        {
                            Vector3 midPoint = (posA + posB) * 0.5f;
                            
                            // Only show labels for selected links to avoid clutter
                            if (editor.selectedLinkIndices.Contains(i))
                            {
                                var labelStyle = new GUIStyle(GUI.skin.label)
                                {
                                    normal = { textColor = linkColor },
                                    fontStyle = FontStyle.Bold,
                                    fontSize = 9
                                };
                                
                                string linkInfo = $"L{i} ({link.nodeA}↔{link.nodeB})";
                                Handles.Label(midPoint + Vector3.up * 0.3f, linkInfo, labelStyle);
                            }
                        }
                    }
                }
            }
            
            // Reset depth test to default
            Handles.zTest = UnityEngine.Rendering.CompareFunction.LessEqual;
        }

        private void HandleNodeCreation(Event e)
        {
            Ray mouseRay = HandleUtility.GUIPointToWorldRay(e.mousePosition);
            RaycastHit hit;
            Vector3 position;
            bool hitValidPosition = Physics.Raycast(mouseRay, out hit);

            position = hitValidPosition ? hit.point : mouseRay.origin + mouseRay.direction * 10f;
            Handles.color = hitValidPosition ? Color.green : Color.red;
            Handles.SphereHandleCap(0, position, Quaternion.identity, editor.nodeSize, EventType.Repaint);

            if (e.type == EventType.MouseDown && e.button == 0)
            {
                position = editor.transform.InverseTransformPoint(position);
                editor.CreateNode(position);
                e.Use();
            }
        }

        private void HandleLinkCreation(Event e)
        {
            for (int i = 0; i < editor.nodes.Count; i++)
            {
                Vector3 position = editor.transform.TransformPoint(editor.nodes[i]);
                Handles.color = Color.blue;
                float size = HandleUtility.GetHandleSize(position) * editor.nodeSize * 2f;
                Handles.SphereHandleCap(0, position, Quaternion.identity, size, EventType.Repaint);

                if (e.type == EventType.MouseDown && e.button == 0)
                {
                    if (HandleUtility.DistanceToCircle(position, size) <= 0)
                    {
                        if (editor.creatingLinkNodeIndex == 0)
                        {
                            editor.creatingLinkNodeIndex = i;
                        }
                        else if (i != editor.creatingLinkNodeIndex)
                        {
                            editor.CreateLink(editor.creatingLinkNodeIndex, i);
                            editor.creatingLinkNodeIndex = -1;
                            e.Use();
                        }
                    }
                }
            }

            if (editor.creatingLinkNodeIndex >= 0 && editor.creatingLinkNodeIndex < editor.nodes.Count)
            {
                Vector3 startPos = editor.transform.TransformPoint(editor.nodes[editor.creatingLinkNodeIndex]);
                Ray mouseRay = HandleUtility.GUIPointToWorldRay(e.mousePosition);
                Vector3 endPos = mouseRay.origin + mouseRay.direction * 10f;
                Handles.color = Color.red;
                Handles.DrawLine(startPos, endPos, 2f);
            }
        }

        private void HandleNodeSelection(Event e)
        {
            // Handle node naming mode selection differently
            if (editor.currentTab == 6 && isNodeNamingMode)
            {
                HandleNodeNamingSelection(e);
                return;
            }
            
            // Normal node selection
            for (int i = 0; i < editor.nodes.Count; i++)
            {
                Vector3 pos = editor.transform.TransformPoint(editor.nodes[i]);
                float size = HandleUtility.GetHandleSize(pos) * editor.nodeSize;
                if (HandleUtility.DistanceToCircle(pos, size) <= 0)
                {
                    Undo.RecordObject(editor, "Select Node");
                    if (e.control)
                    {
                        if (editor.selectedNodeIndices.Contains(i))
                            editor.selectedNodeIndices.Remove(i);
                        else
                            editor.selectedNodeIndices.Add(i);
                    }
                    else
                    {
                        editor.selectedNodeIndices.Clear();
                        editor.selectedNodeIndices.Add(i);
                    }
                    editor.selectedLinkIndices.Clear();
                    EditorUtility.SetDirty(editor);
                    e.Use();
                    break;
                }
            }
        }

        private void HandleNodeNamingSelection(Event e)
        {
            for (int i = 0; i < editor.nodes.Count; i++)
            {
                Vector3 pos = editor.transform.TransformPoint(editor.nodes[i]);
                float size = HandleUtility.GetHandleSize(pos) * editor.nodeSize * 1.5f; // Larger hit area in naming mode
                if (HandleUtility.DistanceToCircle(pos, size) <= 0)
                {
                    // Toggle node in naming selection
                    if (nodeNamingSelection.Contains(i))
                    {
                        nodeNamingSelection.Remove(i);
                    }
                    else
                    {
                        nodeNamingSelection.Add(i);
                    }
                    
                    // Repaint to show selection change
                    SceneView.RepaintAll();
                    e.Use();
                    break;
                }
            }
        }

        private void HandleLinkSelection(Event e)
        {
            for (int i = 0; i < editor.links.Count; i++)
            {
                var link = editor.links[i];
                if (link.nodeA >= 0 && link.nodeA < editor.nodes.Count && link.nodeB >= 0 && link.nodeB < editor.nodes.Count)
                {
                    Vector3 posA = editor.transform.TransformPoint(editor.nodes[link.nodeA]);
                    Vector3 posB = editor.transform.TransformPoint(editor.nodes[link.nodeB]);
                    Vector3 guiPointA = HandleUtility.WorldToGUIPoint(posA);
                    Vector3 guiPointB = HandleUtility.WorldToGUIPoint(posB);
                    if (HandleUtility.DistancePointToLineSegment(e.mousePosition, guiPointA, guiPointB) < 3f)
                    {
                        Undo.RecordObject(editor, "Select Link");
                        if (e.control)
                        {
                            if (editor.selectedLinkIndices.Contains(i))
                                editor.selectedLinkIndices.Remove(i);
                            else
                                editor.selectedLinkIndices.Add(i);
                        }
                        else
                        {
                            editor.selectedLinkIndices.Clear();
                            editor.selectedLinkIndices.Add(i);
                        }
                        editor.selectedNodeIndices.Clear();
                        EditorUtility.SetDirty(editor);
                        e.Use();
                        break;
                    }
                }
            }
        }

        private void HandleNodeTransform(Event e)
        {
            editor.selectedNodeIndices.RemoveAll(i => i < 0 || i >= editor.nodes.Count);
            if (editor.selectedNodeIndices.Count == 0) return;

            Vector3 pos = editor.transform.TransformPoint(editor.GetSelectionCenter());
            if (!isDragging)
            {
                handlePosition = pos;
            }

            EditorGUI.BeginChangeCheck();
            Vector3 newPos = Handles.PositionHandle(pos, Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                isDragging = true;
                Vector3 delta = editor.transform.InverseTransformPoint(newPos) - editor.transform.InverseTransformPoint(pos);
                editor.TransformSelectedNodes(delta);
                handlePosition = newPos;
            }
            else if (e.type == EventType.MouseUp)
            {
                isDragging = false;
            }
        }
    }
}
#endif