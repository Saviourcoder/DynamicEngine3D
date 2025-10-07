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
    [CustomEditor(typeof(SoftBody))]
    public class SoftBodyEditor : UnityEditor.Editor
    {
        private SoftBody softBody;
        private float previousScaledMass;
        private bool showVisualizationSettings = false;

        void OnEnable()
        {
            softBody = (SoftBody)target;
            previousScaledMass = softBody.ScaledMass;
            showVisualizationSettings = EditorPrefs.GetBool("SoftBodyEditor_ShowVisualization", false);
        }

        void OnDisable()
        {
            EditorPrefs.SetBool("SoftBodyEditor_ShowVisualization", showVisualizationSettings);
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("SoftBody Configuration", EditorStyles.boldLabel);

            // Draw truss field
            EditorGUILayout.PropertyField(serializedObject.FindProperty("truss"));

            // Custom Scaled Mass field with automatic distribution feedback
            EditorGUI.BeginChangeCheck();
            float newScaledMass = EditorGUILayout.FloatField("Scaled Mass", softBody.ScaledMass);

            if (EditorGUI.EndChangeCheck())
            {
                if (newScaledMass < 0.01f)
                    newScaledMass = 0.01f;

                Undo.RecordObject(softBody, "Change Scaled Mass");
                softBody.ScaledMass = newScaledMass;
                previousScaledMass = newScaledMass;
                EditorUtility.SetDirty(softBody);

                if (softBody.solver?.nodeManager?.Nodes != null && softBody.solver.nodeManager.Nodes.Count > 0)
                {
                    float massPerNode = newScaledMass / softBody.solver.nodeManager.Nodes.Count;
                    Debug.Log($"SoftBody: Scaled Mass automatically distributed - {newScaledMass:F2}kg total, {massPerNode:F3}kg per node across {softBody.solver.nodeManager.Nodes.Count} nodes.");
                }

                // Sync to Truss asset if assigned (for persistence)
                if (softBody.truss != null)
                {
                    int nodeCount = softBody.truss.NodePositions.Length;
                    if (nodeCount > 0)
                    {
                        float massPerNode = newScaledMass / nodeCount;
                        softBody.truss.SetUniformNodeMass(massPerNode);
                        EditorUtility.SetDirty(softBody.truss);
                        Debug.Log($"Updated Truss node masses to uniform {massPerNode:F3}kg per node (total {newScaledMass:F2}kg)");
                    }
                }
            }

            // Draw other properties
            var influenceRadiusProp = serializedObject.FindProperty("influenceRadius");
            if (influenceRadiusProp != null)
                EditorGUILayout.PropertyField(influenceRadiusProp);

            EditorGUILayout.Space();

            // Internal Pressure System
            EditorGUILayout.LabelField("Internal Pressure System", EditorStyles.boldLabel);
            var enableInternalPressureProp = serializedObject.FindProperty("enableInternalPressure");
            if (enableInternalPressureProp != null)
                EditorGUILayout.PropertyField(enableInternalPressureProp);

            if (softBody.enableInternalPressure)
            {
                var psiProp = serializedObject.FindProperty("PSI");
                if (psiProp != null)
                    EditorGUILayout.PropertyField(psiProp);

                var showForcesProp = serializedObject.FindProperty("showForces");
                if (showForcesProp != null)
                    EditorGUILayout.PropertyField(showForcesProp);

                var pressureDampingProp = serializedObject.FindProperty("pressureDamping");
                if (pressureDampingProp != null)
                    EditorGUILayout.PropertyField(pressureDampingProp);
            }

            EditorGUILayout.Space();

            // Runtime Visualization
            showVisualizationSettings = EditorGUILayout.Foldout(showVisualizationSettings, "Runtime Visualization (Debug)", true, EditorStyles.foldoutHeader);
            if (showVisualizationSettings)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.HelpBox("These settings control debug visualization. Node grabbing is handled by CameraVisualizerController.", MessageType.Info);

                var showNodesProp = serializedObject.FindProperty("showNodes");
                if (showNodesProp != null)
                    EditorGUILayout.PropertyField(showNodesProp, new GUIContent("Show Nodes (Debug)", "Show nodes for debugging purposes"));

                var showLinksProp = serializedObject.FindProperty("showLinks");
                if (showLinksProp != null)
                    EditorGUILayout.PropertyField(showLinksProp);

                var showLinkForcesProp = serializedObject.FindProperty("showLinkForces");
                if (showLinkForcesProp != null)
                    EditorGUILayout.PropertyField(showLinkForcesProp);

                var showInfluenceRadiusProp = serializedObject.FindProperty("showInfluenceRadius");
                if (showInfluenceRadiusProp != null)
                    EditorGUILayout.PropertyField(showInfluenceRadiusProp);

                var nodeDisplaySizeProp = serializedObject.FindProperty("nodeDisplaySize");
                if (nodeDisplaySizeProp != null)
                    EditorGUILayout.PropertyField(nodeDisplaySizeProp);

                var forceVisualizationScaleProp = serializedObject.FindProperty("forceVisualizationScale");
                if (forceVisualizationScaleProp != null)
                    EditorGUILayout.PropertyField(forceVisualizationScaleProp);

                EditorGUI.indentLevel--;
            }

            serializedObject.ApplyModifiedProperties();
        }

        void OnSceneGUI()
        {
            if (softBody == null || !softBody.enabled)
                return;

            Event e = Event.current;
            if (e.control && softBody.solver?.nodeManager?.Nodes != null)
            {
                Camera sceneCam = SceneView.lastActiveSceneView?.camera;
                if (sceneCam == null) return;

                // Draw debug nodes
                if (softBody.ShowNodes)
                {
                    for (int i = 0; i < softBody.solver.nodeManager.Nodes.Count; i++)
                    {
                        if (softBody.solver.nodeManager.Nodes[i] == null)
                            continue;
                        
                        Vector3 nodePos = softBody.solver.nodeManager.Nodes[i].position;
                        bool isPinned = i < softBody.solver.nodeManager.IsPinned.Count && softBody.solver.nodeManager.IsPinned[i];
                        Color nodeColor = isPinned ? softBody.solver.pinnedNodeColor : softBody.solver.nodeColor;

                        Vector3 normal = (nodePos - sceneCam.transform.position).normalized;
                        Handles.color = nodeColor;
                        Handles.DrawSolidDisc(nodePos, normal, softBody.nodeDisplaySize);
                    }
                }

                // Draw debug links
                if (softBody.ShowLinks)
                {
                    Handles.color = softBody.solver.linkColor;
                    if (softBody.solver.beams != null)
                    {
                        foreach (var beam in softBody.solver.beams)
                        {
                            if (beam.nodeA < 0 || beam.nodeA >= softBody.solver.nodeManager.Nodes.Count ||
                                beam.nodeB < 0 || beam.nodeB >= softBody.solver.nodeManager.Nodes.Count ||
                                softBody.solver.nodeManager.Nodes[beam.nodeA] == null ||
                                softBody.solver.nodeManager.Nodes[beam.nodeB] == null)
                                continue;

                            Vector3 pos1 = softBody.solver.nodeManager.Nodes[beam.nodeA].position;
                            Vector3 pos2 = softBody.solver.nodeManager.Nodes[beam.nodeB].position;
                            float currentLength = Vector3.Distance(pos1, pos2);
                            Color color = softBody.solver.linkColor;
                            if (currentLength > beam.restLength * 1.01f)
                                color = softBody.solver.stretchedLinkColor;
                            else if (currentLength < beam.restLength * 0.99f)
                                color = softBody.solver.compressedLinkColor;
                            Handles.color = color;
                            Handles.DrawLine(pos1, pos2);
                        }
                    }
                }

                SceneView.RepaintAll();
            }
        }
    }

    public static class SoftBodyMenu
    {
        [MenuItem("GameObject/DynamicEngine/SoftBody", false, 20)]
        public static void CreateSoftBody()
        {
            GameObject selectedObj = Selection.activeGameObject;
            if (selectedObj == null)
            {
                EditorUtility.DisplayDialog("No Selection", "Please select a GameObject first.", "OK");
                return;
            }

            SoftBody existingSoftBody = selectedObj.GetComponent<SoftBody>();
            if (existingSoftBody != null)
            {
                EditorUtility.DisplayDialog("Already Exists", "This GameObject already has a SoftBody component.", "OK");
                return;
            }

            Undo.AddComponent<SoftBody>(selectedObj);
            Debug.Log($"Added SoftBody to {selectedObj.name}");
        }

        [MenuItem("GameObject/DynamicEngine/SoftBody", true)]
        public static bool CreateSoftBodyValidate()
        {
            GameObject selectedObj = Selection.activeGameObject;
            return selectedObj != null && selectedObj.GetComponent<SoftBody>() == null;
        }
    }
}