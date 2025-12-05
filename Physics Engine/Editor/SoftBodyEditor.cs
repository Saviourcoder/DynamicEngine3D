/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers */
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Reflection;
namespace DynamicEngine
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(SoftBody), true)]
    public class SoftBodyEditor : UnityEditor.Editor
    {
        private SoftBody softBody;
        private float previousScaledMass;
        private bool showVisualizationSettings = false;
        private bool showDeformationSettings = false;
        private bool showPressureSettings = false;
        void OnEnable()
        {
            softBody = (SoftBody)target;
            previousScaledMass = softBody.ScaledMass;
            showVisualizationSettings = EditorPrefs.GetBool("SoftBodyEditor_ShowVisualization", false);
            showDeformationSettings = EditorPrefs.GetBool("SoftBodyEditor_ShowDeformation", false);
            showPressureSettings = EditorPrefs.GetBool("SoftBodyEditor_ShowPressure", false);
        }
        void OnDisable()
        {
            EditorPrefs.SetBool("SoftBodyEditor_ShowVisualization", showVisualizationSettings);
            EditorPrefs.SetBool("SoftBodyEditor_ShowDeformation", showDeformationSettings);
            EditorPrefs.SetBool("SoftBodyEditor_ShowPressure", showPressureSettings);
        }
        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUI.BeginChangeCheck();
            // Truss field
            EditorGUILayout.PropertyField(serializedObject.FindProperty("truss"));

            EditorGUILayout.PropertyField(serializedObject.FindProperty("matter"));

            // Scaled Mass with feedback
            EditorGUI.BeginChangeCheck();
            float newScaledMass = EditorGUILayout.FloatField("Mass", softBody.ScaledMass);
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
                    Debug.Log($"SoftBody: Scaled Mass distributed - {newScaledMass:F2}kg total, {massPerNode:F3}kg per node across {softBody.solver.nodeManager.Nodes.Count} nodes.");
                }
                if (softBody.truss != null)
                {
                    int nodeCount = softBody.truss.NodePositions.Length;
                    if (nodeCount > 0)
                    {
                        float massPerNode = newScaledMass / nodeCount;
                        softBody.truss.SetUniformNodeMass(massPerNode);
                        EditorUtility.SetDirty(softBody.truss);
                        Debug.Log($"Updated Truss node masses to {massPerNode:F3}kg per node");
                    }
                }
            }

            // Deformation Settings
            showDeformationSettings = EditorGUILayout.Foldout(showDeformationSettings, "Deformation Settings", true, EditorStyles.foldoutHeader);
            if (showDeformationSettings)
            {
                EditorGUI.indentLevel++;
                var useAdvancedSkinningProp = serializedObject.FindProperty("useAdvancedSkinning");
                if (useAdvancedSkinningProp != null)
                {
                    EditorGUILayout.PropertyField(useAdvancedSkinningProp, new GUIContent("Advanced Skinning",
                    "Use advanced skinning for complex meshes. Recommended for detailed models."));
                    if (useAdvancedSkinningProp.boolValue)
                    {
                        EditorGUILayout.HelpBox("Advanced: Inverse distance weighting with 4 closest nodes. Best for complex meshes.", MessageType.Info);
                    }
                    else
                    {
                        EditorGUILayout.HelpBox("Basic: Influence radius-based. Works well for simple geometries.", MessageType.Info);
                    }
                }
                EditorGUI.indentLevel--;
            }
            EditorGUILayout.Space();
            // Internal Pressure System (now foldout)
            showPressureSettings = EditorGUILayout.Foldout(showPressureSettings, "Internal Pressure System", true, EditorStyles.foldoutHeader);
            if (showPressureSettings)
            {
                EditorGUI.indentLevel++;
                var enableInternalPressureProp = serializedObject.FindProperty("enableInternalPressure");
                if (enableInternalPressureProp != null)
                    EditorGUILayout.PropertyField(enableInternalPressureProp, new GUIContent("Enable Pressure"));
                if (enableInternalPressureProp != null && enableInternalPressureProp.boolValue)
                {
                    var psiProp = serializedObject.FindProperty("PSI");
                    if (psiProp != null)
                        EditorGUILayout.PropertyField(psiProp, new GUIContent("Pressure (PSI)"));
                    var pressureScaleProp = serializedObject.FindProperty("pressureScale");
                    if (pressureScaleProp != null)
                        EditorGUILayout.PropertyField(pressureScaleProp, new GUIContent("Pressure Scale"));
                    var pressureDampingProp = serializedObject.FindProperty("pressureDamping");
                    if (pressureDampingProp != null)
                        EditorGUILayout.PropertyField(pressureDampingProp, new GUIContent("Damping"));

                }
                EditorGUI.indentLevel--;
            }
            EditorGUILayout.Space();
            // Runtime Visualization
            showVisualizationSettings = EditorGUILayout.Foldout(showVisualizationSettings, "Debug Visualization", true, EditorStyles.foldoutHeader);
            if (showVisualizationSettings)
            {
                EditorGUI.indentLevel++;
                var showNodesProp = serializedObject.FindProperty("showNodes");
                if (showNodesProp != null)
                    EditorGUILayout.PropertyField(showNodesProp, new GUIContent("Show Nodes"));
                var showLinksProp = serializedObject.FindProperty("showLinks");
                if (showLinksProp != null)
                    EditorGUILayout.PropertyField(showLinksProp, new GUIContent("Show Links"));
                var showInfluenceRadiusProp = serializedObject.FindProperty("showInfluenceRadius");
                if (showInfluenceRadiusProp != null)
                    EditorGUILayout.PropertyField(showInfluenceRadiusProp, new GUIContent("Show Influence Areas"));
                var showSkinningInfluencesProp = serializedObject.FindProperty("showSkinningInfluences");
                if (showSkinningInfluencesProp != null)
                    EditorGUILayout.PropertyField(showSkinningInfluencesProp, new GUIContent("Show Skinning Influences"));
                var nodeDisplaySizeProp = serializedObject.FindProperty("nodeDisplaySize");
                if (nodeDisplaySizeProp != null)
                    EditorGUILayout.PropertyField(nodeDisplaySizeProp, new GUIContent("Node Display Size"));
                EditorGUI.indentLevel--;
            }
            EditorGUILayout.Space();
            if (GUILayout.Button("Reinitialize SoftBody"))
            {
                softBody.InitializeInEditMode();
                EditorUtility.SetDirty(softBody);
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
                            Color color = softBody.solver.linkColor;
                            Handles.color = color;
                            Handles.DrawLine(pos1, pos2);
                        }
                    }
                }
                // Draw influence areas
                if (softBody.ShowInfluenceRadius)
                {
                    Handles.color = softBody.solver.influenceRadiusColor;
                    foreach (var node in softBody.solver.nodeManager.Nodes)
                    {
                        if (node == null) continue;
                        Handles.DrawWireDisc(node.position, Vector3.up, softBody.solver.meshDeformer.InfluenceRadius);
                        Handles.DrawWireDisc(node.position, Vector3.right, softBody.solver.meshDeformer.InfluenceRadius);
                        Handles.DrawWireDisc(node.position, Vector3.forward, softBody.solver.meshDeformer.InfluenceRadius);
                    }
                    // Draw pressure normals in editor (access non-public flags via reflection if needed)
                    bool showPressureNormals = false;
                    bool enableInternalPressure = false;
                    var sbType = softBody.GetType();
                    var propEnable = sbType.GetProperty("enableInternalPressure", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                    if (propEnable != null)
                        enableInternalPressure = (bool)propEnable.GetValue(softBody);
                    else
                    {
                        var fieldEnable = sbType.GetField("enableInternalPressure", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                        if (fieldEnable != null)
                            enableInternalPressure = (bool)fieldEnable.GetValue(softBody);
                    }
                    if (showPressureNormals && enableInternalPressure)
                    {
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