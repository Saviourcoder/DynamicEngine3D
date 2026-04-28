/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */

using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

namespace DynamicEngine
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(SoftBody), true)]
    public class SoftBodyEditor : UnityEditor.Editor
    {
        private SoftBody softBody;
        private float previousTotalMass;
        private Texture2D cachedBanner;

        // Foldout states only — persisted per-editor via EditorPrefs.
        // All visualization values live on the component as serialized fields.
        private bool showDeformationSettings = false;
        private bool showPressureSettings = false;
        private bool showVisualizationSettings = false;

        private const string k_ShowViz = "SoftBodyEditor_ShowVisualization";

        void OnEnable()
        {
            softBody = (SoftBody)target;
            previousTotalMass = softBody.TotalMass;
            showDeformationSettings = EditorPrefs.GetBool("SoftBodyEditor_ShowDeformation", false);
            showPressureSettings = EditorPrefs.GetBool("SoftBodyEditor_ShowPressure", true);
            showVisualizationSettings = EditorPrefs.GetBool(k_ShowViz, false);
        }

        void OnDisable()
        {
            EditorPrefs.SetBool("SoftBodyEditor_ShowDeformation", showDeformationSettings);
            EditorPrefs.SetBool("SoftBodyEditor_ShowPressure", showPressureSettings);
            EditorPrefs.SetBool(k_ShowViz, showVisualizationSettings);
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            DrawBanner();

            // ── Truss / Matter ─────────────────────────────────────────────────
            EditorGUILayout.PropertyField(serializedObject.FindProperty("truss"));

            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(serializedObject.FindProperty("matter"));
            if (EditorGUI.EndChangeCheck() && Application.isPlaying)
            {
                if (softBody.solver != null && softBody.GetMatterAsset() != null)
                {
                    softBody.solver.SetMatterAsset(softBody.GetMatterAsset());
                    Debug.Log("Matter settings applied to solver");
                }
            }

            if (softBody.GetMatterAsset() == null)
                EditorGUILayout.HelpBox("No Matter asset assigned. Using default friction and restitution values.", MessageType.Info);

            EditorGUILayout.Space();

            // ── Total Mass ─────────────────────────────────────────────────────
            EditorGUI.BeginChangeCheck();
            float newTotalMass = EditorGUILayout.FloatField(
                new GUIContent("Total Mass (kg)", "Total mass evenly distributed across all nodes"),
                softBody.TotalMass);

            if (EditorGUI.EndChangeCheck())
            {
                if (newTotalMass < 0.01f) newTotalMass = 0.01f;

                Undo.RecordObject(softBody, "Change Total Mass");
                softBody.TotalMass = newTotalMass;
                previousTotalMass = newTotalMass;
                EditorUtility.SetDirty(softBody);

                if (softBody.solver?.nodeManager != null && softBody.solver.nodeManager.NodeCount > 0)
                {
                    float massPerNode = newTotalMass / softBody.solver.nodeManager.NodeCount;
                    Debug.Log($"SoftBody: Mass updated - {newTotalMass:F2}kg total, {massPerNode:F4}kg per node.");
                }
            }

            EditorGUILayout.Space();

            // ── Pressure Settings ──────────────────────────────────────────────
            showPressureSettings = EditorGUILayout.Foldout(showPressureSettings, "Pressure Settings", true, EditorStyles.foldoutHeader);
            if (showPressureSettings)
            {
                EditorGUI.indentLevel++;

                EditorGUI.BeginChangeCheck();
                var pressureProp = serializedObject.FindProperty("internalPressure");
                EditorGUILayout.PropertyField(pressureProp, new GUIContent("Internal Pressure (PSI)",
                    "Pressure applied from the center outwards. Simulates internal air/gas pressure."));

                if (EditorGUI.EndChangeCheck())
                {
                    serializedObject.ApplyModifiedProperties();
                    if (Application.isPlaying && softBody.solver != null)
                        softBody.solver.internalPressure = softBody.InternalPressure;
                }

                if (softBody.InternalPressure > 0f)
                    EditorGUILayout.HelpBox(
                        $"Pressure: {softBody.InternalPressure:F2} PSI - Forces push nodes outward from volume center.\n" +
                        "Higher values create more rigid, inflated structures.",
                        MessageType.Info);
                else
                    EditorGUILayout.HelpBox("Pressure disabled. Set > 0 to inflate the soft body.", MessageType.None);

                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();

            // ── Deformation Settings ───────────────────────────────────────────
            showDeformationSettings = EditorGUILayout.Foldout(showDeformationSettings, "Deformation Settings", true, EditorStyles.foldoutHeader);
            if (showDeformationSettings)
            {
                EditorGUI.indentLevel++;

                var useAdvancedSkinningProp = serializedObject.FindProperty("useAdvancedSkinning");
                if (useAdvancedSkinningProp != null)
                {
                    EditorGUILayout.PropertyField(useAdvancedSkinningProp, new GUIContent("Advanced Skinning",
                        "Use advanced skinning for complex meshes. Recommended for detailed models."));

                    EditorGUILayout.HelpBox(useAdvancedSkinningProp.boolValue
                        ? "Advanced: Inverse distance weighting with 4 closest nodes. Best for complex meshes."
                        : "Basic: Influence radius-based. Works well for simple geometries.",
                        MessageType.Info);
                }

                softBody.influenceRadius = EditorGUILayout.FloatField("Influence Radius", softBody.influenceRadius);
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();

            // ── Visualization ──────────────────────────────────────────────────
            // Settings are serialized on the component — toggling them here
            // updates the Game view (OnRenderObject/GL) and Scene view (OnSceneGUI/Handles)
            // simultaneously with no extra bookkeeping.
            showVisualizationSettings = EditorGUILayout.Foldout(showVisualizationSettings, "Visualization", true, EditorStyles.foldoutHeader);
            if (showVisualizationSettings)
            {
                EditorGUI.indentLevel++;

                EditorGUI.BeginChangeCheck();

                var pNodes = serializedObject.FindProperty("vizNodes");
                var pBeams = serializedObject.FindProperty("vizBeams");
                var pPinned = serializedObject.FindProperty("vizPinnedNodes");
                var pInactive = serializedObject.FindProperty("vizInactiveBeams");
                var pNodeColor = serializedObject.FindProperty("vizNodeColor");
                var pBeamColor = serializedObject.FindProperty("vizBeamColor");
                var pPinnedColor = serializedObject.FindProperty("vizPinnedColor");
                var pInactColor = serializedObject.FindProperty("vizInactiveColor");
                var pCrossColor = serializedObject.FindProperty("vizCrossBodyColor");
                var pNodeSize = serializedObject.FindProperty("vizNodeSize");

                // Nodes
                EditorGUILayout.PropertyField(pNodes, new GUIContent("Show Nodes",
                    "Draw a cross marker at each physics node (Game & Scene views)."));

                if (pNodes.boolValue)
                {
                    EditorGUI.indentLevel++;
                    EditorGUILayout.PropertyField(pNodeColor, new GUIContent("Node Color"));
                    EditorGUILayout.PropertyField(pNodeSize, new GUIContent("Node Size"));
                    EditorGUILayout.PropertyField(pPinned, new GUIContent("Highlight Pinned",
                        "Show fixed/pinned nodes in a separate colour."));
                    if (pPinned.boolValue)
                        EditorGUILayout.PropertyField(pPinnedColor, new GUIContent("Pinned Color"));
                    EditorGUI.indentLevel--;
                }

                EditorGUILayout.Space(2);

                // Beams
                EditorGUILayout.PropertyField(pBeams, new GUIContent("Show Beams",
                    "Draw lines between connected node pairs (Game & Scene views)."));

                if (pBeams.boolValue)
                {
                    EditorGUI.indentLevel++;
                    EditorGUILayout.PropertyField(pBeamColor, new GUIContent("Beam Color"));
                    EditorGUILayout.PropertyField(pCrossColor, new GUIContent("Cross-Body Color"));
                    EditorGUILayout.PropertyField(pInactive, new GUIContent("Show Broken Beams",
                        "Also draw deactivated (broken) beams."));
                    if (pInactive.boolValue)
                        EditorGUILayout.PropertyField(pInactColor, new GUIContent("Broken Color"));
                    EditorGUI.indentLevel--;
                }

                if (EditorGUI.EndChangeCheck())
                {
                    serializedObject.ApplyModifiedProperties();
                    SceneView.RepaintAll();
                }

                // Live stats
                bool hasSolver = softBody.solver?.nodeManager != null;
                if (!hasSolver)
                {
                    EditorGUILayout.HelpBox(
                        "Visualization renders once the solver initialises in Play mode.",
                        MessageType.None);
                }
                else
                {
                    int total = softBody.solver.beams?.Count ?? 0;
                    int active = 0;
                    if (softBody.solver.beams != null)
                        foreach (var b in softBody.solver.beams)
                            if (b.isActive) active++;

                    EditorGUILayout.HelpBox(
                        $"Nodes: {softBody.solver.nodeManager.NodeCount}   |   " +
                        $"Active beams: {active} / {total}",
                        MessageType.None);
                }

                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();

            // ── Status bar ─────────────────────────────────────────────────────
            if (softBody.solver?.nodeManager != null)
            {
                string psi = softBody.InternalPressure > 0f
                    ? $" | Pressure: {softBody.InternalPressure:F1} PSI" : "";
                EditorGUILayout.HelpBox(
                    $"Nodes: {softBody.solver.nodeManager.NodeCount} | " +
                    $"Beams: {softBody.solver.beams.Count} | " +
                    $"Total Mass: {softBody.TotalMass:F2}kg{psi}",
                    MessageType.None);
            }

            serializedObject.ApplyModifiedProperties();
        }

        private void DrawBanner()
        {
            if (cachedBanner == null)
            {
                cachedBanner = AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Textures/Banner.png");
            }

            if (cachedBanner == null) return;

            Rect rect = GUILayoutUtility.GetRect(
                GUIContent.none, GUIStyle.none,
                GUILayout.ExpandWidth(true),
                GUILayout.Height(80f));

            GUI.DrawTexture(rect, cachedBanner, ScaleMode.ScaleAndCrop, true);

            Color prev = GUI.color;
            GUI.color = new Color(0f, 0f, 0f, 0.25f);
            GUI.Box(rect, GUIContent.none);
            GUI.color = prev;

            EditorGUILayout.Space(4);
        }

        void OnSceneGUI()
        {
            if (softBody?.solver == null) return;
            var nm = softBody.solver.nodeManager;
            if (nm == null || nm.NodeCount == 0) return;

            // ── Beams ──────────────────────────────────────────────────────────
            if (softBody.vizBeams && softBody.solver.beams != null)
            {
                foreach (var beam in softBody.solver.beams)
                {
                    if (!beam.isActive && !softBody.vizInactiveBeams) continue;

                    Handles.color = !beam.isActive ? softBody.vizInactiveColor
                                  : beam.IsCrossBody ? softBody.vizCrossBodyColor
                                  : softBody.vizBeamColor;

                    Handles.DrawLine(nm.GetPosition(beam.nodeA), nm.GetPosition(beam.nodeB));
                }
            }

            // ── Nodes ──────────────────────────────────────────────────────────
            if (softBody.vizNodes)
            {
                bool[] pinned = null;
                if (softBody.vizPinnedNodes && softBody.GetTrussAsset() != null)
                {
                    pinned = new bool[nm.NodeCount];
                    foreach (int idx in softBody.GetTrussAsset().PinnedNodes)
                        if (idx >= 0 && idx < pinned.Length) pinned[idx] = true;
                }

                for (int i = 0; i < nm.NodeCount; i++)
                {
                    bool isPinned = pinned != null && pinned[i];
                    Handles.color = isPinned ? softBody.vizPinnedColor : softBody.vizNodeColor;
                    Handles.SphereHandleCap(0, nm.GetPosition(i), Quaternion.identity,
                        softBody.vizNodeSize, EventType.Repaint);
                }
            }

            // ── Pressure center ────────────────────────────────────────────────
            if (softBody.InternalPressure > 0f)
            {
                Vector3 center = Vector3.zero;
                for (int i = 0; i < nm.NodeCount; i++) center += nm.GetPosition(i);
                center /= nm.NodeCount;

                Handles.color = new Color(1f, 0.5f, 0f, 0.3f);
                Handles.SphereHandleCap(0, center, Quaternion.identity, 0.1f, EventType.Repaint);
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

            if (selectedObj.GetComponent<SoftBody>() != null)
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