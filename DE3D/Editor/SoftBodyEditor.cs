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
        private bool showDeformationSettings = false;
        private bool showPressureSettings = false;

        void OnEnable()
        {
            softBody = (SoftBody)target;
            previousTotalMass = softBody.TotalMass;
            showDeformationSettings = EditorPrefs.GetBool("SoftBodyEditor_ShowDeformation", false);
            showPressureSettings = EditorPrefs.GetBool("SoftBodyEditor_ShowPressure", true);
        }

        void OnDisable()
        {
            EditorPrefs.SetBool("SoftBodyEditor_ShowDeformation", showDeformationSettings);
            EditorPrefs.SetBool("SoftBodyEditor_ShowPressure", showPressureSettings);
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            DrawBanner();

            // Truss field
            EditorGUILayout.PropertyField(serializedObject.FindProperty("truss"));

            // Matter field with info box
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(serializedObject.FindProperty("matter"));
            if (EditorGUI.EndChangeCheck() && Application.isPlaying)
            {
                // Apply matter changes in play mode
                if (softBody.solver != null && softBody.GetMatterAsset() != null)
                {
                    softBody.solver.SetMatterAsset(softBody.GetMatterAsset());
                    Debug.Log("Matter settings applied to solver");
                }
            }

            if (softBody.GetMatterAsset() == null)
            {
                EditorGUILayout.HelpBox("No Matter asset assigned. Using default friction and restitution values.", MessageType.Info);
            }

            EditorGUILayout.Space();

            // Total Mass field
            EditorGUI.BeginChangeCheck();
            float newTotalMass = EditorGUILayout.FloatField(new GUIContent("Total Mass (kg)", "Total mass evenly distributed across all nodes"), softBody.TotalMass);

            if (EditorGUI.EndChangeCheck())
            {
                if (newTotalMass < 0.01f)
                    newTotalMass = 0.01f;

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

            // Pressure Settings
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
                    {
                        softBody.solver.internalPressure = softBody.InternalPressure;
                    }
                }

                // Visual feedback
                if (softBody.InternalPressure > 0f)
                {
                    EditorGUILayout.HelpBox(
                        $"Pressure: {softBody.InternalPressure:F2} PSI - Forces push nodes outward from volume center.\n" +
                        "Higher values create more rigid, inflated structures.",
                        MessageType.Info);
                }
                else
                {
                    EditorGUILayout.HelpBox("Pressure disabled. Set > 0 to inflate the soft body.", MessageType.None);
                }

                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();

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
                softBody.influenceRadius = EditorGUILayout.FloatField("Influence Radius", softBody.influenceRadius);

                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();

            // Status info
            if (softBody.solver != null && softBody.solver.nodeManager != null)
            {
                string pressureStatus = softBody.InternalPressure > 0f ? $" | Pressure: {softBody.InternalPressure:F1} PSI" : "";
                EditorGUILayout.HelpBox(
                    $"Nodes: {softBody.solver.nodeManager.NodeCount} | " +
                    $"Beams: {softBody.solver.beams.Count} | " +
                    $"Total Mass: {softBody.TotalMass:F2}kg{pressureStatus}",
                    MessageType.None);
            }
            serializedObject.ApplyModifiedProperties();
        }

        private void DrawBanner()
        {
            SerializedProperty bannerProp = serializedObject.FindProperty("inspectorBanner");
            if (bannerProp == null) return;

            Texture2D banner = bannerProp.objectReferenceValue as Texture2D;
            if (banner == null) return;

            Rect rect = GUILayoutUtility.GetRect(
                GUIContent.none, GUIStyle.none,
                GUILayout.ExpandWidth(true),
                GUILayout.Height(80f));

            GUI.DrawTexture(rect, banner, ScaleMode.ScaleAndCrop, true);

            Color prev = GUI.color;
            GUI.color = new Color(0f, 0f, 0f, 0.25f);
            GUI.Box(rect, GUIContent.none);
            GUI.color = prev;

            EditorGUILayout.Space(4);
        }

        void OnSceneGUI()
        {
            // Optional: Draw pressure visualization in scene view
            if (softBody.solver != null &&
                softBody.InternalPressure > 0f &&
                softBody.solver.nodeManager != null &&
                softBody.solver.nodeManager.NodeCount > 0)
            {
                // Calculate volume center directly (no longer depends on private Solver method)
                Vector3 center = Vector3.zero;
                int count = softBody.solver.nodeManager.NodeCount;
                for (int i = 0; i < count; i++)
                {
                    center += softBody.solver.nodeManager.GetPosition(i);
                }
                center /= count;

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