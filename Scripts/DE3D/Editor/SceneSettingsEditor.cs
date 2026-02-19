/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */

using UnityEditor;
using UnityEngine;

namespace DynamicEngine
{
    [CustomEditor(typeof(SceneSettings))]
    public class SceneSettingsEditor : UnityEditor.Editor
    {
        SceneSettings[] m_targets;

        SerializedProperty gravity;
        SerializedProperty TimeScale;
        SerializedProperty constraintIterations;
        SerializedProperty substepPower;
        SerializedProperty substepCount;
        SerializedProperty substepSize;
        SerializedProperty workerThreads;

        protected virtual void OnEnable()
        {
            m_targets = new SceneSettings[targets.Length];
            for (int i = 0; i < targets.Length; ++i) m_targets[i] = (SceneSettings)targets[i];

            gravity = serializedObject.FindProperty("m_gravity");
            TimeScale = serializedObject.FindProperty("m_TimeScale");
            constraintIterations = serializedObject.FindProperty("m_constraintIterations");
            substepPower = serializedObject.FindProperty("m_substepPower");
            substepCount = serializedObject.FindProperty("m_substepCount");
            substepSize = serializedObject.FindProperty("m_substepSize");
            workerThreads = serializedObject.FindProperty("m_workerThreads");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.LabelField("Simulation");

            EditorGUI.indentLevel++;
            EditorGUILayout.Slider(TimeScale, 0.1f, 1f);
            EditorGUILayout.PropertyField(substepPower);
            GUI.enabled = false;
            EditorGUILayout.PropertyField(substepCount);
            EditorGUILayout.PropertyField(substepSize);
            GUI.enabled = true;
            EditorGUILayout.PropertyField(constraintIterations);
            EditorGUI.indentLevel--;

            EditorGUILayout.Separator();

            EditorGUILayout.LabelField("Environment");

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(gravity);
            EditorGUI.indentLevel--;

            EditorGUILayout.Separator();

            EditorGUILayout.LabelField("Optimization");

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(workerThreads);
            EditorGUI.indentLevel--;

            EditorGUILayout.Separator();
            EditorGUILayout.Separator();

            if (GUILayout.Button("Remove Scene Settings"))
            {
                EditorApplication.delayCall += () => Undo.DestroyObjectImmediate(m_targets[0].gameObject);
            }

            if (GUI.changed) serializedObject.ApplyModifiedProperties();
        }
    }
}