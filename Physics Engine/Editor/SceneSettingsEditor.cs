/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */

using UnityEditor;
using UnityEngine;

namespace DynamicEngine
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(SceneSettings))]
    public class SceneSettingsEditor : UnityEditor.Editor
    {
        SceneSettings[] m_targets;

        SerializedProperty gravity;
        SerializedProperty simulationTimeScale;
        SerializedProperty constraintIterations;
        SerializedProperty baseSubSteps;
        SerializedProperty minSubSteps;
        SerializedProperty workerThreads;

        protected virtual void OnEnable()
        {
            m_targets = new SceneSettings[targets.Length];
            for (int i = 0; i < targets.Length; ++i) m_targets[i] = (SceneSettings)targets[i];

            gravity = serializedObject.FindProperty("m_gravity");
            simulationTimeScale = serializedObject.FindProperty("m_simulationTimeScale");
            constraintIterations = serializedObject.FindProperty("m_constraintIterations");
            baseSubSteps = serializedObject.FindProperty("m_baseSubSteps");
            minSubSteps = serializedObject.FindProperty("m_minSubSteps");
            workerThreads = serializedObject.FindProperty("m_workerThreads");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.LabelField("Simulation");

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(constraintIterations);
            EditorGUILayout.PropertyField(simulationTimeScale);
            EditorGUILayout.PropertyField(baseSubSteps);
            EditorGUILayout.PropertyField(minSubSteps);
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