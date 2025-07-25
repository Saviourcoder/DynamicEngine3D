/* DynamicEngine3D - SoftBody Custom Inspector
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/

using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

namespace DynamicEngine
{
    [CustomEditor(typeof(SoftBody))]
    public class SoftBodyEditor : Editor
    {
        private SoftBody softBody;
        private float previousTotalMass;
        private bool showVisualizationSettings = false;

        void OnEnable()
        {
            softBody = (SoftBody)target;
            previousTotalMass = softBody.TotalMass;
            showVisualizationSettings = EditorPrefs.GetBool("SoftBodyEditor_ShowVisualization", false);
        }

        void OnDisable()
        {
            // Save preferences
            EditorPrefs.SetBool("SoftBodyEditor_ShowVisualization", showVisualizationSettings);
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            // Draw truss field
            EditorGUILayout.PropertyField(serializedObject.FindProperty("truss"));

            // Custom Total Mass field with automatic distribution feedback
            EditorGUI.BeginChangeCheck();
            float newTotalMass = EditorGUILayout.FloatField("Total Mass", softBody.TotalMass);

            if (EditorGUI.EndChangeCheck())
            {
                // Ensure mass is not negative or zero
                if (newTotalMass < 0.01f)
                    newTotalMass = 0.01f;

                // Record undo operation
                Undo.RecordObject(softBody, "Change Total Mass");

                // Set the new mass (this will automatically trigger distribution)
                softBody.TotalMass = newTotalMass;
                previousTotalMass = newTotalMass;

                // Mark dirty for saving
                EditorUtility.SetDirty(softBody);

                // Show feedback
                if (softBody.solver?.nodeManager?.Nodes != null && softBody.solver.nodeManager.Nodes.Count > 0)
                {
                    float massPerNode = newTotalMass / softBody.solver.nodeManager.Nodes.Count;
                    Debug.Log($"SoftBody: Mass automatically distributed - {newTotalMass:F2}kg total, {massPerNode:F3}kg per node across {softBody.solver.nodeManager.Nodes.Count} nodes. NodeLinkEditor updated.");

                    // Verify the distribution actually happened
                    var nodeLinkEditor = softBody.GetComponent<NodeLinkEditor>();
                    if (nodeLinkEditor != null)
                    {
                        Debug.Log($"SoftBody: NodeLinkEditor.nodeMass updated to {nodeLinkEditor.nodeMass:F3}kg (should match {massPerNode:F3}kg)");
                    }
                }
            }

            // Draw other properties manually (using correct field names from SoftBody.cs)
            var nodeRadiusProp = serializedObject.FindProperty("nodeRadius");
            if (nodeRadiusProp != null)
                EditorGUILayout.PropertyField(nodeRadiusProp);

            var influenceRadiusProp = serializedObject.FindProperty("influenceRadius");
            if (influenceRadiusProp != null)
                EditorGUILayout.PropertyField(influenceRadiusProp);

            var beamConnectionDistanceProp = serializedObject.FindProperty("beamConnectionDistance");
            if (beamConnectionDistanceProp != null)
                EditorGUILayout.PropertyField(beamConnectionDistanceProp);

            EditorGUILayout.Space();

            // Draw visualization header with foldout
            showVisualizationSettings = EditorGUILayout.Foldout(showVisualizationSettings, "Runtime Visualization", true, EditorStyles.foldoutHeader);
            EditorPrefs.SetBool("SoftBodyEditor_ShowVisualization", showVisualizationSettings);

            if (showVisualizationSettings)
            {
                EditorGUI.indentLevel++;

                // Draw visualization properties (using correct field names from SoftBody.cs)
                var showGizmosProp = serializedObject.FindProperty("showGizmos");
                if (showGizmosProp != null)
                    EditorGUILayout.PropertyField(showGizmosProp);

                var showNodesAndLinksProp = serializedObject.FindProperty("showNodesAndLinks");
                if (showNodesAndLinksProp != null)
                    EditorGUILayout.PropertyField(showNodesAndLinksProp);

                var showNodesProp = serializedObject.FindProperty("showNodes");
                if (showNodesProp != null)
                    EditorGUILayout.PropertyField(showNodesProp);

                var showLinksProp = serializedObject.FindProperty("showLinks");
                if (showLinksProp != null)
                    EditorGUILayout.PropertyField(showLinksProp);

                var showNodeIndicesProp = serializedObject.FindProperty("showNodeIndices");
                if (showNodeIndicesProp != null)
                    EditorGUILayout.PropertyField(showNodeIndicesProp);

                var showLinkForcesProp = serializedObject.FindProperty("showLinkForces");
                if (showLinkForcesProp != null)
                    EditorGUILayout.PropertyField(showLinkForcesProp);

                var showInfluenceRadiusProp = serializedObject.FindProperty("showInfluenceRadius");
                if (showInfluenceRadiusProp != null)
                    EditorGUILayout.PropertyField(showInfluenceRadiusProp);

                var nodeDisplaySizeProp = serializedObject.FindProperty("nodeDisplaySize");
                if (nodeDisplaySizeProp != null)
                    EditorGUILayout.PropertyField(nodeDisplaySizeProp);

                var nodeColorProp = serializedObject.FindProperty("nodeColor");
                if (nodeColorProp != null)
                    EditorGUILayout.PropertyField(nodeColorProp);

                var pinnedNodeColorProp = serializedObject.FindProperty("pinnedNodeColor");
                if (pinnedNodeColorProp != null)
                    EditorGUILayout.PropertyField(pinnedNodeColorProp);

                var linkColorProp = serializedObject.FindProperty("linkColor");
                if (linkColorProp != null)
                    EditorGUILayout.PropertyField(linkColorProp);

                var stretchedLinkColorProp = serializedObject.FindProperty("stretchedLinkColor");
                if (stretchedLinkColorProp != null)
                    EditorGUILayout.PropertyField(stretchedLinkColorProp);

                var compressedLinkColorProp = serializedObject.FindProperty("compressedLinkColor");
                if (compressedLinkColorProp != null)
                    EditorGUILayout.PropertyField(compressedLinkColorProp);

                var influenceRadiusColorProp = serializedObject.FindProperty("influenceRadiusColor");
                if (influenceRadiusColorProp != null)
                    EditorGUILayout.PropertyField(influenceRadiusColorProp);

                var forceVisualizationScaleProp = serializedObject.FindProperty("forceVisualizationScale");
                if (forceVisualizationScaleProp != null)
                    EditorGUILayout.PropertyField(forceVisualizationScaleProp);

                EditorGUI.indentLevel--;
            }

            serializedObject.ApplyModifiedProperties();
        }
    }
}