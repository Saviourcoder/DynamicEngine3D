using UnityEngine;
using System.Collections.Generic;
using System.Linq;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{

#if UNITY_EDITOR
    [CustomEditor(typeof(Truss))]
    public class TrussAssetEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            Truss truss = (Truss)target;
            SerializedProperty nodePositionsProp = serializedObject.FindProperty("nodePositions");
            SerializedProperty beamsProp = serializedObject.FindProperty("beams");
            SerializedProperty pinnedNodesProp = serializedObject.FindProperty("pinnedNodes");
            SerializedProperty nodeMassProp = serializedObject.FindProperty("nodeMass");
            SerializedProperty maxStretchFactorProp = serializedObject.FindProperty("maxStretchFactor");
            SerializedProperty minStretchFactorProp = serializedObject.FindProperty("minStretchFactor");
            SerializedProperty facesProp = serializedObject.FindProperty("faces");
            SerializedProperty nodeSetsProp = serializedObject.FindProperty("nodeSets");

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Node Positions", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(nodePositionsProp, new GUIContent("Nodes"), true);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Beams", EditorStyles.boldLabel);
            for (int i = 0; i < beamsProp.arraySize; i++)
            {
                EditorGUILayout.BeginVertical(GUI.skin.box);
                SerializedProperty beamProp = beamsProp.GetArrayElementAtIndex(i);
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("nodeA"), new GUIContent("Node A"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("nodeB"), new GUIContent("Node B"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("compliance"), new GUIContent("Compliance"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("damping"), new GUIContent("Damping"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("restLength"), new GUIContent("Rest Length"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("deformationScale"), new GUIContent("Deformation Scale"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("maxDeformation"), new GUIContent("Max Deformation"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("plasticityThreshold"), new GUIContent("Plasticity Threshold"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("plasticityRate"), new GUIContent("Plasticity Rate"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("isActive"), new GUIContent("Is Active"));
                if (GUILayout.Button("Remove Beam"))
                {
                    beamsProp.DeleteArrayElementAtIndex(i);
                    serializedObject.ApplyModifiedProperties();
                    return;
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();
            if (GUILayout.Button("Add New Beam", GUILayout.Height(30)))
            {
                beamsProp.arraySize++;
                SerializedProperty newBeam = beamsProp.GetArrayElementAtIndex(beamsProp.arraySize - 1);
                newBeam.FindPropertyRelative("nodeA").intValue = 0;
                newBeam.FindPropertyRelative("nodeB").intValue = 1;
                newBeam.FindPropertyRelative("compliance").floatValue = 1e-3f;
                newBeam.FindPropertyRelative("damping").floatValue = 0.3f;
                newBeam.FindPropertyRelative("restLength").floatValue = 0f;
                newBeam.FindPropertyRelative("deformationScale").floatValue = 0.1f;
                newBeam.FindPropertyRelative("maxDeformation").floatValue = 0.5f;
                newBeam.FindPropertyRelative("plasticityThreshold").floatValue = 0.05f;
                newBeam.FindPropertyRelative("plasticityRate").floatValue = 0.1f;
                newBeam.FindPropertyRelative("isActive").boolValue = true;
                serializedObject.ApplyModifiedProperties();
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Pinned Nodes", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(pinnedNodesProp, new GUIContent("Pinned Nodes"), true);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Node Mass", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(nodeMassProp, new GUIContent("Node Mass"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Stretch Factors", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(maxStretchFactorProp, new GUIContent("Max Stretch Factor"));
            EditorGUILayout.PropertyField(minStretchFactorProp, new GUIContent("Min Stretch Factor"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Faces", EditorStyles.boldLabel);
            for (int i = 0; i < facesProp.arraySize; i++)
            {
                EditorGUILayout.BeginVertical(GUI.skin.box);
                SerializedProperty faceProp = facesProp.GetArrayElementAtIndex(i);
                EditorGUILayout.PropertyField(faceProp.FindPropertyRelative("nodeA"), new GUIContent("Node A"));
                EditorGUILayout.PropertyField(faceProp.FindPropertyRelative("nodeB"), new GUIContent("Node B"));
                EditorGUILayout.PropertyField(faceProp.FindPropertyRelative("nodeC"), new GUIContent("Node C"));
                if (GUILayout.Button("Remove Face"))
                {
                    facesProp.DeleteArrayElementAtIndex(i);
                    serializedObject.ApplyModifiedProperties();
                    return;
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();
            if (GUILayout.Button("Add New Face", GUILayout.Height(30)))
            {
                facesProp.arraySize++;
                SerializedProperty newFace = facesProp.GetArrayElementAtIndex(facesProp.arraySize - 1);
                newFace.FindPropertyRelative("nodeA").intValue = 0;
                newFace.FindPropertyRelative("nodeB").intValue = 1;
                newFace.FindPropertyRelative("nodeC").intValue = 2;
                serializedObject.ApplyModifiedProperties();
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Node Sets", EditorStyles.boldLabel);
            for (int i = 0; i < nodeSetsProp.arraySize; i++)
            {
                EditorGUILayout.BeginVertical(GUI.skin.box);
                SerializedProperty nodeSetProp = nodeSetsProp.GetArrayElementAtIndex(i);
                EditorGUILayout.PropertyField(nodeSetProp.FindPropertyRelative("name"), new GUIContent("Name"));
                EditorGUILayout.PropertyField(nodeSetProp.FindPropertyRelative("nodeIndices"), new GUIContent("Node Indices"), true);
                EditorGUILayout.PropertyField(nodeSetProp.FindPropertyRelative("color"), new GUIContent("Color"));
                EditorGUILayout.PropertyField(nodeSetProp.FindPropertyRelative("isVisible"), new GUIContent("Is Visible"));
                if (GUILayout.Button("Remove Node Set"))
                {
                    nodeSetsProp.DeleteArrayElementAtIndex(i);
                    serializedObject.ApplyModifiedProperties();
                    return;
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space();
            if (GUILayout.Button("Add New Node Set", GUILayout.Height(30)))
            {
                nodeSetsProp.arraySize++;
                SerializedProperty newNodeSet = nodeSetsProp.GetArrayElementAtIndex(nodeSetsProp.arraySize - 1);
                newNodeSet.FindPropertyRelative("name").stringValue = $"NodeSet{nodeSetsProp.arraySize}";
                newNodeSet.FindPropertyRelative("nodeIndices").arraySize = 0;
                newNodeSet.FindPropertyRelative("color").colorValue = Color.cyan;
                newNodeSet.FindPropertyRelative("isVisible").boolValue = true;
                serializedObject.ApplyModifiedProperties();
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Assign to SoftBody", EditorStyles.boldLabel);
            SoftBody targetSoftBody = (SoftBody)EditorGUILayout.ObjectField("SoftBody", null, typeof(SoftBody), true);
            if (targetSoftBody != null && GUILayout.Button("Apply to SoftBody", GUILayout.Height(30)))
            {
                targetSoftBody.ApplyTrussAsset(truss);
                EditorUtility.SetDirty(targetSoftBody);
                Debug.Log("Applied Truss to SoftBody: " + targetSoftBody.name, targetSoftBody);
            }

            if (GUILayout.Button("Validate Node Sets"))
            {
                truss.ValidateNodeSets();
                Debug.Log("Node sets validated for Truss: " + truss.name);
            }

            serializedObject.ApplyModifiedProperties();
        }
    }
#endif
}
