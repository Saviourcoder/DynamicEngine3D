/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers */
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
            serializedObject.Update();

            Truss truss = (Truss)target;

            SerializedProperty nodePositionsProp = serializedObject.FindProperty("nodePositions");
            SerializedProperty beamsProp = serializedObject.FindProperty("beams");
            SerializedProperty pinnedNodesProp = serializedObject.FindProperty("pinnedNodes");
            SerializedProperty nodeMassesProp = serializedObject.FindProperty("nodeMasses");
            SerializedProperty maxStretchFactorProp = serializedObject.FindProperty("maxStretchFactor");
            SerializedProperty minStretchFactorProp = serializedObject.FindProperty("minStretchFactor");
            SerializedProperty facesProp = serializedObject.FindProperty("faces");
            SerializedProperty nodeSetsProp = serializedObject.FindProperty("nodeSets");
            SerializedProperty uniformMassProp = serializedObject.FindProperty("uniformMassEditorValue");

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Node Positions", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(nodePositionsProp, new GUIContent("Nodes"), true);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Node Masses", EditorStyles.boldLabel);

            // sync nodeMasses with nodePositions
            if (nodePositionsProp != null && nodeMassesProp != null)
            {
                int targetSize = nodePositionsProp.arraySize;
                if (nodeMassesProp.arraySize != targetSize)
                {
                    int oldSize = nodeMassesProp.arraySize;
                    nodeMassesProp.arraySize = targetSize;
                    for (int i = oldSize; i < nodeMassesProp.arraySize; i++)
                    {
                        nodeMassesProp.GetArrayElementAtIndex(i).floatValue = 0.5f;
                    }
                }

                if (nodeMassesProp.arraySize > 0)
                {
                    EditorGUI.indentLevel++;
                    bool showIndividual = EditorPrefs.GetBool("TrussEditor_ShowNodeMasses", false);
                    bool newShow = EditorGUILayout.Foldout(showIndividual, $"Individual Node Masses ({nodeMassesProp.arraySize} nodes)");
                    if (newShow != showIndividual) EditorPrefs.SetBool("TrussEditor_ShowNodeMasses", newShow);

                    if (newShow)
                    {
                        for (int i = 0; i < nodeMassesProp.arraySize; i++)
                        {
                            SerializedProperty massProp = nodeMassesProp.GetArrayElementAtIndex(i);
                            EditorGUI.BeginChangeCheck();
                            EditorGUILayout.PropertyField(massProp, new GUIContent($"Node {i} Mass"));
                            if (EditorGUI.EndChangeCheck())
                            {
                                massProp.floatValue = Mathf.Max(0.1f, massProp.floatValue);
                            }
                        }
                    }
                    EditorGUI.indentLevel--;

                    // --- improved "Set All Masses To" with persistent backing value ---
                    EditorGUILayout.BeginHorizontal();

                    float firstMass = nodeMassesProp.GetArrayElementAtIndex(0).floatValue;
                    bool allEqual = true;
                    for (int i = 1; i < nodeMassesProp.arraySize; i++)
                    {
                        if (!Mathf.Approximately(nodeMassesProp.GetArrayElementAtIndex(i).floatValue, firstMass))
                        {
                            allEqual = false;
                            break;
                        }
                    }

                    EditorGUI.showMixedValue = !allEqual;
                    EditorGUILayout.PropertyField(uniformMassProp, new GUIContent("Set All Masses To:"));
                    EditorGUI.showMixedValue = false;

                    if (GUILayout.Button("Apply", GUILayout.Width(60)))
                    {
                        float newMass = Mathf.Max(0.1f, uniformMassProp.floatValue);
                        for (int i = 0; i < nodeMassesProp.arraySize; i++)
                        {
                            nodeMassesProp.GetArrayElementAtIndex(i).floatValue = newMass;
                        }
                    }

                    EditorGUILayout.EndHorizontal();
                }
            }

            // --------- rest of inspector (beams, pins, faces, sets, etc) ---------

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
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("maxDeformation"), new GUIContent("Max Deformation"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("plasticityThreshold"), new GUIContent("Plasticity Threshold"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("plasticityRate"), new GUIContent("Plasticity Rate"));
                EditorGUILayout.PropertyField(beamProp.FindPropertyRelative("isActive"), new GUIContent("Is Active"));
                if (GUILayout.Button("Remove Beam"))
                {
                    beamsProp.DeleteArrayElementAtIndex(i);
                    serializedObject.ApplyModifiedProperties();
                    EditorUtility.SetDirty(truss);
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
                newBeam.FindPropertyRelative("maxDeformation").floatValue = 0.5f;
                newBeam.FindPropertyRelative("plasticityThreshold").floatValue = 0.05f;
                newBeam.FindPropertyRelative("plasticityRate").floatValue = 0.1f;
                newBeam.FindPropertyRelative("isActive").boolValue = true;
                serializedObject.ApplyModifiedProperties();
                EditorUtility.SetDirty(truss);
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Pinned Nodes", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(pinnedNodesProp, new GUIContent("Pinned Nodes"), true);

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
                    EditorUtility.SetDirty(truss);
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
                EditorUtility.SetDirty(truss);
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
                    EditorUtility.SetDirty(truss);
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
                EditorUtility.SetDirty(truss);
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
            EditorUtility.SetDirty(truss);
        }
    }
#endif
}
