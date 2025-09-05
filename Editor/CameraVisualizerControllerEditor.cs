/* DynamicEngine3D - Camera Visualizer Controller Editor
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

namespace DynamicEngine
{
    [CustomEditor(typeof(CameraVisualizerController))]
    public class CameraVisualizerControllerEditor : Editor
    {
        private SerializedProperty moveSpeed;
        private SerializedProperty sprintMultiplier;
        private SerializedProperty mouseSensitivity;
        private SerializedProperty enableSoftBodyVisualization;
        private SerializedProperty findGrabbersOnStart;
        private SerializedProperty refreshGrabbersEveryFrame;

        void OnEnable()
        {
            moveSpeed = serializedObject.FindProperty("moveSpeed");
            sprintMultiplier = serializedObject.FindProperty("sprintMultiplier");
            mouseSensitivity = serializedObject.FindProperty("mouseSensitivity");
            enableSoftBodyVisualization = serializedObject.FindProperty("enableSoftBodyVisualization");
            findGrabbersOnStart = serializedObject.FindProperty("findGrabbersOnStart");
            refreshGrabbersEveryFrame = serializedObject.FindProperty("refreshGrabbersEveryFrame");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            CameraVisualizerController controller = (CameraVisualizerController)target;

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Camera Visualizer Controller", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox("Combined camera controller with free movement and soft body visualization.", MessageType.Info);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Movement Settings", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox("Hold Right Mouse Button + WASD to move. Shift to sprint. Q/E or Space/C for vertical movement.", MessageType.None);
            
            EditorGUILayout.PropertyField(moveSpeed, new GUIContent("Move Speed", "Base movement speed"));
            EditorGUILayout.PropertyField(sprintMultiplier, new GUIContent("Sprint Multiplier", "Speed multiplier when holding Shift"));
            EditorGUILayout.PropertyField(mouseSensitivity, new GUIContent("Mouse Sensitivity", "Mouse look sensitivity"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Visualization Settings", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(enableSoftBodyVisualization, new GUIContent("Enable Visualization", "Enable soft body node visualization"));
            
            if (enableSoftBodyVisualization.boolValue)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(findGrabbersOnStart, new GUIContent("Find Grabbers on Start", "Automatically find all NodeGrabbers in scene on start"));
                EditorGUILayout.PropertyField(refreshGrabbersEveryFrame, new GUIContent("Refresh Every Frame", "Continuously search for new NodeGrabbers (performance impact)"));
                
                if (refreshGrabbersEveryFrame.boolValue)
                {
                    EditorGUILayout.HelpBox("⚠️ Refreshing every frame impacts performance. Only use during development.", MessageType.Warning);
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.Space();
            
            // Status information
            EditorGUILayout.LabelField("Status", EditorStyles.boldLabel);
            
            Camera cam = controller.AttachedCamera;
            if (cam == null)
            {
                EditorGUILayout.HelpBox("Camera component missing!", MessageType.Error);
            }
            else
            {
                EditorGUILayout.LabelField($"Camera: {cam.name}");
                EditorGUILayout.LabelField($"Found Grabbers: {controller.GrabberCount}");
                
                if (Application.isPlaying)
                {
                    if (controller.EnableSoftBodyVisualization)
                    {
                        EditorGUILayout.HelpBox("✓ Visualization active - Hold CTRL near soft bodies to see nodes", MessageType.Info);
                    }
                    else
                    {
                        EditorGUILayout.HelpBox("Visualization disabled", MessageType.Warning);
                    }
                }
                else
                {
                    EditorGUILayout.HelpBox("Enter Play Mode to see visualization", MessageType.Info);
                }
            }

            // Runtime debugging buttons
            if (Application.isPlaying)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Runtime Controls", EditorStyles.boldLabel);
                
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Refresh Grabbers"))
                {
                    // Use reflection to call the private method
                    var method = controller.GetType().GetMethod("RefreshNodeGrabbers", 
                        System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                    method?.Invoke(controller, null);
                }
                
                if (GUILayout.Button("Log Grabber Info"))
                {
                    controller.LogGrabberInfo();
                }
                EditorGUILayout.EndHorizontal();
            }

            serializedObject.ApplyModifiedProperties();
        }
    }

    // Menu item to add the combined component
    public static class CameraVisualizerControllerMenu
    {
        [MenuItem("GameObject/DynamicEngine/Add Camera Visualizer Controller", false, 15)]
        public static void AddCameraVisualizerController()
        {
            GameObject selectedObj = Selection.activeGameObject;
            if (selectedObj == null)
            {
                EditorUtility.DisplayDialog("No Selection", "Please select a GameObject with a Camera component.", "OK");
                return;
            }

            Camera camera = selectedObj.GetComponent<Camera>();
            if (camera == null)
            {
                EditorUtility.DisplayDialog("No Camera", "The selected GameObject must have a Camera component.", "OK");
                return;
            }

            CameraVisualizerController existingController = selectedObj.GetComponent<CameraVisualizerController>();
            if (existingController != null)
            {
                EditorUtility.DisplayDialog("Already Exists", "This GameObject already has a CameraVisualizerController component.", "OK");
                return;
            }

            Undo.AddComponent<CameraVisualizerController>(selectedObj);
            Debug.Log($"Added CameraVisualizerController to {selectedObj.name}");
        }

        [MenuItem("GameObject/DynamicEngine/Add Camera Visualizer Controller", true)]
        public static bool AddCameraVisualizerControllerValidate()
        {
            GameObject selectedObj = Selection.activeGameObject;
            return selectedObj != null && selectedObj.GetComponent<Camera>() != null;
        }
    }
}