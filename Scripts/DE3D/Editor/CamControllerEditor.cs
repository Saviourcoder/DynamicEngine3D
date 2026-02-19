/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using UnityEngine;
using UnityEditor;

namespace DynamicEngine
{
    [CustomEditor(typeof(CameraVisualizerController))]
    public class CameraVisualizerControllerEditor : UnityEditor.Editor
    {
        private SerializedProperty cameraMoveSpeed;
        private SerializedProperty cameraSprintMultiplier;
        private SerializedProperty cameraMouseSensitivity;
        private SerializedProperty enableNodeVisualization;
        private SerializedProperty autoFindSoftBodies;
        private SerializedProperty autoRefreshSoftBodies;
        private SerializedProperty nodeVisualBaseSize;
        private SerializedProperty nodeVisualDistanceScale;
        private SerializedProperty nodeGrabForce;
        private SerializedProperty nodeGrabRadius;
        private SerializedProperty enableGrabbingInPlay;
        private SerializedProperty enableGrabbingInEdit;
        private SerializedProperty nodeColorNormal;
        private SerializedProperty nodeColorHover;
        private SerializedProperty nodeColorGrabbed;
        private SerializedProperty nodeColorPinned;
        private SerializedProperty showGrabRadiusGizmo;
        private SerializedProperty nodeVisualShowRadius;

        void OnEnable()
        {
            cameraMoveSpeed = serializedObject.FindProperty("cameraMoveSpeed");
            cameraSprintMultiplier = serializedObject.FindProperty("cameraSprintMultiplier");
            cameraMouseSensitivity = serializedObject.FindProperty("cameraMouseSensitivity");
            enableNodeVisualization = serializedObject.FindProperty("enableNodeVisualization");
            autoFindSoftBodies = serializedObject.FindProperty("autoFindSoftBodies");
            autoRefreshSoftBodies = serializedObject.FindProperty("autoRefreshSoftBodies");
            nodeVisualBaseSize = serializedObject.FindProperty("nodeVisualBaseSize");
            nodeVisualDistanceScale = serializedObject.FindProperty("nodeVisualDistanceScale");
            nodeGrabForce = serializedObject.FindProperty("nodeGrabForce");
            nodeGrabRadius = serializedObject.FindProperty("nodeGrabRadius");
            enableGrabbingInPlay = serializedObject.FindProperty("enableGrabbingInPlay");
            enableGrabbingInEdit = serializedObject.FindProperty("enableGrabbingInEdit");
            nodeColorNormal = serializedObject.FindProperty("nodeColorNormal");
            nodeColorHover = serializedObject.FindProperty("nodeColorHover");
            nodeColorGrabbed = serializedObject.FindProperty("nodeColorGrabbed");
            nodeColorPinned = serializedObject.FindProperty("nodeColorPinned");
            showGrabRadiusGizmo = serializedObject.FindProperty("showGrabRadiusGizmo");
            nodeVisualShowRadius = serializedObject.FindProperty("nodeVisualShowRadius");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            CameraVisualizerController controller = (CameraVisualizerController)target;

            EditorGUILayout.LabelField("Camera Controller", EditorStyles.boldLabel);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Movement", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(cameraMoveSpeed, new GUIContent("Move Speed"));
            EditorGUILayout.PropertyField(cameraSprintMultiplier, new GUIContent("Sprint Multiplier"));
            EditorGUILayout.PropertyField(cameraMouseSensitivity, new GUIContent("Mouse Sensitivity"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Node Visualization", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(enableNodeVisualization, new GUIContent("Enable Visualization"));

            if (enableNodeVisualization.boolValue)
            {
                EditorGUILayout.PropertyField(autoFindSoftBodies, new GUIContent("Find on Start"));
                EditorGUILayout.PropertyField(autoRefreshSoftBodies, new GUIContent("Refresh Every Frame"));
            }

            EditorGUILayout.Space();
            bool showGrabbingSettings = EditorGUILayout.Foldout(EditorPrefs.GetBool("CameraVisualizerController_ShowGrabbing", false), "Node Grabbing", true);
            if (showGrabbingSettings)
            {
                EditorPrefs.SetBool("CameraVisualizerController_ShowGrabbing", showGrabbingSettings);

                EditorGUILayout.LabelField("Interaction", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(enableGrabbingInPlay, new GUIContent("Enable in Play Mode"));
                EditorGUILayout.PropertyField(enableGrabbingInEdit, new GUIContent("Enable in Edit Mode"));

                EditorGUILayout.LabelField("Physics", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(nodeGrabForce, new GUIContent("Grab Force"));
                EditorGUILayout.PropertyField(nodeGrabRadius, new GUIContent("Grab Radius"));
                EditorGUILayout.PropertyField(nodeVisualShowRadius, new GUIContent("Visible Radius"));

                EditorGUILayout.LabelField("Visuals", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(nodeVisualBaseSize, new GUIContent("Node Size"));
                EditorGUILayout.PropertyField(nodeVisualDistanceScale, new GUIContent("Distance Scale"));
                EditorGUILayout.PropertyField(showGrabRadiusGizmo, new GUIContent("Show Grab Gizmo"));

                EditorGUILayout.LabelField("Colors", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(nodeColorNormal, new GUIContent("Normal"));
                EditorGUILayout.PropertyField(nodeColorHover, new GUIContent("Hover"));
                EditorGUILayout.PropertyField(nodeColorGrabbed, new GUIContent("Grabbed"));
                EditorGUILayout.PropertyField(nodeColorPinned, new GUIContent("Pinned"));
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Status", EditorStyles.boldLabel);
            Camera cam = controller.AttachedCamera;
            if (cam == null)
            {
                EditorGUILayout.HelpBox("Missing Camera!", MessageType.Error);
            }
            else
            {
                EditorGUILayout.LabelField($"Camera: {cam.name}");
                EditorGUILayout.LabelField($"Soft Bodies: {controller.SoftBodyCount}");
            }

            if (Application.isPlaying)
            {
                EditorGUILayout.Space();
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Refresh Soft Bodies"))
                {
                    var method = controller.GetType().GetMethod("RefreshSoftBodies",
                        System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                    method?.Invoke(controller, null);
                }
                if (GUILayout.Button("Log Info"))
                {
                    controller.LogSoftBodyInfo();
                }
                EditorGUILayout.EndHorizontal();
            }

            serializedObject.ApplyModifiedProperties();
        }

        private int GetTotalNodeCount(CameraVisualizerController controller)
        {
            int total = 0;
            foreach (var sb in controller.GetType().GetField("softBodies", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance).GetValue(controller) as SoftBody[])
            {
                if (sb != null && sb.solver?.nodeManager?.Nodes != null)
                {
                    total += sb.solver.nodeManager.Nodes.Count;
                }
            }
            return total;
        }
    }

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