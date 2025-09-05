/* DynamicEngine3D - Node Grabber Editor
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
    [CustomEditor(typeof(NodeGrabber))]
    public class NodeGrabberEditor : Editor
    {
        private SerializedProperty nodeVisualSize;
        private SerializedProperty grabForce;
        private SerializedProperty grabRadius;
        private SerializedProperty enableInPlayMode;
        private SerializedProperty enableInEditMode;
        private SerializedProperty normalNodeColor;
        private SerializedProperty hoverNodeColor;
        private SerializedProperty grabbedNodeColor;
        private SerializedProperty showGrabRadius;
        private SerializedProperty nodeMaterial;
        private SerializedProperty beamMaterial;

        void OnEnable()
        {
            nodeVisualSize = serializedObject.FindProperty("nodeVisualSize");
            grabForce = serializedObject.FindProperty("grabForce");
            grabRadius = serializedObject.FindProperty("grabRadius");
            enableInPlayMode = serializedObject.FindProperty("enableInPlayMode");
            enableInEditMode = serializedObject.FindProperty("enableInEditMode");
            normalNodeColor = serializedObject.FindProperty("normalNodeColor");
            hoverNodeColor = serializedObject.FindProperty("hoverNodeColor");
            grabbedNodeColor = serializedObject.FindProperty("grabbedNodeColor");
            showGrabRadius = serializedObject.FindProperty("showGrabRadius");
            nodeMaterial = serializedObject.FindProperty("nodeMaterial");
            beamMaterial = serializedObject.FindProperty("beamMaterial");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            NodeGrabber grabber = (NodeGrabber)target;

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Node Grabbing System", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox("Hold CTRL/CMD and drag nodes to interact with the soft body in real-time!", MessageType.Info);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Interaction Settings", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(enableInPlayMode, new GUIContent("Enable in Play Mode", "Allow node grabbing during runtime"));
            EditorGUILayout.PropertyField(enableInEditMode, new GUIContent("Enable in Edit Mode", "Allow node grabbing in scene view"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Physics Settings", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(grabForce, new GUIContent("Grab Force", "Force applied when dragging nodes"));
            EditorGUILayout.PropertyField(grabRadius, new GUIContent("Grab Radius", "Detection radius for mouse interaction"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Visual Settings", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(nodeVisualSize, new GUIContent("Node Visual Size", "Size of node visualization"));
            EditorGUILayout.PropertyField(showGrabRadius, new GUIContent("Show Grab Radius", "Display grab radius around hovered nodes"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Node Colors", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(normalNodeColor, new GUIContent("Normal Color"));
            EditorGUILayout.PropertyField(hoverNodeColor, new GUIContent("Hover Color"));
            EditorGUILayout.PropertyField(grabbedNodeColor, new GUIContent("Grabbed Color"));

            EditorGUILayout.Space();

            // Status information
            EditorGUILayout.LabelField("Status", EditorStyles.boldLabel);

            SoftBody softBody = grabber.GetComponent<SoftBody>();
            
            if (softBody == null)
            {
                EditorGUILayout.HelpBox("NodeGrabber requires a SoftBody component!", MessageType.Error);
            }
            else if (softBody.solver?.nodeManager?.Nodes == null)
            {
                EditorGUILayout.HelpBox("SoftBody solver not initialized. Try playing the scene or reinitializing the SoftBody.", MessageType.Warning);
            }
            else
            {
                int nodeCount = softBody.solver.nodeManager.Nodes.Count;
                EditorGUILayout.LabelField($"Nodes Available: {nodeCount}");

                if (Application.isPlaying)
                {
                    EditorGUILayout.HelpBox("✓ Ready! Hold CTRL and click-drag nodes in the Game/Scene view.", MessageType.Info);
                }
                else
                {
                    EditorGUILayout.HelpBox("✓ Ready! Hold CTRL and click-drag nodes in the Scene view (Edit Mode).", MessageType.Info);
                }
                
                // Rendering information
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Rendering", EditorStyles.boldLabel);
                EditorGUILayout.HelpBox("✓ Unified URP-compatible rendering system - Works in both Game and Scene view", MessageType.Info);
                
                // Add some helpful info about the unified system
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("System Info", EditorStyles.boldLabel);
                EditorGUILayout.LabelField($"Rendering Mode: URP-Compatible (Unified)");
                EditorGUILayout.LabelField($"Materials: Auto-generated with fallbacks");
                EditorGUILayout.LabelField($"Performance: Optimized single-component system");
            }

            serializedObject.ApplyModifiedProperties();
        }

        void OnSceneGUI()
        {
            NodeGrabber grabber = (NodeGrabber)target;
            if (grabber == null || !grabber.enabled || !grabber.enableInEditMode)
                return;

            // Handle scene view events for edit mode interaction
            Event e = Event.current;
            if (e != null)
            {
                // Allow the NodeGrabber to process scene view events
                if (e.type == EventType.MouseDown || e.type == EventType.MouseDrag || e.type == EventType.MouseUp)
                {
                    // Repaint scene view to update node visualizations
                    SceneView.RepaintAll();
                }
            }

            // Draw visualization in edit mode using Handles
            if (grabber.isControlPressed && grabber.softBody?.solver?.nodeManager?.Nodes != null)
            {
                Camera sceneCam = SceneView.lastActiveSceneView.camera;

                // Draw all nodes as filled discs
                for (int i = 0; i < grabber.softBody.solver.nodeManager.Nodes.Count; i++)
                {
                    if (grabber.softBody.solver.nodeManager.Nodes[i] == null)
                        continue;
                    
                    Vector3 nodePos = grabber.softBody.solver.nodeManager.Nodes[i].position;
                    Color nodeColor = grabber.normalNodeColor;
                    
                    if (i == grabber.draggedNodeIndex)
                        nodeColor = grabber.grabbedNodeColor;
                    else if (i == grabber.hoveredNodeIndex)
                        nodeColor = grabber.hoverNodeColor;

                    // Calculate normal facing the camera
                    Vector3 normal = (nodePos - sceneCam.transform.position).normalized;

                    // Draw filled disc
                    Handles.color = nodeColor;
                    Handles.DrawSolidDisc(nodePos, normal, grabber.nodeVisualSize);

                    // Draw grab radius
                    if (grabber.showGrabRadius && (i == grabber.hoveredNodeIndex || i == grabber.draggedNodeIndex))
                    {
                        Handles.color = Color.white * 0.3f;
                        Handles.DrawWireDisc(nodePos, normal, grabber.grabRadius);
                    }
                }

                // Draw links as black lines
                Handles.color = Color.black;
                foreach (var beam in grabber.softBody.solver.beams)
                {
                    if (beam.nodeA < 0 || beam.nodeA >= grabber.softBody.solver.nodeManager.Nodes.Count ||
                        beam.nodeB < 0 || beam.nodeB >= grabber.softBody.solver.nodeManager.Nodes.Count ||
                        grabber.softBody.solver.nodeManager.Nodes[beam.nodeA] == null ||
                        grabber.softBody.solver.nodeManager.Nodes[beam.nodeB] == null)
                        continue;

                    Vector3 pos1 = grabber.softBody.solver.nodeManager.Nodes[beam.nodeA].position;
                    Vector3 pos2 = grabber.softBody.solver.nodeManager.Nodes[beam.nodeB].position;
                    Handles.DrawLine(pos1, pos2);
                }

                // Draw drag line and target position
                if (grabber.draggedNodeIndex >= 0)
                {
                    Vector3 nodePos = grabber.softBody.solver.nodeManager.Nodes[grabber.draggedNodeIndex].position;
                    Handles.color = grabber.grabbedNodeColor;
                    Handles.DrawLine(nodePos, grabber.targetPosition);

                    // Draw target position as smaller filled disc
                    Vector3 targetNormal = (grabber.targetPosition - sceneCam.transform.position).normalized;
                    Handles.color = Color.white;
                    Handles.DrawSolidDisc(grabber.targetPosition, targetNormal, grabber.nodeVisualSize * 0.5f);
                }
            }
        }
    }
    public static class NodeGrabberMenu
    {
        [MenuItem("GameObject/DynamicEngine/Add Node Grabber", false, 20)]
        public static void AddNodeGrabber()
        {
            GameObject selectedObj = Selection.activeGameObject;
            if (selectedObj == null)
            {
                EditorUtility.DisplayDialog("No Selection", "Please select a GameObject with a SoftBody component.", "OK");
                return;
            }

            SoftBody softBody = selectedObj.GetComponent<SoftBody>();
            if (softBody == null)
            {
                EditorUtility.DisplayDialog("No SoftBody", "The selected GameObject must have a SoftBody component.", "OK");
                return;
            }

            NodeGrabber existingGrabber = selectedObj.GetComponent<NodeGrabber>();
            if (existingGrabber != null)
            {
                EditorUtility.DisplayDialog("Already Exists", "This GameObject already has a NodeGrabber component.", "OK");
                return;
            }

            Undo.AddComponent<NodeGrabber>(selectedObj);
            Debug.Log($"Added NodeGrabber to {selectedObj.name}");
        }

        [MenuItem("GameObject/DynamicEngine/Add Node Grabber", true)]
        public static bool AddNodeGrabberValidate()
        {
            GameObject selectedObj = Selection.activeGameObject;
            return selectedObj != null && selectedObj.GetComponent<SoftBody>() != null && selectedObj.GetComponent<NodeGrabber>() == null;
        }

    }
}
