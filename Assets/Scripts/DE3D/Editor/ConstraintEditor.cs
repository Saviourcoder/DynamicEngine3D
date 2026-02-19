/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using UnityEditor;
using UnityEngine;
using System.Linq;
using System.Collections.Generic;
namespace DynamicEngine.Editor
{
    [CustomEditor(typeof(Constraint))]
    [CanEditMultipleObjects]
    public class ConstraintEditor : UnityEditor.Editor
    {
        private Constraint[] m_targets;
        private SerializedProperty softBodyBProp;
        private SerializedProperty disableCollisionProp;
        private SerializedProperty showLinksProp;
        private SerializedProperty linkConfigsProp;
        private SerializedProperty enableMotorProp;
        private SerializedProperty axisNodeSetProp;
        private SerializedProperty targetRateProp;
        private SerializedProperty maxTorqueProp;

        // Track previous transform state to detect scene changes
        private Vector3 lastBodyAPosition;
        private Quaternion lastBodyARotation;
        private Vector3 lastBodyAScale;
        private Vector3 lastBodyBPosition;
        private Quaternion lastBodyBRotation;
        private Vector3 lastBodyBScale;
        private bool transformsInitialized = false;

        protected virtual void OnEnable()
        {
            m_targets = new Constraint[targets.Length];
            for (int i = 0; i < targets.Length; ++i) m_targets[i] = (Constraint)targets[i];
            softBodyBProp = serializedObject.FindProperty("m_baseBody");
            disableCollisionProp = serializedObject.FindProperty("m_disableCollision");
            showLinksProp = serializedObject.FindProperty("m_showLinks");
            linkConfigsProp = serializedObject.FindProperty("m_links");
            enableMotorProp = serializedObject.FindProperty("m_enableMotor");
            axisNodeSetProp = serializedObject.FindProperty("m_axisNodeSet");
            targetRateProp = serializedObject.FindProperty("m_targetRate");
            maxTorqueProp = serializedObject.FindProperty("m_maxTorque");

            // Subscribe to scene view updates
            SceneView.duringSceneGui += OnSceneUpdate;
            transformsInitialized = false;
        }

        private void OnDisable()
        {
            // Unsubscribe from scene view updates
            SceneView.duringSceneGui -= OnSceneUpdate;
        }

        private void OnSceneUpdate(SceneView sceneView)
        {
            // Only check for single target
            if (m_targets.Length != 1) return;

            Constraint constraint = m_targets[0];
            if (constraint == null) return;

            SoftBody bodyA = constraint.GetComponent<SoftBody>();
            SoftBody bodyB = constraint.baseBody;

            if (bodyA == null) return;

            // Initialize transform tracking
            if (!transformsInitialized)
            {
                lastBodyAPosition = bodyA.transform.position;
                lastBodyARotation = bodyA.transform.rotation;
                lastBodyAScale = bodyA.transform.localScale;

                if (bodyB != null)
                {
                    lastBodyBPosition = bodyB.transform.position;
                    lastBodyBRotation = bodyB.transform.rotation;
                    lastBodyBScale = bodyB.transform.localScale;
                }

                transformsInitialized = true;
                return;
            }

            // Check if transforms have changed
            bool transformChanged = false;

            if (bodyA.transform.position != lastBodyAPosition ||
                bodyA.transform.rotation != lastBodyARotation ||
                bodyA.transform.localScale != lastBodyAScale)
            {
                lastBodyAPosition = bodyA.transform.position;
                lastBodyARotation = bodyA.transform.rotation;
                lastBodyAScale = bodyA.transform.localScale;
                transformChanged = true;
            }

            if (bodyB != null)
            {
                if (bodyB.transform.position != lastBodyBPosition ||
                    bodyB.transform.rotation != lastBodyBRotation ||
                    bodyB.transform.localScale != lastBodyBScale)
                {
                    lastBodyBPosition = bodyB.transform.position;
                    lastBodyBRotation = bodyB.transform.rotation;
                    lastBodyBScale = bodyB.transform.localScale;
                    transformChanged = true;
                }
            }

            // If transforms changed, update rest lengths
            if (transformChanged)
            {
                UpdateRestLengths();
                Repaint(); // Force inspector repaint
            }
        }

        private void UpdateRestLengths()
        {
            if (m_targets.Length != 1) return;

            Constraint constraint = m_targets[0];
            SoftBody bodyA = constraint.GetComponent<SoftBody>();
            SoftBody bodyB = constraint.baseBody;

            if (bodyA == null || bodyA.truss == null) return;
            if (bodyB != null && bodyB.truss == null) return;

            serializedObject.Update();

            for (int i = 0; i < linkConfigsProp.arraySize; i++)
            {
                SerializedProperty link = linkConfigsProp.GetArrayElementAtIndex(i);
                SerializedProperty nodeA = link.FindPropertyRelative("attachmentSetA");
                SerializedProperty nodeB = link.FindPropertyRelative("attachmentSetB");
                SerializedProperty attachmentTypeB = link.FindPropertyRelative("attachmentTypeB");
                SerializedProperty restLength = link.FindPropertyRelative("restLength");

                if (string.IsNullOrEmpty(nodeA.stringValue)) continue;

                // For body-to-body constraints
                if (bodyB != null && !string.IsNullOrEmpty(nodeB.stringValue))
                {
                    float calculatedLength = CalcLinkDistance(
                        nodeA.stringValue,
                        nodeB.stringValue,
                        (AttachmentType)attachmentTypeB.enumValueIndex
                    );

                    if (calculatedLength > 0)
                    {
                        restLength.floatValue = calculatedLength;
                    }
                }
                // World constraints don't need rest length updates
            }

            serializedObject.ApplyModifiedProperties();
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUIUtility.labelWidth = 120f;
            bool isPlaying = Application.isPlaying;
            SoftBody softBodyB = (SoftBody)softBodyBProp.objectReferenceValue;
            GUI.enabled = true && !isPlaying;
            EditorGUILayout.PropertyField(softBodyBProp, new GUIContent("Base Body", "The base soft body to connect to."));
            EditorGUILayout.PropertyField(disableCollisionProp, new GUIContent("Disable Collision", "Disable collision between connected bodies."));
            if (m_targets.Length > 1)
            {
                GUI.enabled = false;
                EditorGUILayout.LabelField("Select single body to edit links");
                GUI.enabled = true;
            }
            else
            {
                Constraint targetConstraint = m_targets[0];
                SoftBody softBodyA = targetConstraint.GetComponent<SoftBody>();
                // Clean up invalid links
                for (int i = linkConfigsProp.arraySize - 1; i >= 0; --i)
                {
                    SerializedProperty link = linkConfigsProp.GetArrayElementAtIndex(i);
                    SerializedProperty attachmentTypeB = link.FindPropertyRelative("attachmentTypeB");
                    // Only delete if we have a world attachment but also have a base body,
                    // or if we have a node/edge attachment but don't have a base body
                    bool shouldDelete = (attachmentTypeB.enumValueIndex == (int)AttachmentType.World && softBodyB != null) ||
                    (attachmentTypeB.enumValueIndex != (int)AttachmentType.World && softBodyB == null);
                    if (shouldDelete)
                    {
                        linkConfigsProp.DeleteArrayElementAtIndex(i);
                    }
                }
                GUI.enabled = true && !isPlaying;
                EditorGUILayout.BeginHorizontal();
                showLinksProp.boolValue = EditorGUILayout.Foldout(showLinksProp.boolValue, "Link List (" + linkConfigsProp.arraySize + " Item" + (linkConfigsProp.arraySize == 1 ? "" : "s") + ")");
                if (showLinksProp.boolValue)
                {
                    if (GUILayout.Button("Add", EditorStyles.miniButton, GUILayout.MaxWidth(50)))
                    {
                        int newIndex = linkConfigsProp.arraySize;
                        linkConfigsProp.InsertArrayElementAtIndex(newIndex);
                        // Initialize new link with new properties
                        SerializedProperty newLink = linkConfigsProp.GetArrayElementAtIndex(newIndex);
                        newLink.FindPropertyRelative("name").stringValue = "";
                        newLink.FindPropertyRelative("attachmentTypeA").enumValueIndex = (int)AttachmentType.Node;
                        newLink.FindPropertyRelative("attachmentSetA").stringValue = "";
                        // Set appropriate attachment type based on whether we have a base body
                        if (softBodyB != null)
                        {
                            newLink.FindPropertyRelative("attachmentTypeB").enumValueIndex = (int)AttachmentType.Node;
                        }
                        else
                        {
                            newLink.FindPropertyRelative("attachmentTypeB").enumValueIndex = (int)AttachmentType.World;
                        }
                        newLink.FindPropertyRelative("attachmentSetB").stringValue = "";
                        newLink.FindPropertyRelative("restLength").floatValue = 0f;
                        newLink.FindPropertyRelative("minLength").floatValue = 0f;
                        newLink.FindPropertyRelative("maxLength").floatValue = 2f;
                        newLink.FindPropertyRelative("strength").floatValue = 1000f; // Default strength
                        newLink.FindPropertyRelative("master").boolValue = false;
                        newLink.FindPropertyRelative("show").boolValue = true;
                    }
                }
                EditorGUILayout.EndHorizontal();
                if (showLinksProp.boolValue)
                {
                    EditorGUI.indentLevel++;
                    if (softBodyB != null)
                    {
                        for (int i = 0; i < linkConfigsProp.arraySize; ++i)
                        {
                            SerializedProperty link = linkConfigsProp.GetArrayElementAtIndex(i);
                            SerializedProperty showLink = link.FindPropertyRelative("show");
                            SerializedProperty linkName = link.FindPropertyRelative("name");

                            if (showLink.boolValue)
                            {
                                EditorGUILayout.BeginHorizontal();
                                string displayName = string.IsNullOrEmpty(linkName.stringValue) ? "Link " + (i + 1) : linkName.stringValue;
                                showLink.boolValue = EditorGUILayout.Foldout(showLink.boolValue, displayName);
                                if (GUILayout.Button("Remove", EditorStyles.miniButton, GUILayout.MaxWidth(50)))
                                {
                                    linkConfigsProp.DeleteArrayElementAtIndex(i);
                                    break;
                                }
                                EditorGUILayout.EndHorizontal();
                                EditorGUI.indentLevel++;

                                // Name field at the top
                                EditorGUILayout.PropertyField(linkName, new GUIContent("Name"));

                                // Body A attachment
                                SerializedProperty nodeA = link.FindPropertyRelative("attachmentSetA");
                                string[] nodesA = GetNodes(softBodyA, 1, 1);
                                int indexA = System.Array.IndexOf(nodesA, nodeA.stringValue);
                                indexA = EditorGUILayout.Popup("Node", indexA, nodesA);
                                if (indexA > -1 && indexA < nodesA.Length) nodeA.stringValue = nodesA[indexA];
                                // Attachment type
                                SerializedProperty attachmentTypeB = link.FindPropertyRelative("attachmentTypeB");
                                string[] attachmentOptions = new string[] { "Node", "Edge" };
                                int selectType = EditorGUILayout.Popup("Attach To", attachmentTypeB.enumValueIndex - 1, attachmentOptions);
                                attachmentTypeB.enumValueIndex = selectType + 1;
                                switch (attachmentTypeB.enumValueIndex)
                                {
                                    case 1: // Node
                                        {
                                            SerializedProperty nodeB = link.FindPropertyRelative("attachmentSetB");
                                            string[] nodesB = GetNodes(softBodyB, 1, 1);
                                            int indexB = System.Array.IndexOf(nodesB, nodeB.stringValue);
                                            indexB = EditorGUILayout.Popup("Node B", indexB, nodesB);
                                            if (indexB > -1 && indexB < nodesB.Length) nodeB.stringValue = nodesB[indexB];
                                        }
                                        break;
                                    case 2: // Edge
                                        {
                                            SerializedProperty nodeB = link.FindPropertyRelative("attachmentSetB");
                                            string[] nodesB = GetNodes(softBodyB, 2, 2);
                                            int indexB = System.Array.IndexOf(nodesB, nodeB.stringValue);
                                            indexB = EditorGUILayout.Popup("Edge B", indexB, nodesB);
                                            if (indexB > -1 && indexB < nodesB.Length) nodeB.stringValue = nodesB[indexB];
                                        }
                                        break;
                                }
                                // Auto-calculate rest length when nodes are selected
                                SerializedProperty restLength = link.FindPropertyRelative("restLength");
                                SerializedProperty minLength = link.FindPropertyRelative("minLength");
                                SerializedProperty maxLength = link.FindPropertyRelative("maxLength");
                                SerializedProperty strength = link.FindPropertyRelative("strength");
                                // Calculate rest length automatically
                                if (!string.IsNullOrEmpty(nodeA.stringValue) && !string.IsNullOrEmpty(link.FindPropertyRelative("attachmentSetB").stringValue))
                                {
                                    float calculatedLength = CalcLinkDistance(
                                        nodeA.stringValue,
                                        link.FindPropertyRelative("attachmentSetB").stringValue,
                                        (AttachmentType)attachmentTypeB.enumValueIndex
                                    );
                                    if (calculatedLength > 0)
                                    {
                                        restLength.floatValue = calculatedLength;
                                    }
                                }
                                EditorGUILayout.BeginHorizontal();
                                EditorGUILayout.PropertyField(minLength, new GUIContent("Min Length"));
                                if (GUILayout.Button("Auto", EditorStyles.miniButton, GUILayout.MaxWidth(40)))
                                {
                                    minLength.floatValue = restLength.floatValue * 0.8f;
                                }
                                EditorGUILayout.EndHorizontal();
                                EditorGUILayout.BeginHorizontal();
                                EditorGUILayout.PropertyField(maxLength, new GUIContent("Max Length"));
                                if (GUILayout.Button("Auto", EditorStyles.miniButton, GUILayout.MaxWidth(40)))
                                {
                                    maxLength.floatValue = restLength.floatValue * 1.2f;
                                }
                                EditorGUILayout.EndHorizontal();
                                EditorGUILayout.PropertyField(strength, new GUIContent("Strength", "Maximum force before breaking. Infinity = unbreakable."));
                                SerializedProperty master = link.FindPropertyRelative("master");
                                EditorGUILayout.PropertyField(master, new GUIContent("Master"));
                                EditorGUI.indentLevel--;
                            }
                            else
                            {
                                SerializedProperty nodeA = link.FindPropertyRelative("attachmentSetA");
                                string nodeName = nodeA.stringValue;
                                string targetName = "Point";
                                SerializedProperty attachmentTypeB = link.FindPropertyRelative("attachmentTypeB");
                                if (attachmentTypeB.enumValueIndex > 0)
                                {
                                    SerializedProperty nodeB = link.FindPropertyRelative("attachmentSetB");
                                    targetName = nodeB.stringValue;
                                }
                                // Show custom name if set, otherwise show default info
                                string displayLabel;
                                if (!string.IsNullOrEmpty(linkName.stringValue))
                                {
                                    displayLabel = linkName.stringValue;
                                }
                                else
                                {
                                    displayLabel = $"Link {i + 1}: {nodeName} → {targetName}";
                                }

                                showLink.boolValue = EditorGUILayout.Foldout(showLink.boolValue, displayLabel);
                            }
                        }
                    }
                    else
                    {
                        for (int i = 0; i < linkConfigsProp.arraySize; ++i)
                        {
                            SerializedProperty link = linkConfigsProp.GetArrayElementAtIndex(i);
                            SerializedProperty showLink = link.FindPropertyRelative("show");
                            SerializedProperty linkName = link.FindPropertyRelative("name");

                            if (showLink.boolValue)
                            {
                                EditorGUILayout.BeginHorizontal();
                                string displayName = string.IsNullOrEmpty(linkName.stringValue) ? "Link " + (i + 1) : linkName.stringValue;
                                showLink.boolValue = EditorGUILayout.Foldout(showLink.boolValue, displayName);
                                if (GUILayout.Button("Remove", EditorStyles.miniButton, GUILayout.MaxWidth(50)))
                                {
                                    linkConfigsProp.DeleteArrayElementAtIndex(i);
                                    break;
                                }
                                EditorGUILayout.EndHorizontal();
                                EditorGUI.indentLevel++;

                                // Name field at the top
                                EditorGUILayout.PropertyField(linkName, new GUIContent("Name"));

                                SerializedProperty nodeA = link.FindPropertyRelative("attachmentSetA");
                                string[] nodesA = GetNodes(softBodyA, 1, 1);
                                int indexA = System.Array.IndexOf(nodesA, nodeA.stringValue);
                                indexA = EditorGUILayout.Popup("Node", indexA, nodesA);
                                if (indexA > -1 && indexA < nodesA.Length) nodeA.stringValue = nodesA[indexA];
                                SerializedProperty restLength = link.FindPropertyRelative("restLength");
                                SerializedProperty strength = link.FindPropertyRelative("strength");
                                SerializedProperty maxLength = link.FindPropertyRelative("maxLength");
                                if (!string.IsNullOrEmpty(nodeA.stringValue))
                                {
                                    if (restLength.floatValue <= 0)
                                        restLength.floatValue = 1.0f;
                                    if (strength.floatValue <= 1f && !float.IsInfinity(strength.floatValue))
                                        strength.floatValue = 1000f; // Default for world constraints
                                }
                                EditorGUILayout.BeginHorizontal();
                                EditorGUILayout.PropertyField(maxLength, new GUIContent("Max Distance"));
                                if (GUILayout.Button("Auto", EditorStyles.miniButton, GUILayout.MaxWidth(40)))
                                {
                                    maxLength.floatValue = 1.0f; // Default for world constraints
                                }
                                EditorGUILayout.EndHorizontal();
                                EditorGUILayout.PropertyField(strength, new GUIContent("Strength", "Maximum force before breaking. Infinity = unbreakable."));
                                SerializedProperty master = link.FindPropertyRelative("master");
                                EditorGUILayout.PropertyField(master, new GUIContent("Master"));
                                EditorGUI.indentLevel--;
                            }
                            else
                            {
                                SerializedProperty nodeA = link.FindPropertyRelative("attachmentSetA");
                                SerializedProperty restLength = link.FindPropertyRelative("restLength");
                                string lengthInfo = restLength.floatValue > 0 ? $" D:{restLength.floatValue:F2}" : "";
                                SerializedProperty strength = link.FindPropertyRelative("strength");
                                string strengthInfo = float.IsInfinity(strength.floatValue) ? " STR:∞" : $" STR:{strength.floatValue:F0}";

                                // Show custom name if set, otherwise show default info
                                string displayLabel;
                                if (!string.IsNullOrEmpty(linkName.stringValue))
                                {
                                    displayLabel = linkName.stringValue + lengthInfo + strengthInfo;
                                }
                                else
                                {
                                    displayLabel = "Link " + (i + 1) + ": " + nodeA.stringValue + lengthInfo + strengthInfo;
                                }

                                showLink.boolValue = EditorGUILayout.Foldout(showLink.boolValue, displayLabel);
                            }
                        }
                    }
                    EditorGUI.indentLevel--;
                }
            }
            EditorGUILayout.Separator();
            // Reset GUI.enabled before motor section
            GUI.enabled = true;

            // Motor section - only show if we have a base body
            if (softBodyB != null)
            {
                EditorGUILayout.PropertyField(enableMotorProp, new GUIContent("Enable Motor"));

                if (enableMotorProp.boolValue)
                {
                    EditorGUI.indentLevel++;

                    if (m_targets.Length == 1)
                    {
                        GUI.enabled = !isPlaying; // Allow editing only when not playing
                        Constraint targetConstraint = m_targets[0];
                        SoftBody softBodyA = targetConstraint.GetComponent<SoftBody>();
                        string[] axisNodes = GetNodes(softBodyA, 2, 100); // Allow 2+ nodes for axis
                        int index = System.Array.IndexOf(axisNodes, axisNodeSetProp.stringValue);
                        index = EditorGUILayout.Popup("Axis Nodes (Base)", index, axisNodes);
                        if (index > -1 && index < axisNodes.Length)
                            axisNodeSetProp.stringValue = axisNodes[index];
                        else if (axisNodes.Length > 0 && string.IsNullOrEmpty(axisNodeSetProp.stringValue))
                            axisNodeSetProp.stringValue = axisNodes[0]; // Auto-select first if none selected
                    }
                    else
                    {
                        GUI.enabled = false;
                        string[] empty = { };
                        EditorGUILayout.Popup("Axis Nodes (Base)", -1, empty);
                        GUI.enabled = true;
                    }

                    GUI.enabled = true;
                    EditorGUILayout.PropertyField(targetRateProp, new GUIContent("Target Rate (deg/s)", "Target angular velocity in degrees per second"));
                    EditorGUILayout.PropertyField(maxTorqueProp, new GUIContent("Max Torque", "Maximum torque the motor can apply"));

                    // Show warning if axis nodes not set
                    if (enableMotorProp.boolValue && string.IsNullOrEmpty(axisNodeSetProp.stringValue))
                    {
                        EditorGUILayout.HelpBox("Motor requires axis nodes to be set! Select a NodeSet with 2+ nodes from the base body that define the rotation axis.", MessageType.Warning);
                    }

                    EditorGUI.indentLevel--;
                }
            }
            else
            {
                // No motor for world constraints
                GUI.enabled = false;
                EditorGUILayout.PropertyField(enableMotorProp, new GUIContent("Enable Motor", "Motors require a base body"));
                GUI.enabled = true;
            }

            // Always apply changes at the end
            if (GUI.changed) serializedObject.ApplyModifiedProperties();
        }
        private string[] GetNodes(SoftBody body, int minNodes = 1, int maxNodes = 1)
        {
            List<string> nodes = new List<string>();
            if (body != null && body.truss != null)
            {
                var nodeSets = body.truss.GetNodeSets();
                foreach (var nodeSet in nodeSets)
                {
                    if (nodeSet.nodeIndices.Count >= minNodes && nodeSet.nodeIndices.Count <= maxNodes)
                    {
                        nodes.Add(nodeSet.name);
                    }
                }
            }
            return nodes.ToArray();
        }
        private float CalcLinkDistance(string nodeSetA, string nodeSetB, AttachmentType attachmentTypeB)
        {
            Constraint constraint = m_targets[0];
            SoftBody bodyA = constraint.GetComponent<SoftBody>();
            SoftBody bodyB = constraint.baseBody;
            if (bodyA == null || bodyA.truss == null || bodyB == null || bodyB.truss == null)
                return 0f;
            int[] indicesA = bodyA.truss.GetNodeSetIndices(nodeSetA);
            if (indicesA == null || indicesA.Length != 1)
                return 0f;
            Vector3 positionA = bodyA.transform.TransformPoint(bodyA.truss.NodePositions[indicesA[0]]);
            if (attachmentTypeB == AttachmentType.Node)
            {
                int[] indicesB = bodyB.truss.GetNodeSetIndices(nodeSetB);
                if (indicesB == null || indicesB.Length != 1)
                    return 0f;
                Vector3 positionB = bodyB.transform.TransformPoint(bodyB.truss.NodePositions[indicesB[0]]);
                return Vector3.Distance(positionA, positionB);
            }
            else // Edge
            {
                int[] indicesB = bodyB.truss.GetNodeSetIndices(nodeSetB);
                if (indicesB == null || indicesB.Length != 2)
                    return 0f;
                Vector3 positionB1 = bodyB.transform.TransformPoint(bodyB.truss.NodePositions[indicesB[0]]);
                Vector3 positionB2 = bodyB.transform.TransformPoint(bodyB.truss.NodePositions[indicesB[1]]);
                // Calculate closest point on edge to positionA
                Vector3 edgeDir = (positionB2 - positionB1).normalized;

                // UNCLAMPED FOR INFINITE EDGE
                float t = Vector3.Dot(positionA - positionB1, edgeDir);
                // t = Mathf.Clamp01(t); // Removed

                Vector3 closestPoint = positionB1 + t * edgeDir;
                return Vector3.Distance(positionA, closestPoint);
            }
        }
        private void OnSceneGUI()
        {
            // DISABLE EDITOR VISUALIZATION DURING RUNTIME
            if (Application.isPlaying)
                return;

            Constraint constraint = target as Constraint;
            if (constraint == null) return;

            SoftBody bodyA = constraint.GetComponent<SoftBody>();
            SoftBody bodyB = constraint.baseBody;

            if (bodyA == null || bodyA.truss == null) return;
            if (bodyB != null && bodyB.truss == null) return;

            SerializedObject so = new SerializedObject(constraint);
            SerializedProperty linksProp = so.FindProperty("m_links");
            SerializedProperty linkVisibilityProp = so.FindProperty("linkVisibility");

            // Sync visibility list
            if (linkVisibilityProp.arraySize != linksProp.arraySize)
                return;

            for (int i = 0; i < linksProp.arraySize; i++)
            {
                // Check visibility from linkVisibility list
                if (i >= linkVisibilityProp.arraySize || !linkVisibilityProp.GetArrayElementAtIndex(i).boolValue)
                    continue;

                SerializedProperty linkProp = linksProp.GetArrayElementAtIndex(i);
                var link = constraint.links[i];

                int[] indicesA = bodyA.truss.GetNodeSetIndices(link.attachmentSetA);
                if (indicesA == null) continue;

                Vector3 posA = GetAttachmentPointEditor(bodyA, indicesA, link.attachmentTypeA);
                if (posA == Vector3.zero && bodyA.transform.position.magnitude > 0.1f) continue;

                if (bodyB != null)
                {
                    int[] indicesB = bodyB.truss.GetNodeSetIndices(link.attachmentSetB);
                    if (indicesB == null) continue;

                    Vector3 posB;

                    // Handle Edge attachment type B
                    if (link.attachmentTypeB == AttachmentType.Edge && indicesB.Length == 2)
                    {
                        posB = GetProjectedPointOnEdgeEditor(bodyB, indicesB, posA);
                        if (posB == Vector3.zero && bodyB.transform.position.magnitude > 0.1f) continue;

                        // Draw edge line and endpoints
                        Vector3 edgeStart = bodyB.transform.TransformPoint(bodyB.truss.NodePositions[indicesB[0]]);
                        Vector3 edgeEnd = bodyB.transform.TransformPoint(bodyB.truss.NodePositions[indicesB[1]]);

                        Handles.color = new Color(0.7f, 0.7f, 0.7f, 0.5f);

                        // DRAW INFINITE LINE FOR EDGE
                        Vector3 edgeDir = (edgeEnd - edgeStart).normalized;
                        if (edgeDir.sqrMagnitude > 0f)
                        {
                            Handles.DrawLine(edgeStart - edgeDir * 1000f, edgeEnd + edgeDir * 1000f);
                        }
                        else
                        {
                            Handles.DrawLine(edgeStart, edgeEnd);
                        }

                        float edgeSphereSize = HandleUtility.GetHandleSize(edgeStart) * 0.03f;
                        Handles.SphereHandleCap(0, edgeStart, Quaternion.identity, edgeSphereSize, EventType.Repaint);
                        Handles.SphereHandleCap(0, edgeEnd, Quaternion.identity, edgeSphereSize, EventType.Repaint);
                    }
                    // Handle Edge attachment type A
                    else if (link.attachmentTypeA == AttachmentType.Edge && indicesA.Length == 2)
                    {
                        posB = GetAttachmentPointEditor(bodyB, indicesB, link.attachmentTypeB);
                        if (posB == Vector3.zero && bodyB.transform.position.magnitude > 0.1f) continue;

                        posA = GetProjectedPointOnEdgeEditor(bodyA, indicesA, posB);
                        if (posA == Vector3.zero && bodyA.transform.position.magnitude > 0.1f) continue;

                        // Draw edge line and endpoints
                        Vector3 edgeStart = bodyA.transform.TransformPoint(bodyA.truss.NodePositions[indicesA[0]]);
                        Vector3 edgeEnd = bodyA.transform.TransformPoint(bodyA.truss.NodePositions[indicesA[1]]);

                        Handles.color = new Color(0.7f, 0.7f, 0.7f, 0.5f);

                        // DRAW INFINITE LINE FOR EDGE
                        Vector3 edgeDir = (edgeEnd - edgeStart).normalized;
                        if (edgeDir.sqrMagnitude > 0f)
                        {
                            Handles.DrawLine(edgeStart - edgeDir * 1000f, edgeEnd + edgeDir * 1000f);
                        }
                        else
                        {
                            Handles.DrawLine(edgeStart, edgeEnd);
                        }

                        float edgeSphereSize = HandleUtility.GetHandleSize(edgeStart) * 0.03f;
                        Handles.SphereHandleCap(0, edgeStart, Quaternion.identity, edgeSphereSize, EventType.Repaint);
                        Handles.SphereHandleCap(0, edgeEnd, Quaternion.identity, edgeSphereSize, EventType.Repaint);
                    }
                    else
                    {
                        posB = GetAttachmentPointEditor(bodyB, indicesB, link.attachmentTypeB);
                        if (posB == Vector3.zero && bodyB.transform.position.magnitude > 0.1f) continue;
                    }

                    // Draw main link line
                    Handles.color = link.master ? Color.red : Color.cyan;
                    Handles.DrawLine(posA, posB);

                    // Draw endpoint spheres with semi-transparent color
                    Color sphereColor = link.master ? new Color(1f, 0f, 0f, 0.4f) : new Color(0f, 1f, 1f, 0.4f);
                    float sphereRadius = 0.02f;

                    // Draw sphere at posA
                    Handles.color = sphereColor;
                    Handles.DrawSolidDisc(posA, Vector3.up, sphereRadius);
                    Handles.DrawSolidDisc(posA, Vector3.right, sphereRadius);
                    Handles.DrawSolidDisc(posA, Vector3.forward, sphereRadius);

                    // Draw sphere at posB
                    Handles.DrawSolidDisc(posB, Vector3.up, sphereRadius);
                    Handles.DrawSolidDisc(posB, Vector3.right, sphereRadius);
                    Handles.DrawSolidDisc(posB, Vector3.forward, sphereRadius);

                    // Draw min/max limit spheres as wireframe radii
                    if (link.minLength > 0)
                    {
                        Handles.color = new Color(1f, 0f, 0f, 0.4f);
                        Handles.DrawWireDisc(posA, Vector3.up, link.minLength);
                        Handles.DrawWireDisc(posA, Vector3.right, link.minLength);
                        Handles.DrawWireDisc(posA, Vector3.forward, link.minLength);
                    }

                    if (link.maxLength > 0)
                    {
                        Handles.color = new Color(0f, 1f, 0f, 0.4f);
                        Handles.DrawWireDisc(posA, Vector3.up, link.maxLength);
                        Handles.DrawWireDisc(posA, Vector3.right, link.maxLength);
                        Handles.DrawWireDisc(posA, Vector3.forward, link.maxLength);
                    }
                }
                else
                {
                    // World constraint visualization
                    Handles.color = link.master ? new Color(1f, 0f, 0f, 0.4f) : new Color(0f, 1f, 0f, 0.4f);
                    float sphereRadius = 0.025f;

                    // Draw semi-transparent sphere at posA
                    Handles.DrawSolidDisc(posA, Vector3.up, sphereRadius);
                    Handles.DrawSolidDisc(posA, Vector3.right, sphereRadius);
                    Handles.DrawSolidDisc(posA, Vector3.forward, sphereRadius);

                    // Draw max distance sphere for world constraints
                    if (link.maxLength > 0)
                    {
                        Handles.color = new Color(0f, 1f, 0f, 0.3f);
                        Handles.DrawWireDisc(posA, Vector3.up, link.maxLength);
                        Handles.DrawWireDisc(posA, Vector3.right, link.maxLength);
                        Handles.DrawWireDisc(posA, Vector3.forward, link.maxLength);
                    }
                }
            }
        }

        private Vector3 GetAttachmentPointEditor(SoftBody body, int[] indices, AttachmentType type)
        {
            if (body == null || body.truss == null || indices == null || indices.Length == 0)
                return Vector3.zero;

            if (type == AttachmentType.Node)
            {
                if (indices[0] < 0 || indices[0] >= body.truss.NodePositions.Length)
                    return Vector3.zero;
                return body.transform.TransformPoint(body.truss.NodePositions[indices[0]]);
            }
            else // Edge
            {
                if (indices.Length >= 2)
                {
                    if (indices[0] < 0 || indices[0] >= body.truss.NodePositions.Length ||
                        indices[1] < 0 || indices[1] >= body.truss.NodePositions.Length)
                        return Vector3.zero;
                    Vector3 pos1 = body.transform.TransformPoint(body.truss.NodePositions[indices[0]]);
                    Vector3 pos2 = body.transform.TransformPoint(body.truss.NodePositions[indices[1]]);
                    return (pos1 + pos2) * 0.5f;
                }
                else
                {
                    if (indices[0] < 0 || indices[0] >= body.truss.NodePositions.Length)
                        return Vector3.zero;
                    return body.transform.TransformPoint(body.truss.NodePositions[indices[0]]);
                }
            }
        }

        private Vector3 GetProjectedPointOnEdgeEditor(SoftBody body, int[] edgeIndices, Vector3 point)
        {
            if (body == null || body.truss == null || edgeIndices == null || edgeIndices.Length != 2)
                return Vector3.zero;

            if (edgeIndices[0] < 0 || edgeIndices[0] >= body.truss.NodePositions.Length ||
                edgeIndices[1] < 0 || edgeIndices[1] >= body.truss.NodePositions.Length)
                return Vector3.zero;

            Vector3 edgeStart = body.transform.TransformPoint(body.truss.NodePositions[edgeIndices[0]]);
            Vector3 edgeEnd = body.transform.TransformPoint(body.truss.NodePositions[edgeIndices[1]]);

            Vector3 edgeVector = edgeEnd - edgeStart;
            float edgeLength = edgeVector.magnitude;

            if (edgeLength < 0.001f) return edgeStart;

            Vector3 edgeDir = edgeVector / edgeLength;
            Vector3 pointToEdgeStart = point - edgeStart;
            float t = Vector3.Dot(pointToEdgeStart, edgeDir);

            // UNCLAMPED FOR INFINITE EDGE
            // t = Mathf.Clamp(t, 0f, edgeLength);

            return edgeStart + edgeDir * t;
        }


    }
}