/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */
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
        }
        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUIUtility.labelWidth = 120f;
            bool isPlaying = Application.isPlaying;
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
                SoftBody softBodyB = (SoftBody)softBodyBProp.objectReferenceValue;
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
                        newLink.FindPropertyRelative("stiffness").floatValue = 1000f; // Default to stiff (k)
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
                            if (showLink.boolValue)
                            {
                                EditorGUILayout.BeginHorizontal();
                                showLink.boolValue = EditorGUILayout.Foldout(showLink.boolValue, "Link " + (i + 1));
                                if (GUILayout.Button("Remove", EditorStyles.miniButton, GUILayout.MaxWidth(50)))
                                {
                                    linkConfigsProp.DeleteArrayElementAtIndex(i);
                                    break;
                                }
                                EditorGUILayout.EndHorizontal();
                                EditorGUI.indentLevel++;
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
                                SerializedProperty stiffness = link.FindPropertyRelative("stiffness");
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
                                EditorGUILayout.PropertyField(stiffness, new GUIContent("Stiffness (k)")); // Updated label
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
                                SerializedProperty restLength = link.FindPropertyRelative("restLength");
                                string lengthInfo = restLength.floatValue > 0 ? $" L:{restLength.floatValue:F2}" : "";
                                SerializedProperty stiffness = link.FindPropertyRelative("stiffness");
                                string stiffnessInfo = $" S:{stiffness.floatValue:F0}"; // Show as integer for large k
                                showLink.boolValue = EditorGUILayout.Foldout(showLink.boolValue,
                               $"Link {i + 1}: {nodeName} → {targetName}{lengthInfo}{stiffnessInfo}");
                            }
                        }
                    }
                    else
                    {
                        for (int i = 0; i < linkConfigsProp.arraySize; ++i)
                        {
                            SerializedProperty link = linkConfigsProp.GetArrayElementAtIndex(i);
                            SerializedProperty showLink = link.FindPropertyRelative("show");
                            if (showLink.boolValue)
                            {
                                EditorGUILayout.BeginHorizontal();
                                showLink.boolValue = EditorGUILayout.Foldout(showLink.boolValue, "Link " + (i + 1));
                                if (GUILayout.Button("Remove", EditorStyles.miniButton, GUILayout.MaxWidth(50)))
                                {
                                    linkConfigsProp.DeleteArrayElementAtIndex(i);
                                    break;
                                }
                                EditorGUILayout.EndHorizontal();
                                EditorGUI.indentLevel++;
                                SerializedProperty nodeA = link.FindPropertyRelative("attachmentSetA");
                                string[] nodesA = GetNodes(softBodyA, 1, 1);
                                int indexA = System.Array.IndexOf(nodesA, nodeA.stringValue);
                                indexA = EditorGUILayout.Popup("Node", indexA, nodesA);
                                if (indexA > -1 && indexA < nodesA.Length) nodeA.stringValue = nodesA[indexA];
                                SerializedProperty restLength = link.FindPropertyRelative("restLength");
                                SerializedProperty stiffness = link.FindPropertyRelative("stiffness");
                                SerializedProperty minLength = link.FindPropertyRelative("minLength");
                                if (!string.IsNullOrEmpty(nodeA.stringValue))
                                {
                                    if (restLength.floatValue <= 0)
                                        restLength.floatValue = 1.0f;
                                    if (stiffness.floatValue <= 1f)
                                        stiffness.floatValue = 1000f; // Default for world constraints
                                }
                                EditorGUILayout.BeginHorizontal();
                                EditorGUILayout.PropertyField(minLength, new GUIContent("Max Distance")); // Changed from minLength to maxLength for world constraints
                                if (GUILayout.Button("Auto", EditorStyles.miniButton, GUILayout.MaxWidth(40)))
                                {
                                    minLength.floatValue = 1.0f; // Default for world constraints
                                }
                                EditorGUILayout.EndHorizontal();
                                EditorGUILayout.PropertyField(stiffness, new GUIContent("Strength (k)")); // Updated label
                                SerializedProperty master = link.FindPropertyRelative("master");
                                EditorGUILayout.PropertyField(master, new GUIContent("Master"));
                                EditorGUI.indentLevel--;
                            }
                            else
                            {
                                SerializedProperty nodeA = link.FindPropertyRelative("attachmentSetA");
                                SerializedProperty restLength = link.FindPropertyRelative("restLength");
                                string lengthInfo = restLength.floatValue > 0 ? $" D:{restLength}" : "";
                                SerializedProperty stiffness = link.FindPropertyRelative("stiffness");
                                string stiffnessInfo = $" S:{stiffness.floatValue:F0}"; // Show as integer
                                showLink.boolValue = EditorGUILayout.Foldout(showLink.boolValue,
                               "Link " + (i + 1) + ": " + nodeA.stringValue + lengthInfo + stiffnessInfo);
                            }
                        }
                    }
                    EditorGUI.indentLevel--;
                }
            }
            EditorGUILayout.Separator();
            // Motor section
            EditorGUILayout.PropertyField(enableMotorProp, new GUIContent("Enable Motor"));
            EditorGUI.indentLevel++;
            if (m_targets.Length == 1)
            {
                GUI.enabled = true && !isPlaying;
                SoftBody softBodyA = m_targets[0].GetComponent<SoftBody>();
                string[] axisNodes = GetNodes(softBodyA, 2, 2);
                int index = System.Array.IndexOf(axisNodes, axisNodeSetProp.stringValue);
                index = EditorGUILayout.Popup("Axis Nodes", index, axisNodes);
                if (index > -1 && index < axisNodes.Length) axisNodeSetProp.stringValue = axisNodes[index];
            }
            else
            {
                GUI.enabled = false;
                string[] empty = { };
                EditorGUILayout.Popup("Axis Nodes", -1, empty);
                GUI.enabled = true;
            }
            GUI.enabled = true;
            EditorGUILayout.PropertyField(targetRateProp, new GUIContent("Target Rate"));
            EditorGUILayout.PropertyField(maxTorqueProp, new GUIContent("Max Torque"));
            EditorGUI.indentLevel--;
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
                Vector3 edgeDir = positionB2 - positionB1;
                float t = Vector3.Dot(positionA - positionB1, edgeDir) / edgeDir.sqrMagnitude;
                t = Mathf.Clamp01(t);
                Vector3 closestPoint = positionB1 + t * edgeDir;
                return Vector3.Distance(positionA, closestPoint);
            }
        }
        private void OnSceneGUI()
        {
            Constraint constraint = target as Constraint;
            if (constraint == null || !constraint.showLinks) return;
            SoftBody softBodyA = constraint.GetComponent<SoftBody>();
            SoftBody softBodyB = constraint.baseBody;
            if (softBodyA == null || softBodyA.truss == null) return;
            // Draw all links
            for (int i = 0; i < constraint.links.Count; i++)
            {
                var link = constraint.links[i];
                if (!link.show) continue;
                int[] indicesA = softBodyA.truss.GetNodeSetIndices(link.attachmentSetA);
                if (indicesA == null || indicesA.Length == 0) continue;
                // Get position for body A
                Vector3 positionA = GetWorldPosition(softBodyA, indicesA, link.attachmentTypeA);
                if (softBodyB != null && softBodyB.truss != null && link.attachmentTypeB != AttachmentType.World)
                {
                    // Connection to another soft body
                    int[] indicesB = softBodyB.truss.GetNodeSetIndices(link.attachmentSetB);
                    if (indicesB == null || indicesB.Length == 0) continue;
                    Vector3 positionB = GetWorldPosition(softBodyB, indicesB, link.attachmentTypeB);
                    // Draw connection line
                    Handles.color = link.master ? Color.red : Color.cyan;
                    Handles.DrawDottedLine(positionA, positionB, 2f);
                    // Draw min/max length visualization
                    if (link.restLength > 0)
                    {
                        float currentDistance = Vector3.Distance(positionA, positionB);
                        // Draw min length sphere
                        if (link.minLength > 0)
                        {
                            Handles.color = new Color(1f, 0f, 0f, 0.1f); // Transparent red
                            Handles.SphereHandleCap(0, positionA, Quaternion.identity,
                            link.minLength * 2f, EventType.Repaint);
                            Handles.DrawWireDisc(positionA, Camera.current.transform.forward, link.minLength);
                        }
                        // Draw max length sphere
                        if (link.maxLength > 0)
                        {
                            Handles.color = new Color(0f, 1f, 0f, 0.1f); // Transparent green
                            Handles.SphereHandleCap(0, positionA, Quaternion.identity,
                            link.maxLength * 2f, EventType.Repaint);
                            Handles.DrawWireDisc(positionA, Camera.current.transform.forward, link.maxLength);
                        }
                        // Draw current length indicator
                        Vector3 midPoint = (positionA + positionB) * 0.5f;
                        Handles.Label(midPoint, $"L: {currentDistance:F2}\nMin: {link.minLength:F2}\nMax: {link.maxLength:F2}\nS: {link.stiffness:F0}");
                    }
                }
                else
                {
                    // World constraint
                    Handles.color = link.master ? Color.red : Color.green;
                    Handles.SphereHandleCap(0, positionA, Quaternion.identity,
                    HandleUtility.GetHandleSize(positionA) * 0.15f, EventType.Repaint);
                    // Draw max distance sphere if specified
                    if (link.restLength > 0)
                    {
                        Handles.color = new Color(1f, 1f, 0f, 0.2f); // Transparent yellow
                        Handles.SphereHandleCap(0, positionA, Quaternion.identity,
                        link.restLength * 2f, EventType.Repaint);
                        Handles.color = Color.yellow;
                        Handles.DrawWireDisc(positionA, Camera.current.transform.forward, link.restLength);
                        Handles.Label(positionA + Vector3.up * link.restLength * 1.2f,
                       $"Max: {link.restLength:F2}\nS: {link.stiffness:F0}");
                    }
                }
                // Draw attachment point label with stiffness info
                Handles.Label(positionA, $"{link.attachmentSetA}\nS:{link.stiffness:F0}");
            }
            // Draw motor visualization if enabled
            if (constraint.enableMotor && !string.IsNullOrEmpty(constraint.axisNodeSet))
            {
                int[] axisIndices = softBodyA.truss.GetNodeSetIndices(constraint.axisNodeSet);
                if (axisIndices != null && axisIndices.Length == 2)
                {
                    Vector3 pos1 = softBodyA.transform.TransformPoint(softBodyA.truss.NodePositions[axisIndices[0]]);
                    Vector3 pos2 = softBodyA.transform.TransformPoint(softBodyA.truss.NodePositions[axisIndices[1]]);
                    // Draw axis line
                    Handles.color = Color.magenta;
                    Handles.DrawLine(pos1, pos2);
                    // Draw axis endpoints
                    Handles.SphereHandleCap(0, pos1, Quaternion.identity,
                    HandleUtility.GetHandleSize(pos1) * 0.1f, EventType.Repaint);
                    Handles.SphereHandleCap(0, pos2, Quaternion.identity,
                    HandleUtility.GetHandleSize(pos2) * 0.1f, EventType.Repaint);
                    // Draw rotation indicator
                    Vector3 axisCenter = (pos1 + pos2) * 0.5f;
                    Vector3 axisDir = (pos2 - pos1).normalized;
                    if (axisDir.magnitude > 0.1f)
                    {
                        // Find a perpendicular vector for the circle
                        Vector3 perpendicular = Vector3.Cross(axisDir, Vector3.up);
                        if (perpendicular.magnitude < 0.1f)
                            perpendicular = Vector3.Cross(axisDir, Vector3.right);
                        perpendicular = perpendicular.normalized;
                        float circleRadius = HandleUtility.GetHandleSize(axisCenter) * 0.5f;
                        // Draw rotation circle
                        Handles.color = new Color(1f, 0.5f, 0f, 0.3f); // Orange transparent
                        Handles.DrawSolidArc(axisCenter, axisDir, perpendicular, 360f, circleRadius);
                        // Draw rotation direction arrow
                        Handles.color = Color.red;
                        float arrowSize = circleRadius * 0.3f;
                        Vector3 arrowPos = axisCenter + perpendicular * circleRadius;
                        Handles.ArrowHandleCap(0, arrowPos,
                        Quaternion.LookRotation(Vector3.Cross(perpendicular, axisDir)),
                        arrowSize, EventType.Repaint);
                        // Draw torque label
                        Handles.Label(axisCenter + axisDir * circleRadius * 1.5f,
                        $"Motor: {constraint.targetRate}°/s\nMax: {constraint.maxTorque}Nm");
                    }
                }
            }
        }
        private Vector3 GetWorldPosition(SoftBody body, int[] indices, AttachmentType attachmentType)
        {
            if (body == null || body.truss == null || indices == null || indices.Length == 0)
                return Vector3.zero;
            if (attachmentType == AttachmentType.Node)
            {
                return body.transform.TransformPoint(body.truss.NodePositions[indices[0]]);
            }
            else // Edge or World
            {
                if (indices.Length >= 2)
                {
                    Vector3 pos1 = body.transform.TransformPoint(body.truss.NodePositions[indices[0]]);
                    Vector3 pos2 = body.transform.TransformPoint(body.truss.NodePositions[indices[1]]);
                    return (pos1 + pos2) * 0.5f;
                }
                else
                {
                    return body.transform.TransformPoint(body.truss.NodePositions[indices[0]]);
                }
            }
        }
    }
}