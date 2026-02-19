/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;

namespace DynamicEngine
{
    [RequireComponent(typeof(Camera))]
    public class CameraVisualizerController : MonoBehaviour
    {
        [Header("Movement")]
        [SerializeField, Range(1f, 20f)] private float cameraMoveSpeed = 5f;
        [SerializeField, Range(1f, 5f)] private float cameraSprintMultiplier = 2f;
        [SerializeField, Range(0.1f, 5f)] private float cameraMouseSensitivity = 2f;

        [Header("Visualization")]
        [SerializeField] private bool enableNodeVisualization = true;
        [SerializeField] private bool autoFindSoftBodies = true;
        [SerializeField] private bool autoRefreshSoftBodies = false;

        [Header("Grabbing")]
        [SerializeField, Range(0.1f, 5f)] private float nodeVisualBaseSize = 0.3f;
        [SerializeField, Range(0.1f, 5f)] private float nodeVisualDistanceScale = 0.05f;
        [SerializeField, Range(1f, 1000f)] private float nodeGrabForce = 100f;
        [SerializeField, Range(0.1f, 2f)] private float nodeGrabRadius = 0.5f;
        [SerializeField, Range(0.5f, 10f)] private float nodeVisualShowRadius = 2f;
        [SerializeField] private bool enableGrabbingInPlay = true;
        [SerializeField] private bool enableGrabbingInEdit = true;

        [Header("Colors")]
        [SerializeField] private Color nodeColorNormal = new Color(1f, 0.5f, 0f);
        [SerializeField] private Color nodeColorHover = Color.yellow;
        [SerializeField] private Color nodeColorGrabbed = Color.red;
        [SerializeField] private Color nodeColorPinned = Color.magenta;
        [SerializeField] private bool showGrabRadiusGizmo = true;

        // Movement variables
        private float yaw = 0f;
        private float pitch = 0f;

        // Visualization variables
        private SoftBody[] softBodies;
        private Camera attachedCamera;
        private float lastRefreshTime = 0f;
        private const float REFRESH_INTERVAL = 1f;

        // Grabbing variables
        private SoftBody draggedSoftBody;
        private int draggedNodeIndex = -1;
        private Vector3 dragOffset;
        private Plane fixedDragPlane;
        private Vector3 targetPosition;
        private SoftBody hoveredSoftBody;
        private int hoveredNodeIndex = -1;
        private bool isControlPressed = false;
        private bool wasMousePressed = false;
        private float lastGrabForceMag = 0f;
        private Vector3 lastValidMousePos;

        // Visuals management
        private Dictionary<SoftBody, GameObject> visualsContainers = new Dictionary<SoftBody, GameObject>();
        private Dictionary<SoftBody, List<GameObject>> nodeVisualsDict = new Dictionary<SoftBody, List<GameObject>>();
        private Dictionary<SoftBody, LineRenderer> dragLineRenderers = new Dictionary<SoftBody, LineRenderer>();

        void Awake()
        {
            attachedCamera = GetComponent<Camera>();
        }

        void Start()
        {
            if (autoFindSoftBodies)
            {
                RefreshSoftBodies();
            }

            // Initialize lastValidMousePos to current mouse position so initial rays are sensible.
            if (Mouse.current != null)
            {
                lastValidMousePos = Mouse.current.position.ReadValue();
            }
            else
            {
                lastValidMousePos = new Vector3(Screen.width / 2f, Screen.height / 2f, 0f);
            }

            Vector3 eulerAngles = transform.eulerAngles;
            yaw = eulerAngles.y;
            pitch = eulerAngles.x;
            if (pitch > 180f)
                pitch -= 360f;
        }

        void Update()
        {
            HandleCursor();
            HandleMovement();
            HandleRotation();
            HandleNodeGrabbing();

            if (autoRefreshSoftBodies ||
                (autoFindSoftBodies && Time.time - lastRefreshTime > REFRESH_INTERVAL))
            {
                RefreshSoftBodies();
                lastRefreshTime = Time.time;
            }
        }

        void FixedUpdate()
        {
            if (draggedSoftBody != null && draggedNodeIndex >= 0 && draggedSoftBody.solver?.nodeManager?.Nodes != null)
            {
                if (draggedNodeIndex < draggedSoftBody.solver.nodeManager.Nodes.Count)
                {
                    Vector3 nodePosition = draggedSoftBody.solver.nodeManager.Nodes[draggedNodeIndex].position;
                    Vector3 forceDirection = (targetPosition - nodePosition);
                    float distance = forceDirection.magnitude;

                    if (distance > 0.01f)
                    {
                        forceDirection = forceDirection.normalized;

                        // Force grows with distance
                        float forceMagnitude = distance * nodeGrabForce;

                        // Safety clamp (configurable multiplier)
                        forceMagnitude = Mathf.Min(forceMagnitude, nodeGrabForce * 50f);

                        // Use world-space force directly
                        draggedSoftBody.solver.ApplyWorldForceToNode(draggedNodeIndex, forceDirection * forceMagnitude);
                        lastGrabForceMag = forceMagnitude;
                    }
                    else
                    {
                        lastGrabForceMag = 0f;
                    }
                }
            }
        }

        private void HandleNodeGrabbing()
        {
            if (!enableNodeVisualization || (!enableGrabbingInPlay && Application.isPlaying) || (!enableGrabbingInEdit && !Application.isPlaying))
                return;

            if (Keyboard.current == null || Mouse.current == null)
                return;

#if UNITY_EDITOR
            if (Application.isPlaying && UnityEditor.EditorWindow.focusedWindow != null &&
                UnityEditor.EditorWindow.focusedWindow.GetType().Name == "SceneView")
            {
                return;
            }
#endif

            isControlPressed = Keyboard.current.leftCtrlKey.isPressed || Keyboard.current.rightCtrlKey.isPressed;

            if (isControlPressed)
            {
                // Ensure visuals exist for soft bodies
                if (softBodies != null)
                {
                    foreach (var sb in softBodies)
                    {
                        if (sb != null && !visualsContainers.ContainsKey(sb))
                        {
                            SetupSoftBodyVisuals(sb);
                        }
                    }
                }

                Vector3 mousePos = Mouse.current.position.ReadValue();

                // Only update lastValidMousePos if mouse is inside the screen
                if (mousePos.x >= 0 && mousePos.y >= 0 && mousePos.x <= Screen.width && mousePos.y <= Screen.height &&
                    !float.IsNaN(mousePos.x) && !float.IsNaN(mousePos.y) &&
                    !float.IsInfinity(mousePos.x) && !float.IsInfinity(mousePos.y))
                {
                    lastValidMousePos = mousePos;
                }

                Ray ray = attachedCamera.ScreenPointToRay(lastValidMousePos);

                // Hover detection
                hoveredSoftBody = null;
                hoveredNodeIndex = -1;
                float closestDist = float.MaxValue;

                if (softBodies != null)
                {
                    foreach (var sb in softBodies)
                    {
                        if (sb == null || sb.solver?.nodeManager?.Nodes == null) continue;

                        for (int i = 0; i < sb.solver.nodeManager.Nodes.Count; i++)
                        {
                            var node = sb.solver.nodeManager.Nodes[i];
                            if (node == null) continue;

                            Vector3 nodePos = node.position;
                            float rayDist = Vector3.Cross(ray.direction, nodePos - ray.origin).magnitude;

                            if (rayDist < nodeGrabRadius && rayDist < closestDist)
                            {
                                closestDist = rayDist;
                                hoveredSoftBody = sb;
                                hoveredNodeIndex = i;
                            }
                        }
                    }
                }

                // Start dragging
                if (Mouse.current.leftButton.wasPressedThisFrame && hoveredSoftBody != null && hoveredNodeIndex >= 0)
                {
                    draggedSoftBody = hoveredSoftBody;
                    draggedNodeIndex = hoveredNodeIndex;

                    Vector3 nodePos = draggedSoftBody.solver.nodeManager.Nodes[draggedNodeIndex].position;
                    Ray clickRay = attachedCamera.ScreenPointToRay(lastValidMousePos);

                    fixedDragPlane = new Plane(attachedCamera.transform.forward, nodePos);

                    if (fixedDragPlane.Raycast(clickRay, out float enterClick))
                    {
                        Vector3 clickPoint = clickRay.GetPoint(enterClick);
                        dragOffset = nodePos - clickPoint;
                    }
                    else
                    {
                        float approxDepth = Vector3.Dot(nodePos - clickRay.origin, clickRay.direction);
                        approxDepth = Mathf.Max(0.01f, approxDepth);
                        Vector3 fallbackPoint = clickRay.GetPoint(approxDepth);
                        dragOffset = nodePos - fallbackPoint;
                    }

                    targetPosition = nodePos;
                    wasMousePressed = true;
                }

                // Update target position + middle-click pin + release (ONLY when actually dragging)
                if (draggedSoftBody != null && draggedNodeIndex >= 0 &&
                    draggedSoftBody.solver != null &&
                    draggedSoftBody.solver.nodeManager != null &&
                    draggedSoftBody.solver.nodeManager.Nodes != null &&
                    draggedNodeIndex < draggedSoftBody.solver.nodeManager.Nodes.Count &&
                    draggedSoftBody.solver.nodeManager.Nodes[draggedNodeIndex] != null)
                {
                    Ray dragRay = attachedCamera.ScreenPointToRay(lastValidMousePos);

                    if (fixedDragPlane.Raycast(dragRay, out float enter))
                    {
                        targetPosition = dragRay.GetPoint(enter) + dragOffset;
                    }
                    else
                    {
                        // Safe fallback
                        Vector3 currentNodePos = draggedSoftBody.solver.nodeManager.Nodes[draggedNodeIndex].position;
                        float approxDepth = Vector3.Dot(currentNodePos - dragRay.origin, dragRay.direction);
                        approxDepth = Mathf.Max(0.01f, approxDepth);
                        targetPosition = dragRay.GetPoint(approxDepth) + dragOffset;
                    }

                    // Middle mouse button = toggle pin
                    if (Mouse.current.middleButton.wasPressedThisFrame)
                    {
                        bool currentPinned = draggedSoftBody.solver.nodeManager.IsPinned[draggedNodeIndex];
                        draggedSoftBody.solver.nodeManager.SetPinned(draggedNodeIndex, !currentPinned);
                    }
                }
                else if (draggedSoftBody != null) // safety auto-release if something got destroyed mid-drag
                {
                    draggedSoftBody = null;
                    draggedNodeIndex = -1;
                    wasMousePressed = false;
                    lastGrabForceMag = 0f;
                }

                // Mouse up = release
                if (Mouse.current.leftButton.wasReleasedThisFrame && wasMousePressed)
                {
                    draggedSoftBody = null;
                    draggedNodeIndex = -1;
                    wasMousePressed = false;
                    lastGrabForceMag = 0f;
                }

                // Visuals update
                if (softBodies != null)
                {
                    foreach (var sb in softBodies)
                    {
                        if (sb == null) continue;
                        int hoverIdx = (sb == hoveredSoftBody) ? hoveredNodeIndex : -1;
                        int dragIdx = (sb == draggedSoftBody) ? draggedNodeIndex : -1;
                        Vector3 targ = (sb == draggedSoftBody) ? targetPosition : Vector3.zero;
                        UpdateSoftBodyVisuals(sb, isControlPressed, hoverIdx, dragIdx, targ);
                    }
                }
            }
            else
            {
                if (softBodies != null)
                {
                    foreach (var sb in softBodies)
                    {
                        if (sb != null)
                        {
                            UpdateSoftBodyVisuals(sb, false, -1, -1, Vector3.zero);
                        }
                    }
                }

                if (draggedSoftBody != null)
                {
                    draggedSoftBody = null;
                    draggedNodeIndex = -1;
                    wasMousePressed = false;
                }
            }
        }

        private void SetupSoftBodyVisuals(SoftBody sb)
        {
            if (sb == null || sb.solver?.nodeManager?.Nodes == null) return;

            GameObject container = new GameObject("NodeGrabberVisuals");
            container.transform.SetParent(sb.transform, false);
            container.hideFlags = HideFlags.DontSave;
            visualsContainers[sb] = container;

            List<GameObject> nodeVis = new List<GameObject>();
            int nodeCount = sb.solver.nodeManager.Nodes.Count;
            for (int i = 0; i < nodeCount; i++)
            {
                GameObject nodeVisual = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                nodeVisual.transform.SetParent(container.transform, false);
                nodeVisual.hideFlags = HideFlags.DontSave;
#if UNITY_EDITOR
                DestroyImmediate(nodeVisual.GetComponent<Collider>());
#else
                Destroy(nodeVisual.GetComponent<Collider>());
#endif
                Material mat = new Material(Shader.Find("Sprites/Default"));
                nodeVisual.GetComponent<Renderer>().material = mat;
                nodeVis.Add(nodeVisual);
            }
            nodeVisualsDict[sb] = nodeVis;

            GameObject dragLineObj = new GameObject("DragLine");
            dragLineObj.transform.SetParent(container.transform, false);
            LineRenderer lrDrag = dragLineObj.AddComponent<LineRenderer>();
            lrDrag.enabled = false;
            lrDrag.material = new Material(Shader.Find("Sprites/Default"));
            lrDrag.positionCount = 2;
            lrDrag.useWorldSpace = true;
            lrDrag.startColor = Color.black;
            lrDrag.endColor = Color.black;
            lrDrag.startWidth = 0.05f;
            lrDrag.endWidth = 0.05f;
            dragLineRenderers[sb] = lrDrag;
        }

        private void UpdateSoftBodyVisuals(SoftBody sb, bool show, int hoverIdx, int dragIdx, Vector3 targPos)
        {
            if (!visualsContainers.TryGetValue(sb, out GameObject container) || container == null) return;

            if (!nodeVisualsDict.TryGetValue(sb, out var nodeVisList) || nodeVisList == null) return;
            var nodes = sb.solver.nodeManager.Nodes;
            if (nodes == null) return;
            int nodeCount = nodes.Count;

            int centerIdx = dragIdx >= 0 ? dragIdx : hoverIdx;
            bool hasCenter = centerIdx >= 0;

            // Update node visuals
            for (int i = 0; i < nodeCount; i++)
            {
                if (nodes[i] == null || i >= nodeVisList.Count || nodeVisList[i] == null) continue;

                GameObject nodeObj = nodeVisList[i];

                if (!show)
                {
                    nodeObj.SetActive(false);
                    continue;
                }

                Vector3 nodePos = nodes[i].position;
                float distance = Vector3.Distance(nodePos, attachedCamera.transform.position);
                float scaledSize = nodeVisualBaseSize * (1f + distance * nodeVisualDistanceScale);
                nodeObj.transform.position = nodePos;
                nodeObj.transform.localScale = Vector3.one * scaledSize * 0.2f;

                // Only show nodes within the radius of the hovered/dragged node
                bool isVisible = !hasCenter || Vector3.Distance(nodePos, nodes[centerIdx].position) <= nodeVisualShowRadius;
                nodeObj.SetActive(isVisible);

                if (isVisible)
                {
                    Renderer renderer = nodeObj.GetComponent<Renderer>();
                    if (renderer != null)
                    {
                        Color col = nodeColorNormal;
                        if (sb.solver.nodeManager.IsPinned[i])
                            col = nodeColorPinned;
                        else if (i == dragIdx)
                            col = nodeColorGrabbed;
                        else if (i == hoverIdx)
                            col = nodeColorHover;

                        renderer.material.color = col;
                    }

                    if (showGrabRadiusGizmo && (i == hoverIdx || i == dragIdx))
                    {
                        Debug.DrawRay(nodePos, Vector3.right * nodeGrabRadius, Color.white * 0.3f, 0f);
                        Debug.DrawRay(nodePos, Vector3.up * nodeGrabRadius, Color.white * 0.3f, 0f);
                        Debug.DrawRay(nodePos, Vector3.forward * nodeGrabRadius, Color.white * 0.3f, 0f);
                    }
                }
            }

            // Update drag line
            if (dragLineRenderers.TryGetValue(sb, out LineRenderer dlr))
            {
                if (show && dragIdx >= 0)
                {
                    dlr.enabled = true;
                    if (dlr.positionCount < 2)
                        dlr.positionCount = 2;
                    dlr.SetPosition(0, nodes[dragIdx].position);
                    dlr.SetPosition(1, targPos);
                }
                else
                {
                    dlr.enabled = false;
                }
            }
        }

        void OnDestroy()
        {
            foreach (var kvp in visualsContainers)
            {
                if (kvp.Value != null)
                {
#if UNITY_EDITOR
                    if (!Application.isPlaying)
                        DestroyImmediate(kvp.Value);
                    else
                        Destroy(kvp.Value);
#else
                    Destroy(kvp.Value);
#endif
                }
            }
            visualsContainers.Clear();
            nodeVisualsDict.Clear();
            dragLineRenderers.Clear();
        }

        private void HandleCursor()
        {
            if (Mouse.current != null && Mouse.current.rightButton.isPressed)
            {
                Cursor.lockState = CursorLockMode.Locked;
                Cursor.visible = false;
            }
            else
            {
                Cursor.lockState = CursorLockMode.None;
                Cursor.visible = true;
            }
        }

        private void HandleMovement()
        {
            if (Mouse.current == null || !Mouse.current.rightButton.isPressed)
                return;

            if (Keyboard.current == null)
                return;

            float moveX = 0f;
            float moveZ = 0f;
            float moveY = 0f;

            if (Keyboard.current.aKey.isPressed) moveX -= 1f;
            if (Keyboard.current.dKey.isPressed) moveX += 1f;
            if (Keyboard.current.wKey.isPressed) moveZ += 1f;
            if (Keyboard.current.sKey.isPressed) moveZ -= 1f;

            if (Keyboard.current.qKey.isPressed || Keyboard.current.cKey.isPressed)
                moveY = -1f;
            else if (Keyboard.current.eKey.isPressed || Keyboard.current.spaceKey.isPressed)
                moveY = 1f;

            bool isSprinting = Keyboard.current.leftShiftKey.isPressed || Keyboard.current.rightShiftKey.isPressed;
            float speed = cameraMoveSpeed * (isSprinting ? cameraSprintMultiplier : 1f);

            Vector3 moveDirection = new Vector3(moveX, moveY, moveZ).normalized;
            Vector3 localMovement = transform.TransformDirection(moveDirection) * speed * Time.deltaTime;
            transform.position += localMovement;
        }

        private void HandleRotation()
        {
            if (Mouse.current == null || !Mouse.current.rightButton.isPressed)
                return;

            Vector2 mouseDelta = Mouse.current.delta.ReadValue();
            float mouseX = mouseDelta.x * cameraMouseSensitivity * 0.1f;
            float mouseY = mouseDelta.y * cameraMouseSensitivity * 0.1f;

            yaw += mouseX;
            pitch -= mouseY;
            pitch = Mathf.Clamp(pitch, -90f, 90f);
            transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
        }

        private void RefreshSoftBodies()
        {
            SoftBody[] newSoftBodies = Object.FindObjectsByType<SoftBody>(FindObjectsSortMode.None);

            List<SoftBody> toRemove = new List<SoftBody>();
            foreach (var sb in softBodies ?? new SoftBody[0])
            {
                if (sb == null || System.Array.IndexOf(newSoftBodies, sb) < 0)
                {
                    toRemove.Add(sb);
                }
            }
            foreach (var sb in toRemove)
            {
                if (sb == null) continue;

                if (visualsContainers.TryGetValue(sb, out var container))
                {
#if UNITY_EDITOR
                    if (!Application.isPlaying)
                        DestroyImmediate(container);
                    else
                        Destroy(container);
#else
                    Destroy(container);
#endif
                    visualsContainers.Remove(sb);
                }
                nodeVisualsDict.Remove(sb);
                dragLineRenderers.Remove(sb);
            }

            softBodies = newSoftBodies;
        }

        public void AddSoftBody(SoftBody softBody)
        {
            if (softBody == null) return;

            var newSoftBodies = new SoftBody[(softBodies?.Length ?? 0) + 1];
            if (softBodies != null && softBodies.Length > 0)
            {
                System.Array.Copy(softBodies, newSoftBodies, softBodies.Length);
            }
            newSoftBodies[newSoftBodies.Length - 1] = softBody;
            softBodies = newSoftBodies;
        }

        public void RemoveSoftBody(SoftBody softBody)
        {
            if (softBody == null || softBodies == null) return;

            var softBodyList = new System.Collections.Generic.List<SoftBody>(softBodies);
            softBodyList.Remove(softBody);
            softBodies = softBodyList.ToArray();

            if (visualsContainers.ContainsKey(softBody))
            {
                if (visualsContainers[softBody] != null)
                {
#if UNITY_EDITOR
                    if (!Application.isPlaying)
                        DestroyImmediate(visualsContainers[softBody]);
                    else
                        Destroy(visualsContainers[softBody]);
#else
                    Destroy(visualsContainers[softBody]);
#endif
                }
                visualsContainers.Remove(softBody);
            }
            nodeVisualsDict.Remove(softBody);
            dragLineRenderers.Remove(softBody);
        }

        public void SetVisualizationEnabled(bool enabled)
        {
            enableNodeVisualization = enabled;
        }

        public float CameraMoveSpeed
        {
            get => cameraMoveSpeed;
            set => cameraMoveSpeed = Mathf.Clamp(value, 1f, 20f);
        }

        public float CameraSprintMultiplier
        {
            get => cameraSprintMultiplier;
            set => cameraSprintMultiplier = Mathf.Clamp(value, 1f, 5f);
        }

        public float CameraMouseSensitivity
        {
            get => cameraMouseSensitivity;
            set => cameraMouseSensitivity = Mathf.Clamp(value, 0.1f, 5f);
        }

        public float NodeVisualBaseSize
        {
            get => nodeVisualBaseSize;
            set => nodeVisualBaseSize = Mathf.Clamp(value, 0.1f, 5f);
        }

        public float NodeVisualDistanceScale
        {
            get => nodeVisualDistanceScale;
            set => nodeVisualDistanceScale = Mathf.Clamp(value, 0.1f, 5f);
        }

        public float NodeGrabForce
        {
            get => nodeGrabForce;
            set => nodeGrabForce = Mathf.Clamp(value, 1f, 10000f);
        }

        public float NodeGrabRadius
        {
            get => nodeGrabRadius;
            set => nodeGrabRadius = Mathf.Clamp(value, 0.1f, 2f);
        }

        public bool EnableNodeVisualization
        {
            get => enableNodeVisualization;
            set => enableNodeVisualization = value;
        }

        public Camera AttachedCamera => attachedCamera;

        public int SoftBodyCount => softBodies?.Length ?? 0;

        public void LogSoftBodyInfo()
        {
            if (softBodies == null)
            {
                Debug.Log("CameraVisualizerController: No soft bodies found");
                return;
            }

            Debug.Log($"CameraVisualizerController: {softBodies.Length} soft body(ies) found:");
            for (int i = 0; i < softBodies.Length; i++)
            {
                if (softBodies[i] != null)
                {
                    int nodeCount = softBodies[i].solver?.nodeManager?.Nodes?.Count ?? 0;
                    Debug.Log($"  [{i}] {softBodies[i].name} - {nodeCount} nodes - {(softBodies[i].enabled ? "Enabled" : "Disabled")}");
                }
                else
                {
                    Debug.Log($"  [{i}] NULL soft body");
                }
            }
        }

        void OnValidate()
        {
            cameraMoveSpeed = Mathf.Clamp(cameraMoveSpeed, 1f, 20f);
            cameraSprintMultiplier = Mathf.Clamp(cameraSprintMultiplier, 1f, 5f);
            cameraMouseSensitivity = Mathf.Clamp(cameraMouseSensitivity, 0.1f, 5f);
            nodeVisualBaseSize = Mathf.Clamp(nodeVisualBaseSize, 0.1f, 5f);
            nodeVisualDistanceScale = Mathf.Clamp(nodeVisualDistanceScale, 0.1f, 5f);
            nodeGrabForce = Mathf.Clamp(nodeGrabForce, 1f, 10000f);
            nodeGrabRadius = Mathf.Clamp(nodeGrabRadius, 0.1f, 2f);
        }

        public void FocusOnSoftBody(SoftBody softBody, float distance = 5f)
        {
            if (softBody == null || softBody.solver?.nodeManager?.Nodes == null) return;

            Vector3 center = Vector3.zero;
            int nodeCount = 0;

            foreach (var node in softBody.solver.nodeManager.Nodes)
            {
                if (node != null)
                {
                    center += node.position;
                    nodeCount++;
                }
            }

            if (nodeCount > 0)
            {
                center /= nodeCount;
                transform.position = center - transform.forward * distance;
                transform.LookAt(center);
                Vector3 eulerAngles = transform.eulerAngles;
                yaw = eulerAngles.y;
                pitch = eulerAngles.x;
                if (pitch > 180f)
                    pitch -= 360f;
            }
        }

        public void ToggleAllVisualizations()
        {
            enableNodeVisualization = !enableNodeVisualization;
        }

        void OnGUI()
        {
            if (draggedSoftBody != null && draggedNodeIndex >= 0)
            {
                float kg = lastGrabForceMag / 9.81f;
                string labelText = $"Grab Force: {lastGrabForceMag:F1} N ({kg:F1} kg)";
                Rect rect = new Rect(Screen.width / 2 - 100, Screen.height - 50, 200, 20);
                GUI.Label(rect, labelText);
            }
        }
    }
}