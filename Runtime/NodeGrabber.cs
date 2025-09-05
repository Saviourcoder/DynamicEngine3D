/* DynamicEngine3D - Unified Node Grabbing System (URP Compatible)
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/

using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    public class NodeGrabber : MonoBehaviour
    {
        [Header("Node Grabbing Settings")]
        [SerializeField, Range(0.1f, 5f)] public float nodeVisualSize = 0.3f;
        [SerializeField, Range(1f, 100f)] private float grabForce = 50f;
        [SerializeField, Range(0.1f, 2f)] public float grabRadius = 0.5f;
        [SerializeField] public bool enableInPlayMode = true;
        [SerializeField] public bool enableInEditMode = true;
        
        [Header("Visual Settings")]
        [SerializeField] public Color normalNodeColor = new Color(1f, 0.5f, 0f); // BeamNG-style orange
        [SerializeField] public Color hoverNodeColor = Color.yellow;
        [SerializeField] public Color grabbedNodeColor = Color.red;
        [SerializeField] public bool showGrabRadius = true;
        
        [Header("Rendering Settings")]
        [SerializeField] private Material nodeMaterial;
        [SerializeField] private Material beamMaterial;
        
        // Core logic variables
        public SoftBody softBody;
        private Camera sceneCamera;
        public int draggedNodeIndex = -1;
        private Vector3 dragOffset;
        public Vector3 targetPosition;
        public bool isControlPressed = false;
        public int hoveredNodeIndex = -1;
        private bool wasMousePressed = false;
        
        // URP-compatible rendering variables
        private List<GameObject> nodeVisuals = new List<GameObject>();
        private List<LineRenderer> beamVisuals = new List<LineRenderer>();
        private LineRenderer dragLineRenderer;
        private GameObject targetVisual;
        private GameObject visualsContainer;
        
        void Awake()
        {
            softBody = GetComponent<SoftBody>();
            if (softBody == null)
            {
                Debug.LogError("NodeGrabber requires a SoftBody component!", this);
                enabled = false;
                return;
            }

            SetupVisualization();
            Debug.Log("NodeGrabber: Initialized with unified URP-compatible rendering");
        }
        
        void Start()
        {
            CreateDefaultMaterials();
        }
        
        void Update()
        {
            // Check if we should be active
            bool shouldBeActive = (Application.isPlaying && enableInPlayMode) || 
                                 (!Application.isPlaying && enableInEditMode);
            
            if (!shouldBeActive || softBody?.solver?.nodeManager == null)
            {
                HideVisualization();
                return;
            }
                
            HandleInput();
            UpdateVisualization();
        }
        
        void FixedUpdate()
        {
            if (draggedNodeIndex >= 0 && Application.isPlaying)
            {
                ApplyGrabForce();
            }
        }
        
        #region Input Handling
        
        private void HandleInput()
        {
            // Detect Control key (also check for Command on Mac)
            isControlPressed = Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl) ||
                              Input.GetKey(KeyCode.LeftCommand) || Input.GetKey(KeyCode.RightCommand);
            
            if (!isControlPressed)
            {
                ReleaseDrag();
                hoveredNodeIndex = -1;
                return;
            }
            
            // Get appropriate camera
            sceneCamera = GetActiveCamera();
            if (sceneCamera == null) return;
            
            // Handle mouse input with better event detection
            Event currentEvent = Event.current;
            bool mousePressed = false;
            bool mouseHeld = false;
            bool mouseReleased = false;
            
#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                // In edit mode, use Event system for better scene view integration
                if (currentEvent != null)
                {
                    mousePressed = currentEvent.type == EventType.MouseDown && currentEvent.button == 0;
                    mouseReleased = currentEvent.type == EventType.MouseUp && currentEvent.button == 0;
                    mouseHeld = currentEvent.type == EventType.MouseDrag && currentEvent.button == 0;
                }
            }
            else
#endif
            {
                // In play mode, use Input system
                mousePressed = Input.GetMouseButtonDown(0);
                mouseHeld = Input.GetMouseButton(0);
                mouseReleased = Input.GetMouseButtonUp(0);
            }
            
            if (mousePressed && !wasMousePressed)
            {
                StartDrag();
            }
            else if (mouseReleased && wasMousePressed)
            {
                ReleaseDrag();
            }
            else if (mouseHeld && draggedNodeIndex >= 0)
            {
                UpdateDrag();
            }
            else if (!mouseHeld)
            {
                UpdateHover();
            }
            
            wasMousePressed = mouseHeld;
        }
        
        private Camera GetActiveCamera()
        {
#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                // In edit mode, use scene view camera
                if (SceneView.lastActiveSceneView?.camera != null)
                    return SceneView.lastActiveSceneView.camera;
            }
#endif
            // In play mode, prioritize the camera that was set by CameraVisualizerController
            // If we have a sceneCamera from recent DrawNodeVisualization call, use it
            if (sceneCamera != null)
                return sceneCamera;
                
            // Fallback: Try to find CameraVisualizerController camera first
            var cameraController = Object.FindFirstObjectByType<CameraVisualizerController>();
            if (cameraController != null && cameraController.AttachedCamera != null)
                return cameraController.AttachedCamera;
            
            // Final fallback: use main camera or current camera
            if (Camera.main != null)
                return Camera.main;
                
            if (Camera.current != null)
                return Camera.current;
            
            return Object.FindFirstObjectByType<Camera>();
        }
        
        private void UpdateHover()
        {
            hoveredNodeIndex = GetNodeUnderMouse();
        }
        
        private void StartDrag()
        {
            int nodeIndex = GetNodeUnderMouse();
            if (nodeIndex >= 0)
            {
                draggedNodeIndex = nodeIndex;
                
                Vector3 nodeWorldPos = softBody.solver.nodeManager.Nodes[nodeIndex].position;
                Vector3 mouseWorldPos = GetMouseWorldPosition(nodeWorldPos);
                dragOffset = nodeWorldPos - mouseWorldPos;
                targetPosition = nodeWorldPos;
            }
        }
        
        private void UpdateDrag()
        {
            if (draggedNodeIndex >= 0)
            {
                Vector3 nodeWorldPos = softBody.solver.nodeManager.Nodes[draggedNodeIndex].position;
                targetPosition = GetMouseWorldPosition(nodeWorldPos) + dragOffset;
            }
        }
        
        private void ReleaseDrag()
        {
            if (draggedNodeIndex >= 0)
            {
                draggedNodeIndex = -1;
            }
        }
        
        private int GetNodeUnderMouse()
        {
            if (sceneCamera == null || softBody?.solver?.nodeManager?.Nodes == null)
                return -1;
            
            Vector3 mousePos;
            
#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                // In edit mode, use Event system for scene view mouse position
                Event currentEvent = Event.current;
                if (currentEvent != null)
                {
                    mousePos = currentEvent.mousePosition;
                    // Convert from scene view coordinates to screen coordinates
                    mousePos.y = sceneCamera.pixelHeight - mousePos.y;
                }
                else
                {
                    mousePos = Input.mousePosition;
                }
            }
            else
#endif
            {
                // In play mode, handle cursor lock state properly
                if (Cursor.lockState == CursorLockMode.Locked)
                {
                    // When cursor is locked, use screen center
                    mousePos = new Vector3(Screen.width / 2f, Screen.height / 2f, 0f);
                }
                else
                {
                    // Normal mouse position
                    mousePos = Input.mousePosition;
                }
            }
            
            Ray ray = sceneCamera.ScreenPointToRay(mousePos);
            
            float closestDistance = float.MaxValue;
            int closestNodeIndex = -1;
            
            for (int i = 0; i < softBody.solver.nodeManager.Nodes.Count; i++)
            {
                if (softBody.solver.nodeManager.Nodes[i] == null)
                    continue;
                
                Vector3 nodeWorldPos = softBody.solver.nodeManager.Nodes[i].position;
                
                // Calculate closest point on ray to node
                Vector3 rayToNode = nodeWorldPos - ray.origin;
                float projectionLength = Vector3.Dot(rayToNode, ray.direction);
                Vector3 closestPointOnRay = ray.origin + ray.direction * projectionLength;
                
                float distanceToRay = Vector3.Distance(nodeWorldPos, closestPointOnRay);
                
                // Check if node is within grab radius and in front of camera
                if (projectionLength > 0 && distanceToRay < grabRadius)
                {
                    float totalDistance = distanceToRay + Vector3.Distance(ray.origin, nodeWorldPos) * 0.001f; // Small depth bias
                    
                    if (totalDistance < closestDistance)
                    {
                        closestDistance = totalDistance;
                        closestNodeIndex = i;
                    }
                }
            }
            
            return closestNodeIndex;
        }
        
        private Vector3 GetMouseWorldPosition(Vector3 referenceWorldPos)
        {
            if (sceneCamera == null)
                return referenceWorldPos;
            
            Vector3 mousePos;
            
#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                Event currentEvent = Event.current;
                if (currentEvent != null)
                {
                    mousePos = currentEvent.mousePosition;
                    mousePos.y = sceneCamera.pixelHeight - mousePos.y;
                }
                else
                {
                    mousePos = Input.mousePosition;
                }
            }
            else
#endif
            {
                // In play mode, handle cursor lock state properly
                if (Cursor.lockState == CursorLockMode.Locked)
                {
                    // When cursor is locked, use screen center
                    mousePos = new Vector3(Screen.width / 2f, Screen.height / 2f, 0f);
                }
                else
                {
                    // Normal mouse position
                    mousePos = Input.mousePosition;
                }
            }
            
            float distanceFromCamera = Vector3.Distance(sceneCamera.transform.position, referenceWorldPos);
            Vector3 worldPos = sceneCamera.ScreenToWorldPoint(new Vector3(mousePos.x, mousePos.y, distanceFromCamera));
            return worldPos;
        }
        
        private void ApplyGrabForce()
        {
            if (draggedNodeIndex < 0 || softBody?.solver == null)
                return;
            
            Vector3 currentPos = softBody.solver.nodeManager.Nodes[draggedNodeIndex].position;
            Vector3 forceDirection = (targetPosition - currentPos);
            Vector3 force = forceDirection * grabForce;
            
            // Apply force to the solver
            softBody.solver.ApplyForceToNode(draggedNodeIndex, force);
        }
        
        #endregion
        
        #region URP-Compatible Rendering
        
        private void SetupVisualization()
        {
            // Create container for all visuals
            visualsContainer = new GameObject("NodeGrabberVisuals");
            visualsContainer.transform.SetParent(transform);
            
            // Create drag line renderer
            GameObject dragLineObj = new GameObject("DragLine");
            dragLineObj.transform.SetParent(visualsContainer.transform);
            dragLineRenderer = dragLineObj.AddComponent<LineRenderer>();
            dragLineRenderer.startWidth = 0.02f;
            dragLineRenderer.endWidth = 0.02f;
            dragLineRenderer.positionCount = 2;
            dragLineRenderer.enabled = false;
            
            // Create target visual
            targetVisual = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            targetVisual.transform.SetParent(visualsContainer.transform);
            targetVisual.name = "TargetVisual";
            targetVisual.SetActive(false);
            
            // Remove collider
            var targetCollider = targetVisual.GetComponent<Collider>();
            if (targetCollider != null)
                DestroyImmediate(targetCollider);
        }
        
        private void CreateDefaultMaterials()
        {
            if (nodeMaterial == null)
            {
                nodeMaterial = CreateUnlitMaterial(Color.white);
            }
            if (beamMaterial == null)
            {
                beamMaterial = CreateUnlitMaterial(Color.black);
            }
            
            // Apply materials
            if (dragLineRenderer != null)
                dragLineRenderer.material = beamMaterial;
                
            if (targetVisual != null)
            {
                var targetRenderer = targetVisual.GetComponent<Renderer>();
                if (targetRenderer != null)
                    targetRenderer.material = CreateUnlitMaterial(Color.white);
            }
        }
        
        private Material CreateUnlitMaterial(Color color)
        {
            Shader shader = Shader.Find("Unlit/Color");
            if (shader == null)
                shader = Shader.Find("Legacy Shaders/Unlit/Color");
            if (shader == null)
                shader = Shader.Find("Standard");
                
            Material mat = new Material(shader);
            mat.color = color;
            return mat;
        }
        
        private void UpdateVisualization()
        {
            if (!isControlPressed || softBody?.solver?.nodeManager?.Nodes == null)
            {
                HideVisualization();
                return;
            }
            
            var nodes = softBody.solver.nodeManager.Nodes;
            var beams = softBody.solver.beams;
            
            // Update node visuals
            UpdateNodeVisuals(nodes);
            
            // Update beam visuals
            UpdateBeamVisuals(beams, nodes);
            
            // Update drag line and target
            UpdateDragVisualization();
        }
        
        private void UpdateNodeVisuals(IReadOnlyList<Transform> nodes)
        {
            // Ensure we have enough node visuals
            while (nodeVisuals.Count < nodes.Count)
            {
                GameObject nodeObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                nodeObj.transform.SetParent(visualsContainer.transform);
                nodeObj.name = $"NodeVisual_{nodeVisuals.Count}";
                
                var renderer = nodeObj.GetComponent<Renderer>();
                renderer.material = nodeMaterial;
                
                // Remove collider
                var collider = nodeObj.GetComponent<Collider>();
                if (collider != null)
                    DestroyImmediate(collider);
                
                nodeVisuals.Add(nodeObj);
            }
            
            // Update positions and colors
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] == null || i >= nodeVisuals.Count)
                    continue;
                    
                var nodeVisual = nodeVisuals[i];
                nodeVisual.SetActive(true);
                nodeVisual.transform.position = nodes[i].position;
                nodeVisual.transform.localScale = Vector3.one * nodeVisualSize;
                
                // Update color based on state
                var renderer = nodeVisual.GetComponent<Renderer>();
                Color color = normalNodeColor;
                
                if (i == draggedNodeIndex)
                    color = grabbedNodeColor;
                else if (i == hoveredNodeIndex)
                    color = hoverNodeColor;
                
                renderer.material.color = color;
            }
            
            // Hide extra visuals
            for (int i = nodes.Count; i < nodeVisuals.Count; i++)
            {
                nodeVisuals[i].SetActive(false);
            }
        }
        
        private void UpdateBeamVisuals(IReadOnlyList<DynamicEngine.Beam> beams, IReadOnlyList<Transform> nodes)
        {
            // Ensure we have enough beam visuals
            while (beamVisuals.Count < beams.Count)
            {
                GameObject beamObj = new GameObject($"BeamVisual_{beamVisuals.Count}");
                beamObj.transform.SetParent(visualsContainer.transform);
                
                var lineRenderer = beamObj.AddComponent<LineRenderer>();
                lineRenderer.material = beamMaterial;
                lineRenderer.startWidth = 0.01f;
                lineRenderer.endWidth = 0.01f;
                lineRenderer.positionCount = 2;
                
                beamVisuals.Add(lineRenderer);
            }
            
            // Update beam positions
            for (int i = 0; i < beams.Count; i++)
            {
                if (i >= beamVisuals.Count)
                    break;
                    
                var beam = beams[i];
                var lineRenderer = beamVisuals[i];
                
                if (beam.nodeA >= 0 && beam.nodeA < nodes.Count &&
                    beam.nodeB >= 0 && beam.nodeB < nodes.Count &&
                    nodes[beam.nodeA] != null && nodes[beam.nodeB] != null)
                {
                    lineRenderer.enabled = true;
                    lineRenderer.SetPosition(0, nodes[beam.nodeA].position);
                    lineRenderer.SetPosition(1, nodes[beam.nodeB].position);
                }
                else
                {
                    lineRenderer.enabled = false;
                }
            }
            
            // Hide extra visuals
            for (int i = beams.Count; i < beamVisuals.Count; i++)
            {
                beamVisuals[i].enabled = false;
            }
        }
        
        private void UpdateDragVisualization()
        {
            if (draggedNodeIndex >= 0)
            {
                var nodes = softBody.solver.nodeManager.Nodes;
                if (draggedNodeIndex < nodes.Count && nodes[draggedNodeIndex] != null)
                {
                    Vector3 nodePos = nodes[draggedNodeIndex].position;
                    
                    // Update drag line
                    dragLineRenderer.enabled = true;
                    dragLineRenderer.material.color = grabbedNodeColor;
                    dragLineRenderer.SetPosition(0, nodePos);
                    dragLineRenderer.SetPosition(1, targetPosition);
                    
                    // Update target visual
                    targetVisual.SetActive(true);
                    targetVisual.transform.position = targetPosition;
                    targetVisual.transform.localScale = Vector3.one * nodeVisualSize * 0.5f;
                }
            }
            else
            {
                dragLineRenderer.enabled = false;
                targetVisual.SetActive(false);
            }
        }
        
        private void HideVisualization()
        {
            foreach (var nodeVisual in nodeVisuals)
            {
                if (nodeVisual != null)
                    nodeVisual.SetActive(false);
            }
            
            foreach (var beamVisual in beamVisuals)
            {
                if (beamVisual != null)
                    beamVisual.enabled = false;
            }
            
            if (dragLineRenderer != null)
                dragLineRenderer.enabled = false;
                
            if (targetVisual != null)
                targetVisual.SetActive(false);
        }
        
        #endregion
        
        #region Public API & Methods for CameraVisualizerController
        
        public void DrawNodeVisualization(Camera cam)
        {
            // Set and cache the camera for this frame and future mouse detection
            sceneCamera = cam;
            
            // Force update visualization for this camera
            if (isControlPressed && softBody?.solver?.nodeManager?.Nodes != null)
            {
                UpdateVisualization();
            }
        }
        
        public void SetCamera(Camera cam)
        {
            sceneCamera = cam;
        }
        
        #endregion
        
        void OnDrawGizmos()
        {
            if (!isControlPressed || !Application.isPlaying)
                return;
                
            // Draw instruction text in scene view
#if UNITY_EDITOR
            if (sceneCamera != null)
            {
                Vector3 textPos = sceneCamera.transform.position + sceneCamera.transform.forward * 2f;
                Handles.Label(textPos, "Hold CTRL and drag nodes to interact");
            }
#endif
        }
        
        void OnDestroy()
        {
            // Clean up created materials
            if (nodeMaterial != null)
                DestroyImmediate(nodeMaterial);
            if (beamMaterial != null)
                DestroyImmediate(beamMaterial);
        }

        // Public properties for inspector and external access
        public float NodeVisualSize
        {
            get => nodeVisualSize;
            set => nodeVisualSize = Mathf.Clamp(value, 0.1f, 5f);
        }
        
        public float GrabForce
        {
            get => grabForce;
            set => grabForce = Mathf.Clamp(value, 1f, 1000f);
        }
        
        public float GrabRadius
        {
            get => grabRadius;
            set => grabRadius = Mathf.Clamp(value, 0.1f, 2f);
        }
    }
}