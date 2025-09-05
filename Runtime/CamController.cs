/* DynamicEngine3D - Combined Camera Controller & Soft Body Visualizer
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/

using UnityEngine;

namespace DynamicEngine
{
    [RequireComponent(typeof(Camera))]
    public class CameraVisualizerController : MonoBehaviour
    {
        [Header("Movement Settings")]
        [SerializeField, Range(1f, 20f)] private float moveSpeed = 5f;
        [SerializeField, Range(1f, 5f)] private float sprintMultiplier = 2f;
        [SerializeField, Range(0.1f, 5f)] private float mouseSensitivity = 2f;
        
        [Header("Visualization Settings")]
        [SerializeField] private bool enableSoftBodyVisualization = true;
        [SerializeField] private bool findGrabbersOnStart = true;
        [SerializeField] private bool refreshGrabbersEveryFrame = false;
        
        // Movement variables
        private float yaw = 0f;
        private float pitch = 0f;
        
        // Visualization variables
        private NodeGrabber[] grabbers;
        private Camera attachedCamera;
        private float lastRefreshTime = 0f;
        private const float REFRESH_INTERVAL = 1f; // Refresh every second if needed
        
        void Awake()
        {
            attachedCamera = GetComponent<Camera>();
        }
        
        void Start()
        {
            if (findGrabbersOnStart)
            {
                RefreshNodeGrabbers();
            }
            
            // Initialize rotation based on current transform rotation
            Vector3 eulerAngles = transform.eulerAngles;
            yaw = eulerAngles.y;
            pitch = eulerAngles.x;
            
            // Handle negative pitch values
            if (pitch > 180f)
                pitch -= 360f;
        }

        void Update()
        {
            HandleCursor();
            HandleMovement();
            HandleRotation();
            
            // Always inform grabbers about the current camera for mouse detection
            if (grabbers != null && attachedCamera != null)
            {
                foreach (var grabber in grabbers)
                {
                    if (grabber != null && grabber.enabled && grabber.gameObject.activeInHierarchy)
                    {
                        grabber.SetCamera(attachedCamera);
                    }
                }
            }
            
            // Refresh grabbers periodically if needed
            if (refreshGrabbersEveryFrame || 
                (findGrabbersOnStart && Time.time - lastRefreshTime > REFRESH_INTERVAL))
            {
                RefreshNodeGrabbers();
                lastRefreshTime = Time.time;
            }
        }
        
        void OnPostRender()
        {
            if (!enableSoftBodyVisualization || grabbers == null || attachedCamera == null)
                return;
                
            // Render soft body visualizations for all active grabbers
            foreach (var grabber in grabbers)
            {
                if (grabber != null && grabber.enabled && grabber.gameObject.activeInHierarchy)
                {
                    grabber.DrawNodeVisualization(attachedCamera);
                }
            }
        }
        
        private void HandleCursor()
        {
            // Manage cursor state based on right mouse button
            if (Input.GetMouseButton(1))
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
            // Only process movement if right mouse button is held
            if (!Input.GetMouseButton(1))
                return;

            // Get input axes
            float moveX = Input.GetAxisRaw("Horizontal"); // A/D
            float moveZ = Input.GetAxisRaw("Vertical");   // W/S
            float moveY = 0f;
            
            // Handle vertical movement (Q/E or Space/C)
            if (Input.GetKey(KeyCode.Q) || Input.GetKey(KeyCode.C))
                moveY = -1f;
            else if (Input.GetKey(KeyCode.E) || Input.GetKey(KeyCode.Space))
                moveY = 1f;

            // Apply sprint multiplier
            float speed = moveSpeed * (Input.GetKey(KeyCode.LeftShift) ? sprintMultiplier : 1f);

            // Calculate movement vector in local space
            Vector3 moveDirection = new Vector3(moveX, moveY, moveZ).normalized;
            Vector3 localMovement = transform.TransformDirection(moveDirection) * speed * Time.deltaTime;

            // Apply movement
            transform.position += localMovement;
        }

        private void HandleRotation()
        {
            // Only process rotation if right mouse button is held
            if (!Input.GetMouseButton(1))
                return;

            // Get mouse input
            float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity;
            float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity;

            // Update yaw (horizontal rotation) and pitch (vertical rotation)
            yaw += mouseX;
            pitch -= mouseY; // Invert Y for natural look
            pitch = Mathf.Clamp(pitch, -90f, 90f); // Limit pitch to avoid flipping

            // Apply rotation
            transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
        }
        
        private void RefreshNodeGrabbers()
        {
            grabbers = Object.FindObjectsByType<NodeGrabber>(FindObjectsSortMode.None);
        }
        
        public void AddNodeGrabber(NodeGrabber grabber)
        {
            if (grabber == null) return;
            
            // Create new array with additional grabber
            var newGrabbers = new NodeGrabber[(grabbers?.Length ?? 0) + 1];
            if (grabbers != null && grabbers.Length > 0)
            {
                System.Array.Copy(grabbers, newGrabbers, grabbers.Length);
            }
            newGrabbers[newGrabbers.Length - 1] = grabber;
            grabbers = newGrabbers;
        }
        
        public void RemoveNodeGrabber(NodeGrabber grabber)
        {
            if (grabber == null || grabbers == null) return;
            
            var grabberList = new System.Collections.Generic.List<NodeGrabber>(grabbers);
            grabberList.Remove(grabber);
            grabbers = grabberList.ToArray();
        }
        
        public void SetVisualizationEnabled(bool enabled)
        {
            enableSoftBodyVisualization = enabled;
        }

        public float MoveSpeed
        {
            get => moveSpeed;
            set => moveSpeed = Mathf.Clamp(value, 1f, 20f);
        }

        public float SprintMultiplier
        {
            get => sprintMultiplier;
            set => sprintMultiplier = Mathf.Clamp(value, 1f, 5f);
        }

        public float MouseSensitivity
        {
            get => mouseSensitivity;
            set => mouseSensitivity = Mathf.Clamp(value, 0.1f, 5f);
        }
        
        public bool EnableSoftBodyVisualization
        {
            get => enableSoftBodyVisualization;
            set => enableSoftBodyVisualization = value;
        }
        
        public Camera AttachedCamera => attachedCamera;
        
        public int GrabberCount => grabbers?.Length ?? 0;
        
        // Utility methods for debugging
        public void LogGrabberInfo()
        {
            if (grabbers == null)
            {
                Debug.Log("CameraVisualizerController: No grabbers found");
                return;
            }
            
            Debug.Log($"CameraVisualizerController: {grabbers.Length} grabber(s) found:");
            for (int i = 0; i < grabbers.Length; i++)
            {
                if (grabbers[i] != null)
                {
                    var softBody = grabbers[i].GetComponent<SoftBody>();
                    int nodeCount = softBody?.solver?.nodeManager?.Nodes?.Count ?? 0;
                    Debug.Log($"  [{i}] {grabbers[i].name} - {nodeCount} nodes - {(grabbers[i].enabled ? "Enabled" : "Disabled")}");
                }
                else
                {
                    Debug.Log($"  [{i}] NULL grabber");
                }
            }
        }
        
        void OnValidate()
        {
            // Clamp values when changed in inspector
            moveSpeed = Mathf.Clamp(moveSpeed, 1f, 20f);
            sprintMultiplier = Mathf.Clamp(sprintMultiplier, 1f, 5f);
            mouseSensitivity = Mathf.Clamp(mouseSensitivity, 0.1f, 5f);
        }
    }
}