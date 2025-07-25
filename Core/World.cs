/* DynamicEngine3D - Soft Body Management System
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
using System;
using System.Linq;

namespace DynamicEngine
{
    /// <summary>
    /// Represents a registered soft body with unique identification
    /// </summary>
    [System.Serializable]
    public class SoftBodyComponent : MonoBehaviour
    {
        [Header("Soft Body Identity")]
        [SerializeField] private string softBodyID = "";
        [SerializeField] private string softBodyName = "Unnamed Soft Body";
        [SerializeField] private SoftBodyType bodyType = SoftBodyType.Generic;
        
        [Header("Soft Body References")]
        public Solver solver;
        public NodeManager nodeManager;
        public MeshDeformer meshDeformer;
        public MaterialProps materialProps;
        
        [Header("Soft Body Properties")]
        public float nodeRadius = 0.1f;
        public float influenceRadius = 0.2f;
        public bool isActive = true;
        public bool isVisible = true;
        
        [Header("Debug Info")]
        [SerializeField] private float creationTime;
        [SerializeField] private int nodeCount;
        [SerializeField] private int beamCount;
        
        // Events
        public System.Action<SoftBodyComponent> OnSoftBodyDestroyed;
        public System.Action<SoftBodyComponent> OnSoftBodyModified;
        
        public string SoftBodyID => softBodyID;
        public string SoftBodyName => softBodyName;
        public SoftBodyType BodyType => bodyType;
        public float CreationTime => creationTime;
        public int NodeCount => nodeCount;
        public int BeamCount => beamCount;
        
        private void Awake()
        {
            creationTime = Time.time;
            
            // Auto-find components if not assigned
            if (solver == null)
            {
                // Try to find solver from existing components
                var nodeLink = GetComponent<NodeLinkEditor>();
                if (nodeLink != null)
                {
                    solver = nodeLink.GetSolver();
                    if (solver != null)
                    {
                        nodeManager = solver.nodeManager;
                        meshDeformer = solver.meshDeformer;
                        materialProps = solver.materialProps;
                    }
                }
            }
        }
        
        private void Start()
        {
            // Register with the manager
            if (string.IsNullOrEmpty(softBodyID))
            {
                SoftBodyManager.Instance.RegisterSoftBody(this);
            }
            
            UpdateDebugInfo();
        }
        
        private void Update()
        {
            if (Time.frameCount % 60 == 0) // Update every 60 frames
            {
                UpdateDebugInfo();
            }
        }
        
        private void OnDestroy()
        {
            OnSoftBodyDestroyed?.Invoke(this);
            SoftBodyManager.Instance.UnregisterSoftBody(this);
        }
        
        internal void SetID(string id)
        {
            softBodyID = id;
        }
        
        public void SetName(string name)
        {
            softBodyName = name;
            OnSoftBodyModified?.Invoke(this);
        }
        
        public void SetType(SoftBodyType type)
        {
            bodyType = type;
            OnSoftBodyModified?.Invoke(this);
        }
        
        public void SetSolver(Solver newSolver)
        {
            solver = newSolver;
            if (solver != null)
            {
                nodeManager = solver.nodeManager;
                meshDeformer = solver.meshDeformer;
                materialProps = solver.materialProps;
            }
            OnSoftBodyModified?.Invoke(this);
        }
        
        private void UpdateDebugInfo()
        {
            if (solver != null && nodeManager != null)
            {
                nodeCount = nodeManager.Nodes?.Count ?? 0;
                beamCount = solver.beams?.Count ?? 0;
            }
        }
        
        public Vector3 GetCenterPosition()
        {
            if (nodeManager?.Nodes == null || nodeManager.Nodes.Count == 0)
                return transform.position;
                
            Vector3 center = Vector3.zero;
            int validNodes = 0;
            
            foreach (var node in nodeManager.Nodes)
            {
                if (node != null)
                {
                    center += node.position;
                    validNodes++;
                }
            }
            
            return validNodes > 0 ? center / validNodes : transform.position;
        }
        
        public Bounds GetBounds()
        {
            if (nodeManager?.Nodes == null || nodeManager.Nodes.Count == 0)
                return new Bounds(transform.position, Vector3.one);
                
            Vector3 min = Vector3.positiveInfinity;
            Vector3 max = Vector3.negativeInfinity;
            
            foreach (var node in nodeManager.Nodes)
            {
                if (node != null)
                {
                    Vector3 pos = node.position;
                    min = Vector3.Min(min, pos);
                    max = Vector3.Max(max, pos);
                }
            }
            
            Vector3 center = (min + max) * 0.5f;
            Vector3 size = max - min;
            return new Bounds(center, size);
        }
        
        public Transform[] GetNodes()
        {
            if (nodeManager?.Nodes == null) return new Transform[0];
            var result = new Transform[nodeManager.Nodes.Count];
            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                result[i] = nodeManager.Nodes[i];
            }
            return result;
        }
        
        public void SetActive(bool active)
        {
            isActive = active;
            gameObject.SetActive(active);
        }
        
        public void SetVisible(bool visible)
        {
            isVisible = visible;
            var renderers = GetComponentsInChildren<Renderer>();
            foreach (var renderer in renderers)
            {
                renderer.enabled = visible;
            }
        }
    }
    
    /// <summary>
    /// Types of soft bodies for categorization
    /// </summary>
    public enum SoftBodyType
    {
        Generic,
        Vehicle,
        Character,
        Environment,
        Projectile,
        Structure,
        Fabric,
        Liquid,
        Custom
    }
    
    /// <summary>
    /// Singleton manager for all soft bodies in the scene
    /// </summary>
    public class SoftBodyManager : MonoBehaviour
    {
        private static SoftBodyManager instance;
        public static SoftBodyManager Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = FindFirstObjectByType<SoftBodyManager>();
                    if (instance == null)
                    {
                        GameObject go = new GameObject("SoftBodyManager");
                        instance = go.AddComponent<SoftBodyManager>();
                        DontDestroyOnLoad(go);
                    }
                }
                return instance;
            }
        }
        
        [Header("Manager Settings")]
        public bool debugMode = false;
        public bool autoGenerateIDs = true;
        public string idPrefix = "SB_";
        
        [Header("Statistics")]
        [SerializeField] private int totalSoftBodies = 0;
        [SerializeField] private int activeSoftBodies = 0;
        
        // Storage
        private Dictionary<string, SoftBodyComponent> softBodies = new Dictionary<string, SoftBodyComponent>();
        private Dictionary<SoftBodyType, List<SoftBodyComponent>> softBodiesByType = new Dictionary<SoftBodyType, List<SoftBodyComponent>>();
        private int nextID = 1;
        
        // Events
        public static event System.Action<SoftBodyComponent> OnSoftBodyRegistered;
        public static event System.Action<SoftBodyComponent> OnSoftBodyUnregistered;
        public static event System.Action<SoftBodyComponent> OnSoftBodyModified;
        
        private void Awake()
        {
            if (instance == null)
            {
                instance = this;
                DontDestroyOnLoad(gameObject);
                InitializeManager();
            }
            else if (instance != this)
            {
                Destroy(gameObject);
            }
        }
        
        private void Start()
        {
            // Find and register existing soft bodies
            RegisterExistingSoftBodies();
        }
        
        private void Update()
        {
            if (Time.frameCount % 120 == 0) // Update every 2 seconds
            {
                UpdateStatistics();
            }
        }
        
        private void InitializeManager()
        {
            // Initialize type dictionaries
            foreach (SoftBodyType type in System.Enum.GetValues(typeof(SoftBodyType)))
            {
                softBodiesByType[type] = new List<SoftBodyComponent>();
            }
            
            if (debugMode)
            {
                Debug.Log("SoftBodyManager initialized");
            }
        }
        
        private void RegisterExistingSoftBodies()
        {
            var existingSoftBodies = FindObjectsByType<SoftBodyComponent>(FindObjectsSortMode.None);
            foreach (var softBody in existingSoftBodies)
            {
                if (string.IsNullOrEmpty(softBody.SoftBodyID))
                {
                    RegisterSoftBody(softBody);
                }
            }
            
            if (debugMode)
            {
                Debug.Log($"Registered {existingSoftBodies.Length} existing soft bodies");
            }
        }
        
        public string RegisterSoftBody(SoftBodyComponent softBody)
        {
            if (softBody == null) return null;
            
            string id = softBody.SoftBodyID;
            
            // Generate new ID if needed
            if (string.IsNullOrEmpty(id) || softBodies.ContainsKey(id))
            {
                id = GenerateUniqueID();
                softBody.SetID(id);
            }
            
            // Register in main dictionary
            softBodies[id] = softBody;
            
            // Register by type
            if (!softBodiesByType[softBody.BodyType].Contains(softBody))
            {
                softBodiesByType[softBody.BodyType].Add(softBody);
            }
            
            // Subscribe to events
            softBody.OnSoftBodyDestroyed += HandleSoftBodyDestroyed;
            softBody.OnSoftBodyModified += HandleSoftBodyModified;
            
            totalSoftBodies = softBodies.Count;
            
            OnSoftBodyRegistered?.Invoke(softBody);
            
            if (debugMode)
            {
                Debug.Log($"Registered soft body: {id} ({softBody.SoftBodyName})");
            }
            
            return id;
        }
        
        public void UnregisterSoftBody(SoftBodyComponent softBody)
        {
            if (softBody == null) return;
            
            string id = softBody.SoftBodyID;
            
            if (!string.IsNullOrEmpty(id) && softBodies.ContainsKey(id))
            {
                softBodies.Remove(id);
                softBodiesByType[softBody.BodyType].Remove(softBody);
                
                totalSoftBodies = softBodies.Count;
                
                OnSoftBodyUnregistered?.Invoke(softBody);
                
                if (debugMode)
                {
                    Debug.Log($"Unregistered soft body: {id}");
                }
            }
        }
        
        private void HandleSoftBodyDestroyed(SoftBodyComponent softBody)
        {
            UnregisterSoftBody(softBody);
        }
        
        private void HandleSoftBodyModified(SoftBodyComponent softBody)
        {
            OnSoftBodyModified?.Invoke(softBody);
        }
        
        private string GenerateUniqueID()
        {
            string id;
            do
            {
                id = $"{idPrefix}{nextID:D4}";
                nextID++;
            }
            while (softBodies.ContainsKey(id));
            
            return id;
        }
        
        private void UpdateStatistics()
        {
            activeSoftBodies = 0;
            foreach (var softBody in softBodies.Values)
            {
                if (softBody != null && softBody.isActive)
                {
                    activeSoftBodies++;
                }
            }
        }
        
        // Public API methods
        public SoftBodyComponent GetSoftBody(string id)
        {
            return softBodies.ContainsKey(id) ? softBodies[id] : null;
        }
        
        public SoftBodyComponent[] GetSoftBodiesByType(SoftBodyType type)
        {
            return softBodiesByType.ContainsKey(type) ? softBodiesByType[type].ToArray() : new SoftBodyComponent[0];
        }
        
        public SoftBodyComponent[] GetAllSoftBodies()
        {
            var result = new SoftBodyComponent[softBodies.Count];
            softBodies.Values.CopyTo(result, 0);
            return result;
        }
        
        public string[] GetAllSoftBodyIDs()
        {
            var result = new string[softBodies.Count];
            softBodies.Keys.CopyTo(result, 0);
            return result;
        }
        
        public SoftBodyComponent GetClosestSoftBody(Vector3 position, SoftBodyType? type = null)
        {
            SoftBodyComponent closest = null;
            float minDistance = float.MaxValue;
            
            foreach (var softBody in softBodies.Values)
            {
                if (softBody == null || !softBody.isActive) continue;
                if (type.HasValue && softBody.BodyType != type.Value) continue;
                
                float distance = Vector3.Distance(position, softBody.GetCenterPosition());
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closest = softBody;
                }
            }
            
            return closest;
        }
        
        public SoftBodyComponent[] GetSoftBodiesInRadius(Vector3 center, float radius, SoftBodyType? type = null)
        {
            var result = new List<SoftBodyComponent>();
            
            foreach (var softBody in softBodies.Values)
            {
                if (softBody == null || !softBody.isActive) continue;
                if (type.HasValue && softBody.BodyType != type.Value) continue;
                
                if (Vector3.Distance(center, softBody.GetCenterPosition()) <= radius)
                {
                    result.Add(softBody);
                }
            }
            
            return result.ToArray();
        }
        
        public bool DoesSoftBodyExist(string id)
        {
            return !string.IsNullOrEmpty(id) && softBodies.ContainsKey(id);
        }
        
        public SoftBodyComponent FindSoftBodyByName(string name)
        {
            foreach (var softBody in softBodies.Values)
            {
                if (softBody != null && softBody.SoftBodyName == name)
                {
                    return softBody;
                }
            }
            return null;
        }
        
        public SoftBodyComponent FindSoftBodyByGameObject(GameObject go)
        {
            if (go == null) return null;
            
            var component = go.GetComponent<SoftBodyComponent>();
            if (component != null && softBodies.ContainsValue(component))
            {
                return component;
            }
            
            return null;
        }
        
        // Utility methods for other scripts
        public static string GetCurrentSoftBodyID(Transform transform)
        {
            if (transform == null) return null;
            
            var softBody = transform.GetComponentInParent<SoftBodyComponent>();
            return softBody?.SoftBodyID;
        }
        
        public static SoftBodyComponent GetCurrentSoftBody(Transform transform)
        {
            if (transform == null) return null;
            
            return transform.GetComponentInParent<SoftBodyComponent>();
        }
        
        // Debug and editor utilities
        public void LogSoftBodyInfo()
        {
            Debug.Log($"=== Soft Body Manager Statistics ===");
            Debug.Log($"Total Soft Bodies: {totalSoftBodies}");
            Debug.Log($"Active Soft Bodies: {activeSoftBodies}");
            
            foreach (var type in System.Enum.GetValues(typeof(SoftBodyType)))
            {
                var count = softBodiesByType[(SoftBodyType)type].Count;
                if (count > 0)
                {
                    Debug.Log($"{type}: {count}");
                }
            }
        }
        
        public void ClearAllSoftBodies()
        {
            var toDestroy = new List<SoftBodyComponent>(softBodies.Values);
            foreach (var softBody in toDestroy)
            {
                if (softBody != null && softBody.gameObject != null)
                {
                    DestroyImmediate(softBody.gameObject);
                }
            }
            
            softBodies.Clear();
            foreach (var list in softBodiesByType.Values)
            {
                list.Clear();
            }
            
            totalSoftBodies = 0;
            activeSoftBodies = 0;
            
            Debug.Log("Cleared all soft bodies");
        }
        
        // Gizmos for debugging
        private void OnDrawGizmos()
        {
            if (!debugMode) return;
            
            foreach (var softBody in softBodies.Values)
            {
                if (softBody == null || !softBody.isActive) continue;
                
                Gizmos.color = GetTypeColor(softBody.BodyType);
                Gizmos.DrawWireCube(softBody.GetCenterPosition(), Vector3.one * 0.5f);
                
                #if UNITY_EDITOR
                UnityEditor.Handles.Label(softBody.GetCenterPosition() + Vector3.up * 0.3f, softBody.SoftBodyID);
                #endif
            }
        }
        
        private Color GetTypeColor(SoftBodyType type)
        {
            switch (type)
            {
                case SoftBodyType.Vehicle: return Color.blue;
                case SoftBodyType.Character: return Color.green;
                case SoftBodyType.Environment: return Color.gray;
                case SoftBodyType.Projectile: return Color.red;
                case SoftBodyType.Structure: return Color.cyan;
                case SoftBodyType.Fabric: return Color.magenta;
                case SoftBodyType.Liquid: return Color.blue;
                default: return Color.white;
            }
        }
    }
    
    /// <summary>
    /// Extension methods for easier soft body access
    /// </summary>
    public static class SoftBodyExtensions
    {
        public static string GetSoftBodyID(this Transform transform)
        {
            return SoftBodyManager.GetCurrentSoftBodyID(transform);
        }
        
        public static SoftBodyComponent GetSoftBody(this Transform transform)
        {
            return SoftBodyManager.GetCurrentSoftBody(transform);
        }
        
        public static SoftBodyComponent GetSoftBody(this GameObject gameObject)
        {
            return SoftBodyManager.Instance.FindSoftBodyByGameObject(gameObject);
        }
    }
}