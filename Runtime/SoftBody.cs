/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */

using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    public class SoftBody : MonoBehaviour
    {
        private static bool s_isApplicationQuitting = false;
        
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void InitializeOnLoad()
        {
            s_isApplicationQuitting = false;
            Application.quitting += () => s_isApplicationQuitting = true;
        }
        
        [Header("Truss Configuration")]
        [SerializeField] public Truss truss;
        [SerializeField, Range(0.1f, 5000f)] private float totalMass = 900f;
        [SerializeField, Range(0.01f, 5f)] private float influenceRadius = 1f;

        [Header("Internal Pressure System")]
        [SerializeField, Tooltip("Enable internal pressure simulation for inflated soft-bodies")]
        private bool enableInternalPressure = false;

        [SerializeField, Range(0f, 100f), Tooltip("Internal pressure in PSI (Pounds per Square Inch)")]
        private float PSI = 14.7f;

        [SerializeField, Tooltip("Show pressure force vectors in scene view")]
        private bool showForces = false;

        [SerializeField, Range(0f, 1f), Tooltip("Pressure damping to stabilize inflation")]
        private float pressureDamping = 0.3f;

        [Header("Runtime Visualization")]
        [SerializeField] private bool showNodes = true;
        [SerializeField] private bool showLinks = true;
        [SerializeField] private bool showLinkForces = false;
        [SerializeField] private bool showInfluenceRadius = false;
        [SerializeField, Range(0.05f, 0.5f)] private float nodeDisplaySize = 0.1f;
        [SerializeField, Range(0.001f, 0.1f)] private float forceVisualizationScale = 0.01f;
        // Public properties for external access
        public bool ShowNodes 
        { 
            get => showNodes; 
            set => showNodes = value; 
        }
        
        public bool ShowLinks 
        { 
            get => showLinks; 
            set => showLinks = value; 
        }
        
        public bool ShowLinkForces 
        { 
            get => showLinkForces; 
            set => showLinkForces = value; 
        }
        
        public bool ShowInfluenceRadius 
        { 
            get => showInfluenceRadius; 
            set => showInfluenceRadius = value; 
        }
        
        // Pressure system properties
        public bool EnableInternalPressure
        {
            get => enableInternalPressure;
            set 
            {
                enableInternalPressure = value;
                if (solver != null)
                {
                    solver.SetInternalPressure(enableInternalPressure, InternalPressurePascals, 1f, pressureDamping);
                }
            }
        }
        
        public float InternalPressurePSI
        {
            get => PSI;
            set 
            {
                PSI = Mathf.Max(0f, value);
                if (solver != null)
                {
                    solver.SetInternalPressure(enableInternalPressure, InternalPressurePascals, 1f, pressureDamping);
                }
            }
        }
        
        public float InternalPressurePascals => PSI * 6895f; // Convert PSI to Pascals
        
        
        // Pressure damping property to propagate runtime changes
        public float PressureDamping
        {
            get => pressureDamping;
            set
            {
                pressureDamping = Mathf.Clamp01(value);
                if (solver != null)
                {
                    solver.SetInternalPressure(enableInternalPressure, InternalPressurePascals, 1f, pressureDamping);
                }
            }
        }
        
        public bool ShowPressureForces
        {
            get => showForces;
            set 
            {
                showForces = value;
                if (solver != null)
                {
                    solver.SetShowPressureForces(showForces);
                }
            }
        }
        
        // Core system components
        private MeshFilter meshFilter;
        private Mesh mesh;
        public Solver solver;

        // Properties for external access
        public float InfluenceRadius => influenceRadius;
        
        // Mass calculation properties
        public float TotalMass 
        { 
            get => totalMass; 
            set 
            { 
                totalMass = Mathf.Max(0.1f, value);
            } 
        }
        
        public float CalculatedTotalMass 
        { 
            get 
            {
                if (solver?.nodeManager?.Nodes == null) 
                    return 0f;
                return totalMass; // Return the set total mass instead of calculated
            }
        }

        // Unity lifecycle methods
        private void Awake()
        {
            InitializeInPlayMode();
            if (truss == null)
            {
                Debug.LogError("No Truss assigned!", this);
            }
        }
    
        private void OnValidate()
        {
            ValidateParameters();
            
            if (truss == null)
            {
                Debug.LogWarning("No Truss assigned", this);
            }
        }

        void FixedUpdate()
        {
            if (solver != null)
            {
                solver.Solve();
                solver.DeformMesh(transform);
            }
        }

        void Update()
        {
            // Draw runtime visualization
            if (solver != null)
            {
                DrawRuntimeVisualization();
            }
        }

        void OnDestroy()
        {
            solver?.nodeManager.Clear();
            CleanupNodes(); 
        }

        private void OnDisable()
        {
           CleanupNodes();
        }

        // Core initialization methods
        private void InitializeInPlayMode()
        {
            if (Application.isPlaying) 
                InitializeCore();
        }

        public void InitializeInEditMode()
        {
            if (!Application.isPlaying)
            {
                InitializeCore();
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }

        private bool InitializeCore()
        {
            if (!SetupMesh()) 
                return false;

            // Initialize solver with basic configuration
            solver = new Solver(influenceRadius, mesh, mesh.vertices, transform);
        
            if (truss != null) 
            {
                ApplyTruss();
            }
            else 
            {
                solver.GenerateCubeTest(transform);
            }

            return true;
        }

        private bool SetupMesh()
        {
            meshFilter = GetComponent<MeshFilter>();
            if (!meshFilter || meshFilter.sharedMesh == null) 
                return false;

            mesh = Application.isPlaying
                ? (meshFilter.mesh = Instantiate(meshFilter.sharedMesh))
                : meshFilter.sharedMesh;
            return true;
        }

        // Public interface methods
        public Truss GetTrussAsset()
        {
            return truss;
        }

        private void ApplyTruss()
        {
            if (truss == null || solver == null)
            {
                Debug.LogWarning("[SoftBody] Cannot apply truss: No Truss or Solver", this);
                return;
            }

            solver.GenerateNodesAndBeams(truss.NodePositions, -1f, truss.GetTrussBeams().ToArray(), transform);
            solver.nodeMass = truss.NodeMass;
            ApplyStretchFactorsFromTruss();
            ApplyPinnedNodesFromTruss();

            solver.SetInternalPressure(enableInternalPressure, InternalPressurePascals, 1f, pressureDamping);
        }

        public void ApplyTrussAsset(Truss asset)
        {  
            if (asset == null) 
            { 
                Debug.LogWarning("Cannot apply null TrussAsset", this); 
                return; 
            }
            
            // Don't apply truss during application quit or when GameObject is being destroyed
            if (s_isApplicationQuitting || gameObject == null)
                return;
                
            truss = asset;
            ApplyTruss();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif 
        }
        
        public void ForceValidateTrussConfiguration()
        {
            if (truss != null && solver != null)
            {
                ApplyTruss();
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }

        public string VerifyTrussConfiguration()
        {
            if (truss == null)
                return "No Truss assigned";
                
            if (solver == null)
                return "Solver not initialized";

            var report = new System.Text.StringBuilder();
            report.AppendLine("Truss Configuration Verification:");
            
            // Check node count
            int trussNodes = truss.NodePositions?.Length ?? 0;
            int solverNodes = solver.nodeManager?.Nodes?.Count ?? 0;
            report.AppendLine($"Nodes: TrussAsset={trussNodes}, Solver={solverNodes} {(trussNodes == solverNodes ? "✓" : "✗")}");
            
            // Check beam count
            int trussBeams = truss.GetTrussBeams()?.Count ?? 0;
            int solverBeams = solver.beams?.Count ?? 0;
            report.AppendLine($"Beams: TrussAsset={trussBeams}, Solver={solverBeams} {(trussBeams == solverBeams ? "✓" : "✗")}");
            
            // Check pinned nodes
            int trussPin = truss.PinnedNodes?.Count ?? 0;
            int solverPin = 0;
            if (solver.nodeManager?.IsPinned != null)
            {
                foreach (bool pinned in solver.nodeManager.IsPinned)
                {
                    if (pinned) solverPin++;
                }
            }
            report.AppendLine($"Pinned Nodes: TrussAsset={trussPin}, Solver={solverPin} {(trussPin == solverPin ? "✓" : "✗")}");
            
            return report.ToString();
        }

        // Visualization and debugging methods
        private void DrawRuntimeVisualization()
        {
            if (solver?.nodeManager?.Nodes == null) 
                return;

            if (showNodes)
                DrawNodes();
            
            if (showLinks)
                DrawLinks();
                
            if (showInfluenceRadius)
                DrawInfluenceRadius();
                
            if (showForces && enableInternalPressure)
                DrawPressureForces();
        }

        private void DrawNodes()
        {
            for (int i = 0; i < solver.nodeManager.Nodes.Count; i++)
            {
                if (solver.nodeManager.Nodes[i] == null) 
                    continue;

                Vector3 position = solver.nodeManager.Nodes[i].position;
                bool isPinned = i < solver.nodeManager.IsPinned.Count && solver.nodeManager.IsPinned[i];
                // Draw node as wireframe sphere
                DrawWireSphere(position, nodeDisplaySize, Color.orange);
            }
        }

        private void DrawLinks()
        {
            if (solver.beams == null) 
                return;

            for (int i = 0; i < solver.beams.Count; i++)
            {
                var beam = solver.beams[i];
                
                if (beam.nodeA >= solver.nodeManager.Nodes.Count || beam.nodeB >= solver.nodeManager.Nodes.Count ||
                    solver.nodeManager.Nodes[beam.nodeA] == null || solver.nodeManager.Nodes[beam.nodeB] == null)
                    continue;

                Vector3 posA = solver.nodeManager.Nodes[beam.nodeA].position;
                Vector3 posB = solver.nodeManager.Nodes[beam.nodeB].position;

                // Draw force visualization if enabled
                if (showLinkForces && beam.lagrangeMultiplier != 0)
                {
                    Vector3 midPoint = (posA + posB) * 0.5f;
                    Vector3 direction = (posB - posA).normalized;
                    float forceMagnitude = beam.lagrangeMultiplier * forceVisualizationScale;
                    
                    if (forceMagnitude > 0)
                    {
                        // Tension (pulling apart)
                        Debug.DrawRay(midPoint, direction * forceMagnitude, Color.red, Time.fixedDeltaTime);
                        Debug.DrawRay(midPoint, -direction * forceMagnitude, Color.red, Time.fixedDeltaTime);
                    }
                    else
                    {
                        // Compression (pushing together)
                        Debug.DrawRay(midPoint, direction * Mathf.Abs(forceMagnitude), Color.blue, Time.fixedDeltaTime);
                        Debug.DrawRay(midPoint, -direction * Mathf.Abs(forceMagnitude), Color.blue, Time.fixedDeltaTime);
                    }
                }
            }
        }

        private void DrawInfluenceRadius()
        {
            if (solver?.meshDeformer == null) 
                return;
            
            float influenceRadius = solver.meshDeformer.InfluenceRadius;
            
            for (int i = 0; i < solver.nodeManager.Nodes.Count; i++)
            {
                if (solver.nodeManager.Nodes[i] == null) 
                    continue;

                Vector3 position = solver.nodeManager.Nodes[i].position;
                
                // Draw influence radius as wireframe sphere with transparent color
                DrawWireSphere(position, influenceRadius, Color.blue);
            }
        }

        public void ReinitializeWithExistingSettings()
        {
            if (truss == null) return;

            // Preserve current settings
            var savedStretchLimits = new Vector2(truss.MaxStretchFactor, truss.MinStretchFactor);
            var savedPinnedNodes = new List<int>(truss.PinnedNodes);

            // Reinitialize
            InitializeCore();

            // Reapply saved settings
            ApplyTruss();

#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
            EditorUtility.SetDirty(truss);
#endif
        }

        private void DrawWireSphere(Vector3 center, float radius, Color color)
        {
            // Draw three circles to form a wireframe sphere
            DrawCircle(center, radius, Vector3.up, color);
            DrawCircle(center, radius, Vector3.right, color);
            DrawCircle(center, radius, Vector3.forward, color);
        }
        
        private void DrawPressureForces()
        {
            if (!enableInternalPressure || solver?.nodeManager?.Nodes == null)
                return;
                
            Vector3 center = solver.GetPressureCenter();
            
            // Draw pressure center as a small sphere
            DrawWireSphere(center, 0.05f, Color.cyan);
            
            // Draw pressure force vectors from center to each node
            for (int i = 0; i < solver.nodeManager.Nodes.Count; i++)
            {
                if (solver.nodeManager.Nodes[i] == null)
                    continue;
                    
                Vector3 nodePos = solver.nodeManager.Nodes[i].position;
                Vector3 forceDirection = (nodePos - center).normalized;
                float forceMagnitude = InternalPressurePascals * 0.001f; // Scale for visualization
                
                // Draw force vector
                Debug.DrawRay(center, forceDirection * forceMagnitude, Color.cyan, Time.fixedDeltaTime);
                Debug.DrawRay(nodePos - forceDirection * 0.1f, forceDirection * 0.1f, Color.white, Time.fixedDeltaTime);
            }
        }

        private void DrawCircle(Vector3 center, float radius, Vector3 normal, Color color)
        {
            Vector3 forward = Vector3.Cross(normal, Vector3.up);
            if (forward.magnitude < 0.1f)
                forward = Vector3.Cross(normal, Vector3.right);
            
            Vector3 right = Vector3.Cross(normal, forward).normalized;
            forward = forward.normalized;

            int segments = 16;
            Vector3 prevPos = center + right * radius;
            
            for (int i = 1; i <= segments; i++)
            {
                float angle = (float)i / segments * Mathf.PI * 2f;
                Vector3 newPos = center + (right * Mathf.Cos(angle) + forward * Mathf.Sin(angle)) * radius;
                Debug.DrawLine(prevPos, newPos, color, Time.fixedDeltaTime);
                prevPos = newPos;
            }
        }

        // System utility methods
        private void ValidateParameters()
        {  
            // Validate pressure parameters
            PSI = Mathf.Max(0f, PSI);
            pressureDamping = Mathf.Clamp01(pressureDamping);
            
            // Update solver with new pressure settings if available
            if (solver != null)
            {
                solver.SetInternalPressure(enableInternalPressure, InternalPressurePascals, 1f, pressureDamping);
                solver.SetShowPressureForces(showForces);
            }
        }
        
        private void CleanupNodes()
        {
            if (transform == null) 
                return;

#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                // Editor: safe delayed destruction
                EditorApplication.delayCall += () =>
                {
                    if (this == null || transform == null) 
                        return;
                    for (int i = transform.childCount - 1; i >= 0; i--)
                    {
                        Transform child = transform.GetChild(i);
                        if (child.name.StartsWith("Node_"))
                            Undo.DestroyObjectImmediate(child.gameObject);
                    }
                };
            }
            else
#endif
            {
                // Runtime: standard Destroy
                for (int i = transform.childCount - 1; i >= 0; i--)
                {
                    Transform child = transform.GetChild(i);
                    if (child.name.StartsWith("Node_"))
                        Destroy(child.gameObject);
                }
            }
        }

        // End of SoftBody class

        /// <summary>
        /// Applies stretch factors from Truss to solver
        /// </summary>
        private void ApplyStretchFactorsFromTruss()
        {
            if (truss == null || solver == null)
                return;

            try
            {
                var maxField = typeof(Solver).GetField("maxStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                var minField = typeof(Solver).GetField("minStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                if (maxField != null && minField != null)
                {
                    maxField.SetValue(solver, truss.MaxStretchFactor);
                    minField.SetValue(solver, truss.MinStretchFactor);
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[SoftBody] Failed to apply stretch factors from TrussAsset: {e.Message}", this);
            }
        }

        /// <summary>
        /// Applies pinned nodes from Truss to solver
        /// </summary>
        private void ApplyPinnedNodesFromTruss()
        {
            if (truss == null || solver == null || truss.PinnedNodes == null)
                return;

            try
            {
                var pinMethod = typeof(Solver).GetMethod("PinNode", System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
                if (pinMethod != null)
                {
                    foreach (int nodeIndex in truss.PinnedNodes)
                    {
                        if (nodeIndex >= 0 && nodeIndex < truss.NodePositions.Length)
                        {
                            pinMethod.Invoke(solver, new object[] { nodeIndex });
                        }
                    }
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[SoftBody] Failed to apply pinned nodes from TrussAsset: {e.Message}", this);
            }
        }
    }
}