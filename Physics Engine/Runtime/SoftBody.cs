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
        [SerializeField, Range(0.1f, 5000f)] private float scaledMass = 900f;
        [SerializeField, Range(0.01f, 5f)] private float influenceRadius = 1f;

        [Header("Internal Pressure System")]
        [SerializeField, Tooltip("Enable internal pressure simulation for inflated soft-bodies")]
        public bool enableInternalPressure = false;

        [SerializeField, Range(0f, 100f), Tooltip("Internal pressure in PSI (Pounds per Square Inch)")]
        private float PSI = 14.7f;

        [SerializeField, Tooltip("Show pressure force vectors in scene view")]
        private bool showForces = false;

        [SerializeField, Range(0f, 1f), Tooltip("Pressure damping to stabilize inflation")]
        private float pressureDamping = 0.3f;

        [Header("Runtime Visualization")]
        [SerializeField, Tooltip("Show nodes for debugging purposes")]
        private bool showNodes = true;
        [SerializeField] private bool showLinks = true;
        [SerializeField] private bool showLinkForces = false;
        [SerializeField] private bool showInfluenceRadius = false;
        [SerializeField, Range(0.05f, 0.5f)] public float nodeDisplaySize = 0.1f;
        [SerializeField, Range(0.001f, 0.1f)] private float forceVisualizationScale = 0.01f;
        
        [Header("Rendering Settings")]
        [SerializeField] private Material nodeMaterial;
        [SerializeField] private Material beamMaterial;
        
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
        public float ScaledMass 
        { 
            get => scaledMass;
            set
            {
                scaledMass = Mathf.Max(0.1f, value);
                if (solver != null && solver.nodeManager.Nodes != null)
                {
                    int nodeCount = solver.nodeManager.Nodes.Count;
                    if (nodeCount > 0)
                    {
                        float massPerNode = scaledMass / nodeCount;
                        solver.nodeMasses.Clear();
                        for (int i = 0; i < nodeCount; i++)
                        {
                            solver.nodeMasses.Add(Mathf.Max(0.1f, massPerNode));
                        }
                    }
                }
            }
        }
        
        public float CalculatedScaledMass 
        { 
            get 
            {
                if (solver?.nodeManager?.Nodes == null) 
                    return 0f;
                return scaledMass; // Return the set scaled mass instead of calculated
            }
        }

        // Unity lifecycle methods
        private void Awake()
        {
            InitializeInPlayMode();
            if (truss == null)
            {
                Debug.LogError("No Truss assigned!", this);
                enabled = false;
                return;
            }

            Debug.Log("SoftBody: Initialized");
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
            
            // Clean up created materials
            if (nodeMaterial != null)
                DestroyImmediate(nodeMaterial);
            if (beamMaterial != null)
                DestroyImmediate(beamMaterial);
        }

        private void OnDisable()
        {
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

            // Trigger uniform mass distribution based on ScaledMass (overrides Truss if present)
            ScaledMass = scaledMass;

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
            solver.InitializeFromTruss(truss);  
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
            
            if (s_isApplicationQuitting || gameObject == null)
            {
                return;
            }
            
            truss = asset;
            ApplyTruss();
        }

        private void ApplyPinnedNodesFromTruss()
        {
            if (truss == null || solver == null)
                return;
                
            var pinnedList = new List<bool>(new bool[solver.nodeManager.Nodes.Count]);
            foreach (int pinnedNode in truss.PinnedNodes)
            {
                if (pinnedNode < pinnedList.Count)
                {
                    pinnedList[pinnedNode] = true;
                }
            }
            solver.nodeManager.SetPinnedNodes(pinnedList);
        }

        private void ApplyStretchFactorsFromTruss()
        {
            if (truss == null || solver == null)
                return;

            for (int i = 0; i < solver.beams.Count; i++)
            {
                if (i < truss.GetTrussBeams().Count)
                {
                    solver.maxStretchFactor = truss.MaxStretchFactor;
                    solver.minStretchFactor = truss.MinStretchFactor;
                }
            }
        }

        private void DrawRuntimeVisualization()
        {
            if (!showNodes && !showLinks && !showLinkForces && !showInfluenceRadius)
                return;

            if (solver == null || solver.nodeManager?.Nodes == null)
                return;

            // Draw nodes
            if (showNodes)
            {
                for (int i = 0; i < solver.nodeManager.Nodes.Count; i++)
                {
                    if (solver.nodeManager.Nodes[i] == null) continue;
                    Vector3 pos = solver.nodeManager.Nodes[i].position;
                    bool isPinned = i < solver.nodeManager.IsPinned.Count && solver.nodeManager.IsPinned[i];
                    Color color = isPinned ? solver.pinnedNodeColor : solver.nodeColor;
                    Debug.DrawRay(pos, Vector3.up * nodeDisplaySize, color, 0f);
                }
            }

            // Draw links and forces
            if (showLinks || showLinkForces)
            {
                foreach (var beam in solver.beams)
                {
                    if (beam.nodeA >= solver.nodeManager.Nodes.Count || beam.nodeB >= solver.nodeManager.Nodes.Count ||
                        solver.nodeManager.Nodes[beam.nodeA] == null || solver.nodeManager.Nodes[beam.nodeB] == null)
                        continue;

                    Vector3 posA = solver.nodeManager.Nodes[beam.nodeA].position;
                    Vector3 posB = solver.nodeManager.Nodes[beam.nodeB].position;

                    if (showLinks)
                    {
                        Color color = solver.linkColor;
                        float currentLength = Vector3.Distance(posA, posB);
                        if (currentLength > beam.restLength * 1.01f)
                            color = solver.stretchedLinkColor;
                        else if (currentLength < beam.restLength * 0.99f)
                            color = solver.compressedLinkColor;
                        Debug.DrawLine(posA, posB, color, 0f);
                    }

                    if (showLinkForces && beam.lagrangeMultiplier != 0)
                    {
                        Vector3 midPoint = (posA + posB) * 0.5f;
                        Vector3 direction = (posB - posA).normalized;
                        float forceMagnitude = beam.lagrangeMultiplier * forceVisualizationScale;
                        if (forceMagnitude > 0)
                        {
                            Debug.DrawRay(midPoint, direction * forceMagnitude, Color.red, 0f);
                            Debug.DrawRay(midPoint, -direction * forceMagnitude, Color.red, 0f);
                        }
                        else
                        {
                            Debug.DrawRay(midPoint, direction * Mathf.Abs(forceMagnitude), Color.blue, 0f);
                            Debug.DrawRay(midPoint, -direction * Mathf.Abs(forceMagnitude), Color.blue, 0f);
                        }
                    }
                }
            }

            // Draw influence radius
            if (showInfluenceRadius)
            {
                for (int i = 0; i < solver.nodeManager.Nodes.Count; i++)
                {
                    if (solver.nodeManager.Nodes[i] == null) continue;
                    Vector3 pos = solver.nodeManager.Nodes[i].position;
                    Debug.DrawRay(pos, Vector3.right * solver.meshDeformer.InfluenceRadius, solver.influenceRadiusColor, 0f);
                    Debug.DrawRay(pos, Vector3.up * solver.meshDeformer.InfluenceRadius, solver.influenceRadiusColor, 0f);
                    Debug.DrawRay(pos, Vector3.forward * solver.meshDeformer.InfluenceRadius, solver.influenceRadiusColor, 0f);
                }
            }
        }

        private void ValidateParameters()
        {
            scaledMass = Mathf.Max(0.1f, scaledMass);
            influenceRadius = Mathf.Max(0.01f, influenceRadius);
            PSI = Mathf.Max(0f, PSI);
            pressureDamping = Mathf.Clamp01(pressureDamping);
            nodeDisplaySize = Mathf.Clamp(nodeDisplaySize, 0.05f, 0.5f);
            forceVisualizationScale = Mathf.Clamp(forceVisualizationScale, 0.001f, 0.1f);
        }
    }
}