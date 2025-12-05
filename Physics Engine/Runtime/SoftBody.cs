/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
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
        [SerializeField] public Truss truss;
        [SerializeField] public Matter matter;
        [SerializeField, Range(0.1f, 5000f)] private float Mass = 1f;

        [Header("Deformation Settings")]
        [SerializeField,]
        private bool useAdvancedSkinning = true;
        [Header("Internal Pressure System")]
        [SerializeField,]
        private bool enableInternalPressure = false;
        [SerializeField, Range(0f, 100f)]
        private float PSI = 14.7f;
        [SerializeField, Range(0.1f, 1f)]
        private float pressureScale = 1f;
        [SerializeField, Range(0f, 1f)]
        private float pressureDamping = 0.3f;
        [SerializeField]
        private bool showForces = true;

        [Header("Debug Visualization")]
        private bool showNodes = true;
        [SerializeField] private bool showLinks = true;
        [SerializeField] private bool showInfluenceRadius = false;
        [SerializeField] private bool showSkinningInfluences = false;
        [SerializeField] private bool SolverDebug = true;
        [SerializeField, Range(0.05f, 0.5f)] public float nodeDisplaySize = 0.1f;

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
        public bool ShowInfluenceRadius
        {
            get => showInfluenceRadius;
            set => showInfluenceRadius = value;
        }
        public bool ShowSkinningInfluences
        {
            get => showSkinningInfluences;
            set => showSkinningInfluences = value;
        }
        // Deformation properties
        public bool UseAdvancedSkinning
        {
            get => useAdvancedSkinning;
            set
            {
                useAdvancedSkinning = value;
                if (solver != null && solver.meshDeformer != null)
                {
                    solver.meshDeformer.SetSkinningMethod(useAdvancedSkinning);
                    // Re-map vertices if we're already initialized
                    if (solver.nodeManager.Nodes.Count > 0)
                    {
                        solver.meshDeformer.MapVerticesToNodes(transform, solver.nodeManager.Nodes, solver.nodeManager.InitialPositions);
                    }
                }
            }
        }
        public bool EnableInternalPressure
        {
            get => enableInternalPressure;
            set
            {
                enableInternalPressure = value;
                if (solver != null)
                {
                    // Update solver immediately
                    solver.internalPressure = value ? InternalPressurePascals * pressureScale : 0f;
                }
            }
        }

        public float InternalPressurePSI
        {
            get => PSI;
            set
            {
                PSI = Mathf.Max(0f, value);
                if (solver != null && enableInternalPressure)
                {
                    solver.internalPressure = InternalPressurePascals * pressureScale;
                }
            }
        }

        public float PressureScale
        {
            get => pressureScale;
            set
            {
                pressureScale = Mathf.Max(0.1f, value);
                if (solver != null && enableInternalPressure)
                {
                    solver.internalPressure = InternalPressurePascals * pressureScale;
                }
            }
        }
        public float InternalPressurePascals
        {
            get => PSI * 6894.76f;
        }

        private MeshFilter meshFilter;
        private Mesh mesh;
        public Solver solver;
        public float ScaledMass
        {
            get => Mass;
            set
            {
                Mass = Mathf.Max(0.1f, value);
                if (solver != null && solver.nodeManager.Nodes != null)
                {
                    int nodeCount = solver.nodeManager.Nodes.Count;
                    if (nodeCount > 0)
                    {
                        float baseMassPerNode = Mass / nodeCount;
                        solver.nodeMasses.Clear();
                        for (int i = 0; i < nodeCount; i++)
                        {
                            int beamCount = 0;
                            foreach (var beam in solver.beams)
                            {
                                if (beam.nodeA == i || beam.nodeB == i) beamCount++;
                            }
                            float nodeMass = baseMassPerNode * (1f + beamCount * 0.1f);
                            solver.nodeMasses.Add(Mathf.Max(0.01f, nodeMass));
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
                return Mass;
            }
        }
        private void Awake()
        {
            InitializeInPlayMode();
            if (truss == null)
            {
                Debug.LogError("No Truss assigned!", this);
                enabled = false;
                return;
            }
            SetPressure(PSI, enableInternalPressure, pressureDamping);
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
                if (enableInternalPressure)
                {
                }
                solver.visualizeForces = showForces;

            }
        }
        void Update()
        {
            if (solver != null)
            {
                DrawRuntimeVisualization();
                if (showSkinningInfluences && Application.isPlaying)
                {
                    solver.meshDeformer.DebugInfluences(transform, solver.nodeManager.Nodes, solver.nodeManager.InitialPositions);
                }
            }
        }
        void OnDestroy()
        {
            solver?.nodeManager.Clear();
        }
        private void OnDisable()
        {
        }
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

            solver = new Solver(1f, mesh, mesh.vertices, transform);
            solver.visualizeForces = showForces;

            if (matter != null)
            {
                solver.SetMatterAsset(matter);
            }

            if (solver.meshDeformer != null)
            {
                solver.meshDeformer.SetSkinningMethod(useAdvancedSkinning);
            }

            if (truss != null)
            {
                ApplyTruss();
            }
            else
            {
                solver.GenerateCubeTest(transform);
                ScaledMass = Mass; // Only use this if we generated a test cube
            }
            return true;
        }
        public void SetPressure(float psi, bool enabled = true, float damping = 0.3f)
        {
            enableInternalPressure = enabled;
            PSI = Mathf.Max(0f, psi);
            pressureDamping = Mathf.Clamp01(damping);

            if (solver != null)
            {
            }
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
        public Solver GetSolver()
        {
            return solver;
        }
        public Truss GetTrussAsset()
        {
            return truss;
        }

        public Matter GetMatterAsset()
        {
            return matter;
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
            }
        }
        private void DrawRuntimeVisualization()
        {
            if (!showNodes && !showLinks && !showInfluenceRadius)
                return;
            if (solver == null || solver.nodeManager?.Nodes == null)
                return;
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
            if (showLinks)
            {
                foreach (var beam in solver.beams)
                {
                    if (beam.nodeA >= solver.nodeManager.Nodes.Count || beam.nodeB >= solver.nodeManager.Nodes.Count ||
                    solver.nodeManager.Nodes[beam.nodeA] == null || solver.nodeManager.Nodes[beam.nodeB] == null)
                        continue;
                    Vector3 posA = solver.nodeManager.Nodes[beam.nodeA].position;
                    Vector3 posB = solver.nodeManager.Nodes[beam.nodeB].position;
                    Color color = solver.linkColor;
                    Debug.DrawLine(posA, posB, color, 0f);
                }
            }
            if (showInfluenceRadius)
            {
                for (int i = 0; i < solver.nodeManager.Nodes.Count; i++)
                {
                    if (solver.nodeManager.Nodes[i] == null) continue;
                    Vector3 pos = solver.nodeManager.Nodes[i].position;
                    DrawWireSphere(pos, solver.meshDeformer.InfluenceRadius, solver.influenceRadiusColor);
                }
            }
            if (enableInternalPressure)
            {
            }
            if (SolverDebug)
            {
                solver.visualizeForces = true;
            }
        }
        private void DrawWireSphere(Vector3 center, float radius, Color color)
        {
            int segments = 16;
            DrawCircle(center, radius, Vector3.up, Vector3.forward, segments, color);
            DrawCircle(center, radius, Vector3.right, Vector3.forward, segments, color);
            DrawCircle(center, radius, Vector3.right, Vector3.up, segments, color);
        }
        private void DrawCircle(Vector3 center, float radius, Vector3 axis1, Vector3 axis2, int segments, Color color)
        {
            float angleStep = 360f / segments;
            Vector3 prevPoint = center + axis1 * radius;
            for (int i = 1; i <= segments; i++)
            {
                float angle = i * angleStep * Mathf.Deg2Rad;
                Vector3 newPoint = center + (axis1 * Mathf.Cos(angle) + axis2 * Mathf.Sin(angle)) * radius;
                Debug.DrawLine(prevPoint, newPoint, color, 0f);
                prevPoint = newPoint;
            }
        }
        private void ValidateParameters()
        {
            Mass = Mathf.Max(0.1f, Mass);
        }
        public void InitializePressureSystem()
        {
            if (solver != null)
            { }
        }
    }
}