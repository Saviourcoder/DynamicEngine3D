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
        [SerializeField] private bool useAdvancedSkinning = true;

        [Header("Debug Visualization")]
        [SerializeField] private bool showNodes = true;
        [SerializeField] private bool showLinks = true;
        [SerializeField] private bool showInfluenceRadius = false;
        [SerializeField] private bool showSkinningInfluences = false;
        [SerializeField] private bool showForces = true;
        [SerializeField] private bool SolverDebug = true;
        [SerializeField, Range(0.05f, 0.5f)] public float nodeDisplaySize = 0.1f;

        // Properties
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

        public bool UseAdvancedSkinning
        {
            get => useAdvancedSkinning;
            set
            {
                useAdvancedSkinning = value;
                if (solver != null && solver.meshDeformer != null)
                {
                    solver.meshDeformer.SetSkinningMethod(useAdvancedSkinning);
                    if (solver.nodeManager.Nodes.Count > 0)
                    {
                        solver.meshDeformer.MapVerticesToNodes(transform, solver.nodeManager.Nodes, solver.nodeManager.InitialPositions);
                    }
                }
            }
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
                UpdateMassDistribution();
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
            if (solver != null)
            {
                // Set visualization flags
                solver.visualizeForces = SolverDebug || showForces;

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

            if (solver.meshDeformer != null)
            {
                solver.meshDeformer.SetSkinningMethod(useAdvancedSkinning);
            }

            if (truss != null)
            {
                ApplyTruss();
                // Apply Matter settings after truss is set up
                if (matter != null)
                {
                    solver.SetMatterAsset(matter);
                }
            }
            else
            {
                solver.GenerateCubeTest(transform);
                UpdateMassDistribution(); // Apply mass distribution for test cube
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

            // Apply Matter asset if available
            if (matter != null && solver != null)
            {
                solver.SetMatterAsset(matter);
            }
        }

        private void ApplyPinnedNodesFromTruss()
        {
            if (truss == null || solver == null)
                return;

            var pinnedList = new List<bool>(new bool[solver.nodeManager.Nodes.Count]);

            foreach (int pinnedNode in truss.PinnedNodes)
            {
                if (pinnedNode >= 0 && pinnedNode < pinnedList.Count)
                {
                    pinnedList[pinnedNode] = true;
                }
            }

            solver.nodeManager.SetPinnedNodes(pinnedList);
        }

        /// <summary>
        /// Updates mass distribution across all nodes based on total mass and beam connectivity
        /// </summary>
        private void UpdateMassDistribution()
        {
            if (solver == null || solver.nodeManager.Nodes == null || solver.nodeManager.Nodes.Count == 0)
                return;

            int nodeCount = solver.nodeManager.Nodes.Count;
            float baseMassPerNode = Mass / nodeCount;

            solver.nodeMasses.Clear();

            for (int i = 0; i < nodeCount; i++)
            {
                // Count how many beams connect to this node
                int beamCount = 0;
                foreach (var beam in solver.beams)
                {
                    if (beam.nodeA == i || beam.nodeB == i)
                        beamCount++;
                }

                // Nodes with more connections get slightly more mass (structural importance)
                float nodeMass = baseMassPerNode * (1f + beamCount * 0.1f);
                solver.nodeMasses.Add(Mathf.Max(0.01f, nodeMass));
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
                    if (beam.nodeA >= solver.nodeManager.Nodes.Count ||
                        beam.nodeB >= solver.nodeManager.Nodes.Count ||
                        solver.nodeManager.Nodes[beam.nodeA] == null ||
                        solver.nodeManager.Nodes[beam.nodeB] == null)
                        continue;

                    Vector3 posA = solver.nodeManager.Nodes[beam.nodeA].position;
                    Vector3 posB = solver.nodeManager.Nodes[beam.nodeB].position;

                    Debug.DrawLine(posA, posB, solver.linkColor, 0f);
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
    }
}