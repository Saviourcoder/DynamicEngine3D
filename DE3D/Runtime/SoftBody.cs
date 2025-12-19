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

        [Header("Pressure Settings")]
        [SerializeField, Range(0f, 100f)] private float internalPressure = 0f;

        [Header("Deformation Settings")]
        [SerializeField] private bool useAdvancedSkinning = true;

        public float InternalPressure
        {
            get => internalPressure;
            set
            {
                internalPressure = Mathf.Max(0f, value);
                if (solver != null)
                {
                    solver.internalPressure = internalPressure;
                }
            }
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

            // Update pressure in solver if it exists
            if (solver != null)
            {
                solver.internalPressure = internalPressure;
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

        private bool InitializeCore()
        {
            if (!SetupMesh())
                return false;

            solver = new Solver(1f, mesh, mesh.vertices, transform);

            if (solver.meshDeformer != null)
            {
                solver.meshDeformer.SetSkinningMethod(useAdvancedSkinning);
            }

            // Set initial pressure
            solver.internalPressure = internalPressure;

            if (truss != null)
            {
                ApplyTruss();
                if (matter != null)
                {
                    solver.SetMatterAsset(matter);
                }
            }
            else
            {
                solver.GenerateCubeTest(transform);
                UpdateMassDistribution();
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

        private void ValidateParameters()
        {
            Mass = Mathf.Max(0.1f, Mass);
            internalPressure = Mathf.Max(0f, internalPressure);
        }
    }
}