/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */

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
        public static List<SoftBody> AllSoftBodies = new List<SoftBody>();
        [SerializeField, Range(0.1f, 5000f)] private float Mass = 1f;

        [Header("Pressure Settings")]
        [SerializeField, Range(0f, 100f)] private float internalPressure = 0f;

        [Header("Deformation Settings")]
        [SerializeField] private bool useAdvancedSkinning = true;

        void OnEnable() => AllSoftBodies.Add(this);
        void OnDisable() => AllSoftBodies.Remove(this);
        public Bounds GetBounds()
        {
            return meshFilter.mesh.bounds; // Or calculate manually from solver nodes
        }
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
        public float MassScale
        {
            get
            {
                if (truss == null || truss.NodeMasses == null || truss.NodeMasses.Count == 0)
                    return 1f;

                float totalTrussMass = 0f;
                foreach (float m in truss.NodeMasses)
                {
                    totalTrussMass += m;
                }

                return totalTrussMass > 0f ? (Mass / totalTrussMass) : 1f;
            }
        }

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

            if (solver != null)
            {
                solver.internalPressure = internalPressure;
                // ADD THIS LINE:
                UpdateMassDistribution();
            }
        }

        private void FixedUpdate()
        {
            // Only the first body in the list orchestrates the global update
            if (AllSoftBodies.Count == 0 || AllSoftBodies[0] != this) return;

            RunGlobalPhysicsStep();
        }
        private static void RunGlobalPhysicsStep()
        {
            if (SceneSettings.Instance == null) return;

            int numSubSteps = SceneSettings.Instance.SubstepCount;
            float subDt = SceneSettings.Instance.SubstepSize;

            for (int step = 0; step < numSubSteps; step++)
            {
                foreach (var sb in AllSoftBodies)
                    if (sb.solver != null) sb.solver.SubStep_PrepareAndIntegrate(subDt);

                foreach (var sb in AllSoftBodies)
                    if (sb.solver != null) sb.solver.SubStep_ResolveConstraints(subDt);

                foreach (var sb in AllSoftBodies)
                    if (sb.solver != null) sb.solver.SubStep_Finalize();
            }

            foreach (var sb in AllSoftBodies)
                if (sb.solver != null) sb.solver.DeformMesh(sb.transform);
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

            solver = new Solver(0.01f, mesh, mesh.vertices, transform);

            if (solver.meshDeformer != null)
            {
                solver.meshDeformer.SetSkinningMethod(useAdvancedSkinning);
            }

            solver.internalPressure = internalPressure;

            if (truss != null)
            {
                ApplyTruss();
                if (matter != null)
                {
                    solver.SetMatterAsset(matter);
                }

                // ADD THIS LINE:
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

            solver.nodeMasses.Clear();

            if (truss != null && truss.NodeMasses != null && truss.NodeMasses.Count > 0)
            {
                // Apply the mass scale multiplier strictly to the runtime solver instance
                float scale = MassScale;
                int nodeCount = solver.nodeManager.Nodes.Count;

                for (int i = 0; i < nodeCount; i++)
                {
                    float originalMass = (i < truss.NodeMasses.Count) ? truss.NodeMasses[i] : 1f;
                    solver.nodeMasses.Add(Mathf.Max(0.01f, originalMass * scale));
                }
            }
            else
            {
                // Fallback behavior if no truss is assigned
                int nodeCount = solver.nodeManager.Nodes.Count;
                float baseMassPerNode = Mass / nodeCount;
                for (int i = 0; i < nodeCount; i++)
                {
                    solver.nodeMasses.Add(Mathf.Max(0.01f, baseMassPerNode));
                }
            }
        }

        private void ValidateParameters()
        {
            Mass = Mathf.Max(0.1f, Mass);
            internalPressure = Mathf.Max(0f, internalPressure);
        }
    }
}