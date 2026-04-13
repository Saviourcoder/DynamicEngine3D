/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */

using UnityEngine;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Collections;

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

        [Header("Inspector")]
        [SerializeField] public Texture2D inspectorBanner;
        public static List<SoftBody> AllSoftBodies = new List<SoftBody>();
        [SerializeField, Range(0.1f, 5000f)] private float Mass = 1f;

        [Header("Pressure Settings")]
        [SerializeField, Range(0f, 1000f)] private float internalPressure = 0f;

        [Header("Deformation Settings")]
        [SerializeField] private bool useAdvancedSkinning = true;
        [SerializeField] public float influenceRadius = 0.5f;

        private MeshFilter meshFilter;
        private Mesh mesh;
        public Solver solver;

        public float TotalMass
        {
            get => Mass;
            set
            {
                Mass = Mathf.Max(0.1f, value);
                UpdateMassDistribution();
            }
        }

        public float InternalPressure
        {
            get => internalPressure;
            set
            {
                internalPressure = Mathf.Max(0f, value);
                if (solver != null) solver.internalPressure = internalPressure;
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
                    if (solver.nodeManager.NodeCount > 0)
                    {
                        solver.meshDeformer.MapVerticesToNodes(transform, solver.nodeManager.CurrentPositions, solver.nodeManager.InitialPositions);
                    }
                }
            }
        }

        void OnEnable() => AllSoftBodies.Add(this);
        void OnDisable() => AllSoftBodies.Remove(this);

        public Bounds GetBounds() => meshFilter.mesh.bounds;

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
                UpdateMassDistribution();
            }
        }

        private void FixedUpdate()
        {
            if (AllSoftBodies.Count == 0 || AllSoftBodies[0] != this) return;
            RunGlobalPhysicsStep();
        }

        private static void RunGlobalPhysicsStep()
        {
            if (SceneSettings.Instance == null) return;

            int numSubSteps = Mathf.Max(1, SceneSettings.Instance.SubstepCount);
            float fullDt = Time.fixedDeltaTime * SceneSettings.Instance.TimeScale;
            float subDt = fullDt / numSubSteps;

            for (int step = 0; step < numSubSteps; step++)
            {
                foreach (var sb in AllSoftBodies)
                    if (sb.solver != null) sb.solver.Integrate(subDt);

                foreach (var sb in AllSoftBodies)
                    if (sb.solver != null) sb.solver.SolveConstraints(subDt);

                foreach (var sb in AllSoftBodies)
                    if (sb.solver != null) sb.solver.ApplyPlasticityStep(subDt);

                foreach (var sb in AllSoftBodies)
                    if (sb.solver != null) sb.solver.ResolveCollisions(subDt);

                foreach (var sb in AllSoftBodies)
                    if (sb.solver != null) sb.solver.FinalizePositions();
            }

            foreach (var sb in AllSoftBodies)
            {
                if (sb.solver != null)
                {
                    sb.UpdateTransformToCenter();
                    sb.solver.DeformMesh(sb.transform);
                }
            }
        }

        private void UpdateTransformToCenter()
        {
            if (solver == null || solver.nodeManager == null || solver.nodeManager.NodeCount == 0) return;

            Vector3 center = Vector3.zero;
            for (int i = 0; i < solver.nodeManager.NodeCount; i++)
            {
                center += solver.nodeManager.GetPosition(i);
            }
            transform.position = center / solver.nodeManager.NodeCount;
        }

        void OnDestroy()
        {
            solver?.Cleanup();
            solver?.nodeManager.Clear();
        }

        private void InitializeInPlayMode()
        {
            if (Application.isPlaying)
                InitializeCore();
        }

        private bool InitializeCore()
        {
            if (!SetupMesh()) return false;

            solver = new Solver(influenceRadius, mesh, mesh.vertices, transform);

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
                UpdateMassDistribution();
            }

            return true;
        }

        private bool SetupMesh()
        {
            meshFilter = GetComponent<MeshFilter>();
            if (!meshFilter || meshFilter.sharedMesh == null) return false;

            mesh = Application.isPlaying
                ? (meshFilter.mesh = Instantiate(meshFilter.sharedMesh))
                : meshFilter.sharedMesh;

            return true;
        }

        public Solver GetSolver() => solver;
        public Truss GetTrussAsset() => truss;
        public Matter GetMatterAsset() => matter;

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

            if (s_isApplicationQuitting || gameObject == null) return;

            truss = asset;
            ApplyTruss();

            if (matter != null && solver != null)
            {
                solver.SetMatterAsset(matter);
            }
        }

        private void ApplyPinnedNodesFromTruss()
        {
            if (truss == null || solver == null) return;

            var pinnedList = new List<bool>(new bool[solver.nodeManager.NodeCount]);
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
            if (solver == null || solver.nodeManager.NodeCount == 0) return;

            solver.nodeMasses.Clear();
            int nodeCount = solver.nodeManager.NodeCount;
            float massPerNode = Mathf.Max(0.01f, Mass / nodeCount);

            for (int i = 0; i < nodeCount; i++)
            {
                solver.nodeMasses.Add(massPerNode);
            }
        }

        private void ValidateParameters()
        {
            Mass = Mathf.Max(0.1f, Mass);
            internalPressure = Mathf.Max(0f, internalPressure);
        }
    }
}