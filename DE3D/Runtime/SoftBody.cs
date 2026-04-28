/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */

using UnityEngine;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Collections;
using System.Linq;
using Unity.Mathematics;

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
            AllSoftBodies.Clear();
            Application.quitting += () => s_isApplicationQuitting = true;
        }

        [SerializeField] public Truss truss;
        [SerializeField] public Matter matter;

        public static HashSet<SoftBody> AllSoftBodies = new HashSet<SoftBody>();
        [SerializeField, Range(0.1f, 5000f)] private float Mass = 1f;

        [Header("Pressure Settings")]
        [SerializeField, Range(0f, 1000f)] private float internalPressure = 0f;

        [Header("Deformation Settings")]
        [SerializeField] private bool useAdvancedSkinning = true;
        [SerializeField] public float influenceRadius = 0.5f;

        // ── Visualization ──────────────────────────────────────────────────────
        // Controlled via the custom inspector foldout; drives both Game and Scene views.
        [SerializeField] public bool vizNodes = false;
        [SerializeField] public bool vizBeams = false;
        [SerializeField] public bool vizPinnedNodes = true;
        [SerializeField] public bool vizInactiveBeams = false;
        [SerializeField] public Color vizNodeColor = new Color(0.2f, 0.8f, 1f, 1f);
        [SerializeField] public Color vizBeamColor = new Color(0.9f, 0.9f, 0.2f, 1f);
        [SerializeField] public Color vizPinnedColor = new Color(1f, 0.3f, 0.3f, 1f);
        [SerializeField] public Color vizInactiveColor = new Color(0.5f, 0.5f, 0.5f, 0.4f);
        [SerializeField] public Color vizCrossBodyColor = new Color(0.2f, 1f, 0.5f, 0.8f);
        [SerializeField, Range(0.005f, 0.3f)] public float vizNodeSize = 0.04f;

        private Material _vizMaterial;
        // Cached pinned lookup rebuilt whenever a truss is applied.
        private bool[] _pinnedLookup;
        // ──────────────────────────────────────────────────────────────────────

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
            // Guard first before initializing anything
            if (truss == null)
            {
                Debug.LogError("No Truss assigned!", this);
                enabled = false;
                return;
            }
            InitializeInPlayMode();
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
            // Check if this is the first valid body in the set
            if (AllSoftBodies.Count == 0 || AllSoftBodies.First() != this) return;
            RunGlobalPhysicsStep();
        }

        private static void RunGlobalPhysicsStep()
        {
            if (SceneSettings.Instance == null) return;

            int numSubSteps = Mathf.Max(1, SceneSettings.Instance.SubstepCount);
            float fullDt = Time.fixedDeltaTime * SceneSettings.Instance.TimeScale;
            float subDt = fullDt / numSubSteps;

            // Snap a copy to avoid mutation exceptions if objects are destroyed mid-step
            var activeBodies = AllSoftBodies.Where(sb => sb != null && sb.solver != null).ToList();

            for (int step = 0; step < numSubSteps; step++)
            {
                // Apply pre-integration forces
                foreach (var sb in activeBodies) sb.solver.ApplyPressure(subDt);
                foreach (var sb in activeBodies) sb.solver.ApplyBeamDampingForces(subDt);

                foreach (var sb in activeBodies) sb.solver.Integrate(subDt);
                foreach (var sb in activeBodies) sb.solver.SolveConstraints(subDt);

                // Apply post-constraint modifications
                foreach (var sb in activeBodies) sb.solver.ApplyPlasticityStep(subDt);
                foreach (var sb in activeBodies) sb.solver.CheckAndBreakConstraints(subDt);

                foreach (var sb in activeBodies) sb.solver.ResolveCollisions(subDt);
                foreach (var sb in activeBodies) sb.solver.FinalizePositions();
            }

            // Pass 1: Update transforms parents-first so children don't get dragged
            foreach (var sb in activeBodies.OrderBy(sb => GetTransformDepth(sb.transform)))
            {
                sb.UpdateTransformToCenter();
            }

            // Pass 2: Deform meshes — all transforms are now stable
            foreach (var sb in activeBodies)
            {
                sb.solver.DeformMesh(sb.transform);
            }
        }

        private static int GetTransformDepth(Transform t)
        {
            int depth = 0;
            while (t.parent != null) { depth++; t = t.parent; }
            return depth;
        }

        private void UpdateTransformToCenter()
        {
            if (solver == null || solver.nodeManager == null || solver.nodeManager.NodeCount == 0) return;

            Vector3 center = Vector3.zero;
            for (int i = 0; i < solver.nodeManager.NodeCount; i++)
                center += solver.nodeManager.GetPosition(i);

            transform.position = center / solver.nodeManager.NodeCount;
        }

        // ── Runtime GL Visualization ───────────────────────────────────────────
        // Called by Unity after each camera finishes rendering — works in both
        // the Game view and the Scene view without any editor-only dependencies.
        private void OnRenderObject()
        {
            if (solver == null || solver.nodeManager == null || solver.nodeManager.NodeCount == 0) return;
            if (!vizNodes && !vizBeams) return;

            EnsureVizMaterial();

            var nm = solver.nodeManager;

            // Build a world-space MVP that maps world positions directly to clip space.
            // LoadProjectionMatrix * LoadIdentity is the standard GL world-space pattern.
            _vizMaterial.SetPass(0);
            GL.PushMatrix();
            GL.LoadProjectionMatrix(Camera.current.projectionMatrix * Camera.current.worldToCameraMatrix);
            GL.LoadIdentity();

            // ── Beams ──────────────────────────────────────────────────────────
            if (vizBeams && solver.beams != null && solver.beams.Count > 0)
            {
                GL.Begin(GL.LINES);
                foreach (var beam in solver.beams)
                {
                    if (!beam.isActive && !vizInactiveBeams) continue;

                    if (!beam.isActive)
                        GL.Color(vizInactiveColor);
                    else if (beam.IsCrossBody)
                        GL.Color(vizCrossBodyColor);
                    else
                        GL.Color(vizBeamColor);

                    GL.Vertex(nm.GetPosition(beam.nodeA));
                    GL.Vertex(nm.GetPosition(beam.nodeB));
                }
                GL.End();
            }

            // ── Nodes — drawn as small 3-axis crosses ──────────────────────────
            if (vizNodes)
            {
                float h = vizNodeSize * 0.5f;
                GL.Begin(GL.LINES);
                for (int i = 0; i < nm.NodeCount; i++)
                {
                    bool pinned = vizPinnedNodes && _pinnedLookup != null && i < _pinnedLookup.Length && _pinnedLookup[i];
                    GL.Color(pinned ? vizPinnedColor : vizNodeColor);

                    Vector3 p = nm.GetPosition(i);

                    GL.Vertex(p - new Vector3(h, 0, 0)); GL.Vertex(p + new Vector3(h, 0, 0)); // X
                    GL.Vertex(p - new Vector3(0, h, 0)); GL.Vertex(p + new Vector3(0, h, 0)); // Y
                    GL.Vertex(p - new Vector3(0, 0, h)); GL.Vertex(p + new Vector3(0, 0, h)); // Z
                }
                GL.End();
            }

            GL.PopMatrix();
        }

        private void EnsureVizMaterial()
        {
            if (_vizMaterial != null) return;
            // Hidden/Internal-Colored supports vertex colours and depth testing,
            // making it ideal for debug overlays.
            _vizMaterial = new Material(Shader.Find("Hidden/Internal-Colored"))
            {
                hideFlags = HideFlags.HideAndDontSave
            };
            _vizMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            _vizMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            _vizMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            _vizMaterial.SetInt("_ZWrite", 0);
        }
        // ──────────────────────────────────────────────────────────────────────

        void OnDestroy()
        {
            solver?.Cleanup();
            solver?.nodeManager.Clear();

            if (_vizMaterial != null)
            {
                if (Application.isPlaying) Destroy(_vizMaterial);
                else DestroyImmediate(_vizMaterial);
            }
        }

        private void InitializeInPlayMode()
        {
            if (Application.isPlaying)
                InitializeCore();
        }

        private bool InitializeCore()
        {
            if (!SetupMesh()) return false;

          Vector3[] vertices = mesh.vertices;

    // Convert Vector3[] -> float3[]
    float3[] floatVertices = new float3[vertices.Length];

    for (int i = 0; i < vertices.Length; i++)
    {
        floatVertices[i] = new float3(vertices[i].x, vertices[i].y, vertices[i].z);
    }

    // Pass converted data
    solver = new Solver(influenceRadius, mesh, floatVertices, transform);

            if (solver.meshDeformer != null)
                solver.meshDeformer.SetSkinningMethod(useAdvancedSkinning);

            solver.internalPressure = internalPressure;

            if (truss != null)
            {
                ApplyTruss();
                if (matter != null)
                    solver.SetMatterAsset(matter);

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
                solver.SetMatterAsset(matter);
        }

        private void ApplyPinnedNodesFromTruss()
        {
            if (truss == null || solver == null) return;

            int count = solver.nodeManager.NodeCount;
            var pinnedList = new List<bool>(new bool[count]);
            _pinnedLookup = new bool[count];

            foreach (int idx in truss.PinnedNodes)
            {
                if (idx >= 0 && idx < count)
                {
                    pinnedList[idx] = true;
                    _pinnedLookup[idx] = true;
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
                solver.nodeMasses.Add(massPerNode);
        }

        private void ValidateParameters()
        {
            Mass = Mathf.Max(0.1f, Mass);
            internalPressure = Mathf.Max(0f, internalPressure);
        }
    }
}