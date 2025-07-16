/* DynamicEngine3D - Soft Body Simulation
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
using DynamicEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    public class SoftBody : MonoBehaviour
    {
        // ──────────────────────────────────────────────────────────
        //  Material presets (unchanged)
        // ──────────────────────────────────────────────────────────
        private static readonly Dictionary<MaterialType, MaterialProperties> MaterialPresets =
            new Dictionary<MaterialType, MaterialProperties>
            {
                { MaterialType.Metal,   new MaterialProperties(2.0f, 1e-6f, 0.005f, 0.01f, 0.01f, 0.05f, 0.1f) },
                { MaterialType.Plastic, new MaterialProperties(1.0f, 1e-4f, 0.01f, 0.05f, 0.05f, 0.03f, 0.2f) },
                { MaterialType.Rubber,  new MaterialProperties(0.5f, 1e-3f, 0.3f, 0.1f, 0.15f, 0.1f, 0.05f) }
            };

        [Header("Material Settings")]
        [SerializeField] private MaterialType materialType = MaterialType.Rubber;
        [SerializeField] private MaterialProperties customMaterialProps = new MaterialProperties(0.5f, 1e-5f, 0.1f, 0.1f, 0.5f, 0.05f, 0.1f);

        [Header("Physics Settings")]
        [SerializeField, Range(0.01f, 0.5f)] private float nodeRadius = 0.05f;
        [SerializeField, Range(0.5f, 5f)]   private float influenceRadius = 1f;
        [SerializeField, Range(0.5f, 5f)]   private float beamConnectionDistance = 1.5f;

        [Header("Visualization")]
        [SerializeField] public bool showGizmos = true;

        [Header("Node Design")]
        [SerializeField] public TrussAsset trussAsset;

        private MeshFilter meshFilter;
        private Mesh mesh;
        private SoftBodyCore core;
        private Camera mainCamera;
        private bool isDragging = false;
        private Plane dragPlane;
        private int draggedNodeIndex = -1;

        private MaterialProperties MaterialProps =>
            materialType == MaterialType.Custom ? customMaterialProps : MaterialPresets[materialType];

        // ──────────────────────────────────────────────────────────
        //  Unity messages
        // ──────────────────────────────────────────────────────────
        private void Awake()  => InitializeInPlayMode();
        private void OnValidate()
        {
            ApplyMaterialProperties();
            ValidateParameters();
            InitializeInEditMode();
        }

        private void InitializeInPlayMode()
        {
            mainCamera = Camera.main;
            if (Application.isPlaying) InitializeCore();
        }

        private void InitializeInEditMode()
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
            if (!SetupMesh()) return false;

            core = new SoftBodyCore(nodeRadius, influenceRadius, MaterialProps, mesh, mesh.vertices);

            if (trussAsset != null) ApplyTruss();
            else                    core.GenerateCubeTest(transform);

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

        // ──────────────────────────────────────────────────────────
        //  Public helpers
        // ──────────────────────────────────────────────────────────
        
        public TrussAsset GetTrussAsset()
        {
            return trussAsset;
        } 
        
        public void ApplyTruss()
        {
            if (trussAsset == null) { Debug.LogWarning("No TrussAsset assigned.", this); return; }

            var positions = trussAsset.NodePositions;
            var beams = trussAsset.GetBeams();

            if (positions == null || positions.Length < 2) { Debug.LogWarning("Invalid node positions.", this); return; }
            if (beams == null || beams.Length < 1) { Debug.LogWarning("No beams defined.", this); return; }

            core.GenerateNodesAndBeams(positions, beamsArray: beams, parent: transform);
        }

        public void ApplyTrussAsset(TrussAsset asset)
        {
            if (asset == null) { Debug.LogWarning("Cannot apply null TrussAsset.", this); return; }
            trussAsset = asset;
            ApplyTruss();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        // ──────────────────────────────────────────────────────────
        //  Unity lifecycle
        // ──────────────────────────────────────────────────────────
        void Update()   => HandleMouseInteraction();
        void FixedUpdate()
        {
            if (core != null) { core.Solve(); core.DeformMesh(transform); }
        }
        void OnDestroy() => core?.nodeManager.Clear();

        // ──────────────────────────────────────────────────────────
        //  Utilities
        // ──────────────────────────────────────────────────────────
        private void ApplyMaterialProperties()
        {
            if (materialType != MaterialType.Custom && MaterialPresets.TryGetValue(materialType, out var props))
                customMaterialProps = props;
        }

        private void ValidateParameters()
        {
            if (nodeRadius < 0.01f) nodeRadius = 0.01f;
            if (beamConnectionDistance < nodeRadius * 2f)
                beamConnectionDistance = nodeRadius * 2f;
        }

        // ──────────────────────────────────────────────────────────
        //  Mouse interaction & gizmos (unchanged)
        // ──────────────────────────────────────────────────────────
        #region Mouse & Gizmos
        private void HandleMouseInteraction()
        {
            if (mainCamera == null || core == null) return;

            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);

            if (Input.GetMouseButtonDown(0))
            {
                draggedNodeIndex = core.FindClosestNodeToRay(ray, 100f);
                if (draggedNodeIndex >= 0)
                {
                    isDragging = true;
                    dragPlane = new Plane(mainCamera.transform.forward,
                                          core.nodeManager.Nodes[draggedNodeIndex].position);
                }
            }

            if (Input.GetMouseButtonUp(0)) { isDragging = false; draggedNodeIndex = -1; }

            if (isDragging && draggedNodeIndex >= 0 &&
                dragPlane.Raycast(ray, out float dist))
            {
                core.ApplyImpulse(draggedNodeIndex, ray.GetPoint(dist), 10f);
            }
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (!showGizmos || core == null) return;

            Gizmos.color = Color.blue;
            foreach (var n in core.nodeManager.Nodes)
                if (n != null) Gizmos.DrawWireSphere(n.position, core.nodeManager.NodeRadius);

            Gizmos.color = Color.yellow;
            foreach (var b in core.beams)
            {
                if (b.nodeA >= 0 && b.nodeA < core.nodeManager.Nodes.Count &&
                    b.nodeB >= 0 && b.nodeB < core.nodeManager.Nodes.Count)
                {
                    var a = core.nodeManager.Nodes[b.nodeA];
                    var bPos = core.nodeManager.Nodes[b.nodeB];
                    if (a != null && bPos != null) Gizmos.DrawLine(a.position, bPos.position);
                }
            }

            Gizmos.color = Color.green;
            foreach (var p in core.collisionPoints)
                Gizmos.DrawWireSphere(p, core.nodeManager.NodeRadius * 0.5f);
        }
#endif
        #endregion
    }
}
