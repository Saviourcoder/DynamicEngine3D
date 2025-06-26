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
        // Material presets for different soft body types
        private static readonly Dictionary<MaterialType, MaterialProperties> MaterialPresets = new Dictionary<MaterialType, MaterialProperties>
        {
            { MaterialType.Metal, new MaterialProperties(2.0f, 1e-6f, 0.005f, 0.01f, 0.01f, 0.05f, 0.1f) },
            { MaterialType.Plastic, new MaterialProperties(1.0f, 1e-4f, 0.01f, 0.05f, 0.05f, 0.03f, 0.2f) },
            { MaterialType.Rubber, new MaterialProperties(0.5f, 1e-3f, 0.3f, 0.1f, 0.15f, 0.1f, 0.05f) }
        };

        [Header("Material Settings")]
        [SerializeField, Tooltip("Material type for the soft body (use Custom for manual settings)")]
        private MaterialType materialType = MaterialType.Rubber;
        [SerializeField, Tooltip("Custom material properties when Material Type is set to Custom")]
        private MaterialProperties customMaterialProps = new MaterialProperties(0.5f, 1e-5f, 0.1f, 0.1f, 0.5f, 0.05f, 0.1f);

        [Header("Physics Settings")]
        [SerializeField, Range(0.01f, 0.5f), Tooltip("Radius of nodes for collision detection")]
        private float nodeRadius = 0.05f;
        [SerializeField, Range(0.5f, 5f), Tooltip("Radius for vertex influence during mesh deformation")]
        private float influenceRadius = 1f;
        [SerializeField, Range(0.5f, 5f), Tooltip("Maximum distance for connecting nodes with beams")]
        private float beamConnectionDistance = 1.5f;

        [Header("Visualization")]
        [SerializeField, Tooltip("Show node colliders, beams, and collision points in Gizmos")]
        public bool showGizmos = true;

        [Header("Node Design")]
        [SerializeField, Tooltip("Truss asset defining nodes and beams")]
        public TrussAsset trussAsset;

        private MeshFilter meshFilter;
        private Mesh mesh;
        private SoftBodyCore core;
        private Camera mainCamera;
        private bool isDragging = false;
        private Plane dragPlane;
        private int draggedNodeIndex = -1;

        private MaterialProperties MaterialProps => materialType == MaterialType.Custom ? customMaterialProps : MaterialPresets[materialType];

        void Awake()
        {
            mainCamera = Camera.main;
            Initialize();
        }

        void OnValidate()
        {
            ApplyMaterialProperties();
            ValidateParameters();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        private void ApplyMaterialProperties()
        {
            if (materialType != MaterialType.Custom && MaterialPresets.TryGetValue(materialType, out MaterialProperties props))
            {
                customMaterialProps = props;
            }
        }

        public TrussAsset GetTrussAsset()
        {
            return trussAsset;
        }

        public void ApplyTruss()
        {
            if (trussAsset == null)
            {
                Debug.LogWarning("No TrussAsset assigned to SoftBody.", this);
                return;
            }

            if (core == null)
            {
                Debug.LogError("SoftBodyCore instance not initialized.", this);
                return;
            }

            Vector3[] positions = trussAsset.NodePositions;
            if (positions == null || positions.Length < 2)
            {
                Debug.LogWarning("TrussAsset has insufficient or invalid node positions.", this);
                return;
            }

            var beams = trussAsset.GetBeams();
            if (beams == null || beams.Length == 0)
            {
                Debug.LogWarning("TrussAsset has no beams defined.", this);
                return;
            }

            core.GenerateNodesAndBeams(positions, beams, transform);
        }

        private void ValidateParameters()
        {
            if (nodeRadius < 0.01f)
            {
                Debug.LogWarning("Node Radius is very small; collisions may be unreliable.", this);
                nodeRadius = 0.01f;
            }
            if (beamConnectionDistance < nodeRadius * 2f)
            {
                Debug.LogWarning("Beam Connection Distance is too small relative to Node Radius; beams may be skipped.", this);
                beamConnectionDistance = nodeRadius * 2f;
            }
        }

        void Update()
        {
            HandleMouseInteraction();
        }

        void FixedUpdate()
        {
            if (core != null)
            {
                core.Solve();
                core.DeformMesh(transform);
            }
        }

        void OnDestroy()
        {
            core?.nodeManager.Clear();
        }

        private bool Initialize()
        {
            if (!SetupMesh())
            {
                Debug.LogError("Failed to initialize soft body: MeshFilter or mesh missing!", this);
                return false;
            }

            core = new SoftBodyCore(nodeRadius, influenceRadius, MaterialProps, mesh, mesh.vertices);

            if (trussAsset != null)
            {
                ApplyTruss();
            }
            else
            {
                core.GenerateCubeTest(transform);
            }

            return true;
        }

        private bool SetupMesh()
        {
            meshFilter = GetComponent<MeshFilter>();
            if (!meshFilter || meshFilter.sharedMesh == null)
            {
                return false;
            }

            mesh = Application.isPlaying ? (meshFilter.mesh = Instantiate(meshFilter.sharedMesh)) : meshFilter.sharedMesh;
            return true;
        }

        public void ApplyTrussAsset(TrussAsset truss)
        {
            if (truss == null)
            {
                Debug.LogWarning("Cannot apply null TrussAsset.", this);
                return;
            }
            trussAsset = truss;
            ApplyTruss();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        private void HandleMouseInteraction()
        {
            if (mainCamera == null || core == null || core.nodeManager == null)
            {
                Debug.LogWarning("Main camera or core not initialized, skipping mouse interaction.", this);
                return;
            }

            Vector3 mousePos = Input.mousePosition;
            if (mousePos.x < 0 || mousePos.y < 0 ||
                mousePos.x > Screen.width || mousePos.y > Screen.height ||
                float.IsNaN(mousePos.x) || float.IsNaN(mousePos.y))
            {
                Debug.LogWarning("Mouse position out of valid screen bounds, skipping interaction.", this);
                return;
            }

            Ray ray = mainCamera.ScreenPointToRay(mousePos);

            if (Input.GetMouseButtonDown(0))
            {
                draggedNodeIndex = core.FindClosestNodeToRay(ray, 100f);
                if (draggedNodeIndex >= 0)
                {
                    isDragging = true;
                    dragPlane = new Plane(mainCamera.transform.forward, core.nodeManager.Nodes[draggedNodeIndex].position);
                }
            }

            if (Input.GetMouseButtonUp(0))
            {
                isDragging = false;
                draggedNodeIndex = -1;
            }

            if (isDragging && draggedNodeIndex >= 0)
            {
                if (dragPlane.Raycast(ray, out float distance))
                {
                    Vector3 targetPos = ray.GetPoint(distance);
                    core.ApplyImpulse(draggedNodeIndex, targetPos, 10f);
                }
            }
        }

#if UNITY_EDITOR
        void OnDrawGizmos()
        {
            if (!showGizmos || core == null || core.nodeManager == null)
                return;

            Gizmos.color = Color.blue;
            foreach (var node in core.nodeManager.Nodes)
            {
                if (node != null)
                {
                    Gizmos.DrawWireSphere(node.position, core.nodeManager.NodeRadius);
                }
            }

            Gizmos.color = Color.yellow;
            foreach (var beam in core.beams)
            {
                if (beam.nodeA >= 0 && beam.nodeA < core.nodeManager.Nodes.Count &&
                    beam.nodeB >= 0 && beam.nodeB < core.nodeManager.Nodes.Count)
                {
                    Transform nodeA = core.nodeManager.Nodes[beam.nodeA];
                    Transform nodeB = core.nodeManager.Nodes[beam.nodeB];
                    if (nodeA != null && nodeB != null)
                    {
                        Gizmos.DrawLine(nodeA.position, nodeB.position);
                    }
                }
            }

            Gizmos.color = Color.green;
            foreach (var point in core.collisionPoints)
            {
                Gizmos.DrawWireSphere(point, core.nodeManager.NodeRadius * 0.5f);
            }
        }
#endif
    }
}
