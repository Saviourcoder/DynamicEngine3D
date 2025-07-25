/* NodeVisualizer - Visualizes nodes for all SoftBody components in the scene using LineRenderer circles
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
    public class NodeVisualizer : MonoBehaviour
    {
        [Header("Visualization Settings")]
        [SerializeField, Range(0.01f, 0.5f)] private float nodeDisplaySize = 0.05f;
        [SerializeField] private Color nodeColor = Color.blue;
        [SerializeField, Range(10, 50)] private int circleSegments = 20;

        private List<SoftBody> softBodies = new List<SoftBody>();
        private Dictionary<SoftBody, List<LineRenderer>> nodeRenderersMap = new Dictionary<SoftBody, List<LineRenderer>>();
        private Material lineMaterial;

        private void Awake()
        {
            Debug.Log("NodeVisualizer: Awake called.");
            Initialize();
        }

        private void Initialize()
        {
            Debug.Log("NodeVisualizer: Initializing.");
            // Find all SoftBody components in the scene
            softBodies.Clear();
            softBodies.AddRange(FindObjectsByType<SoftBody>(FindObjectsSortMode.None));
            if (softBodies.Count == 0)
            {
                Debug.LogWarning("NodeVisualizer: No SoftBody components found in the scene.");
                return;
            }
            Debug.Log($"NodeVisualizer: Found {softBodies.Count} SoftBody components.");

            InitializeLineRenderers();
        }

        private void InitializeLineRenderers()
        {
            Debug.Log("NodeVisualizer: Initializing LineRenderers for nodes.");
            ClearLineRenderers();

            // Create a simple unlit material for LineRenderers (visible in build)
            lineMaterial = new Material(Shader.Find("Unlit/Color"));
            lineMaterial.color = nodeColor;

            foreach (var softBody in softBodies)
            {
                if (softBody == null || softBody.solver == null || softBody.solver.nodeManager == null)
                {
                    Debug.LogWarning($"NodeVisualizer: Skipping SoftBody {softBody?.name ?? "null"} - core or nodeManager is null.");
                    continue;
                }

                List<LineRenderer> renderers = new List<LineRenderer>();
                for (int i = 0; i < softBody.solver.nodeManager.Nodes.Count; i++)
                {
                    if (softBody.solver.nodeManager.Nodes[i] == null) continue;

                    GameObject lineObj = new GameObject($"NodeVisual_{softBody.name}_{i}");
                    lineObj.transform.parent = softBody.transform; // Parent to the SoftBody
                    lineObj.transform.position = softBody.solver.nodeManager.Nodes[i].position;

                    LineRenderer renderer = lineObj.AddComponent<LineRenderer>();
                    renderer.material = lineMaterial;
                    renderer.startWidth = 0.01f;
                    renderer.endWidth = 0.01f;
                    renderer.positionCount = circleSegments + 1;
                    renderer.loop = true;
                    renderer.useWorldSpace = true;

                    UpdateCirclePoints(renderer, softBody.solver.nodeManager.Nodes[i].position);
                    renderer.gameObject.hideFlags = HideFlags.None; // Ensure visibility in build

                    renderers.Add(renderer);
                    Debug.Log($"NodeVisualizer: Created LineRenderer for node {i} on SoftBody {softBody.name} at position {softBody.solver.nodeManager.Nodes[i].position}.");
                }
                nodeRenderersMap[softBody] = renderers;
                Debug.Log($"NodeVisualizer: Initialized {renderers.Count} LineRenderers for SoftBody {softBody.name}.");
            }
        }

        private void UpdateCirclePoints(LineRenderer renderer, Vector3 center)
        {
            Vector3[] positions = new Vector3[circleSegments + 1];
            for (int i = 0; i <= circleSegments; i++)
            {
                float angle = i * 2f * Mathf.PI / circleSegments;
                Vector3 offset = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0) * nodeDisplaySize;
                // Orient circle to face camera (main camera or scene view camera)
                Camera cam = Camera.main != null ? Camera.main : Camera.current;
                if (cam != null)
                {
                    offset = cam.transform.TransformDirection(offset);
                }
                positions[i] = center + offset;
            }
            renderer.SetPositions(positions);
        }

        private void UpdateVisualization()
        {
            Debug.Log("NodeVisualizer: Updating visualization.");
            // Check for new or removed SoftBodies
            var currentSoftBodies = new List<SoftBody>(FindObjectsByType<SoftBody>(FindObjectsSortMode.None));
            if (currentSoftBodies.Count != softBodies.Count || !currentSoftBodies.TrueForAll(s => softBodies.Contains(s)))
            {
                Debug.Log("NodeVisualizer: SoftBody list changed, reinitializing.");
                Initialize();
                return;
            }

            if (lineMaterial != null && lineMaterial.color != nodeColor)
            {
                lineMaterial.color = nodeColor;
                Debug.Log($"NodeVisualizer: Updated node color to {nodeColor}.");
            }

            foreach (var softBody in softBodies)
            {
                if (softBody == null || softBody.solver == null || softBody.solver.nodeManager == null)
                {
                    Debug.LogWarning($"NodeVisualizer: Skipping update for SoftBody {softBody?.name ?? "null"} - core or nodeManager is null.");
                    continue;
                }

                if (!nodeRenderersMap.ContainsKey(softBody) || nodeRenderersMap[softBody].Count != softBody.solver.nodeManager.Nodes.Count)
                {
                    Debug.LogWarning($"NodeVisualizer: Node count mismatch for SoftBody {softBody.name}, reinitializing LineRenderers.");
                    InitializeLineRenderers();
                    return;
                }

                var renderers = nodeRenderersMap[softBody];
                for (int i = 0; i < renderers.Count; i++)
                {
                    if (renderers[i] == null || softBody.solver.nodeManager.Nodes[i] == null)
                    {
                        Debug.LogWarning($"NodeVisualizer: Skipping update for node {i} on SoftBody {softBody.name} - renderer or node is null.");
                        continue;
                    }

                    UpdateCirclePoints(renderers[i], softBody.solver.nodeManager.Nodes[i].position);
                    Debug.Log($"NodeVisualizer: Updated node {i} on SoftBody {softBody.name} to position {softBody.solver.nodeManager.Nodes[i].position}.");
                }
            }
        }

        private void ClearLineRenderers()
        {
            Debug.Log("NodeVisualizer: Clearing LineRenderers.");
            foreach (var renderers in nodeRenderersMap.Values)
            {
                foreach (var renderer in renderers)
                {
                    if (renderer != null)
                    {
#if UNITY_EDITOR
                        if (!Application.isPlaying)
                            Undo.DestroyObjectImmediate(renderer.gameObject);
                        else
#endif
                            Destroy(renderer.gameObject);
                    }
                }
            }
            nodeRenderersMap.Clear();
            if (lineMaterial != null)
            {
                Destroy(lineMaterial);
                lineMaterial = null;
            }
            Debug.Log("NodeVisualizer: LineRenderers cleared.");
        }

        private void Update()
        {
            if (softBodies.Count == 0)
            {
                Debug.LogWarning("NodeVisualizer: No SoftBodies to visualize.");
                return;
            }
            UpdateVisualization();
        }

        private void OnDestroy()
        {
            Debug.Log("NodeVisualizer: OnDestroy called, cleaning up.");
            ClearLineRenderers();
        }

        private void OnDisable()
        {
            Debug.Log("NodeVisualizer: OnDisable called, cleaning up.");
            ClearLineRenderers();
        }

        private void OnValidate()
        {
            if (nodeDisplaySize < 0.01f) nodeDisplaySize = 0.01f;
            if (circleSegments < 10) circleSegments = 10;
            Debug.Log($"NodeVisualizer: Validated parameters - nodeDisplaySize: {nodeDisplaySize}, circleSegments: {circleSegments}");
            UpdateVisualization();
        }
    }
}