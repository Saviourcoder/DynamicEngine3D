/* DynamicEngine3D - Soft Body Visualizer
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/

using UnityEngine;
using UnityEngine.UI;
using DynamicEngine;

namespace DynamicEngine
{
    /// <summary>
    /// Runtime UI controller for soft body visualization
    /// </summary>
    public class SoftBodyVisualizer : MonoBehaviour
    {
        [Header("Solver Reference")]
        public Solver solver;
        
        [Header("UI Controls (Optional)")]
        public Toggle showNodesToggle;
        public Toggle showLinksToggle;
        public Toggle showIndicesToggle;
        public Toggle showForcesToggle;
        public Toggle showInfluenceRadiusToggle;
        public Slider nodeSizeSlider;
        public Text statsText;
        
        [Header("Keyboard Controls")]
        public KeyCode toggleVisualizationKey = KeyCode.V;
        public KeyCode toggleNodesKey = KeyCode.N;
        public KeyCode toggleLinksKey = KeyCode.L;
        public KeyCode toggleIndicesKey = KeyCode.I;
        public KeyCode toggleForcesKey = KeyCode.F;
        public KeyCode toggleInfluenceRadiusKey = KeyCode.R;
        
        [Header("Auto-Find Solver")]
        public bool autoFindSolver = true;
        
        private NodeLinkEditor nodeLinkEditor;
        
        void Start()
        {
            if (autoFindSolver && solver == null)
            {
                nodeLinkEditor = FindFirstObjectByType<NodeLinkEditor>();
                if (nodeLinkEditor != null)
                {
                    solver = nodeLinkEditor.GetSolver();
                }
            }
            
            SetupUICallbacks();
            UpdateUIFromSolver();
        }
        
        void Update()
        {
            HandleKeyboardInput();
            UpdateStats();
        }
        
        private void HandleKeyboardInput()
        {
            if (solver == null) return;
            
            if (Input.GetKeyDown(toggleVisualizationKey))
            {
                solver.ToggleVisualization();
                UpdateUIFromSolver();
            }
            
            if (Input.GetKeyDown(toggleNodesKey))
            {
                solver.showNodes = !solver.showNodes;
                UpdateUIFromSolver();
            }
            
            if (Input.GetKeyDown(toggleLinksKey))
            {
                solver.showLinks = !solver.showLinks;
                UpdateUIFromSolver();
            }
            
            if (Input.GetKeyDown(toggleIndicesKey))
            {
                solver.showNodeIndices = !solver.showNodeIndices;
                UpdateUIFromSolver();
            }
            
            if (Input.GetKeyDown(toggleForcesKey))
            {
                solver.showLinkForces = !solver.showLinkForces;
                UpdateUIFromSolver();
            }
            
            if (Input.GetKeyDown(toggleInfluenceRadiusKey))
            {
                solver.showInfluenceRadius = !solver.showInfluenceRadius;
                UpdateUIFromSolver();
            }
        }
        
        private void SetupUICallbacks()
        {
            if (showNodesToggle != null)
                showNodesToggle.onValueChanged.AddListener(OnShowNodesChanged);
                
            if (showLinksToggle != null)
                showLinksToggle.onValueChanged.AddListener(OnShowLinksChanged);
                
            if (showIndicesToggle != null)
                showIndicesToggle.onValueChanged.AddListener(OnShowIndicesChanged);
                
            if (showForcesToggle != null)
                showForcesToggle.onValueChanged.AddListener(OnShowForcesChanged);
                
            if (showInfluenceRadiusToggle != null)
                showInfluenceRadiusToggle.onValueChanged.AddListener(OnShowInfluenceRadiusChanged);
                
            if (nodeSizeSlider != null)
                nodeSizeSlider.onValueChanged.AddListener(OnNodeSizeChanged);
        }
        
        private void UpdateUIFromSolver()
        {
            if (solver == null) return;
            
            if (showNodesToggle != null)
                showNodesToggle.isOn = solver.showNodes;
                
            if (showLinksToggle != null)
                showLinksToggle.isOn = solver.showLinks;
                
            if (showIndicesToggle != null)
                showIndicesToggle.isOn = solver.showNodeIndices;
                
            if (showForcesToggle != null)
                showForcesToggle.isOn = solver.showLinkForces;
                
            if (showInfluenceRadiusToggle != null)
                showInfluenceRadiusToggle.isOn = solver.showInfluenceRadius;
                
            if (nodeSizeSlider != null)
                nodeSizeSlider.value = solver.nodeDisplaySize;
        }
        
        private void UpdateStats()
        {
            if (solver == null || statsText == null) return;
            
            string stats = solver.GetVisualizationStats();
            string controls = $"\nControls: {toggleVisualizationKey}=Toggle | {toggleNodesKey}=Nodes | {toggleLinksKey}=Links | {toggleIndicesKey}=Indices | {toggleForcesKey}=Forces | {toggleInfluenceRadiusKey}=Influence";
            statsText.text = stats + controls;
        }
        
        // UI Callback Methods
        private void OnShowNodesChanged(bool value)
        {
            if (solver != null) solver.showNodes = value;
        }
        
        private void OnShowLinksChanged(bool value)
        {
            if (solver != null) solver.showLinks = value;
        }
        
        private void OnShowIndicesChanged(bool value)
        {
            if (solver != null) solver.showNodeIndices = value;
        }
        
        private void OnShowForcesChanged(bool value)
        {
            if (solver != null) solver.showLinkForces = value;
        }
        
        private void OnShowInfluenceRadiusChanged(bool value)
        {
            if (solver != null) solver.showInfluenceRadius = value;
        }
        
        private void OnNodeSizeChanged(float value)
        {
            if (solver != null) solver.nodeDisplaySize = value;
        }
        
        // Public methods for external control
        public void ToggleVisualization()
        {
            if (solver != null)
            {
                solver.ToggleVisualization();
                UpdateUIFromSolver();
            }
        }
        
        public void SetVisualizationOptions(bool nodes, bool links, bool indices, bool forces)
        {
            if (solver != null)
            {
                solver.SetVisualizationOptions(nodes, links, indices, forces);
                UpdateUIFromSolver();
            }
        }
        
        public void SetVisualizationOptions(bool nodes, bool links, bool indices, bool forces, bool influenceRadius)
        {
            if (solver != null)
            {
                solver.SetVisualizationOptions(nodes, links, indices, forces, influenceRadius);
                UpdateUIFromSolver();
            }
        }
        
        // Method to set custom colors
        public void SetVisualizationColors(Color nodeColor, Color pinnedNodeColor, Color linkColor, Color stretchedColor, Color compressedColor)
        {
            if (solver == null) return;
            
            solver.nodeColor = nodeColor;
            solver.pinnedNodeColor = pinnedNodeColor;
            solver.linkColor = linkColor;
            solver.stretchedLinkColor = stretchedColor;
            solver.compressedLinkColor = compressedColor;
        }
        
        void OnGUI()
        {
            if (solver == null) return;
            
            // Simple on-screen controls if no UI elements are assigned
            if (showNodesToggle == null && showLinksToggle == null)
            {
                GUILayout.BeginArea(new Rect(10, 10, 300, 250));
                GUILayout.Label("Soft Body Visualization");
                
                solver.showNodesAndLinks = GUILayout.Toggle(solver.showNodesAndLinks, "Show Nodes & Links");
                
                if (solver.showNodesAndLinks)
                {
                    solver.showNodes = GUILayout.Toggle(solver.showNodes, "Show Nodes");
                    solver.showLinks = GUILayout.Toggle(solver.showLinks, "Show Links");
                    solver.showNodeIndices = GUILayout.Toggle(solver.showNodeIndices, "Show Node Indices");
                    solver.showLinkForces = GUILayout.Toggle(solver.showLinkForces, "Show Link Forces");
                    solver.showInfluenceRadius = GUILayout.Toggle(solver.showInfluenceRadius, "Show Influence Radius");
                    
                    GUILayout.Label($"Node Size: {solver.nodeDisplaySize:F2}");
                    solver.nodeDisplaySize = GUILayout.HorizontalSlider(solver.nodeDisplaySize, 0.05f, 0.5f);
                }
                
                GUILayout.Label(solver.GetVisualizationStats());
                GUILayout.EndArea();
            }
        }
    }
}