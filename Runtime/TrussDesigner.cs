    /* DynamicEngine3D - Node Management
       *---*---*
      / \ / \ / \
     *---*---*---*
     | DynamicEngine3D |  By: Elitmers
     *---*---*---*
      \ / \ / \ /
       *---*---*
    */
    using System.Collections.Generic;
    using System.Linq;
    using UnityEngine;
#if UNITY_EDITOR
    using UnityEditor;
#endif

namespace DynamicEngine
{
    public class NodeLinkEditor : MonoBehaviour
    {
        [Header("Node and Link Data")]
        [SerializeField] public List<Vector3> nodes = new List<Vector3>();
        [SerializeField] public List<Link> links = new List<Link>();
        [SerializeField] public List<int> selectedNodeIndices = new List<int>();
        [SerializeField] public List<int> selectedLinkIndices = new List<int>();
        [SerializeField] public List<int> pinnedNodes = new List<int>();

        [Header("Material Properties")]
        [SerializeField] private MaterialProps materialProps = new MaterialProps("Metal", 0.5f, 1e-3f, 0.3f, 0.1f, 0.15f, 0.1f, 0.05f);
        public MaterialProps MaterialProps => materialProps;
        
        [Header("Material Presets")]
        [SerializeField] [Tooltip("Pre-configured material presets with realistic physical properties")] public List<MaterialProps> materialPresets = new List<MaterialProps>();

        [Header("Node Sets")]
        [SerializeField] public List<NodeSet> nodeSets = new List<NodeSet>();
        
        [Header("Editor States")]
        [SerializeField] public bool isCreatingNode = false;
        [SerializeField] public int creatingLinkNodeIndex = -1;
        [SerializeField] public int currentTab = 0; // 0: Info, 1: Nodes, 2: Links, 3: Debugging, 4: Node Sets

        [Header("Physical Properties")]
        [SerializeField, Range(0.1f, 5.0f)] public float nodeMass = 0.5f;
        [SerializeField, Range(1.0f, 1.5f), Tooltip("Maximum stretch factor for beams (e.g., 1.05 = 105% of rest length)")] public float maxStretchFactor = 1.05f;
        [SerializeField, Range(0.5f, 1.0f), Tooltip("Minimum stretch factor for beams (e.g., 0.95 = 95% of rest length)")] public float minStretchFactor = 0.95f;

        [Header("Visualization Settings")]
        [SerializeField, Tooltip("Visualize forces (red: tension, blue: compression, green: collisions, magenta: ground)")] public bool visualizeForces = false;
        [SerializeField, Tooltip("Size of node spheres in the Scene view")] public float nodeSize = 0.1f;
        [SerializeField, Tooltip("Thickness of link lines in the Scene view")] public float linkThickness = 3f;
        [SerializeField, Tooltip("Color for unselected nodes")] public Color nodeColor = Color.red;
        [SerializeField, Tooltip("Color for selected nodes")] public Color selectedNodeColor = Color.blue;
        [SerializeField, Tooltip("Color for pinned nodes")] public Color pinnedNodeColor = Color.magenta;
        [SerializeField, Tooltip("Color for unselected links")] public Color linkColor = Color.yellow;
        [SerializeField, Tooltip("Color for selected links")] public Color selectedLinkColor = Color.blue;

        [Header("Debugging")]
        [SerializeField, Tooltip("Enable debug logs for operations like pinning and stretch limit application")] public bool enableDebugLogs = false;

        [Header("References")]
        [SerializeField] public SoftBody softBody;
        public SoftBody SoftBody => softBody;

        private void Awake()
        {
            Initialize();
        }

        private void Update()
        {
            UpdateVisualization();
        }

        private void Initialize()
        {
            if (Application.isPlaying)
            {
                enabled = false;
                if (enableDebugLogs) Debug.Log("NodeLinkEditor is inactive during Play Mode.", this);
                return;
            }

            nodes ??= new List<Vector3>();
            links ??= new List<Link>();
            selectedNodeIndices ??= new List<int>();
            selectedLinkIndices ??= new List<int>();
            pinnedNodes ??= new List<int>();
            
            // Initialize material presets if empty
            InitializeMaterialPresets();

            softBody = GetComponent<SoftBody>();
            if (softBody == null)
                softBody = gameObject.GetComponent<SoftBody>();   // same object

            LoadFromTrussAsset();
            ApplyStretchLimits();
            ApplyPinnedNodes();
        }

        private void UpdateVisualization()
        {
            if (Application.isPlaying)
                return;
            if (softBody == null) return;
            var core = GetSolver();
            if (core != null) core.visualizeForces = visualizeForces;
        }

        public Solver GetSolver()
        {
            return typeof(SoftBody).GetField("core", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)?.GetValue(softBody) as Solver;
        }

        #region Node and Link Management
        public void CreateNode(Vector3 position)
        {
            Undo.RecordObject(this, "Create Node");
            nodes.Add(position);
            selectedNodeIndices.Clear();
            selectedNodeIndices.Add(nodes.Count - 1);
            selectedLinkIndices.Clear();
            SaveToTrussAsset();
        }

        public void CreateLink(int nodeA, int nodeB)
        {
            if (!IsValidLink(nodeA, nodeB)) return;
            float restLength = Vector3.Distance(nodes[nodeA], nodes[nodeB]);
            if (restLength < 0.01f)
            {
                Debug.LogWarning($"Cannot create link between nodes {nodeA} and {nodeB}: rest length {restLength} is too small.", this);
                return;
            }

            Undo.RecordObject(this, "Create Link");
            links.Add(new Link { nodeA = nodeA, nodeB = nodeB, springForce = 800f, damping = 30f, restLength = restLength });
            selectedNodeIndices.Clear();
            selectedLinkIndices.Clear();
            selectedLinkIndices.Add(links.Count - 1);
            SaveToTrussAsset();
        }

        private bool IsValidLink(int nodeA, int nodeB)
        {
            if (nodeA == nodeB || nodeA < 0 || nodeB < 0 || nodeA >= nodes.Count || nodeB >= nodes.Count)
            {
                Debug.LogWarning($"Invalid link indices: nodeA={nodeA}, nodeB={nodeB}, nodes.Count={nodes.Count}.", this);
                return false;
            }
            return true;
        }

        public void DeleteSelected()
        {
            Undo.RecordObject(this, "Delete Selection");
            if (currentTab == 1 && selectedNodeIndices.Count > 0)
            {
                DeleteSelectedNodes();
            }
            else if (currentTab == 2 && selectedLinkIndices.Count > 0)
            {
                DeleteSelectedLinks();
            }
            ApplyPinnedNodes();
            SaveToTrussAsset();
        }

        private void DeleteSelectedNodes()
        {
            selectedNodeIndices.Sort((a, b) => b.CompareTo(a));
            foreach (int index in selectedNodeIndices)
            {
                if (index >= 0 && index < nodes.Count)
                {
                    pinnedNodes.Remove(index);
                    nodes.RemoveAt(index);
                    links.RemoveAll(link => link.nodeA == index || link.nodeB == index);
                    UpdateLinkIndicesAfterNodeDeletion(index);
                }
            }
            selectedNodeIndices.Clear();
            SaveToTrussAsset();
        }

        private void UpdateLinkIndicesAfterNodeDeletion(int deletedIndex)
        {
            for (int i = 0; i < links.Count; i++)
            {
                var link = links[i];
                if (link.nodeA > deletedIndex) link.nodeA--;
                if (link.nodeB > deletedIndex) link.nodeB--;
                links[i] = link;
            }
            for (int i = 0; i < pinnedNodes.Count; i++)
            {
                if (pinnedNodes[i] > deletedIndex) pinnedNodes[i]--;
            }
            
            // Update node sets after node deletion
            UpdateNodeSetIndicesAfterNodeDeletion(deletedIndex);
            ValidateNodeSets();
        }

        private void DeleteSelectedLinks()
        {
            selectedLinkIndices.Sort((a, b) => b.CompareTo(a));
            foreach (int index in selectedLinkIndices)
            {
                if (index >= 0 && index < links.Count)
                {
                    links.RemoveAt(index);
                }
            }
            selectedLinkIndices.Clear();
            SaveToTrussAsset();
        }

        public void TransformSelectedNodes(Vector3 translation)
        {
            if (selectedNodeIndices.Count == 0) return;
            Undo.RecordObject(this, "Transform Nodes");
            foreach (int index in selectedNodeIndices.ToList())
            {
                if (index >= 0 && index < nodes.Count && !pinnedNodes.Contains(index))
                {
                    nodes[index] += translation;
                }
                else
                {
                    selectedNodeIndices.Remove(index);
                }
            }
            UpdateLinkRestLengths();
            SaveToTrussAsset();
        }

        public void UpdateLinkRestLengths()
        {
            for (int i = 0; i < links.Count; i++)
            {
                var link = links[i];
                if (link.nodeA < nodes.Count && link.nodeB < nodes.Count)
                {
                    link.restLength = Vector3.Distance(nodes[link.nodeA], nodes[link.nodeB]);
                    links[i] = link;
                }
            }
        }

        public Vector3 GetSelectionCenter()
        {
            if (selectedNodeIndices.Count == 0) return Vector3.zero;
            Vector3 center = Vector3.zero;
            int validCount = 0;
            foreach (int index in selectedNodeIndices.ToList())
            {
                if (index >= 0 && index < nodes.Count)
                {
                    center += nodes[index];
                    validCount++;
                }
                else
                {
                    selectedNodeIndices.Remove(index);
                }
            }
            return validCount > 0 ? center / validCount : Vector3.zero;
        }
        #endregion

        #region Truss Asset Management
        public void LoadFromTrussAsset()
        {
            if (softBody == null || softBody.GetTrussAsset() == null)
            {
                if (enableDebugLogs) Debug.LogWarning("Cannot load from TrussAsset: SoftBody or TrussAsset is missing.", this);
                return;
            }

            TrussAsset truss = softBody.GetTrussAsset();
            
            // Load node positions
            nodes = truss.NodePositions?.ToList() ?? new List<Vector3>();
            
            // Load beam data
            links.Clear();
            foreach (var beam in truss.GetTrussBeams())
            {
                links.Add(new Link
                {
                    nodeA = beam.nodeA,
                    nodeB = beam.nodeB,
                    springForce = beam.compliance,
                    damping = beam.damping,
                    restLength = beam.restLength
                });
            }
            
            // Load node sets if available
            nodeSets = truss.NodeSets?.ToList() ?? new List<NodeSet>();
            ValidateNodeSets(); // Ensure node sets are valid after loading
            
            // 🎯 CRITICAL: Restore ALL material properties from TrussAsset
            if (truss.MaterialProperties != null)
            {
                materialProps = truss.MaterialProperties;
            }
            
            // Restore physics properties
            nodeMass = truss.NodeMass;
            maxStretchFactor = truss.MaxStretchFactor;
            minStretchFactor = truss.MinStretchFactor;
            pinnedNodes = truss.PinnedNodes?.ToList() ?? new List<int>();
            
            // Clear selections
            selectedNodeIndices.Clear();
            selectedLinkIndices.Clear();
            
            if (enableDebugLogs) Debug.Log($"📂 Loaded COMPLETE truss data from TrussAsset: {nodes.Count} nodes, {links.Count} links, material properties, and physics settings.", this);
        }

        public void SaveToTrussAsset()
        {
            if (softBody == null || softBody.GetTrussAsset() == null)
            {
                if (enableDebugLogs) Debug.LogWarning("Cannot save to TrussAsset: SoftBody or TrussAsset is missing.", this);
                return;
            }

            TrussAsset truss = softBody.GetTrussAsset();
            
            // Save node positions
            truss.SetNodePositions(nodes.ToArray());
            
            // Save beam data with updated properties
            var trussBeams = links
                .Where(link => link.nodeA >= 0 && link.nodeA < nodes.Count && link.nodeB >= 0 && link.nodeB < nodes.Count)
                .Select(link => new TrussAsset.TrussBeam(
                    link.nodeA,
                    link.nodeB,
                    link.springForce,
                    link.damping,
                    link.restLength
                )).ToList();
            truss.SetBeams(trussBeams);
            
            // 🎯 CRITICAL: Save ALL material properties to TrussAsset
            truss.SetMaterialProperties(materialProps);
            truss.SetPhysicsProperties(nodeMass, maxStretchFactor, minStretchFactor, new List<int>(pinnedNodes));
            
            // Save node sets
            truss.SetNodeSets(new List<NodeSet>(nodeSets));
            
            // Apply to SoftBody
            softBody.ApplyTrussAsset(truss);
            
            // Push node mass into the physics engine
            var core = GetSolver();
            if (core != null)
            {
                core.materialProps.nodeMass = nodeMass;
            }

#if UNITY_EDITOR
            EditorUtility.SetDirty(truss);
            EditorUtility.SetDirty(softBody);
#endif
            if (enableDebugLogs) Debug.Log($"💾 Saved COMPLETE truss data to TrussAsset: {nodes.Count} nodes, {trussBeams.Count} beams, material properties, and physics settings.", this);
        }
        #endregion

        #region Physics Settings
        public void ApplyStretchLimits()
        {
            if (softBody == null)
            {
                if (enableDebugLogs) Debug.LogWarning("Cannot apply stretch limits: SoftBody is missing.", this);
                return;
            }

            maxStretchFactor = Mathf.Clamp(maxStretchFactor, 1.0f, 1.5f);
            minStretchFactor = Mathf.Clamp(minStretchFactor, 0.5f, 1.0f);
            var core = GetSolver();
            if (core == null) return;

            try
            {
                var maxField = typeof(Solver).GetField("maxStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                var minField = typeof(Solver).GetField("minStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                if (maxField != null && minField != null)
                {
                    maxField.SetValue(core, maxStretchFactor);
                    minField.SetValue(core, minStretchFactor);
                    if (enableDebugLogs)
                        Debug.Log($"Applied stretch limits to SoftBodyCore: maxStretchFactor={maxStretchFactor}, minStretchFactor={minStretchFactor}", this);
                }
                else
                {
                    Debug.LogWarning("Could not access stretch limit fields in SoftBodyCore.", this);
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Failed to apply stretch limits: {e.Message}", this);
            }
        }

        public void ApplyPinnedNodes()
        {
            if (softBody == null)
            {
                if (enableDebugLogs) Debug.LogWarning("Cannot apply pinned nodes: SoftBody is missing.", this);
                return;
            }

            var core = GetSolver();
            if (core == null) return;

            try
            {
                var pinMethod = typeof(Solver).GetMethod("PinNode", System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
                var unpinMethod = typeof(Solver).GetMethod("UnpinNode", System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
                if (pinMethod != null && unpinMethod != null)
                {
                    for (int i = 0; i < nodes.Count; i++)
                    {
                        unpinMethod.Invoke(core, new object[] { i });
                    }
                    foreach (var nodeIndex in pinnedNodes)
                    {
                        if (nodeIndex >= 0 && nodeIndex < nodes.Count)
                        {
                            pinMethod.Invoke(core, new object[] { nodeIndex });
                            if (enableDebugLogs) Debug.Log($"Pinned node {nodeIndex} in SoftBodyCore.", this);
                        }
                        else
                        {
                            Debug.LogWarning($"Invalid pinned node index: {nodeIndex}.", this);
                        }
                    }
                }
                else
                {
                    Debug.LogWarning("Could not access PinNode/UnpinNode methods in SoftBodyCore.", this);
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Failed to apply pinned nodes: {e.Message}", this);
            }
            SaveToTrussAsset();
        }

        public void ApplyRigidSettings()
        {
            Undo.RecordObject(this, "Apply Rigid Settings");
            maxStretchFactor = 1.02f;
            minStretchFactor = 0.98f;
            nodeMass = 2.0f;
            UpdateLinks(10000f, 0.005f);
            ApplyStretchLimits();
            SaveToTrussAsset();
            if (enableDebugLogs)
                Debug.Log("Applied rigid settings: maxStretchFactor=1.02, minStretchFactor=0.98, springForce=10000, damping=0.005, nodeMass=2.0.", this);
        }

        public void ResetToDefaultSettings()
        {
            Undo.RecordObject(this, "Reset to Default Settings");
            maxStretchFactor = 1.05f;
            minStretchFactor = 0.95f;
            nodeMass = 0.5f;
            UpdateLinks(800f, 30f);
            ApplyStretchLimits();
            SaveToTrussAsset();
            if (enableDebugLogs)
                Debug.Log("Reset to default settings: maxStretchFactor=1.05, minStretchFactor=0.95, springForce=800, damping=30, nodeMass=0.5.", this);
        }

        private void UpdateLinks(float springForce, float damping)
        {
            var updatedLinks = new List<Link>();
            foreach (var link in links)
            {
                var updatedLink = link;
                updatedLink.springForce = springForce;
                updatedLink.damping = damping;
                updatedLinks.Add(updatedLink);
            }
            links = updatedLinks;
        }

        public void PinSelectedNodes()
        {
            Undo.RecordObject(this, "Pin Selected Nodes");
            foreach (var index in selectedNodeIndices)
            {
                if (index >= 0 && index < nodes.Count && !pinnedNodes.Contains(index))
                {
                    pinnedNodes.Add(index);
                    if (enableDebugLogs) Debug.Log($"Pinned node {index}.", this);
                }
            }
            ApplyPinnedNodes();
        }

        public void UnpinSelectedNodes()
        {
            Undo.RecordObject(this, "Unpin Selected Nodes");
            foreach (var index in selectedNodeIndices)
            {
                if (pinnedNodes.Contains(index))
                {
                    pinnedNodes.Remove(index);
                    if (enableDebugLogs) Debug.Log($"Unpinned node {index}.", this);
                }
            }
            ApplyPinnedNodes();
        }
        #endregion

        #region Material Presets
        private void InitializeMaterialPresets()
        {
            if (materialPresets == null || materialPresets.Count == 0)
            {
                materialPresets = new List<MaterialProps>();
                
                // Add pre-configured material presets based on the MaterialProps defaults
                materialPresets.Add(new MaterialProps(
                    name: "Steel - Rigid",
                    nodeMass: 1.2f,
                    defaultCompliance: 15000f, // High spring force for rigidity
                    defaultDamping: 0.005f,    // Low damping for minimal energy loss
                    deformationScale: 0.02f,
                    maxDeformation: 0.1f,
                    plasticityThreshold: 0.08f,
                    plasticityRate: 0.02f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Steel - Flexible",
                    nodeMass: 1.0f,
                    defaultCompliance: 8000f,  // Lower spring force for more flexibility
                    defaultDamping: 0.02f,     // Moderate damping
                    deformationScale: 0.05f,
                    maxDeformation: 0.2f,
                    plasticityThreshold: 0.06f,
                    plasticityRate: 0.05f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Aluminum",
                    nodeMass: 0.7f,
                    defaultCompliance: 6000f,
                    defaultDamping: 0.015f,
                    deformationScale: 0.07f,
                    maxDeformation: 0.25f,
                    plasticityThreshold: 0.05f,
                    plasticityRate: 0.08f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Rubber - Soft",
                    nodeMass: 0.3f,
                    defaultCompliance: 800f,   // Low spring force for high flexibility
                    defaultDamping: 0.8f,      // High damping for energy absorption
                    deformationScale: 0.3f,
                    maxDeformation: 0.8f,
                    plasticityThreshold: 0.2f,
                    plasticityRate: 0.01f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Rubber - Firm",
                    nodeMass: 0.5f,
                    defaultCompliance: 2000f,
                    defaultDamping: 0.4f,
                    deformationScale: 0.15f,
                    maxDeformation: 0.5f,
                    plasticityThreshold: 0.15f,
                    plasticityRate: 0.02f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Plastic - Rigid",
                    nodeMass: 0.6f,
                    defaultCompliance: 4000f,
                    defaultDamping: 0.1f,
                    deformationScale: 0.08f,
                    maxDeformation: 0.3f,
                    plasticityThreshold: 0.04f,
                    plasticityRate: 0.15f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Plastic - Flexible",
                    nodeMass: 0.5f,
                    defaultCompliance: 1500f,
                    defaultDamping: 0.25f,
                    deformationScale: 0.12f,
                    maxDeformation: 0.4f,
                    plasticityThreshold: 0.06f,
                    plasticityRate: 0.12f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Wood - Oak",
                    nodeMass: 0.8f,
                    defaultCompliance: 3500f,
                    defaultDamping: 0.05f,
                    deformationScale: 0.06f,
                    maxDeformation: 0.2f,
                    plasticityThreshold: 0.07f,
                    plasticityRate: 0.25f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Wood - Pine",
                    nodeMass: 0.6f,
                    defaultCompliance: 2500f,
                    defaultDamping: 0.08f,
                    deformationScale: 0.09f,
                    maxDeformation: 0.3f,
                    plasticityThreshold: 0.05f,
                    plasticityRate: 0.18f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Glass Fiber",
                    nodeMass: 0.4f,
                    defaultCompliance: 5500f,
                    defaultDamping: 0.012f,
                    deformationScale: 0.04f,
                    maxDeformation: 0.15f,
                    plasticityThreshold: 0.03f,
                    plasticityRate: 0.8f // Breaks easily after threshold
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Carbon Fiber",
                    nodeMass: 0.35f,
                    defaultCompliance: 12000f,
                    defaultDamping: 0.008f,
                    deformationScale: 0.02f,
                    maxDeformation: 0.08f,
                    plasticityThreshold: 0.04f,
                    plasticityRate: 0.9f // Very brittle
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Foam - Soft",
                    nodeMass: 0.1f,
                    defaultCompliance: 200f,
                    defaultDamping: 1.2f,      // Very high damping
                    deformationScale: 0.5f,
                    maxDeformation: 0.9f,
                    plasticityThreshold: 0.3f,
                    plasticityRate: 0.001f
                ));
            }
        }
        
        public void ApplyMaterialPreset(MaterialProps preset)
        {
            if (preset == null) return;
            
            Undo.RecordObject(this, $"Apply Material Preset: {preset.name}");
            
            // Apply material properties to node mass
            nodeMass = preset.nodeMass;
            
            // Apply spring force and damping to all selected links, or all links if none selected
            var targetLinks = selectedLinkIndices.Count > 0 ? selectedLinkIndices : Enumerable.Range(0, links.Count).ToList();
            
            foreach (int idx in targetLinks)
            {
                if (idx >= 0 && idx < links.Count)
                {
                    var link = links[idx];
                    link.springForce = preset.Springforce;
                    link.damping = preset.Damping;
                    links[idx] = link;
                }
            }
            
            // Update the solver with new material properties
            var core = GetSolver();
            if (core != null)
            {
                core.materialProps.nodeMass = preset.nodeMass;
                
                // Update beam properties in the solver
                foreach (int idx in targetLinks)
                {
                    if (idx >= 0 && idx < core.beams.Count)
                    {
                        var beam = core.beams[idx];
                        beam.compliance = 1f / preset.Springforce; // Convert spring force to compliance
                        beam.damping = preset.Damping;
                        core.beams[idx] = beam;
                    }
                }
            }
            
            SaveToTrussAsset();
            
            if (enableDebugLogs)
            {
                Debug.Log($"Applied material preset '{preset.name}' to {(selectedLinkIndices.Count > 0 ? selectedLinkIndices.Count : links.Count)} links", this);
            }
        }
        
        public void RefreshMaterialPresets()
        {
            materialPresets.Clear();
            InitializeMaterialPresets();
            if (enableDebugLogs)
            {
                Debug.Log($"Refreshed material presets. Total presets: {materialPresets.Count}", this);
            }
        }
        #endregion

        #region SoftBody Synchronization
        public void SyncToSolver(Solver core, Transform transform)
        {
            if (core == null)
            {
                Debug.LogWarning("Cannot sync to null SoftBodyCore.", this);
                return;
            }

            core.SetMaterialProperties(MaterialProps);   // push NodeLinkEditor values into SoftBodyCore

            List<Beam> beams = links
                .Where(link => link.nodeA >= 0 && link.nodeA < nodes.Count && link.nodeB >= 0 && link.nodeB < nodes.Count && link.restLength > 0.01f)
                .Select(link => new Beam(link.nodeA, link.nodeB, link.springForce, link.damping, link.restLength))
                .ToList();

            core.GenerateNodesAndBeams(nodes.ToArray(), beamsArray: beams.ToArray(), parent: transform);

            try
            {
                var maxField = typeof(Solver).GetField("maxStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                var minField = typeof(Solver).GetField("minStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                if (maxField != null && minField != null)
                {
                    maxField.SetValue(core, Mathf.Max(1.0f, maxStretchFactor));
                    minField.SetValue(core, Mathf.Min(1.0f, minStretchFactor));
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Failed to sync stretch limits: {e.Message}", this);
            }

            ApplyPinnedNodes();
            if (enableDebugLogs)
                Debug.Log($"Synced {nodes.Count} nodes and {beams.Count} beams to SoftBodyCore with stretch limits: max={maxStretchFactor}, min={minStretchFactor}.", this);
        }
        #endregion

        #region Node Set Management
        public void CreateNodeSet(string name, List<int> nodeIndices)
        {
            if (string.IsNullOrEmpty(name) || nodeIndices == null || nodeIndices.Count == 0)
            {
                Debug.LogWarning("Cannot create node set: invalid name or empty node indices.");
                return;
            }

            // Validate node indices
            var validIndices = nodeIndices.Where(i => i >= 0 && i < nodes.Count).ToList();
            if (validIndices.Count == 0)
            {
                Debug.LogWarning($"Cannot create node set '{name}': no valid node indices.");
                return;
            }

            // Check if name already exists
            if (nodeSets.Any(ns => ns.name == name))
            {
                Debug.LogWarning($"Node set '{name}' already exists. Use a different name.");
                return;
            }

            Undo.RecordObject(this, $"Create Node Set: {name}");
            nodeSets.Add(new NodeSet { name = name, nodeIndices = validIndices });
            SaveToTrussAsset();
            
            if (enableDebugLogs)
                Debug.Log($"Created node set '{name}' with {validIndices.Count} nodes.", this);
        }

        public void DeleteNodeSet(string name)
        {
            var nodeSet = nodeSets.FirstOrDefault(ns => ns.name == name);
            if (nodeSet == null)
            {
                Debug.LogWarning($"Node set '{name}' not found.");
                return;
            }

            Undo.RecordObject(this, $"Delete Node Set: {name}");
            nodeSets.Remove(nodeSet);
            SaveToTrussAsset();
            
            if (enableDebugLogs)
                Debug.Log($"Deleted node set '{name}'.", this);
        }

        public void RenameNodeSet(string oldName, string newName)
        {
            if (string.IsNullOrEmpty(newName))
            {
                Debug.LogWarning("Cannot rename node set: new name is empty.");
                return;
            }

            var nodeSet = nodeSets.FirstOrDefault(ns => ns.name == oldName);
            if (nodeSet == null)
            {
                Debug.LogWarning($"Node set '{oldName}' not found.");
                return;
            }

            if (nodeSets.Any(ns => ns.name == newName))
            {
                Debug.LogWarning($"Node set '{newName}' already exists.");
                return;
            }

            Undo.RecordObject(this, $"Rename Node Set: {oldName} to {newName}");
            nodeSet.name = newName;
            SaveToTrussAsset();
            
            if (enableDebugLogs)
                Debug.Log($"Renamed node set '{oldName}' to '{newName}'.", this);
        }

        public void UpdateNodeSet(string name, List<int> newNodeIndices)
        {
            var nodeSet = nodeSets.FirstOrDefault(ns => ns.name == name);
            if (nodeSet == null)
            {
                Debug.LogWarning($"Node set '{name}' not found.");
                return;
            }

            if (newNodeIndices == null || newNodeIndices.Count == 0)
            {
                Debug.LogWarning($"Cannot update node set '{name}': empty node indices.");
                return;
            }

            // Validate node indices
            var validIndices = newNodeIndices.Where(i => i >= 0 && i < nodes.Count).ToList();
            if (validIndices.Count == 0)
            {
                Debug.LogWarning($"Cannot update node set '{name}': no valid node indices.");
                return;
            }

            Undo.RecordObject(this, $"Update Node Set: {name}");
            nodeSet.nodeIndices = validIndices;
            SaveToTrussAsset();
            
            if (enableDebugLogs)
                Debug.Log($"Updated node set '{name}' with {validIndices.Count} nodes.", this);
        }

        public NodeSet GetNodeSet(string name)
        {
            return nodeSets.FirstOrDefault(ns => ns.name == name);
        }

        public int[] GetNodeSetIndices(string name)
        {
            var nodeSet = GetNodeSet(name);
            return nodeSet?.nodeIndices.ToArray();
        }

        public void CreateNodeSetFromSelection(string name)
        {
            if (selectedNodeIndices.Count == 0)
            {
                Debug.LogWarning("Cannot create node set: no nodes selected.");
                return;
            }

            CreateNodeSet(name, new List<int>(selectedNodeIndices));
        }

        public void SelectNodeSet(string name)
        {
            var nodeSet = GetNodeSet(name);
            if (nodeSet == null)
            {
                Debug.LogWarning($"Node set '{name}' not found.");
                return;
            }

            selectedNodeIndices.Clear();
            selectedNodeIndices.AddRange(nodeSet.nodeIndices.Where(i => i >= 0 && i < nodes.Count));
            selectedLinkIndices.Clear();
            
            if (enableDebugLogs)
                Debug.Log($"Selected node set '{name}' with {selectedNodeIndices.Count} nodes.", this);
        }

        public void ValidateNodeSets()
        {
            // Remove node sets with invalid indices or update them
            for (int i = nodeSets.Count - 1; i >= 0; i--)
            {
                var nodeSet = nodeSets[i];
                var validIndices = nodeSet.nodeIndices.Where(idx => idx >= 0 && idx < nodes.Count).ToList();
                
                if (validIndices.Count == 0)
                {
                    // Remove empty node set
                    if (enableDebugLogs)
                        Debug.Log($"Removing empty node set '{nodeSet.name}' due to deleted nodes.", this);
                    nodeSets.RemoveAt(i);
                }
                else if (validIndices.Count != nodeSet.nodeIndices.Count)
                {
                    // Update node set with valid indices only
                    if (enableDebugLogs)
                        Debug.Log($"Updated node set '{nodeSet.name}' due to deleted nodes: {nodeSet.nodeIndices.Count} -> {validIndices.Count} nodes.", this);
                    nodeSet.nodeIndices = validIndices;
                }
            }
        }

        public void UpdateNodeSetIndicesAfterNodeDeletion(int deletedIndex)
        {
            foreach (var nodeSet in nodeSets)
            {
                // Remove the deleted node
                nodeSet.nodeIndices.Remove(deletedIndex);
                
                // Update indices for nodes that were shifted down
                for (int i = 0; i < nodeSet.nodeIndices.Count; i++)
                {
                    if (nodeSet.nodeIndices[i] > deletedIndex)
                        nodeSet.nodeIndices[i]--;
                }
            }
        }
        #endregion
    }

    [System.Serializable]
    public class NodeSet
    {
        public string name;
        public List<int> nodeIndices = new List<int>();
        public Color color = Color.cyan;
        public bool isVisible = true;

        public NodeSet()
        {
            nodeIndices = new List<int>();
        }

        public NodeSet(string name, List<int> indices)
        {
            this.name = name;
            this.nodeIndices = new List<int>(indices);
        }

        public bool IsValid()
        {
            return !string.IsNullOrEmpty(name) && nodeIndices != null && nodeIndices.Count > 0;
        }

        public bool ContainsNode(int nodeIndex)
        {
            return nodeIndices.Contains(nodeIndex);
        }
    }
}
