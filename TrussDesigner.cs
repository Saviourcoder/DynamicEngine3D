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
    public class Designer : MonoBehaviour
    {
        private static bool s_isApplicationQuitting = false;
        
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void InitializeOnLoad()
        {
            s_isApplicationQuitting = false;
            Application.quitting += () => s_isApplicationQuitting = true;
        }
        
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

        private void OnDestroy()
        {
            // Don't save during application quit or when GameObject is being destroyed
            if (s_isApplicationQuitting || gameObject == null)
                return;
                
            if (softBody != null && softBody.GetTrussAsset() != null)
            {
                SaveToTrussAsset();
            }
        }

        private void OnDisable()
        {
            if (!Application.isPlaying && softBody != null && softBody.GetTrussAsset() != null)
            {
                SaveToTrussAsset();
            }
        }

        private void Initialize()
        {
            if (Application.isPlaying)
            {
                enabled = false;
                return;
            }

            nodes ??= new List<Vector3>();
            links ??= new List<Link>();
            selectedNodeIndices ??= new List<int>();
            selectedLinkIndices ??= new List<int>();
            pinnedNodes ??= new List<int>();
            
            InitializeMaterialPresets();

            softBody = GetComponent<SoftBody>();
            if (softBody == null)
                softBody = gameObject.GetComponent<SoftBody>();

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
                return;

            TrussAsset truss = softBody.GetTrussAsset();
            
            // Load node positions
            nodes = truss.NodePositions?.ToList() ?? new List<Vector3>();
            
            // Load beam data with validation
            links.Clear();
            int skippedBeams = 0;
            foreach (var beam in truss.GetTrussBeams())
            {
                // Validate beam nodes
                if (beam.nodeA < 0 || beam.nodeA >= nodes.Count || 
                    beam.nodeB < 0 || beam.nodeB >= nodes.Count ||
                    beam.nodeA == beam.nodeB)
                {
                    skippedBeams++;
                    continue;
                }
                
                // Check for duplicate node positions
                Vector3 posA = nodes[beam.nodeA];
                Vector3 posB = nodes[beam.nodeB];
                float distance = Vector3.Distance(posA, posB);
                if (distance < 0.01f)
                {
                    skippedBeams++;
                    continue;
                }
                
                links.Add(new Link
                {
                    nodeA = beam.nodeA,
                    nodeB = beam.nodeB,
                    springForce = beam.compliance,
                    damping = beam.damping,
                    restLength = Mathf.Max(beam.restLength, distance)
                });
            }
            
            // Load node sets if available
            nodeSets = truss.NodeSets?.ToList() ?? new List<NodeSet>();
            ValidateNodeSets();
            
            // Restore material properties from TrussAsset
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
        }

        public void SaveToTrussAsset()
        {
            if (softBody == null || softBody.GetTrussAsset() == null)
                return;

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
            
            truss.SetMaterialProperties(materialProps);
            truss.SetPhysicsProperties(nodeMass, maxStretchFactor, minStretchFactor, new List<int>(pinnedNodes));
            truss.SetNodeSets(new List<NodeSet>(nodeSets));

#if UNITY_EDITOR
            UnityEditor.EditorUtility.SetDirty(truss);
#endif

            // Apply to SoftBody
            softBody.ApplyTrussAsset(truss);
            
            // Push node mass into the physics engine
            if (!Application.isPlaying)
            {
                var core = GetSolver();
                if (core != null)
                {
                    core.materialProps.nodeMass = nodeMass;
                }
            }

#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                UnityEditor.EditorUtility.SetDirty(softBody);
            }
#endif
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
                
                // Add only the essential material presets: Rubber, Metal, Glass
                materialPresets.Add(new MaterialProps(
                    name: "Rubber",
                    nodeMass: 0.4f,
                    defaultCompliance: 1200f,   // Low spring force for high flexibility
                    defaultDamping: 0.6f,       // High damping for energy absorption
                    deformationScale: 0.25f,
                    maxDeformation: 0.7f,
                    plasticityThreshold: 0.18f,
                    plasticityRate: 0.015f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Metal",
                    nodeMass: 1.0f,
                    defaultCompliance: 10000f,  // High spring force for rigidity
                    defaultDamping: 0.01f,      // Low damping for minimal energy loss
                    deformationScale: 0.03f,
                    maxDeformation: 0.12f,
                    plasticityThreshold: 0.075f,
                    plasticityRate: 0.03f
                ));
                
                materialPresets.Add(new MaterialProps(
                    name: "Glass",
                    nodeMass: 0.5f,
                    defaultCompliance: 6000f,
                    defaultDamping: 0.008f,
                    deformationScale: 0.02f,
                    maxDeformation: 0.08f,
                    plasticityThreshold: 0.025f,
                    plasticityRate: 0.9f // Very brittle - breaks easily after threshold
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

        #region Validation and Cleanup
        public void CleanupDuplicateNodes()
        {
            Undo.RecordObject(this, "Cleanup Duplicate Nodes");
            
            Dictionary<Vector3, int> positionToIndex = new Dictionary<Vector3, int>();
            List<Vector3> uniqueNodes = new List<Vector3>();
            Dictionary<int, int> oldToNewIndex = new Dictionary<int, int>();
            
            // Find unique nodes
            for (int i = 0; i < nodes.Count; i++)
            {
                Vector3 pos = nodes[i];
                // Round position to avoid floating point precision issues
                Vector3 roundedPos = new Vector3(
                    Mathf.Round(pos.x * 10000f) / 10000f,
                    Mathf.Round(pos.y * 10000f) / 10000f,
                    Mathf.Round(pos.z * 10000f) / 10000f
                );
                
                if (!positionToIndex.ContainsKey(roundedPos))
                {
                    positionToIndex[roundedPos] = uniqueNodes.Count;
                    uniqueNodes.Add(pos);
                    oldToNewIndex[i] = uniqueNodes.Count - 1;
                }
                else
                {
                    oldToNewIndex[i] = positionToIndex[roundedPos];
                    if (enableDebugLogs) Debug.Log($"Found duplicate node {i} at position {pos}, merging with node {positionToIndex[roundedPos]}", this);
                }
            }
            
            // Update nodes
            int removedNodes = nodes.Count - uniqueNodes.Count;
            nodes = uniqueNodes;
            
            // Update links to use new indices and remove invalid ones
            List<Link> validLinks = new List<Link>();
            int removedLinks = 0;
            
            for (int i = 0; i < links.Count; i++)
            {
                var link = links[i];
                int newNodeA = oldToNewIndex[link.nodeA];
                int newNodeB = oldToNewIndex[link.nodeB];
                
                // Skip links between the same node (would have zero length)
                if (newNodeA == newNodeB)
                {
                    if (enableDebugLogs) Debug.Log($"Removing link {i} between duplicate nodes {link.nodeA} and {link.nodeB}", this);
                    removedLinks++;
                    continue;
                }
                
                // Check if this link already exists
                bool linkExists = validLinks.Any(l => 
                    (l.nodeA == newNodeA && l.nodeB == newNodeB) || 
                    (l.nodeA == newNodeB && l.nodeB == newNodeA));
                
                if (linkExists)
                {
                    if (enableDebugLogs) Debug.Log($"Removing duplicate link {i} between nodes {newNodeA} and {newNodeB}", this);
                    removedLinks++;
                    continue;
                }
                
                // Update the link with new indices
                link.nodeA = newNodeA;
                link.nodeB = newNodeB;
                link.restLength = Vector3.Distance(nodes[newNodeA], nodes[newNodeB]);
                validLinks.Add(link);
            }
            
            links = validLinks;
            
            // Update pinned nodes
            List<int> validPinnedNodes = new List<int>();
            foreach (int pinnedNode in pinnedNodes)
            {
                if (oldToNewIndex.ContainsKey(pinnedNode))
                {
                    int newIndex = oldToNewIndex[pinnedNode];
                    if (!validPinnedNodes.Contains(newIndex))
                        validPinnedNodes.Add(newIndex);
                }
            }
            pinnedNodes = validPinnedNodes;
            
            // Update node sets
            foreach (var nodeSet in nodeSets)
            {
                List<int> validIndices = new List<int>();
                foreach (int index in nodeSet.nodeIndices)
                {
                    if (oldToNewIndex.ContainsKey(index))
                    {
                        int newIndex = oldToNewIndex[index];
                        if (!validIndices.Contains(newIndex))
                            validIndices.Add(newIndex);
                    }
                }
                nodeSet.nodeIndices = validIndices;
            }
            
            // Clear selections
            selectedNodeIndices.Clear();
            selectedLinkIndices.Clear();
            
            SaveToTrussAsset();
            
            Debug.Log($"Cleanup complete: Removed {removedNodes} duplicate nodes and {removedLinks} invalid links. Final: {nodes.Count} nodes, {links.Count} links", this);
        }
        
        public void ValidateAndFixBeams()
        {
            Undo.RecordObject(this, "Validate and Fix Beams");
            
            int fixedBeams = 0;
            for (int i = links.Count - 1; i >= 0; i--)
            {
                var link = links[i];
                
                // Check for invalid node indices
                if (link.nodeA < 0 || link.nodeA >= nodes.Count || 
                    link.nodeB < 0 || link.nodeB >= nodes.Count ||
                    link.nodeA == link.nodeB)
                {
                    links.RemoveAt(i);
                    fixedBeams++;
                    if (enableDebugLogs) Debug.Log($"Removed beam {i} with invalid nodes: {link.nodeA}, {link.nodeB}", this);
                    continue;
                }
                
                // Check for zero-length beams
                float distance = Vector3.Distance(nodes[link.nodeA], nodes[link.nodeB]);
                if (distance < 0.01f)
                {
                    links.RemoveAt(i);
                    fixedBeams++;
                    if (enableDebugLogs) Debug.Log($"Removed beam {i} with zero length between nodes {link.nodeA} and {link.nodeB}", this);
                    continue;
                }
                
                // Fix rest length if needed
                if (link.restLength < 0.01f || Mathf.Abs(link.restLength - distance) > 0.001f)
                {
                    var updatedLink = link;
                    updatedLink.restLength = distance;
                    links[i] = updatedLink;
                    fixedBeams++;
                    if (enableDebugLogs) Debug.Log($"Fixed rest length for beam {i}: {link.restLength:F6} -> {distance:F6}", this);
                }
            }
            
            if (fixedBeams > 0)
            {
                SaveToTrussAsset();
                Debug.Log($"Fixed {fixedBeams} problematic beams. Total beams: {links.Count}", this);
            }
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
