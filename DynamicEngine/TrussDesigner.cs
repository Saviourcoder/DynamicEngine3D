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
        [SerializeField] private MaterialProperties materialProps = new MaterialProperties(0.5f, 1e-3f, 0.3f, 0.1f, 0.15f, 0.1f, 0.05f);
        public MaterialProperties MaterialProps => materialProps;

        [Header("Editor States")]
        [SerializeField] public bool isCreatingNode = false;
        [SerializeField] public int creatingLinkNodeIndex = -1;
        [SerializeField] public int currentTab = 0; // 0: Info, 1: Nodes, 2: Links, 3: Debugging

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

        [SerializeField]
        public List<MaterialPreset> materialPresets = new List<MaterialPreset>
        {
            new MaterialPreset { name = "Rubber", springForce = 800f, damping = 30f, nodeMass = 0.5f, maxStretchFactor = 1.1f, minStretchFactor = 0.9f },
            new MaterialPreset { name = "Metal", springForce = 10000f, damping = 0.005f, nodeMass = 2.0f, maxStretchFactor = 1.02f, minStretchFactor = 0.98f },
            new MaterialPreset { name = "Plastic", springForce = 5000f, damping = 0.01f, nodeMass = 1.0f, maxStretchFactor = 1.05f, minStretchFactor = 0.95f }
        };

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
            nodes = truss.NodePositions?.ToList() ?? new List<Vector3>();
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
            selectedNodeIndices.Clear();
            selectedLinkIndices.Clear();
            pinnedNodes.Clear();
            if (enableDebugLogs) Debug.Log($"Loaded {nodes.Count} nodes and {links.Count} links from TrussAsset.", this);
        }

        public void SaveToTrussAsset()
        {
            if (softBody == null || softBody.GetTrussAsset() == null)
            {
                if (enableDebugLogs) Debug.LogWarning("Cannot save to TrussAsset: SoftBody or TrussAsset is missing.", this);
                return;
            }

            TrussAsset truss = softBody.GetTrussAsset();
            truss.SetNodePositions(nodes.ToArray());
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
            softBody.ApplyTrussAsset(truss);
            // push node mass into the physics engine
            var core = GetSolver();
            if (core != null)
            {
                core.materialProps.nodeMass = nodeMass;
            }

#if UNITY_EDITOR
            EditorUtility.SetDirty(truss);
            EditorUtility.SetDirty(softBody);
#endif
            if (enableDebugLogs) Debug.Log($"Saved {nodes.Count} nodes and {trussBeams.Count} beams to TrussAsset.", this);
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
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(NodeLinkEditor))]
    public class NodeLinkEditorCustom : Editor
    {
        private NodeLinkEditor editor;
        private Vector3 handlePosition;
        private bool isDragging;
        private bool stretchLimitsFoldout = true;
        private bool visualizationFoldout = true;
        private bool materialPresetsFoldout = true;
        private Dictionary<int, int> nodeConnections = new Dictionary<int, int>();

        private void OnEnable()
        {
            editor = (NodeLinkEditor)target;
            if (editor.softBody == null)
            {
                editor.softBody = editor.GetComponent<SoftBody>();
            }

            if (EditorApplication.isPlayingOrWillChangePlaymode)
            return;

            EditorApplication.delayCall += DelayedInitializeSoftBody;
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            DrawInspectorGUI();
            serializedObject.ApplyModifiedProperties();
            SceneView.RepaintAll();
        }

        private void DrawInspectorGUI()
        {
            DrawSoftBodyReference();
            EditorGUILayout.Space();
            DrawTabs();
        }
        

        private void DelayedInitializeSoftBody()
        {
            // if the editor window might have been closed already
            if (editor == null)
            return;

            // get the softBody component if we dont have it yet
            var softBody = editor.GetComponent<SoftBody>();
            if (softBody == null)
            return;

            // make sure we have a trussasset
            if (softBody.trussAsset == null)
            {
               Debug.LogWarning("SoftBody has no TrussAsset assigned.", softBody);
               return;
            }

            softBody.InitializeInEditMode();
            EditorUtility.SetDirty(softBody);
        }
        

        private void DrawSoftBodyReference()
{
    EditorGUILayout.BeginHorizontal();
    GUI.enabled = false;
    EditorGUILayout.ObjectField("SoftBody", editor.SoftBody, typeof(SoftBody), true);
    GUI.enabled = true;
    EditorGUILayout.EndHorizontal();

    if (EditorApplication.isPlayingOrWillChangePlaymode) return;

    editor.softBody = editor.GetComponent<SoftBody>();
    if (editor.softBody == null)
    {
        Debug.LogWarning("No SoftBody component found on this GameObject.", editor);
        return;
    }


}
        private void DrawTabs()
        {
            string[] tabs = { "Info", "Nodes", "Links", "Debug", "Visual", "Stretch"};
            editor.currentTab = GUILayout.Toolbar(editor.currentTab, tabs);
            EditorGUILayout.Space();

            switch (editor.currentTab)
            {
                case 0: DrawInfoTab(); break;
                case 1: DrawNodesTab(); break;
                case 2: DrawLinksTab(); break;
                case 3: DrawDebuggingTab(); break;
                case 4: DrawVisualizationTab(); break;
                case 5: DrawStretchLimitsTab(); break;
            }
        }

        private void DrawInfoTab()
        {
            EditorGUILayout.LabelField("Asset Information", EditorStyles.boldLabel);
            GUI.enabled = false;
            EditorGUILayout.ObjectField("NodeLinkEditor", editor, typeof(NodeLinkEditor), true);
            EditorGUILayout.IntField("Nodes", editor.nodes.Count);
            EditorGUILayout.IntField("Links", editor.links.Count);
            EditorGUILayout.IntField("Pinned Nodes", editor.pinnedNodes.Count);
            GUI.enabled = true;
            if (GUILayout.Button("Reload from TrussAsset"))
            {
                editor.LoadFromTrussAsset();
                editor.ApplyStretchLimits();
                editor.ApplyPinnedNodes();
                EditorUtility.SetDirty(editor);
            }
            editor.isCreatingNode = false;
            editor.creatingLinkNodeIndex = -1;
        }

        private void DrawNodesTab()
        {
            if (GUILayout.Button(editor.isCreatingNode ? "Cancel Create Node" : "Create Node"))
            {
                editor.isCreatingNode = !editor.isCreatingNode;
                editor.creatingLinkNodeIndex = -1;   // cancel link creation if any
            }
           EditorGUI.BeginChangeCheck();
           EditorGUILayout.LabelField("Node Mass", EditorStyles.boldLabel);
           editor.nodeMass = EditorGUILayout.FloatField(new GUIContent("Mass", "Mass of each node for physics simulation"), Mathf.Max(0.1f, editor.nodeMass));

            if (EditorGUI.EndChangeCheck())
            {
                var core = editor.GetSolver();
                if (core != null) core.materialProps.nodeMass = editor.nodeMass;
            }
            string nodeInfo = editor.selectedNodeIndices.Count switch
            {
                0 => "No Nodes Selected",
                1 => $"Node {editor.selectedNodeIndices[0]} Selected",
                _ => $"{editor.selectedNodeIndices.Count} Nodes Selected"
            };
            EditorGUILayout.LabelField(nodeInfo, new GUIStyle(EditorStyles.label) { alignment = TextAnchor.MiddleRight });

            if (editor.selectedNodeIndices.Count > 0)
            {
                EditorGUILayout.LabelField("Selected Node Positions", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                foreach (var idx in editor.selectedNodeIndices)
                {
                    if (idx >= 0 && idx < editor.nodes.Count)
                    {
                        EditorGUILayout.LabelField($"Node {idx}: {editor.nodes[idx]}");
                    }
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Delete Selected")) editor.DeleteSelected();
            if (GUILayout.Button("Select All"))
            {
                editor.selectedNodeIndices.Clear();
                for (int i = 0; i < editor.nodes.Count; i++) editor.selectedNodeIndices.Add(i);
            }
            if (GUILayout.Button("Clear Selection")) editor.selectedNodeIndices.Clear();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Pin Selected")) editor.PinSelectedNodes();
            if (GUILayout.Button("Unpin Selected")) editor.UnpinSelectedNodes();
            EditorGUILayout.EndHorizontal();

            editor.creatingLinkNodeIndex = -1;
        }

        private void DrawLinksTab()
        {
            bool isCreatingLink = editor.creatingLinkNodeIndex >= 0;
            editor.isCreatingNode = false;
            string linkInfo = editor.selectedLinkIndices.Count switch
            {
                0 => "No Links Selected",
                1 => $"Link {editor.selectedLinkIndices[0]} Selected",
                _ => $"{editor.selectedLinkIndices.Count} Links Selected"
            };
            EditorGUILayout.LabelField(linkInfo, new GUIStyle(EditorStyles.label) { alignment = TextAnchor.MiddleRight });

            if (editor.selectedLinkIndices.Count > 0)
            {
                EditorGUILayout.LabelField("Selected Link Details", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                foreach (var idx in editor.selectedLinkIndices)
                {
                    if (idx >= 0 && idx < editor.links.Count)
                    {
                        var link = editor.links[idx];
                        EditorGUILayout.LabelField($"Link {idx}: Nodes {link.nodeA}-{link.nodeB}, Spring: {link.springForce}, Damping: {link.damping}, Rest Length: {link.restLength}");
                    }
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.BeginHorizontal();
            bool createLinkPressed = GUILayout.Button("Create Link", isCreatingLink ? new GUIStyle(GUI.skin.button) { normal = { background = Texture2D.grayTexture } } : GUI.skin.button);
            if (createLinkPressed)
            {
                editor.creatingLinkNodeIndex = isCreatingLink ? -1 : 0;
            }
            if (GUILayout.Button("Delete Selected")) editor.DeleteSelected();
            if (GUILayout.Button("Select All"))
            {
                editor.selectedLinkIndices.Clear();
                for (int i = 0; i < editor.links.Count; i++) editor.selectedLinkIndices.Add(i);
            }
            if (GUILayout.Button("Clear Selection")) editor.selectedLinkIndices.Clear();
            EditorGUILayout.EndHorizontal();

            materialPresetsFoldout = EditorGUILayout.Foldout(materialPresetsFoldout, "Material Presets", true, EditorStyles.boldLabel);
            if (materialPresetsFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.BeginHorizontal();
                foreach (var preset in editor.materialPresets)
                {
                    if (GUILayout.Button(preset.name))
                    {
                        ApplyPresetToSelectedLinks(preset);
                    }
                }
                EditorGUILayout.EndHorizontal();
                EditorGUI.indentLevel--;
            }

            if (editor.selectedLinkIndices.Count > 0)
            {
                EditorGUILayout.LabelField("Selected Link Properties", EditorStyles.boldLabel);
                var firstLink = editor.links[editor.selectedLinkIndices[0]];
                float springForce = firstLink.springForce;
                float damping = firstLink.damping;
                bool uniform = true;

                foreach (int idx in editor.selectedLinkIndices)
                {
                    var link = editor.links[idx];
                    if (link.springForce != springForce || link.damping != damping)
                    {
                        uniform = false;
                        break;
                    }
                }

                EditorGUI.BeginChangeCheck();
                springForce = EditorGUILayout.FloatField(new GUIContent("Spring Force", "Force applied to maintain rest length"), Mathf.Clamp(springForce, 100f, 20000f));
                damping = EditorGUILayout.FloatField(new GUIContent("Damping", "Damping factor to reduce oscillations"), Mathf.Clamp(damping, 0.001f, 50f));
                if (EditorGUI.EndChangeCheck() && uniform)
                {
                    Undo.RecordObject(editor, "Modify Link Properties");
                    for (int i = 0; i < editor.selectedLinkIndices.Count; i++)
                    {
                        int idx = editor.selectedLinkIndices[i];
                        var link = editor.links[idx];
                        link.springForce = Mathf.Max(100f, springForce);
                        link.damping = Mathf.Max(0.001f, damping);
                        editor.links[idx] = link;
                    }
                    editor.SaveToTrussAsset();
                }
                else if (!uniform)
                {
                    EditorGUILayout.HelpBox("Multiple links with different properties selected. Values shown are for the first selected link.", MessageType.Info);
                }

                var core = editor.GetSolver();
                if (core != null)
                {
                    foreach (int idx in editor.selectedLinkIndices)
                    {
                        var beam = core.beams[idx];
                        beam.compliance = 1f / editor.links[idx].springForce; // invert because Solver expects “compliance”
                        beam.damping = editor.links[idx].damping;
                        core.beams[idx] = beam;
                    }
                }
            }
        }

        private void DrawDebuggingTab()
        {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("enableDebugLogs"), new GUIContent("Enable Debug Logs", "Log detailed operations like pinning and stretch limit application"));

            if (GUILayout.Button("Validate Node Connectivity"))
            {
                nodeConnections.Clear();
                if (editor.nodes.Count == 0 || editor.links.Count == 0)
                {
                    EditorGUILayout.LabelField("No nodes or links to validate.");
                }
                else
                {
                    for (int i = 0; i < editor.nodes.Count; i++) nodeConnections[i] = 0;
                    foreach (var link in editor.links)
                    {
                        if (link.nodeA >= 0 && link.nodeA < editor.nodes.Count) nodeConnections[link.nodeA]++;
                        if (link.nodeB >= 0 && link.nodeB < editor.nodes.Count) nodeConnections[link.nodeB]++;
                    }
                }
            }

            if (nodeConnections.Count > 0)
            {
                EditorGUILayout.LabelField("Node Connectivity Details", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                foreach (var pair in nodeConnections.OrderBy(kvp => kvp.Key))
                {
                    EditorGUILayout.LabelField($"Node {pair.Key}: {pair.Value} connections");
                }
                EditorGUI.indentLevel--;
            }

            if (GUILayout.Button("Reapply Truss Limits and Pinned Nodes"))
            {
                editor.ApplyStretchLimits();
                editor.ApplyPinnedNodes();
            }
        }

                private void DrawVisualizationTab()
        {
            visualizationFoldout = EditorGUILayout.Foldout(visualizationFoldout, "Visualization Settings", true, EditorStyles.boldLabel);
            if (visualizationFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(serializedObject.FindProperty("visualizeForces"), new GUIContent("Visualize Forces", "Show force vectors in simulation (red: tension, blue: compression)"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("nodeSize"), new GUIContent("Node Size", "Size of node spheres in Scene view"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("linkThickness"), new GUIContent("Link Thickness", "Thickness of link lines in Scene view"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("nodeColor"), new GUIContent("Node Color", "Color for unselected nodes"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("selectedNodeColor"), new GUIContent("Selected Node Color"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("pinnedNodeColor"), new GUIContent("Pinned Node Color"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("linkColor"), new GUIContent("Link Color", "Color for unselected links"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("selectedLinkColor"), new GUIContent("Selected Link Color"));
                EditorGUI.indentLevel--;
            }
        }

          private void DrawStretchLimitsTab()
        {
            stretchLimitsFoldout = EditorGUILayout.Foldout(stretchLimitsFoldout, "Stretch Limits", true, EditorStyles.boldLabel);
            if (stretchLimitsFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUI.BeginChangeCheck();
                float newMaxFactor = EditorGUILayout.FloatField(new GUIContent("Max Stretch Factor", "Maximum allowable stretch (e.g., 1.05 = 105% of rest length)"), editor.maxStretchFactor);
                float newMinFactor = EditorGUILayout.FloatField(new GUIContent("Min Stretch Factor", "Minimum allowable stretch (e.g., 0.95 = 95% of rest length)"), editor.minStretchFactor);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(editor, "Change Stretch Limits");
                    editor.maxStretchFactor = Mathf.Clamp(newMaxFactor, 1.0f, 1.5f);
                    editor.minStretchFactor = Mathf.Clamp(newMinFactor, 0.5f, 1.0f);
                    editor.ApplyStretchLimits();
                    serializedObject.FindProperty("maxStretchFactor").floatValue = editor.maxStretchFactor;
                    serializedObject.FindProperty("minStretchFactor").floatValue = editor.minStretchFactor;
                }

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Apply Rigid Settings")) editor.ApplyRigidSettings();
                if (GUILayout.Button("Reset to Defaults")) editor.ResetToDefaultSettings();
                EditorGUILayout.EndHorizontal();
                EditorGUI.indentLevel--;
            }
        }

        private void ApplyPresetToSelectedLinks(MaterialPreset preset)
        {
            Undo.RecordObject(editor, "Apply Material Preset");
            for (int i = 0; i < editor.selectedLinkIndices.Count; i++)
            {
                int idx = editor.selectedLinkIndices[i];
                var link = editor.links[idx];
                link.springForce = preset.springForce;
                link.damping = preset.damping;
                editor.links[idx] = link;
            }
            editor.nodeMass = preset.nodeMass;
            editor.maxStretchFactor = preset.maxStretchFactor;
            editor.minStretchFactor = preset.minStretchFactor;
            editor.ApplyStretchLimits();
            editor.SaveToTrussAsset();
        }

        public void OnSceneGUI()
        {
            if (Application.isPlaying) return;       // stop the whole editor
            
            Event e = Event.current;
            int controlID = GUIUtility.GetControlID(FocusType.Passive);
            HandleUtility.AddDefaultControl(controlID);

            if (editor.currentTab == 1 || editor.currentTab == 2)
            {
                DrawNodesAndLinks();
            }

            if (editor.isCreatingNode && editor.currentTab == 1)
            {
                HandleNodeCreation(e);
            }
            else if (editor.creatingLinkNodeIndex >= 0 && editor.currentTab == 2)
            {
                HandleLinkCreation(e);
            }
            else if (e.type == EventType.MouseDown && e.button == 0 && !editor.isCreatingNode && editor.creatingLinkNodeIndex < 0)
            {
                if (editor.currentTab == 1) HandleNodeSelection(e);
                else if (editor.currentTab == 2) HandleLinkSelection(e);
            }

            if (editor.selectedNodeIndices.Count > 0 && editor.currentTab == 1)
            {
                HandleNodeTransform(e);
            }

            if (GUI.changed) EditorUtility.SetDirty(editor);
            HandleUtility.Repaint();
        }

        private void DrawNodesAndLinks()
        {
            for (int i = 0; i < editor.nodes.Count; i++)
            {
                Vector3 pos = editor.transform.TransformPoint(editor.nodes[i]);
                Handles.color = editor.pinnedNodes.Contains(i) ? editor.pinnedNodeColor : (editor.selectedNodeIndices.Contains(i) ? editor.selectedNodeColor : editor.nodeColor);
                float size = HandleUtility.GetHandleSize(pos) * editor.nodeSize;
                Handles.SphereHandleCap(0, pos, Quaternion.identity, size, EventType.Repaint);
            }

            for (int i = 0; i < editor.links.Count; i++)
            {
                Handles.color = editor.selectedLinkIndices.Contains(i) ? editor.selectedLinkColor : editor.linkColor;
                var link = editor.links[i];
                if (link.nodeA >= 0 && link.nodeA < editor.nodes.Count && link.nodeB >= 0 && link.nodeB < editor.nodes.Count)
                {
                    Vector3 posA = editor.transform.TransformPoint(editor.nodes[link.nodeA]);
                    Vector3 posB = editor.transform.TransformPoint(editor.nodes[link.nodeB]);
                    Handles.DrawLine(posA, posB, editor.linkThickness);
                }
            }
        }

        private void HandleNodeCreation(Event e)
        {
            Ray mouseRay = HandleUtility.GUIPointToWorldRay(e.mousePosition);
            RaycastHit hit;
            Vector3 position;
            bool hitValidPosition = Physics.Raycast(mouseRay, out hit);

            position = hitValidPosition ? hit.point : mouseRay.origin + mouseRay.direction * 10f;
            Handles.color = hitValidPosition ? Color.green : Color.red;
            Handles.SphereHandleCap(0, position, Quaternion.identity, editor.nodeSize, EventType.Repaint);

            if (e.type == EventType.MouseDown && e.button == 0)
            {
                position = editor.transform.InverseTransformPoint(position);
                editor.CreateNode(position);
                e.Use();
            }
        }

        private void HandleLinkCreation(Event e)
        {
            for (int i = 0; i < editor.nodes.Count; i++)
            {
                Vector3 position = editor.transform.TransformPoint(editor.nodes[i]);
                Handles.color = Color.blue;
                float size = HandleUtility.GetHandleSize(position) * editor.nodeSize * 2f;
                Handles.SphereHandleCap(0, position, Quaternion.identity, size, EventType.Repaint);

                if (e.type == EventType.MouseDown && e.button == 0)
                {
                    if (HandleUtility.DistanceToCircle(position, size) <= 0)
                    {
                        if (editor.creatingLinkNodeIndex == 0)
                        {
                            editor.creatingLinkNodeIndex = i;
                        }
                        else if (i != editor.creatingLinkNodeIndex)
                        {
                            editor.CreateLink(editor.creatingLinkNodeIndex, i);
                            editor.creatingLinkNodeIndex = -1;
                            e.Use();
                        }
                    }
                }
            }

            if (editor.creatingLinkNodeIndex >= 0 && editor.creatingLinkNodeIndex < editor.nodes.Count)
            {
                Vector3 startPos = editor.transform.TransformPoint(editor.nodes[editor.creatingLinkNodeIndex]);
                Ray mouseRay = HandleUtility.GUIPointToWorldRay(e.mousePosition);
                Vector3 endPos = mouseRay.origin + mouseRay.direction * 10f;
                Handles.color = Color.red;
                Handles.DrawLine(startPos, endPos, 2f);
            }
        }

        private void HandleNodeSelection(Event e)
        {
            for (int i = 0; i < editor.nodes.Count; i++)
            {
                Vector3 pos = editor.transform.TransformPoint(editor.nodes[i]);
                float size = HandleUtility.GetHandleSize(pos) * editor.nodeSize;
                if (HandleUtility.DistanceToCircle(pos, size) <= 0)
                {
                    Undo.RecordObject(editor, "Select Node");
                    if (e.control)
                    {
                        if (editor.selectedNodeIndices.Contains(i))
                            editor.selectedNodeIndices.Remove(i);
                        else
                            editor.selectedNodeIndices.Add(i);
                    }
                    else
                    {
                        editor.selectedNodeIndices.Clear();
                        editor.selectedNodeIndices.Add(i);
                    }
                    editor.selectedLinkIndices.Clear();
                    EditorUtility.SetDirty(editor);
                    e.Use();
                    break;
                }
            }
        }

        private void HandleLinkSelection(Event e)
        {
            for (int i = 0; i < editor.links.Count; i++)
            {
                var link = editor.links[i];
                if (link.nodeA >= 0 && link.nodeA < editor.nodes.Count && link.nodeB >= 0 && link.nodeB < editor.nodes.Count)
                {
                    Vector3 posA = editor.transform.TransformPoint(editor.nodes[link.nodeA]);
                    Vector3 posB = editor.transform.TransformPoint(editor.nodes[link.nodeB]);
                    Vector3 guiPointA = HandleUtility.WorldToGUIPoint(posA);
                    Vector3 guiPointB = HandleUtility.WorldToGUIPoint(posB);
                    if (HandleUtility.DistancePointToLineSegment(e.mousePosition, guiPointA, guiPointB) < 3f)
                    {
                        Undo.RecordObject(editor, "Select Link");
                        if (e.control)
                        {
                            if (editor.selectedLinkIndices.Contains(i))
                                editor.selectedLinkIndices.Remove(i);
                            else
                                editor.selectedLinkIndices.Add(i);
                        }
                        else
                        {
                            editor.selectedLinkIndices.Clear();
                            editor.selectedLinkIndices.Add(i);
                        }
                        editor.selectedNodeIndices.Clear();
                        EditorUtility.SetDirty(editor);
                        e.Use();
                        break;
                    }
                }
            }
        }

        private void HandleNodeTransform(Event e)
        {
            editor.selectedNodeIndices.RemoveAll(i => i < 0 || i >= editor.nodes.Count);
            if (editor.selectedNodeIndices.Count == 0) return;

            Vector3 pos = editor.transform.TransformPoint(editor.GetSelectionCenter());
            if (!isDragging)
            {
                handlePosition = pos;
            }

            EditorGUI.BeginChangeCheck();
            Vector3 newPos = Handles.PositionHandle(pos, Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                isDragging = true;
                Vector3 delta = editor.transform.InverseTransformPoint(newPos) - editor.transform.InverseTransformPoint(pos);
                editor.TransformSelectedNodes(delta);
                handlePosition = newPos;
            }
            else if (e.type == EventType.MouseUp)
            {
                isDragging = false;
            }
        }
    }
#endif

    [System.Serializable]
    public struct Link
    {
        public int nodeA;
        public int nodeB;
        public float springForce;
        public float damping;
        public float restLength;
    }

    [System.Serializable]
    public struct MaterialPreset
    {
        public string name;
        public float springForce;
        public float damping;
        public float nodeMass;
        public float maxStretchFactor;
        public float minStretchFactor;
    }
}
