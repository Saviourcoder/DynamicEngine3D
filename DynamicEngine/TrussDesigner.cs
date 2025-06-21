using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using SoftBodySystem;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    public class NodeLinkEditor : MonoBehaviour
    {
        [Header("Node, Link, and Face Data")]
        [SerializeField] public List<Vector3> Nodes = new List<Vector3>();
        [SerializeField] public List<Link> Links = new List<Link>();
        [SerializeField] public List<Face> Faces = new List<Face>();
        [SerializeField] public List<int> SelectedNodeIndices = new List<int>();
        [SerializeField] public List<int> SelectedLinkIndices = new List<int>();
        [SerializeField] public List<int> SelectedFaceIndices = new List<int>();
        [SerializeField] public List<int> PinnedNodes = new List<int>();

        [Header("Editor States")]
        [SerializeField] public bool IsCreatingNode = false;
        [SerializeField] public int CreatingLinkNodeIndex = -1;
        [SerializeField] public List<int> CreatingFaceNodeIndices = new List<int>();
        [SerializeField] public int CurrentTab = 0; // 0: Nodes, 1: Links, 2: Faces, 3: Visualization, 4: Debugging

        [Header("Physical Properties")]
        [SerializeField, Range(0.1f, 5.0f)] public float NodeMass = 0.5f;
        [SerializeField, Range(1.0f, 1.5f), Tooltip("Maximum stretch factor for beams (e.g., 1.05 = 105% of rest length)")]
        public float MaxStretchFactor = 1.05f;
        [SerializeField, Range(0.5f, 1.0f), Tooltip("Minimum stretch factor for beams (e.g., 0.95 = 95% of rest length)")]
        public float MinStretchFactor = 0.95f;

        [Header("Visualization Settings")]
        [SerializeField, Tooltip("Visualize forces (red: tension, blue: compression, green: collisions, magenta: ground)")]
        public bool VisualizeForces = false;
        [SerializeField, Tooltip("Size of node spheres in the Scene view")]
        public float NodeSize = 0.1f;
        [SerializeField, Tooltip("Thickness of link lines in the Scene view")]
        public float LinkThickness = 3f;
        [SerializeField, Tooltip("Color for unselected nodes")] public Color NodeColor = Color.red;
        [SerializeField, Tooltip("Color for selected nodes")] public Color SelectedNodeColor = Color.blue;
        [SerializeField, Tooltip("Color for pinned nodes")] public Color PinnedNodeColor = Color.magenta;
        [SerializeField, Tooltip("Color for unselected links")] public Color LinkColor = Color.yellow;
        [SerializeField, Tooltip("Color for selected links")] public Color SelectedLinkColor = Color.blue;
        [SerializeField, Tooltip("Color for unselected faces")] public Color FaceColor = Color.green;
        [SerializeField, Tooltip("Color for selected faces")] public Color SelectedFaceColor = Color.cyan;

        [Header("Debugging")]
        [SerializeField, Tooltip("Enable debug logs for operations like pinning and stretch limit application")]
        public bool EnableDebugLogs = false;

        [Header("References")]
        [SerializeField] public SoftBody SoftBodyReference;
        public SoftBody SoftBody => SoftBodyReference;

        [SerializeField]
        public List<MaterialPreset> MaterialPresets = new List<MaterialPreset>
        {
            new MaterialPreset { name = "Rubber", compliance = 1e-3f, damping = 0.3f, nodeMass = 0.5f, maxStretchFactor = 1.1f, minStretchFactor = 0.9f, plasticityThreshold = 0.1f, plasticityRate = 0.05f },
            new MaterialPreset { name = "Metal", compliance = 1e-6f, damping = 0.005f, nodeMass = 2.0f, maxStretchFactor = 1.02f, minStretchFactor = 0.98f, plasticityThreshold = 0.05f, plasticityRate = 0.1f },
            new MaterialPreset { name = "Plastic", compliance = 1e-4f, damping = 0.01f, nodeMass = 1.0f, maxStretchFactor = 1.05f, minStretchFactor = 0.95f, plasticityThreshold = 0.03f, plasticityRate = 0.2f }
        };

        private void Awake()
        {
            Initialize();
        }

        private void Update()
        {
            if (Application.isPlaying) return;
            UpdateVisualization();
        }

        private void Initialize()
        {
            if (Application.isPlaying)
            {
                if (EnableDebugLogs) Debug.Log("NodeLinkEditor is inactive during Play Mode.", this);
                return;
            }

            Nodes ??= new List<Vector3>();
            Links ??= new List<Link>();
            Faces ??= new List<Face>();
            SelectedNodeIndices ??= new List<int>();
            SelectedLinkIndices ??= new List<int>();
            SelectedFaceIndices ??= new List<int>();
            PinnedNodes ??= new List<int>();
            CreatingFaceNodeIndices ??= new List<int>();

            SoftBodyReference = GetComponent<SoftBody>();
            if (SoftBodyReference == null)
            {
                Debug.LogWarning("No SoftBody component found on this GameObject.", this);
                return;
            }

            LoadFromTrussAsset();
            ApplyStretchLimits();
            ValidateNodeConnectivity();
            ApplyPinnedNodes();
        }

        private void UpdateVisualization()
        {
            if (SoftBodyReference == null) return;
            var core = GetSoftBodyCore();
            if (core != null) core.visualizeForces = VisualizeForces;
        }

        private SoftBodyCore GetSoftBodyCore()
        {
            return typeof(SoftBody).GetField("core", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)?.GetValue(SoftBodyReference) as SoftBodyCore;
        }

        #region Node Management
        public void CreateNode(Vector3 position)
        {
            Undo.RecordObject(this, "Create Node");
            Nodes.Add(position);
            SelectedNodeIndices.Clear();
            SelectedNodeIndices.Add(Nodes.Count - 1);
            SelectedLinkIndices.Clear();
            SelectedFaceIndices.Clear();
            SaveToTrussAsset();
        }

        public void DeleteSelectedNodes()
        {
            SelectedNodeIndices.Sort((a, b) => b.CompareTo(a));
            foreach (int index in SelectedNodeIndices)
            {
                if (index >= 0 && index < Nodes.Count)
                {
                    PinnedNodes.Remove(index);
                    Nodes.RemoveAt(index);
                    Links.RemoveAll(link => link.nodeA == index || link.nodeB == index);
                    Faces.RemoveAll(face => face.nodeA == index || face.nodeB == index || face.nodeC == index);
                    UpdateLinkIndicesAfterNodeDeletion(index);
                    UpdateFaceIndicesAfterNodeDeletion(index);
                }
            }
            SelectedNodeIndices.Clear();
            SaveToTrussAsset();
        }

        private void UpdateLinkIndicesAfterNodeDeletion(int deletedIndex)
        {
            for (int i = 0; i < Links.Count; i++)
            {
                var link = Links[i];
                if (link.nodeA > deletedIndex) link.nodeA--;
                if (link.nodeB > deletedIndex) link.nodeB--;
                Links[i] = link;
            }
        }

        private void UpdateFaceIndicesAfterNodeDeletion(int deletedIndex)
        {
            for (int i = 0; i < Faces.Count; i++)
            {
                var face = Faces[i];
                if (face.nodeA > deletedIndex) face.nodeA--;
                if (face.nodeB > deletedIndex) face.nodeB--;
                if (face.nodeC > deletedIndex) face.nodeC--;
                Faces[i] = face;
            }
            for (int i = 0; i < PinnedNodes.Count; i++)
            {
                if (PinnedNodes[i] > deletedIndex) PinnedNodes[i]--;
            }
        }

        public void TransformSelectedNodes(Vector3 translation)
        {
            if (SelectedNodeIndices.Count == 0) return;
            Undo.RecordObject(this, "Transform Nodes");
            foreach (int index in SelectedNodeIndices.ToList())
            {
                if (index >= 0 && index < Nodes.Count && !PinnedNodes.Contains(index))
                {
                    Nodes[index] += translation;
                }
                else
                {
                    SelectedNodeIndices.Remove(index);
                }
            }
            UpdateLinkRestLengths();
            SaveToTrussAsset();
        }

        public Vector3 GetSelectionCenter()
        {
            if (SelectedNodeIndices.Count == 0) return Vector3.zero;
            Vector3 center = Vector3.zero;
            int validCount = 0;
            foreach (int index in SelectedNodeIndices.ToList())
            {
                if (index >= 0 && index < Nodes.Count)
                {
                    center += Nodes[index];
                    validCount++;
                }
                else
                {
                    SelectedNodeIndices.Remove(index);
                }
            }
            return validCount > 0 ? center / validCount : Vector3.zero;
        }
        #endregion

        #region Link Management
        public void CreateLink(int nodeA, int nodeB)
        {
            if (!IsValidLink(nodeA, nodeB)) return;
            float restLength = Vector3.Distance(Nodes[nodeA], Nodes[nodeB]);
            if (restLength < 0.01f)
            {
                Debug.LogWarning($"Cannot create link between nodes {nodeA} and {nodeB}: rest length {restLength} is too small.", this);
                return;
            }

            Undo.RecordObject(this, "Create Link");
            Links.Add(new Link { nodeA = nodeA, nodeB = nodeB, compliance = 1e-3f, damping = 0.3f, restLength = restLength });
            SelectedNodeIndices.Clear();
            SelectedLinkIndices.Clear();
            SelectedFaceIndices.Clear();
            SelectedLinkIndices.Add(Links.Count - 1);
            SaveToTrussAsset();
        }

        private bool IsValidLink(int nodeA, int nodeB)
        {
            if (nodeA == nodeB || nodeA < 0 || nodeB < 0 || nodeA >= Nodes.Count || nodeB >= Nodes.Count)
            {
                Debug.LogWarning($"Invalid link indices: nodeA={nodeA}, nodeB={nodeB}, Nodes.Count={Nodes.Count}.", this);
                return false;
            }
            return true;
        }

        public void DeleteSelectedLinks()
        {
            SelectedLinkIndices.Sort((a, b) => b.CompareTo(a));
            foreach (int index in SelectedLinkIndices)
            {
                if (index >= 0 && index < Links.Count)
                {
                    Links.RemoveAt(index);
                }
            }
            SelectedLinkIndices.Clear();
            SaveToTrussAsset();
        }

        public void UpdateLinkRestLengths()
        {
            for (int i = 0; i < Links.Count; i++)
            {
                var link = Links[i];
                if (link.nodeA < Nodes.Count && link.nodeB < Nodes.Count)
                {
                    link.restLength = Vector3.Distance(Nodes[link.nodeA], Nodes[link.nodeB]);
                    Links[i] = link;
                }
            }
        }
        #endregion

        #region Face Management
        public void CreateFace(int nodeA, int nodeB, int nodeC)
        {
            if (!IsValidFace(nodeA, nodeB, nodeC)) return;
            Undo.RecordObject(this, "Create Face");
            Faces.Add(new Face { nodeA = nodeA, nodeB = nodeB, nodeC = nodeC });
            SelectedNodeIndices.Clear();
            SelectedLinkIndices.Clear();
            SelectedFaceIndices.Clear();
            SelectedFaceIndices.Add(Faces.Count - 1);
            SaveToTrussAsset();
        }

        private bool IsValidFace(int nodeA, int nodeB, int nodeC)
        {
            if (nodeA == nodeB || nodeB == nodeC || nodeA == nodeC ||
                nodeA < 0 || nodeB < 0 || nodeC < 0 ||
                nodeA >= Nodes.Count || nodeB >= Nodes.Count || nodeC >= Nodes.Count)
            {
                Debug.LogWarning($"Invalid face indices: nodeA={nodeA}, nodeB={nodeB}, nodeC={nodeC}, Nodes.Count={Nodes.Count}.", this);
                return false;
            }
            return true;
        }

        public void DeleteSelectedFaces()
        {
            SelectedFaceIndices.Sort((a, b) => b.CompareTo(a));
            foreach (int index in SelectedFaceIndices)
            {
                if (index >= 0 && index < Faces.Count)
                {
                    Faces.RemoveAt(index);
                }
            }
            SelectedFaceIndices.Clear();
            SaveToTrussAsset();
        }
        #endregion

        #region Deletion
        public void DeleteSelected()
        {
            Undo.RecordObject(this, "Delete Selection");
            if (CurrentTab == 0 && SelectedNodeIndices.Count > 0)
            {
                DeleteSelectedNodes();
            }
            else if (CurrentTab == 1 && SelectedLinkIndices.Count > 0)
            {
                DeleteSelectedLinks();
            }
            else if (CurrentTab == 2 && SelectedFaceIndices.Count > 0)
            {
                DeleteSelectedFaces();
            }
            ApplyPinnedNodes();
            SaveToTrussAsset();
        }
        #endregion

        #region Truss Asset Management
        public void LoadFromTrussAsset()
        {
            if (SoftBodyReference == null || SoftBodyReference.GetTrussAsset() == null)
            {
                if (EnableDebugLogs) Debug.LogWarning("Cannot load from TrussAsset: SoftBody or TrussAsset is missing.", this);
                return;
            }

            TrussAsset truss = SoftBodyReference.GetTrussAsset();
            Nodes = truss.NodePositions?.ToList() ?? new List<Vector3>();
            Links.Clear();
            Faces.Clear();
            foreach (var beam in truss.GetTrussBeams())
            {
                Links.Add(new Link
                {
                    nodeA = beam.nodeA,
                    nodeB = beam.nodeB,
                    compliance = beam.compliance,
                    damping = beam.damping,
                    restLength = beam.restLength
                });
            }
            foreach (var face in truss.GetTrussFaces())
            {
                Faces.Add(new Face
                {
                    nodeA = face.nodeA,
                    nodeB = face.nodeB,
                    nodeC = face.nodeC
                });
            }
            SelectedNodeIndices.Clear();
            SelectedLinkIndices.Clear();
            SelectedFaceIndices.Clear();
            PinnedNodes.Clear();
            if (EnableDebugLogs) Debug.Log($"Loaded {Nodes.Count} nodes, {Links.Count} links, and {Faces.Count} faces from TrussAsset.", this);
            ValidateNodeConnectivity();
        }

        public void SaveToTrussAsset()
        {
            if (SoftBodyReference == null || SoftBodyReference.GetTrussAsset() == null)
            {
                if (EnableDebugLogs) Debug.LogWarning("Cannot save to TrussAsset: SoftBody or TrussAsset is missing.", this);
                return;
            }

            if (GetSoftBodyCore() == null)
            {
                if (EnableDebugLogs) Debug.LogWarning("Cannot save to TrussAsset: SoftBodyCore is not initialized.", this);
                return;
            }

            TrussAsset truss = SoftBodyReference.GetTrussAsset();
            truss.SetNodePositions(Nodes.ToArray());
            var trussBeams = Links
            .Where(link => link.nodeA >= 0 && link.nodeA < Nodes.Count && link.nodeB >= 0 && link.nodeB < Nodes.Count)
            .Select(link => new TrussAsset.TrussBeam(
            link.nodeA,
            link.nodeB,
            link.compliance,
            link.damping,
            link.restLength
             )).ToList();
            var trussFaces = Faces
                .Where(face => face.nodeA >= 0 && face.nodeA < Nodes.Count && face.nodeB >= 0 && face.nodeB < Nodes.Count && face.nodeC >= 0 && face.nodeC < Nodes.Count)
                .Select(face => new TrussAsset.TrussFace(face.nodeA, face.nodeB, face.nodeC))
                .ToList();
            truss.SetBeams(trussBeams);
            truss.SetFaces(trussFaces);
            SoftBodyReference.ApplyTrussAsset(truss);

#if UNITY_EDITOR
            EditorUtility.SetDirty(truss);
            EditorUtility.SetDirty(SoftBodyReference);
#endif
            if (EnableDebugLogs) Debug.Log($"Saved {Nodes.Count} nodes, {trussBeams.Count} beams, and {trussFaces.Count} faces to TrussAsset.", this);
            ValidateNodeConnectivity();
        }
        #endregion

        #region Physics Settings
        public void ApplyStretchLimits()
        {
            if (SoftBodyReference == null)
            {
                if (EnableDebugLogs) Debug.LogWarning("Cannot apply stretch limits: SoftBody is missing.", this);
                return;
            }

            MaxStretchFactor = Mathf.Clamp(MaxStretchFactor, 1.0f, 1.5f);
            MinStretchFactor = Mathf.Clamp(MinStretchFactor, 0.5f, 1.0f);
            var core = GetSoftBodyCore();
            if (core == null) return;

            try
            {
                var maxField = typeof(SoftBodyCore).GetField("maxStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                var minField = typeof(SoftBodyCore).GetField("minStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                if (maxField != null && minField != null)
                {
                    maxField.SetValue(core, MaxStretchFactor);
                    minField.SetValue(core, MinStretchFactor);
                    if (EnableDebugLogs)
                        Debug.Log($"Applied stretch limits to SoftBodyCore: maxStretchFactor={MaxStretchFactor}, minStretchFactor={MinStretchFactor}", this);
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
            if (SoftBodyReference == null)
            {
                if (EnableDebugLogs) Debug.LogWarning("Cannot apply pinned nodes: SoftBody is missing.", this);
                return;
            }

            var core = GetSoftBodyCore();
            if (core == null) return;

            try
            {
                var pinMethod = typeof(SoftBodyCore).GetMethod("PinNode", System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
                var unpinMethod = typeof(SoftBodyCore).GetMethod("UnpinNode", System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
                if (pinMethod != null && unpinMethod != null)
                {
                    for (int i = 0; i < Nodes.Count; i++)
                    {
                        unpinMethod.Invoke(core, new object[] { i });
                    }
                    foreach (var nodeIndex in PinnedNodes)
                    {
                        if (nodeIndex >= 0 && nodeIndex < Nodes.Count)
                        {
                            pinMethod.Invoke(core, new object[] { nodeIndex });
                            if (EnableDebugLogs) Debug.Log($"Pinned node {nodeIndex} in SoftBodyCore.", this);
                        }
                        else
                        {
                            Debug.LogWarning($"Invalid pinned node index: {nodeIndex}.", this);
                        }
                    }
                }
                else
                {
                    Debug.LogWarning("PinNode/UnpinNode methods not implemented in SoftBodyCore. Add these for pinning functionality.", this);
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Failed to apply pinned nodes: {e.Message}", this);
            }
        }

        public void ApplyRigidSettings()
        {
            Undo.RecordObject(this, "Apply Rigid Settings");
            MaxStretchFactor = 1.02f;
            MinStretchFactor = 0.98f;
            NodeMass = 2.0f;
            UpdateLinks(1e-6f, 0.005f);
            ApplyStretchLimits();
            SaveToTrussAsset();
            if (EnableDebugLogs)
                Debug.Log("Applied rigid settings: maxStretchFactor=1.02, minStretchFactor=0.98, compliance=1e-6, damping=0.005, nodeMass=2.0.", this);
        }

        public void ResetToDefaultSettings()
        {
            Undo.RecordObject(this, "Reset to Default Settings");
            MaxStretchFactor = 1.05f;
            MinStretchFactor = 0.95f;
            NodeMass = 0.5f;
            UpdateLinks(1e-3f, 0.3f);
            ApplyStretchLimits();
            SaveToTrussAsset();
            if (EnableDebugLogs)
                Debug.Log("Reset to default settings: maxStretchFactor=1.05, minStretchFactor=0.95, compliance=1e-3, damping=0.3, nodeMass=0.5.", this);
        }

        private void UpdateLinks(float compliance, float damping)
        {
            var updatedLinks = new List<Link>();
            foreach (var link in Links)
            {
                var updatedLink = link;
                updatedLink.compliance = compliance;
                updatedLink.damping = damping;
                updatedLinks.Add(updatedLink);
            }
            Links = updatedLinks;
        }

        public void PinSelectedNodes()
        {
            Undo.RecordObject(this, "Pin Selected Nodes");
            foreach (var index in SelectedNodeIndices)
            {
                if (index >= 0 && index < Nodes.Count && !PinnedNodes.Contains(index))
                {
                    PinnedNodes.Add(index);
                    if (EnableDebugLogs) Debug.Log($"Pinned node {index}.", this);
                }
            }
            ApplyPinnedNodes();
        }

        public void UnpinSelectedNodes()
        {
            Undo.RecordObject(this, "Unpin Selected Nodes");
            foreach (var index in SelectedNodeIndices)
            {
                if (PinnedNodes.Contains(index))
                {
                    PinnedNodes.Remove(index);
                    if (EnableDebugLogs) Debug.Log($"Unpinned node {index}.", this);
                }
            }
            ApplyPinnedNodes();
        }
        #endregion

        #region Validation
        public void ValidateNodeConnectivity()
        {
            if (Nodes.Count == 0 || Links.Count == 0)
            {
                if (EnableDebugLogs) Debug.LogWarning("No nodes or links to validate.", this);
                return;
            }

            Dictionary<int, int> nodeConnections = new Dictionary<int, int>();
            for (int i = 0; i < Nodes.Count; i++)
                nodeConnections[i] = 0;

            foreach (var link in Links)
            {
                if (link.nodeA >= 0 && link.nodeA < Nodes.Count) nodeConnections[link.nodeA]++;
                if (link.nodeB >= 0 && link.nodeB < Nodes.Count) nodeConnections[link.nodeB]++;
            }

            bool hasIssues = false;
            foreach (var pair in nodeConnections)
            {
                if (pair.Value < 2)
                {
                    Debug.LogWarning($"Node {pair.Key} has only {pair.Value} connections. Consider adding more links for stability.", this);
                    hasIssues = true;
                }
                else if (pair.Value > 8)
                {
                    Debug.LogWarning($"Node {pair.Key} has {pair.Value} connections. Excessive connections may cause instability.", this);
                    hasIssues = true;
                }
            }

            if (nodeConnections.ContainsKey(6) && nodeConnections.ContainsKey(5))
            {
                if (EnableDebugLogs)
                    Debug.Log($"Beam 12 (nodes 6-5): Node 6 has {nodeConnections[6]} connections, Node 5 has {nodeConnections[5]} connections.", this);
            }

            if (!hasIssues && EnableDebugLogs)
                Debug.Log("Node connectivity validated: All nodes have 2-8 connections.", this);
        }
        #endregion

        #region SoftBody Synchronization
        public void SyncToSoftBodyCore(SoftBodyCore core, Transform transform)
        {
            if (core == null)
            {
                Debug.LogWarning("Cannot sync to null SoftBodyCore.", this);
                return;
            }

            List<Beam> beams = Links
                .Where(link => link.nodeA >= 0 && link.nodeA < Nodes.Count && link.nodeB >= 0 && link.nodeB < Nodes.Count && link.restLength > 0.01f)
                .Select(link => new Beam(link.nodeA, link.nodeB, link.compliance, link.damping, link.restLength))
                .ToList();

            core.GenerateNodesAndBeams(Nodes.ToArray(), beams.ToArray(), transform);

            try
            {
                var maxField = typeof(SoftBodyCore).GetField("maxStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                var minField = typeof(SoftBodyCore).GetField("minStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                if (maxField != null && minField != null)
                {
                    maxField.SetValue(core, Mathf.Max(1.0f, MaxStretchFactor));
                    minField.SetValue(core, Mathf.Min(1.0f, MinStretchFactor));
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Failed to sync stretch limits: {e.Message}", this);
            }

            ApplyPinnedNodes();
            if (EnableDebugLogs)
                Debug.Log($"Synced {Nodes.Count} nodes and {beams.Count} beams to SoftBodyCore with stretch limits: max={MaxStretchFactor}, min={MinStretchFactor}.", this);
        }
        #endregion
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(NodeLinkEditor))]
    public class NodeLinkEditorCustom : Editor
    {
        public NodeLinkEditor EditorTarget;
        public Vector3 HandlePosition;
        public bool IsDragging;
        public bool StretchLimitsFoldout = true;
        public bool MaterialPresetsFoldout = true;
        public Dictionary<int, int> NodeConnections = new Dictionary<int, int>();

        private void OnEnable()
        {
            EditorTarget = (NodeLinkEditor)target;
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
            EditorGUILayout.LabelField("Node & Link Editor", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            DrawSoftBodyReference();
            EditorGUILayout.Space();
            DrawTabs();
        }

        private void DrawSoftBodyReference()
        {
            EditorGUILayout.BeginHorizontal();
            GUI.enabled = false;
            EditorGUILayout.ObjectField("SoftBody", EditorTarget.SoftBody, typeof(SoftBody), true);
            GUI.enabled = true;
            if (GUILayout.Button("Reassign SoftBody", GUILayout.Width(120)))
            {
                Undo.RecordObject(EditorTarget, "Reassign SoftBody");
                EditorTarget.SoftBodyReference = EditorTarget.GetComponent<SoftBody>();
                if (EditorTarget.SoftBodyReference != null)
                {
                    EditorTarget.LoadFromTrussAsset();
                    EditorTarget.ApplyStretchLimits();
                    EditorTarget.ValidateNodeConnectivity();
                    EditorTarget.ApplyPinnedNodes();
                    EditorUtility.SetDirty(EditorTarget);
                }
                else
                {
                    Debug.LogWarning("No SoftBody component found on this GameObject.", EditorTarget);
                }
            }
            if (GUILayout.Button("Reload TrussAsset", GUILayout.Width(120)))
            {
                EditorTarget.LoadFromTrussAsset();
            }
            EditorGUILayout.EndHorizontal();
        }

        private void DrawTabs()
        {
            string[] tabs = { "Nodes", "Links", "Faces", "Stretch Limits", "Visualization", "Debugging" };
            EditorTarget.CurrentTab = GUILayout.Toolbar(EditorTarget.CurrentTab, tabs);
            EditorGUILayout.Space();

            switch (EditorTarget.CurrentTab)
            {
                case 0: DrawNodesTab(); break;
                case 1: DrawLinksTab(); break;
                case 2: DrawFacesTab(); break;
                case 3: DrawStretchLimitsTab(); break;
                case 4: DrawVisualizationTab(); break;
                case 5: DrawDebuggingTab(); break;
            }
        }

        private void DrawNodesTab()
        {
            EditorTarget.IsCreatingNode = EditorGUILayout.Toggle(new GUIContent("Create Node Mode", "Click on a surface to place a new node"), EditorTarget.IsCreatingNode);
            if (EditorTarget.IsCreatingNode)
            {
                EditorGUILayout.LabelField("Click on a surface to place node", new GUIStyle(EditorStyles.label) { normal = { textColor = Color.green } });
                EditorTarget.CreatingLinkNodeIndex = -1;
                EditorTarget.CreatingFaceNodeIndices.Clear();
            }

            EditorGUILayout.LabelField("Node Mass", EditorStyles.boldLabel);
            EditorTarget.NodeMass = EditorGUILayout.FloatField(new GUIContent("Mass", "Mass of each node for physics simulation"), Mathf.Max(0.1f, EditorTarget.NodeMass));

            string nodeInfo = EditorTarget.SelectedNodeIndices.Count switch
            {
                0 => "No Nodes Selected",
                1 => $"Node {EditorTarget.SelectedNodeIndices[0]} Selected",
                _ => $"{EditorTarget.SelectedNodeIndices.Count} Nodes Selected"
            };
            EditorGUILayout.LabelField(nodeInfo, new GUIStyle(EditorStyles.label) { alignment = TextAnchor.MiddleRight });

            if (EditorTarget.SelectedNodeIndices.Count > 0)
            {
                EditorGUILayout.LabelField("Selected Node Positions", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                foreach (var idx in EditorTarget.SelectedNodeIndices)
                {
                    if (idx >= 0 && idx < EditorTarget.Nodes.Count)
                    {
                        EditorGUILayout.LabelField($"Node {idx}: {EditorTarget.Nodes[idx]}");
                    }
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Delete Selected")) EditorTarget.DeleteSelected();
            if (GUILayout.Button("Select All"))
            {
                EditorTarget.SelectedNodeIndices.Clear();
                for (int i = 0; i < EditorTarget.Nodes.Count; i++) EditorTarget.SelectedNodeIndices.Add(i);
            }
            if (GUILayout.Button("Clear Selection")) EditorTarget.SelectedNodeIndices.Clear();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Pin Selected")) EditorTarget.PinSelectedNodes();
            if (GUILayout.Button("Unpin Selected")) EditorTarget.UnpinSelectedNodes();
            EditorGUILayout.EndHorizontal();

            EditorTarget.CreatingLinkNodeIndex = -1;
            EditorTarget.CreatingFaceNodeIndices.Clear();
        }

        private void DrawLinksTab()
        {
            if (EditorTarget == null) return;

            bool isCreatingLink = EditorTarget.CreatingLinkNodeIndex >= 0;
            EditorTarget.IsCreatingNode = false;
            EditorTarget.CreatingFaceNodeIndices.Clear();

            string linkInfo = EditorTarget.SelectedLinkIndices.Count switch
            {
                0 => "No Links Selected",
                1 => $"Link {EditorTarget.SelectedLinkIndices[0]} Selected",
                _ => $"{EditorTarget.SelectedLinkIndices.Count} Links Selected"
            };
            EditorGUILayout.LabelField(linkInfo, new GUIStyle(EditorStyles.label) { alignment = TextAnchor.MiddleRight });

            if (EditorTarget.SelectedLinkIndices.Count > 0 && EditorTarget.Links != null)
            {
                EditorGUILayout.LabelField("Selected Link Details", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                foreach (var idx in EditorTarget.SelectedLinkIndices)
                {
                    if (idx >= 0 && idx < EditorTarget.Links.Count)
                    {
                        var link = EditorTarget.Links[idx];
                        EditorGUILayout.LabelField($"Link {idx}: Nodes {link.nodeA}-{link.nodeB}, Compliance: {link.compliance}, Damping: {link.damping}, Rest Length: {link.restLength}");
                    }
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.BeginHorizontal();
            try
            {
                bool createLinkPressed = GUILayout.Button("Create Link", isCreatingLink ? new GUIStyle(GUI.skin.button) { normal = { background = Texture2D.grayTexture } } : GUI.skin.button);
                if (createLinkPressed)
                {
                    EditorTarget.CreatingLinkNodeIndex = isCreatingLink ? -1 : 0;
                }
                if (GUILayout.Button("Delete Selected")) EditorTarget.DeleteSelected();
                if (GUILayout.Button("Select All"))
                {
                    EditorTarget.SelectedLinkIndices.Clear();
                    if (EditorTarget.Links != null)
                    {
                        for (int i = 0; i < EditorTarget.Links.Count; i++)
                        {
                            EditorTarget.SelectedLinkIndices.Add(i);
                        }
                    }
                }
                if (GUILayout.Button("Clear Selection")) EditorTarget.SelectedLinkIndices.Clear();
            }
            finally
            {
                EditorGUILayout.EndHorizontal();
            }

            MaterialPresetsFoldout = EditorGUILayout.Foldout(MaterialPresetsFoldout, "Material Presets", true, EditorStyles.boldLabel);
            if (MaterialPresetsFoldout && EditorTarget.MaterialPresets != null)
            {
                foreach (var preset in EditorTarget.MaterialPresets)
                {
                    EditorGUILayout.BeginHorizontal();
                    try
                    {
                        if (GUILayout.Button(preset.name))
                        {
                            ApplyMaterialPresetToSelectedLinks(preset);
                        }
                    }
                    finally
                    {
                        EditorGUILayout.EndHorizontal();
                    }
                }
            }

            if (EditorTarget.SelectedLinkIndices.Count > 0 && EditorTarget.Links != null)
            {
                EditorGUILayout.LabelField("Selected Link Properties", EditorStyles.boldLabel);
                var firstLink = EditorTarget.Links[EditorTarget.SelectedLinkIndices[0]];
                float compliance = firstLink.compliance;
                float damping = firstLink.damping;
                bool uniform = true;

                foreach (int idx in EditorTarget.SelectedLinkIndices)
                {
                    if (idx >= 0 && idx < EditorTarget.Links.Count)
                    {
                        var link = EditorTarget.Links[idx];
                        if (link.compliance != compliance || link.damping != damping)
                        {
                            uniform = false;
                            break;
                        }
                    }
                }

                EditorGUI.BeginChangeCheck();
                compliance = EditorGUILayout.FloatField(new GUIContent("Compliance", "Inverse stiffness (lower = stiffer)"), Mathf.Clamp(compliance, 1e-8f, 1e-2f));
                damping = EditorGUILayout.FloatField(new GUIContent("Damping", "Damping factor to reduce oscillations"), Mathf.Clamp(damping, 0.001f, 1.0f));
                if (EditorGUI.EndChangeCheck() && uniform)
                {
                    Undo.RecordObject(EditorTarget, "Modify Link Properties");
                    for (int i = 0; i < EditorTarget.SelectedLinkIndices.Count; i++)
                    {
                        int idx = EditorTarget.SelectedLinkIndices[i];
                        if (idx >= 0 && idx < EditorTarget.Links.Count)
                        {
                            var link = EditorTarget.Links[idx];
                            link.compliance = Mathf.Max(1e-8f, compliance);
                            link.damping = Mathf.Max(0.001f, damping);
                            EditorTarget.Links[idx] = link;
                        }
                    }
                    EditorTarget.SaveToTrussAsset();
                }
                else if (!uniform)
                {
                    EditorGUILayout.HelpBox("Multiple links selected with different properties. Values shown are for the first link.", MessageType.Info);
                }
            }
        }

        private void DrawFacesTab()
        {
            if (EditorTarget == null) return;

            bool isCreatingFace = EditorTarget.CreatingFaceNodeIndices.Count > 0;
            EditorTarget.IsCreatingNode = false;
            EditorTarget.CreatingLinkNodeIndex = -1;

            string faceInfo = EditorTarget.SelectedFaceIndices.Count switch
            {
                0 => "No Faces Selected",
                1 => $"Face {EditorTarget.SelectedFaceIndices[0]} Selected",
                _ => $"{EditorTarget.SelectedFaceIndices.Count} Faces Selected"
            };
            EditorGUILayout.LabelField(faceInfo, new GUIStyle(EditorStyles.label) { alignment = TextAnchor.MiddleRight });

            if (EditorTarget.SelectedFaceIndices.Count > 0 && EditorTarget.Faces != null)
            {
                EditorGUILayout.LabelField("Selected Face Details", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                foreach (var idx in EditorTarget.SelectedFaceIndices)
                {
                    if (idx >= 0 && idx < EditorTarget.Faces.Count)
                    {
                        var face = EditorTarget.Faces[idx];
                        EditorGUILayout.LabelField($"Face {idx}: Nodes {face.nodeA}-{face.nodeB}-{face.nodeC}");
                    }
                }
                EditorGUI.indentLevel--;
            }

            EditorGUILayout.BeginHorizontal();
            try
            {
                bool createFacePressed = GUILayout.Button("Create Face", isCreatingFace ? new GUIStyle(GUI.skin.button) { normal = { background = Texture2D.grayTexture } } : GUI.skin.button);
                if (createFacePressed)
                {
                    EditorTarget.CreatingFaceNodeIndices.Clear();
                    if (!isCreatingFace) EditorTarget.CreatingFaceNodeIndices.Add(-1);
                }
                if (GUILayout.Button("Delete Selected")) EditorTarget.DeleteSelected();
                if (GUILayout.Button("Select All"))
                {
                    EditorTarget.SelectedFaceIndices.Clear();
                    if (EditorTarget.Faces != null)
                    {
                        for (int i = 0; i < EditorTarget.Faces.Count; i++) EditorTarget.SelectedFaceIndices.Add(i);
                    }
                }
                if (GUILayout.Button("Clear Selection")) EditorTarget.SelectedFaceIndices.Clear();
            }
            finally
            {
                EditorGUILayout.EndHorizontal();
            }

            if (isCreatingFace)
            {
                EditorGUILayout.LabelField($"Select {3 - EditorTarget.CreatingFaceNodeIndices.Count} more node(s)", new GUIStyle(EditorStyles.label) { normal = { textColor = Color.blue } });
            }
        }

        private void DrawStretchLimitsTab()
        {
            if (EditorTarget == null) return;

            EditorGUILayout.LabelField("Stretch Limits Settings", EditorStyles.boldLabel);
            EditorGUI.indentLevel++;

            EditorGUI.BeginChangeCheck();
            float newMaxFactor = EditorGUILayout.FloatField(new GUIContent("Max Stretch Factor", "Maximum allowable stretch (e.g., 1.05 = 105% of rest length)"), EditorTarget.MaxStretchFactor);
            float newMinFactor = EditorGUILayout.FloatField(new GUIContent("Min Stretch Factor", "Minimum allowable stretch (e.g., 0.95 = 95% of rest length)"), EditorTarget.MinStretchFactor);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(EditorTarget, "Change Stretch Limits");
                EditorTarget.MaxStretchFactor = Mathf.Clamp(newMaxFactor, 1.0f, 1.5f);
                EditorTarget.MinStretchFactor = Mathf.Clamp(newMinFactor, 0.5f, 1.0f);
                EditorTarget.ApplyStretchLimits();
                serializedObject.FindProperty("MaxStretchFactor").floatValue = EditorTarget.MaxStretchFactor;
                serializedObject.FindProperty("MinStretchFactor").floatValue = EditorTarget.MinStretchFactor;
            }

            EditorGUILayout.BeginHorizontal();
            try
            {
                if (GUILayout.Button("Apply Rigid Settings")) EditorTarget.ApplyRigidSettings();
                if (GUILayout.Button("Reset to Defaults")) EditorTarget.ResetToDefaultSettings();
            }
            finally
            {
                EditorGUILayout.EndHorizontal();
            }

            EditorGUI.indentLevel--;
        }

        private void DrawVisualizationTab()
        {
            EditorGUILayout.LabelField("Visualization Settings", EditorStyles.boldLabel);
            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(serializedObject.FindProperty("VisualizeForces"), new GUIContent("Visualize Forces", "Show force vectors in simulation (red: tension, blue: compression)"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("NodeSize"), new GUIContent("Node Size", "Size of node spheres in Scene view"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("LinkThickness"), new GUIContent("Link Thickness", "Thickness of link lines in Scene view"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("NodeColor"), new GUIContent("Node Color", "Color for unselected nodes"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("SelectedNodeColor"), new GUIContent("Selected Node Color"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("PinnedNodeColor"), new GUIContent("Pinned Node Color"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("LinkColor"), new GUIContent("Link Color", "Color for unselected links"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("SelectedLinkColor"), new GUIContent("Selected Link Color"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("FaceColor"), new GUIContent("Face Color", "Color for unselected faces"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("SelectedFaceColor"), new GUIContent("Selected Face Color"));
            EditorGUI.indentLevel--;
            EditorTarget.IsCreatingNode = false;
            EditorTarget.CreatingLinkNodeIndex = -1;
            EditorTarget.CreatingFaceNodeIndices.Clear();
        }

        private void DrawDebuggingTab()
        {
            EditorGUILayout.LabelField("Debugging Tools", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("EnableDebugLogs"), new GUIContent("Enable Debug Logs", "Log detailed operations like pinning and stretch limit application"));

            if (GUILayout.Button("Validate Node Connections"))
            {
                NodeConnections.Clear();
                if (EditorTarget.Nodes.Count == 0 || EditorTarget.Links.Count == 0)
                {
                    EditorGUILayout.LabelField("No nodes or links to validate.");
                }
                else
                {
                    for (int i = 0; i < EditorTarget.Nodes.Count; i++)
                    {
                        NodeConnections[i] = 0;
                    }
                    foreach (var link in EditorTarget.Links)
                    {
                        if (link.nodeA >= 0 && link.nodeA < EditorTarget.Nodes.Count)
                            NodeConnections[link.nodeA]++;
                        if (link.nodeB >= 0 && link.nodeB < EditorTarget.Nodes.Count)
                            NodeConnections[link.nodeB]++;
                    }
                }
            }

            if (NodeConnections.Count > 0)
            {
                EditorGUILayout.LabelField("Node Connectivity Details", EditorStyles.boldLabel);
                EditorGUI.indentLevel++;
                foreach (var pair in NodeConnections.OrderBy(kvp => kvp.Key))
                {
                    EditorGUILayout.LabelField($"Node {pair.Key}: {pair.Value} connections");
                }
                EditorGUI.indentLevel--;
            }

            if (GUILayout.Button("Reapply Truss Limits and Pinned Nodes"))
            {
                EditorTarget.ApplyStretchLimits();
                EditorTarget.ApplyPinnedNodes();
            }
            EditorTarget.IsCreatingNode = false;
            EditorTarget.CreatingLinkNodeIndex = -1;
            EditorTarget.CreatingFaceNodeIndices.Clear();
        }

        private void ApplyMaterialPresetToSelectedLinks(MaterialPreset preset)
        {
            Undo.RecordObject(EditorTarget, "Apply Material Preset");
            for (int i = 0; i < EditorTarget.SelectedLinkIndices.Count; i++)
            {
                int idx = EditorTarget.SelectedLinkIndices[i];
                if (idx >= 0 && idx < EditorTarget.Links.Count)
                {
                    var link = EditorTarget.Links[idx];
                    link.compliance = preset.compliance;
                    link.damping = preset.damping;
                    EditorTarget.Links[idx] = link;
                }
            }
            EditorTarget.NodeMass = preset.nodeMass;
            EditorTarget.MaxStretchFactor = preset.maxStretchFactor;
            EditorTarget.MinStretchFactor = preset.minStretchFactor;
            EditorTarget.ApplyStretchLimits();
            EditorTarget.SaveToTrussAsset();

            // Apply material preset to SoftBody
            if (EditorTarget.SoftBodyReference != null)
            {
                var materialField = typeof(SoftBody).GetField("customMaterialProps", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                if (materialField != null)
                {
                    materialField.SetValue(EditorTarget.SoftBodyReference, new MaterialProperties(
                        preset.nodeMass,
                        preset.compliance,
                        preset.damping,
                        0.1f, // deformationScale (default)
                        0.5f, // maxDeformation (default)
                        preset.plasticityThreshold,
                        preset.plasticityRate
                    ));
                    EditorTarget.SoftBodyReference.ApplyTrussAsset(EditorTarget.SoftBodyReference.GetTrussAsset());
                    EditorUtility.SetDirty(EditorTarget.SoftBodyReference);
                }
            }
        }

        public void OnSceneGUI()
        {
            Event e = Event.current;
            int controlID = GUIUtility.GetControlID(FocusType.Passive);
            HandleUtility.AddDefaultControl(controlID);

            if (EditorTarget.CurrentTab <= 2) // Nodes, Links, Faces
            {
                DrawNodesAndLinks();
                DrawFaces();
            }

            if (EditorTarget.IsCreatingNode && EditorTarget.CurrentTab == 0)
            {
                HandleNodeCreation(e);
            }
            else if (EditorTarget.CreatingLinkNodeIndex >= 0 && EditorTarget.CurrentTab == 1)
            {
                HandleLinkCreation(e);
            }
            else if (EditorTarget.CreatingFaceNodeIndices.Count >= 1 && EditorTarget.CurrentTab == 2)
            {
                HandleFaceCreation(e);
            }
            else if (e.type == EventType.MouseDown && e.button == 0 && !EditorTarget.IsCreatingNode && EditorTarget.CreatingLinkNodeIndex == -1 && EditorTarget.CreatingFaceNodeIndices.Count == 0)
            {
                if (EditorTarget.CurrentTab == 0) HandleNodeSelection(e);
                else if (EditorTarget.CurrentTab == 1) HandleLinkSelection(e);
                else if (EditorTarget.CurrentTab == 2) HandleFaceSelection(e);
            }

            if (EditorTarget.SelectedNodeIndices.Count > 0 && EditorTarget.CurrentTab == 0)
            {
                HandleNodeTransform(e);
            }

            if (GUI.changed) EditorUtility.SetDirty(EditorTarget);
            HandleUtility.Repaint();
        }

        private void DrawNodesAndLinks()
        {
            for (int i = 0; i < EditorTarget.Nodes.Count; i++)
            {
                Vector3 pos = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[i]);
                Handles.color = EditorTarget.PinnedNodes.Contains(i) ? EditorTarget.PinnedNodeColor : (EditorTarget.SelectedNodeIndices.Contains(i) ? EditorTarget.SelectedNodeColor : EditorTarget.NodeColor);
                float size = HandleUtility.GetHandleSize(pos) * EditorTarget.NodeSize;
                Handles.SphereHandleCap(0, pos, Quaternion.identity, size, EventType.Repaint);
            }

            for (int i = 0; i < EditorTarget.Links.Count; i++)
            {
                Handles.color = EditorTarget.SelectedLinkIndices.Contains(i) ? EditorTarget.SelectedLinkColor : EditorTarget.LinkColor;
                var link = EditorTarget.Links[i];
                if (link.nodeA >= 0 && link.nodeA < EditorTarget.Nodes.Count && link.nodeB >= 0 && link.nodeB < EditorTarget.Nodes.Count)
                {
                    Vector3 posA = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[link.nodeA]);
                    Vector3 posB = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[link.nodeB]);
                    Handles.DrawLine(posA, posB, EditorTarget.LinkThickness);
                }
            }
        }

        private void DrawFaces()
        {
            for (int i = 0; i < EditorTarget.Faces.Count; i++)
            {
                var face = EditorTarget.Faces[i];
                if (face.nodeA >= 0 && face.nodeA < EditorTarget.Nodes.Count &&
                    face.nodeB >= 0 && face.nodeB < EditorTarget.Nodes.Count &&
                    face.nodeC >= 0 && face.nodeC < EditorTarget.Nodes.Count)
                {
                    Handles.color = EditorTarget.SelectedFaceIndices.Contains(i) ? EditorTarget.SelectedFaceColor : EditorTarget.FaceColor;
                    Vector3 posA = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[face.nodeA]);
                    Vector3 posB = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[face.nodeB]);
                    Vector3 posC = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[face.nodeC]);
                    Handles.DrawLine(posA, posB, 1f);
                    Handles.DrawLine(posB, posC, 1f);
                    Handles.DrawLine(posC, posA, 1f);
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
            Handles.SphereHandleCap(0, position, Quaternion.identity, EditorTarget.NodeSize, EventType.Repaint);

            if (e.type == EventType.MouseDown && e.button == 0)
            {
                position = EditorTarget.transform.InverseTransformPoint(position);
                EditorTarget.CreateNode(position);
                e.Use();
            }
        }

        private void HandleLinkCreation(Event e)
        {
            for (int i = 0; i < EditorTarget.Nodes.Count; i++)
            {
                Vector3 position = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[i]);
                Handles.color = Color.blue;
                float size = HandleUtility.GetHandleSize(position) * EditorTarget.NodeSize * 2f;
                Handles.SphereHandleCap(0, position, Quaternion.identity, size, EventType.Repaint);

                if (e.type == EventType.MouseDown && e.button == 0)
                {
                    if (HandleUtility.DistanceToCircle(position, size) <= 0)
                    {
                        if (EditorTarget.CreatingLinkNodeIndex == 0)
                        {
                            EditorTarget.CreatingLinkNodeIndex = i;
                        }
                        else if (i != EditorTarget.CreatingLinkNodeIndex)
                        {
                            EditorTarget.CreateLink(EditorTarget.CreatingLinkNodeIndex, i);
                            EditorTarget.CreatingLinkNodeIndex = -1;
                            e.Use();
                        }
                    }
                }
            }

            if (EditorTarget.CreatingLinkNodeIndex >= 0 && EditorTarget.CreatingLinkNodeIndex < EditorTarget.Nodes.Count)
            {
                Vector3 startPos = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[EditorTarget.CreatingLinkNodeIndex]);
                Ray mouseRay = HandleUtility.GUIPointToWorldRay(e.mousePosition);
                Vector3 endPos = mouseRay.origin + mouseRay.direction * 10f;
                Handles.color = Color.red;
                Handles.DrawLine(startPos, endPos, 2f);
            }
        }

        private void HandleFaceCreation(Event e)
        {
            if (EditorTarget == null || EditorTarget.Nodes == null || EditorTarget.CreatingFaceNodeIndices == null)
            {
                return;
            }

            for (int i = 0; i < EditorTarget.Nodes.Count; i++)
            {
                Vector3 position = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[i]);
                Handles.color = Color.blue;
                float size = HandleUtility.GetHandleSize(position) * EditorTarget.NodeSize * 2f;

                Handles.BeginGUI();
                try
                {
                    Handles.SphereHandleCap(0, position, Quaternion.identity, size, EventType.Repaint);
                }
                finally
                {
                    Handles.EndGUI();
                }

                if (e.type == EventType.MouseDown && e.button == 0)
                {
                    if (HandleUtility.DistanceToCircle(position, size) <= 0)
                    {
                        if (!EditorTarget.CreatingFaceNodeIndices.Contains(i))
                        {
                            EditorTarget.CreatingFaceNodeIndices.Add(i);
                            if (EditorTarget.CreatingFaceNodeIndices.Count == 3)
                            {
                                if (EditorTarget.CreatingFaceNodeIndices.All(idx => idx >= 0 && idx < EditorTarget.Nodes.Count))
                                {
                                    EditorTarget.CreateFace(
                                        EditorTarget.CreatingFaceNodeIndices[0],
                                        EditorTarget.CreatingFaceNodeIndices[1],
                                        EditorTarget.CreatingFaceNodeIndices[2]
                                    );
                                }
                                else
                                {
                                    Debug.LogWarning("Invalid node indices selected for face creation.", EditorTarget);
                                }
                                EditorTarget.CreatingFaceNodeIndices.Clear();
                                e.Use();
                            }
                        }
                    }
                }
            }

            if (EditorTarget.CreatingFaceNodeIndices.Count >= 1)
            {
                int firstIndex = EditorTarget.CreatingFaceNodeIndices[0];
                if (firstIndex < 0 || firstIndex >= EditorTarget.Nodes.Count)
                {
                    EditorTarget.CreatingFaceNodeIndices.Clear();
                    return;
                }

                Vector3 startPos = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[firstIndex]);

                Handles.BeginGUI();
                try
                {
                    if (EditorTarget.CreatingFaceNodeIndices.Count == 2)
                    {
                        int secondIndex = EditorTarget.CreatingFaceNodeIndices[1];
                        if (secondIndex >= 0 && secondIndex < EditorTarget.Nodes.Count)
                        {
                            Vector3 midPos = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[secondIndex]);
                            Handles.color = Color.red;
                            Handles.DrawLine(startPos, midPos, 2f);

                            Ray mouseRay = HandleUtility.GUIPointToWorldRay(e.mousePosition);
                            Vector3 endPos = mouseRay.origin + mouseRay.direction * 10f;
                            Handles.DrawLine(midPos, endPos, 2f);
                        }
                        else
                        {
                            EditorTarget.CreatingFaceNodeIndices.RemoveAt(1);
                        }
                    }
                    else
                    {
                        Ray mouseRay = HandleUtility.GUIPointToWorldRay(e.mousePosition);
                        Vector3 endPos = mouseRay.origin + mouseRay.direction * 10f;
                        Handles.color = Color.red;
                        Handles.DrawLine(startPos, endPos, 2f);
                    }
                }
                finally
                {
                    Handles.EndGUI();
                }
            }
        }

        private void HandleNodeSelection(Event e)
        {
            for (int i = 0; i < EditorTarget.Nodes.Count; i++)
            {
                Vector3 pos = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[i]);
                float size = HandleUtility.GetHandleSize(pos) * EditorTarget.NodeSize;
                if (HandleUtility.DistanceToCircle(pos, size) <= 0)
                {
                    Undo.RecordObject(EditorTarget, "Select Node");
                    if (e.control)
                    {
                        if (EditorTarget.SelectedNodeIndices.Contains(i))
                            EditorTarget.SelectedNodeIndices.Remove(i);
                        else
                            EditorTarget.SelectedNodeIndices.Add(i);
                    }
                    else
                    {
                        EditorTarget.SelectedNodeIndices.Clear();
                        EditorTarget.SelectedNodeIndices.Add(i);
                    }
                    EditorTarget.SelectedLinkIndices.Clear();
                    EditorTarget.SelectedFaceIndices.Clear();
                    EditorUtility.SetDirty(EditorTarget);
                    e.Use();
                    break;
                }
            }
        }

        private void HandleLinkSelection(Event e)
        {
            for (int i = 0; i < EditorTarget.Links.Count; i++)
            {
                var link = EditorTarget.Links[i];
                if (link.nodeA >= 0 && link.nodeA < EditorTarget.Nodes.Count && link.nodeB >= 0 && link.nodeB < EditorTarget.Nodes.Count)
                {
                    Vector3 posA = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[link.nodeA]);
                    Vector3 posB = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[link.nodeB]);
                    Vector3 guiPointA = HandleUtility.WorldToGUIPoint(posA);
                    Vector3 guiPointB = HandleUtility.WorldToGUIPoint(posB);
                    if (HandleUtility.DistancePointToLineSegment(e.mousePosition, guiPointA, guiPointB) < 3f)
                    {
                        Undo.RecordObject(EditorTarget, "Select Link");
                        if (e.control)
                        {
                            if (EditorTarget.SelectedLinkIndices.Contains(i))
                                EditorTarget.SelectedLinkIndices.Remove(i);
                            else
                                EditorTarget.SelectedLinkIndices.Add(i);
                        }
                        else
                        {
                            EditorTarget.SelectedLinkIndices.Clear();
                            EditorTarget.SelectedLinkIndices.Add(i);
                        }
                        EditorTarget.SelectedNodeIndices.Clear();
                        EditorTarget.SelectedFaceIndices.Clear();
                        EditorUtility.SetDirty(EditorTarget);
                        e.Use();
                        break;
                    }
                }
            }
        }

        private void HandleFaceSelection(Event e)
        {
            for (int i = 0; i < EditorTarget.Faces.Count; i++)
            {
                var face = EditorTarget.Faces[i];
                if (face.nodeA >= 0 && face.nodeA < EditorTarget.Nodes.Count && face.nodeB >= 0 && face.nodeB < EditorTarget.Nodes.Count && face.nodeC >= 0 && face.nodeC < EditorTarget.Nodes.Count)
                {
                    Vector3 posA = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[face.nodeA]);
                    Vector3 posB = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[face.nodeB]);
                    Vector3 posC = EditorTarget.transform.TransformPoint(EditorTarget.Nodes[face.nodeC]);
                    Vector3 centroid = (posA + posB + posC) / 3f;
                    Vector3 guiCentroid = HandleUtility.WorldToGUIPoint(centroid);
                    float size = HandleUtility.GetHandleSize(centroid) * EditorTarget.NodeSize * 2f;
                    if (HandleUtility.DistanceToCircle(centroid, size) <= 0)
                    {
                        Undo.RecordObject(EditorTarget, "Select Face");
                        if (e.control)
                        {
                            if (EditorTarget.SelectedFaceIndices.Contains(i))
                                EditorTarget.SelectedFaceIndices.Remove(i);
                            else
                                EditorTarget.SelectedFaceIndices.Add(i);
                        }
                        else
                        {
                            EditorTarget.SelectedFaceIndices.Clear();
                            EditorTarget.SelectedFaceIndices.Add(i);
                        }
                        EditorTarget.SelectedNodeIndices.Clear();
                        EditorTarget.SelectedLinkIndices.Clear();
                        EditorUtility.SetDirty(EditorTarget);
                        e.Use();
                        break;
                    }
                }
            }
        }

        private void HandleNodeTransform(Event e)
        {
            EditorTarget.SelectedNodeIndices.RemoveAll(i => i < 0 || i >= EditorTarget.Nodes.Count);
            if (EditorTarget.SelectedNodeIndices.Count == 0) return;

            Vector3 pos = EditorTarget.transform.TransformPoint(EditorTarget.GetSelectionCenter());
            if (!IsDragging)
            {
                HandlePosition = pos;
            }

            EditorGUI.BeginChangeCheck();
            Vector3 newPos = Handles.PositionHandle(pos, Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                IsDragging = true;
                Vector3 delta = EditorTarget.transform.InverseTransformPoint(newPos) - EditorTarget.transform.InverseTransformPoint(pos);
                EditorTarget.TransformSelectedNodes(delta);
                HandlePosition = newPos;
            }
            else if (e.type == EventType.MouseUp)
            {
                IsDragging = false;
            }
        }
    }
#endif
}

[System.Serializable]
public struct Link
{
    public int nodeA;
    public int nodeB;
    public float compliance;
    public float damping;
    public float restLength;
}

[System.Serializable]
public struct Face
{
    public int nodeA;
    public int nodeB;
    public int nodeC;
}

[System.Serializable]
public struct MaterialPreset
{
    public string name;
    public float compliance;
    public float damping;
    public float nodeMass;
    public float maxStretchFactor;
    public float minStretchFactor;
    public float plasticityThreshold;
    public float plasticityRate;
}