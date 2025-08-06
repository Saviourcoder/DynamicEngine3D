/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */

using UnityEngine;
using System.Collections.Generic;

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
        
        [Header("Truss Configuration")]
        [SerializeField] public TrussAsset truss;
        [SerializeField, Range(0.1f, 5000f)] private float totalMass = 900f;
        [SerializeField, Range(0.01f, 0.5f)] private float nodeRadius = 0.05f;
        [SerializeField, Range(0.01f, 5f)] private float influenceRadius = 1f;
        [SerializeField, Range(0.5f, 5f)] private float beamConnectionDistance = 1.5f;

        [SerializeField] public bool showNodes = true;
        [SerializeField] public bool showLinks = true;
        [SerializeField] public bool showLinkForces = false;
        [SerializeField] public bool showInfluenceRadius = false;
        [SerializeField, Range(0.05f, 0.5f)] public float nodeDisplaySize = 0.1f;
        [SerializeField, Range(0.001f, 0.1f)] public float forceVisualizationScale = 0.01f;
         private Color nodeColor = Color.green;
         private Color pinnedNodeColor = Color.red;
         private Color linkColor = Color.blue;
         private Color influenceRadiusColor = new Color(1f, 1f, 0f, 0.3f);
        
        // Core system components
        private MeshFilter meshFilter;
        private Mesh mesh;
        public Solver solver;
        
        // Properties for external access
        public float NodeRadius => nodeRadius;
        public float InfluenceRadius => influenceRadius;
        public float BeamConnectionDistance => beamConnectionDistance;
        // Mass calculation properties
        public float TotalMass 
        { 
            get => totalMass; 
            set 
            { 
                totalMass = Mathf.Max(0.1f, value);
                if (solver?.materialProps != null)
                {
                    DistributeMassEvenly();
                }
            } 
        }
        
        public float CalculatedTotalMass 
        { 
            get 
            {
                if (solver?.materialProps == null || solver?.nodeManager?.Nodes == null) 
                    return 0f;
                return solver.materialProps.nodeMass * solver.nodeManager.Nodes.Count;
            }
        }

        // Unity lifecycle methods
        private void Awake()
        {
            InitializeInPlayMode();
            if  (truss == null)
            truss = GetComponent<Designer>()?.SoftBody?.truss;

            if  (truss == null)
                Debug.LogError("No TrussAsset assigned and none found on Designer!", this);
        }
    
        private void OnValidate()
        {
            
            ApplyMaterialProperties();
            ValidateParameters();
            
            // Update mass distribution when total mass changes
            if (solver?.materialProps != null && solver?.nodeManager?.Nodes != null)
            {
                DistributeMassEvenly();
            }

            // CRITICAL: Ensure TrussAsset reference is maintained even if Designer component is removed
            if (truss == null)
            {
                Debug.Log("SoftBody.OnValidate() - TrussAsset is null, searching for Designer component", this);
                var designer = GetComponent<Designer>();
                if (designer?.SoftBody?.truss != null)
                {
                    truss = designer.SoftBody.truss;
                    Debug.Log($"SoftBody.OnValidate() - Restored TrussAsset reference from Designer: {truss.name}", this);
                }
                else
                {
                    Debug.LogWarning("SoftBody.OnValidate() - No Designer component or TrussAsset found", this);
                }
            }

        }

        void FixedUpdate()
        {
            if (solver != null) 
            { 
                solver.Solve(); 
                solver.DeformMesh(transform); 
            }
        }

        void Update()
        {
            // Draw runtime visualization
            if (solver != null)
            {
                DrawRuntimeVisualization();
            }
        }

        void OnDestroy()
        {
            solver?.nodeManager.Clear();
            CleanupNodes(); 
        }

        private void OnDisable()
        {
           CleanupNodes();
        }

        // Core initialization methods
        private void InitializeInPlayMode()
        {
            if (Application.isPlaying) 
                InitializeCore();
        }

        public void InitializeInEditMode()
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
            if (!SetupMesh()) 
                return false;

            // Initialize core solver with default material properties
            solver = new Solver(nodeRadius, influenceRadius, MaterialProps.GetDefault(MaterialType.Metal), mesh, mesh.vertices, transform);
        
            if  (truss != null) 
                ApplyTruss();
            else 
                solver.GenerateCubeTest(transform);

            return true;
        }

        private bool SetupMesh()
        {
            meshFilter = GetComponent<MeshFilter>();
            if (!meshFilter || meshFilter.sharedMesh == null) 
                return false;

            mesh = Application.isPlaying
                ? (meshFilter.mesh = Instantiate(meshFilter.sharedMesh))
                : meshFilter.sharedMesh;
            return true;
        }

        // Public interface methods
        public TrussAsset GetTrussAsset()
        {
            return truss;
        }

        public void ApplyTruss()
        {
            if  (truss == null) 
            { 
                Debug.LogWarning("No TrussAsset assigned.", this); 
                return; 
            }

            var positions = truss.NodePositions;
            var beams = truss.GetBeams();

            if (positions == null || positions.Length < 2) 
            { 
                Debug.LogWarning("Invalid node positions.", this); 
                return; 
            }

            MaterialProps materialProps = truss.MaterialProperties ?? MaterialProps.GetDefault(MaterialType.Metal);
            
            // Update solver material properties
            if (solver != null)
            {
                solver.SetMaterialProperties(materialProps);
                solver.materialProps.nodeMass = truss.NodeMass;
                
                // Apply stretch factors using reflection
                try
                {
                    var maxField = typeof(Solver).GetField("maxStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                    var minField = typeof(Solver).GetField("minStretchFactor", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                    if (maxField != null && minField != null)
                    {
                        maxField.SetValue(solver, truss.MaxStretchFactor);
                        minField.SetValue(solver, truss.MinStretchFactor);
                    }
                }
                catch (System.Exception e)
                {
                    Debug.LogError($"Failed to apply stretch factors: {e.Message}", this);
                }
            }

            solver.GenerateNodesAndBeams(positions, beamsArray: beams, parent: transform);
            
            // Apply pinned nodes from TrussAsset
            if (truss.PinnedNodes != null)
            {
                try
                {
                    var pinMethod = typeof(Solver).GetMethod("PinNode", System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
                    if (pinMethod != null)
                    {
                        foreach (int nodeIndex in truss.PinnedNodes)
                        {
                            if (nodeIndex >= 0 && nodeIndex < positions.Length)
                            {
                                pinMethod.Invoke(solver, new object[] { nodeIndex });
                            }
                        }
                    }
                }
                catch (System.Exception e)
                {
                    Debug.LogError($"Failed to apply pinned nodes: {e.Message}", this);
                }
            }
            
            // Distribute mass evenly after nodes are created
            DistributeMassEvenly();
            DistributeMassEvenly();
        }

        public void ApplyTrussAsset(TrussAsset asset)
        {  
            if (asset == null) 
            { 
                Debug.LogWarning("Cannot apply null TrussAsset", this); 
                return; 
            }
            
            // Don't apply truss during application quit or when GameObject is being destroyed
            if (s_isApplicationQuitting || gameObject == null)
                return;
                
            truss = asset;
            ApplyTruss();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif 
        }

        // Visualization and debugging methods
        private void DrawRuntimeVisualization()
        {
            if (solver?.nodeManager?.Nodes == null) 
                return;

            if (showNodes)
                DrawNodes();
            
            if (showLinks)
                DrawLinks();
                
            if (showInfluenceRadius)
                DrawInfluenceRadius();
        }

        private void DrawNodes()
        {
            for (int i = 0; i < solver.nodeManager.Nodes.Count; i++)
            {
                if (solver.nodeManager.Nodes[i] == null) 
                    continue;

                Vector3 position = solver.nodeManager.Nodes[i].position;
                bool isPinned = i < solver.nodeManager.IsPinned.Count && solver.nodeManager.IsPinned[i];
                Color color = isPinned ? pinnedNodeColor : nodeColor;

                // Draw node as wireframe sphere
                DrawWireSphere(position, nodeDisplaySize, color);
            }
        }

        private void DrawLinks()
        {
            if (solver.beams == null) 
                return;

            for (int i = 0; i < solver.beams.Count; i++)
            {
                var beam = solver.beams[i];
                
                if (beam.nodeA >= solver.nodeManager.Nodes.Count || beam.nodeB >= solver.nodeManager.Nodes.Count ||
                    solver.nodeManager.Nodes[beam.nodeA] == null || solver.nodeManager.Nodes[beam.nodeB] == null)
                    continue;

                Vector3 posA = solver.nodeManager.Nodes[beam.nodeA].position;
                Vector3 posB = solver.nodeManager.Nodes[beam.nodeB].position;

                // Draw force visualization if enabled
                if (showLinkForces && beam.lagrangeMultiplier != 0)
                {
                    Vector3 midPoint = (posA + posB) * 0.5f;
                    Vector3 direction = (posB - posA).normalized;
                    float forceMagnitude = beam.lagrangeMultiplier * forceVisualizationScale;
                    
                    if (forceMagnitude > 0)
                    {
                        // Tension (pulling apart)
                        Debug.DrawRay(midPoint, direction * forceMagnitude, Color.red, Time.fixedDeltaTime);
                        Debug.DrawRay(midPoint, -direction * forceMagnitude, Color.red, Time.fixedDeltaTime);
                    }
                    else
                    {
                        // Compression (pushing together)
                        Debug.DrawRay(midPoint, direction * Mathf.Abs(forceMagnitude), Color.blue, Time.fixedDeltaTime);
                        Debug.DrawRay(midPoint, -direction * Mathf.Abs(forceMagnitude), Color.blue, Time.fixedDeltaTime);
                    }
                }
            }
        }

        private void DrawInfluenceRadius()
        {
            if (solver?.meshDeformer == null) 
                return;
            
            float influenceRadius = solver.meshDeformer.InfluenceRadius;
            
            for (int i = 0; i < solver.nodeManager.Nodes.Count; i++)
            {
                if (solver.nodeManager.Nodes[i] == null) 
                    continue;

                Vector3 position = solver.nodeManager.Nodes[i].position;
                
                // Draw influence radius as wireframe sphere with transparent color
                DrawWireSphere(position, influenceRadius, influenceRadiusColor);
            }
        }

        private void DrawWireSphere(Vector3 center, float radius, Color color)
        {
            // Draw three circles to form a wireframe sphere
            DrawCircle(center, radius, Vector3.up, color);
            DrawCircle(center, radius, Vector3.right, color);
            DrawCircle(center, radius, Vector3.forward, color);
        }

        private void DrawCircle(Vector3 center, float radius, Vector3 normal, Color color)
        {
            Vector3 forward = Vector3.Cross(normal, Vector3.up);
            if (forward.magnitude < 0.1f)
                forward = Vector3.Cross(normal, Vector3.right);
            
            Vector3 right = Vector3.Cross(normal, forward).normalized;
            forward = forward.normalized;

            int segments = 16;
            Vector3 prevPos = center + right * radius;
            
            for (int i = 1; i <= segments; i++)
            {
                float angle = (float)i / segments * Mathf.PI * 2f;
                Vector3 newPos = center + (right * Mathf.Cos(angle) + forward * Mathf.Sin(angle)) * radius;
                Debug.DrawLine(prevPos, newPos, color, Time.fixedDeltaTime);
                prevPos = newPos;
            }
        }

        // System utility methods
        private void ApplyMaterialProperties()
        {
            // Reserved for future material property applications
        }

        private void ValidateParameters()
        {
            if (nodeRadius < 0.01f) 
                nodeRadius = 0.01f;
            if (beamConnectionDistance < nodeRadius * 2f)
                beamConnectionDistance = nodeRadius * 2f;
        }
        
        private void DistributeMassEvenly()
        {
            if (solver?.materialProps == null || solver?.nodeManager?.Nodes == null || solver.nodeManager.Nodes.Count == 0)
                return;
                
            float massPerNode = totalMass / solver.nodeManager.Nodes.Count;
            solver.materialProps.nodeMass = massPerNode;
            
            // Update the Designer's nodeMass to match our distribution
            var nodeLinkEditor = GetComponent<Designer>();
            if (nodeLinkEditor != null)
            {
                nodeLinkEditor.nodeMass = massPerNode;
            }
            
            // Update any existing Rigidbody components on nodes
            foreach (var node in solver.nodeManager.Nodes)
            {
                if (node != null)
                {
                    var rigidbody = node.GetComponent<Rigidbody>();
                    if (rigidbody != null)
                    {
                        rigidbody.mass = massPerNode;
                    }
                }
            }
        }

        private void CleanupNodes()
        {
            if (transform == null) 
                return;

#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                // Editor: safe delayed destruction
                EditorApplication.delayCall += () =>
                {
                    if (this == null || transform == null) 
                        return;
                    for (int i = transform.childCount - 1; i >= 0; i--)
                    {
                        Transform child = transform.GetChild(i);
                        if (child.name.StartsWith("Node_"))
                            Undo.DestroyObjectImmediate(child.gameObject);
                    }
                };
            }
            else
#endif
            {
                // Runtime: standard Destroy
                for (int i = transform.childCount - 1; i >= 0; i--)
                {
                    Transform child = transform.GetChild(i);
                    if (child.name.StartsWith("Node_"))
                        Destroy(child.gameObject);
                }
            }
        }
    }
}
