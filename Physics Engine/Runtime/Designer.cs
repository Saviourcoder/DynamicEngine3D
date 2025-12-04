using UnityEngine;
using System.Collections.Generic;

namespace DynamicEngine
{
    [RequireComponent(typeof(SoftBody))]
    [ExecuteInEditMode]
    public class SoftBodyDesigner : MonoBehaviour
    {
        #region Enums
        public enum DesignerMode
        {
            Node,
            Beam,
            Face,
            MeshConvert,
            Optimization
        }

        public enum SelectionMode
        {
            Single,
            Multi,
            Box,
            Sphere
        }

        public enum BeamPreset
        {
            Custom,
            Metal,
            Rubber
        }
        #endregion

        #region State Data
        // Core State
        public DesignerMode currentMode = DesignerMode.Node;
        public SelectionMode selectionMode = SelectionMode.Single;
        public bool autoApplyChanges = true;

        // Selections (Serialized to keep selection when clicking off object)
        [HideInInspector] public List<int> selectedNodes = new List<int>();
        [HideInInspector] public List<int> selectedBeams = new List<int>();
        [HideInInspector] public List<int> selectedFaces = new List<int>();

        // Node Settings
        public float nodeCreationMass = 0.5f;
        public bool showTransformHandles = false;

        // Beam Settings
public float defaultBeamCompliance = 0.01f;
public float defaultBeamDamping = 0.3f;
public float defaultPlasticityThreshold = 0.02f;
public float defaultPlasticityRate = 0.5f;
public float defaultDeformationScale = 1.0f;  // ← ADD THIS!
public float defaultMaxDeformation = 0.8f;
        public BeamPreset selectedBeamPreset = BeamPreset.Custom;

        // Visualization Settings
        public bool showNodeIndices = true;
        public bool showBeamIndices = false;
        public bool showFaceIndices = false;
        public float nodeDisplaySize = 0.15f;
        public float beamLineThickness = 2.0f;
        public Color nodeColor = Color.green;
        public Color selectedNodeColor = Color.yellow;
        public Color pinnedNodeColor = Color.red;
        public Color beamColor = Color.cyan;
        public Color selectedBeamColor = Color.magenta;
        public Color faceColor = new Color(0.2f, 0.8f, 1.0f, 0.3f);
        public Color selectedFaceColor = new Color(1.0f, 0.8f, 0.2f, 0.5f);

        #endregion

        // Reference to the SoftBody on this object
        public SoftBody TargetSoftBody => GetComponent<SoftBody>();

        private void OnEnable()
        {
            // Ensure we have a SoftBody
            if (GetComponent<SoftBody>() == null)
            {
                gameObject.AddComponent<SoftBody>();
            }
        }
    }
}