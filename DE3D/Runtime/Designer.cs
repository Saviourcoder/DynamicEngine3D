/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers */
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
        public void LoadDefaultsFromTruss(Truss truss)
        {
            if (truss == null) return;

            var beams = truss.GetTrussBeams();
            if (beams != null && beams.Count > 0)
            {
                // Use the first beam as reference for defaults
                var firstBeam = beams[0];
                defaultBeamCompliance = firstBeam.compliance;
                defaultBeamDamping = firstBeam.damping;
                defaultPlasticityThreshold = firstBeam.plasticityThreshold;
                defaultPlasticityRate = firstBeam.plasticityRate;
                defaultMaxDeformation = firstBeam.maxDeformation;
            }

            // Also load node mass if available
            if (truss.NodeMasses != null && truss.NodeMasses.Count > 0)
            {
                nodeCreationMass = truss.NodeMasses[0];
            }
        }
    }
}