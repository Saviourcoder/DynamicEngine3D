/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */
using System;
using UnityEngine;

namespace DynamicEngine
{
    public enum SnapType
    {
        Point = 0,      // Snap to a world position
        Node = 1,       // Snap to another soft body node
        Edge = 2        // Snap to an edge between two nodes
    }

    [Serializable]
    public class Snap
    {
        public SnapType type;
        public string node;                         // Node set name on this soft body
        public string targetNode;                   // Target node(s) on base body (for Node/Edge types)
        public float minLimit = 0f;                 // Minimum distance constraint
        public float maxLimit = 0f;                 // Maximum distance constraint  
        public float strength = float.PositiveInfinity; // Constraint strength
        public bool master = true;                  // Whether this constraint is master
        public bool show = false;                   // Show visualization
        public Vector3 targetPosition = Vector3.zero; // Target position for Point snap type
        
        public int flags => master ? 1 : 0;
        
        public bool IsValid()
        {
            return !string.IsNullOrEmpty(node);
        }
    }
}
