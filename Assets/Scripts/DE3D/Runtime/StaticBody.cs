/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */

using UnityEngine;

namespace DynamicEngine
{
    [RequireComponent(typeof(Collider))]
    public class StaticBody : MonoBehaviour
    {
        [Tooltip("Physics material properties for this static object")]
        public Matter matter;

        // Cache the collider and ensure we can access it quickly
        private Collider _collider;
        public Collider Collider
        {
            get
            {
                if (_collider == null) _collider = GetComponent<Collider>();
                return _collider;
            }
        }

        private void Reset()
        {
            // Try to find a default matter asset or create one if needed (editor convenience)
        }
    }
}