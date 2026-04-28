/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */

using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    [CreateAssetMenu(fileName = "Matter", menuName = "DynamicEngine/Matter")]
    public class Matter : ScriptableObject
    {
        [Header("Friction Properties")]
        [SerializeField, Range(0f, 2f), Tooltip("Sliding friction coefficient (kinetic friction)")]
        private float slidingFriction = 0.4f;

        [SerializeField, Range(0f, 1f), Tooltip("Static friction coefficient (resistance to initial movement)")]
        private float staticFriction = 0.5f;

        [Header("Collision Response")]
        [SerializeField, Range(0f, 1f), Tooltip("Bounciness - how much velocity is retained after collision")]
        private float restitution = 0.02f;

        [SerializeField, Range(0f, 1f), Tooltip("Collision damping - reduces velocity during collisions")]
        private float collisionDamping = 0.95f;

        [Header("Collision Parameters")]
        [Range(0f, 0.001f)]
        [Tooltip("Collision compliance (0 = infinitely stiff, higher = softer). Recommended: 0.0 for rigid, 0.0001 for soft.")]
        public float CollisionCompliance = 0.0f;

        [Range(0f, 0.0001f)]
        [Tooltip("Friction compliance. Recommended: 0.00001")]
        public float FrictionCompliance = 0.00001f;

        public float SlidingFriction => slidingFriction;
        public float StaticFriction => staticFriction;
        public float Restitution => restitution;
        public float CollisionDamping => collisionDamping;

        public void SetFrictionProperties(float dynamic, float staticFric)
        {
            // Clamp sliding friction to [0, 2] to match the Range attribute
            slidingFriction = Mathf.Clamp(dynamic, 0f, 2f);
            staticFriction = Mathf.Clamp01(staticFric);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetCollisionProperties(float rest, float damping)
        {
            restitution = Mathf.Clamp01(rest);
            collisionDamping = Mathf.Clamp01(damping);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }
    }
}