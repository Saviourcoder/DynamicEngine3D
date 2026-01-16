/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */

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
        [SerializeField, Range(0f, 1f), Tooltip("Bounciness (0 = no bounce, 1 = perfect bounce)")]
        private float restitution = 0.02f;

        [SerializeField, Range(0f, 1f), Tooltip("Collision damping (energy loss on impact)")]
        private float collisionDamping = 0.95f;

        // Public accessors
        public float SlidingFriction => slidingFriction;
        public float StaticFriction => staticFriction;
        public float Restitution => restitution;
        public float CollisionDamping => collisionDamping;

        // Setter methods
        public void SetFrictionProperties(float dynamic, float staticFric)
        {
            slidingFriction = Mathf.Clamp01(dynamic);
            staticFriction = Mathf.Clamp01(staticFric);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetCollisionProperties(float rest, float damp)
        {
            restitution = Mathf.Clamp01(rest);
            collisionDamping = Mathf.Clamp01(damp);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetPhysicsProperties(float dynamic, float staticFric, float rest, float damp)
        {
            slidingFriction = Mathf.Clamp01(dynamic);
            staticFriction = Mathf.Clamp01(staticFric);
            restitution = Mathf.Clamp01(rest);
            collisionDamping = Mathf.Clamp01(damp);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        // Preset materials
        public void ApplyRubberPreset()
        {
            SetPhysicsProperties(0.8f, 0.9f, 0.7f, 0.85f);
        }

        public void ApplyMetalPreset()
        {
            SetPhysicsProperties(0.3f, 0.4f, 0.3f, 0.95f);
        }

        public void ApplyWoodPreset()
        {
            SetPhysicsProperties(0.4f, 0.5f, 0.2f, 0.9f);
        }

        public void ApplyGelPreset()
        {
            SetPhysicsProperties(0.6f, 0.7f, 0.1f, 0.7f);
        }
    }
}