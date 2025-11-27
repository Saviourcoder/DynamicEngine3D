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
        [SerializeField, Range(0f, 1f)] 
        private float dynamicFriction = 0.4f;
        
        [SerializeField, Range(0f, 1f)] 
        private float staticFriction = 0.5f;
        
        [Header("Additional Physics Properties")]
        [SerializeField, Range(0f, 1f)] 
        private float restitution = 0.02f;
        
        [SerializeField, Range(0f, 1f)] 
        private float damping = 0.95f;

        public float DynamicFriction => dynamicFriction;
        public float StaticFriction => staticFriction;
        public float Restitution => restitution;
        public float Damping => damping;

        public void SetFrictionProperties(float dynamic, float staticFric)
        {
            dynamicFriction = Mathf.Clamp01(dynamic);
            staticFriction = Mathf.Clamp01(staticFric);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetPhysicsProperties(float dynamic, float staticFric, float rest, float damp)
        {
            dynamicFriction = Mathf.Clamp01(dynamic);
            staticFriction = Mathf.Clamp01(staticFric);
            restitution = Mathf.Clamp01(rest);
            damping = Mathf.Clamp01(damp);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        private void OnValidate()
        {
            dynamicFriction = Mathf.Clamp01(dynamicFriction);
            staticFriction = Mathf.Clamp01(staticFriction);
            restitution = Mathf.Clamp01(restitution);
            damping = Mathf.Clamp01(damping);
        }
    }
}