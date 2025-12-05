/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
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
        [SerializeField]
        private float dynamicFriction = 0.4f;

        [SerializeField]
        private float staticFriction = 0.5f;
        public float DynamicFriction => dynamicFriction;
        public float StaticFriction => staticFriction;


        public void SetFrictionProperties(float dynamic, float staticFric)
        {
            dynamicFriction = dynamic;
            staticFriction = staticFric;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetPhysicsProperties(float dynamic, float staticFric)
        {
            dynamicFriction = dynamic;
            staticFriction = staticFric;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }
    }
}