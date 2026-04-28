using System.Collections.Generic;
using UnityEngine;

namespace DynamicEngine
{
    [RequireComponent(typeof(Collider))]
    public class StaticBody : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void InitializeOnLoad()
        {
            AllStaticBodies.Clear();
        }
        public Matter matter;

        // Custom Registry
        public static readonly HashSet<StaticBody> AllStaticBodies = new HashSet<StaticBody>();

        private Collider _collider;
        public Collider Collider => _collider != null ? _collider : (_collider = GetComponent<Collider>());

        private void OnEnable()
        {
            AllStaticBodies.Add(this);
        }

        private void OnDisable()
        {
            AllStaticBodies.Remove(this);
        }
    }
}