using UnityEngine;

namespace DynamicEngine
{
    [System.Serializable]
    public struct Link
    {
        public int nodeA;
        public int nodeB;
        public float springForce;
        public float damping;
        public float restLength;
        public float deformationScale;
        public float maxDeformation;
        public float plasticityThreshold;
        public float plasticityRate;
    }

}
