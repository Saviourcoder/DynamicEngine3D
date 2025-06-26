/* DynamicEngine3D - Soft Body Simulation
   *---*---*
  / \ / \ / \
 *---*---*---*
 | DynamicEngine3D |  By: Elitmers
 *---*---*---*
  \ / \ / \ /
   *---*---*
*/
using UnityEngine;

namespace DynamicEngine
{
    public enum MaterialType { Metal, Plastic, Rubber, Custom }

    [System.Serializable]
    public class MaterialProperties
    {
        [SerializeField] public float nodeMass;
        [SerializeField] public float defaultCompliance;
        [SerializeField] public float defaultDamping;
        [SerializeField] public float deformationScale;
        [SerializeField] public float maxDeformation;
        [SerializeField] public float plasticityThreshold;
        [SerializeField] public float plasticityRate;

        public MaterialProperties(
            float nodeMass,
            float defaultCompliance,
            float defaultDamping,
            float deformationScale,
            float maxDeformation,
            float plasticityThreshold,
            float plasticityRate)
        {
            this.nodeMass = nodeMass;
            this.defaultCompliance = defaultCompliance;
            this.defaultDamping = defaultDamping;
            this.deformationScale = deformationScale;
            this.maxDeformation = maxDeformation;
            this.plasticityThreshold = plasticityThreshold;
            this.plasticityRate = plasticityRate;
        }

        public static MaterialProperties GetDefault(MaterialType type)
        {
            switch (type)
            {
                case MaterialType.Metal:
                    return new MaterialProperties(
                        nodeMass: 1f,
                        defaultCompliance: 1e-6f,
                        defaultDamping: 0.1f,
                        deformationScale: 0.05f,
                        maxDeformation: 0.3f,
                        plasticityThreshold: 0.05f,
                        plasticityRate: 0.1f
                    );
                case MaterialType.Plastic:
                    return new MaterialProperties(
                        nodeMass: 0.8f,
                        defaultCompliance: 1e-4f,
                        defaultDamping: 0.2f,
                        deformationScale: 0.1f,
                        maxDeformation: 0.5f,
                        plasticityThreshold: 0.03f,
                        plasticityRate: 0.2f
                    );
                case MaterialType.Rubber:
                    return new MaterialProperties(
                        nodeMass: 0.5f,
                        defaultCompliance: 1e-3f,
                        defaultDamping: 0.3f,
                        deformationScale: 0.2f,
                        maxDeformation: 1f,
                        plasticityThreshold: 0.1f,
                        plasticityRate: 0.05f
                    );
                case MaterialType.Custom:
                default:
                    return new MaterialProperties(
                        nodeMass: 1f,
                        defaultCompliance: 1e-5f,
                        defaultDamping: 0.1f,
                        deformationScale: 0.1f,
                        maxDeformation: 0.5f,
                        plasticityThreshold: 0.05f,
                        plasticityRate: 0.1f
                    );
            }
        }
    }

    [System.Serializable]
    public class Beam
    {
        public int nodeA;
        public int nodeB;
        public float compliance;
        public float damping;
        public float restLength;
        public float originalRestLength;
        public float lagrangeMultiplier;

        public Beam(int nodeA, int nodeB, float compliance, float damping, float restLength)
        {
            this.nodeA = nodeA;
            this.nodeB = nodeB;
            this.compliance = compliance;
            this.damping = damping;
            this.restLength = restLength;
            this.originalRestLength = restLength;
            this.lagrangeMultiplier = 0f;
        }
    }
}
