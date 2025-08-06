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
    public class SceneSettings : MonoBehaviour
    {
        #region Singleton

        private static SceneSettings instance;
        public static SceneSettings Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = FindFirstObjectByType<SceneSettings>();
                    if (instance == null)
                    {
                        GameObject settingsObj = new GameObject("SceneSettings");
                        instance = settingsObj.AddComponent<SceneSettings>();
                        Debug.LogWarning("No SceneSettings found in scene. Created new instance.", settingsObj);
                    }
                }
                return instance;
            }
        }

        void Awake()
        {
            if (instance != null && instance != this)
            {
                Debug.LogWarning("Multiple SceneSettings instances detected. Destroying duplicate.", this);
                Destroy(gameObject);
                return;
            }
            instance = this;
            
            // Ensure the GameObject is a root object before applying DontDestroyOnLoad
            if (transform.parent != null)
            {
                Debug.LogWarning("SceneSettings must be a root GameObject to persist across scenes. Moving to root.", this);
                transform.SetParent(null);
            }
            DontDestroyOnLoad(gameObject);
        }

        #endregion

        #region Properties

        public float Gravity
        {
            get { return m_gravity; }
            set { m_gravity = value; }
        }

        public float SimulationTimeScale
        {
            get { return m_simulationTimeScale; }
            set { m_simulationTimeScale = value; }
        }

        public int ConstraintIterations
        {
            get { return m_constraintIterations; }
            set { m_constraintIterations = value; }
        }

        public int WorkerThreads
        {
            get { return m_workerThreads; }
            set { m_workerThreads = value; }
        }

        public float CollisionDamping
        {
            get { return m_collisionDamping; }
            set { m_collisionDamping = value; }
        }

        public int BaseSubSteps
        {
            get { return m_baseSubSteps; }
            set { m_baseSubSteps = value; }
        }

        public int MinSubSteps
        {
            get { return m_minSubSteps; }
            set { m_minSubSteps = value; }
        }

        #endregion

        #region Unity

        void OnValidate()
        {
            m_gravity = Mathf.Max(m_gravity, 0f);
            m_simulationTimeScale = Mathf.Clamp(m_simulationTimeScale, 0.1f, 1.0f);
            m_constraintIterations = Mathf.Clamp(m_constraintIterations, 1, 50);
            m_workerThreads = Mathf.Clamp(m_workerThreads, 1, 8);
            m_collisionDamping = Mathf.Clamp(m_collisionDamping, 0f, 1f);
            m_baseSubSteps = Mathf.Clamp(m_baseSubSteps, 1, 50);
            m_minSubSteps = Mathf.Clamp(m_minSubSteps, 1, 10);

            if (Application.isPlaying)
            {
                Time.timeScale = m_simulationTimeScale;
            }
        }

        #endregion

        #region Private

        [Header("Physics Settings")]
        [SerializeField, Range(0f, 20f), Tooltip("Magnitude of gravitational acceleration (m/s²)")]
        private float m_gravity = 9.81f;

        [SerializeField, Range(0.1f, 1.0f), Tooltip("Simulation time scale (0.1 = 10% speed, 1.0 = normal speed)")]
        private float m_simulationTimeScale = 1.0f;

        [SerializeField, Range(1, 50), Tooltip("Number of iterations for constraint solving")]
        private int m_constraintIterations = 5;

        [SerializeField, Range(1, 8), Tooltip("Number of worker threads for parallel processing")]
        private int m_workerThreads = 1;

        [SerializeField, Range(0f, 1f), Tooltip("Damping factor for collision response")]
        private float m_collisionDamping = 0.5f;

        [SerializeField, Range(1, 50), Tooltip("Maximum number of sub-steps per fixed update for high-FPS simulation (e.g., 20 = 1000 FPS at 50 FPS fixed updates)")]
        private int m_baseSubSteps = 20;

        [SerializeField, Range(1, 10), Tooltip("Minimum number of sub-steps per fixed update for stability")]
        private int m_minSubSteps = 2;

        #endregion
    }
}
