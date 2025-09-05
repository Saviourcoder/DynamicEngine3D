/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                           
                                                                    By: Elitmers */
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

        #region Properties - Physics Settings

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
            // Physics Settings Validation
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

        #region Private - Physics Settings

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

        [Header("Sub-stepping")]
        public int m_baseSubSteps = 1;
        public int m_minSubSteps = 1;

        [Header("Time Compensation")]
        public float ReferenceSubSteps = 1; 
        public bool EnableTimeCompensation = true;

        #endregion
    }
}
