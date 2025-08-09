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

        #region Properties - Enhanced Collision Settings

        public LayerMask CollisionLayers
        {
            get { return m_collisionLayers; }
            set { m_collisionLayers = value; }
        }

        public bool UseTerrainCollision
        {
            get { return m_useTerrainCollision; }
            set { m_useTerrainCollision = value; }
        }

        public bool UseContinuousCollisionDetection
        {
            get { return m_useContinuousCollisionDetection; }
            set { m_useContinuousCollisionDetection = value; }
        }

        public float MaxCollisionDistance
        {
            get { return m_maxCollisionDistance; }
            set { m_maxCollisionDistance = value; }
        }

        public int MaxCollisionIterations
        {
            get { return m_maxCollisionIterations; }
            set { m_maxCollisionIterations = value; }
        }

        public float SkinWidth
        {
            get { return m_skinWidth; }
            set { m_skinWidth = value; }
        }

        public float CollisionRestitution
        {
            get { return m_collisionRestitution; }
            set { m_collisionRestitution = value; }
        }

        public float CollisionFriction
        {
            get { return m_collisionFriction; }
            set { m_collisionFriction = value; }
        }

        public float CollisionRestThreshold
        {
            get { return m_collisionRestThreshold; }
            set { m_collisionRestThreshold = value; }
        }

        public float ImpactDeformationThreshold
        {
            get { return m_impactDeformationThreshold; }
            set { m_impactDeformationThreshold = value; }
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

            // Enhanced Collision Settings Validation
            m_maxCollisionDistance = Mathf.Clamp(m_maxCollisionDistance, 1f, 100f);
            m_maxCollisionIterations = Mathf.Clamp(m_maxCollisionIterations, 1, 10);
            m_skinWidth = Mathf.Clamp(m_skinWidth, 0.001f, 0.1f);
            m_collisionRestitution = Mathf.Clamp(m_collisionRestitution, 0f, 1f);
            m_collisionFriction = Mathf.Clamp(m_collisionFriction, 0f, 1f);
            m_collisionRestThreshold = Mathf.Clamp(m_collisionRestThreshold, 0.001f, 1f);
            m_impactDeformationThreshold = Mathf.Clamp(m_impactDeformationThreshold, 1f, 50f);

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

        [SerializeField, Range(1, 50), Tooltip("Maximum number of sub-steps per fixed update for high-FPS simulation (e.g., 20 = 1000 FPS at 50 FPS fixed updates)")]
        private int m_baseSubSteps = 20;

        [SerializeField, Range(1, 10), Tooltip("Minimum number of sub-steps per fixed update for stability")]
        private int m_minSubSteps = 2;

        #endregion

        #region Private - Enhanced Collision Settings

        [Header("Enhanced Collision Detection")]
        [SerializeField, Tooltip("Layers that soft body nodes can collide with")]
        private LayerMask m_collisionLayers = -1;

        [SerializeField, Tooltip("Enable specialized terrain collision detection")]
        private bool m_useTerrainCollision = true;

        [SerializeField, Tooltip("Enable continuous collision detection to prevent tunneling")]
        private bool m_useContinuousCollisionDetection = true;

        [SerializeField, Range(1f, 100f), Tooltip("Maximum distance to check for collisions")]
        private float m_maxCollisionDistance = 10f;

        [SerializeField, Range(1, 10), Tooltip("Maximum iterations for collision resolution per frame")]
        private int m_maxCollisionIterations = 3;

        [SerializeField, Range(0.001f, 0.1f), Tooltip("Collision skin width to prevent jitter and intersection")]
        private float m_skinWidth = 0.01f;

        [Header("Collision Response")]
        [SerializeField, Range(0f, 1f), Tooltip("Collision restitution (bounciness) - 0 = no bounce, 1 = perfect bounce")]
        private float m_collisionRestitution = 0.05f;

        [SerializeField, Range(0f, 1f), Tooltip("Collision friction - 0 = no friction, 1 = maximum friction")]
        private float m_collisionFriction = 0.7f;

        [SerializeField, Range(0.001f, 1f), Tooltip("Velocity threshold below which objects are considered at rest")]
        private float m_collisionRestThreshold = 0.01f;

        [SerializeField, Range(1f, 50f), Tooltip("Impact speed threshold for deformation (units per second)")]
        private float m_impactDeformationThreshold = 5f;

        #endregion
    }
}
