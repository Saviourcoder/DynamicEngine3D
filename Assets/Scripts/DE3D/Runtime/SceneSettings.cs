/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using UnityEngine;

namespace DynamicEngine
{
    public class SceneSettings : MonoBehaviour
    {
        #region Private

        [SerializeField]
        private float m_gravity = 9.81f;

        [SerializeField]
        private float m_TimeScale = 1.0f;

        [SerializeField]
        private int m_constraintIterations = 5;

        [SerializeField]
        private int m_workerThreads = 1;

        [SerializeField]
        private int m_substepPower = 2;
        [SerializeField]
        private int m_substepCount = 4;
        [SerializeField]
        private float m_substepSize = 0.02f;

        #endregion

        #region Properties

        public float Gravity
        {
            get { return m_gravity; }
            set { m_gravity = value; }
        }

        public float TimeScale
        {
            get { return m_TimeScale; }
            set { m_TimeScale = value; }
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
        public int SubstepPower
        {
            get { return m_substepPower; }
            set { m_substepPower = value; }
        }
        public int SubstepCount
        {
            get { return m_substepCount; }
            set { m_substepCount = value; }
        }
        public float SubstepSize
        {
            get { return m_substepSize; }
            set { m_substepSize = value; }
        }

        #endregion

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

        #region Unity

        void OnValidate()
        {
            // logic: count = 2 ^ Power
            m_substepCount = (int)Mathf.Pow(2, m_substepPower);
            float effectiveTime = Time.fixedDeltaTime * m_TimeScale;
            if (effectiveTime <= 0) effectiveTime = 0.02f;

            m_substepSize = effectiveTime / m_substepCount;
            m_gravity = Mathf.Max(m_gravity, 0f);
            m_TimeScale = Mathf.Clamp(m_TimeScale, 0.1f, 1.0f);
            m_constraintIterations = Mathf.Max(m_constraintIterations, 1);
        }

        void OnGUI()
        {
            GUIStyle style = new GUIStyle();
            style.normal.textColor = Color.red;
            GUIStyle size = new GUIStyle();
            size.fontSize = 20;
            GUI.Label(new Rect(10, 10, 200, 40), "Info:", size);
            GUI.Label(new Rect(10, 40, 200, 40), "Iterations: " + m_constraintIterations, style);
            GUI.Label(new Rect(10, 60, 200, 40), "Simulation Speed: " + m_TimeScale, style);
            GUI.Label(new Rect(10, 80, 200, 40), "Substep Count: " + m_substepCount, style);
            GUI.Label(new Rect(10, 100, 200, 40), "Gravity: " + m_gravity, style);
        }

        #endregion
    }
}