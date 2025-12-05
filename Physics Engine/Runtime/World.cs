/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers */
using UnityEngine;
using System.Collections.Generic;
using System.Collections;
using System.Threading;
using System.Threading.Tasks;
using System;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    public interface IPhysicsComponent
    {
        bool IsEnabled { get; }
        void OnPhysicsStep(float deltaTime);
        void OnPhysicsInterpolate(float alpha);
        Transform Transform { get; }
    }

    [System.Serializable]
    public struct WorldSettings
    {
        [Header("Global Physics")]
        public Vector3 gravity;
        public float pressure;
        public float airDensity;
        public float timeScale;

        [Header("Simulation")]
        public int fixedStepsPerSecond;
        public int maxSubSteps;
        public bool enableInterpolation;
        public bool enableMultithreading;

        [Header("Performance")]
        public int workerThreadCount;
        public bool adaptiveTimeStep;
        public float maxDeltaTime;

        [Header("Collision")]
        public LayerMask physicsLayers;
        public int collisionIterations;
        public float contactTolerance;

        public static WorldSettings Default => new WorldSettings
        {
            gravity = new Vector3(0, -9.81f, 0),
            pressure = 101325f,
            airDensity = 1.225f,
            timeScale = 1f,
            fixedStepsPerSecond = 60,
            maxSubSteps = 5,
            enableInterpolation = true,
            enableMultithreading = true,
            workerThreadCount = System.Environment.ProcessorCount - 1,
            adaptiveTimeStep = true,
            maxDeltaTime = 0.033f,
            physicsLayers = -1,
            collisionIterations = 4,
            contactTolerance = 0.001f
        };
    }

    public sealed class World : MonoBehaviour
    {
        private static World instance;

        public static World Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = FindFirstObjectByType<World>();
                    if (instance == null)
                    {
                        GameObject worldObject = new GameObject("DynamicEngine World");
                        instance = worldObject.AddComponent<World>();
                        DontDestroyOnLoad(worldObject);
                    }
                }
                return instance;
            }
        }

        [Header("World Configuration")]
        [SerializeField] private WorldSettings settings = WorldSettings.Default;
        [SerializeField] private bool debugMode = false;
        [SerializeField] private bool showStatistics = true;

        [Header("Runtime Statistics")]
        [SerializeField] private int activeComponents = 0;
        [SerializeField] private float currentPhysicsTime = 0f;
        [SerializeField] private float averagePhysicsFrameTime = 0f;
        [SerializeField] private int physicsStepsPerFrame = 0;

        private readonly HashSet<IPhysicsComponent> physicsComponents = new HashSet<IPhysicsComponent>();
        private readonly List<IPhysicsComponent> componentsToRemove = new List<IPhysicsComponent>();
        private readonly List<IPhysicsComponent> interpolationComponents = new List<IPhysicsComponent>();

        private float physicsAccumulator = 0f;
        private float fixedDeltaTime;
        private float lastPhysicsTime;
        private float interpolationAlpha = 0f;

        private Thread[] workerThreads;
        private volatile bool threadsRunning = false;
        private readonly object physicsLock = new object();
        private ManualResetEventSlim physicsStepEvent = new ManualResetEventSlim(false);
        private CountdownEvent threadCompletionEvent;

        private float[] frameTimes = new float[60];
        private int frameTimeIndex = 0;

        public static event Action<float> OnPhysicsStep;
        public static event Action<float> OnPhysicsInterpolate;
        public static event Action<IPhysicsComponent> OnComponentEnabled;
        public static event Action<IPhysicsComponent> OnComponentDisabled;
        public static event Action OnWorldDestroyed;

        public WorldSettings Settings => settings;
        public int ActiveComponentCount => activeComponents;
        public float PhysicsTime => currentPhysicsTime;
        public float InterpolationAlpha => interpolationAlpha;
        public bool IsRunning { get; private set; }

        private void Awake()
        {
            if (instance == null)
            {
                instance = this;
                DontDestroyOnLoad(gameObject);
                InitializeWorld();
            }
            else if (instance != this)
            {
                Destroy(gameObject);
                return;
            }
        }

        private void Start()
        {
            StartPhysicsSimulation();
        }

        private void Update()
        {
            if (!IsRunning) return;

            UpdatePhysicsSimulation();
            UpdateInterpolation();
            UpdateStatistics();
        }

        private void FixedUpdate()
        {
            if (!IsRunning) return;

            ProcessPhysicsStep();
        }

        private void OnDestroy()
        {
            StopPhysicsSimulation();
            OnWorldDestroyed?.Invoke();
        }

        private void InitializeWorld()
        {
            fixedDeltaTime = 1f / settings.fixedStepsPerSecond;
            Time.fixedDeltaTime = fixedDeltaTime;

            Physics.gravity = settings.gravity;

            InitializeWorkerThreads();
            SetupCollisionMatrix();

            if (debugMode)
            {
                Debug.Log($"DynamicEngine World initialized - Fixed Delta: {fixedDeltaTime:F4}s, Workers: {settings.workerThreadCount}");
            }
        }

        private void InitializeWorkerThreads()
        {
            if (!settings.enableMultithreading) return;

            int threadCount = Mathf.Max(1, settings.workerThreadCount);
            workerThreads = new Thread[threadCount];
            threadCompletionEvent = new CountdownEvent(threadCount);

            for (int i = 0; i < threadCount; i++)
            {
                int threadIndex = i;
                workerThreads[i] = new Thread(() => WorkerThreadLoop(threadIndex))
                {
                    Name = $"DynamicEngine Worker {threadIndex}",
                    IsBackground = true
                };
            }
        }

        private void SetupCollisionMatrix()
        {
            int layerCount = 32;
            for (int i = 0; i < layerCount; i++)
            {
                for (int j = 0; j < layerCount; j++)
                {
                    bool shouldCollide = (settings.physicsLayers & (1 << i)) != 0 &&
                                       (settings.physicsLayers & (1 << j)) != 0;
                    Physics.IgnoreLayerCollision(i, j, !shouldCollide);
                }
            }
        }

        private void StartPhysicsSimulation()
        {
            IsRunning = true;
            threadsRunning = true;

            if (settings.enableMultithreading && workerThreads != null)
            {
                foreach (var thread in workerThreads)
                {
                    thread.Start();
                }
            }

            if (debugMode)
            {
                Debug.Log("Physics simulation started");
            }
        }

        private void StopPhysicsSimulation()
        {
            IsRunning = false;
            threadsRunning = false;

            physicsStepEvent?.Set();

            if (workerThreads != null)
            {
                foreach (var thread in workerThreads)
                {
                    if (thread.IsAlive)
                    {
                        thread.Join(1000);
                    }
                }
            }

            physicsStepEvent?.Dispose();
            threadCompletionEvent?.Dispose();

            if (debugMode)
            {
                Debug.Log("Physics simulation stopped");
            }
        }

        private void UpdatePhysicsSimulation()
        {
            float deltaTime = Time.unscaledDeltaTime * settings.timeScale;
            deltaTime = Mathf.Min(deltaTime, settings.maxDeltaTime);

            physicsAccumulator += deltaTime;

            int steps = 0;
            while (physicsAccumulator >= fixedDeltaTime && steps < settings.maxSubSteps)
            {
                physicsAccumulator -= fixedDeltaTime;
                steps++;
            }

            physicsStepsPerFrame = steps;
        }

        private void ProcessPhysicsStep()
        {
            float startTime = Time.realtimeSinceStartup;

            lock (physicsLock)
            {
                CleanupDisabledComponents();

                currentPhysicsTime += fixedDeltaTime;

                OnPhysicsStep?.Invoke(fixedDeltaTime);

                if (settings.enableMultithreading)
                {
                    ProcessComponentsMultithreaded(fixedDeltaTime);
                }
                else
                {
                    ProcessComponentsSingleThreaded(fixedDeltaTime);
                }
            }

            float frameTime = Time.realtimeSinceStartup - startTime;
            UpdateFrameTimeAverage(frameTime);
            lastPhysicsTime = frameTime;
        }

        private void ProcessComponentsSingleThreaded(float deltaTime)
        {
            foreach (var component in physicsComponents)
            {
                if (component != null && component.IsEnabled)
                {
                    component.OnPhysicsStep(deltaTime);
                }
            }
        }

        private void ProcessComponentsMultithreaded(float deltaTime)
        {
            if (workerThreads == null || !threadsRunning)
            {
                ProcessComponentsSingleThreaded(deltaTime);
                return;
            }

            threadCompletionEvent.Reset();
            physicsStepEvent.Set();

            threadCompletionEvent.Wait();
            physicsStepEvent.Reset();
        }

        private void WorkerThreadLoop(int threadIndex)
        {
            while (threadsRunning)
            {
                physicsStepEvent.Wait();

                if (!threadsRunning) break;

                ProcessComponentBatch(threadIndex, fixedDeltaTime);

                threadCompletionEvent.Signal();
            }
        }

        private void ProcessComponentBatch(int threadIndex, float deltaTime)
        {
            var components = new IPhysicsComponent[physicsComponents.Count];
            physicsComponents.CopyTo(components);

            int batchSize = Mathf.CeilToInt((float)components.Length / workerThreads.Length);
            int startIndex = threadIndex * batchSize;
            int endIndex = Mathf.Min(startIndex + batchSize, components.Length);

            for (int i = startIndex; i < endIndex; i++)
            {
                var component = components[i];
                if (component != null && component.IsEnabled)
                {
                    component.OnPhysicsStep(deltaTime);
                }
            }
        }

        private void UpdateInterpolation()
        {
            if (!settings.enableInterpolation) return;

            interpolationAlpha = physicsAccumulator / fixedDeltaTime;
            interpolationAlpha = Mathf.Clamp01(interpolationAlpha);

            OnPhysicsInterpolate?.Invoke(interpolationAlpha);

            interpolationComponents.Clear();
            foreach (var component in physicsComponents)
            {
                if (component != null && component.IsEnabled)
                {
                    interpolationComponents.Add(component);
                }
            }

            foreach (var component in interpolationComponents)
            {
                component.OnPhysicsInterpolate(interpolationAlpha);
            }
        }

        private void CleanupDisabledComponents()
        {
            componentsToRemove.Clear();

            foreach (var component in physicsComponents)
            {
                if (component == null || !component.IsEnabled)
                {
                    componentsToRemove.Add(component);
                }
            }

            foreach (var component in componentsToRemove)
            {
                physicsComponents.Remove(component);
                activeComponents = physicsComponents.Count;

                if (component != null)
                {
                    OnComponentDisabled?.Invoke(component);
                }
            }

            if (activeComponents == 0 && !Application.isPlaying)
            {
                DestroyWorld();
            }
        }

        private void UpdateStatistics()
        {
            if (!showStatistics) return;

            activeComponents = physicsComponents.Count;
        }

        private void UpdateFrameTimeAverage(float frameTime)
        {
            frameTimes[frameTimeIndex] = frameTime;
            frameTimeIndex = (frameTimeIndex + 1) % frameTimes.Length;

            float sum = 0f;
            for (int i = 0; i < frameTimes.Length; i++)
            {
                sum += frameTimes[i];
            }
            averagePhysicsFrameTime = sum / frameTimes.Length;
        }

        public void ComponentEnabled(IPhysicsComponent component)
        {
            if (component == null) return;

            lock (physicsLock)
            {
                physicsComponents.Add(component);
                activeComponents = physicsComponents.Count;
            }

            OnComponentEnabled?.Invoke(component);

            if (debugMode)
            {
                Debug.Log($"Physics component enabled: {component.GetType().Name}");
            }
        }

        public void ComponentDisabled(IPhysicsComponent component)
        {
            if (component == null) return;

            lock (physicsLock)
            {
                physicsComponents.Remove(component);
                activeComponents = physicsComponents.Count;
            }

            OnComponentDisabled?.Invoke(component);

            if (debugMode)
            {
                Debug.Log($"Physics component disabled: {component.GetType().Name}");
            }
        }

        public void UpdateSettings(WorldSettings newSettings)
        {
            settings = newSettings;
            fixedDeltaTime = 1f / settings.fixedStepsPerSecond;
            Time.fixedDeltaTime = fixedDeltaTime;
            Physics.gravity = settings.gravity;

            SetupCollisionMatrix();

            if (debugMode)
            {
                Debug.Log("World settings updated");
            }
        }

        public void PauseSimulation()
        {
            IsRunning = false;
            if (debugMode) Debug.Log("Simulation paused");
        }

        public void ResumeSimulation()
        {
            IsRunning = true;
            if (debugMode) Debug.Log("Simulation resumed");
        }

        public void ResetWorld()
        {
            lock (physicsLock)
            {
                physicsComponents.Clear();
                activeComponents = 0;
                currentPhysicsTime = 0f;
                physicsAccumulator = 0f;
            }

            if (debugMode) Debug.Log("World reset");
        }

        private void DestroyWorld()
        {
            if (instance == this)
            {
                instance = null;
                StopPhysicsSimulation();

                if (Application.isPlaying)
                {
                    Destroy(gameObject);
                }
                else
                {
                    DestroyImmediate(gameObject);
                }
            }
        }

        public void LogWorldStatistics()
        {
            Debug.Log($"=== DynamicEngine World Statistics ===");
            Debug.Log($"Active Components: {activeComponents}");
            Debug.Log($"Physics Time: {currentPhysicsTime:F3}s");
            Debug.Log($"Average Frame Time: {averagePhysicsFrameTime * 1000:F2}ms");
            Debug.Log($"Physics Steps/Frame: {physicsStepsPerFrame}");
            Debug.Log($"Interpolation Alpha: {interpolationAlpha:F3}");
            Debug.Log($"Multithreading: {(settings.enableMultithreading ? "Enabled" : "Disabled")}");
            Debug.Log($"Worker Threads: {(workerThreads?.Length ?? 0)}");
        }

        private void OnDrawGizmos()
        {
            if (!debugMode || !Application.isPlaying) return;

#if UNITY_EDITOR
            UnityEditor.Handles.BeginGUI();

            GUILayout.BeginArea(new Rect(10, 10, 300, 200));
            GUILayout.Label("DynamicEngine World", EditorStyles.boldLabel);
            GUILayout.Label($"Components: {activeComponents}");
            GUILayout.Label($"Physics Time: {currentPhysicsTime:F2}s");
            GUILayout.Label($"Frame Time: {averagePhysicsFrameTime * 1000:F1}ms");
            GUILayout.Label($"Steps/Frame: {physicsStepsPerFrame}");
            GUILayout.Label($"Alpha: {interpolationAlpha:F2}");
            GUILayout.EndArea();

            UnityEditor.Handles.EndGUI();
#endif
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        public static void RegisterComponent(IPhysicsComponent component)
        {
            Instance.ComponentEnabled(component);
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        public static void UnregisterComponent(IPhysicsComponent component)
        {
            if (instance != null)
            {
                instance.ComponentDisabled(component);
            }
        }
    }

    public static class WorldExtensions
    {
        public static void RegisterWithWorld(this IPhysicsComponent component)
        {
            World.RegisterComponent(component);
        }

        public static void UnregisterFromWorld(this IPhysicsComponent component)
        {
            World.UnregisterComponent(component);
        }
    }
}
