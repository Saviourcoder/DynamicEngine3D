/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  CarController — ported from Tx physics engine            ║
   ╚═══════════════════════════════════════════════════════════╝ */
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using UnityEngine.InputSystem;
using DynamicEngine;

[RequireComponent(typeof(SoftBody))]
public class CarController : MonoBehaviour
{
    // ── Named constants ────────────────────────────────────────────────────
    private static readonly float TwoPI = Mathf.PI * 2f;
    private static readonly float HalfPI = Mathf.PI / 2f;
    private static readonly float SixthPI = Mathf.PI / 6f;
    private const float Sqrt2 = 1.4142135f;

    // ── Cameras ────────────────────────────────────────────────────────────
    [Header("Cameras")]
    public Camera targetCamera;
    public Camera interiorCamera;

    private float smoothCollisionDistance;
    private float speedTilt = 0f;
    private LayerMask cameraCollisionMask;

    // ── Throttle & Gear ────────────────────────────────────────────────────
    [Header("Throttle & Gear Control")]
    [HideInInspector] float throttle;
    [HideInInspector] public int gear;
    [HideInInspector] float engineForce;

    private float throttleTime;
    private float oldThrottle;
    private float gearTime;

    // ── Braking ────────────────────────────────────────────────────────────
    [Header("Braking System")]
    public float maxBrakeTorque = 12000f;
    public float brakeResponseSpeed = 8f;

    private float currentBrakeTorque = 0f;

    // ── Engine ─────────────────────────────────────────────────────────────
    [Header("Engine")]

    public float idleRPM = 800f;
    public float maxRPM = 6000f;
    public float accelSmoothing = 5f;
    public float decelSmoothing = 10f;
    public float idleRPMVariation = 10f;
    public float idleRPMSmoothing = 10f;
    public float maxHorsepower = 64f;
    public float peakTorqueNm = 120f;
    public float peakTorqueRPM = 3500f;
    [HideInInspector] public float forwardSpeed;
    [HideInInspector] public float engineRPM;
    [HideInInspector] public float engineTargetRPM;
    public AnimationCurve torqueCurve;

    [Tooltip("How fast the constraint's target rate ramps toward the desired wheel RPM. " +
             "Lower = smoother / less wheelspin; higher = more responsive.")]
    public float motorRampSpeed = 6f;

    // Smoothed angular-rate (rad/s) currently commanded to the wheel constraints.
    private float _currentMotorRate = 0f;

    private readonly bool engineRunning = true;

    // ── Shifting ───────────────────────────────────────────────────────────
    private bool isShifting = false;
    private float shiftTimer = 0f;
    private float shiftDuration = 0.3f;

    // ── Transmission ───────────────────────────────────────────────────────
    [Header("Transmission Customization")]
    public float[] gearRatios = { 3.67f, 2.10f, 1.36f, 1.00f };
    public float finalDriveRatio = 4.1f;
    public float shiftUpRPM = 5600f;
    public float shiftDownRPM = 3000f;
    public float reverseGearRatio = 3.53f;

    // ── Steering ───────────────────────────────────────────────────────────
    [Header("Steering")]
    [HideInInspector] public float steering;

    private float steeringSpeed = HalfPI;
    private float steeringAngle = SixthPI;

    public enum SteeringAxis { X, Y, Z }

    [Header("Steering Wheel")]
    public Transform steeringWheel;
    public float steeringWheelMaxAngle = 450f;
    public SteeringAxis steeringAxis = SteeringAxis.Y;

    private float steeringWheelCurrentAngle = 0f;
    private Quaternion steeringWheelBaseRotation;

    // ── Camera ─────────────────────────────────────────────────────────────
    [Header("Camera Control")]
    [Tooltip("Mouse look sensitivity. Increase if camera feels sluggish, decrease if too fast.")]
    public float mouseSensitivity = 0.1f;
    private float cameraDistance = 6f;
    private float targetPitch = -0.05235988f;
    private float targetYaw = 3.14159265f;
    public bool isInteriorCameraActive = false;

    // ── Drive / Transmission Mode ──────────────────────────────────────────
    public enum DriveMode { AWD, RWD, FWD }
    [Header("Drive Mode")]
    public DriveMode driveMode = DriveMode.RWD;

    public enum TransmissionMode { Automatic, Manual }
    [Header("Transmission Mode")]
    public TransmissionMode transmissionMode = TransmissionMode.Manual;

    // ── DynamicEngine references ───────────────────────────────────────────
    private SoftBody frame;
    private Solver solver;
    private Constraint[] wheels = new Constraint[0];

    private int steerR0, steerR1, steerL0, steerL1;
    private float steerHand;
    private float origLenR0, origLenR1, origLenL0, origLenL1;

    private Vector3 _lastPosition;

    // ── Reflection fields (cached once in Start) ───────────────────────────
    private static FieldInfo _enableMotorField;
    private static FieldInfo _targetRateField;
    private static FieldInfo _maxTorqueField;
    private static FieldInfo _angularVelField;

    // ── Node Grabbing ──────────────────────────────────────────────────────
    [Header("Node Grabbing")]
    [SerializeField, Range(0.1f, 5f)] private float nodeVisualBaseSize = 0.3f;
    [SerializeField, Range(0.1f, 5f)] private float nodeVisualDistanceScale = 0.05f;
    [SerializeField, Range(1f, 10000f)] private float nodeGrabForce = 100f;
    [SerializeField, Range(1f, 50f)] private float nodeGrabStiffness = 15f;
    [SerializeField, Range(0.1f, 2f)] private float nodeGrabRadius = 0.5f;
    [SerializeField, Range(0.5f, 10f)] private float nodeVisualShowRadius = 2f;
    [SerializeField] private Color nodeColorNormal = new Color(1f, 0.5f, 0f);
    [SerializeField] private Color nodeColorHover = Color.yellow;
    [SerializeField] private Color nodeColorGrabbed = Color.red;
    [SerializeField] private Color nodeColorPinned = Color.magenta;
    [SerializeField] private bool showGrabRadiusGizmo = true;

    private SoftBody draggedSoftBody;
    private int draggedNodeIndex = -1;
    private Vector3 dragOffset;
    private Plane fixedDragPlane;
    private Vector3 grabTargetPosition;
    private SoftBody hoveredSoftBody;
    private int hoveredNodeIndex = -1;
    private bool isGrabControlPressed = false;
    private bool wasGrabMousePressed = false;
    private float lastGrabForceMag = 0f;
    private Vector3 lastValidGrabMousePos;
    private Vector3 _smoothedGrabTarget;

    private Dictionary<SoftBody, GameObject> grabVisualsContainers = new Dictionary<SoftBody, GameObject>();
    private Dictionary<SoftBody, List<GameObject>> grabNodeVisualsDict = new Dictionary<SoftBody, List<GameObject>>();
    private Dictionary<SoftBody, LineRenderer> grabDragLineRenderers = new Dictionary<SoftBody, LineRenderer>();

    // ── Accessors ──────────────────────────────────────────────────────────
    public float GetThrottle() => throttle;
    public float GetEngineRPM() => engineRPM;
    public int GetGear() => gear;

    // ──────────────────────────────────────────────────────────────────────
    private void Start()
    {
        frame = GetComponent<SoftBody>();
        if (frame == null)
        {
            Debug.LogError("[CarController] No SoftBody component found!", this);
            enabled = false;
            return;
        }

        solver = frame.GetSolver();
        if (solver == null)
        {
            Debug.LogError("[CarController] Solver not ready. Ensure SoftBody has a Truss assigned.", this);
            enabled = false;
            return;
        }

        var ct = typeof(Constraint);
        const BindingFlags bf = BindingFlags.NonPublic | BindingFlags.Instance;
        _enableMotorField = ct.GetField("m_enableMotor", bf);
        _targetRateField = ct.GetField("m_targetRate", bf);
        _maxTorqueField = ct.GetField("m_maxTorque", bf);
        _angularVelField = typeof(Constraint).GetField("m_currentAngularVelocity",
    BindingFlags.NonPublic | BindingFlags.Instance);

        if (_angularVelField == null)
            Debug.LogError("[CarController] Reflection field m_currentAngularVelocity not found on Constraint!", this);

        InitializeTorqueCurve();
        InitializeWheels();
        InitializeSteering();
        FindMainCamera();
        FindInteriorCamera();
        SetInitialCameraPosition();

        cameraCollisionMask = ~LayerMask.GetMask("Ignore Raycast");

        if (targetCamera != null)
            targetCamera.gameObject.SetActive(!isInteriorCameraActive);

        if (interiorCamera != null)
        {
            interiorCamera.gameObject.SetActive(isInteriorCameraActive);
            var lowPass = interiorCamera.GetComponent<AudioLowPassFilter>()
                          ?? interiorCamera.gameObject.AddComponent<AudioLowPassFilter>();
            lowPass.cutoffFrequency = 1000f;
            lowPass.lowpassResonanceQ = 1f;
        }

        if (steeringWheel != null)
            steeringWheelBaseRotation = steeringWheel.localRotation;

        _lastPosition = transform.position;

        // ── Node grab init ─────────────────────────────────────────────────
        lastValidGrabMousePos = Mouse.current != null
            ? (Vector3)Mouse.current.position.ReadValue()
            : new Vector3(Screen.width / 2f, Screen.height / 2f, 0f);

        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 1000;

        Cursor.lockState = CursorLockMode.Locked;
    }

    private void Update()
    {
        if (Keyboard.current == null) return;

        if (Keyboard.current.cKey.wasPressedThisFrame) ToggleInteriorCamera();
        bool shiftHeld = Keyboard.current.leftShiftKey.isPressed ||
                         Keyboard.current.rightShiftKey.isPressed;

        if (Mouse.current != null)
        {
            float scroll = Mouse.current.scroll.ReadValue().y;
            if (!shiftHeld)
            {
                cameraDistance = Mathf.Clamp(cameraDistance - scroll * 0.3f, 3f, 30f);
            }
            else
            {
                if (scroll < 0f) Time.timeScale = Mathf.Max(Time.timeScale * 0.5f, 0f);
                else if (scroll > 0f) Time.timeScale = Mathf.Min(Time.timeScale * 2f, 1f);
            }
        }

        if (Keyboard.current.pKey.wasPressedThisFrame)
            Time.timeScale = (Time.timeScale == 1f) ? 0f : 1f;

        if (Keyboard.current.digit1Key.wasPressedThisFrame) { driveMode = DriveMode.AWD; Debug.Log("Drive Mode: AWD"); }
        if (Keyboard.current.digit2Key.wasPressedThisFrame) { driveMode = DriveMode.RWD; Debug.Log("Drive Mode: RWD"); }
        if (Keyboard.current.digit3Key.wasPressedThisFrame) { driveMode = DriveMode.FWD; Debug.Log("Drive Mode: FWD"); }

        if (transmissionMode == TransmissionMode.Manual)
        {
            if (Keyboard.current.zKey.wasPressedThisFrame && gear < gearRatios.Length)
            {
                gear++;
                Debug.Log("Shifted Up. Gear: " + (gear == 0 ? "Neutral" : gear.ToString()));
                StartShift();
            }
            if (Keyboard.current.xKey.wasPressedThisFrame && gear > -1)
            {
                gear--;
                Debug.Log("Shifted Down. Gear: " + (gear == 0 ? "Neutral" : gear.ToString()));
                StartShift();
            }
        }

        HandleNodeGrabbing();
    }

    private void FixedUpdate()
    {
        if (frame == null || !frame.gameObject.activeSelf) return;

        Vector3 velocity = (transform.position - _lastPosition) / Time.fixedDeltaTime;
        forwardSpeed = Vector3.Dot(velocity, transform.forward);
        _lastPosition = transform.position;

        UpdateSteering();
        UpdateThrottle();
        NodeGrabFixedUpdate();
    }

    private void LateUpdate()
    {
        LateUpdateCamera();
    }

    private void OnApplicationFocus(bool focusStatus)
    {
        if (!Application.isEditor)
        {
            Cursor.lockState = focusStatus ? CursorLockMode.Locked : CursorLockMode.None;
            Cursor.visible = !focusStatus;
        }
    }

    private void OnGUI()
    {
    }

    private void OnDestroy()
    {
        foreach (var kvp in grabVisualsContainers)
        {
            if (kvp.Value == null) continue;
#if UNITY_EDITOR
            if (!Application.isPlaying)
                DestroyImmediate(kvp.Value);
            else
                Destroy(kvp.Value);
#else
            Destroy(kvp.Value);
#endif
        }
        grabVisualsContainers.Clear();
        grabNodeVisualsDict.Clear();
        grabDragLineRenderers.Clear();
    }

    // ── Initialization ─────────────────────────────────────────────────────

    private void InitializeWheels()
    {
        var all = GetComponentsInChildren<Constraint>(includeInactive: true);
        wheels = System.Array.FindAll(all, c => c.isWheel);

        foreach (Constraint c in wheels)
            c.enableMotor = true;
    }

    private void InitializeSteering()
    {
        Truss truss = frame.GetTrussAsset();
        if (truss == null)
        {
            Debug.LogError("[CarController] No Truss asset on SoftBody — steering disabled.", this);
            return;
        }

        int[] r0 = truss.GetLinkSetIndices("Steer_R0");
        int[] r1 = truss.GetLinkSetIndices("Steer_R1");
        int[] l0 = truss.GetLinkSetIndices("Steer_L0");
        int[] l1 = truss.GetLinkSetIndices("Steer_L1");

        if (r0 == null || r0.Length == 0 || r1 == null || r1.Length == 0 ||
            l0 == null || l0.Length == 0 || l1 == null || l1.Length == 0)
        {
            Debug.LogError("[CarController] Steering link sets (Steer_R0/R1/L0/L1) missing.", this);
            return;
        }

        steerR0 = r0[0]; steerR1 = r1[0];
        steerL0 = l0[0]; steerL1 = l1[0];

        origLenR0 = solver.beams[steerR0].restLength;
        origLenR1 = solver.beams[steerR1].restLength;
        origLenL0 = solver.beams[steerL0].restLength;
        origLenL1 = solver.beams[steerL1].restLength;

        steerHand = origLenR0 / Sqrt2;
    }

    private void FindMainCamera()
    {
        if (targetCamera != null) return;
        var cams = GameObject.FindGameObjectsWithTag("MainCamera");
        if (cams.Length > 0) targetCamera = cams[^1].GetComponent<Camera>();
        if (targetCamera == null) targetCamera = Camera.main;
        if (targetCamera == null)
            Debug.LogWarning("[CarController] No main camera found.");
    }

    private void FindInteriorCamera()
    {
        if (interiorCamera != null) return;
        var cams = GameObject.FindGameObjectsWithTag("InteriorCam");
        if (cams.Length > 0) interiorCamera = cams[0].GetComponent<Camera>();
    }

    private void SetInitialCameraPosition()
    {
        if (targetCamera == null) return;
        Vector3 lookTarget = transform.position + transform.forward * 0.5f + Vector3.up * 0.5f;
        Vector3 dir = Quaternion.Euler(targetPitch * Mathf.Rad2Deg, targetYaw * Mathf.Rad2Deg, 0f) * Vector3.forward;
        targetCamera.transform.position = lookTarget + dir * cameraDistance;
        smoothCollisionDistance = cameraDistance;
        targetCamera.transform.LookAt(lookTarget);
    }

    private void ToggleInteriorCamera()
    {
        if (interiorCamera == null || targetCamera == null) return;
        isInteriorCameraActive = !isInteriorCameraActive;
        targetCamera.gameObject.SetActive(!isInteriorCameraActive);
        interiorCamera.gameObject.SetActive(isInteriorCameraActive);
    }

    // ── Camera ─────────────────────────────────────────────────────────────
    private void LateUpdateCamera()
    {
        if (isInteriorCameraActive || targetCamera == null) return;

        if (Mouse.current != null && Mouse.current.rightButton.isPressed)
        {
            // Lock & hide cursor while rotating
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;

            Vector2 delta = Mouse.current.delta.ReadValue();

            float mouseX = delta.x * mouseSensitivity * 0.01f;
            float mouseY = delta.y * mouseSensitivity * 0.01f;

            if (Mathf.Abs(mouseX) > float.Epsilon) targetYaw += mouseX;
            if (Mathf.Abs(mouseY) > float.Epsilon)
                targetPitch = Mathf.Clamp(targetPitch + mouseY, -1.5393804f, 1.5393804f);
        }
        else
        {
            // Unlock & show cursor when not rotating
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }

        Vector3 lookTarget = transform.position + transform.forward * 0.5f + Vector3.up * 0.5f;
        float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(forwardSpeed) / 100f);

        speedTilt = Mathf.Lerp(speedTilt, normalizedSpeed, Time.deltaTime * 2f);

        float dynamicPitch = targetPitch + Mathf.Lerp(0f, 10f, speedTilt) * -Mathf.Deg2Rad;
        float dynamicDistance = cameraDistance + Mathf.Lerp(0f, 2f, speedTilt);

        Vector3 dir = Quaternion.Euler(dynamicPitch * Mathf.Rad2Deg, targetYaw * Mathf.Rad2Deg, 0f) * Vector3.forward;

        float desiredDistance = dynamicDistance;
        if (Physics.SphereCast(lookTarget, 0.3f, dir, out RaycastHit hit, dynamicDistance, cameraCollisionMask))
            desiredDistance = Mathf.Clamp(hit.distance - 0.1f, 0.5f, dynamicDistance);

        float lerpSpeed = (desiredDistance < smoothCollisionDistance) ? 10f : 2f;
        smoothCollisionDistance = Mathf.Lerp(smoothCollisionDistance, desiredDistance, Time.deltaTime * lerpSpeed);

        targetCamera.transform.position = lookTarget + dir * smoothCollisionDistance;
        targetCamera.transform.LookAt(lookTarget);
    }

    // ── Input helpers ───────────────────────────────────────────────────────
    // Replaces Input.GetAxis("Horizontal") / ("Vertical") / GetButton("Jump").
    // Returns -1, 0, or 1 — digital keys, same as the default Unity axis settings.

    private static float GetHorizontalInput()
    {
        if (Keyboard.current == null) return 0f;
        float v = 0f;
        if (Keyboard.current.aKey.isPressed) v -= 1f;
        if (Keyboard.current.dKey.isPressed) v += 1f;
        return v;
    }

    private static float GetVerticalInput()
    {
        if (Keyboard.current == null) return 0f;
        float v = 0f;
        if (Keyboard.current.wKey.isPressed) v += 1f;
        if (Keyboard.current.sKey.isPressed) v -= 1f;
        return v;
    }

    private static bool GetHandbrake()
        => Keyboard.current != null && Keyboard.current.spaceKey.isPressed;

    // ── Steering ───────────────────────────────────────────────────────────
    private void UpdateSteering()
    {
        if (solver == null || solver.beams == null) return;

        float horizontalInput = GetHorizontalInput();
        float targetSteering = horizontalInput * steeringAngle;
        float steeringDelta = targetSteering - steering;
        steering += Mathf.Clamp(steeringDelta,
                                -steeringSpeed * Time.fixedDeltaTime,
                                 steeringSpeed * Time.fixedDeltaTime);

        solver.beams[steerR0].restLength = steerHand * Mathf.Sin((HalfPI + steering) * 0.5f) * 2f;
        solver.beams[steerR1].restLength = steerHand * Mathf.Sin((HalfPI - steering) * 0.5f) * 2f;
        solver.beams[steerL0].restLength = steerHand * Mathf.Sin((HalfPI - steering) * 0.5f) * 2f;
        solver.beams[steerL1].restLength = steerHand * Mathf.Sin((HalfPI + steering) * 0.5f) * 2f;

        if (steeringWheel != null)
        {
            float normalizedSteer = Mathf.Clamp(horizontalInput, -1f, 1f);
            float targetAngle = normalizedSteer * (steeringWheelMaxAngle * 0.5f);
            steeringWheelCurrentAngle = Mathf.Lerp(steeringWheelCurrentAngle, targetAngle,
                                                    Time.fixedDeltaTime * 10f);

            Vector3 axis = steeringAxis switch
            {
                SteeringAxis.X => Vector3.right,
                SteeringAxis.Z => Vector3.forward,
                _ => Vector3.up
            };
            steeringWheel.localRotation =
                steeringWheelBaseRotation * Quaternion.AngleAxis(steeringWheelCurrentAngle, axis);
        }
    }

    // ── Motor helpers ──────────────────────────────────────────────────────

    private void SetConstraintMotor(Constraint c, float targetAngularSpeedRadS, float torque)
    {
        if (c == null) return;
        c.enableMotor = true;
        c.targetRate = targetAngularSpeedRadS * Mathf.Rad2Deg;
        c.maxTorque = torque;
    }

    private void DisableConstraintMotor(Constraint c)
    {
        if (c == null) return;
        c.enableMotor = false;
    }

    private float GetMinWheelRPM()
    {
        float rpm = float.PositiveInfinity;
        foreach (Constraint w in wheels)
        {
            if (w == null || !w.enabled) continue;
            if (_angularVelField != null)
                rpm = Mathf.Min(rpm, Mathf.Abs((float)_angularVelField.GetValue(w)));
        }
        return float.IsPositiveInfinity(rpm) ? 0f : rpm;
    }

    // ── Auto-shift ─────────────────────────────────────────────────────────
    private void TryAutoShift(bool useGearDelay)
    {
        if (transmissionMode != TransmissionMode.Automatic || gear < 1) return;

        if (useGearDelay)
        {
            gearTime -= Time.fixedDeltaTime;
            if (gearTime > 0f) goto clamp;
        }

        if (engineTargetRPM < shiftDownRPM && gear > 1)
        {
            gear--;
            if (useGearDelay) gearTime = 0.5f;
            StartShift();
        }
        else if (engineTargetRPM > shiftUpRPM && gear < gearRatios.Length)
        {
            gear++;
            if (useGearDelay) gearTime = 0.5f;
            StartShift();
        }

    clamp:
        gear = Mathf.Clamp(gear, 1, gearRatios.Length);
    }

    private void StartShift()
    {
        isShifting = true;
        shiftTimer = shiftDuration;
    }

    // ── Engine torque curve ────────────────────────────────────────────────
    private float EngineCurve(float rpm, float minRPM, float maxRPMVal)
    {
        float span = maxRPMVal - minRPM;
        if (span <= 0f) return 0f;
        return Mathf.Clamp01(torqueCurve.Evaluate(Mathf.Clamp01((rpm - minRPM) / span)));
    }

    private void InitializeTorqueCurve()
    {
        if (torqueCurve != null && torqueCurve.keys.Length > 1) return;

        torqueCurve = new AnimationCurve(
            new Keyframe(0.00f, 0.00f),
            new Keyframe(0.13f, 0.52f),
            new Keyframe(0.33f, 0.82f),
            new Keyframe(0.58f, 1.00f),
            new Keyframe(0.80f, 0.88f),
            new Keyframe(1.00f, 0.58f)
        );
        for (int i = 0; i < torqueCurve.keys.Length; i++)
            torqueCurve.SmoothTangents(i, 0f);
    }

    // ── Throttle / Physics ─────────────────────────────────────────────────
    private void UpdateThrottle()
    {
        // ── Engine off ────────────────────────────────────────────────────
        if (!engineRunning)
        {
            throttle = 0f;
            engineRPM = Mathf.Lerp(engineRPM, 0f, Time.fixedDeltaTime * 3f);
            engineTargetRPM = 0f;
            engineForce = 0f;

            float brakeInput = Mathf.Clamp01(-GetVerticalInput());
            float engineOffBrake = brakeInput * maxBrakeTorque;
            currentBrakeTorque = Mathf.Lerp(currentBrakeTorque, engineOffBrake,
                                              Time.fixedDeltaTime * brakeResponseSpeed * 2f);

            foreach (Constraint w in wheels)
            {
                if (w == null) continue;
                SetConstraintMotor(w, 0f, currentBrakeTorque);
            }
            return;
        }

        // ── Shift animation ───────────────────────────────────────────────
        if (isShifting)
        {
            shiftTimer -= Time.fixedDeltaTime;
            if (shiftTimer <= 0f)
                isShifting = false;
            else
            {
                throttle = Mathf.Lerp(throttle, 0f, shiftTimer / shiftDuration);
                return;
            }
        }

        float targetThrottle = GetVerticalInput();
        if (Mathf.Abs(targetThrottle) < 0.05f) targetThrottle = 0f;

        float smoothFactor = (Mathf.Abs(targetThrottle) >= Mathf.Abs(throttle))
                             ? accelSmoothing : decelSmoothing;
        throttle = Mathf.Lerp(throttle, targetThrottle, smoothFactor * Time.fixedDeltaTime);

        float maxTorque = peakTorqueNm;
        float targetRPM = 0f;
        float engineTorque = 0f;

        if (transmissionMode == TransmissionMode.Automatic)
        {
            if (throttle < -0.1f && Mathf.Abs(forwardSpeed) < 2f && gear >= 0) gear = -1;
            else if (throttle > 0.1f && gear == -1) gear = 0;
            else if (throttle > 0.1f && gear == 0) gear = 1;
        }

        float rawInput = GetVerticalInput();
        if (Mathf.Abs(rawInput) < 0.05f) rawInput = 0f;

        bool isBraking =
            (rawInput < 0f && gear > 0 && Mathf.Abs(forwardSpeed) > 1f) ||
            (rawInput > 0f && gear == -1 && Mathf.Abs(forwardSpeed) > 1f);

        if (isBraking)
        {
            float targetBrakeTorque = Mathf.Clamp01(Mathf.Abs(rawInput)) * maxBrakeTorque;
            currentBrakeTorque = Mathf.Lerp(currentBrakeTorque, targetBrakeTorque,
                                             Time.fixedDeltaTime * brakeResponseSpeed);
        }
        else
        {
            currentBrakeTorque = 0f;
        }

        float currentGearRatio = 0f;
        if (gear > 0)
            currentGearRatio = (gear <= gearRatios.Length
                                ? gearRatios[gear - 1]
                                : gearRatios[^1]) * finalDriveRatio;
        else if (gear == -1)
            currentGearRatio = reverseGearRatio * finalDriveRatio;

        if (isBraking)
        {
            engineTorque = currentBrakeTorque;
            throttleTime = 0f;
            float wheelRPM = GetMinWheelRPM();
            float idleShake = Random.Range(-idleRPMVariation, idleRPMVariation);
            engineTargetRPM = Mathf.Max(idleRPM, wheelRPM / TwoPI * 60f * currentGearRatio) + idleShake;
            engineRPM = Mathf.Lerp(engineRPM, engineTargetRPM, Time.fixedDeltaTime * idleRPMSmoothing * 2f);
            targetRPM = 0f;
            TryAutoShift(false);
        }
        else if (Mathf.Abs(throttle) > 0.01f)
        {
            oldThrottle = throttle;
            throttleTime += Time.fixedDeltaTime;

            if (gear == 0)
            {
                float idleShake = Random.Range(-idleRPMVariation, idleRPMVariation);
                engineTargetRPM = idleRPM + Mathf.Abs(throttle) * (maxRPM - idleRPM) + idleShake;
                engineRPM = Mathf.Lerp(engineRPM, engineTargetRPM, Time.fixedDeltaTime * idleRPMSmoothing);
                targetRPM = 0f;
                engineTorque = 0f;
            }
            else
            {
                float wheelRPM = GetMinWheelRPM();
                float idleShake = Random.Range(-idleRPMVariation, idleRPMVariation);
                engineTargetRPM = Mathf.Max(idleRPM, wheelRPM / TwoPI * 60f * currentGearRatio) + idleShake;
                engineRPM = Mathf.Lerp(engineRPM, engineTargetRPM, Time.fixedDeltaTime * idleRPMSmoothing);
                TryAutoShift(true);
                targetRPM = Mathf.Sign(throttle) * maxRPM * TwoPI / 60f / Mathf.Max(currentGearRatio, 0.01f);
                engineTorque = Mathf.Abs(throttle) * maxTorque
                               * EngineCurve(engineTargetRPM, 0f, maxRPM)
                               * Mathf.Clamp01(throttleTime)
                               * currentGearRatio;
            }
        }
        else
        {
            float wheelRPM = GetMinWheelRPM();
            float idleShake = Random.Range(-idleRPMVariation, idleRPMVariation);
            engineTargetRPM = Mathf.Max(idleRPM, wheelRPM / TwoPI * 60f * currentGearRatio) + idleShake;
            engineRPM = Mathf.Lerp(engineRPM, engineTargetRPM, Time.fixedDeltaTime * idleRPMSmoothing);
            TryAutoShift(false);
            targetRPM = 0f;
            engineTorque = (1f + engineTargetRPM / maxRPM) * currentGearRatio;
            throttleTime = 0f;
            oldThrottle = 0f;
        }

        engineForce = engineTorque;

        bool handbrake = GetHandbrake();

        // ── Gradual motor-rate ramp ────────────────────────────────────────
        // Instead of slamming the full targetRPM onto the constraint every tick,
        // smoothly move _currentMotorRate toward it.  This prevents the sudden
        // angular-velocity spike that makes the soft-body car unstable.
        bool motorActive = Mathf.Abs(throttle) > 0.01f && !isBraking && gear != 0;
        if (motorActive)
        {
            _currentMotorRate = Mathf.Lerp(_currentMotorRate, targetRPM,
                                            motorRampSpeed * Time.fixedDeltaTime);
        }
        else
        {
            // Snap back to zero so the next throttle press always ramps from rest.
            _currentMotorRate = Mathf.Lerp(_currentMotorRate, 0f,
                                            motorRampSpeed * 2f * Time.fixedDeltaTime);
        }

        foreach (Constraint wheel in wheels)
        {
            if (wheel == null) continue;

            if (!wheel.enabled)
            {
                DisableConstraintMotor(wheel);
                continue;
            }

            if (gear == 0)
            {
                DisableConstraintMotor(wheel);
                continue;
            }

            float dirMult = (Vector3.Dot(wheel.transform.right, transform.right) < 0f) ? 1f : -1f;
            bool isFront = Vector3.Dot(transform.forward,
                                        wheel.transform.position - transform.position) > 0f;

            bool driven = driveMode switch
            {
                DriveMode.AWD => true,
                DriveMode.RWD => !isFront,
                DriveMode.FWD => isFront,
                _ => false
            };

            if (handbrake)
            {
                if (!isFront) SetConstraintMotor(wheel, 0f, 50000f);
                else DisableConstraintMotor(wheel);
                continue;
            }

            if (driven)
            {
                if (Mathf.Abs(throttle) > 0.01f || isBraking)
                    // Use the ramped rate instead of the raw targetRPM.
                    SetConstraintMotor(wheel, _currentMotorRate * dirMult, engineTorque);
                else
                    DisableConstraintMotor(wheel);
            }
            else
            {
                DisableConstraintMotor(wheel);
            }
        }
    }

    // ── Node Grabbing ──────────────────────────────────────────────────────

    private Camera GetActiveCamera() => isInteriorCameraActive ? interiorCamera : targetCamera;

    private void NodeGrabFixedUpdate()
    {
        if (draggedSoftBody == null || draggedNodeIndex < 0) return;

        var nm = draggedSoftBody.solver?.nodeManager;
        if (nm == null || draggedNodeIndex >= nm.NodeCount) return;

        float blend = Mathf.Clamp01(nodeGrabStiffness * Time.fixedDeltaTime);
        _smoothedGrabTarget = Vector3.Lerp(_smoothedGrabTarget, grabTargetPosition, blend);

        Vector3 current = nm.GetPosition(draggedNodeIndex);
        Vector3 toTarget = _smoothedGrabTarget - current;
        float distance = toTarget.magnitude;

        if (distance < 0.001f)
        {
            nm.PreviousPositions[draggedNodeIndex] = current;
            lastGrabForceMag = 0f;
            return;
        }

        // Read the node's current implicit Verlet velocity
        Vector3 currentVelocity = (current - nm.PreviousPositions[draggedNodeIndex]) / Time.fixedDeltaTime;

        // Spring force toward target + critical damping to kill oscillation
        float damping = 2f * Mathf.Sqrt(nodeGrabStiffness);
        Vector3 injectVelocity = currentVelocity
            + toTarget * (nodeGrabStiffness * Time.fixedDeltaTime)
            - currentVelocity * (damping * Time.fixedDeltaTime);

        injectVelocity = Vector3.ClampMagnitude(injectVelocity, nodeGrabForce);
        nm.PreviousPositions[draggedNodeIndex] = current - injectVelocity * Time.fixedDeltaTime;
        lastGrabForceMag = injectVelocity.magnitude;
    }

    private void HandleNodeGrabbing()
    {
        if (Keyboard.current == null || Mouse.current == null) return;

        Camera grabCam = GetActiveCamera();
        if (grabCam == null) return;

        // Track last valid mouse position
        Vector3 mousePos = Mouse.current.position.ReadValue();
        if (mousePos.x >= 0 && mousePos.y >= 0 &&
            mousePos.x <= Screen.width && mousePos.y <= Screen.height &&
            !float.IsNaN(mousePos.x) && !float.IsNaN(mousePos.y) &&
            !float.IsInfinity(mousePos.x) && !float.IsInfinity(mousePos.y))
        {
            lastValidGrabMousePos = mousePos;
        }

        isGrabControlPressed = Keyboard.current.leftCtrlKey.isPressed ||
                               Keyboard.current.rightCtrlKey.isPressed;

        if (isGrabControlPressed)
        {
            // Lazily set up visuals for the car's frame
            if (frame != null && !grabVisualsContainers.ContainsKey(frame))
                SetupNodeGrabVisuals(frame);

            Ray ray = grabCam.ScreenPointToRay(lastValidGrabMousePos);

            hoveredSoftBody = null;
            hoveredNodeIndex = -1;
            float closestDist = float.MaxValue;

            if (frame != null && frame.solver?.nodeManager != null)
            {
                for (int i = 0; i < frame.solver.nodeManager.NodeCount; i++)
                {
                    Vector3 nodePos = frame.solver.nodeManager.GetPosition(i);
                    float rayDist = Vector3.Cross(ray.direction, nodePos - ray.origin).magnitude;
                    if (rayDist < nodeGrabRadius && rayDist < closestDist)
                    {
                        closestDist = rayDist;
                        hoveredSoftBody = frame;
                        hoveredNodeIndex = i;
                    }
                }
            }

            // Start drag
            if (Mouse.current.leftButton.wasPressedThisFrame &&
                hoveredSoftBody != null && hoveredNodeIndex >= 0)
            {
                draggedSoftBody = hoveredSoftBody;
                draggedNodeIndex = hoveredNodeIndex;

                Vector3 nodePos = draggedSoftBody.solver.nodeManager.GetPosition(draggedNodeIndex);
                Ray clickRay = grabCam.ScreenPointToRay(lastValidGrabMousePos);
                fixedDragPlane = new Plane(grabCam.transform.forward, nodePos);

                if (fixedDragPlane.Raycast(clickRay, out float enterClick))
                {
                    dragOffset = nodePos - clickRay.GetPoint(enterClick);
                }
                else
                {
                    float approxDepth = Mathf.Max(0.01f,
                        Vector3.Dot(nodePos - clickRay.origin, clickRay.direction));
                    dragOffset = nodePos - clickRay.GetPoint(approxDepth);
                }

                grabTargetPosition = nodePos;
                _smoothedGrabTarget = nodePos;
                wasGrabMousePressed = true;
            }

            // Continue drag
            if (draggedSoftBody != null && draggedNodeIndex >= 0 &&
                draggedSoftBody.solver?.nodeManager != null &&
                draggedNodeIndex < draggedSoftBody.solver.nodeManager.NodeCount)
            {
                Ray dragRay = grabCam.ScreenPointToRay(lastValidGrabMousePos);
                if (fixedDragPlane.Raycast(dragRay, out float enter))
                {
                    grabTargetPosition = dragRay.GetPoint(enter) + dragOffset;
                }
                else
                {
                    Vector3 curPos = draggedSoftBody.solver.nodeManager.GetPosition(draggedNodeIndex);
                    float d = Mathf.Max(0.01f,
                        Vector3.Dot(curPos - dragRay.origin, dragRay.direction));
                    grabTargetPosition = dragRay.GetPoint(d) + dragOffset;
                }

                // Middle click toggles pin on the grabbed node
                if (Mouse.current.middleButton.wasPressedThisFrame)
                {
                    bool pinned = draggedSoftBody.solver.nodeManager.IsPinned[draggedNodeIndex];
                    draggedSoftBody.solver.nodeManager.SetPinned(draggedNodeIndex, !pinned);
                }
            }
            else if (draggedSoftBody != null)
            {
                draggedSoftBody = null;
                draggedNodeIndex = -1;
                wasGrabMousePressed = false;
                lastGrabForceMag = 0f;
            }

            // Release drag
            if (Mouse.current.leftButton.wasReleasedThisFrame && wasGrabMousePressed)
            {
                draggedSoftBody = null;
                draggedNodeIndex = -1;
                wasGrabMousePressed = false;
                lastGrabForceMag = 0f;
            }

            // Update visuals
            if (frame != null)
            {
                int hoverIdx = (frame == hoveredSoftBody) ? hoveredNodeIndex : -1;
                int dragIdx = (frame == draggedSoftBody) ? draggedNodeIndex : -1;
                Vector3 targ = (frame == draggedSoftBody) ? grabTargetPosition : Vector3.zero;
                UpdateNodeGrabVisuals(frame, true, hoverIdx, dragIdx, targ);
            }
        }
        else
        {
            // Ctrl released — hide visuals and cancel any drag
            if (frame != null)
                UpdateNodeGrabVisuals(frame, false, -1, -1, Vector3.zero);

            if (draggedSoftBody != null)
            {
                draggedSoftBody = null;
                draggedNodeIndex = -1;
                wasGrabMousePressed = false;
            }
        }
    }

    private void SetupNodeGrabVisuals(SoftBody sb)
    {
        if (sb == null || sb.solver?.nodeManager == null) return;

        GameObject container = new GameObject("NodeGrabberVisuals");
        container.transform.SetParent(sb.transform, false);
        container.hideFlags = HideFlags.DontSave;
        grabVisualsContainers[sb] = container;

        List<GameObject> nodeVis = new List<GameObject>();
        int nodeCount = sb.solver.nodeManager.NodeCount;
        for (int i = 0; i < nodeCount; i++)
        {
            GameObject nodeVisual = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            nodeVisual.transform.SetParent(container.transform, false);
            nodeVisual.hideFlags = HideFlags.DontSave;
#if UNITY_EDITOR
            DestroyImmediate(nodeVisual.GetComponent<Collider>());
#else
            Destroy(nodeVisual.GetComponent<Collider>());
#endif
            Material mat = new Material(Shader.Find("Sprites/Default"));
            nodeVisual.GetComponent<Renderer>().material = mat;
            nodeVis.Add(nodeVisual);
        }
        grabNodeVisualsDict[sb] = nodeVis;

        GameObject dragLineObj = new GameObject("DragLine");
        dragLineObj.transform.SetParent(container.transform, false);
        LineRenderer lr = dragLineObj.AddComponent<LineRenderer>();
        lr.enabled = false;
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.positionCount = 2;
        lr.useWorldSpace = true;
        lr.startColor = Color.black;
        lr.endColor = Color.black;
        lr.startWidth = 0.05f;
        lr.endWidth = 0.05f;
        grabDragLineRenderers[sb] = lr;
    }

    private void UpdateNodeGrabVisuals(SoftBody sb, bool show, int hoverIdx, int dragIdx, Vector3 targPos)
    {
        if (!grabVisualsContainers.TryGetValue(sb, out GameObject container) || container == null) return;
        if (!grabNodeVisualsDict.TryGetValue(sb, out var nodeVisList) || nodeVisList == null) return;

        var nodes = sb.solver.nodeManager.CurrentPositions;
        if (nodes == null) return;

        int nodeCount = nodes.Count;
        int centerIdx = dragIdx >= 0 ? dragIdx : hoverIdx;
        bool hasCenter = centerIdx >= 0;

        Camera grabCam = GetActiveCamera();

        for (int i = 0; i < nodeCount; i++)
        {
            if (i >= nodeVisList.Count || nodeVisList[i] == null) continue;

            GameObject nodeObj = nodeVisList[i];

            if (!show)
            {
                nodeObj.SetActive(false);
                continue;
            }

            Vector3 nodePos = nodes[i];
            float distance = grabCam != null
                ? Vector3.Distance(nodePos, grabCam.transform.position)
                : 1f;
            float scaledSize = nodeVisualBaseSize * (1f + distance * nodeVisualDistanceScale);
            nodeObj.transform.position = nodePos;
            nodeObj.transform.localScale = Vector3.one * scaledSize * 0.2f;

            bool isVisible = !hasCenter ||
                Vector3.Distance(nodePos, nodes[centerIdx]) <= nodeVisualShowRadius;
            nodeObj.SetActive(isVisible);

            if (isVisible)
            {
                Renderer rend = nodeObj.GetComponent<Renderer>();
                if (rend != null)
                {
                    Color col = nodeColorNormal;
                    if (sb.solver.nodeManager.IsPinned[i])
                        col = nodeColorPinned;
                    else if (i == dragIdx)
                        col = nodeColorGrabbed;
                    else if (i == hoverIdx)
                        col = nodeColorHover;
                    rend.material.color = col;
                }

                if (showGrabRadiusGizmo && (i == hoverIdx || i == dragIdx))
                {
                    Debug.DrawRay(nodePos, Vector3.right * nodeGrabRadius, Color.white * 0.3f, 0f);
                    Debug.DrawRay(nodePos, Vector3.up * nodeGrabRadius, Color.white * 0.3f, 0f);
                    Debug.DrawRay(nodePos, Vector3.forward * nodeGrabRadius, Color.white * 0.3f, 0f);
                }
            }
        }

        if (grabDragLineRenderers.TryGetValue(sb, out LineRenderer dlr))
        {
            if (show && dragIdx >= 0)
            {
                dlr.enabled = true;
                if (dlr.positionCount < 2) dlr.positionCount = 2;
                dlr.SetPosition(0, nodes[dragIdx]);
                dlr.SetPosition(1, targPos);
            }
            else
            {
                dlr.enabled = false;
            }
        }
    }
}