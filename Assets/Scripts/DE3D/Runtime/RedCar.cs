/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using UnityEngine;
using UnityEngine.InputSystem;
using System.Reflection;
using System.Collections.Generic;
using DynamicEngine;

[RequireComponent(typeof(SoftBody))]
public class RedCar : MonoBehaviour
{
    [Header("Car Configuration")]
    public Camera targetCamera;
    public LayerMask cameraGroundMask;
    public float baseLength = 3f;
    public float baseWidth = 1.5f;
    public float wheelRadius = 0.5f;
    [Header("Camera Follow")]
    public Transform followTarget;

    [Header("Engine")]
    public float maxEngineTorque = 10f;
    public float maxEngineRPM = 1000f;
    public float[] gearRatios = { 6.6f, 4f, 2.8f, 2.1f, 1.6f };
    public float reverseGearRatio = 5.0f;
    public float finalDriveRatio = 4.5f;

    [Header("Steering")]
    public float maxSteeringAngle = 65f;
    public float steeringSpeed = 90f;

    [Header("Suspension")]
    public float suspensionStiffness = 1000f;
    public float suspensionDamping = 50f;

    // Private variables
    private SoftBody frame;
    private Solver solver;

    // Steering
    private Beam steerR0Beam;
    private Beam steerR1Beam;
    private Beam steerL0Beam;
    private Beam steerL1Beam;
    private float originalSteerR0Length;
    private float originalSteerR1Length;
    private float originalSteerL0Length;
    private float originalSteerL1Length;

    // Wheels
    private List<WheelData> wheels = new List<WheelData>();
    private float currentSteeringAngle;
    private static FieldInfo m_angularVelField;

    // Engine
    private float throttle;
    private int currentGear = 0;
    private float engineRPM;
    private float engineTorque;
    private float forwardSpeed;
    private Vector3 lastPosition;

    // Camera
    private float pitch = -3f;
    private float targetPitch = -3f;
    private float yaw = 180f;
    private float targetYaw = 180f;
    private float cameraDistance = 6f;

    [System.Serializable]
    private class WheelData
    {
        public Constraint constraint;
        public SoftBody wheelBody;
        public bool isRearWheel;
        public float currentRotation;
        public float angularVelocity;
        public float motorAngularVelocity;

        public void UpdateRotation(float deltaTime)
        {
            if (constraint != null)
            {
                // FIX: Sample actual physics velocity from the constraint
                if (m_angularVelField == null)
                    m_angularVelField = typeof(Constraint).GetField("m_currentAngularVelocity", BindingFlags.NonPublic | BindingFlags.Instance);

                angularVelocity = (float)m_angularVelField.GetValue(constraint);
                currentRotation += angularVelocity * deltaTime; // Correctly accumulate rotation
            }
        }
    }
    private void Start()
    {
        frame = GetComponent<SoftBody>();
        if (frame == null)
        {
            Debug.LogError("RedCar requires a SoftBody component!");
            enabled = false;
            return;
        }
        if (followTarget == null)
            followTarget = transform;

        solver = frame.GetSolver();
        if (solver == null)
        {
            Debug.LogError("Solver not found on SoftBody!");
            enabled = false;
            return;
        }

        // Initialize wheels
        InitializeWheels();

        // Initialize steering
        InitializeSteering();

        // Camera setup
        if (targetCamera == null)
            targetCamera = Camera.main;

        UpdateCameraPosition();
        lastPosition = transform.position;

        Cursor.lockState = CursorLockMode.Locked;
    }

    private void SetConstraintMotor(Constraint c, float targetAngularSpeed, float torque)
    {
        if (c == null) return;

        var constraintType = typeof(Constraint);

        // Enable motor
        var enableMotorField = constraintType.GetField("m_enableMotor",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (enableMotorField != null)
            enableMotorField.SetValue(c, true);

        // Set target rate (convert rad/s to deg/s for the constraint)
        var targetRateField = constraintType.GetField("m_targetRate",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (targetRateField != null)
            targetRateField.SetValue(c, targetAngularSpeed * Mathf.Rad2Deg);

        // Set max torque
        var maxTorqueField = constraintType.GetField("m_maxTorque",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (maxTorqueField != null)
            maxTorqueField.SetValue(c, torque);
    }

    private void DisableConstraintMotor(Constraint c)
    {
        if (c == null) return;

        var constraintType = typeof(Constraint);
        var enableMotorField = constraintType.GetField("m_enableMotor",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (enableMotorField != null)
            enableMotorField.SetValue(c, false);
    }

    private void InitializeWheels()
    {
        // Find all constraints attached to wheels
        Constraint[] allConstraints = GetComponentsInChildren<Constraint>();

        foreach (Constraint constraint in allConstraints)
        {
            var constraintType = typeof(Constraint);
            var attachedBodyField = constraintType.GetField("m_attachedBody",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);

            SoftBody attachedBody = attachedBodyField?.GetValue(constraint) as SoftBody;

            if (attachedBody != null && attachedBody != frame)
            {
                WheelData wheelData = new WheelData
                {
                    constraint = constraint,
                    wheelBody = attachedBody,
                    isRearWheel = IsRearWheel(constraint.transform),
                    currentRotation = 0f,
                    angularVelocity = 0f,
                    motorAngularVelocity = 0f
                };

                wheels.Add(wheelData);

                // Verify the constraint has an axis node set configured for motor
                if (wheelData.isRearWheel)
                {
                    if (string.IsNullOrEmpty(constraint.axisNodeSet))
                    {
                        Debug.LogWarning($"Rear wheel constraint on {constraint.name} has no axisNodeSet defined. Motor won't work!");
                    }
                }
            }
        }
    }

    private bool IsRearWheel(Transform wheelTransform)
    {
        Vector3 localPos = transform.InverseTransformPoint(wheelTransform.position);
        return localPos.z < 0; // Rear wheels are behind center
    }

    private void InitializeSteering()
    {
        // Find steering beams in the solver
        if (solver.beams == null)
        {
            Debug.LogError("No beams found in solver!");
            return;
        }

        Truss frameTruss = frame.GetTrussAsset();
        if (frameTruss == null)
        {
            Debug.LogError("No Truss asset assigned to frame!");
            return;
        }

        // Get beam indices for steering (single index per link set)
        int[] steerR0Indices = frameTruss.GetLinkSetIndices("Steer_R0");
        int[] steerR1Indices = frameTruss.GetLinkSetIndices("Steer_R1");
        int[] steerL0Indices = frameTruss.GetLinkSetIndices("Steer_L0");
        int[] steerL1Indices = frameTruss.GetLinkSetIndices("Steer_L1");

        if (steerR0Indices == null || steerR0Indices.Length == 0)
        {
            Debug.LogError("Steer_R0 link set not found!");
            return;
        }
        if (steerR1Indices == null || steerR1Indices.Length == 0)
        {
            Debug.LogError("Steer_R1 link set not found!");
            return;
        }
        if (steerL0Indices == null || steerL0Indices.Length == 0)
        {
            Debug.LogError("Steer_L0 link set not found!");
            return;
        }
        if (steerL1Indices == null || steerL1Indices.Length == 0)
        {
            Debug.LogError("Steer_L1 link set not found!");
            return;
        }

        steerR0Beam = solver.beams[steerR0Indices[0]];
        steerR1Beam = solver.beams[steerR1Indices[0]];
        steerL0Beam = solver.beams[steerL0Indices[0]];
        steerL1Beam = solver.beams[steerL1Indices[0]];

        // Store original lengths
        originalSteerR0Length = steerR0Beam.restLength;
        originalSteerR1Length = steerR1Beam.restLength;
        originalSteerL0Length = steerL0Beam.restLength;
        originalSteerL1Length = steerL1Beam.restLength;
    }
    private void FixedUpdate()
    {
        float deltaTime = Time.fixedDeltaTime;
        UpdateSteering(deltaTime);
        UpdateThrottle(deltaTime);
        foreach (WheelData wheel in wheels) wheel.UpdateRotation(deltaTime);

        Vector3 velocity = (transform.position - lastPosition) / deltaTime;
        forwardSpeed = Vector3.Dot(velocity, transform.forward);
        lastPosition = transform.position;
    }

    private void LateUpdate()
    {
        UpdateCamera();
    }

    private void UpdateSteering(float deltaTime)
    {
        if (Keyboard.current == null)
            return;

        float horizontalInput = 0f;
        if (Keyboard.current.aKey.isPressed) horizontalInput = -1f;
        if (Keyboard.current.dKey.isPressed) horizontalInput = 1f;

        float targetSteering = horizontalInput * maxSteeringAngle;
        float steeringDelta = targetSteering - currentSteeringAngle;
        float maxDelta = steeringSpeed * deltaTime;

        currentSteeringAngle += Mathf.Clamp(steeringDelta, -maxDelta, maxDelta);

        // Apply Ackermann steering geometry
        ApplyAckermannSteering(currentSteeringAngle * Mathf.Deg2Rad);
    }

    private void ApplyAckermannSteering(float steeringAngle)
    {
        if (steerR0Beam == null || steerR1Beam == null ||
            steerL0Beam == null || steerL1Beam == null)
            return;

        float innerAngle, outerAngle;

        if (steeringAngle >= 0f)
        {
            outerAngle = steeringAngle;
            innerAngle = Mathf.Atan2(baseLength, baseLength / Mathf.Tan(steeringAngle) + baseWidth);
        }
        else
        {
            outerAngle = -Mathf.Atan2(baseLength, baseLength / Mathf.Tan(-steeringAngle) + baseWidth);
            innerAngle = steeringAngle;
        }

        // Calculate new beam lengths using steering geometry
        float steerHand = originalSteerR0Length / 1.4142135f;

        float lengthR0 = steerHand * Mathf.Sin((Mathf.PI / 2f + outerAngle) * 0.5f) * 2f;
        float lengthR1 = steerHand * Mathf.Sin((Mathf.PI / 2f - outerAngle) * 0.5f) * 2f;
        float lengthL0 = steerHand * Mathf.Sin((Mathf.PI / 2f - innerAngle) * 0.5f) * 2f;
        float lengthL1 = steerHand * Mathf.Sin((Mathf.PI / 2f + innerAngle) * 0.5f) * 2f;

        // Apply with smoothing
        float smoothing = 0.1f;
        steerR0Beam.restLength = Mathf.Lerp(steerR0Beam.restLength, lengthR0, smoothing);
        steerR1Beam.restLength = Mathf.Lerp(steerR1Beam.restLength, lengthR1, smoothing);
        steerL0Beam.restLength = Mathf.Lerp(steerL0Beam.restLength, lengthL0, smoothing);
        steerL1Beam.restLength = Mathf.Lerp(steerL1Beam.restLength, lengthL1, smoothing);
    }

    private void UpdateThrottle(float dt)
    {
        if (Keyboard.current == null)
            return;

        float verticalInput = 0f;
        if (Keyboard.current.wKey.isPressed) verticalInput = 1f;
        if (Keyboard.current.sKey.isPressed) verticalInput = -1f;

        // Logic to switch to Reverse: If nearly stopped and holding down
        if (Mathf.Abs(forwardSpeed) < 0.5f)
        {
            if (verticalInput < -0.1f) currentGear = -1;
            else if (verticalInput > 0.1f) currentGear = 0;
        }

        throttle = Mathf.Clamp01(Mathf.Abs(verticalInput));
        UpdateGearing();

        float gearRatio = (currentGear == -1) ? reverseGearRatio * finalDriveRatio : gearRatios[Mathf.Max(0, currentGear)] * finalDriveRatio;

        engineRPM = Mathf.Lerp(900f, maxEngineRPM, throttle);
        float engineAngularVelocity = (engineRPM * 2f * Mathf.PI) / 60f;
        float wheelTargetAngularSpeed = engineAngularVelocity / Mathf.Max(gearRatio, 0.01f);
        engineTorque = maxEngineTorque * (1f - Mathf.Abs((engineRPM / maxEngineRPM) - 0.5f) * 0.8f) * throttle;
        float wheelTorque = engineTorque * gearRatio;

        // FIX: Pre-calculate driven wheels for even torque distribution
        int drivenWheelCount = 0;
        foreach (var w in wheels) if (w.isRearWheel && w.constraint != null) drivenWheelCount++;
        float wheelTorqueShare = wheelTorque / Mathf.Max(drivenWheelCount, 1);

        float motorDirection = (currentGear == -1) ? 1f : -1f;

        foreach (WheelData wheel in wheels)
        {
            if (!wheel.isRearWheel || wheel.constraint == null)
            {
                DisableConstraintMotor(wheel.constraint);
                continue;
            }

            wheel.motorAngularVelocity = wheelTargetAngularSpeed;
            if (throttle > 0.01f)
                SetConstraintMotor(wheel.constraint, motorDirection * wheelTargetAngularSpeed, wheelTorqueShare);
            else
                DisableConstraintMotor(wheel.constraint);
        }
    }
    private void UpdateGearing()
    {
        if (currentGear == -1) return; // Stay in Reverse gear logic

        float speedKmh = Mathf.Abs(forwardSpeed * 3.6f);
        float[] shiftUpSpeeds = { 15f, 30f, 50f, 75f };
        float[] shiftDownSpeeds = { 10f, 25f, 45f, 65f };

        if (currentGear < gearRatios.Length - 1 && speedKmh > shiftUpSpeeds[currentGear]) currentGear++;
        else if (currentGear > 0 && speedKmh < shiftDownSpeeds[currentGear - 1]) currentGear--;
    }

    private void UpdateCamera()
    {
        if (Mouse.current == null)
            return;

        // Mouse look
        Vector2 mouseDelta = Mouse.current.delta.ReadValue();
        float mouseX = mouseDelta.x * 2f * 0.1f;
        float mouseY = mouseDelta.y * 2f * 0.1f;

        if (Mathf.Abs(mouseX) > 0.01f || Mathf.Abs(mouseY) > 0.01f)
        {
            targetYaw += mouseX;
            targetPitch -= mouseY;
            targetPitch = Mathf.Clamp(targetPitch, -80f, 80f);
        }

        // Zoom
        bool isShiftPressed = Keyboard.current != null && (Keyboard.current.leftShiftKey.isPressed || Keyboard.current.rightShiftKey.isPressed);

        if (!isShiftPressed)
        {
            Vector2 scrollDelta = Mouse.current.scroll.ReadValue();
            cameraDistance = Mathf.Clamp(cameraDistance - scrollDelta.y * 0.003f, 3f, 30f);
        }

        // Smooth camera movement
        float smoothTime = 0.1f;
        yaw = Mathf.LerpAngle(yaw, targetYaw, smoothTime);
        pitch = Mathf.Lerp(pitch, targetPitch, smoothTime);

        UpdateCameraPosition();

        // Time scale controls
        if (isShiftPressed)
        {
            Vector2 scrollDelta = Mouse.current.scroll.ReadValue();
            float scrollY = scrollDelta.y;
            if (scrollY < 0f)
            {
                Time.timeScale = Mathf.Max(0f, Time.timeScale * 0.5f);
            }
            else if (scrollY > 0f)
            {
                Time.timeScale = Mathf.Min(1f, Time.timeScale * 2f);
            }
        }

        if (Keyboard.current != null && Keyboard.current.zKey.wasPressedThisFrame)
        {
            Time.timeScale = Time.timeScale == 1f ? 0f : 1f;
        }
    }

    private void UpdateCameraPosition()
    {
        if (targetCamera == null || followTarget == null)
            return;

        // Where the camera looks
        Vector3 lookAtPosition =
            followTarget.position +
            followTarget.forward * 0.5f +
            Vector3.up * 0.5f;

        // Camera rotation from mouse
        Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);

        // Camera offset (behind target)
        Vector3 desiredPosition =
            lookAtPosition +
            rotation * Vector3.back * cameraDistance;

        // Ground / obstacle avoidance
        if (Physics.Linecast(
            lookAtPosition,
            desiredPosition,
            out RaycastHit hit,
            cameraGroundMask,
            QueryTriggerInteraction.Ignore))
        {
            desiredPosition = hit.point + hit.normal * 0.3f;
        }

        targetCamera.transform.position = desiredPosition;
        targetCamera.transform.LookAt(lookAtPosition);
    }


    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 250, 220));
        GUILayout.Label($"Speed: {forwardSpeed * 3.6f:F1} km/h");
        GUILayout.Label($"RPM: {engineRPM:F0}");
        string gearDisplay = (currentGear == -1) ? "R" : (currentGear + 1).ToString();
        GUILayout.Label($"Gear: {gearDisplay}");
        GUILayout.Label($"Torque: {engineTorque:F1} Nm");
        GUILayout.EndArea();
    }

    private void OnApplicationFocus(bool focus)
    {
        if (!Application.isEditor)
        {
            Cursor.lockState = focus ? CursorLockMode.Locked : CursorLockMode.None;
        }
    }
}