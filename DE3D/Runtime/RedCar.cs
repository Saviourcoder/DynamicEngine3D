using UnityEngine;
using System;
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

    [Header("Engine")]
    public float maxEngineTorque = 10f;
    public float maxEngineRPM = 1000f;
    public float[] gearRatios = { 6.6f, 4f, 2.8f, 2.1f, 1.6f };
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

    // Engine
    private float throttle;
    private int currentGear = 1;
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
        public float lastRotation;
        public float angularVelocity;
        public float motorAngularVelocity; // rad/s

        public void UpdateRotation(float deltaTime)
        {
            if (wheelBody != null && wheelBody.solver != null && wheelBody.solver.nodeManager.Nodes.Count > 0)
            {
                // Simple rotation estimation (for RPM calculation)
                angularVelocity = (currentRotation - lastRotation) / deltaTime;
                lastRotation = currentRotation;
                currentRotation += angularVelocity * deltaTime;
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
            if (constraint.attachedBody != null && constraint.attachedBody != frame)
            {
                WheelData wheelData = new WheelData
                {
                    constraint = constraint,
                    wheelBody = constraint.attachedBody,
                    isRearWheel = IsRearWheel(constraint.transform),
                    currentRotation = 0f,
                    lastRotation = 0f,
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
                    else
                    {
                        Debug.Log($"Rear wheel initialized: {constraint.name} with axis set: {constraint.axisNodeSet}");
                    }
                }
            }
        }

        Debug.Log($"Initialized {wheels.Count} wheels ({wheels.FindAll(w => w.isRearWheel).Count} rear wheels)");
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

        if (steerR0Indices.Length == 0 || steerR1Indices.Length == 0 ||
            steerL0Indices.Length == 0 || steerL1Indices.Length == 0)
        {
            Debug.LogError("Could not find all steering link sets!");
            return;
        }

        // Get beams directly by index (first element of each link set)
        int steerR0Index = steerR0Indices[0];
        int steerR1Index = steerR1Indices[0];
        int steerL0Index = steerL0Indices[0];
        int steerL1Index = steerL1Indices[0];

        // Access beams directly from solver
        steerR0Beam = solver.beams[steerR0Index];
        steerR1Beam = solver.beams[steerR1Index];
        steerL0Beam = solver.beams[steerL0Index];
        steerL1Beam = solver.beams[steerL1Index];

        if (steerR0Beam == null || steerR1Beam == null ||
            steerL0Beam == null || steerL1Beam == null)
        {
            Debug.LogError("Could not find all steering beams!");
            return;
        }

        // Store original lengths
        originalSteerR0Length = steerR0Beam.restLength;
        originalSteerR1Length = steerR1Beam.restLength;
        originalSteerL0Length = steerL0Beam.restLength;
        originalSteerL1Length = steerL1Beam.restLength;

        Debug.Log("Steering initialized successfully");
    }
    private void FixedUpdate()
    {
        float deltaTime = Time.fixedDeltaTime;

        // Update inputs
        UpdateSteering(deltaTime);
        UpdateThrottle(deltaTime);

        // Update wheel rotations for RPM calculation
        foreach (WheelData wheel in wheels)
        {
            wheel.UpdateRotation(deltaTime);
        }

        // Calculate forward speed
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
        float targetSteering = Input.GetAxis("Horizontal") * maxSteeringAngle;
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
        throttle = Mathf.Clamp01(Input.GetAxis("Vertical"));

        // Engine parameters
        float idleRPM = 900f;
        float maxRPM = maxEngineRPM;

        // Target engine RPM from throttle
        float targetEngineRPM = Mathf.Lerp(idleRPM, maxRPM, throttle);

        // Update engine RPM display
        engineRPM = targetEngineRPM;

        // Simple automatic gearing based on speed
        UpdateGearing();

        // Calculate gear ratio
        float gearRatio = gearRatios[currentGear] * finalDriveRatio;

        // Convert engine RPM to wheel angular velocity (rad/s)
        float engineAngularVelocity = (targetEngineRPM * 2f * Mathf.PI) / 60f;
        float wheelTargetAngularSpeed = engineAngularVelocity / Mathf.Max(gearRatio, 0.01f);

        // Torque curve (simple flat curve with peak at mid-range)
        float rpmNorm = targetEngineRPM / maxRPM;
        float torqueFactor = Mathf.Clamp01(1f - Mathf.Abs(rpmNorm - 0.5f) * 0.8f);
        engineTorque = maxEngineTorque * torqueFactor * throttle;

        // Apply torque through gearing
        float wheelTorque = engineTorque * gearRatio;

        // Apply motor to rear wheels
        int drivenWheelCount = 0;
        foreach (WheelData wheel in wheels)
        {
            if (!wheel.isRearWheel)
            {
                // Disable motor on front wheels
                DisableConstraintMotor(wheel.constraint);
                continue;
            }

            if (wheel.constraint == null) continue;
            drivenWheelCount++;

            // Store motor angular velocity for display
            wheel.motorAngularVelocity = wheelTargetAngularSpeed;

            // Distribute torque among driven wheels
            float wheelTorqueShare = wheelTorque / Mathf.Max(drivenWheelCount, 1);

            // Apply motor to constraint
            if (throttle > 0.01f)
            {
                SetConstraintMotor(wheel.constraint, wheelTargetAngularSpeed, wheelTorqueShare);
            }
            else
            {
                // Disable motor when not throttling
                DisableConstraintMotor(wheel.constraint);
            }
        }

        if (drivenWheelCount == 0)
        {
            Debug.LogWarning("No driven wheels found!");
        }
    }

    private void UpdateGearing()
    {
        // Simple automatic transmission based on speed
        float speedKmh = Mathf.Abs(forwardSpeed * 3.6f);

        // Shift up thresholds
        float[] shiftUpSpeeds = { 15f, 30f, 50f, 75f };
        // Shift down thresholds (with hysteresis)
        float[] shiftDownSpeeds = { 10f, 25f, 45f, 65f };

        // Shift up
        if (currentGear < gearRatios.Length - 1 && speedKmh > shiftUpSpeeds[currentGear])
        {
            currentGear++;
        }
        // Shift down
        else if (currentGear > 0 && speedKmh < shiftDownSpeeds[currentGear - 1])
        {
            currentGear--;
        }
    }

    private void UpdateCamera()
    {
        // Mouse look
        float mouseX = Input.GetAxis("Mouse X") * 2f;
        float mouseY = Input.GetAxis("Mouse Y") * 2f;

        if (Mathf.Abs(mouseX) > 0.01f || Mathf.Abs(mouseY) > 0.01f)
        {
            targetYaw += mouseX;
            targetPitch -= mouseY;
            targetPitch = Mathf.Clamp(targetPitch, -80f, 80f);
        }

        // Zoom
        if (!Input.GetKey(KeyCode.LeftShift))
        {
            cameraDistance = Mathf.Clamp(cameraDistance - Input.mouseScrollDelta.y * 0.3f, 3f, 30f);
        }

        // Smooth camera movement
        float smoothTime = 0.1f;
        yaw = Mathf.LerpAngle(yaw, targetYaw, smoothTime);
        pitch = Mathf.Lerp(pitch, targetPitch, smoothTime);

        UpdateCameraPosition();

        // Time scale controls
        if (Input.GetKey(KeyCode.LeftShift))
        {
            float scrollY = Input.mouseScrollDelta.y;
            if (scrollY < 0f)
            {
                Time.timeScale = Mathf.Max(0f, Time.timeScale * 0.5f);
            }
            else if (scrollY > 0f)
            {
                Time.timeScale = Mathf.Min(1f, Time.timeScale * 2f);
            }
        }

        if (Input.GetKeyDown(KeyCode.Z))
        {
            Time.timeScale = Time.timeScale == 1f ? 0f : 1f;
        }
    }

    private void UpdateCameraPosition()
    {
        if (targetCamera == null) return;

        Vector3 lookAtPosition = transform.position + transform.forward * 0.5f + Vector3.up * 0.5f;
        Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);
        Vector3 cameraPosition = lookAtPosition + rotation * Vector3.back * cameraDistance;

        if (Physics.Linecast(
                lookAtPosition,
                cameraPosition,
                out RaycastHit hit,
                cameraGroundMask,
                QueryTriggerInteraction.Ignore))
        {
            cameraPosition = hit.point + hit.normal * 0.3f;
        }

        targetCamera.transform.position = cameraPosition;
        targetCamera.transform.LookAt(lookAtPosition);
    }

    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 250, 220));
        GUILayout.Label($"Speed: {forwardSpeed * 3.6f:F1} km/h");
        GUILayout.Label($"RPM: {engineRPM:F0}");
        GUILayout.Label($"Gear: {currentGear + 1}/{gearRatios.Length}");
        GUILayout.Label($"Torque: {engineTorque:F1} Nm");
        GUILayout.Label($"Throttle: {throttle * 100:F0}%");
        GUILayout.Label($"Steering: {currentSteeringAngle:F1}°");

        // Show rear wheel motor info
        int rearWheelCount = 0;
        foreach (var wheel in wheels)
        {
            if (wheel.isRearWheel)
            {
                rearWheelCount++;
                float wheelRPM = (wheel.motorAngularVelocity * 60f) / (2f * Mathf.PI);
                GUILayout.Label($"Wheel {rearWheelCount} RPM: {wheelRPM:F0}");
            }
        }

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