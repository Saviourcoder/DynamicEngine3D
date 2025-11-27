using UnityEngine;

public class Spinner : MonoBehaviour
{
    [Header("Spinner Settings")]
    [SerializeField, Range(0f, 5000f)] private float rotationSpeed = 1000f; // RPM, adjustable in inspector
    [SerializeField] private Axis rotationAxis = Axis.Y; // Which axis to rotate around

    private float angularVelocity; // Radians per second

    private void Start()
    {
        // Convert RPM to radians per second
        angularVelocity = (rotationSpeed * 2f * Mathf.PI) / 60f;
    }

    private void Update()
    {
        // Continuous rotation like in BeamNG spinners
        float deltaTime = Time.deltaTime;
        Vector3 rotationVector = Vector3.zero;

        switch (rotationAxis)
        {
            case Axis.X:
                rotationVector = Vector3.right * angularVelocity * deltaTime;
                break;
            case Axis.Y:
                rotationVector = Vector3.up * angularVelocity * deltaTime;
                break;
            case Axis.Z:
                rotationVector = Vector3.forward * angularVelocity * deltaTime;
                break;
        }

        transform.Rotate(rotationVector, Space.Self);
    }

    public enum Axis
    {
        X,
        Y,
        Z
    }
}