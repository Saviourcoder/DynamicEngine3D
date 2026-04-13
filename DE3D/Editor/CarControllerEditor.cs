#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(CarController))]
public class CarControllerEditor : Editor
{
    private float _lastRepaintTime;
    private const float RepaintInterval = 0f;

    public override bool RequiresConstantRepaint() => false;

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        if (Application.isPlaying &&
            Time.realtimeSinceStartup - _lastRepaintTime >= RepaintInterval)
        {
            _lastRepaintTime = Time.realtimeSinceStartup;
            Repaint();
        }
    }
}
#endif