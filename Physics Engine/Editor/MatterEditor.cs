#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

namespace DynamicEngine
{
    [CustomEditor(typeof(Matter))]
    public class MatterEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            Matter matter = (Matter)target;
            
            EditorGUILayout.HelpBox("Define friction and physics properties for soft-body materials.", MessageType.Info);
            
            DrawDefaultInspector();
        }
    }
}
#endif