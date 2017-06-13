using Assets.Scripts.FX;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof (ParticlesPathNode))]
public class ParticlasPathNodeEditor : Editor
{
    protected virtual void OnSceneGUI()
    {
        ParticlesPathNode example = (ParticlesPathNode) target;

        EditorGUI.BeginChangeCheck();
        Vector3 newTargetPosition = Handles.FreeMoveHandle(example.Point + example.Tangent, Quaternion.identity, 0.3f, Vector3.zero, Handles.SphereHandleCap);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(example, "Change Look At Target Position");
            var tangent = newTargetPosition - example.Point;
            var tension = tangent.magnitude;
            tangent /= tension;
            example.Tension = tension;
            var rot = Quaternion.LookRotation(tangent, example.Normal);
            example.transform.rotation = rot;
        }
    }
}
