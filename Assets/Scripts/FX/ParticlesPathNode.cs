using JetBrains.Annotations;
using UnityEditor;
#if UNITY_EDITOR
using UnityEngine;
#endif

namespace Assets.Scripts.FX
{
    public class ParticlesPathNode : MonoBehaviour
    {
        [SerializeField]
        private float _tension = 1;

        public Vector3 Point { get { return transform.position; } }
        public Quaternion Rotation { get { return transform.rotation; } }
        public float Tension { get { return _tension; } set { _tension = value; } }
        public Vector3 Tangent { get { return Rotation * Vector3.forward * _tension; } }
        public Vector3 Normal { get { return Rotation * Vector3.up; } }
        public bool HasChanged { get { return transform.hasChanged || _prevTension != _tension; } }

        public void ResetHasChanged()
        {
            transform.hasChanged = false;
            _prevTension = _tension;
        }

        [UsedImplicitly]
        private void Awake()
        {
            transform.hasChanged = true;
        }

        private float _prevTension;


#if UNITY_EDITOR
        public void OnDrawGizmos()
        {
            Handles.matrix = Matrix4x4.identity;
            Handles.color = Color.red;
            Handles.DrawDottedLine(Point, Point + Tangent, 2f);
            Handles.color = Color.green;
            Handles.DrawDottedLine(Point, Point + Normal, 2f);
            Handles.color = Color.white;
            Gizmos.DrawSphere(Point + Tangent, 0.15f);
        }
#endif
    }
}