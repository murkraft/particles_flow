using System;
using NUnit.Framework;
using UnityEngine;

namespace Assets.Scripts.FX
{
    [Serializable]
    public class Spline
    {
        private const int SamplesPerSegment = 10;

        public Spline(Vector3[] points, Quaternion[] rotations, float[] tensions)
        {
            Assert.AreEqual(points.Length, rotations.Length);
            Assert.AreEqual(points.Length, tensions.Length);

            _nodes = new Node[points.Length];
            for (int i = 0; i < points.Length; i++)
            {
                _nodes[i] = new Node {Point = points[i], Rotation = rotations[i], Tension = tensions[i]};
            }

            if (_nodes.Length >= 2)
            {
                float distance = 0;
                float step = 1f/SamplesPerSegment;
                Keyframe[] keys = new Keyframe[SamplesPerSegment + 1];
                Vector3 prevPt = GetHermite(0, 0, 0);
                for (int i = 0; i < _nodes.Length - 1; ++i)
                {
                    keys[0] = new Keyframe(distance, 0);
                    for (int j = 1; j <= SamplesPerSegment; j++)
                    {
                        var t = j*step;
                        var pt = GetHermite(i, t, 0);
                        distance += (pt - prevPt).magnitude;
                        keys[j] = new Keyframe(distance, t);
                        prevPt = pt;
                    }
                    var ease = new AnimationCurve(keys);
                    for (int k = 0; k < ease.length; ++k)
                        ease.SmoothTangents(k, 1);
                    _nodes[i].Ease = ease;
                }
                _length = distance;
            }
        }

        public float Length { get { return _length; } }

        public int NodesCount { get { return _nodes != null ? _nodes.Length : 0; } }

        public Vector3 Point(float param, float divergence = 0)
        {
            float t;
            var nodeIdx = FindNode(param, out t);
            if (nodeIdx == -1)
                return Vector3.zero;
            return GetHermite(nodeIdx, t, divergence);
        }

        public Vector3 Tangent(float param)
        {
            float t;
            var nodeIdx = FindNode(param, out t);
            if (nodeIdx == -1)
                return Vector3.zero;
            return GetHermiteDerivative(nodeIdx, t).normalized;
        }

        public Vector3 Normal(float param)
        {
            float t;
            var nodeIdx = FindNode(param, out t);
            if (nodeIdx == -1)
                return Vector3.zero;
            return GetHermiteNormal(nodeIdx, t, param);
        }

        public void Frame(float param, out Vector3 point, out Vector3 tangent, out Vector3 normal)
        {
            Frame(param, 0, out point, out tangent, out normal);
        }

        public void Frame(float param, float divergence, out Vector3 point, out Vector3 tangent, out Vector3 normal)
        {
            point = tangent = normal = Vector3.zero;
            float t;
            var nodeIdx = FindNode(param, out t);
            if (nodeIdx == -1)
                return;
            point = GetHermite(nodeIdx, t, divergence);
            tangent = GetHermiteDerivative(nodeIdx, t).normalized;
            normal = GetHermiteNormal(nodeIdx, tangent, t, param);
        }

        public float GetNodeParam(int idx)
        {
            if (_nodes.Length < 2)
                return 0;
            return (idx < _nodes.Length - 1 ? _nodes[idx].DistanceBgn : _nodes[idx - 1].DistanceEnd)/_length;
        }

        private int FindNode(float param, out float t)
        {
            t = 0;

            if (_nodes == null || _nodes.Length == 0)
                return -1;

            if (_nodes.Length == 1)
                return 0;

            var distance = Mathf.Clamp(_length*param, 0, _length);
            for (int i = 0; i < _nodes.Length - 1; i++)
            {
                var ease = _nodes[i].Ease;
                var nodeDistance = ease.keys[ease.length - 1].time;
                if (distance <= nodeDistance)
                {
                    t = ease.Evaluate(distance);
                    return i;
                }
            }
            t = 1;
            return _nodes.Length - 1;
        }

        private Vector3 GetHermite(int idxFirstPoint, float t, float divergence)
        {
            Vector3 P1 = _nodes[idxFirstPoint].Point + _nodes[idxFirstPoint].Normal*divergence;
            Vector3 T1 = _nodes[idxFirstPoint].TangentWithTension;
            if (idxFirstPoint >= _nodes.Length)
                return P1;

            Vector3 P2 = _nodes[idxFirstPoint + 1].Point + _nodes[idxFirstPoint + 1].Normal*divergence;
            Vector3 T2 = _nodes[idxFirstPoint + 1].TangentWithTension;

            float t2 = t*t;
            float t3 = t2*t;
            float blend1 = 2*t3 - 3*t2 + 1;
            float blend2 = -2*t3 + 3*t2;
            float blend3 = t3 - 2*t2 + t;
            float blend4 = t3 - t2;

            return blend1*P1 + blend2*P2 + blend3*T1 + blend4*T2;
        }

        private Vector3 GetHermiteDerivative(int idxFirstPoint, float t)
        {
            Vector3 P1 = _nodes[idxFirstPoint].Point;
            Vector3 T1 = _nodes[idxFirstPoint].TangentWithTension;
            if (idxFirstPoint >= _nodes.Length)
                return T1;

            Vector3 P2 = _nodes[idxFirstPoint + 1].Point;
            Vector3 T2 = _nodes[idxFirstPoint + 1].TangentWithTension;

            float t2 = t*t;
            float blend1 = 6*t2 - 6*t;
            float blend2 = -6*t2 + 6*t;
            float blend3 = 3*t2 - 4*t + 1;
            float blend4 = 3*t2 - 2*t;

            return blend1*P1 + blend2*P2 + blend3*T1 + blend4*T2;
        }

        public Vector3 GetHermiteNormal(int idxFirstPoint, float localT, float globalT)
        {
            if (idxFirstPoint >= _nodes.Length)
                return _nodes[idxFirstPoint].Normal;
            var tan = GetHermiteDerivative(idxFirstPoint, localT).normalized;
            return GetHermiteNormal(idxFirstPoint, tan, localT, globalT);
        }

        public Vector3 GetHermiteNormal(int idxFirstPoint, Vector3 tan, float localT, float globalT)
        {
            if (idxFirstPoint >= _nodes.Length)
                return _nodes[idxFirstPoint].Normal;
            var distBgn = _nodes[idxFirstPoint].DistanceBgn;
            var distEnd = _nodes[idxFirstPoint].DistanceEnd;
            Quaternion Q1 = _nodes[idxFirstPoint].Rotation;
            Quaternion Q2 = _nodes[idxFirstPoint + 1].Rotation;
            var qatT = (globalT*_length - distBgn)/(distEnd - distBgn);
            var binorm = Quaternion.Slerp(Q1, Q2, qatT)*Vector3.right;
            return Vector3.Cross(binorm, tan).normalized;
        }

        [Serializable]
        private struct Node
        {
            public Vector3 Point;
            public float Tension;
            public Quaternion Rotation;
            public AnimationCurve Ease;
            public Vector3 Tangent { get { return Rotation*Vector3.forward; } }
            public Vector3 TangentWithTension { get { return Rotation*Vector3.forward*Tension; } }
            public Vector3 Normal { get { return Rotation*Vector3.up; } }
            public float DistanceBgn { get { return Ease.keys[0].time; } }
            public float DistanceEnd { get { return Ease.keys[Ease.keys.Length - 1].time; } }
        }

        [SerializeField] private readonly Node[] _nodes;
        [SerializeField] private readonly float _length;
    }
}
