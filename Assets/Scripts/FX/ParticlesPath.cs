using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Assets.Scripts.FX;
using JetBrains.Annotations;
using UnityEngine;
using UnityEngine.Assertions;
#if UNITY_EDITOR
using UnityEditor;

#endif

[ExecuteInEditMode]
public class ParticlesPath : MonoBehaviour
{
    [SerializeField] private ParticleSystem _particleSystem;
    [SerializeField] private ParticlesPathNode[] _nodes;
    [SerializeField] private float _tensionFactor = 10;
    [SerializeField] private float _divergenceFactor = 1;
    [SerializeField] private float _rotationFrequency = 1;
    [SerializeField] private DivergenceMode _divergenceMode = DivergenceMode.Normals;
    [SerializeField] private ParticleSystemCustomData _customDataChannel = ParticleSystemCustomData.Custom1;

    [SerializeField] [HideInInspector] private Spline _spline;
    [SerializeField] [HideInInspector] private Spline _splineMin;
    [SerializeField] [HideInInspector] private Spline _splineMax;

    [UsedImplicitly]
    private void Update()
    {
        UpdateSplineIfChanged();
    }

    [UsedImplicitly]
    private void LateUpdate()
    {
        UpdateParticles();
    }

    [UsedImplicitly]
    private void Awake()
    {
        _particleSystem = _particleSystem ?? GetComponent<ParticleSystem>();
        Assert.IsNotNull(_particleSystem);
    }

    private void UpdateParticles()
    {
        if (_particleSystem == null)
            return;

        if (_particles == null || _particles.Length < _particleSystem.main.maxParticles)
            _particles = new ParticleSystem.Particle[_particleSystem.main.maxParticles];

        if (_particlesData == null)
            _particlesData = new List<Vector4>(_particleSystem.main.maxParticles);

        var particlesCount = _particleSystem.GetParticles(_particles);
        _particleSystem.GetCustomParticleData(_particlesData, _customDataChannel);

        var divergenceFunction = _particleSystem.customData.GetVector(_customDataChannel, 1);

        float deltaTime = Time.deltaTime;
        for (int i = 0; i < particlesCount; i++)
        {
            var t = _particlesData[i].x;
            var random = _particles[i].randomSeed;
            var shift = _particlesData[i].z;
            var pos = Point(t, divergenceFunction, _divergenceFactor, random, shift);
            if (_particleSystem.main.simulationSpace == ParticleSystemSimulationSpace.Local)
                pos = _particleSystem.transform.InverseTransformPoint(pos);
            var prevPos = _particles[i].position;
            _particles[i].velocity = (pos - prevPos)/deltaTime;
        }
        _particleSystem.SetParticles(_particles, particlesCount);
    }

    [Conditional("UNITY_EDITOR")]
    private void UpdateSplineIfChanged()
    {
        if (_nodes == null)
        {
            _spline = null;
        }
        else
        {
            var changed = _spline == null;
            changed |= _oldTensionFactor != _tensionFactor;
            changed |= _oldDivergenceFactor != _divergenceFactor;
            if (!changed)
            {
                int count = 0;
                for (int i = 0; i < _nodes.Length; i++)
                    if (_nodes[i] != null)
                    {
                        count++;
                        if (_nodes[i].HasChanged)
                        {
                            _nodes[i].ResetHasChanged();
                            changed = true;
                        }
                    }
                changed |= count != _spline.NodesCount;
            }

            if (changed)
            {
                UpdateSpline();
                _oldTensionFactor = _tensionFactor;
                _oldDivergenceFactor = _divergenceFactor;
            }
        }
    }

    private void UpdateSpline()
    {
        var nodes = _nodes.Where(x => x != null).ToArray();

        var points = new Vector3[nodes.Length];
        var rotations = new Quaternion[nodes.Length];
        var tensions = new float[nodes.Length];
        for (int i = 0; i < nodes.Length; i++)
        {
            points[i] = nodes[i].Point;
            rotations[i] = nodes[i].Rotation;
            tensions[i] = nodes[i].Tension*_tensionFactor;
        }

        _spline = new Spline(points, rotations, tensions);

        var divergence = _particleSystem.customData.GetVector(_customDataChannel, 1);

        for (int i = 0; i < nodes.Length; i++)
        {
            var param = _spline.GetNodeParam(i);
            points[i] = nodes[i].Point + nodes[i].Normal*(divergence.Evaluate(param, 0)*_divergenceFactor);
        }
        _splineMin = new Spline(points, rotations, tensions);

        for (int i = 0; i < nodes.Length; i++)
        {
            var param = _spline.GetNodeParam(i);
            points[i] = nodes[i].Point + nodes[i].Normal*(divergence.Evaluate(param, 1)*_divergenceFactor);
        }
        _splineMax = new Spline(points, rotations, tensions);
    }

    private Vector3 Point(float t, ParticleSystem.MinMaxCurve divergence, float divergenceFactor, uint random, float shift = 0)
    {
        Vector3 point, tangent, normal, origin;
        float div;
        switch (_divergenceMode)
        {
            case DivergenceMode.Normals:
                _spline.Frame(t, out point, out tangent, out normal);
                div = divergence.Evaluate(t, (float) random/uint.MaxValue)*divergenceFactor;
                point += Roatation(tangent, t + shift)*normal*div;
                break;
            case DivergenceMode.Parallels:
                _spline.Frame(t, out origin, out tangent, out normal);
                div = divergence.Evaluate(t, (float) random/uint.MaxValue)*divergenceFactor;
                point = _spline.Point(t, div);
                point = origin + Roatation(tangent, t + shift)*(point - origin);
                break;
            case DivergenceMode.Multispline:
                _spline.Frame(t, out origin, out tangent, out normal);
                div = (float) random/uint.MaxValue;
                point = Vector3.Lerp(_splineMin.Point(t), _splineMax.Point(t), div);
                point = (point - origin)*Mathf.Sign(divergenceFactor) + origin;
                point = origin + Roatation(tangent, t + shift)*(point - origin);
                break;
            default:
                throw new NotImplementedException("_divergenceMode=" + _divergenceMode);
        }
        return point;
    }

    private Quaternion Roatation(Vector3 tangent, float t)
    {
        float angle = _rotationFrequency*Mathf.PI*2*t*Mathf.Rad2Deg;
        return Quaternion.AngleAxis(angle, tangent);
    }


    [ContextMenu("Gather Child Nodes")]
    private void GatherChildNodes()
    {
        _nodes = transform.GetComponentsInChildren<ParticlesPathNode>();
    }


    [ContextMenu("Recalculate path")]
    private void RecalculateSpline()
    {
        UpdateSpline();
    }

    private enum DivergenceMode
    {
        Normals,
        Parallels,
        Multispline
    }

    private float _oldTensionFactor;
    private float _oldDivergenceFactor;
    private ParticleSystem.Particle[] _particles;
    private List<Vector4> _particlesData;


#if UNITY_EDITOR
    private const int _gizmoSegmentsCount = 100;
    private readonly Vector3[] _gizmoPoints = new Vector3[(_gizmoSegmentsCount + 1)*2];

    [UsedImplicitly]
    private void OnDrawGizmos()
    {
        if (_spline != null)
        {
            float step = 1f/_gizmoSegmentsCount;
            Handles.color = Color.cyan;
            for (int j = 0; j <= _gizmoSegmentsCount; j++)
            {
                _gizmoPoints[j*2] = _spline.Point(step*j);
                _gizmoPoints[j*2 + 1] = _spline.Point(step*(j + 1));
            }
            Handles.DrawDottedLines(_gizmoPoints, 3);
/*
            if (_divergenceMode == DivergenceMode.Multispline)
            {
                for (int j = 0; j <= _gizmoSegmentsCount; j++)
                {
                    _gizmoPoints[j*2] = _splineMax.Point(step*j);
                    _gizmoPoints[j*2 + 1] = _splineMax.Point(step*(j + 1));
                }
                Handles.DrawDottedLines(_gizmoPoints, 3);
                for (int j = 0; j <= _gizmoSegmentsCount; j++)
                {
                    _gizmoPoints[j*2] = _splineMin.Point(step*j);
                    _gizmoPoints[j*2 + 1] = _splineMin.Point(step*(j + 1));
                }
                Handles.DrawDottedLines(_gizmoPoints, 3);
            }
*/
            var divergence = _particleSystem.customData.GetVector(_customDataChannel, 1);
            Handles.color = Color.green;
            for (int j = 0; j <= _gizmoSegmentsCount; j++)
            {
                float t = step*j;
                _gizmoPoints[j*2] = Point(t, divergence, _divergenceFactor, 0, -t);
                _gizmoPoints[j*2 + 1] = Point(t, divergence, _divergenceFactor, uint.MaxValue, -t);
            }
            Handles.DrawLines(_gizmoPoints);
            for (int j = 0; j <= _gizmoSegmentsCount; j++)
            {
                float t = step*j;
                _gizmoPoints[j*2] = Point(t, divergence, -_divergenceFactor, 0, -t);
                _gizmoPoints[j*2 + 1] = Point(t, divergence, -_divergenceFactor, uint.MaxValue, -t);
            }
            Handles.DrawLines(_gizmoPoints);
        }
    }
#endif
}
