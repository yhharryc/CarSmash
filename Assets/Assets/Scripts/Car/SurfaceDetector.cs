using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#region Surface Detector

public struct SurfaceBlend
{
    public bool hasGround;
    public Vector3 avgNormal;
    public float longAccelMult;
    public float brakeMult;
    public float sideFriction;
    public float forwardDrag;
    public float steerResponseMult;
    public float slopeSlide;
    public float topSpeedMult;

    public static SurfaceBlend Lerp(in SurfaceBlend a, in SurfaceBlend b, float t)
    {
        t = Mathf.Clamp01(t);
        return new SurfaceBlend
        {
            hasGround = t < 0.5f ? a.hasGround : b.hasGround,
            avgNormal = Vector3.Slerp(a.avgNormal, b.avgNormal, t),
            longAccelMult = Mathf.Lerp(a.longAccelMult, b.longAccelMult, t),
            brakeMult = Mathf.Lerp(a.brakeMult, b.brakeMult, t),
            sideFriction = Mathf.Lerp(a.sideFriction, b.sideFriction, t),
            forwardDrag = Mathf.Lerp(a.forwardDrag, b.forwardDrag, t),
            steerResponseMult = Mathf.Lerp(a.steerResponseMult, b.steerResponseMult, t),
            slopeSlide = Mathf.Lerp(a.slopeSlide, b.slopeSlide, t),
            topSpeedMult = Mathf.Lerp(a.topSpeedMult, b.topSpeedMult, t)
        };
    }
}

[DefaultExecutionOrder(-10)]
public class SurfaceDetector : MonoBehaviour
{
    [Header("Sampling")]
    [Tooltip("Points used for ground sampling (e.g., corners of the car). If empty, uses the object's position.")]
    public Transform[] samplePoints;

    [Tooltip("LayerMask for ground surfaces.")]
    public LayerMask groundMask = ~0;

    [Tooltip("Raycast length from each sample point.")]
    public float rayLength = 2.0f;

    [Tooltip("Blend speed when surface changes (higher = snappier).")]
    public float blendLerp = 12f;

    [Header("Profiles")]
    public SurfaceProfile defaultProfile;

    public SurfaceBlend Current => current;
    private SurfaceBlend current;

    void Reset()
    {
        rayLength = 2f;
        blendLerp = 12f;
    }

    void Awake()
    {
        current = CreateFromProfile(defaultProfile, true, Vector3.up);
    }

    void FixedUpdate()
    {
        SurfaceBlend target = Sample();
        current = SurfaceBlend.Lerp(current, target, 1f - Mathf.Exp(-blendLerp * Time.fixedDeltaTime));
    }

    SurfaceBlend Sample()
    {
        if (samplePoints == null || samplePoints.Length == 0)
        {
            return SampleAtPoint(transform.position);
        }

        // Weighted average by hit proximity (closer hit = higher weight)
        float totalW = 0f;
        Vector3 n = Vector3.zero;
        // Accumulate profile scalars
        float longAccel = 0, brake = 0, sideFric = 0, fdrag = 0, steer = 0, slide = 0, top = 0;
        bool anyHit = false;

        foreach (var p in samplePoints)
        {
            var s = SampleAtPoint(p.position);
            if (!s.hasGround) continue;
            anyHit = true;
            float w = 1f; // Could be based on distance; use 1 for stability.
            totalW += w;
            n += s.avgNormal * w;
            longAccel += s.longAccelMult * w;
            brake += s.brakeMult * w;
            sideFric += s.sideFriction * w;
            fdrag += s.forwardDrag * w;
            steer += s.steerResponseMult * w;
            slide += s.slopeSlide * w;
            top += s.topSpeedMult * w;
        }

        if (!anyHit)
        {
            return CreateFromProfile(defaultProfile, false, Vector3.up);
        }

        float inv = 1f / Mathf.Max(0.0001f, totalW);
        return new SurfaceBlend
        {
            hasGround = true,
            avgNormal = n.normalized,
            longAccelMult = longAccel * inv,
            brakeMult = brake * inv,
            sideFriction = sideFric * inv,
            forwardDrag = fdrag * inv,
            steerResponseMult = steer * inv,
            slopeSlide = slide * inv,
            topSpeedMult = top * inv,
        };
    }

    SurfaceBlend SampleAtPoint(Vector3 start)
    {
        if (Physics.Raycast(start, Vector3.down, out RaycastHit hit, rayLength, groundMask, QueryTriggerInteraction.Ignore))
        {
            var prof = defaultProfile;
            var tag = hit.collider.GetComponent<SurfaceTag>();
            if (tag && tag.profile) prof = tag.profile;
            return CreateFromProfile(prof, true, hit.normal);
        }
        return CreateFromProfile(defaultProfile, false, Vector3.up);
    }

    static SurfaceBlend CreateFromProfile(SurfaceProfile p, bool hasGround, Vector3 n)
    {
        if (!p) p = ScriptableObject.CreateInstance<SurfaceProfile>();
        return new SurfaceBlend
        {
            hasGround = hasGround,
            avgNormal = n.sqrMagnitude > 0.01f ? n.normalized : Vector3.up,
            longAccelMult = p.longAccelMult,
            brakeMult = p.brakeMult,
            sideFriction = p.sideFriction,
            forwardDrag = p.forwardDrag,
            steerResponseMult = p.steerResponseMult,
            slopeSlide = p.slopeSlide,
            topSpeedMult = p.topSpeedMult,
        };
    }
}

#endregion