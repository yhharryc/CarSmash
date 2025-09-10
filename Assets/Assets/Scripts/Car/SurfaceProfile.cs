using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#region Surface Profiles

[CreateAssetMenu(fileName = "SurfaceProfile", menuName = "ArcadeVehicle/Surface Profile")]
public class SurfaceProfile : ScriptableObject
{
    [Header("Longitudinal/Braking")]
    [Tooltip("Multiplier for forward acceleration on this surface (1 = unchanged).")]
    public float longAccelMult = 1f;

    [Tooltip("Multiplier for braking effectiveness on this surface (1 = unchanged). Lower on ice.")]
    public float brakeMult = 1f;

    [Header("Friction/Drag")]
    [Tooltip("Side friction strength (how quickly lateral velocity is killed). High on asphalt, low on ice.")]
    public float sideFriction = 12f;

    [Tooltip("Forward drag / rolling resistance (per second). Higher in mud/grass.")]
    public float forwardDrag = 0.5f;

    [Header("Steer & Slope")]
    [Tooltip("Steering response multiplier ( <1 = sluggish, >1 = snappier ).")]
    public float steerResponseMult = 1f;

    [Tooltip("How much gravity projected along the surface contributes to sliding (ice > asphalt).")]
    public float slopeSlide = 0.2f;

    [Header("Top Speed")]
    [Tooltip("Optional cap multiplier for top speed on this surface.")]
    public float topSpeedMult = 1f;
}



#endregion