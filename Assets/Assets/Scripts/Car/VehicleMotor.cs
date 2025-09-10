using UnityEngine;

// ----------------------------------------------
//  ARCADE VEHICLE PACKAGE (drop-in modules)
//  - SurfaceProfile (ScriptableObject)
//  - SurfaceTag (attach to ground colliders)
//  - SurfaceDetector (raycasts & blends surface)
//  - VehicleMotor (speed-driven, physics-friendly)
//  - KnockbackReceiver (deterministic hit reaction)
//  - ImpactResolver (computes knockback from hits)
// ----------------------------------------------

#region Vehicle Motor (speed-driven, physics-friendly)

[RequireComponent(typeof(Rigidbody))]
public class VehicleMotor : MonoBehaviour
{
    [Header("References")]
    public Transform meshTransform; // optional for visual tilt

    private Rigidbody rb;

    [Header("Speed Model (like your controller)")]
    public float maxSpeed = 10f;
    public float maxReverseSpeed = 5f;
    public float acceleration = 5f;
    public float reverseAcceleration = 3f;
    public float brakeAcceleration = 8f;
    public float idleDeceleration = 3f;

    [Header("Steering")]
    public float maxSteeringAngle = 30f;
    public float steeringSpeed = 90f;
    public float minSteeringSpeed = 0.5f; // threshold to rotate visually

    [Header("Drift & Grip")]
    public float highGrip = 12f;  // strong side friction (asphalt)
    public float lowGrip = 3f;    // weak side friction (drift/ice)
    public float driftEnterAngle = 12f; // degrees
    public float driftMinSpeed = 3f;
    [Range(0f, 0.8f)] public float driftYawFollow = 0.25f; // how much we bias yaw towards velocity
    [Range(0.1f, 1.2f)] public float driftSteerMult = 0.7f;  // slower steer when drifting

    [Header("Visuals")]
    public float tiltMaxRoll = 8f; // degrees
    public float tiltLerp = 10f;

    // runtime
    private float currentSteeringAngle;
    private float visualRoll;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    /// <summary>
    /// One-call step that: reads rb.velocity, applies your arcade model + surface, rotates via rb.MoveRotation, writes rb.velocity.
    /// </summary>
    public void Step(
        Transform vehicleTransform,
        Transform cameraTransform,
        Vector2 moveInput,
        float accelerateInput,
        float brakeInput,
        bool topDownMode,
        in SurfaceBlend surface,
        float dt)
    {
        Vector3 rbVel = rb.velocity;
        Vector3 up = Vector3.up; // could blend with surface.avgNormal for banking

        // split velocities
        Vector3 verticalVel = Vector3.Project(rbVel, up);
        Vector3 horizontalVel = rbVel - verticalVel;

        // compute steering target (similar to your code)
        Vector2 targetSteering = ComputeTargetSteering(vehicleTransform, cameraTransform, moveInput, topDownMode);
        Vector3 forward = vehicleTransform.forward;

        // turn wheel angle (local forward vs target dir)
        Vector3 targetDirLocal = new Vector3(targetSteering.x, 0f, targetSteering.y);
        float angleDifference = Vector3.SignedAngle(Vector3.forward, targetDirLocal, Vector3.up);
        float targetAngle = Mathf.Clamp(angleDifference, -maxSteeringAngle, maxSteeringAngle);
        currentSteeringAngle = Mathf.MoveTowards(currentSteeringAngle, targetAngle, steeringSpeed * dt * surface.steerResponseMult);

        // desired steering world direction
        Vector3 steeringDir = Quaternion.Euler(0f, currentSteeringAngle, 0f) * forward;

        // forward speed sign on ground plane
        float forwardSpeed = Vector3.Dot(horizontalVel, forward);

        // 1) acceleration/brake similar to your controller (scaled by surface)
        if (accelerateInput > 0.1f)
        {
            if (forwardSpeed < -0.1f)
            {
                // reduce reverse first
                horizontalVel += forward * (brakeAcceleration * surface.brakeMult * 0.5f * dt);
            }
            else
            {
                horizontalVel += steeringDir * (acceleration * surface.longAccelMult * accelerateInput * dt);
            }
        }
        else if (brakeInput > 0.1f)
        {
            if (forwardSpeed > 1f)
            {
                // braking
                Vector3 brakeDir = -horizontalVel.normalized;
                horizontalVel += brakeDir * (brakeAcceleration * surface.brakeMult * brakeInput * dt);
            }
            else
            {
                // reverse
                Vector3 reverseDir = -steeringDir.normalized;
                horizontalVel += reverseDir * (reverseAcceleration * surface.longAccelMult * brakeInput * dt);
            }
        }
        else
        {
            // idle decel
            horizontalVel = Vector3.MoveTowards(horizontalVel, Vector3.zero, idleDeceleration * dt);
        }

        // 2) SIDE FRICTION & FORWARD DRAG (the car-feel core)
        Vector3 fwd = forward;
        Vector3 vFwd = Vector3.Project(horizontalVel, fwd);
        Vector3 vSide = horizontalVel - vFwd;

        // drift detection
        float speed = horizontalVel.magnitude;
        float slipAngle = speed > 0.1f ? Vector3.SignedAngle(fwd, horizontalVel, up) : 0f;
        bool drifting = Mathf.Abs(slipAngle) > driftEnterAngle && speed > driftMinSpeed;

        float curGrip = Mathf.Lerp(highGrip, lowGrip, drifting ? 1f : 0f);
        curGrip = Mathf.Max(0f, curGrip) + surface.sideFriction; // surface contributes baseline

        vSide = Vector3.MoveTowards(vSide, Vector3.zero, curGrip * dt);
        vFwd = Vector3.MoveTowards(vFwd, Vector3.zero, surface.forwardDrag * dt);
        horizontalVel = vFwd + vSide;

        // 3) SLOPE SLIDE (along ground plane)
        if (surface.hasGround)
        {
            Vector3 n = surface.avgNormal.sqrMagnitude > 0.01f ? surface.avgNormal.normalized : up;
            Vector3 gAlong = Physics.gravity - n * Vector3.Dot(Physics.gravity, n);
            horizontalVel += gAlong * surface.slopeSlide * dt;
        }

        // 4) CLAMP TOP SPEEDS (also scale by surface.topSpeedMult)
        float topFwd = maxSpeed * Mathf.Max(0.25f, surface.topSpeedMult);
        float topRev = maxReverseSpeed * Mathf.Max(0.25f, surface.topSpeedMult);

        float fwdSpd = Vector3.Dot(horizontalVel, fwd);
        if (fwdSpd > 0f)
        {
            if (fwdSpd > topFwd)
            {
                horizontalVel += fwd * (topFwd - fwdSpd);
            }
        }
        else
        {
            float revAbs = Mathf.Abs(fwdSpd);
            if (revAbs > topRev)
            {
                float newRev = -Mathf.Sign(fwdSpd) * topRev; // negative
                horizontalVel += fwd * (newRev - fwdSpd);
            }
        }

        // 5) ROTATION (prefer rb.MoveRotation)
        bool shouldRotate = speed > minSteeringSpeed || accelerateInput > 0.1f || brakeInput > 0.1f;
        if (shouldRotate)
        {
            // during drift, bias yaw a bit towards velocity direction (countersteer feel)
            Vector3 yawTarget = drifting && horizontalVel.sqrMagnitude > 0.01f
                ? Vector3.Slerp(steeringDir, horizontalVel.normalized, driftYawFollow)
                : steeringDir;

            Quaternion targetRot = Quaternion.LookRotation(yawTarget, up);
            float maxAngleStep = steeringSpeed * surface.steerResponseMult * (drifting ? driftSteerMult : 1f) * dt;
            rb.MoveRotation(Quaternion.RotateTowards(rb.rotation, targetRot, maxAngleStep));

            // optional visual roll by slip
            if (meshTransform)
            {
                float targetRoll = Mathf.Clamp(-slipAngle, -tiltMaxRoll, tiltMaxRoll);
                visualRoll = Mathf.MoveTowards(visualRoll, targetRoll, tiltLerp * dt);
                var yaw = meshTransform.rotation.eulerAngles.y; // preserve yaw from rb
                meshTransform.rotation = Quaternion.Euler(0f, yaw, visualRoll);
            }
        }

        // 6) WRITE BACK VELOCITY (preserve vertical)
        rb.velocity = horizontalVel + verticalVel;
    }

    private static Vector2 ComputeTargetSteering(Transform vehicle, Transform cameraTransform, Vector2 moveInput, bool topDownMode)
    {
        if (moveInput.magnitude < 0.1f)
            return Vector2.zero;

        if (!topDownMode && cameraTransform)
        {
            Vector3 camF = cameraTransform.forward; camF.y = 0f; camF.Normalize();
            Vector3 camR = cameraTransform.right;  camR.y = 0f; camR.Normalize();
            Vector3 worldTarget = (camF * moveInput.y + camR * moveInput.x).normalized;
            Vector3 local = vehicle.InverseTransformDirection(worldTarget);
            return new Vector2(local.x, local.z);
        }
        else
        {
            // top-down style: keep forward, steer by x
            Vector3 local = new Vector3(moveInput.x, 0f, 1f).normalized;
            return new Vector2(local.x, local.z);
        }
    }
}

#endregion

