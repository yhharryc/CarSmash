using UnityEngine;
using UnityEngine.InputSystem; // Requires the new Input System (optional if you drive via code)

// -------------------------------------------------------------
// ArcadeVehicleController
// Thin controller that: collects input -> calls VehicleMotor.Step
// - Works with Unity Input System (Send Messages or C# callbacks)
// - Camera-relative or Top-Down steering (toggle at runtime)
// - Falls back to Camera.main if no camera assigned
// -------------------------------------------------------------

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(VehicleMotor))]
[RequireComponent(typeof(SurfaceDetector))]
public class ArcadeVehicleController : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Optional. If null, will use Camera.main.transform when available.")]
    public Transform cameraTransform;

    private VehicleMotor motor;
    private SurfaceDetector surfaceDetector;

    [Header("Control Mode")]
    [Tooltip("False = camera-relative (TPS style). True = top-down car style (x = steer).")]
    public bool topDownMode = false;

    [Header("Input (runtime)")]
    [SerializeField] private Vector2 moveInput;     // x: left/right, y: forward/back wish
    [SerializeField] private float accelerateInput; // 0..1
    [SerializeField] private float brakeInput;      // 0..1 (also used for reverse)

    [Header("Input Tuning")]
    [Range(0f, 0.3f)] public float deadzone = 0.1f;
    [Range(0.5f, 5f)] public float inputLerp = 8f; // input smoothing per second

    // cached
    private Rigidbody rb;

    // For non-InputSystem users, you can drive inputs via public setters below.

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        motor = GetComponent<VehicleMotor>();
        surfaceDetector = GetComponent<SurfaceDetector>();
    }

    void Start()
    {
        if (cameraTransform == null && Camera.main != null)
            cameraTransform = Camera.main.transform;
    }

    void FixedUpdate()
    {
        // Smooth deadzoned input
        Vector2 rawMove = moveInput.magnitude < deadzone ? Vector2.zero : moveInput;
        // Optionally smooth (small Euler integration)
        moveInput = Vector2.Lerp(moveInput, rawMove, 1f - Mathf.Exp(-inputLerp * Time.fixedDeltaTime));

        float a = Mathf.Clamp01(accelerateInput);
        float b = Mathf.Clamp01(brakeInput);

        var surface = surfaceDetector != null ? surfaceDetector.Current : DefaultSurface();

        motor.Step(
            transform,
            cameraTransform,
            moveInput,
            a,
            b,
            topDownMode,
            surface,
            Time.fixedDeltaTime
        );
    }

    // -------------------------------
    // Input System (Send Messages) hooks
    // Add a PlayerInput (Behavior: Send Messages), map actions to these method names.
    //   Move (Vector2)      -> OnMove
    //   Accelerate (Axis)   -> OnAccelerate
    //   Brake/Reverse (Axis)-> OnBrake
    //   Toggle View (Button)-> OnToggleView
    // -------------------------------

    public void OnMove(InputAction.CallbackContext ctx)
    {
        moveInput = ctx.ReadValue<Vector2>();
    }

    public void OnAccelerate(InputAction.CallbackContext ctx)
    {
        accelerateInput = ctx.ReadValue<float>();
    }

    public void OnBrake(InputAction.CallbackContext ctx)
    {
        brakeInput = ctx.ReadValue<float>();
    }

    public void OnToggleView(InputAction.CallbackContext ctx)
    {
        if (ctx.performed)
            topDownMode = !topDownMode;
    }

    // -------------------------------
    // Optional: public API for code-driven input (no Input System).
    // -------------------------------

    public void SetMove(Vector2 move) => moveInput = move;
    public void SetAccelerate(float accel01) => accelerateInput = accel01;
    public void SetBrake(float brake01) => brakeInput = brake01;
    public void SetTopDownMode(bool enabled) => topDownMode = enabled;

    // -------------------------------
    // Helper: default surface if detector missing (shouldn't happen due to RequireComponent)
    // -------------------------------
    private static SurfaceBlend DefaultSurface()
    {
        return new SurfaceBlend
        {
            hasGround = true,
            avgNormal = Vector3.up,
            longAccelMult = 1f,
            brakeMult = 1f,
            sideFriction = 12f,
            forwardDrag = 0.5f,
            steerResponseMult = 1f,
            slopeSlide = 0.2f,
            topSpeedMult = 1f
        };
    }

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        // Quick viz: show move input in local space
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, transform.position + (transform.right * moveInput.x + transform.forward * moveInput.y));
    }
#endif
}
