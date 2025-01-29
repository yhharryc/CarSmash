using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class AdvancedCarController : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform cameraTransform;
    private Rigidbody rb;
    [SerializeField] private Transform meshTransform;

    [Header("Settings")]
    public bool topDownMode = false; // <-- 新增：切换控制模式

    [SerializeField] private float maxSpeed = 10f;
    [SerializeField] private float maxReverseSpeed = 5f;   // 倒车最大速度(绝对值)
    [SerializeField] private float acceleration = 5f;      // 前进加速度
    [SerializeField] private float reverseAcceleration = 3f;// 倒车加速度
    [SerializeField] private float brakeAcceleration = 8f; // 制动减速度
    [SerializeField] private float idleDeceleration = 3f;  // 自然减速
    [SerializeField] private float maxSteeringAngle = 30f;
    [SerializeField] private float steeringSpeed = 90f;
    [SerializeField] private float minSteeringSpeed = 1f;  // 允许转向的最低速度

    [Header("Runtime Values")]
    private Vector2 moveInput;
    private float accelerateInput; // 油门(前进)
    private float brakeInput;      // 刹车或倒车
    private Vector3 currentVelocity;
    private float currentSteeringAngle;
    private Vector2 targetSteering;
    private bool isReversing;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        // 由于后面我们直接设 rb.velocity，可让 isKinematic = false 
        rb.isKinematic = false;
    }

    // 输入系统绑定
    public void OnMovement(InputAction.CallbackContext context) => moveInput = context.ReadValue<Vector2>();
    public void OnAccelerate(InputAction.CallbackContext context) => accelerateInput = context.ReadValue<float>();
    public void OnBrake(InputAction.CallbackContext context) => brakeInput = context.ReadValue<float>();

    private void FixedUpdate()
    {
        UpdateSteeringDirection();
        HandleSteering();
        HandleAcceleration();
        ApplyMovement();
    }

    /// <summary>
    /// 根据控制模式，计算转向向量 targetSteering。
    /// topDownMode == false => 摄像机方向模式；
    /// topDownMode == true  => 仅左右输入控制转向，类似俯视车的方式
    /// </summary>
    private void UpdateSteeringDirection()
    {
        // 如果输入幅度很小，就不做任何转向
        if (moveInput.magnitude < 0.1f)
        {
            targetSteering = Vector2.zero;
        }
        else
        {
            if (!topDownMode)
            {
                // ============== 原先的摄像机方向模式 ==============
                Vector3 cameraForward = cameraTransform.forward;
                cameraForward.y = 0;
                cameraForward.Normalize();

                Vector3 cameraRight = cameraTransform.right;
                cameraRight.y = 0;
                cameraRight.Normalize();

                // 组合输入方向（保留原始输入比例）
                Vector3 worldTargetDirection =
                    (cameraForward * moveInput.y +
                     cameraRight * moveInput.x).normalized;

                // 转换为车辆局部空间的相对方向
                Vector3 localTargetDirection = transform.InverseTransformDirection(worldTargetDirection);
                targetSteering = new Vector2(localTargetDirection.x, localTargetDirection.z);
            }
            else
            {
                // ============== top-down 模式 ==============
                // 假设 moveInput.x => 左右转向，moveInput.y => 前进/后退(仍在 HandleAcceleration 里处理)。
                // 在此，我们只关心水平值 x 来决定转向方向。
                // 例如，我们假定：把本地前方视为 z=1，左右则由 x=±(moveInput.x)
                // 如果你只想让 "left / right" 输入影响转向，可这样：
                
                // 强制让 z=1，表示“目标前向”始终是本地Z前，
                // x=moveInput.x 用于侧向偏移
                Vector3 localTargetDirection = new Vector3(moveInput.x, 0f, 1f);

                // 如果希望在 top-down 下也做一个归一化，避免极端输入失真
                localTargetDirection.Normalize();

                targetSteering = new Vector2(localTargetDirection.x, localTargetDirection.z);
            }
        }

        // 同时检查是否处于倒车状态
        isReversing = Vector3.Dot(currentVelocity, transform.forward) < 0f;
    }

    private void HandleSteering()
    {
        if (targetSteering.magnitude < 0.1f)
        {
            currentSteeringAngle = 0;
            return;
        }

        // 计算当前前向与目标方向的夹角
        Vector3 currentForward = Vector3.forward; // 车辆局部空间的前向
        Vector3 targetDir = new Vector3(targetSteering.x, 0, targetSteering.y).normalized;

        float angleDifference = Vector3.SignedAngle(currentForward, targetDir, Vector3.up);

        // 应用最大转向限制
        float targetSteeringAngle = Mathf.Clamp(angleDifference, -maxSteeringAngle, maxSteeringAngle);

        // 平滑转向过渡
        currentSteeringAngle = Mathf.MoveTowards(
            currentSteeringAngle,
            targetSteeringAngle,
            steeringSpeed * Time.fixedDeltaTime
        );
    }

    /// <summary>
    /// 区分：油门 => 前进加速度；刹车 => 如果前进速度高则减速，否则倒车
    /// </summary>
    private void HandleAcceleration()
    {
        // 1. 计算车辆当前的前向、以及前向速度标量(点积)
        Vector3 forwardDir = transform.forward;
        float forwardSpeed = Vector3.Dot(currentVelocity, forwardDir);

        // 2. 计算转向角对应的局部前向
        Vector3 steeringDir = Quaternion.Euler(0, currentSteeringAngle, 0) * forwardDir;

        // 3. 前进油门逻辑
        if (accelerateInput > 0.1f)
        {
            // 如果车辆已经在倒车状态，则先尝试减小后退速度
            if (forwardSpeed < -0.1f)
            {
                // 这种情况：人在倒车时又按了油门(前进)。很多赛车游戏会先把后退速度减到0，再开始前进。
                // 简化做法：对 currentVelocity 做一个小制动
                Vector3 stopReverseForce = forwardDir * (brakeAcceleration * 0.5f);
                currentVelocity += stopReverseForce * Time.fixedDeltaTime;
            }
            else
            {
                // 正常前进加速
                Vector3 accel = steeringDir * (acceleration * accelerateInput);
                currentVelocity += accel * Time.fixedDeltaTime;
            }
        }
        // 4. 刹车或倒车逻辑
        else if (brakeInput > 0.1f)
        {
            // 如果当前前进速度明显>0，表示我们在前进，需要刹车
            if (forwardSpeed > 1f)
            {
                // 直接对当前速度施加制动力(反向于当前速度)
                Vector3 brakeDir = -currentVelocity.normalized;
                Vector3 brakeForce = brakeDir * (brakeAcceleration * brakeInput);
                currentVelocity += brakeForce * Time.fixedDeltaTime;
            }
            else
            {
                // 当前速度已经很小或在倒退，则视为“倒车”输入
                Vector3 reverseDir = -steeringDir.normalized;
                Vector3 reverseAccel = reverseDir * (reverseAcceleration * brakeInput);
                currentVelocity += reverseAccel * Time.fixedDeltaTime;
            }
        }
        // 5. 如果既没有油门也没有刹车，则自然减速
        else
        {
            currentVelocity = Vector3.MoveTowards(
                currentVelocity,
                Vector3.zero,
                idleDeceleration * Time.fixedDeltaTime
            );
        }

        // 6. 限制最大速度 & 最大倒车速度
        if (forwardSpeed > 0f)
        {
            float newForwardSpeed = Mathf.Min(forwardSpeed, maxSpeed);
            currentVelocity = currentVelocity + forwardDir * (newForwardSpeed - forwardSpeed);
        }
        else
        {
            float reverseSpd = Mathf.Abs(forwardSpeed);
            if (reverseSpd > maxReverseSpeed)
            {
                float newReverseSpeed = maxReverseSpeed;
                float sign = (forwardSpeed < 0f) ? -1f : 1f;
                currentVelocity = currentVelocity + forwardDir * ((sign * newReverseSpeed) - forwardSpeed);
            }
        }

        // 7. 更新车身旋转
        UpdateRotation(steeringDir);
    }

    /// <summary>
    /// 更新车身朝向：倒车时不强行让车头翻 180°，而是分情况处理
    /// </summary>
    private void UpdateRotation(Vector3 steeringDir)
    {
        bool shouldRotate = currentVelocity.magnitude > minSteeringSpeed
                            || accelerateInput > 0.1f
                            || brakeInput > 0.1f;
        if (!shouldRotate) return;

        float dotForward = Vector3.Dot(currentVelocity, meshTransform.forward);
        bool reversingNow = (dotForward < 0f);

        // 计算一个刹车因子：刹车越多，越偏向保持当前 velocity 方向
        float brakeFactor = Mathf.Clamp01(brakeInput);

        Vector3 finalDirection;
        if (!reversingNow)
        {
            // 前进：可以在这里再决定是否插值到 currentVelocity.normalized,
            // 或者仅插值到 transform.forward (如你自己调试)
            // 这里演示插值到当前车身 forward
            finalDirection = Vector3.Slerp(steeringDir, transform.forward, brakeFactor);
        }
        else
        {
            // 倒车：不想让车头翻转到速度方向，那就只按 steeringDir
            finalDirection = steeringDir.normalized;
        }

        if (finalDirection.sqrMagnitude > 0.0001f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(finalDirection, Vector3.up);
            float maxAngleStep = steeringSpeed * Time.fixedDeltaTime;
            transform.rotation = Quaternion.RotateTowards(
                transform.rotation,
                targetRotation,
                maxAngleStep
            );
        }
    }

    private void ApplyMovement()
    {
        // 把我们更新后的 currentVelocity 直接赋给刚体
        rb.velocity = currentVelocity;
    }

    // 调试显示
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;

        // 显示速度方向（红色）
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + currentVelocity);

        // 显示当前转向方向（绿色）
        Vector3 steeringDir = Quaternion.Euler(0, currentSteeringAngle, 0) * transform.forward;
        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, transform.position + steeringDir * 2);
    }
}
