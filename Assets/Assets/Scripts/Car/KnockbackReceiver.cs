using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface IKnockbackReceiver
{
    void ApplyKnockback(Vector3 velocity, float stunTime);
}

[RequireComponent(typeof(Rigidbody))]
public class KnockbackReceiver : MonoBehaviour, IKnockbackReceiver
{
    public float friction = 4f; // how quickly the forced velocity decays

    private Rigidbody rb;
    private Vector3 forcedVel;
    private float stunTimer;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    public void ApplyKnockback(Vector3 velocity, float stunTime)
    {
        forcedVel = velocity;
        stunTimer = Mathf.Max(stunTimer, stunTime);
    }

    void FixedUpdate()
    {
        // decay forced velocity (horizontal only) and overlay on rb
        Vector3 up = Vector3.up;
        Vector3 vertical = Vector3.Project(rb.velocity, up);
        Vector3 baseHoriz = rb.velocity - vertical;

        // reduce forced component over time
        forcedVel = Vector3.MoveTowards(forcedVel, Vector3.zero, friction * Time.fixedDeltaTime);

        rb.velocity = baseHoriz + forcedVel + vertical;
    }

    public bool IsStunned => stunTimer > 0f;
}