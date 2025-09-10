using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class ImpactResolver : MonoBehaviour
{
    [Header("Hit Tuning")]
    public float baseKnockback = 8f;
    public float knockbackScale = 0.5f; // multiplied by relative speed at impact
    public float hitStunTime = 0.2f;
    public float minHitSpeed = 2f;      // ignore tiny bumps

    [Tooltip("Layers considered as hittable opponents.")]
    public LayerMask hittableMask = ~0;

    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
    }

    void OnCollisionEnter(Collision c) { HandleCollision(c); }
    void OnCollisionStay(Collision c)  { HandleCollision(c); }

    void HandleCollision(Collision c)
    {
        if (((1 << c.gameObject.layer) & hittableMask) == 0)
            return;

        // choose the deepest/first contact normal
        if (c.contactCount == 0) return;
        var contact = c.GetContact(0);
        Vector3 n = contact.normal; // normal pointing from other->this (usually)

        // relative speed along impact
        Vector3 relVel = rb.velocity - GetOtherVelocity(c.rigidbody);
        float relSpeed = Vector3.Dot(relVel, -n); // positive if moving into each other
        if (relSpeed < minHitSpeed) return;

        // knockback direction: away from me to them
        Vector3 hitDir = (c.transform.position - transform.position).normalized;
        float launch = baseKnockback + knockbackScale * relSpeed;
        Vector3 launchVel = hitDir * launch;

        // apply to other via interface or Rigidbody
        var recv = c.gameObject.GetComponentInParent<IKnockbackReceiver>();
        if (recv != null)
        {
            recv.ApplyKnockback(launchVel, hitStunTime);
        }
        else if (c.rigidbody)
        {
            // direct velocity change for simplicity
            c.rigidbody.velocity = c.rigidbody.velocity + launchVel;
        }

        // (optional) self recoil or state changes could go here
    }

    static Vector3 GetOtherVelocity(Rigidbody other)
    {
        return other ? other.velocity : Vector3.zero;
    }
}