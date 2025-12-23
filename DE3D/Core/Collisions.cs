/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Collision System for Unity3D        /____/                            
                                                                    By: Elitmers */
using UnityEngine;
using System.Collections.Generic;

namespace DynamicEngine
{
    public class CollisionHandler
    {
        private readonly Collider[] overlapResults = new Collider[32];
        private LayerMask collisionLayerMask = ~0;
        public readonly List<Vector3> collisionPoints;

        public CollisionHandler()
        {
            collisionPoints = new List<Vector3>();
        }

        public LayerMask CollisionLayerMask
        {
            get => collisionLayerMask;
            set => collisionLayerMask = value;
        }

        public void ResolveCollisions(
            NodeManager nodeManager,
            Transform owner,
            Matter matterAsset,
            List<float> nodeMasses,
            float dt,
            float epsilon,
            bool visualizeForces)
        {
            int nodeCount = nodeManager.Nodes.Count;
            collisionPoints.Clear();

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i]) continue;

                Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                Vector3 predictedWorldPos = nodeManager.PredictedPositions[i];
                float radius = nodeManager.GetNodeRadius(i);

                int hitCount = Physics.OverlapSphereNonAlloc(predictedWorldPos, radius, overlapResults, collisionLayerMask);

                for (int j = 0; j < hitCount; j++)
                {
                    Collider col = overlapResults[j];
                    if (IsOwnCollider(col, i, nodeManager)) continue;

                    if (GetCollisionInfo(col, predictedWorldPos, radius, out Vector3 contactPoint, out Vector3 normal, out float penetration))
                    {
                        // Step 1: Correct penetration
                        predictedWorldPos += normal * (penetration + epsilon);

                        // Step 2: Calculate velocity from corrected position
                        Vector3 velocity = (predictedWorldPos - currentWorldPos) / dt;

                        // Step 3: Apply impulse (restitution + friction)
                        Vector3 correctedVelocity = ApplyCollisionImpulse(velocity, normal, 1f / nodeMasses[i], matterAsset);

                        // Step 4: Update position based on corrected velocity
                        predictedWorldPos = currentWorldPos + correctedVelocity * dt;

                        nodeManager.PredictedPositions[i] = predictedWorldPos;
                        collisionPoints.Add(contactPoint);

                        if (visualizeForces)
                        {
                            Debug.DrawRay(contactPoint, normal * 0.2f, Color.red, dt);
                        }
                    }
                }
            }
        }

        private Vector3 ApplyCollisionImpulse(Vector3 vel, Vector3 normal, float invMass, Matter matterAsset)
        {
            float restitution = matterAsset != null ? matterAsset.Restitution : 0.02f;
            float dynamicFriction = matterAsset != null ? matterAsset.DynamicFriction : 0.6f;
            float staticFriction = matterAsset != null ? matterAsset.StaticFriction : 0.8f;

            float normalVel = Vector3.Dot(vel, normal);

            // Calculate tangential velocity
            Vector3 tangentVel = vel - normal * normalVel;
            float tangentSpeed = tangentVel.magnitude;

            Vector3 newVel = vel;

            // NORMAL IMPULSE: Only apply if moving into surface
            if (normalVel < 0)
            {
                float jn = -(1 + restitution) * normalVel / invMass;
                newVel += normal * jn * invMass;

                // Update normal velocity after collision response
                normalVel = Vector3.Dot(newVel, normal);
            }

            // FRICTION: Apply for both resting and sliding contacts
            if (tangentSpeed > 0.0001f)
            {
                Vector3 tangent = tangentVel / tangentSpeed;

                // For resting contacts, use gravity-based normal force
                // For collision contacts, use the impulse-based force
                float normalForce;
                if (normalVel < -0.01f)
                {
                    // Active collision - use impulse-based normal force
                    float jn = -(1 + restitution) * normalVel / invMass;
                    normalForce = Mathf.Abs(jn);
                }
                else
                {
                    // Resting or slow sliding contact - use weight-based normal force
                    float gravityValue = SceneSettings.Instance != null ? SceneSettings.Instance.Gravity : 9.81f;
                    float mass = 1f / invMass;
                    normalForce = mass * gravityValue * Time.fixedDeltaTime;
                }

                // Calculate friction impulse magnitude
                float tangentImpulse = tangentSpeed / invMass;
                float maxStaticFriction = staticFriction * normalForce;
                float maxDynamicFriction = dynamicFriction * normalForce;

                // Apply static or dynamic friction
                if (tangentImpulse <= maxStaticFriction)
                {
                    // Static friction: stop all tangential motion
                    newVel -= tangentVel;
                }
                else
                {
                    // Dynamic friction: reduce tangential velocity
                    Vector3 frictionForce = -tangent * maxDynamicFriction * invMass;
                    newVel += frictionForce;
                }
            }

            return newVel;
        }

        private bool GetCollisionInfo(Collider col, Vector3 pos, float radius, out Vector3 contactPoint, out Vector3 normal, out float penetration)
        {
            contactPoint = Vector3.zero;
            normal = Vector3.up;
            penetration = 0f;

            if (col is TerrainCollider terrainCol)
            {
                Terrain terrain = terrainCol.GetComponent<Terrain>();
                if (terrain != null)
                {
                    float height = terrain.SampleHeight(pos) + terrain.transform.position.y;

                    if (pos.y < height + radius)
                    {
                        contactPoint = new Vector3(pos.x, height, pos.z);
                        normal = Vector3.up;
                        penetration = (height + radius) - pos.y;
                        return true;
                    }
                }
            }
            else if (col is MeshCollider meshCol && !meshCol.convex)
            {
                Vector3 direction = Vector3.down;
                if (col.Raycast(new Ray(pos + Vector3.up * radius, direction), out RaycastHit meshHit, radius * 2f))
                {
                    contactPoint = meshHit.point;
                    normal = meshHit.normal;
                    penetration = radius - Vector3.Distance(pos, meshHit.point);
                    return penetration > 0;
                }
            }
            else
            {
                contactPoint = col.ClosestPoint(pos);
                float dist = Vector3.Distance(contactPoint, pos);

                if (dist < radius)
                {
                    normal = dist > 0.001f ? (pos - contactPoint).normalized : Vector3.up;
                    penetration = radius - dist;
                    return true;
                }
            }

            return false;
        }

        private bool IsOwnCollider(Collider col, int nodeIndex, NodeManager nodeManager)
        {
            if (col == nodeManager.Colliders[nodeIndex]) return true;
            if (col.transform == nodeManager.Nodes[nodeIndex]) return true;

            int otherNodeIndex = nodeManager.FindIndex(n => n != null && n == col.transform);
            return otherNodeIndex != -1;
        }
    }
}