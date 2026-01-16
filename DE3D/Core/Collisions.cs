/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Collision System with Full CCD for Unity3D        /____/                            
                                                                    By: Elitmers */
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace DynamicEngine
{
    #region Collision Strategy Interface
    internal interface ICollisionStrategy
    {
        bool GetSweptCollisionInfo(Collider col, Vector3 startPos, Vector3 endPos, float radius,
                                   out Vector3 contactPoint, out Vector3 normal,
                                   out float penetration, out float hitTime);
    }

    #endregion
    struct ContactCache
    {
        public Vector3 normal;
        public float penetration;
        public int frameLastSeen;
        public bool wasResting;
    }

    #region Collision Strategies

    internal class TerrainCollisionStrategy : ICollisionStrategy
    {
        public bool GetSweptCollisionInfo(Collider col, Vector3 startPos, Vector3 endPos,
                                          float radius, out Vector3 contactPoint,
                                          out Vector3 normal, out float penetration,
                                          out float hitTime)
        {
            contactPoint = Vector3.zero;
            normal = Vector3.up;
            penetration = 0f;
            hitTime = 1f;

            if (!(col is TerrainCollider terrainCol))
                return false;

            Terrain terrain = terrainCol.GetComponent<Terrain>();
            if (terrain == null)
                return false;

            Vector3 direction = endPos - startPos;
            float distance = direction.magnitude;

            if (distance < 0.001f)
            {
                float height = terrain.SampleHeight(endPos) + terrain.transform.position.y;
                if (endPos.y >= height + radius)
                    return false;

                contactPoint = new Vector3(endPos.x, height, endPos.z);
                penetration = (height + radius) - endPos.y;
                hitTime = 0f;
                return true;
            }

            int samples = Mathf.CeilToInt(distance / (radius * 0.5f)) + 1;
            samples = Mathf.Min(samples, 20);

            for (int i = 0; i <= samples; i++)
            {
                float t = i / (float)samples;
                Vector3 testPos = Vector3.Lerp(startPos, endPos, t);
                float height = terrain.SampleHeight(testPos) + terrain.transform.position.y;

                if (testPos.y < height + radius)
                {
                    contactPoint = new Vector3(testPos.x, height, testPos.z);
                    penetration = (height + radius) - testPos.y;
                    hitTime = t;
                    return true;
                }
            }

            return false;
        }
    }

    internal class MeshCollisionStrategy : ICollisionStrategy
    {
        private CollisionHandler _collisionHandler;

        public MeshCollisionStrategy(CollisionHandler collisionHandler)
        {
            _collisionHandler = collisionHandler;
        }

        public bool GetSweptCollisionInfo(Collider col, Vector3 startPos, Vector3 endPos,
                                         float radius, out Vector3 contactPoint,
                                         out Vector3 normal, out float penetration,
                                         out float hitTime)
        {
            contactPoint = Vector3.zero;
            normal = Vector3.up;
            penetration = 0f;
            hitTime = 1f;

            if (!(col is MeshCollider meshCol) || meshCol.convex)
                return false;

            Mesh mesh = meshCol.sharedMesh;
            if (mesh == null) return false;

            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;

            Matrix4x4 localToWorld = meshCol.transform.localToWorldMatrix;

            Vector3 velocityDir = (endPos - startPos).normalized;
            float velocityMag = (endPos - startPos).magnitude;

            float bestT = float.MaxValue;
            Vector3 bestContact = Vector3.zero;
            Vector3 bestNormal = Vector3.zero;
            float bestPenetration = 0f;
            bool hasSweptHit = false;

            for (int tri = 0; tri < triangles.Length; tri += 3)
            {
                Vector3 v0 = localToWorld.MultiplyPoint3x4(vertices[triangles[tri]]);
                Vector3 v1 = localToWorld.MultiplyPoint3x4(vertices[triangles[tri + 1]]);
                Vector3 v2 = localToWorld.MultiplyPoint3x4(vertices[triangles[tri + 2]]);

                if (_collisionHandler.CheckSweptSphereTriangle(startPos, endPos, radius, velocityDir, velocityMag,
                                            v0, v1, v2, out Vector3 triContact, out Vector3 triNormal,
                                            out float triPen, out float triT))
                {
                    hasSweptHit = true;
                    if (triT < bestT)
                    {
                        bestT = triT;
                        bestContact = triContact;
                        bestNormal = triNormal;
                        bestPenetration = triPen;
                    }
                }
            }

            if (hasSweptHit && bestT <= 1f)
            {
                contactPoint = bestContact;
                normal = bestNormal;
                penetration = bestPenetration;
                hitTime = bestT;
                return true;
            }

            // Static check at end position if no swept hit
            float maxPenetration = -float.MaxValue;
            bool hasStaticHit = false;

            for (int tri = 0; tri < triangles.Length; tri += 3)
            {
                Vector3 v0 = localToWorld.MultiplyPoint3x4(vertices[triangles[tri]]);
                Vector3 v1 = localToWorld.MultiplyPoint3x4(vertices[triangles[tri + 1]]);
                Vector3 v2 = localToWorld.MultiplyPoint3x4(vertices[triangles[tri + 2]]);

                if (_collisionHandler.CheckSphereTriangleCollision(endPos, radius, v0, v1, v2,
                                                 out Vector3 triContact, out Vector3 triNormal, out float triPen))
                {
                    hasStaticHit = true;
                    if (triPen > maxPenetration)
                    {
                        maxPenetration = triPen;
                        bestContact = triContact;
                        bestNormal = triNormal;
                        bestPenetration = triPen;
                    }
                }
            }

            if (hasStaticHit)
            {
                contactPoint = bestContact;
                normal = bestNormal;
                penetration = bestPenetration;
                hitTime = 1f;
                return true;
            }

            return false;
        }
    }

    internal class ConvexCollisionStrategy : ICollisionStrategy
    {
        public bool GetSweptCollisionInfo(Collider col, Vector3 startPos, Vector3 endPos,
                                         float radius, out Vector3 contactPoint,
                                         out Vector3 normal, out float penetration,
                                         out float hitTime)
        {
            contactPoint = Vector3.zero;
            normal = Vector3.up;
            penetration = 0f;
            hitTime = 1f;

            Vector3 direction = endPos - startPos;
            float distance = direction.magnitude;

            if (distance > 0.001f)
            {
                if (Physics.SphereCast(startPos, radius, direction.normalized,
                                      out RaycastHit hit, distance,
                                      LayerMask.GetMask("Default")))
                {
                    if (hit.collider == col)
                    {
                        contactPoint = hit.point;
                        normal = hit.normal;
                        hitTime = hit.distance / distance;
                        penetration = radius * (1f - hitTime);
                        return true;
                    }
                }
            }

            contactPoint = col.ClosestPoint(endPos);
            float distToSurface = Vector3.Distance(contactPoint, endPos);

            if (distToSurface >= radius)
                return false;

            normal = distToSurface > PhysicsConstants.MIN_COLLISION_DISTANCE
                ? (endPos - contactPoint).normalized
                : Vector3.up;
            penetration = radius - distToSurface;
            hitTime = 1f;
            return true;
        }
    }

    #endregion

    public class CollisionHandler
    {
        #region Fields

        private readonly RaycastHit[] _sphereCastResults = new RaycastHit[PhysicsConstants.MAX_OVERLAP_RESULTS];
        private readonly Dictionary<int, ContactCache> _contactCache = new Dictionary<int, ContactCache>();
        private int _frameCount = 0;
        private readonly List<ICollisionStrategy> _collisionStrategies;
        private readonly HashSet<(Transform, Transform)> _ignoredBodyPairs = new HashSet<(Transform, Transform)>();
        private LayerMask _collisionLayerMask = ~0;

        public readonly List<Vector3> collisionPoints;
        public bool enableDebugLogging = false;
        public float triangleThickness = 0.02f;

        #endregion

        #region Properties

        public LayerMask CollisionLayerMask
        {
            get => _collisionLayerMask;
            set => _collisionLayerMask = value;
        }

        #endregion

        #region Constructor

        public CollisionHandler()
        {
            collisionPoints = new List<Vector3>();

            _collisionStrategies = new List<ICollisionStrategy>
            {
                new TerrainCollisionStrategy(),
                new MeshCollisionStrategy(this),
                new ConvexCollisionStrategy()
            };
        }

        #endregion

        #region Public Methods

        public void ResolveCollisions(
            NodeManager nodeManager,
            Transform owner,
            Matter matterAsset,
            List<float> nodeMasses,
            float dt,
            float epsilon,
            bool visualizeForces)
        {
            if (!ValidateCollisionParameters(nodeManager, owner, nodeMasses))
                return;
            _frameCount++;
            if (_frameCount % 60 == 0) // Every 60 frames
            {
                var keysToRemove = _contactCache
                    .Where(kvp => _frameCount - kvp.Value.frameLastSeen > 5)
                    .Select(kvp => kvp.Key)
                    .ToList();

                foreach (var key in keysToRemove)
                    _contactCache.Remove(key);
            }

            collisionPoints.Clear();

            ResolveStaticCollisions(nodeManager, owner, matterAsset, nodeMasses,
                                   dt, epsilon, visualizeForces);

            ResolveCrossBodyCollisions(nodeManager, owner, matterAsset, nodeMasses,
                                       dt, epsilon, visualizeForces);
        }

        #endregion

        #region Private Validation Methods

        private bool ValidateCollisionParameters(NodeManager nodeManager,
                                                Transform owner,
                                                List<float> nodeMasses)
        {
            if (nodeManager?.Nodes == null)
            {
                if (enableDebugLogging)
                    Debug.LogError("[CollisionHandler] Invalid NodeManager");
                return false;
            }

            if (owner == null)
            {
                if (enableDebugLogging)
                    Debug.LogError("[CollisionHandler] Owner transform is null");
                return false;
            }

            if (nodeMasses == null || nodeMasses.Count != nodeManager.Nodes.Count)
            {
                if (enableDebugLogging)
                    Debug.LogError("[CollisionHandler] Node masses mismatch");
                return false;
            }

            return true;
        }

        #endregion

        #region Static Collision Methods
        private void ResolveStaticCollisions(NodeManager nodeManager, Transform owner,
                                            Matter matterAsset, List<float> nodeMasses,
                                            float dt, float epsilon, bool visualizeForces)
        {
            int nodeCount = nodeManager.Nodes.Count;

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i])
                    continue;

                Vector3 currentWorldPos = owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                Vector3 predictedWorldPos = nodeManager.PredictedPositions[i];
                float radius = nodeManager.GetNodeRadius(i);

                Vector3 sweepDir = predictedWorldPos - currentWorldPos;
                float sweepDist = sweepDir.magnitude;
                sweepDir /= sweepDist;

                int hitCount = Physics.SphereCastNonAlloc(
                    currentWorldPos,
                    radius,
                    sweepDir,
                    _sphereCastResults,
                    sweepDist,
                    _collisionLayerMask,
                    QueryTriggerInteraction.Ignore
                );

                float earliestHitTime = 1f;
                int earliestHitIndex = -1;

                for (int j = 0; j < hitCount; j++)
                {
                    Collider col = _sphereCastResults[j].collider;

                    if (IsOwnCollider(col, i, nodeManager))
                        continue;
                    if (IsFromIgnoredBody(col, owner))
                        continue;

                    float normalizedHitTime = _sphereCastResults[j].distance / sweepDist;
                    if (normalizedHitTime < earliestHitTime)
                    {
                        earliestHitTime = normalizedHitTime;
                        earliestHitIndex = j;
                    }
                }

                if (earliestHitIndex >= 0)
                {
                    RaycastHit hit = _sphereCastResults[earliestHitIndex];
                    Vector3 collisionPos = currentWorldPos + sweepDir * hit.distance;

                    float hitPenetration = radius - (hit.distance > 0 ? 0 : radius);

                    ApplyCollisionResolution(nodeManager, i, currentWorldPos,
                                            collisionPos, hit.point, hit.normal,
                                            hitPenetration, epsilon, dt, nodeMasses[i],
                                            matterAsset, visualizeForces, Color.red);
                    continue;
                }

                if (GetSweptCollisionInfo(currentWorldPos, predictedWorldPos, radius,
                                         out Collider fallbackCol,
                                         out Vector3 contactPoint,
                                         out Vector3 normal,
                                         out float penetration,
                                         out float hitTime, owner))
                {
                    if (!IsOwnCollider(fallbackCol, i, nodeManager))
                    {
                        Vector3 collisionPos = Vector3.Lerp(currentWorldPos, predictedWorldPos, hitTime);
                        ApplyCollisionResolution(nodeManager, i, currentWorldPos,
                                                collisionPos, contactPoint, normal,
                                                penetration, epsilon, dt, nodeMasses[i],
                                                matterAsset, visualizeForces, Color.yellow);
                    }
                }
            }
        }

        private bool GetSweptCollisionInfo(Vector3 startPos, Vector3 endPos, float radius,
                                          out Collider hitCollider,
                                          out Vector3 contactPoint, out Vector3 normal,
                                          out float penetration, out float hitTime, Transform ownerTransform)
        {
            hitCollider = null;
            contactPoint = Vector3.zero;
            normal = Vector3.up;
            penetration = 0f;
            hitTime = 1f;

            Collider[] overlaps = Physics.OverlapSphere(endPos, radius, _collisionLayerMask);

            foreach (var col in overlaps)
            {
                if (IsFromIgnoredBody(col, ownerTransform))
                    continue;
                foreach (var strategy in _collisionStrategies)
                {
                    if (strategy.GetSweptCollisionInfo(col, startPos, endPos, radius,
                                                       out contactPoint, out normal,
                                                       out penetration, out hitTime))
                    {
                        hitCollider = col;
                        return true;
                    }
                }
            }

            return false;
        }

        private void ApplyCollisionResolution(NodeManager nodeManager, int nodeIndex,
                                      Vector3 currentWorldPos, Vector3 collisionPos,
                                      Vector3 contactPoint, Vector3 normal,
                                      float penetration, float epsilon, float dt,
                                      float nodeMass, Matter matterAsset,
                                      bool visualizeForces, Color debugColor)
        {
            float invMass = 1f / nodeMass;
            if (invMass <= 0f) return;

            normal.Normalize();

            // 1. POSITION CORRECTION FIRST
            float correctionFactor = Mathf.Clamp01(PhysicsConstants.MAX_CORRECTION_FRACTION + penetration / epsilon);
            Vector3 positionCorrection = normal * (penetration + epsilon) * correctionFactor;
            float correctionMag = positionCorrection.magnitude;

            if (correctionMag > PhysicsConstants.MAX_ALLOWED_DISPLACEMENT)
            {
                positionCorrection *= PhysicsConstants.MAX_ALLOWED_DISPLACEMENT / correctionMag;
            }

            nodeManager.PredictedPositions[nodeIndex] += positionCorrection * invMass;

            // 2. GET VELOCITY (BEFORE damping)
            Vector3 vel = (nodeManager.PredictedPositions[nodeIndex] - currentWorldPos) / dt;

            // 3. APPLY COLLISION IMPULSE (bounce)
            float restitution = matterAsset?.Restitution ?? PhysicsConstants.DEFAULT_RESTITUTION;
            float normalVel = Vector3.Dot(vel, normal);

            if (normalVel < 0f) // Moving into surface
            {
                float effectiveRestitution = Mathf.Abs(normalVel) < 0.5f ? 0f : restitution;
                float jn = -(1f + effectiveRestitution) * normalVel / invMass;
                vel += normal * (jn * invMass);

                // 4. APPLY FRICTION (BEFORE damping)
                vel = ApplyFriction(vel, normal, jn, invMass, matterAsset);
            }

            // 5. APPLY DAMPING LAST
            float collisionDamping = matterAsset?.CollisionDamping ?? PhysicsConstants.DEFAULT_COLLISION_DAMPING;
            vel *= collisionDamping;

            // 6. UPDATE POSITION
            nodeManager.PredictedPositions[nodeIndex] = currentWorldPos + vel * dt;

            collisionPoints.Add(contactPoint);

            if (visualizeForces)
            {
                Debug.DrawRay(contactPoint, normal * PhysicsConstants.DEBUG_NORMAL_LENGTH, debugColor, 0.1f);
                Debug.DrawRay(contactPoint, vel * 0.1f, Color.yellow, 0.1f); // Visualize final velocity
            }
        }

        #endregion

        #region Cross-Body Collision Methods

        private void ResolveCrossBodyCollisions(NodeManager nodeManagerA, Transform ownerA,
                                                Matter matterAssetA, List<float> massesA,
                                                float dt, float epsilon, bool visualizeForces)
        {
            var allBodies = Object.FindObjectsOfType<SoftBody>();

            foreach (var bodyB in allBodies)
            {
                if (bodyB.transform == ownerA || !bodyB.enabled)
                    continue;

                if (ShouldIgnoreCollision(ownerA, bodyB.transform))
                    continue;

                NodeManager nodeManagerB = bodyB.solver.nodeManager;
                List<float> massesB = bodyB.solver.nodeMasses;

                if (nodeManagerB == null || massesB == null)
                    continue;

                Matter matterB = bodyB.matter;

                for (int i = 0; i < nodeManagerA.Nodes.Count; i++)
                {
                    if (nodeManagerA.IsPinned[i]) continue;

                    Vector3 predictedA = nodeManagerA.PredictedPositions[i];
                    float radiusA = nodeManagerA.GetNodeRadius(i);

                    for (int j = 0; j < nodeManagerB.Nodes.Count; j++)
                    {
                        if (nodeManagerB.IsPinned[j]) continue;

                        Vector3 predictedB = nodeManagerB.PredictedPositions[j];
                        float radiusB = nodeManagerB.GetNodeRadius(j);

                        Vector3 delta = predictedB - predictedA;
                        float dist = delta.magnitude;

                        if (dist > radiusA + radiusB + epsilon) continue;

                        float penetration = radiusA + radiusB - dist + epsilon;

                        Vector3 normal = dist > PhysicsConstants.MIN_COLLISION_DISTANCE
                            ? delta / dist
                            : Vector3.up;

                        ApplyCrossBodyCollision(nodeManagerA, nodeManagerB, i, j,
                                                predictedA, predictedB, normal, penetration,
                                                epsilon, dt, matterAssetA, matterB,
                                                massesA, massesB, visualizeForces);
                    }
                }
            }
        }

        private void ApplyCrossBodyCollision(NodeManager nodeManagerA, NodeManager nodeManagerB,
                                            int nodeA, int nodeB, Vector3 posA, Vector3 posB,
                                            Vector3 normal, float penetration, float epsilon,
                                            float dt, Matter matterA, Matter matterB,
                                            List<float> massesA, List<float> massesB,
                                            bool visualizeForces)
        {
            float wA = nodeManagerA.IsPinned[nodeA] ? 0f : 1f / massesA[nodeA];
            float wB = nodeManagerB.IsPinned[nodeB] ? 0f : 1f / massesB[nodeB];
            float wSum = wA + wB;

            if (wSum <= 0f) return;

            float correctionFactor = Mathf.Clamp01(PhysicsConstants.MAX_CORRECTION_FRACTION +
                                                   penetration / epsilon);

            float correctionMagnitude = penetration * correctionFactor / wSum;
            Vector3 correction = normal * correctionMagnitude;

            if (!nodeManagerA.IsPinned[nodeA])
                nodeManagerA.PredictedPositions[nodeA] -= correction * wA;

            if (!nodeManagerB.IsPinned[nodeB])
                nodeManagerB.PredictedPositions[nodeB] += correction * wB;

            Vector3 velA = (nodeManagerA.PredictedPositions[nodeA] - posA) / dt;
            Vector3 velB = (nodeManagerB.PredictedPositions[nodeB] - posB) / dt;
            Vector3 relativeVel = velB - velA;

            float normalVel = Vector3.Dot(relativeVel, normal);
            if (normalVel >= 0f) return;

            float restitution = Mathf.Min(matterA.Restitution, matterB.Restitution);
            float effectiveRestitution = Mathf.Abs(normalVel) < 0.5f ? 0f : restitution;

            float invMassA = nodeManagerA.IsPinned[nodeA] ? 0f : 1f / massesA[nodeA];
            float invMassB = nodeManagerB.IsPinned[nodeB] ? 0f : 1f / massesB[nodeB];
            float reducedInvMass = invMassA + invMassB;

            if (reducedInvMass <= 0f) return;

            float jn = -(1f + effectiveRestitution) * normalVel / reducedInvMass;

            Vector3 impulse = -normal * jn;  // Flipped sign here for correct separation direction

            if (!nodeManagerA.IsPinned[nodeA])
                nodeManagerA.PredictedPositions[nodeA] = posA + (velA + impulse * invMassA) * dt;

            if (!nodeManagerB.IsPinned[nodeB])
                nodeManagerB.PredictedPositions[nodeB] = posB + (velB - impulse * invMassB) * dt;

            ApplyCrossBodyFriction(nodeA, nodeB, normal, jn, relativeVel,
                                   matterA, matterB, nodeManagerA, nodeManagerB,
                                   massesA, massesB, dt);

            if (visualizeForces)
            {
                Vector3 contact = (posA + posB) * 0.5f;
                Debug.DrawRay(contact, normal * PhysicsConstants.DEBUG_NORMAL_LENGTH, Color.green, 0.1f);
            }
        }

        #endregion

        #region Friction Methods for Cross-Body

        private void ApplyCrossBodyFriction(int nodeA, int nodeB, Vector3 normal,
                                           float jn, Vector3 relativeVel,
                                           Matter matterA, Matter matterB,
                                           NodeManager nodeManagerA, NodeManager nodeManagerB,
                                           List<float> massesA, List<float> massesB, float dt)
        {
            float staticFriction = Mathf.Max(matterA.StaticFriction, matterB.StaticFriction);
            float slidingFriction = Mathf.Max(matterA.SlidingFriction, matterB.SlidingFriction);

            Vector3 velNormal = Vector3.Dot(relativeVel, normal) * normal;
            Vector3 velTangent = relativeVel - velNormal;
            float tangentSpeed = velTangent.magnitude;

            if (tangentSpeed < PhysicsConstants.MIN_TANGENT_SPEED) return;

            Vector3 tangentDir = velTangent / tangentSpeed;

            float invMassA = nodeManagerA.IsPinned[nodeA] ? 0f : 1f / massesA[nodeA];
            float invMassB = nodeManagerB.IsPinned[nodeB] ? 0f : 1f / massesB[nodeB];
            float invMassSum = invMassA + invMassB;

            if (invMassSum <= 0f) return;

            float jt = tangentSpeed / invMassSum;

            float maxStatic = staticFriction * Mathf.Abs(jn);

            Vector3 frictionImpulse;

            if (jt <= maxStatic)
            {
                // Static Friction: Stop relative tangential motion
                frictionImpulse = velTangent / invMassSum;
            }
            else
            {
                // Dynamic Friction: Apply opposing impulse
                frictionImpulse = tangentDir * (slidingFriction * Mathf.Abs(jn));
            }

            if (!nodeManagerA.IsPinned[nodeA])
                nodeManagerA.PredictedPositions[nodeA] += frictionImpulse * invMassA * dt;

            if (!nodeManagerB.IsPinned[nodeB])
                nodeManagerB.PredictedPositions[nodeB] -= frictionImpulse * invMassB * dt;
        }

        #endregion

        #region Mesh Collision Methods

        public bool CheckSweptSphereTriangle(Vector3 sphereStart, Vector3 sphereEnd, float radius,
                                             Vector3 velocityDir, float velocityMag,
                                             Vector3 v0, Vector3 v1, Vector3 v2,
                                             out Vector3 contact, out Vector3 normal,
                                             out float penetration, out float collisionTime)
        {
            contact = Vector3.zero;
            normal = Vector3.zero;
            penetration = 0f;
            collisionTime = 1f;

            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;
            normal = Vector3.Cross(edge1, edge2).normalized;

            float planeD = -Vector3.Dot(normal, v0);

            if (CheckSweptSpherePlane(sphereStart, sphereEnd, normal, planeD, velocityDir, velocityMag,
                                      out float planeT, out Vector3 intersection))
            {
                float effectiveRadius = radius + triangleThickness;

                Vector3 sweptCenter = sphereStart + velocityDir * (planeT * velocityMag);
                float distToPlane = Vector3.Dot(sweptCenter, normal) + planeD;

                if (Mathf.Abs(distToPlane) > effectiveRadius) return false;

                Vector3 projected = sweptCenter - normal * distToPlane;

                if (IsPointInTriangle(projected, v0, v1, v2))
                {
                    contact = projected;
                    penetration = effectiveRadius - Mathf.Abs(distToPlane);
                    if (distToPlane < 0) normal = -normal;
                    collisionTime = planeT;
                    return true;
                }
            }

            return CheckSweptSphereTriangleBoundary(sphereStart, sphereEnd, radius, velocityDir, velocityMag,
                                                   v0, v1, v2, out contact, out normal, out penetration, out collisionTime);
        }

        private bool CheckSweptSpherePlane(Vector3 segStart, Vector3 segEnd,
                                          Vector3 planeNormal, float planeD,
                                          Vector3 velocityDir, float velocityMag,
                                          out float t, out Vector3 intersection)
        {
            t = 0f;
            intersection = Vector3.zero;

            Vector3 velocity = segEnd - segStart;
            float denom = Vector3.Dot(planeNormal, velocityDir);

            if (Mathf.Abs(denom) < 0.0001f)
                return false;

            t = -(Vector3.Dot(planeNormal, segStart) + planeD) / denom;

            if (t < 0f || t > 1f)
                return false;

            intersection = segStart + velocityDir * (t * velocityMag);
            return true;
        }

        private bool CheckSweptSphereTriangleBoundary(Vector3 sphereStart, Vector3 sphereEnd,
                                                      float radius, Vector3 velocityDir, float velocityMag,
                                                      Vector3 v0, Vector3 v1, Vector3 v2,
                                                      out Vector3 contact, out Vector3 normal,
                                                      out float penetration, out float collisionTime)
        {
            contact = Vector3.zero;
            normal = Vector3.zero;
            penetration = 0f;
            collisionTime = 1f;

            float bestT = 2f;
            Vector3 bestContact = Vector3.zero;
            Vector3 bestNormal = Vector3.zero;

            CheckSweptSphereEdge(sphereStart, velocityDir, velocityMag, radius, v0, v1,
                               ref bestT, ref bestContact, ref bestNormal);
            CheckSweptSphereEdge(sphereStart, velocityDir, velocityMag, radius, v1, v2,
                               ref bestT, ref bestContact, ref bestNormal);
            CheckSweptSphereEdge(sphereStart, velocityDir, velocityMag, radius, v2, v0,
                               ref bestT, ref bestContact, ref bestNormal);

            CheckSweptSphereVertex(sphereStart, velocityDir, velocityMag, radius, v0,
                                 ref bestT, ref bestContact, ref bestNormal);
            CheckSweptSphereVertex(sphereStart, velocityDir, velocityMag, radius, v1,
                                 ref bestT, ref bestContact, ref bestNormal);
            CheckSweptSphereVertex(sphereStart, velocityDir, velocityMag, radius, v2,
                                 ref bestT, ref bestContact, ref bestNormal);

            if (bestT <= 1f)
            {
                contact = bestContact;
                normal = bestNormal;
                collisionTime = bestT;
                penetration = Mathf.Max(0f, radius * 0.1f);
                return true;
            }

            return false;
        }

        private void CheckSweptSphereEdge(Vector3 sphereStart, Vector3 velocityDir, float velocityMag,
                                         float radius, Vector3 edgeA, Vector3 edgeB,
                                         ref float bestT, ref Vector3 bestContact, ref Vector3 bestNormal)
        {
            Vector3 edgeVec = edgeB - edgeA;
            Vector3 edgeToSphere = sphereStart - edgeA;

            float a = velocityMag * velocityMag - Vector3.Dot(velocityDir, edgeVec) * Vector3.Dot(velocityDir, edgeVec);
            float b = 2f * (velocityMag * Vector3.Dot(edgeToSphere, velocityDir) -
                           Vector3.Dot(edgeToSphere, edgeVec) * Vector3.Dot(velocityDir, edgeVec));
            float c = edgeToSphere.sqrMagnitude - Vector3.Dot(edgeToSphere, edgeVec) * Vector3.Dot(edgeToSphere, edgeVec) - radius * radius;

            float discriminant = b * b - 4f * a * c;
            if (discriminant < 0f || Mathf.Abs(a) < 0.0001f)
                return;

            float t = (-b - Mathf.Sqrt(discriminant)) / (2f * a);
            if (t < 0f || t > 1f || t >= bestT)
                return;

            Vector3 spherePos = sphereStart + velocityDir * (t * velocityMag);
            Vector3 closestOnEdge = ClosestPointOnSegment(spherePos, edgeA, edgeB);

            float distSq = (spherePos - closestOnEdge).sqrMagnitude;
            if (distSq <= radius * radius + 0.001f)
            {
                bestT = t;
                bestContact = closestOnEdge;
                bestNormal = (spherePos - closestOnEdge).normalized;
            }
        }

        private void CheckSweptSphereVertex(Vector3 sphereStart, Vector3 velocityDir, float velocityMag,
                                           float radius, Vector3 vertex,
                                           ref float bestT, ref Vector3 bestContact, ref Vector3 bestNormal)
        {
            Vector3 toVertex = vertex - sphereStart;
            float b = Vector3.Dot(toVertex, velocityDir);
            float c = toVertex.sqrMagnitude - radius * radius;

            if (c <= 0f)
            {
                bestT = 0f;
                bestContact = vertex;
                bestNormal = (sphereStart - vertex).normalized;
                return;
            }

            if (b < 0f)
                return;

            float discriminant = b * b - c;
            if (discriminant < 0f)
                return;

            float t = (b - Mathf.Sqrt(discriminant)) / velocityMag;
            if (t >= 0f && t <= 1f && t < bestT)
            {
                bestT = t;
                bestContact = vertex;
                Vector3 spherePos = sphereStart + velocityDir * (t * velocityMag);
                bestNormal = (spherePos - vertex).normalized;
            }
        }

        #endregion

        #region Standard Sphere-Triangle Collision Detection

        public bool CheckSphereTriangleCollision(Vector3 spherePos, float radius,
                                                 Vector3 v0, Vector3 v1, Vector3 v2,
                                                 out Vector3 contact, out Vector3 normal,
                                                 out float penetration)
        {
            contact = Vector3.zero;
            normal = Vector3.zero;
            penetration = 0f;

            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;
            normal = Vector3.Cross(edge1, edge2).normalized;

            float effectiveRadius = radius + triangleThickness;

            float dist = Vector3.Dot(spherePos - v0, normal);

            if (Mathf.Abs(dist) > effectiveRadius)
                return false;

            Vector3 projected = spherePos - normal * dist;

            if (IsPointInTriangle(projected, v0, v1, v2))
            {
                contact = projected;
                penetration = effectiveRadius - Mathf.Abs(dist);
                if (dist < 0) normal = -normal;
                return true;
            }

            contact = ClosestPointOnTriangleBoundary(spherePos, v0, v1, v2);
            float distSq = (spherePos - contact).sqrMagnitude;

            if (distSq < radius * radius)
            {
                float d = Mathf.Sqrt(distSq);
                normal = d > PhysicsConstants.MIN_COLLISION_DISTANCE
                    ? (spherePos - contact) / d
                    : normal;
                penetration = radius - d;
                return true;
            }

            return false;
        }

        private bool IsPointInTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 v0 = b - a;
            Vector3 v1 = c - a;
            Vector3 v2 = p - a;

            float d00 = Vector3.Dot(v0, v0);
            float d01 = Vector3.Dot(v0, v1);
            float d11 = Vector3.Dot(v1, v1);
            float d20 = Vector3.Dot(v2, v0);
            float d21 = Vector3.Dot(v2, v1);

            float denom = d00 * d11 - d01 * d01;

            if (Mathf.Abs(denom) < PhysicsConstants.CONSTRAINT_EPSILON)
                return false;

            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;

            return (v >= 0) && (w >= 0) && (v + w <= 1);
        }

        private Vector3 ClosestPointOnTriangleBoundary(Vector3 p, Vector3 v0, Vector3 v1, Vector3 v2)
        {
            Vector3 p1 = ClosestPointOnSegment(p, v0, v1);
            Vector3 p2 = ClosestPointOnSegment(p, v1, v2);
            Vector3 p3 = ClosestPointOnSegment(p, v2, v0);

            float d1 = (p - p1).sqrMagnitude;
            float d2 = (p - p2).sqrMagnitude;
            float d3 = (p - p3).sqrMagnitude;

            if (d1 < d2 && d1 < d3) return p1;
            return (d2 < d3) ? p2 : p3;
        }

        private Vector3 ClosestPointOnSegment(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float t = Vector3.Dot(p - a, ab) / Mathf.Max(Vector3.Dot(ab, ab),
                                                         PhysicsConstants.CONSTRAINT_EPSILON);
            return a + Mathf.Clamp01(t) * ab;
        }

        #endregion

        #region Collision Impulse Methods
        private Vector3 ApplyCollisionImpulse(Vector3 vel, Vector3 normal, float invMass, Matter matterAsset)
        {
            if (invMass <= 0f) return vel;

            float restitution = matterAsset?.Restitution ?? PhysicsConstants.DEFAULT_RESTITUTION;
            normal.Normalize();

            float normalVel = Vector3.Dot(vel, normal);
            if (normalVel >= 0f) return vel; // Already separating

            // Calculate Normal Impulse (Bounce)
            float effectiveRestitution = Mathf.Abs(normalVel) < 0.5f ? 0f : restitution;
            float jn = -(1f + effectiveRestitution) * normalVel / invMass;

            // Apply bounce to velocity
            vel += normal * (jn * invMass);

            // Apply Friction using the new method
            return ApplyFriction(vel, normal, jn, invMass, matterAsset);
        }


        #endregion

        #region Debug Visualization Methods

        private void DrawDebugCross(Vector3 center, float size, Color color, float duration)
        {
            Debug.DrawLine(center - Vector3.left * size, center + Vector3.left * size,
                          color, duration);
            Debug.DrawLine(center - Vector3.up * size, center + Vector3.up * size,
                          color, duration);
            Debug.DrawLine(center - Vector3.forward * size, center + Vector3.forward * size,
                          color, duration);
        }

        #endregion

        #region Helper Methods
        private Vector3 ApplyFriction(Vector3 velocity, Vector3 normal, float jn, float invMass, Matter matter)
        {
            float staticFriction = matter != null ? matter.StaticFriction : PhysicsConstants.DEFAULT_STATIC_FRICTION;
            float slidingFriction = matter != null ? matter.SlidingFriction : PhysicsConstants.DEFAULT_DYNAMIC_FRICTION;

            // Separate velocity into normal and tangential components
            Vector3 velNormal = Vector3.Dot(velocity, normal) * normal;
            Vector3 velTangent = velocity - velNormal;
            float tangentSpeed = velTangent.magnitude;

            if (tangentSpeed < PhysicsConstants.MIN_TANGENT_SPEED)
                return velocity;

            Vector3 tangentDir = velTangent / tangentSpeed;

            // Calculate friction impulse magnitude using Coulomb's law
            float maxStaticImpulse = staticFriction * Mathf.Abs(jn);
            float jt = tangentSpeed / invMass; // Impulse needed to stop tangential motion

            Vector3 frictionImpulse;

            if (jt <= maxStaticImpulse)
            {
                // Static friction: stop completely
                frictionImpulse = -velTangent / invMass;
            }
            else
            {
                // Dynamic friction: sliding
                float dynamicImpulse = slidingFriction * Mathf.Abs(jn);
                frictionImpulse = -tangentDir * dynamicImpulse;
            }

            // Apply the friction impulse (MULTIPLY by invMass, not divide)
            return velocity + frictionImpulse * invMass;
        }
        private bool IsOwnCollider(Collider col, int nodeIndex, NodeManager nodeManager)
        {
            if (col == nodeManager.Colliders[nodeIndex] ||
                col.transform == nodeManager.Nodes[nodeIndex])
                return true;

            return nodeManager.FindIndex(n => n != null && n == col.transform) != -1;
        }
        public void SetIgnoreCollision(Transform bodyA, Transform bodyB, bool ignore)
        {
            if (bodyA == null || bodyB == null) return;

            // Ensure consistent ordering (smaller instance ID first)
            Transform first = bodyA.GetInstanceID() < bodyB.GetInstanceID() ? bodyA : bodyB;
            Transform second = bodyA.GetInstanceID() < bodyB.GetInstanceID() ? bodyB : bodyA;

            var pair = (first, second);

            if (ignore)
            {
                _ignoredBodyPairs.Add(pair);
                if (enableDebugLogging)
                    Debug.Log($"[CollisionHandler] Added ignored pair: {bodyA.name} <-> {bodyB.name}");
            }
            else
            {
                _ignoredBodyPairs.Remove(pair);
                if (enableDebugLogging)
                    Debug.Log($"[CollisionHandler] Removed ignored pair: {bodyA.name} <-> {bodyB.name}");
            }
        }
        private bool ShouldIgnoreCollision(Transform bodyA, Transform bodyB)
        {
            if (bodyA == null || bodyB == null) return false;

            // Ensure consistent ordering (smaller instance ID first)
            Transform first = bodyA.GetInstanceID() < bodyB.GetInstanceID() ? bodyA : bodyB;
            Transform second = bodyA.GetInstanceID() < bodyB.GetInstanceID() ? bodyB : bodyA;

            return _ignoredBodyPairs.Contains((first, second));
        }
        private bool IsFromIgnoredBody(Collider col, Transform ownerBody)
        {
            if (col == null || ownerBody == null) return false;

            // Check if the collider belongs to any SoftBody
            Transform colliderRoot = col.transform;
            while (colliderRoot.parent != null)
            {
                SoftBody sb = colliderRoot.GetComponent<SoftBody>();
                if (sb != null)
                {
                    // Check if this body forms an ignored pair with our owner
                    return ShouldIgnoreCollision(ownerBody, sb.transform);
                }
                colliderRoot = colliderRoot.parent;
            }

            return false;
        }
        private Vector3 GetBarycentric(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 v0 = b - a, v1 = c - a, v2 = p - a;
            float d00 = Vector3.Dot(v0, v0);
            float d01 = Vector3.Dot(v0, v1);
            float d11 = Vector3.Dot(v1, v1);
            float d20 = Vector3.Dot(v2, v0);
            float d21 = Vector3.Dot(v2, v1);
            float denom = d00 * d11 - d01 * d01;
            if (Mathf.Abs(denom) < 1e-6f) return new Vector3(1, 0, 0);
            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1.0f - v - w;
            return new Vector3(u, v, w);
        }

        #endregion
    }
}