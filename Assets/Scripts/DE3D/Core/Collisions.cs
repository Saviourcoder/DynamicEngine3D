/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D - FIXED FRICTION VERSION                 ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using System.Collections.Generic;
using System.Linq;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

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
    }
    struct CombinedMatterProps
    {
        public float StaticFriction;
        public float SlidingFriction;
    }

    public class CollisionHandler
    {
        #region Fields

        private readonly Dictionary<int, ContactCache> _contactCache = new Dictionary<int, ContactCache>();
        private int _frameCount = 0;
        private readonly List<ICollisionStrategy> _collisionStrategies;
        private readonly HashSet<(int, int)> _ignoredBodyPairs = new HashSet<(int, int)>();
        private LayerMask _collisionLayerMask = ~0;
        private readonly Dictionary<int, List<int>> _spatialHash = new Dictionary<int, List<int>>();
        private readonly List<int> _nearbyKeys = new List<int>();
        private const float SPATIAL_CELL_SIZE = 0.15f;

        // Face-to-node spatial acceleration (coarser grid than node-node)
        private readonly Dictionary<int, List<int>> _faceNodeHash = new Dictionary<int, List<int>>();
        private readonly List<int> _faceNodeNearbyKeys = new List<int>();
        private readonly List<int> _faceNodeCandidates = new List<int>();
        private bool[] _nodeVisitedFlags = System.Array.Empty<bool>();
        private const float FACE_NODE_CELL_SIZE = 0.3f;

        public readonly List<Vector3> collisionPoints;
        public bool enableDebugLogging = false;
        public float triangleThickness = 0.02f;

        private const float SKIN_WIDTH = 0.005f;
        private SoftBody[] _cachedBodies;
        private float _lastCacheTime;

        private List<Collider> _customColliderCache = new List<Collider>();

        // Static collider caching
        private Collider[] _staticColliders;
        private float _lastStaticColliderRefreshTime;
        private const float STATIC_COLLIDER_REFRESH_INTERVAL = 1.0f;

        private const float MIN_EFFECTIVE_SUBSTEP = 0.001f;

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
        }
        #endregion

        #region Main Resolve Method

        public void ResolveCollisions(NodeManager nodeManager, Transform owner, Matter matterAsset, List<float> nodeMasses, float dt, float epsilon, bool visualizeForces)
        {
            if (!ValidateCollisionParameters(nodeManager, owner, nodeMasses)) return;

            _frameCount++;
            if (_frameCount % 60 == 0) CleanCache();

            collisionPoints.Clear();

            ResolveCrossBodyCollisions(nodeManager, owner, matterAsset, nodeMasses, dt, epsilon, visualizeForces);

            ResolveStaticCollisions(nodeManager, owner, matterAsset, nodeMasses, dt, epsilon, visualizeForces);
        }
        private void CleanCache()
        {
            var keysToRemove = _contactCache.Where(kvp => _frameCount - kvp.Value.frameLastSeen > 5).Select(kvp => kvp.Key).ToList();
            foreach (var key in keysToRemove) _contactCache.Remove(key);
        }

        #endregion

        #region Broad & Narrow Phase Cross-Body

        private void ResolveCrossBodyCollisions(NodeManager nodeManagerA, Transform ownerA, Matter matterAssetA, List<float> massesA, float dt, float epsilon, bool visualizeForces)
        {
            if (_cachedBodies == null || Time.time - _lastCacheTime > 1.0f)
            {
                _cachedBodies = Object.FindObjectsByType<SoftBody>(FindObjectsSortMode.None);
                _lastCacheTime = Time.time;
            }

            Bounds boundsA = CalculateBodyBounds(nodeManagerA);
            float radiusA = nodeManagerA.GetNodeRadius();

            int ownerAId = ownerA.GetInstanceID();

            foreach (var bodyB in _cachedBodies)
            {
                if (bodyB == null || bodyB.transform == ownerA || !bodyB.enabled) continue;

                int ownerBId = bodyB.transform.GetInstanceID();
                var pair = ownerAId < ownerBId ? (ownerAId, ownerBId) : (ownerBId, ownerAId);
                if (_ignoredBodyPairs.Contains(pair)) continue;

                if (((1 << bodyB.gameObject.layer) & _collisionLayerMask) == 0) continue;

                NodeManager nodeManagerB = bodyB.solver.nodeManager;
                if (nodeManagerB == null) continue;

                Bounds boundsB = CalculateBodyBounds(nodeManagerB);
                boundsB.Expand(radiusA * 2f + epsilon);

                if (!boundsA.Intersects(boundsB)) continue;

                ResolveNodeToNodeOptimized(nodeManagerA, nodeManagerB, massesA, bodyB.solver.nodeMasses, matterAssetA, bodyB.matter, dt, epsilon, visualizeForces);
                ResolveFaceToNode(nodeManagerA, massesA, bodyB, nodeManagerB, bodyB.solver.nodeMasses, matterAssetA, bodyB.matter, dt, epsilon, visualizeForces);

                SoftBody bodyA = ownerA.GetComponent<SoftBody>();
                if (bodyA != null && bodyA.truss != null)
                {
                    ResolveFaceToNode(nodeManagerB, bodyB.solver.nodeMasses, bodyA, nodeManagerA, massesA, bodyB.matter, matterAssetA, dt, epsilon, visualizeForces);
                }
            }
        }

        private Bounds CalculateBodyBounds(NodeManager nm)
        {
            int count = nm.PredictedPositions.Count;
            if (count == 0) return new Bounds();

            Vector3 min = nm.PredictedPositions[0];
            Vector3 max = nm.PredictedPositions[0];

            for (int i = 1; i < count; i++)
            {
                Vector3 p = nm.PredictedPositions[i];
                if (p.x < min.x) min.x = p.x; else if (p.x > max.x) max.x = p.x;
                if (p.y < min.y) min.y = p.y; else if (p.y > max.y) max.y = p.y;
                if (p.z < min.z) min.z = p.z; else if (p.z > max.z) max.z = p.z;
            }

            Bounds b = new Bounds();
            b.SetMinMax(min, max);
            return b;
        }

        #endregion

        #region Custom Static Collision Detection with CCD

        private void ResolveStaticCollisions(NodeManager nodeManager, Transform owner,
                                            Matter softBodyMatter, List<float> nodeMasses,
                                            float dt, float epsilon, bool visualizeForces)
        {
            float effectiveDt = Mathf.Max(dt, MIN_EFFECTIVE_SUBSTEP);

            int nodeCount = nodeManager.Nodes.Count;

            // Refresh collider cache periodically, or on first use when still null
            if (_staticColliders == null || Time.time - _lastStaticColliderRefreshTime > STATIC_COLLIDER_REFRESH_INTERVAL)
            {
                RefreshStaticColliderCache();
                _lastStaticColliderRefreshTime = Time.time;
            }

            for (int i = 0; i < nodeCount; i++)
            {
                Vector3 startPos = nodeManager.Nodes[i].position;
                Vector3 endPos = nodeManager.PredictedPositions[i];
                float radius = nodeManager.GetNodeRadius() + SKIN_WIDTH;

                Vector3 currentPos = startPos;
                Vector3 remainingDisplacement = endPos - startPos;
                float remainingTime = 1f;
                int maxIterations = 8;

                for (int iteration = 0; iteration < maxIterations && remainingTime > 0.001f; iteration++)
                {
                    Vector3 targetPos = currentPos + remainingDisplacement;
                    float sweepDistance = remainingDisplacement.magnitude;

                    if (sweepDistance < epsilon * 0.1f) break;

                    bool hitSomething = PerformAnalyticalSweptTest(
                        currentPos, targetPos, radius, owner, nodeManager.Colliders[i],
                        out Vector3 hitPoint, out Vector3 hitNormal, out float hitTime, out Collider hitCollider
                    );

                    if (hitSomething)
                    {
                        float actualDistance = sweepDistance * hitTime;
                        Vector3 collisionPos = currentPos + remainingDisplacement.normalized * actualDistance;

                        float pushDistance = epsilon * 2f;
                        nodeManager.PredictedPositions[i] = collisionPos + hitNormal * pushDistance;

                        float penetration = radius - actualDistance;
                        penetration = Mathf.Max(penetration, epsilon);

                        ApplyCollisionResponseCCD(i, nodeManager, softBodyMatter, nodeMasses,
                            hitPoint, hitNormal, penetration, hitTime, effectiveDt, epsilon, visualizeForces);

                        currentPos = nodeManager.PredictedPositions[i];
                        float remainingFraction = 1f - hitTime;
                        Vector3 projectedDisplacement = remainingDisplacement * remainingFraction;
                        projectedDisplacement = projectedDisplacement - Vector3.Dot(projectedDisplacement, hitNormal) * hitNormal;

                        remainingDisplacement = projectedDisplacement;
                        remainingTime *= remainingFraction;

                        collisionPoints.Add(hitPoint);

                        if (visualizeForces)
                        {
                            Debug.DrawLine(hitPoint, hitPoint + hitNormal * 0.2f, Color.red, dt);
                            Debug.DrawLine(startPos, currentPos, Color.green, dt);
                        }
                    }
                    else
                    {
                        nodeManager.PredictedPositions[i] = targetPos;
                        break;
                    }
                }
            }
        }

        private void RefreshStaticColliderCache()
        {
            _staticColliders = Object.FindObjectsByType<Collider>(FindObjectsSortMode.None)
                .Where(c => c != null &&
                       !c.isTrigger &&
                       ((1 << c.gameObject.layer) & _collisionLayerMask) != 0 &&
                       (c.attachedRigidbody == null || c.attachedRigidbody.isKinematic))
                .ToArray();
        }

        private bool PerformAnalyticalSweptTest(Vector3 startPos, Vector3 endPos, float radius, Transform owner,
                                                SphereCollider nodeCollider, out Vector3 hitPoint, out Vector3 hitNormal,
                                                out float hitTime, out Collider hitCollider)
        {
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;
            hitTime = float.MaxValue;
            hitCollider = null;

            Vector3 sweepVelocity = endPos - startPos;
            float sweepDistance = sweepVelocity.magnitude;

            if (sweepDistance < 0.0001f)
            {
                hitTime = 0f;
                return false;
            }

            bool foundHit = false;
            float earliestTime = float.MaxValue;

            foreach (var col in _staticColliders)
            {
                if (col == null || col.transform == owner) continue;
                if (nodeCollider != null && col == nodeCollider) continue;

                float t;
                Vector3 point, normal;

                if (col is SphereCollider sphere)
                {
                    if (SweptSphereSphere(startPos, sweepVelocity, radius, sphere, out t, out point, out normal))
                    {
                        if (t < earliestTime && t >= 0f && t <= 1f)
                        {
                            earliestTime = t;
                            hitPoint = point;
                            hitNormal = normal;
                            hitCollider = col;
                            foundHit = true;
                        }
                    }
                }
                else if (col is BoxCollider box)
                {
                    if (SweptSphereBox(startPos, sweepVelocity, radius, box, out t, out point, out normal))
                    {
                        if (t < earliestTime && t >= 0f && t <= 1f)
                        {
                            earliestTime = t;
                            hitPoint = point;
                            hitNormal = normal;
                            hitCollider = col;
                            foundHit = true;
                        }
                    }
                }
                else if (col is CapsuleCollider capsule)
                {
                    if (SweptSphereCapsule(startPos, sweepVelocity, radius, capsule, out t, out point, out normal))
                    {
                        if (t < earliestTime && t >= 0f && t <= 1f)
                        {
                            earliestTime = t;
                            hitPoint = point;
                            hitNormal = normal;
                            hitCollider = col;
                            foundHit = true;
                        }
                    }
                }
                else if (col is TerrainCollider terrain)
                {
                    if (SweptSphereTerrain(startPos, sweepVelocity, radius, terrain, out t, out point, out normal))
                    {
                        if (t < earliestTime && t >= 0f && t <= 1f)
                        {
                            earliestTime = t;
                            hitPoint = point;
                            hitNormal = normal;
                            hitCollider = col;
                            foundHit = true;
                        }
                    }
                }
                else if (col is MeshCollider meshCol)
                {
                    if (SweptSphereMesh(startPos, sweepVelocity, radius, meshCol, out t, out point, out normal))
                    {
                        if (t < earliestTime && t >= 0f && t <= 1f)
                        {
                            earliestTime = t;
                            hitPoint = point;
                            hitNormal = normal;
                            hitCollider = col;
                            foundHit = true;
                        }
                    }
                }
            }

            if (foundHit)
            {
                hitTime = earliestTime;
                return true;
            }

            return false;
        }

        #endregion

        #region Analytical Swept Collision Tests
        private bool SweptSphereSphere(Vector3 startPos, Vector3 velocity, float radiusA, SphereCollider sphereB,
                                       out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 centerB = sphereB.transform.TransformPoint(sphereB.center);
            float radiusB = sphereB.radius * Mathf.Max(sphereB.transform.lossyScale.x, sphereB.transform.lossyScale.y, sphereB.transform.lossyScale.z);
            float combinedRadius = radiusA + radiusB;

            Vector3 p = startPos - centerB;
            Vector3 v = velocity;

            float a = Vector3.Dot(v, v);
            float b = 2f * Vector3.Dot(p, v);
            float c = Vector3.Dot(p, p) - combinedRadius * combinedRadius;

            if (a < 0.0001f) return false;

            float discriminant = b * b - 4f * a * c;
            if (discriminant < 0) return false;

            float sqrtDisc = Mathf.Sqrt(discriminant);
            float t1 = (-b - sqrtDisc) / (2f * a);
            float t2 = (-b + sqrtDisc) / (2f * a);

            float t = t1 >= 0f ? t1 : t2;
            if (t < 0f || t > 1f) return false;

            hitTime = t;
            Vector3 posAtHit = startPos + velocity * t;
            hitNormal = (posAtHit - centerB).normalized;
            hitPoint = centerB + hitNormal * radiusB;

            return true;
        }

        private bool SweptSphereBox(Vector3 startPos, Vector3 velocity, float radius, BoxCollider box,
                                    out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 localStart = box.transform.InverseTransformPoint(startPos);
            Vector3 localEnd = box.transform.InverseTransformPoint(startPos + velocity);
            Vector3 localVelocity = localEnd - localStart;

            float localRadius = radius / Mathf.Max(box.transform.lossyScale.x, box.transform.lossyScale.y, box.transform.lossyScale.z);
            Vector3 boxMin = box.center - box.size * 0.5f - Vector3.one * localRadius;
            Vector3 boxMax = box.center + box.size * 0.5f + Vector3.one * localRadius;

            float tMin = 0f, tMax = 1f;
            int hitAxis = -1, hitSign = 0;

            for (int axis = 0; axis < 3; axis++)
            {
                float p = axis == 0 ? localStart.x : (axis == 1 ? localStart.y : localStart.z);
                float v = axis == 0 ? localVelocity.x : (axis == 1 ? localVelocity.y : localVelocity.z);
                float min = axis == 0 ? boxMin.x : (axis == 1 ? boxMin.y : boxMin.z);
                float max = axis == 0 ? boxMax.x : (axis == 1 ? boxMax.y : boxMax.z);

                if (Mathf.Abs(v) < 0.0001f)
                {
                    if (p < min || p > max) return false;
                }
                else
                {
                    float t1 = (min - p) / v;
                    float t2 = (max - p) / v;
                    if (t1 > t2) { float temp = t1; t1 = t2; t2 = temp; }
                    if (t1 > tMin) { tMin = t1; hitAxis = axis; hitSign = v > 0 ? -1 : 1; }
                    if (t2 < tMax) tMax = t2;
                    if (tMin > tMax) return false;
                }
            }

            if (tMin < 0f || tMin > 1f) return false;
            hitTime = tMin;

            Vector3 localNormal = Vector3.zero;
            if (hitAxis == 0) localNormal = new Vector3(hitSign, 0, 0);
            else if (hitAxis == 1) localNormal = new Vector3(0, hitSign, 0);
            else localNormal = new Vector3(0, 0, hitSign);

            hitNormal = box.transform.TransformDirection(localNormal).normalized;
            Vector3 localHitPos = localStart + localVelocity * hitTime;
            hitPoint = box.transform.TransformPoint(localHitPos);

            return true;
        }

        private bool SweptSphereCapsule(Vector3 startPos, Vector3 velocity, float radius, CapsuleCollider capsule,
                                       out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 capsuleCenter = capsule.transform.TransformPoint(capsule.center);
            float capsuleRadius = capsule.radius * Mathf.Max(capsule.transform.lossyScale.x, capsule.transform.lossyScale.z);
            float capsuleHeight = capsule.height * capsule.transform.lossyScale.y;

            Vector3 axis = Vector3.up;
            if (capsule.direction == 0) axis = Vector3.right;
            else if (capsule.direction == 2) axis = Vector3.forward;
            axis = capsule.transform.TransformDirection(axis);

            float halfHeight = Mathf.Max(0, (capsuleHeight * 0.5f) - capsuleRadius);
            Vector3 p1 = capsuleCenter + axis * halfHeight;
            Vector3 p2 = capsuleCenter - axis * halfHeight;

            return SweptSphereLineSegment(startPos, velocity, radius + capsuleRadius, p1, p2,
                                         out hitTime, out hitPoint, out hitNormal);
        }

        private bool SweptSphereLineSegment(Vector3 startPos, Vector3 velocity, float radius,
                                           Vector3 segmentA, Vector3 segmentB,
                                           out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 segmentDir = segmentB - segmentA;
            float segmentLength = segmentDir.magnitude;
            if (segmentLength < 0.0001f)
                return SweptSpherePoint(startPos, velocity, radius, segmentA, out hitTime, out hitPoint, out hitNormal);
            segmentDir /= segmentLength;

            Vector3 w = startPos - segmentA;
            float a = Vector3.Dot(velocity, velocity);
            float b = Vector3.Dot(velocity, segmentDir);
            float c = Vector3.Dot(segmentDir, segmentDir);
            float d = Vector3.Dot(velocity, w);
            float e = Vector3.Dot(segmentDir, w);
            float denom = a * c - b * b;

            if (Mathf.Abs(denom) < 0.0001f)
            {
                bool hit1 = SweptSpherePoint(startPos, velocity, radius, segmentA, out float t1, out Vector3 p1, out Vector3 n1);
                bool hit2 = SweptSpherePoint(startPos, velocity, radius, segmentB, out float t2, out Vector3 p2, out Vector3 n2);
                if (hit1 && hit2) { if (t1 < t2) { hitTime = t1; hitPoint = p1; hitNormal = n1; } else { hitTime = t2; hitPoint = p2; hitNormal = n2; } return true; }
                else if (hit1) { hitTime = t1; hitPoint = p1; hitNormal = n1; return true; }
                else if (hit2) { hitTime = t2; hitPoint = p2; hitNormal = n2; return true; }
                return false;
            }

            float t = (b * e - c * d) / denom;
            float s = (a * e - b * d) / denom;
            s = Mathf.Clamp(s, 0f, segmentLength);

            Vector3 segmentPoint = segmentA + segmentDir * s;
            return SweptSpherePoint(startPos, velocity, radius, segmentPoint, out hitTime, out hitPoint, out hitNormal);
        }

        private bool SweptSpherePoint(Vector3 startPos, Vector3 velocity, float radius, Vector3 point,
                                     out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 p = startPos - point;
            Vector3 v = velocity;
            float a = Vector3.Dot(v, v);
            float b = 2f * Vector3.Dot(p, v);
            float c = Vector3.Dot(p, p) - radius * radius;

            if (a < 0.0001f) return false;
            float discriminant = b * b - 4f * a * c;
            if (discriminant < 0) return false;

            float sqrtDisc = Mathf.Sqrt(discriminant);
            float t1 = (-b - sqrtDisc) / (2f * a);
            float t2 = (-b + sqrtDisc) / (2f * a);
            float t = t1 >= 0f ? t1 : t2;

            if (t < 0f || t > 1f) return false;

            hitTime = t;
            Vector3 posAtHit = startPos + velocity * t;
            hitNormal = (posAtHit - point).normalized;
            hitPoint = point;
            return true;
        }

        private bool SweptSphereTerrain(Vector3 startPos, Vector3 velocity, float radius, TerrainCollider terrain,
                                       out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Terrain terrainComponent = terrain.GetComponent<Terrain>();
            if (terrainComponent == null || terrainComponent.terrainData == null) return false;

            TerrainData terrainData = terrainComponent.terrainData;
            Vector3 terrainPos = terrain.transform.position;
            float sweepDistance = velocity.magnitude;
            int steps = Mathf.Max(5, Mathf.CeilToInt(sweepDistance / (radius * 0.25f)));
            steps = Mathf.Min(steps, 50);

            for (int i = 0; i <= steps; i++)
            {
                float t = (float)i / steps;
                Vector3 testPos = startPos + velocity * t;
                Vector3 localPos = testPos - terrainPos;
                float normX = localPos.x / terrainData.size.x;
                float normZ = localPos.z / terrainData.size.z;

                if (normX < 0 || normX > 1 || normZ < 0 || normZ > 1) continue;

                float terrainHeight = terrainData.GetInterpolatedHeight(normX, normZ) + terrainPos.y;
                float sphereBottom = testPos.y - radius;

                if (sphereBottom <= terrainHeight)
                {
                    hitTime = t;
                    hitPoint = new Vector3(testPos.x, terrainHeight, testPos.z);
                    hitNormal = terrainData.GetInterpolatedNormal(normX, normZ);
                    hitNormal = terrain.transform.TransformDirection(hitNormal).normalized;
                    return true;
                }
            }
            return false;
        }

        private bool SweptSphereMesh(Vector3 startPos, Vector3 velocity, float radius, MeshCollider meshCol,
                                    out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            if (meshCol.sharedMesh == null || !meshCol.sharedMesh.isReadable) return false;

            Mesh mesh = meshCol.sharedMesh;
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;

            Vector3 localStart = meshCol.transform.InverseTransformPoint(startPos);
            Vector3 localEnd = meshCol.transform.InverseTransformPoint(startPos + velocity);
            Vector3 localVelocity = localEnd - localStart;
            float localRadius = radius / Mathf.Max(meshCol.transform.lossyScale.x, meshCol.transform.lossyScale.y, meshCol.transform.lossyScale.z);

            bool foundHit = false;
            float earliestT = float.MaxValue;
            Vector3 bestHitPoint = Vector3.zero;
            Vector3 bestHitNormal = Vector3.up;

            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 v0 = vertices[triangles[i]];
                Vector3 v1 = vertices[triangles[i + 1]];
                Vector3 v2 = vertices[triangles[i + 2]];

                if (SweptSphereTriangle(localStart, localVelocity, localRadius, v0, v1, v2,
                                       out float t, out Vector3 point, out Vector3 normal))
                {
                    if (t < earliestT && t >= 0f && t <= 1f)
                    {
                        earliestT = t;
                        bestHitPoint = point;
                        bestHitNormal = normal;
                        foundHit = true;
                    }
                }
            }

            if (foundHit)
            {
                hitTime = earliestT;
                hitPoint = meshCol.transform.TransformPoint(bestHitPoint);
                hitNormal = meshCol.transform.TransformDirection(bestHitNormal).normalized;
                return true;
            }
            return false;
        }

        private bool SweptSphereTriangle(Vector3 startPos, Vector3 velocity, float radius,
                                        Vector3 v0, Vector3 v1, Vector3 v2,
                                        out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;
            Vector3 triNormal = Vector3.Cross(edge1, edge2);

            if (triNormal.sqrMagnitude < 0.0001f) return false;
            triNormal.Normalize();

            if (Vector3.Dot(velocity, triNormal) >= 0) return false;

            float denom = Vector3.Dot(triNormal, velocity);
            if (Mathf.Abs(denom) < 0.0001f) return false;

            float d = Vector3.Dot(triNormal, v0 - (startPos - triNormal * radius));
            float t = d / Vector3.Dot(triNormal, velocity);

            if (t < 0f || t > 1f) return false;

            Vector3 sphereCenter = startPos + velocity * t;
            Vector3 planePoint = sphereCenter - triNormal * radius;

            if (IsPointInTriangleBary(planePoint, v0, v1, v2))
            {
                hitTime = t;
                hitPoint = planePoint;
                hitNormal = triNormal;
                return true;
            }

            bool edgeHit = false;
            float minEdgeT = float.MaxValue;
            Vector3 edgeHitPoint = Vector3.zero;
            Vector3 edgeHitNormal = Vector3.up;

            if (SweptSphereLineSegment(startPos, velocity, radius, v0, v1, out float t01, out Vector3 p01, out Vector3 n01))
            {
                if (t01 < minEdgeT) { minEdgeT = t01; edgeHitPoint = p01; edgeHitNormal = n01; edgeHit = true; }
            }
            if (SweptSphereLineSegment(startPos, velocity, radius, v1, v2, out float t12, out Vector3 p12, out Vector3 n12))
            {
                if (t12 < minEdgeT) { minEdgeT = t12; edgeHitPoint = p12; edgeHitNormal = n12; edgeHit = true; }
            }
            if (SweptSphereLineSegment(startPos, velocity, radius, v2, v0, out float t20, out Vector3 p20, out Vector3 n20))
            {
                if (t20 < minEdgeT) { minEdgeT = t20; edgeHitPoint = p20; edgeHitNormal = n20; edgeHit = true; }
            }

            if (edgeHit && minEdgeT >= 0f && minEdgeT <= 1f)
            {
                hitTime = minEdgeT;
                hitPoint = edgeHitPoint;
                hitNormal = edgeHitNormal;
                return true;
            }
            return false;
        }

        private bool IsPointInTriangleBary(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 v0 = b - a;
            Vector3 v1 = c - a;
            Vector3 v2 = p - a;

            float dot00 = Vector3.Dot(v0, v0);
            float dot01 = Vector3.Dot(v0, v1);
            float dot02 = Vector3.Dot(v0, v2);
            float dot11 = Vector3.Dot(v1, v1);
            float dot12 = Vector3.Dot(v1, v2);

            float denom = dot00 * dot11 - dot01 * dot01;
            if (Mathf.Abs(denom) < 0.0001f) return false;

            float invDenom = 1f / denom;
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            return (u >= -0.001f) && (v >= -0.001f) && (u + v <= 1.001f);
        }
        #endregion

        #region Collision Response - REPLACED WITH STRICT LOGIC

        private void ApplyCollisionResponseCCD(int nodeIndex, NodeManager nodeManager, Matter softBodyMatter,
                                              List<float> nodeMasses, Vector3 contactPoint, Vector3 normal,
                                              float penetration, float impactTime, float dt, float epsilon, bool visualizeForces)
        {
            Vector3 predPos = nodeManager.PredictedPositions[nodeIndex];
            Vector3 positionCorrection = normal * (penetration + epsilon);
            nodeManager.PredictedPositions[nodeIndex] = predPos + positionCorrection;

            int substeps = 1;
#if UNITY_EDITOR || UNITY_STANDALONE
            if (SceneSettings.Instance != null) substeps = SceneSettings.Instance.SubstepCount;
#endif

            float normalCorrectionMag = penetration + epsilon;
            float effectiveNormalForce = normalCorrectionMag * substeps;

            if (normalCorrectionMag > 0)
            {
                float muStatic = softBodyMatter != null ? softBodyMatter.StaticFriction : 0.5f;
                float muKinetic = softBodyMatter != null ? softBodyMatter.SlidingFriction : 0.4f;

                Vector3 currentPred = nodeManager.PredictedPositions[nodeIndex];
                Vector3 prevWorldPos = nodeManager.Nodes[nodeIndex].position; 

                Vector3 totalDisplacement = currentPred - prevWorldPos;

                // Project displacement onto the tangent plane
                Vector3 tangentialDisplacement = totalDisplacement - Vector3.Dot(totalDisplacement, normal) * normal;
                float tangentMag = tangentialDisplacement.magnitude;

                if (tangentMag > 1e-8f)
                {
                    float staticLimit = muStatic * effectiveNormalForce;

                    if (tangentMag <= staticLimit)
                    {
                        nodeManager.PredictedPositions[nodeIndex] -= tangentialDisplacement;
                    }
                    else
                    {
                        float kineticReduction = muKinetic * effectiveNormalForce;
                        float correctionAmount = Mathf.Min(kineticReduction, tangentMag);
                        Vector3 frictionCorrection = tangentialDisplacement.normalized * correctionAmount;

                        nodeManager.PredictedPositions[nodeIndex] -= frictionCorrection;
                    }
                }
            }

            if (visualizeForces)
            {
                Debug.DrawLine(contactPoint, contactPoint + normal * 0.1f, Color.red, dt);
                Debug.DrawLine(predPos, nodeManager.PredictedPositions[nodeIndex], Color.yellow, dt);
            }
        }

        #endregion

        #region Node-to-Node Collision (Unchanged)

        private void ResolveNodeToNodeOptimized(NodeManager nmA, NodeManager nmB, List<float> massesA, List<float> massesB,
                                               Matter matterA, Matter matterB, float dt, float epsilon, bool visualizeForces)
        {
            _spatialHash.Clear();

            float nodeRadiusA = nmA.GetNodeRadius();
            float nodeRadiusB = nmB.GetNodeRadius();
            float combinedRadius = nodeRadiusA + nodeRadiusB;
            float threshold = combinedRadius + epsilon;
            float thresholdSq = threshold * threshold;

            for (int i = 0; i < nmB.PredictedPositions.Count; i++)
            {
                Vector3 pos = nmB.PredictedPositions[i];
                int hashKey = ComputeSpatialHash(pos);
                if (!_spatialHash.ContainsKey(hashKey))
                    _spatialHash[hashKey] = new List<int>();
                _spatialHash[hashKey].Add(i);
            }

            for (int i = 0; i < nmA.PredictedPositions.Count; i++)
            {
                Vector3 posA = nmA.PredictedPositions[i];
                GetNearbyCellKeys(posA, threshold, _nearbyKeys);

                foreach (int key in _nearbyKeys)
                {
                    if (!_spatialHash.TryGetValue(key, out List<int> indices)) continue;

                    foreach (int j in indices)
                    {
                        Vector3 posB = nmB.PredictedPositions[j];
                        Vector3 delta = posA - posB;
                        float distSq = delta.sqrMagnitude;

                        if (distSq < thresholdSq && distSq > 1e-9f)
                        {
                            float dist = Mathf.Sqrt(distSq);
                            float penetrationDepth = threshold - dist;
                            Vector3 normal = delta / dist;

                            float massA = massesA[i];
                            float massB = massesB[j];
                            float totalMass = massA + massB;
                            float ratioA = massB / totalMass;
                            float ratioB = massA / totalMass;

                            Vector3 correctionA = normal * (penetrationDepth * ratioA + epsilon * 0.5f);
                            Vector3 correctionB = -normal * (penetrationDepth * ratioB + epsilon * 0.5f);

                            nmA.PredictedPositions[i] += correctionA;
                            nmB.PredictedPositions[j] += correctionB;

                            ApplyFrictionAndRestitution(nmA, nmB, i, j, massesA, massesB, matterA, matterB, normal, dt);

                            Vector3 contactPt = posA - normal * nodeRadiusA;
                            collisionPoints.Add(contactPt);

                            if (visualizeForces)
                            {
                                Debug.DrawLine(contactPt, contactPt + normal * 0.1f, Color.cyan, dt);
                            }
                        }
                    }
                }
            }
        }

        private int ComputeSpatialHash(Vector3 pos)
        {
            int x = Mathf.FloorToInt(pos.x / SPATIAL_CELL_SIZE);
            int y = Mathf.FloorToInt(pos.y / SPATIAL_CELL_SIZE);
            int z = Mathf.FloorToInt(pos.z / SPATIAL_CELL_SIZE);

            return (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
        }

        private void GetNearbyCellKeys(Vector3 pos, float radius, List<int> outKeys)
        {
            outKeys.Clear();
            int range = Mathf.CeilToInt(radius / SPATIAL_CELL_SIZE);

            int x0 = Mathf.FloorToInt(pos.x / SPATIAL_CELL_SIZE);
            int y0 = Mathf.FloorToInt(pos.y / SPATIAL_CELL_SIZE);
            int z0 = Mathf.FloorToInt(pos.z / SPATIAL_CELL_SIZE);

            for (int dx = -range; dx <= range; dx++)
            {
                for (int dy = -range; dy <= range; dy++)
                {
                    for (int dz = -range; dz <= range; dz++)
                    {
                        int x = x0 + dx;
                        int y = y0 + dy;
                        int z = z0 + dz;
                        int hash = (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
                        outKeys.Add(hash);
                    }
                }
            }
        }

        private void InsertIntoFaceNodeHash(int nodeIdx, Vector3 pos)
        {
            int x = Mathf.FloorToInt(pos.x / FACE_NODE_CELL_SIZE);
            int y = Mathf.FloorToInt(pos.y / FACE_NODE_CELL_SIZE);
            int z = Mathf.FloorToInt(pos.z / FACE_NODE_CELL_SIZE);
            int key = (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
            if (!_faceNodeHash.TryGetValue(key, out List<int> bucket))
                _faceNodeHash[key] = bucket = new List<int>(4);
            bucket.Add(nodeIdx);
        }

        private void GetFaceNodeCellKeysAABB(Vector3 bMin, Vector3 bMax, List<int> outKeys)
        {
            outKeys.Clear();
            int x0 = Mathf.FloorToInt(bMin.x / FACE_NODE_CELL_SIZE);
            int y0 = Mathf.FloorToInt(bMin.y / FACE_NODE_CELL_SIZE);
            int z0 = Mathf.FloorToInt(bMin.z / FACE_NODE_CELL_SIZE);
            int x1 = Mathf.Min(Mathf.FloorToInt(bMax.x / FACE_NODE_CELL_SIZE), x0 + 4);
            int y1 = Mathf.Min(Mathf.FloorToInt(bMax.y / FACE_NODE_CELL_SIZE), y0 + 4);
            int z1 = Mathf.Min(Mathf.FloorToInt(bMax.z / FACE_NODE_CELL_SIZE), z0 + 4);
            for (int x = x0; x <= x1; x++)
                for (int y = y0; y <= y1; y++)
                    for (int z = z0; z <= z1; z++)
                        outKeys.Add((x * 73856093) ^ (y * 19349663) ^ (z * 83492791));
        }

        #endregion

        #region Face-to-Node Collision (CCD)

        private void ResolveFaceToNode(NodeManager nmNode, List<float> massesNode, SoftBody bodyFace,
                                       NodeManager nmFace, List<float> massesFace, Matter matterNode,
                                       Matter matterFace, float dt, float epsilon, bool visualizeForces)
        {
            Truss truss = bodyFace.truss;
            if (truss == null) return;

            List<Face> faces = truss.GetTrussFaces();
            if (faces == null || faces.Count == 0) return;

            int nodeCount = nmNode.PredictedPositions.Count;
            if (nodeCount == 0) return;

            float radiusNode = nmNode.GetNodeRadius() + SKIN_WIDTH;
            float thickness = radiusNode + triangleThickness;

            _faceNodeHash.Clear();
            if (_nodeVisitedFlags.Length < nodeCount)
                _nodeVisitedFlags = new bool[nodeCount];

            for (int n = 0; n < nodeCount; n++)
            {
                InsertIntoFaceNodeHash(n, nmNode.PreviousPositions[n]);
                InsertIntoFaceNodeHash(n, nmNode.PredictedPositions[n]);
            }

            foreach (Face face in faces)
            {
                if (face.nodeA < 0 || face.nodeA >= nmFace.PredictedPositions.Count ||
                    face.nodeB < 0 || face.nodeB >= nmFace.PredictedPositions.Count ||
                    face.nodeC < 0 || face.nodeC >= nmFace.PredictedPositions.Count) continue;

                Vector3 v0s = nmFace.PreviousPositions[face.nodeA];
                Vector3 v1s = nmFace.PreviousPositions[face.nodeB];
                Vector3 v2s = nmFace.PreviousPositions[face.nodeC];
                Vector3 v0e = nmFace.PredictedPositions[face.nodeA];
                Vector3 v1e = nmFace.PredictedPositions[face.nodeB];
                Vector3 v2e = nmFace.PredictedPositions[face.nodeC];

                float bMinX = Mathf.Min(Mathf.Min(v0s.x, v1s.x), Mathf.Min(v2s.x, Mathf.Min(Mathf.Min(v0e.x, v1e.x), v2e.x))) - thickness;
                float bMinY = Mathf.Min(Mathf.Min(v0s.y, v1s.y), Mathf.Min(v2s.y, Mathf.Min(Mathf.Min(v0e.y, v1e.y), v2e.y))) - thickness;
                float bMinZ = Mathf.Min(Mathf.Min(v0s.z, v1s.z), Mathf.Min(v2s.z, Mathf.Min(Mathf.Min(v0e.z, v1e.z), v2e.z))) - thickness;
                float bMaxX = Mathf.Max(Mathf.Max(v0s.x, v1s.x), Mathf.Max(v2s.x, Mathf.Max(Mathf.Max(v0e.x, v1e.x), v2e.x))) + thickness;
                float bMaxY = Mathf.Max(Mathf.Max(v0s.y, v1s.y), Mathf.Max(v2s.y, Mathf.Max(Mathf.Max(v0e.y, v1e.y), v2e.y))) + thickness;
                float bMaxZ = Mathf.Max(Mathf.Max(v0s.z, v1s.z), Mathf.Max(v2s.z, Mathf.Max(Mathf.Max(v0e.z, v1e.z), v2e.z))) + thickness;

                GetFaceNodeCellKeysAABB(
                    new Vector3(bMinX, bMinY, bMinZ),
                    new Vector3(bMaxX, bMaxY, bMaxZ),
                    _faceNodeNearbyKeys);

                _faceNodeCandidates.Clear();
                foreach (int key in _faceNodeNearbyKeys)
                {
                    if (!_faceNodeHash.TryGetValue(key, out List<int> bucket)) continue;
                    foreach (int n in bucket)
                    {
                        if (_nodeVisitedFlags[n]) continue;
                        _nodeVisitedFlags[n] = true;
                        _faceNodeCandidates.Add(n);
                    }
                }

                foreach (int n in _faceNodeCandidates)
                {
                    _nodeVisitedFlags[n] = false;

                    Vector3 nodeStart = nmNode.PreviousPositions[n];
                    Vector3 nodeEnd = nmNode.PredictedPositions[n];

                    float nsX = nodeStart.x < nodeEnd.x ? nodeStart.x : nodeEnd.x;
                    float nsY = nodeStart.y < nodeEnd.y ? nodeStart.y : nodeEnd.y;
                    float nsZ = nodeStart.z < nodeEnd.z ? nodeStart.z : nodeEnd.z;
                    float neX = nodeStart.x > nodeEnd.x ? nodeStart.x : nodeEnd.x;
                    float neY = nodeStart.y > nodeEnd.y ? nodeStart.y : nodeEnd.y;
                    float neZ = nodeStart.z > nodeEnd.z ? nodeStart.z : nodeEnd.z;
                    if (nsX > bMaxX || neX < bMinX ||
                        nsY > bMaxY || neY < bMinY ||
                        nsZ > bMaxZ || neZ < bMinZ) continue;

                    if (CheckSweptFaceNode(nodeStart, nodeEnd, radiusNode, v0s, v0e, v1s, v1e, v2s, v2e, out float t, out Vector3 hitNormal))
                    {
                        Vector3 pAtT = Vector3.Lerp(nodeStart, nodeEnd, t);
                        Vector3 aAtT = Vector3.Lerp(v0s, v0e, t);
                        Vector3 bAtT = Vector3.Lerp(v1s, v1e, t);
                        Vector3 cAtT = Vector3.Lerp(v2s, v2e, t);

                        if (GetBarycentric(pAtT, aAtT, bAtT, cAtT, out float u, out float v, out float w))
                        {
                            Vector3 triPoint = u * v0e + v * v1e + w * v2e;
                            float dist = Vector3.Dot(nodeEnd - triPoint, hitNormal);
                            float penetration = (radiusNode + epsilon) - dist;

                            if (penetration > 0 || t < 1.0f)
                            {
                                penetration = Mathf.Max(penetration, epsilon);

                                ApplyXPBDFaceNode(nmNode, n, nmFace, face.nodeA, face.nodeB, face.nodeC,
                                    hitNormal, penetration, u, v, w, dt, matterNode, matterFace, massesNode, massesFace, visualizeForces);

                                collisionPoints.Add(pAtT);
                                if (visualizeForces)
                                    Debug.DrawLine(pAtT, pAtT + hitNormal * 0.15f, Color.magenta, dt);
                            }
                        }
                    }
                }
            }
        }

        private bool CheckSweptFaceNode(Vector3 p0, Vector3 p1, float radius, Vector3 a0, Vector3 a1, Vector3 b0, Vector3 b1, Vector3 c0, Vector3 c1, out float t, out Vector3 normal)
        {
            t = 1.0f;
            normal = Vector3.Cross(b0 - a0, c0 - a0).normalized;

            Vector3 tri0 = (a0 + b0 + c0) / 3f;
            Vector3 relMove = (p1 - p0) - ((a1 + b1 + c1) / 3f - tri0);

            float d0 = Vector3.Dot(p0 - tri0, normal);
            float d1 = Vector3.Dot(p0 + relMove - tri0, normal);

            if (Mathf.Sign(d0) != Mathf.Sign(d1))
            {
                t = Mathf.Clamp01(Mathf.Abs(d0) / (Mathf.Abs(d0) + Mathf.Abs(d1)));
                if (d0 < 0) normal = -normal;
                return true;
            }

            if (Mathf.Abs(d0) < radius) { t = 0; return true; }
            return false;
        }

        private void ApplyXPBDFaceNode(NodeManager nmP, int idxP,
                                       NodeManager nmT, int idxA, int idxB, int idxC,
                                       Vector3 normal, float penetration,
                                       float u, float v, float w,
                                       float dt, Matter matP, Matter matT,
                                       List<float> massesP, List<float> massesT, bool visualize)
        {
            float wP = nmP.IsPinned[idxP] ? 0f : 1f / massesP[idxP];
            float wA = nmT.IsPinned[idxA] ? 0f : 1f / massesT[idxA];
            float wB = nmT.IsPinned[idxB] ? 0f : 1f / massesT[idxB];
            float wC = nmT.IsPinned[idxC] ? 0f : 1f / massesT[idxC];

            float wTri = (u * u * wA) + (v * v * wB) + (w * w * wC);
            float wSum = wP + wTri;
            if (wSum <= 1e-9f) return;

            float dLambda = penetration / wSum;
            Vector3 correction = dLambda * normal;

            if (!nmP.IsPinned[idxP]) nmP.PredictedPositions[idxP] += wP * correction;
            if (!nmT.IsPinned[idxA]) nmT.PredictedPositions[idxA] -= wA * u * correction;
            if (!nmT.IsPinned[idxB]) nmT.PredictedPositions[idxB] -= wB * v * correction;
            if (!nmT.IsPinned[idxC]) nmT.PredictedPositions[idxC] -= wC * w * correction;
        }

        private bool GetBarycentric(Vector3 p, Vector3 a, Vector3 b, Vector3 c, out float u, out float v, out float w)
        {
            Vector3 v0 = b - a, v1 = c - a, v2 = p - a;
            float d00 = Vector3.Dot(v0, v0), d01 = Vector3.Dot(v0, v1), d11 = Vector3.Dot(v1, v1);
            float d20 = Vector3.Dot(v2, v0), d21 = Vector3.Dot(v2, v1);
            float denom = d00 * d11 - d01 * d01;
            if (Mathf.Abs(denom) < 1e-9f) { u = v = w = 0; return false; }
            v = (d11 * d20 - d01 * d21) / denom;
            w = (d00 * d21 - d01 * d20) / denom;
            u = 1.0f - v - w;
            return (u >= -0.05f && v >= -0.05f && w >= -0.05f && (u + v + w) <= 1.05f);
        }

        private static float CombineMatterProperty(Matter a, Matter b,
            System.Func<Matter, float> getter, float defaultVal)
        {
            float va = a != null ? getter(a) : defaultVal;
            float vb = b != null ? getter(b) : defaultVal;
            return (va + vb) * 0.5f;
        }

        #endregion

        #region Friction and Restitution (Cross Body)

        private void ApplyFrictionAndRestitution(NodeManager nmA, NodeManager nmB, int idxA, int idxB,
                                                List<float> massesA, List<float> massesB,
                                                Matter matterA, Matter matterB, Vector3 normal, float dt)
        {
            Vector3 velA = (nmA.PredictedPositions[idxA] - nmA.Nodes[idxA].position) / dt;
            Vector3 velB = (nmB.PredictedPositions[idxB] - nmB.Nodes[idxB].position) / dt;
            Vector3 relVel = velA - velB;

            float normalVel = Vector3.Dot(relVel, normal);
            if (normalVel >= 0) return;

            float restitutionA = matterA != null ? matterA.Restitution : PhysicsConstants.DEFAULT_RESTITUTION;
            float restitutionB = matterB != null ? matterB.Restitution : PhysicsConstants.DEFAULT_RESTITUTION;
            float restitution = (restitutionA + restitutionB) * 0.5f;

            float frictionA = matterA != null ? matterA.SlidingFriction : PhysicsConstants.DEFAULT_DYNAMIC_FRICTION;
            float frictionB = matterB != null ? matterB.SlidingFriction : PhysicsConstants.DEFAULT_DYNAMIC_FRICTION;
            float friction = (frictionA + frictionB) * 0.5f;

            float massA = massesA[idxA];
            float massB = massesB[idxB];
            float invMassA = 1f / massA;
            float invMassB = 1f / massB;
            float invMassSum = invMassA + invMassB;

            float massRatio = massA / (massA + massB);
            float impactMultiplier = 1f + (Mathf.Abs(massA - massB) / Mathf.Max(massA, massB)) * 0.5f;

            float jn = -(1f + restitution) * normalVel / invMassSum * impactMultiplier;
            Vector3 normalImpulse = normal * jn;

            Vector3 tangentVel = relVel - normal * normalVel;
            Vector3 frictionImpulse = Vector3.zero;

            if (tangentVel.magnitude > PhysicsConstants.MIN_TANGENT_SPEED)
            {
                float maxFrictionMagnitude = friction * Mathf.Abs(jn);
                float stopImpulseMagnitude = tangentVel.magnitude / invMassSum;
                float frictionMagnitude = Mathf.Min(maxFrictionMagnitude, stopImpulseMagnitude);
                frictionImpulse = -tangentVel.normalized * frictionMagnitude;
            }

            Vector3 totalImpulse = normalImpulse + frictionImpulse;

            nmA.Nodes[idxA].position -= (totalImpulse * invMassA) * dt;
            nmB.Nodes[idxB].position -= (-totalImpulse * invMassB) * dt;
        }

        #endregion

        #region Utility

        private bool ValidateCollisionParameters(NodeManager nodeManager, Transform owner, List<float> nodeMasses)
        {
            if (nodeManager == null || nodeManager.Nodes == null || nodeManager.Nodes.Count == 0)
            {
                if (enableDebugLogging) Debug.LogWarning($"[CollisionHandler] Invalid nodeManager for {owner?.name}");
                return false;
            }

            if (nodeMasses == null || nodeMasses.Count != nodeManager.Nodes.Count)
            {
                if (enableDebugLogging) Debug.LogWarning($"[CollisionHandler] Mass count mismatch for {owner?.name}");
                return false;
            }

            return true;
        }

        public void IgnoreBodyCollision(Transform bodyA, Transform bodyB)
        {
            int idA = bodyA.GetInstanceID();
            int idB = bodyB.GetInstanceID();
            var pair = idA < idB ? (idA, idB) : (idB, idA);
            _ignoredBodyPairs.Add(pair);
        }

        public void SetIgnoreCollision(Transform bodyA, Transform bodyB, bool ignore)
        {
            if (ignore)
                IgnoreBodyCollision(bodyA, bodyB);
            else
                RemoveIgnoredBodyCollision(bodyA, bodyB);
        }

        public void RemoveIgnoredBodyCollision(Transform bodyA, Transform bodyB)
        {
            int idA = bodyA.GetInstanceID();
            int idB = bodyB.GetInstanceID();
            var pair = idA < idB ? (idA, idB) : (idB, idA);
            _ignoredBodyPairs.Remove(pair);
        }

        public void ClearIgnoredPairs()
        {
            _ignoredBodyPairs.Clear();
        }

        #endregion
    }
}