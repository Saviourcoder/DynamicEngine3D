using UnityEngine;
using Vella.UnityNativeHull;
using Vella.Common;
using Unity.Mathematics;
using DynamicEngine;
using System;
using System.Collections.Generic;
using System.Linq;

public class TrussCollider : MonoBehaviour
{
    [Header("Truss Reference")]
    [SerializeField] private Truss truss;

    [Header("Geometry Correction")]
    [Tooltip("Fix inconsistent face winding automatically")]
    [SerializeField] private bool autoFixWinding = true;

    [Header("Update Settings")]
    [SerializeField] private bool autoUpdate = true;
    [SerializeField] private int updateInterval = 5;

    [Header("Collision Detection")]
    [Tooltip("Automatically detect collisions with other TrussColliders")]
    [SerializeField] private bool detectCollisions = true;
    [Tooltip("Check collisions every N frames (0 = every frame)")]
    [SerializeField] private int collisionCheckInterval = 1;
    [Tooltip("Show collision contact points")]
    [SerializeField] private bool showContactPoints = true;
    [Tooltip("Show collision normal")]
    [SerializeField] private bool showCollisionNormal = true;

    [Header("Visualization")]
    [SerializeField] private bool visualizeFaces = true;
    [SerializeField] private bool visualizeNormals = false;
    [SerializeField] private Color normalFaceColor = new Color(0, 1, 0, 0.5f);
    [SerializeField] private Color collidingFaceColor = new Color(1, 0, 0, 0.7f);
    [SerializeField] private Color contactPointColor = Color.yellow;
    [SerializeField] private Color collisionNormalColor = Color.cyan;
    [SerializeField] private Color normalColor = Color.blue;
    [SerializeField] private float normalLength = 0.5f;
    [SerializeField] private float contactPointSize = 0.1f;
    [SerializeField] private float collisionNormalLength = 1.0f;

    [Header("Debug")]
    [SerializeField] private bool showDebugInfo = false;
    [SerializeField] private bool logErrors = true;

    private NativeHull _hull;
    private int _lastTrussHash;
    private bool _hullValid;
    private int _frameCounter;
    private int _collisionCheckCounter;

    // Collision state
    private bool _isColliding;
    private List<TrussCollider> _collidingWith = new List<TrussCollider>();
    private List<ContactInfo> _contactInfos = new List<ContactInfo>();

    public NativeHull Hull => _hull;
    public bool IsValid => _hullValid && _hull.IsCreated;
    public bool IsColliding => _isColliding;
    public IReadOnlyList<TrussCollider> CollidingWith => _collidingWith;

    private struct ContactInfo
    {
        public TrussCollider other;
        public Vector3[] contactPoints;
        public Vector3 normal;
    }

    private void OnEnable()
    {
        ForceUpdateHull();
    }

    private void Update()
    {
        if (autoUpdate)
        {
            _frameCounter++;
            if (updateInterval == 0 || _frameCounter % updateInterval == 0)
            {
                if (HasTrussChanged())
                {
                    UpdateHull();
                }
            }
        }

        if (detectCollisions && _hullValid)
        {
            _collisionCheckCounter++;
            if (collisionCheckInterval == 0 || _collisionCheckCounter % collisionCheckInterval == 0)
            {
                CheckCollisions();
            }
        }
    }

    public void ForceUpdateHull()
    {
        UpdateHull();
    }

    private void CheckCollisions()
    {
        _collidingWith.Clear();
        _contactInfos.Clear();
        _isColliding = false;

        // Find all other TrussColliders in the scene
        var allColliders = FindObjectsOfType<TrussCollider>();

        foreach (var other in allColliders)
        {
            if (other == this || !other.IsValid)
                continue;

            if (IsCollidingDetailed(other, out ContactInfo contactInfo))
            {
                _isColliding = true;
                _collidingWith.Add(other);
                _contactInfos.Add(contactInfo);

                if (showDebugInfo)
                {
                    Debug.Log($"[TrussCollider] {gameObject.name} is colliding with {other.gameObject.name}");
                }
            }
        }
    }

    private bool IsCollidingDetailed(TrussCollider other, out ContactInfo contactInfo)
    {
        contactInfo = new ContactInfo
        {
            other = other,
            contactPoints = new Vector3[0],
            normal = Vector3.zero
        };

        var t1 = new RigidTransform(transform.rotation, transform.position);
        var t2 = new RigidTransform(other.transform.rotation, other.transform.position);

        // Try to get detailed contact information
        var manifold = new NativeManifold(Unity.Collections.Allocator.Temp);

        try
        {
            bool hasContact = HullIntersection.NativeHullHullContact(ref manifold, t1, _hull, t2, other._hull);

            if (hasContact && manifold.Length > 0)
            {
                var points = new List<Vector3>();

                for (int i = 0; i < manifold.Length; i++)
                {
                    var contact = manifold[i];
                    points.Add(contact.Position);
                }

                contactInfo.contactPoints = points.ToArray();
                contactInfo.normal = manifold.Normal;

                return true;
            }
        }
        finally
        {
            manifold.Dispose();
        }

        // Fallback to simple collision check
        return HullCollision.IsColliding(t1, _hull, t2, other._hull);
    }

    private bool HasTrussChanged()
    {
        if (truss == null) return false;

        int currentHash = CalculateTrussHash();
        return currentHash != _lastTrussHash;
    }

    private int CalculateTrussHash()
    {
        if (truss == null) return 0;

        unchecked
        {
            int hash = 17;
            var faces = truss.GetTrussFaces();
            var positions = truss.NodePositions;

            if (faces != null)
            {
                hash = hash * 31 + faces.Count;
                foreach (var face in faces)
                {
                    hash = hash * 31 + face.nodeA + face.nodeB + face.nodeC;
                }
            }

            if (positions != null)
            {
                hash = hash * 31 + positions.Length;
            }

            return hash;
        }
    }

    private void UpdateHull()
    {
        if (_hullValid && _hull.IsCreated)
        {
            _hull.Dispose();
        }

        _hullValid = false;

        if (truss == null)
        {
            LogError("Truss reference is null");
            return;
        }

        var faces = truss.GetTrussFaces();
        var positions = truss.NodePositions;

        if (faces == null || faces.Count == 0)
        {
            LogError("Truss has no faces");
            return;
        }

        if (positions == null || positions.Length == 0)
        {
            LogError("Truss has no node positions");
            return;
        }

        try
        {
            if (autoFixWinding)
            {
                var correctedMesh = CreateCorrectedMesh(faces, positions);
                _hull = HullFactory.CreateFromMesh(correctedMesh);
                Destroy(correctedMesh);
            }
            else
            {
                _hull = HullFactory.CreateFromTruss(truss);
            }

            if (_hull.IsCreated)
            {
                _hullValid = true;
                _lastTrussHash = CalculateTrussHash();

                if (showDebugInfo)
                {
                    Debug.Log($"[TrussCollider] Hull created successfully:\n" +
                              $"  Vertices: {_hull.VertexCount}\n" +
                              $"  Faces: {_hull.FaceCount}\n" +
                              $"  Edges: {_hull.EdgeCount}");
                }
            }
            else
            {
                LogError("Hull was not created properly");
            }
        }
        catch (Exception e)
        {
            LogError($"Exception creating hull: {e.Message}\n{e.StackTrace}");
        }
    }

    private Mesh CreateCorrectedMesh(List<Face> faces, Vector3[] positions)
    {
        var mesh = new Mesh();
        var vertices = new List<Vector3>();
        var triangles = new List<int>();

        Vector3 centroid = Vector3.zero;
        foreach (var pos in positions)
        {
            centroid += pos;
        }
        centroid /= positions.Length;

        int fixedCount = 0;

        foreach (var face in faces)
        {
            if (face.nodeA >= positions.Length ||
                face.nodeB >= positions.Length ||
                face.nodeC >= positions.Length)
            {
                continue;
            }

            Vector3 v0 = positions[face.nodeA];
            Vector3 v1 = positions[face.nodeB];
            Vector3 v2 = positions[face.nodeC];

            if (v0 == v1 || v1 == v2 || v2 == v0)
            {
                continue;
            }

            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;
            Vector3 faceNormal = Vector3.Cross(edge1, edge2);

            if (faceNormal.sqrMagnitude < 0.0001f)
            {
                continue;
            }

            faceNormal.Normalize();

            Vector3 faceCenter = (v0 + v1 + v2) / 3f;
            Vector3 outwardDirection = (faceCenter - centroid).normalized;

            bool isOutward = Vector3.Dot(faceNormal, outwardDirection) > 0;

            int idx0 = vertices.Count;
            int idx1 = vertices.Count + 1;
            int idx2 = vertices.Count + 2;

            vertices.Add(v0);
            vertices.Add(v1);
            vertices.Add(v2);

            if (isOutward)
            {
                triangles.Add(idx0);
                triangles.Add(idx1);
                triangles.Add(idx2);
            }
            else
            {
                triangles.Add(idx0);
                triangles.Add(idx2);
                triangles.Add(idx1);
                fixedCount++;
            }
        }

        if (showDebugInfo && fixedCount > 0)
        {
            Debug.Log($"[TrussCollider] Fixed {fixedCount} faces with incorrect winding");
        }

        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        return mesh;
    }

    public bool IsCollidingWith(TrussCollider other)
    {
        if (!_hullValid || !other._hullValid) return false;

        var t1 = new RigidTransform(transform.rotation, transform.position);
        var t2 = new RigidTransform(other.transform.rotation, other.transform.position);

        return HullCollision.IsColliding(t1, _hull, t2, other._hull);
    }

    public bool ContainsPoint(Vector3 worldPoint)
    {
        if (!_hullValid) return false;

        var t = new RigidTransform(transform.rotation, transform.position);
        return HullCollision.Contains(t, _hull, worldPoint);
    }

    public Vector3 ClosestPoint(Vector3 worldPoint)
    {
        if (!_hullValid) return worldPoint;

        var t = new RigidTransform(transform.rotation, transform.position);
        return HullCollision.ClosestPoint(t, _hull, worldPoint);
    }

    private void OnDrawGizmos()
    {
        if (!_hullValid || !_hull.IsCreated) return;

        if (visualizeFaces) DrawFaces();
        if (visualizeNormals) DrawNormals();

        if (detectCollisions && _isColliding)
        {
            if (showContactPoints) DrawContactPoints();
            if (showCollisionNormal) DrawCollisionNormals();
        }
    }

    private unsafe void DrawFaces()
    {
        // Use different color based on collision state
        Gizmos.color = _isColliding ? collidingFaceColor : normalFaceColor;

        for (int i = 0; i < _hull.FaceCount; i++)
        {
            var vertices = _hull.GetVertices(i);
            if (vertices.Count < 3) continue;

            // Draw filled triangles
            for (int j = 1; j < vertices.Count - 1; j++)
            {
                var v0 = transform.TransformPoint(vertices[0]);
                var v1 = transform.TransformPoint(vertices[j]);
                var v2 = transform.TransformPoint(vertices[j + 1]);

                // Draw edges
                Gizmos.DrawLine(v0, v1);
                Gizmos.DrawLine(v1, v2);
                Gizmos.DrawLine(v2, v0);
            }
        }
    }

    private void DrawContactPoints()
    {
        Gizmos.color = contactPointColor;

        foreach (var contactInfo in _contactInfos)
        {
            foreach (var point in contactInfo.contactPoints)
            {
                Gizmos.DrawWireSphere(point, contactPointSize);
                Gizmos.DrawSphere(point, contactPointSize * 0.5f);
            }
        }
    }

    private void DrawCollisionNormals()
    {
        Gizmos.color = collisionNormalColor;

        foreach (var contactInfo in _contactInfos)
        {
            if (contactInfo.contactPoints.Length > 0 && contactInfo.normal != Vector3.zero)
            {
                // Draw normal from the center of contact points
                Vector3 center = Vector3.zero;
                foreach (var point in contactInfo.contactPoints)
                {
                    center += point;
                }
                center /= contactInfo.contactPoints.Length;

                Gizmos.DrawRay(center, contactInfo.normal * collisionNormalLength);

                // Draw arrowhead
                Vector3 right = Vector3.Cross(contactInfo.normal, Vector3.up);
                if (right.sqrMagnitude < 0.001f)
                    right = Vector3.Cross(contactInfo.normal, Vector3.forward);
                right.Normalize();

                Vector3 up = Vector3.Cross(right, contactInfo.normal).normalized;
                Vector3 arrowTip = center + contactInfo.normal * collisionNormalLength;
                float arrowSize = collisionNormalLength * 0.2f;

                Gizmos.DrawLine(arrowTip, arrowTip - contactInfo.normal * arrowSize + right * arrowSize * 0.5f);
                Gizmos.DrawLine(arrowTip, arrowTip - contactInfo.normal * arrowSize - right * arrowSize * 0.5f);
            }
        }
    }

    private unsafe void DrawNormals()
    {
        Gizmos.color = normalColor;

        for (int i = 0; i < _hull.FaceCount; i++)
        {
            var face = _hull.GetFace(i);
            var plane = _hull.GetPlane(i);

            var centroid = _hull.CalculateFaceCentroid(face);
            var worldCentroid = transform.TransformPoint(centroid);
            var worldNormal = transform.TransformDirection(plane.Normal);

            Gizmos.DrawRay(worldCentroid, worldNormal * normalLength);
        }
    }

    private void LogError(string message)
    {
        if (logErrors)
        {
            Debug.LogError($"[TrussCollider] {message}", this);
        }
    }

    private void OnDisable()
    {
        if (_hullValid && _hull.IsCreated)
        {
            _hull.Dispose();
            _hullValid = false;
        }
    }

    private void OnDestroy()
    {
        if (_hullValid && _hull.IsCreated)
        {
            _hull.Dispose();
            _hullValid = false;
        }
    }

    private void OnValidate()
    {
        if (Application.isPlaying && truss != null)
        {
            ForceUpdateHull();
        }
    }
}