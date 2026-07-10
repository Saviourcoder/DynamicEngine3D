/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#pragma once

#include "EngineTypes.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>
#include <xmmintrin.h>
#include <smmintrin.h>

   // Vector3 helpers
inline Vector3 V3(float x, float y, float z) { return { x, y, z, 0.0f }; }
inline Vector3 V3Zero() { return { 0.0f, 0.0f, 0.0f, 0.0f }; }

inline Vector3 Add(const Vector3& a, const Vector3& b) {
    Vector3 r;
    _mm_storeu_ps(&r.x, _mm_add_ps(_mm_loadu_ps(&a.x), _mm_loadu_ps(&b.x)));
    return r;
}
inline Vector3 Sub(const Vector3& a, const Vector3& b) {
    Vector3 r;
    _mm_storeu_ps(&r.x, _mm_sub_ps(_mm_loadu_ps(&a.x), _mm_loadu_ps(&b.x)));
    return r;
}
inline Vector3 Scale(const Vector3& a, float s) {
    Vector3 r;
    _mm_storeu_ps(&r.x, _mm_mul_ps(_mm_loadu_ps(&a.x), _mm_set1_ps(s)));
    return r;
}
inline Vector3 Neg(const Vector3& a) {
    Vector3 r;
    _mm_storeu_ps(&r.x, _mm_sub_ps(_mm_setzero_ps(), _mm_loadu_ps(&a.x)));
    return r;
}

inline float Dot(const Vector3& a, const Vector3& b)
{
    return _mm_cvtss_f32(_mm_dp_ps(_mm_loadu_ps(&a.x), _mm_loadu_ps(&b.x), 0x7F)); // sum x,y,z; broadcast
}
inline Vector3 Cross(const Vector3& a, const Vector3& b)
{
    __m128 va = _mm_loadu_ps(&a.x);
    __m128 vb = _mm_loadu_ps(&b.x);
    __m128 res = _mm_sub_ps(
        _mm_mul_ps(_mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 1, 0, 2))),
        _mm_mul_ps(_mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 0, 2, 1)))
    );
    Vector3 r;
    _mm_storeu_ps(&r.x, res);
    r.w = 0.0f;
    return r;
}
inline float LengthSq(const Vector3& a) { return Dot(a, a); }
inline float Length(const Vector3& a) { return _mm_cvtss_f32(_mm_sqrt_ss(_mm_dp_ps(_mm_loadu_ps(&a.x), _mm_loadu_ps(&a.x), 0x7F))); }

inline Vector3 Normalize(const Vector3& a)
{
    float len = Length(a);
    if (len < 1e-7f) return { 0.0f, 0.0f, 0.0f, 0.0f };
    float inv = 1.0f / len;
    return { a.x * inv, a.y * inv, a.z * inv, 0.0f };
}

inline Vector3 Lerp(const Vector3& a, const Vector3& b, float t)
{
    return { a.x + (b.x - a.x) * t,
             a.y + (b.y - a.y) * t,
             a.z + (b.z - a.z) * t,
             0.0f };
}

inline float SignF(float v) { return (v > 0.0f) ? 1.0f : ((v < 0.0f) ? -1.0f : 0.0f); }
inline float ClampF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float MaxF(float a, float b) { return a > b ? a : b; }
inline float MinF(float a, float b) { return a < b ? a : b; }
inline float AbsF(float v) { return v < 0.0f ? -v : v; }

// Quaternion helpers
inline Quaternion QIdentity() { return { 0.0f, 0.0f, 0.0f, 1.0f }; }

inline Quaternion QMul(const Quaternion& a, const Quaternion& b)
{
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
}

inline Quaternion QConj(const Quaternion& q) { return { -q.x, -q.y, -q.z, q.w }; }
inline Quaternion QInverse(const Quaternion& q) { return QConj(q); } // assumes unit quaternion

inline Quaternion QNormalize(const Quaternion& q)
{
    float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-7f) return QIdentity();
    float inv = 1.0f / n;
    return { q.x * inv, q.y * inv, q.z * inv, q.w * inv };
}

inline Vector3 QRotate(const Quaternion& q, const Vector3& v)
{
    // Standard quaternion-vector rotation
    float tx = 2.0f * (q.y * v.z - q.z * v.y);
    float ty = 2.0f * (q.z * v.x - q.x * v.z);
    float tz = 2.0f * (q.x * v.y - q.y * v.x);
    return {
        v.x + q.w * tx + (q.y * tz - q.z * ty),
        v.y + q.w * ty + (q.z * tx - q.x * tz),
        v.z + q.w * tz + (q.x * ty - q.y * tx),
        0.0f
    };
}

// Quaternion from axis-angle (radians)
inline Quaternion QAxisAngle(const Vector3& axis, float angleRad)
{
    float h = angleRad * 0.5f;
    float s = std::sin(h);
    return { axis.x * s, axis.y * s, axis.z * s, std::cos(h) };
}

// Matrix4x4 helpers
// Stored column-major. m[col*4 + row].
inline Vector3 MulMatPoint(const Matrix4x4& m, const Vector3& p)
{
    return {
        m.m[0] * p.x + m.m[4] * p.y + m.m[8] * p.z + m.m[12],
        m.m[1] * p.x + m.m[5] * p.y + m.m[9] * p.z + m.m[13],
        m.m[2] * p.x + m.m[6] * p.y + m.m[10] * p.z + m.m[14],
        0.0f
    };
}

// Spatial hash
inline uint32_t SpatialHash(int x, int y, int z) {
    return ((uint32_t)x * 73856093u) ^ ((uint32_t)y * 19349663u) ^ ((uint32_t)z * 83492791u);
}

// Floor
inline int FloorToInt(float v) { return (int)std::floor(v); }

// Closest-point-on-triangle (returns point + barycentric weights)
inline Vector3 ClosestPointTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c,
    float& u, float& v, float& w)
{
    Vector3 ab = Sub(b, a);
    Vector3 ac = Sub(c, a);
    Vector3 ap = Sub(p, a);
    float d1 = Dot(ab, ap);
    float d2 = Dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) { u = 1.0f; v = 0.0f; w = 0.0f; return a; }

    Vector3 bp = Sub(p, b);
    float d3 = Dot(ab, bp);
    float d4 = Dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) { u = 0.0f; v = 1.0f; w = 0.0f; return b; }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        v = d1 / (d1 - d3); u = 1.0f - v; w = 0.0f;
        return Add(a, Scale(ab, v));
    }

    Vector3 cp = Sub(p, c);
    float d5 = Dot(ab, cp);
    float d6 = Dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) { u = 0.0f; v = 0.0f; w = 1.0f; return c; }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        w = d2 / (d2 - d6); u = 1.0f - w; v = 0.0f;
        return Add(a, Scale(ac, w));
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        float d43 = d4 - d3;
        float d56 = d5 - d6;
        w = d43 / (d43 + d56); v = 1.0f - w; u = 0.0f;
        return Add(b, Scale(Sub(c, b), w));
    }

    float den = 1.0f / (va + vb + vc);
    v = vb * den;
    w = vc * den;
    u = 1.0f - v - w;
    return Add(a, Add(Scale(ab, v), Scale(ac, w)));
}