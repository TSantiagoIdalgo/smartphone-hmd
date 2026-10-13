#pragma once

#include <cmath>

// Minimal vector and quaternion helpers. Quaternions rotate body (head) vectors into the world.
struct Vec3 {
    double x = 0, y = 0, z = 0;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double Length(Vec3 a) { return std::sqrt(Dot(a, a)); }

struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
};

inline Quat operator*(Quat a, Quat b)
{
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

inline Quat Conjugate(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }

inline Quat Normalize(Quat q)
{
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

// Rotation of `angle` radians around the unit vector `axis`.
inline Quat AxisAngle(Vec3 axis, double angle)
{
    const double s = std::sin(angle * 0.5);
    return {std::cos(angle * 0.5), axis.x * s, axis.y * s, axis.z * s};
}

inline Vec3 Rotate(Quat q, Vec3 v)
{
    const Quat r = q * Quat{0, v.x, v.y, v.z} * Conjugate(q);
    return {r.x, r.y, r.z};
}
