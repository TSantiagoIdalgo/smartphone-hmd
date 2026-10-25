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

// Spherical interpolation from `a` (t = 0) to `b` (t = 1) along the shorter arc.
inline Quat Slerp(Quat a, Quat b, double t)
{
    double cosine = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (cosine < 0.0) {
        b = {-b.w, -b.x, -b.y, -b.z};
        cosine = -cosine;
    }
    double wa = 1.0 - t;
    double wb = t;
    if (cosine < 0.9995) {
        const double angle = std::acos(cosine);
        const double s = std::sin(angle);
        wa = std::sin((1.0 - t) * angle) / s;
        wb = std::sin(t * angle) / s;
    }
    return Normalize({wa * a.w + wb * b.w, wa * a.x + wb * b.x, wa * a.y + wb * b.y, wa * a.z + wb * b.z});
}

// 3x3 matrix for the head position filter's covariances.
struct Mat3 {
    double m[3][3] = {};

    static Mat3 Identity(double s = 1.0)
    {
        Mat3 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = s;
        return r;
    }
};

inline Mat3 operator+(const Mat3& a, const Mat3& b)
{
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][j] + b.m[i][j];
    return r;
}

inline Mat3 operator-(const Mat3& a, const Mat3& b)
{
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][j] - b.m[i][j];
    return r;
}

inline Mat3 operator*(const Mat3& a, double s)
{
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][j] * s;
    return r;
}

inline Mat3 operator*(const Mat3& a, const Mat3& b)
{
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}

inline Vec3 operator*(const Mat3& a, Vec3 v)
{
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}

inline Mat3 Transpose(const Mat3& a)
{
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[j][i];
    return r;
}

// a * b^T
inline Mat3 Outer(Vec3 a, Vec3 b)
{
    const double u[3] = {a.x, a.y, a.z};
    const double v[3] = {b.x, b.y, b.z};
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = u[i] * v[j];
    return r;
}

// False if `a` is (nearly) singular, relative to the size of its entries.
inline bool Inverse(const Mat3& a, Mat3* out)
{
    const auto& m = a.m;
    double scale = 0.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) scale = std::fmax(scale, std::fabs(m[i][j]));
    const double c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    const double c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    const double c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    const double det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    if (!(std::fabs(det) > 1e-12 * scale * scale * scale)) {
        return false;
    }
    const double s = 1.0 / det;
    out->m[0][0] = c00 * s;
    out->m[1][0] = c01 * s;
    out->m[2][0] = c02 * s;
    out->m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * s;
    out->m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * s;
    out->m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * s;
    out->m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * s;
    out->m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * s;
    out->m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * s;
    return true;
}

inline Mat3 Rows(Vec3 a, Vec3 b, Vec3 c)
{
    Mat3 r;
    const Vec3 rows[3] = {a, b, c};
    for (int i = 0; i < 3; ++i) {
        r.m[i][0] = rows[i].x;
        r.m[i][1] = rows[i].y;
        r.m[i][2] = rows[i].z;
    }
    return r;
}

inline Vec3 Row(const Mat3& a, int i) { return {a.m[i][0], a.m[i][1], a.m[i][2]}; }

inline double Determinant(const Mat3& a)
{
    return Dot(Row(a, 0), Cross(Row(a, 1), Row(a, 2)));
}

// The rotation matrix of a quaternion: Rotate(q, v) == QuatToMat3(q) * v.
inline Mat3 QuatToMat3(Quat q)
{
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    return Rows({1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)});
}

// The quaternion of a rotation matrix (nearly orthonormal is enough).
inline Quat Mat3ToQuat(const Mat3& r)
{
    const auto& m = r.m;
    const double trace = m[0][0] + m[1][1] + m[2][2];
    Quat q;
    if (trace > 0.0) {
        const double s = 2.0 * std::sqrt(trace + 1.0);
        q = {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
        q = {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
    } else if (m[1][1] > m[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]);
        q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s};
    } else {
        const double s = 2.0 * std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]);
        q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s};
    }
    return Normalize(q);
}

// The rotation by the vector's length (radians) around its direction.
inline Mat3 RotationFromVector(Vec3 v)
{
    const double angle = Length(v);
    return angle > 0.0 ? QuatToMat3(AxisAngle(v * (1.0 / angle), angle)) : Mat3::Identity();
}

// Where `orientation` will be after `seconds` turning at `world_angular_velocity` (rad/s).
inline Quat Predict(Quat orientation, Vec3 world_angular_velocity, double seconds)
{
    const double speed = Length(world_angular_velocity);
    if (speed <= 0.0 || seconds <= 0.0) {
        return orientation;
    }
    return Normalize(AxisAngle(world_angular_velocity * (1.0 / speed), speed * seconds) * orientation);
}
