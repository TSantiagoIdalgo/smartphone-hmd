#include "camera_model.h"

#include <algorithm>
#include <cmath>
#include <locale>
#include <random>
#include <sstream>

namespace {

constexpr int kMinSamples = 30;
constexpr int kRansacRounds = 400;
constexpr int kRansacSampleSize = 8;
constexpr double kMinParallax = 0.05;  // radians between two rays, about 3 degrees

double ProjectedWithMatrix(const double p[3][4], Vec3 x, double* u, double* v)
{
    const double w = p[2][0] * x.x + p[2][1] * x.y + p[2][2] * x.z + p[2][3];
    *u = (p[0][0] * x.x + p[0][1] * x.y + p[0][2] * x.z + p[0][3]) / w;
    *v = (p[1][0] * x.x + p[1][1] * x.y + p[1][2] * x.z + p[1][3]) / w;
    return w;
}

// The eigenvector of the smallest eigenvalue of the symmetric n x n matrix `a` (Jacobi rotations).
std::vector<double> SmallestEigenvector(std::vector<double> a, int n)
{
    std::vector<double> vectors(static_cast<size_t>(n * n), 0.0);
    for (int i = 0; i < n; ++i) {
        vectors[i * n + i] = 1.0;
    }
    double scale = 0.0;
    for (double x : a) {
        scale += x * x;
    }
    for (int sweep = 0; sweep < 100; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < n; ++p)
            for (int q = p + 1; q < n; ++q) off += a[p * n + q] * a[p * n + q];
        if (off <= 1e-30 * scale) {
            break;
        }
        for (int p = 0; p < n; ++p) {
            for (int q = p + 1; q < n; ++q) {
                const double apq = a[p * n + q];
                if (apq == 0.0) {
                    continue;
                }
                const double theta = (a[q * n + q] - a[p * n + p]) / (2.0 * apq);
                const double t = (theta >= 0.0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;
                for (int k = 0; k < n; ++k) {
                    const double kp = a[k * n + p], kq = a[k * n + q];
                    a[k * n + p] = c * kp - s * kq;
                    a[k * n + q] = s * kp + c * kq;
                }
                for (int k = 0; k < n; ++k) {
                    const double pk = a[p * n + k], qk = a[q * n + k];
                    a[p * n + k] = c * pk - s * qk;
                    a[q * n + k] = s * pk + c * qk;
                }
                for (int k = 0; k < n; ++k) {
                    const double kp = vectors[k * n + p], kq = vectors[k * n + q];
                    vectors[k * n + p] = c * kp - s * kq;
                    vectors[k * n + q] = s * kp + c * kq;
                }
            }
        }
    }
    int smallest = 0;
    for (int i = 1; i < n; ++i) {
        if (a[i * n + i] < a[smallest * n + smallest]) {
            smallest = i;
        }
    }
    std::vector<double> out(static_cast<size_t>(n));
    for (int k = 0; k < n; ++k) {
        out[k] = vectors[k * n + smallest];
    }
    return out;
}

// Solves the n x n system a x = b (Gaussian elimination with partial pivoting).
bool Solve(std::vector<double> a, std::vector<double> b, int n, std::vector<double>* x)
{
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int r = col + 1; r < n; ++r) {
            if (std::fabs(a[r * n + col]) > std::fabs(a[pivot * n + col])) {
                pivot = r;
            }
        }
        if (std::fabs(a[pivot * n + col]) < 1e-300) {
            return false;
        }
        if (pivot != col) {
            for (int k = 0; k < n; ++k) std::swap(a[col * n + k], a[pivot * n + k]);
            std::swap(b[col], b[pivot]);
        }
        for (int r = col + 1; r < n; ++r) {
            const double f = a[r * n + col] / a[col * n + col];
            for (int k = col; k < n; ++k) a[r * n + k] -= f * a[col * n + k];
            b[r] -= f * b[col];
        }
    }
    x->assign(static_cast<size_t>(n), 0.0);
    for (int r = n - 1; r >= 0; --r) {
        double sum = b[r];
        for (int k = r + 1; k < n; ++k) sum -= a[r * n + k] * (*x)[k];
        (*x)[r] = sum / a[r * n + r];
    }
    return true;
}

// The direct linear transform: the 3x4 projection that best maps the chosen samples' points to
// their pixels, with the usual normalization so the numbers are of similar size.
bool Dlt(const std::vector<CalibrationSample>& samples, const std::vector<int>& chosen, double p[3][4])
{
    Vec3 centre;
    double cu = 0.0, cv = 0.0;
    const double n = static_cast<double>(chosen.size());
    for (int i : chosen) {
        centre = centre + samples[i].world * (1.0 / n);
        cu += samples[i].u / n;
        cv += samples[i].v / n;
    }
    double spread3 = 0.0, spread2 = 0.0;
    for (int i : chosen) {
        spread3 += Length(samples[i].world - centre) / n;
        spread2 += std::hypot(samples[i].u - cu, samples[i].v - cv) / n;
    }
    if (spread3 < 1e-9 || spread2 < 1e-9) {
        return false;
    }
    const double s3 = std::sqrt(3.0) / spread3;
    const double s2 = std::sqrt(2.0) / spread2;

    std::vector<double> ata(144, 0.0);
    for (int i : chosen) {
        const Vec3 x = (samples[i].world - centre) * s3;
        const double u = (samples[i].u - cu) * s2;
        const double v = (samples[i].v - cv) * s2;
        const double rows[2][12] = {
            {x.x, x.y, x.z, 1, 0, 0, 0, 0, -u * x.x, -u * x.y, -u * x.z, -u},
            {0, 0, 0, 0, x.x, x.y, x.z, 1, -v * x.x, -v * x.y, -v * x.z, -v},
        };
        for (const auto& row : rows)
            for (int a = 0; a < 12; ++a)
                for (int b = 0; b < 12; ++b) ata[a * 12 + b] += row[a] * row[b];
    }
    const std::vector<double> h = SmallestEigenvector(ata, 12);

    // Undo the normalization: P = T2^-1 * Pn * T3.
    const double t3[4][4] = {{s3, 0, 0, -s3 * centre.x}, {0, s3, 0, -s3 * centre.y}, {0, 0, s3, -s3 * centre.z}, {0, 0, 0, 1}};
    const double t2_inverse[3][3] = {{1 / s2, 0, cu}, {0, 1 / s2, cv}, {0, 0, 1}};
    double pn_t3[3][4] = {};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            for (int k = 0; k < 4; ++k) pn_t3[r][c] += h[r * 4 + k] * t3[k][c];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) {
            p[r][c] = 0.0;
            for (int k = 0; k < 3; ++k) p[r][c] += t2_inverse[r][k] * pn_t3[k][c];
        }
    return true;
}

// Splits a projection matrix into the lens and where the camera is (P = K [R | t]), keeping only
// a focal length: the optical centre is taken as the picture's middle.
bool Decompose(const double p[3][4], Vec3 inside, int width, int height, CameraModel* out)
{
    Vec3 m[3];
    double b[3];
    for (int i = 0; i < 3; ++i) {
        m[i] = {p[i][0], p[i][1], p[i][2]};
        b[i] = p[i][3];
    }
    const double norm = Length(m[2]);
    if (norm < 1e-300) {
        return false;
    }
    // The sign that puts the samples in front of the camera.
    double lambda = 1.0 / norm;
    if ((Dot(m[2], inside) + b[2]) * lambda < 0.0) {
        lambda = -lambda;
    }
    const Vec3 a1 = m[0] * lambda, a2 = m[1] * lambda, r3 = m[2] * lambda;
    const double cy = Dot(a2, r3);
    const Vec3 y_part = a2 - r3 * cy;
    const double fy = Length(y_part);
    if (fy < 1e-9) {
        return false;
    }
    const Vec3 r2 = y_part * (1.0 / fy);
    const double cx = Dot(a1, r3);
    const double skew = Dot(a1, r2);
    const Vec3 x_part = a1 - r3 * cx - r2 * skew;
    const double fx = Length(x_part);
    if (fx < 1e-9) {
        return false;
    }
    const Mat3 rotation = Rows(x_part * (1.0 / fx), r2, r3);
    if (Determinant(rotation) <= 0.0) {
        return false;  // a mirrored picture: not a real camera
    }
    const double t3 = b[2] * lambda;
    const double t2 = (b[1] * lambda - cy * t3) / fy;
    const double t1 = (b[0] * lambda - skew * t2 - cx * t3) / fx;
    out->width = width;
    out->height = height;
    out->focal = 0.5 * (fx + fy);
    out->k1 = 0.0;
    out->rotation = QuatToMat3(Mat3ToQuat(rotation));
    out->position = Transpose(out->rotation) * (Vec3{t1, t2, t3} * -1.0);
    return true;
}

// Refines a camera so its pixels match the samples' best (Levenberg-Marquardt on the focal
// length, k1, the rotation and the position).
CameraModel Refine(const CameraModel& start, const std::vector<CalibrationSample>& samples, const std::vector<int>& used)
{
    constexpr int kParams = 8;
    CameraModel camera = start;
    // Parameters: focal, k1, a small rotation (applied before the current one), the camera-frame
    // translation t, where camera = rotation * world + t.
    const auto with = [&](const CameraModel& base, const double* d) {
        CameraModel c = base;
        c.focal = base.focal + d[0];
        c.k1 = base.k1 + d[1];
        c.rotation = RotationFromVector({d[2], d[3], d[4]}) * base.rotation;
        const Vec3 t = base.rotation * (base.position * -1.0) + Vec3{d[5], d[6], d[7]};
        c.position = Transpose(c.rotation) * (t * -1.0);
        return c;
    };
    const auto residuals = [&](const CameraModel& c, std::vector<double>* r) {
        r->resize(used.size() * 2);
        double cost = 0.0;
        for (size_t k = 0; k < used.size(); ++k) {
            const CalibrationSample& s = samples[used[k]];
            double u = 0.0, v = 0.0;
            if (!c.Project(s.world, &u, &v)) {
                u = s.u + 1e4;
                v = s.v + 1e4;
            }
            (*r)[2 * k] = u - s.u;
            (*r)[2 * k + 1] = v - s.v;
            cost += (u - s.u) * (u - s.u) + (v - s.v) * (v - s.v);
        }
        return cost;
    };

    std::vector<double> r, r_plus, r_minus;
    double cost = residuals(camera, &r);
    double damping = 1e-3;
    const double steps[kParams] = {1e-3 * camera.focal, 1e-5, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6};
    for (int iteration = 0; iteration < 100 && damping < 1e12; ++iteration) {
        const size_t count = r.size();
        std::vector<double> jacobian(count * kParams);
        for (int j = 0; j < kParams; ++j) {
            double d[kParams] = {};
            d[j] = steps[j];
            residuals(with(camera, d), &r_plus);
            d[j] = -steps[j];
            residuals(with(camera, d), &r_minus);
            for (size_t i = 0; i < count; ++i) jacobian[i * kParams + j] = (r_plus[i] - r_minus[i]) / (2 * steps[j]);
        }
        std::vector<double> jtj(kParams * kParams, 0.0), jtr(kParams, 0.0);
        for (size_t i = 0; i < count; ++i) {
            const double* row = &jacobian[i * kParams];
            for (int a = 0; a < kParams; ++a) {
                jtr[a] -= row[a] * r[i];
                for (int b = 0; b < kParams; ++b) jtj[a * kParams + b] += row[a] * row[b];
            }
        }
        bool improved = false;
        while (damping < 1e12) {
            std::vector<double> lhs = jtj, delta;
            for (int a = 0; a < kParams; ++a) lhs[a * kParams + a] += damping * std::max(jtj[a * kParams + a], 1e-12);
            if (!Solve(lhs, jtr, kParams, &delta)) {
                damping *= 10.0;
                continue;
            }
            const CameraModel trial = with(camera, delta.data());
            std::vector<double> r_trial;
            const double trial_cost = residuals(trial, &r_trial);
            if (trial_cost < cost) {
                const double gain = cost - trial_cost;
                camera = trial;
                r = r_trial;
                cost = trial_cost;
                damping = std::max(damping / 10.0, 1e-9);
                improved = gain > 1e-12 * cost;
                break;
            }
            damping *= 10.0;
        }
        if (!improved) {
            break;
        }
    }
    return camera;
}

double PixelError(const CameraModel& camera, const CalibrationSample& s)
{
    double u = 0.0, v = 0.0;
    if (!camera.Project(s.world, &u, &v)) {
        return 1e9;
    }
    return std::hypot(u - s.u, v - s.v);
}

}  // namespace

bool CameraModel::Project(Vec3 world, double* u, double* v) const
{
    const Vec3 c = rotation * (world - position);
    if (c.z <= 1e-6) {
        return false;
    }
    const double x = c.x / c.z, y = c.y / c.z;
    const double d = 1.0 + k1 * (x * x + y * y);
    *u = 0.5 * width + focal * x * d;
    *v = 0.5 * height + focal * y * d;
    return true;
}

Vec3 CameraModel::Ray(double u, double v) const
{
    const double xd = (u - 0.5 * width) / focal, yd = (v - 0.5 * height) / focal;
    double x = xd, y = yd;
    for (int i = 0; i < 8; ++i) {
        const double d = 1.0 + k1 * (x * x + y * y);
        x = xd / d;
        y = yd / d;
    }
    const Vec3 direction = Transpose(rotation) * Vec3{x, y, 1.0};
    return direction * (1.0 / Length(direction));
}

std::string CameraModel::Serialize() const
{
    const Quat q = Mat3ToQuat(rotation);
    std::ostringstream out;
    out.imbue(std::locale::classic());  // a point for decimals, whatever the PC's language
    out.precision(9);
    out << "v1 " << width << ' ' << height << ' ' << focal << ' ' << k1 << ' ' << q.w << ' ' << q.x << ' ' << q.y
        << ' ' << q.z << ' ' << position.x << ' ' << position.y << ' ' << position.z;
    return out.str();
}

bool CameraModel::Parse(const std::string& text, CameraModel* out)
{
    CameraModel c;
    Quat q;
    std::istringstream in(text);
    in.imbue(std::locale::classic());
    std::string version;
    in >> version >> c.width >> c.height >> c.focal >> c.k1 >> q.w >> q.x >> q.y >> q.z >> c.position.x >> c.position.y >>
        c.position.z;
    if (!in || version != "v1") {
        return false;
    }
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (c.width <= 0 || c.height <= 0 || !(c.focal > 0.0) || !(norm > 0.5) || !std::isfinite(c.position.x + c.position.y + c.position.z)) {
        return false;
    }
    c.rotation = QuatToMat3(Normalize(q));
    *out = c;
    return true;
}

CalibrationResult CalibrateCamera(const std::vector<CalibrationSample>& samples, int width, int height)
{
    CalibrationResult result;
    const int count = static_cast<int>(samples.size());
    if (count < kMinSamples) {
        result.problem = "too few samples";
        return result;
    }
    const double threshold = 0.0125 * std::max(width, height);
    Vec3 inside;
    for (const CalibrationSample& s : samples) {
        inside = inside + s.world * (1.0 / count);
    }

    // RANSAC: projections from small random sets, keeping the one most samples agree with.
    std::mt19937 random(12345);
    std::uniform_int_distribution<int> pick(0, count - 1);
    std::vector<int> best;
    for (int round = 0; round < kRansacRounds; ++round) {
        std::vector<int> chosen;
        while (static_cast<int>(chosen.size()) < kRansacSampleSize) {
            const int i = pick(random);
            if (std::find(chosen.begin(), chosen.end(), i) == chosen.end()) {
                chosen.push_back(i);
            }
        }
        double p[3][4];
        if (!Dlt(samples, chosen, p)) {
            continue;
        }
        const double inside_w = p[2][0] * inside.x + p[2][1] * inside.y + p[2][2] * inside.z + p[2][3];
        std::vector<int> agree;
        for (int i = 0; i < count; ++i) {
            double u = 0.0, v = 0.0;
            const double w = ProjectedWithMatrix(p, samples[i].world, &u, &v);
            if (w * inside_w > 0.0 && std::hypot(u - samples[i].u, v - samples[i].v) < threshold) {
                agree.push_back(i);
            }
        }
        if (agree.size() > best.size()) {
            best.swap(agree);
        }
    }
    if (static_cast<int>(best.size()) < kMinSamples) {
        result.problem = "the samples do not fit any camera";
        return result;
    }

    double p[3][4];
    CameraModel camera;
    if (!Dlt(samples, best, p) || !Decompose(p, inside, width, height, &camera)) {
        result.problem = "the samples do not fit a real camera";
        return result;
    }
    std::vector<int> used = best;
    for (int pass = 0; pass < 3; ++pass) {
        camera = Refine(camera, samples, used);
        double sum = 0.0;
        for (int i : used) {
            const double e = PixelError(camera, samples[i]);
            sum += e * e;
        }
        const double rms = std::sqrt(sum / static_cast<double>(used.size()));
        const double cut = std::min(threshold, std::max(3.0 * rms, 2.0));
        used.clear();
        for (int i = 0; i < count; ++i) {
            if (PixelError(camera, samples[i]) < cut) {
                used.push_back(i);
            }
        }
        if (static_cast<int>(used.size()) < kMinSamples) {
            break;
        }
    }
    camera = Refine(camera, samples, used);

    double sum_px = 0.0, sum_m = 0.0;
    for (int i : used) {
        const double e = PixelError(camera, samples[i]);
        const double depth = Dot(Row(camera.rotation, 2), samples[i].world - camera.position);
        sum_px += e * e;
        sum_m += (e * depth / camera.focal) * (e * depth / camera.focal);
    }
    result.camera = camera;
    result.inliers = static_cast<int>(used.size());
    result.outliers = count - result.inliers;
    result.rms_px = result.inliers > 0 ? std::sqrt(sum_px / result.inliers) : 0.0;
    result.rms_m = result.inliers > 0 ? std::sqrt(sum_m / result.inliers) : 0.0;
    const int longest = std::max(width, height);
    if (result.inliers < kMinSamples || result.inliers * 2 < count) {
        result.problem = "most samples do not agree";
    } else if (!(camera.focal > 0.2 * longest && camera.focal < 5.0 * longest)) {
        result.problem = "the lens came out implausible";
    } else if (std::fabs(camera.k1) > 0.8) {
        result.problem = "the lens distortion came out implausible";
    } else {
        result.ok = true;
    }
    return result;
}

bool Triangulate(const std::vector<Sighting>& sightings, Vec3* out)
{
    if (sightings.size() < 2) {
        return false;
    }
    Mat3 a;
    Vec3 b;
    std::vector<Vec3> rays;
    for (const Sighting& s : sightings) {
        const Vec3 d = s.camera->Ray(s.u, s.v);
        const Mat3 across = Mat3::Identity() - Outer(d, d);
        a = a + across;
        b = b + across * s.camera->position;
        rays.push_back(d);
    }
    double widest = 0.0;
    for (size_t i = 0; i < rays.size(); ++i)
        for (size_t j = i + 1; j < rays.size(); ++j) widest = std::max(widest, Length(Cross(rays[i], rays[j])));
    Mat3 inverse;
    if (widest < std::sin(kMinParallax) || !Inverse(a, &inverse)) {
        return false;
    }
    *out = inverse * b;
    return true;
}
