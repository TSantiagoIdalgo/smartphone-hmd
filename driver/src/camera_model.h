#pragma once

#include <string>
#include <vector>

#include "math3d.h"

// A body tracking camera (a phone, later perhaps a webcam) as its calibration found it: a pinhole
// with one term of radial lens distortion, its optical centre in the middle of the picture.
// Camera axes: +x right, +y down, +z out of the lens. Pixels count from the upright picture's
// top-left corner, as the pose points do (x * width, y * height).
struct CameraModel {
    int width = 0;
    int height = 0;
    double focal = 0.0;  // pixels
    double k1 = 0.0;     // a point r focal lengths from the centre lands at r (1 + k1 r^2)
    Mat3 rotation = Mat3::Identity();  // world to camera
    Vec3 position;                     // the lens, world

    // The pixel `world` lands on; false if it is behind the camera.
    bool Project(Vec3 world, double* u, double* v) const;
    // The world direction (unit) that lands on pixel (u, v).
    Vec3 Ray(double u, double v) const;

    // One line of text, for the driver's settings, and back.
    std::string Serialize() const;
    static bool Parse(const std::string& text, CameraModel* out);
};

// A Move's bulb at a known place (from PSMoveServiceEx) and where a camera saw it.
struct CalibrationSample {
    Vec3 world;  // m
    double u = 0.0;
    double v = 0.0;
};

struct CalibrationResult {
    bool ok = false;
    std::string problem;  // why not, when !ok
    CameraModel camera;
    int inliers = 0;
    int outliers = 0;    // samples left out as wrong (a misread bulb, a mismatched time)
    double rms_px = 0.0;
    double rms_m = 0.0;  // the same error as a distance, at each sample's depth
};

// Finds where a camera is, where it looks and its lens from many samples spread through the room.
// Robust to a share of wrong samples.
CalibrationResult CalibrateCamera(const std::vector<CalibrationSample>& samples, int width, int height);

// A pixel where a calibrated camera sees something.
struct Sighting {
    const CameraModel* camera;
    double u;
    double v;
};

// The point closest to the sightings' rays (least squares). False if there are fewer than two or
// they are nearly parallel.
bool Triangulate(const std::vector<Sighting>& sightings, Vec3* out);
