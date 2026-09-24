#include "mirror_profile.h"
#include "config.h"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

static const float MM = 1000.0f;

// Loads a mirror_profile.csv: columns theta_deg,r_mm,z_mm (header optional),
// one ring per row, z measured up along the optical axis from the camera
// focus O — see the format description in docs/tasks/mirror-camera-vision.md.
// Converts to metres and returns rows sorted ascending by z (the file is
// expected to already be monotonic in z, since r = f(h) must be single-
// valued for intersectRay's bisection to work; this just guards against the
// file being written tip-first vs. base-first).
static bool loadMirrorCsv(const std::string& path, std::vector<float>& outZ, std::vector<float>& outR)
{
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[MirrorProfile] Cannot open profile CSV: " << path << std::endl;
        return false;
    }

    std::vector<float> z, r;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string tok;
        std::vector<std::string> cols;
        while (std::getline(ss, tok, ',')) cols.push_back(tok);
        if (cols.size() < 3) continue;
        try {
            size_t idx;
            (void)std::stof(cols[0], &idx); // theta_deg, unused but validates the row is numeric
            float rMm = std::stof(cols[1]);
            float zMm = std::stof(cols[2]);
            r.push_back(rMm / MM);
            z.push_back(zMm / MM);
        } catch (const std::exception&) {
            continue; // header row or malformed line — skip
        }
    }

    if (z.size() < 2) {
        std::cerr << "[MirrorProfile] Profile CSV has fewer than 2 valid rows: " << path << std::endl;
        return false;
    }

    if (z.front() > z.back()) {
        std::reverse(z.begin(), z.end());
        std::reverse(r.begin(), r.end());
    }

    outZ = std::move(z);
    outR = std::move(r);
    return true;
}

void MirrorProfile::loadFromConfig(Config& cfg, const std::string& base, float cameraHeight)
{
    std::string typeStr = cfg.getString(base + "/type", "cone");
    if (typeStr == "hyperbola") m_type = Type::Hyperbola;
    else if (typeStr == "profile") m_type = Type::Profile;
    else m_type = Type::Cone;

    m_baseHeight   = cfg.getFloat(base + "/base_height",   150.0f) / MM;
    m_baseDiameter = cfg.getFloat(base + "/base_diameter", 170.0f) / MM;
    m_baseRadius   = m_baseDiameter * 0.5f;
    m_coneHeight   = cfg.getFloat(base + "/cone_height",   40.0f)  / MM;
    m_hyperbolaA   = cfg.getFloat(base + "/hyperbola_a",   35.0f)  / MM;
    m_hyperbolaB   = cfg.getFloat(base + "/hyperbola_b",   40.0f)  / MM;

    // Derived hyperbola geometry. The mirror is the upper branch of
    // (z/a)^2 - (r/b)^2 = 1, z measured upward from the mirror center. The
    // base edge sits at z = zBase where r = baseRadius; the tip is at z = a.
    m_c = std::sqrt(m_hyperbolaA * m_hyperbolaA + m_hyperbolaB * m_hyperbolaB);
    float scaled = (m_hyperbolaA * m_baseRadius) / m_hyperbolaB;
    m_zBase = std::sqrt(m_hyperbolaA * m_hyperbolaA + scaled * scaled);

    if (m_type == Type::Profile) {
        std::string relPath = cfg.getString(base + "/profile_csv", "");
        bool ok = !relPath.empty() && loadMirrorCsv(cfg.configDir() + relPath, m_csvZ, m_csvR);
        if (!ok) {
            std::cerr << "[MirrorProfile] Falling back to type \"cone\" (no usable profile_csv)." << std::endl;
            m_type = Type::Cone;
        } else {
            // The CSV's own origin O is the camera focus, so its z axis maps
            // directly onto robot-local Y as cameraHeight + z (see the
            // requirement that the virtual camera sit exactly at O for the
            // linearity/single-viewpoint property to hold). base_height/
            // base_radius are therefore derived from the table, not the
            // base_height/base_diameter config keys above (which stay
            // unused for this type).
            m_baseHeight = cameraHeight + m_csvZ.back();
            m_baseRadius = m_csvR.back();
        }
    }
}

// Linear interpolation of r(z) over the table, z ascending. Clamps to the
// endpoints outside the table's range (shouldn't happen in practice since
// callers already clamp height to [0, mirrorHeight()]).
static float interpTable(const std::vector<float>& z, const std::vector<float>& r, float zq)
{
    if (zq <= z.front()) return r.front();
    if (zq >= z.back()) return r.back();
    size_t lo = 0, hi = z.size() - 1;
    while (lo + 1 < hi) {
        size_t mid = (lo + hi) / 2;
        if (z[mid] < zq) lo = mid; else hi = mid;
    }
    float t = (zq - z[lo]) / (z[hi] - z[lo]);
    return r[lo] + t * (r[hi] - r[lo]);
}

float MirrorProfile::mirrorHeight() const
{
    if (m_type == Type::Hyperbola) return m_zBase - m_hyperbolaA;
    if (m_type == Type::Profile) return m_csvZ.back() - m_csvZ.front();
    return m_coneHeight;
}

float MirrorProfile::radiusAt(float height) const
{
    float h = std::clamp(height, 0.0f, mirrorHeight());
    if (m_type == Type::Hyperbola) {
        float z = m_zBase - h; // z in [a, zBase]
        float arg = z * z - m_hyperbolaA * m_hyperbolaA;
        if (arg < 0.0f) arg = 0.0f;
        return (m_hyperbolaB / m_hyperbolaA) * std::sqrt(arg);
    }
    if (m_type == Type::Profile) {
        // h=0 (base) -> z=csvZ.back() (max); h=mirrorHeight (tip) -> z=csvZ.front() (min).
        float z = m_csvZ.back() - h;
        return interpTable(m_csvZ, m_csvR, z);
    }
    // Cone: linear from baseRadius at h=0 to 0 at h=coneHeight.
    return m_baseRadius * (1.0f - h / m_coneHeight);
}

float MirrorProfile::slopeAt(float height) const
{
    float h = std::clamp(height, 0.0f, mirrorHeight());
    if (m_type == Type::Profile) {
        // No closed form for a table — central finite difference on radiusAt
        // itself (one-sided at the ends), per the generic-signature note in
        // docs/tasks/mirror-camera-vision.md decision #2.
        float mh = mirrorHeight();
        float eps = std::max(mh * 1e-4f, 1e-7f);
        float h0 = std::max(0.0f, h - eps);
        float h1 = std::min(mh, h + eps);
        float dh = h1 - h0;
        if (dh < 1e-9f) return 0.0f;
        return (radiusAt(h1) - radiusAt(h0)) / dh;
    }
    if (m_type == Type::Hyperbola) {
        float z = m_zBase - h;
        float arg = z * z - m_hyperbolaA * m_hyperbolaA;
        if (arg <= 0.0f) return -1e9f; // tip: vertical surface
        return -(m_hyperbolaB / m_hyperbolaA) * z / std::sqrt(arg);
    }
    return -m_baseRadius / m_coneHeight;
}

glm::vec3 MirrorProfile::normalAt(float height, float radiusOnSurface) const
{
    (void)radiusOnSurface;
    // In the (r, y) plane the profile tangent is (f'(h), -1), so an outward
    // normal is (1, f'(h)). Return it in 3D with the radial component along +X.
    float slope = slopeAt(height);
    glm::vec3 n(1.0f, slope, 0.0f);
    float len = glm::length(n);
    if (len < 1e-6f) return glm::vec3(0.0f, 1.0f, 0.0f);
    return n / len;
}

bool MirrorProfile::intersectRay(const glm::vec3& origin, const glm::vec3& dir, float& tHit) const
{
    if (dir.y <= 1e-6f) return false;

    float h = mirrorHeight();
    float yTip  = m_baseHeight - h; // tip (narrowest, near camera)
    float yBase = m_baseHeight;     // base (top)

    float tTip  = (yTip  - origin.y) / dir.y;
    float tBase = (yBase - origin.y) / dir.y;
    float tA = std::min(tTip, tBase);
    float tB = std::max(tTip, tBase);
    if (tB < 0.0f) return false; // mirror entirely behind the origin
    tA = std::max(tA, 0.0f);

    auto g = [&](float t) {
        glm::vec3 p = origin + dir * t;
        float hh = yBase - p.y; // height from base
        float rad = std::sqrt(p.x * p.x + p.z * p.z);
        return rad - radiusAt(hh);
    };

    const int N = 64;
    float prevT = tA;
    float prevG = g(prevT);
    for (int i = 1; i <= N; i++) {
        float t = tA + (tB - tA) * (float)i / (float)N;
        float gi = g(t);
        if (prevG * gi <= 0.0f) {
            float lo = prevT, hi = t;
            float glo = prevG;
            for (int k = 0; k < 40; k++) {
                float mid = 0.5f * (lo + hi);
                float gm = g(mid);
                if (glo * gm <= 0.0f) hi = mid;
                else { lo = mid; glo = gm; }
            }
            tHit = 0.5f * (lo + hi);
            return true;
        }
        prevT = t;
        prevG = gi;
    }
    return false;
}

glm::vec3 MirrorProfile::effectiveViewpointLocal(float cameraHeight) const
{
    if (m_type == Type::Hyperbola) {
        // Upper focus of the hyperboloid (the single-viewpoint position).
        return glm::vec3(0.0f, m_baseHeight - m_zBase + m_c, 0.0f);
    }
    // Cone: no true SVP, approximate with the physical camera position.
    return glm::vec3(0.0f, cameraHeight, 0.0f);
}
