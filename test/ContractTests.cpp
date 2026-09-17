// Contract tests for IVHACD::Compute: typed results, input validation, cancellation, instance reuse and
// degenerate parameters. Build in a configuration with assertions enabled as well as an optimized one;
// the library's internal invariants are checked by its asserts.
#define ENABLE_VHACD_IMPLEMENTATION 1
#include "VHACD.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <random>
#include <vector>

namespace
{

using VHACD::IVHACD;
using Result = IVHACD::ComputeResult;

int g_failures = 0;

#define CHECK(expr)                                                                                                    \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(expr))                                                                                                   \
        {                                                                                                              \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #expr);                                              \
            ++g_failures;                                                                                              \
        }                                                                                                              \
    } while (false)

struct Mesh
{
    std::vector<double> points;
    std::vector<uint32_t> triangles;
};

Mesh Box(const double sx, const double sy, const double sz)
{
    Mesh m;
    for (int i = 0; i < 8; ++i)
    {
        m.points.insert(m.points.end(), {(i & 1) ? sx : 0.0, (i & 2) ? sy : 0.0, (i & 4) ? sz : 0.0});
    }
    m.triangles = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                   2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
    return m;
}

// Two overlapping boxes forming an L, which is concave.
Mesh LShape()
{
    Mesh a = Box(3, 1, 1);
    const Mesh b = Box(1, 3, 1);
    for (const uint32_t t : b.triangles)
    {
        a.triangles.push_back(t + 8);
    }
    a.points.insert(a.points.end(), b.points.begin(), b.points.end());
    return a;
}

IVHACD::Parameters DefaultParams()
{
    IVHACD::Parameters p;
    p.m_maxConvexHulls = 8;
    p.m_resolution = 20000;
    p.m_maxNumVerticesPerCH = 32;
    p.m_maxRecursionDepth = 6;
    return p;
}

Result Run(IVHACD& v, const Mesh& m, const IVHACD::Parameters& p)
{
    return v.Compute(m.points.data(), uint32_t(m.points.size() / 3), m.triangles.data(),
                     uint32_t(m.triangles.size() / 3), p);
}

struct CancelOnFirstUpdate final : IVHACD::IUserCallback
{
    IVHACD* target = nullptr;
    int updates = 0;

    void Update(double, double, const char* const, const char*) override
    {
        if (++updates == 1)
        {
            target->Cancel();
        }
    }
};

struct RecordingLogger final : IVHACD::IUserLogger
{
    int messages = 0;
    char last[512] = {};

    void Log(const char* const msg) override
    {
        ++messages;
        std::snprintf(last, sizeof(last), "%s", msg);
    }
};

std::vector<std::vector<double>> Snapshot(const IVHACD& v)
{
    std::vector<std::vector<double>> hulls;
    for (uint32_t i = 0; i < v.GetNConvexHulls(); ++i)
    {
        IVHACD::ConvexHull ch;
        v.GetConvexHull(i, ch);
        std::vector<double> flat;
        for (const VHACD::Vertex& p : ch.m_points)
        {
            flat.insert(flat.end(), {p.mX, p.mY, p.mZ});
        }
        hulls.push_back(flat);
    }
    return hulls;
}

void ValidMeshCompletes(IVHACD& v)
{
    CHECK(Run(v, LShape(), DefaultParams()) == Result::Completed);
    CHECK(v.GetNConvexHulls() > 1);
}

void InvalidMeshIsRejectedAndReleasesResults(IVHACD& v)
{
    RecordingLogger logger;
    IVHACD::Parameters p = DefaultParams();
    p.m_logger = &logger;

    CHECK(Run(v, LShape(), p) == Result::Completed);
    Mesh nan = Box(1, 1, 1);
    nan.points[4] = std::numeric_limits<double>::quiet_NaN();
    CHECK(Run(v, nan, p) == Result::InvalidInput);
    CHECK(v.GetNConvexHulls() == 0);
    CHECK(logger.messages >= 1 && std::strstr(logger.last, "not finite") != nullptr);

    Mesh infinite = Box(1, 1, 1);
    infinite.points[0] = std::numeric_limits<double>::infinity();
    CHECK(Run(v, infinite, p) == Result::InvalidInput);

    Mesh badIndex = Box(1, 1, 1);
    badIndex.triangles[5] = 8;
    CHECK(Run(v, badIndex, p) == Result::InvalidInput);
    CHECK(std::strstr(logger.last, "index") != nullptr);

    const std::vector<uint32_t> triangle = {0, 1, 2};
    CHECK(v.Compute(static_cast<const double*>(nullptr), 3, triangle.data(), 1, p) == Result::InvalidInput);
    const std::vector<double> points = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    CHECK(v.Compute(points.data(), 3, nullptr, 1, p) == Result::InvalidInput);

    const std::vector<float> floatPoints = {0, 0, 0, 1, 0, 0, std::numeric_limits<float>::quiet_NaN(), 1, 0};
    CHECK(v.Compute(floatPoints.data(), 3, triangle.data(), 1, p) == Result::InvalidInput);
}

void InvalidParametersAreRejected(IVHACD& v)
{
    const Mesh box = Box(1, 1, 1);
    IVHACD::Parameters p = DefaultParams();
    p.m_maxConvexHulls = 0;
    CHECK(Run(v, box, p) == Result::InvalidInput);

    p = DefaultParams();
    p.m_maxNumVerticesPerCH = 3;
    CHECK(Run(v, box, p) == Result::InvalidInput);

    p = DefaultParams();
    p.m_minimumVolumePercentErrorAllowed = std::numeric_limits<double>::quiet_NaN();
    CHECK(Run(v, box, p) == Result::InvalidInput);
    p.m_minimumVolumePercentErrorAllowed = -1;
    CHECK(Run(v, box, p) == Result::InvalidInput);

    p = DefaultParams();
    p.m_resolution = std::numeric_limits<uint32_t>::max();
    CHECK(Run(v, box, p) == Result::InvalidInput);

    p = DefaultParams();
    p.m_tiltedSurfaceAllowance = std::numeric_limits<double>::infinity();
    CHECK(Run(v, box, p) == Result::InvalidInput);
    p.m_tiltedSurfaceAllowance = -0.5;
    CHECK(Run(v, box, p) == Result::InvalidInput);
}

// A unit cube rotated off every axis is convex, so it must remain one hull. Its voxelized faces are
// staircases whose corner hull always exceeds the voxels; without the tilted-surface allowance that
// gap reads as concavity and the cube splits down to the recursion limit.
void RotatedCubeRemainsOneHull(IVHACD& v)
{
    const double a = 0.5;
    const double c = std::cos(a);
    const double s = std::sin(a);
    Mesh cube = Box(1, 1, 1);
    for (size_t i = 0; i < cube.points.size(); i += 3)
    {
        const double x = cube.points[i];
        const double y = cube.points[i + 1];
        const double z = cube.points[i + 2];
        cube.points[i] = x * c - y * s;
        cube.points[i + 1] = x * s * c + y * c * c - z * s;
        cube.points[i + 2] = x * s * s + y * c * s + z * c;
    }
    IVHACD::Parameters p;
    p.m_maxConvexHulls = 32;
    p.m_resolution = 100000;
    p.m_maxNumVerticesPerCH = 44;
    p.m_maxRecursionDepth = 9;
    CHECK(Run(v, cube, p) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 1);

    p.m_tiltedSurfaceAllowance = 0;
    CHECK(Run(v, cube, p) == Result::Completed);
    CHECK(v.GetNConvexHulls() > 1);
}

void EmptyMeshCompletesWithoutHulls(IVHACD& v)
{
    CHECK(v.Compute(static_cast<const double*>(nullptr), 0, nullptr, 0, DefaultParams()) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 0);
}

void ZeroMinimumEdgeLengthNeverProducesEmptyPieces(IVHACD& v)
{
    IVHACD::Parameters p = DefaultParams();
    p.m_minEdgeLength = 0;
    p.m_maxRecursionDepth = 12;
    p.m_minimumVolumePercentErrorAllowed = 0;
    CHECK(Run(v, LShape(), p) == Result::Completed);
    CHECK(v.GetNConvexHulls() > 0);
    for (uint32_t i = 0; i < v.GetNConvexHulls(); ++i)
    {
        IVHACD::ConvexHull ch;
        v.GetConvexHull(i, ch);
        CHECK(std::isfinite(ch.m_volume) && ch.m_volume > 0);
    }
}

void DepthZeroDoesNotSplit(IVHACD& v)
{
    IVHACD::Parameters p = DefaultParams();
    p.m_maxRecursionDepth = 0;
    CHECK(Run(v, LShape(), p) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 1);
}

void CancelFromUpdateStopsAndReleasesResults(IVHACD& v)
{
    IVHACD::Parameters p = DefaultParams();
    CancelOnFirstUpdate cancel;
    cancel.target = &v;
    p.m_callback = &cancel;
    CHECK(Run(v, LShape(), p) == Result::Canceled);
    CHECK(v.GetNConvexHulls() == 0);
}

void ReusedInstanceReproducesResults(IVHACD& v)
{
    const IVHACD::Parameters p = DefaultParams();
    CHECK(Run(v, LShape(), p) == Result::Completed);
    const auto first = Snapshot(v);
    CHECK(Run(v, Box(2, 1, 3), p) == Result::Completed);
    CHECK(Run(v, LShape(), p) == Result::Completed);
    CHECK(Snapshot(v) == first);

    IVHACD* const fresh = VHACD::CreateVHACD();
    CHECK(Run(*fresh, LShape(), p) == Result::Completed);
    CHECK(Snapshot(*fresh) == first);
    fresh->Release();
}

// m_center must be the centroid of the returned hull as a uniform solid. The oracle integrates the
// hull's triangles over tetrahedra from a point far outside the hull instead of from a hull vertex.
void CenterIsTheSolidCentroid(IVHACD& v)
{
    CHECK(Run(v, LShape(), DefaultParams()) == Result::Completed);
    CHECK(v.GetNConvexHulls() > 1);
    for (uint32_t h = 0; h < v.GetNConvexHulls(); ++h)
    {
        IVHACD::ConvexHull ch;
        CHECK(v.GetConvexHull(h, ch));
        const double reference[3] = {-10.0, 7.0, 13.0};
        double moment[3] = {0, 0, 0};
        double volume6 = 0;
        for (const VHACD::Triangle& t : ch.m_triangles)
        {
            const VHACD::Vertex& a = ch.m_points[t.mI0];
            const VHACD::Vertex& b = ch.m_points[t.mI1];
            const VHACD::Vertex& c = ch.m_points[t.mI2];
            const double ax = a.mX - reference[0], ay = a.mY - reference[1], az = a.mZ - reference[2];
            const double bx = b.mX - reference[0], by = b.mY - reference[1], bz = b.mZ - reference[2];
            const double cx = c.mX - reference[0], cy = c.mY - reference[1], cz = c.mZ - reference[2];
            const double tetra6 = ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
            moment[0] += tetra6 * (a.mX + b.mX + c.mX + reference[0]);
            moment[1] += tetra6 * (a.mY + b.mY + c.mY + reference[1]);
            moment[2] += tetra6 * (a.mZ + b.mZ + c.mZ + reference[2]);
            volume6 += tetra6;
        }
        CHECK(std::abs(volume6) > 1e-9);
        for (int axis = 0; axis < 3; ++axis)
        {
            CHECK(std::abs(ch.m_center[axis] - moment[axis] / (4.0 * volume6)) < 1e-9);
        }
    }
}

// A hull limited to fewer vertices than it has input points adds the farthest remaining point first, so
// points far outside all the others are kept whatever order the faces were created in.
void LimitedHullKeepsFarthestPoints()
{
    std::vector<VHACD::Vertex> points;
    const double golden = 3.14159265358979323846 * (3.0 - std::sqrt(5.0));
    for (int i = 0; i < 200; ++i)
    {
        const double y = 1.0 - 2.0 * (i + 0.5) / 200.0;
        const double r = std::sqrt(1.0 - y * y);
        points.emplace_back(r * std::cos(golden * i), y, r * std::sin(golden * i));
    }
    const VHACD::Vertex spikes[] = {VHACD::Vertex(3.3, -0.4, -0.9), VHACD::Vertex(-2.7, -3.5, -0.2), VHACD::Vertex(3.25, -1.1, -1.6)};
    points.insert(points.end(), std::begin(spikes), std::end(spikes));

    const auto keepsSpikes = [&](const VHACD::QuickHull& hull) {
        for (const VHACD::Vertex& spike : spikes)
        {
            bool kept = false;
            for (const VHACD::Vertex& p : hull.GetVertices())
            {
                kept = kept || (p.mX == spike.mX && p.mY == spike.mY && p.mZ == spike.mZ);
            }
            if (!kept)
            {
                return false;
            }
        }
        return true;
    };

    // Every spike is a vertex of the full hull, so a hull that drops one misses part of the shape.
    VHACD::ConvexHullWorkspace workspace;
    VHACD::QuickHull full;
    CHECK(full.ComputeConvexHull(workspace, points, uint32_t(points.size())) > 0);
    CHECK(keepsSpikes(full));

    // Taking the oldest face first spends this budget on sphere points and drops a spike.
    VHACD::QuickHull limited;
    CHECK(limited.ComputeConvexHull(workspace, points, 8) > 0);
    CHECK(limited.GetVertices().size() <= 8);
    CHECK(keepsSpikes(limited));
}

// Exact integers for the orientation oracle: sign and magnitude in 32-bit limbs, least significant first.
struct ExactInt
{
    int sign = 0;
    std::vector<uint32_t> limbs;
};

void TrimLimbs(std::vector<uint32_t>& limbs)
{
    while (!limbs.empty() && limbs.back() == 0)
    {
        limbs.pop_back();
    }
}

int CompareMagnitudes(const std::vector<uint32_t>& a, const std::vector<uint32_t>& b)
{
    if (a.size() != b.size())
    {
        return a.size() < b.size() ? -1 : 1;
    }
    for (size_t i = a.size(); i-- > 0;)
    {
        if (a[i] != b[i])
        {
            return a[i] < b[i] ? -1 : 1;
        }
    }
    return 0;
}

ExactInt AddExact(const ExactInt& a, const ExactInt& b)
{
    if (a.sign == 0)
    {
        return b;
    }
    if (b.sign == 0)
    {
        return a;
    }
    ExactInt out;
    if (a.sign == b.sign)
    {
        out.sign = a.sign;
        out.limbs.assign(std::max(a.limbs.size(), b.limbs.size()) + 1, 0);
        uint64_t carry = 0;
        for (size_t i = 0; i < out.limbs.size(); ++i)
        {
            uint64_t sum = carry;
            sum += i < a.limbs.size() ? a.limbs[i] : 0;
            sum += i < b.limbs.size() ? b.limbs[i] : 0;
            out.limbs[i] = uint32_t(sum);
            carry = sum >> 32;
        }
        TrimLimbs(out.limbs);
        return out;
    }
    const int order = CompareMagnitudes(a.limbs, b.limbs);
    if (order == 0)
    {
        return out;
    }
    const ExactInt& larger = order > 0 ? a : b;
    const ExactInt& smaller = order > 0 ? b : a;
    out.sign = larger.sign;
    out.limbs.assign(larger.limbs.size(), 0);
    int64_t borrow = 0;
    for (size_t i = 0; i < larger.limbs.size(); ++i)
    {
        int64_t difference = int64_t(larger.limbs[i]) - borrow - (i < smaller.limbs.size() ? int64_t(smaller.limbs[i]) : 0);
        borrow = difference < 0 ? 1 : 0;
        out.limbs[i] = uint32_t(difference + (borrow << 32));
    }
    TrimLimbs(out.limbs);
    return out;
}

ExactInt SubtractExact(const ExactInt& a, ExactInt b)
{
    b.sign = -b.sign;
    return AddExact(a, b);
}

ExactInt MultiplyExact(const ExactInt& a, const ExactInt& b)
{
    ExactInt out;
    if (a.sign == 0 || b.sign == 0)
    {
        return out;
    }
    out.sign = a.sign * b.sign;
    out.limbs.assign(a.limbs.size() + b.limbs.size(), 0);
    for (size_t i = 0; i < a.limbs.size(); ++i)
    {
        uint64_t carry = 0;
        for (size_t j = 0; j < b.limbs.size(); ++j)
        {
            const uint64_t t = uint64_t(a.limbs[i]) * b.limbs[j] + out.limbs[i + j] + carry;
            out.limbs[i + j] = uint32_t(t);
            carry = t >> 32;
        }
        for (size_t k = i + b.limbs.size(); carry != 0; ++k)
        {
            const uint64_t t = uint64_t(out.limbs[k]) + carry;
            out.limbs[k] = uint32_t(t);
            carry = t >> 32;
        }
    }
    TrimLimbs(out.limbs);
    return out;
}

// value = mantissa * 2^exponent with an integer mantissa below 2^53.
int ExactExponent(const double value)
{
    int exponent;
    std::frexp(value, &exponent);
    return exponent - 53;
}

// value / 2^base as an exact integer; base must not exceed ExactExponent(value).
ExactInt ExactFromDouble(const double value, const int base)
{
    ExactInt out;
    if (value == 0)
    {
        return out;
    }
    int exponent;
    const uint64_t mantissa = uint64_t(std::ldexp(std::frexp(std::fabs(value), &exponent), 53));
    const int shift = exponent - 53 - base;
    // A multiple of 2^base keeps its highest mantissa bit at or above bit 0 of the result.
    const int highest = 52 + shift;
    out.sign = value < 0 ? -1 : 1;
    out.limbs.assign(size_t(highest / 32) + 1, 0);
    for (int bit = 0; bit < 53; ++bit)
    {
        if ((mantissa >> bit) & 1)
        {
            const int target = bit + shift;
            out.limbs[size_t(target / 32)] |= uint32_t(1) << (target % 32);
        }
    }
    TrimLimbs(out.limbs);
    return out;
}

// ConvexHull decides which faces a point sees by the sign of ConvexHullFace::Evalue, a 3x3 determinant of point
// differences, and near-degenerate cases (lattice points on or next to a face plane) need that sign exactly. The
// oracle rebuilds every double as an exact integer multiple of a common power of two and expands the determinant in
// integers. Lattice cases have exact differences, which Evalue sums as expansions; mixed-magnitude cases have
// rounded differences, which Evalue passes to its extended-precision fallback.
void FaceOrientationSignIsExact()
{
    std::mt19937_64 rng(20260916);
    const auto uniformInt = [&rng](const int64_t lo, const int64_t hi) {
        return std::uniform_int_distribution<int64_t>(lo, hi)(rng);
    };
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    int exactDifferenceCases = 0;
    int roundedDifferenceCases = 0;
    int zeroCases = 0;

    const auto check = [&](const VHACD::Vect3& p0, const VHACD::Vect3& p1, const VHACD::Vect3& p2, const VHACD::Vect3& q) {
        const std::vector<VHACD::Vect3> points = { p0, p1, p2 };
        VHACD::ConvexHullFace face;
        face.m_index = { 0, 1, 2 };
        const double evalue = face.Evalue(points, q);
        const int evalueSign = evalue > 0 ? 1 : (evalue < 0 ? -1 : 0);

        int base = 0;
        for (const VHACD::Vect3* point : { &p0, &p1, &p2, &q })
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                if ((*point)[axis] != 0)
                {
                    base = std::min(base, ExactExponent((*point)[axis]));
                }
            }
        }
        bool exactDifferences = true;
        ExactInt rows[3][3];
        const VHACD::Vect3* const others[3] = { &p2, &p1, &q };
        for (int row = 0; row < 3; ++row)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                rows[row][axis] = SubtractExact(ExactFromDouble((*others[row])[axis], base), ExactFromDouble(p0[axis], base));
                const ExactInt rounded = SubtractExact(ExactFromDouble((*others[row])[axis] - p0[axis], base), rows[row][axis]);
                exactDifferences = exactDifferences && rounded.sign == 0;
            }
        }
        const ExactInt minorX = SubtractExact(MultiplyExact(rows[1][1], rows[2][2]), MultiplyExact(rows[1][2], rows[2][1]));
        const ExactInt minorY = SubtractExact(MultiplyExact(rows[1][0], rows[2][2]), MultiplyExact(rows[1][2], rows[2][0]));
        const ExactInt minorZ = SubtractExact(MultiplyExact(rows[1][0], rows[2][1]), MultiplyExact(rows[1][1], rows[2][0]));
        const ExactInt det = AddExact(SubtractExact(MultiplyExact(rows[0][0], minorX), MultiplyExact(rows[0][1], minorY)),
                                      MultiplyExact(rows[0][2], minorZ));
        CHECK(evalueSign == det.sign);
        exactDifferenceCases += exactDifferences ? 1 : 0;
        roundedDifferenceCases += exactDifferences ? 0 : 1;
        zeroCases += det.sign == 0 ? 1 : 0;
    };

    // Integer lattices scaled by powers of two: exactly coplanar points, determinants of +-1 against large entries
    // (Cassini's identity for consecutive Fibonacci numbers), and general points.
    for (int i = 0; i < 3000; ++i)
    {
        const double scale = std::ldexp(1.0, -int(uniformInt(0, 40)));
        const int64_t origin[3] = { uniformInt(-(1 << 20), 1 << 20), uniformInt(-(1 << 20), 1 << 20), uniformInt(-(1 << 20), 1 << 20) };
        int64_t u[3], v[3], w[3];
        switch (i % 3)
        {
            case 0:
            {
                const int64_t a = uniformInt(-3, 3);
                const int64_t b = uniformInt(-3, 3);
                for (int axis = 0; axis < 3; ++axis)
                {
                    u[axis] = uniformInt(-1024, 1024);
                    v[axis] = uniformInt(-1024, 1024);
                    w[axis] = a * u[axis] + b * v[axis];
                }
                break;
            }
            case 1:
            {
                int64_t fibonacci[30] = { 0, 1 };
                for (int n = 2; n < 30; ++n)
                {
                    fibonacci[n] = fibonacci[n - 1] + fibonacci[n - 2];
                }
                const int n = int(uniformInt(8, 27));
                const int64_t zu[3] = { fibonacci[n + 1], fibonacci[n], 0 };
                const int64_t zv[3] = { fibonacci[n], fibonacci[n - 1], 0 };
                const int64_t zw[3] = { uniformInt(-1024, 1024), uniformInt(-1024, 1024), uniformInt(0, 1) * 2 - 1 };
                const int rotation = int(uniformInt(0, 2));
                for (int axis = 0; axis < 3; ++axis)
                {
                    u[(axis + rotation) % 3] = zu[axis];
                    v[(axis + rotation) % 3] = zv[axis];
                    w[(axis + rotation) % 3] = zw[axis];
                }
                break;
            }
            default:
                for (int axis = 0; axis < 3; ++axis)
                {
                    u[axis] = uniformInt(-4096, 4096);
                    v[axis] = uniformInt(-4096, 4096);
                    w[axis] = uniformInt(-4096, 4096);
                }
                break;
        }
        const auto point = [&](const int64_t* offset) {
            return VHACD::Vect3(double(origin[0] + (offset ? offset[0] : 0)) * scale,
                                double(origin[1] + (offset ? offset[1] : 0)) * scale,
                                double(origin[2] + (offset ? offset[2] : 0)) * scale);
        };
        check(point(nullptr), point(v), point(u), point(w));
    }

    // A tiny base point among unit-scale points makes the differences round.
    for (int i = 0; i < 2000; ++i)
    {
        const VHACD::Vect3 p0(std::ldexp(unit(rng), -60), std::ldexp(unit(rng), -61), std::ldexp(unit(rng), -62));
        const VHACD::Vect3 p1(unit(rng), unit(rng), unit(rng));
        const VHACD::Vect3 p2(unit(rng), unit(rng), unit(rng));
        const double alpha = unit(rng);
        const double beta = unit(rng);
        const VHACD::Vect3 q = p0 + (p2 - p0) * alpha + (p1 - p0) * beta;
        check(p0, p1, p2, q);
    }

    CHECK(exactDifferenceCases >= 2500);
    CHECK(roundedDifferenceCases >= 1500);
    CHECK(zeroCases >= 900);
}

} // namespace

int main()
{
    IVHACD* const v = VHACD::CreateVHACD();
    ValidMeshCompletes(*v);
    InvalidMeshIsRejectedAndReleasesResults(*v);
    InvalidParametersAreRejected(*v);
    EmptyMeshCompletesWithoutHulls(*v);
    ZeroMinimumEdgeLengthNeverProducesEmptyPieces(*v);
    DepthZeroDoesNotSplit(*v);
    CancelFromUpdateStopsAndReleasesResults(*v);
    ReusedInstanceReproducesResults(*v);
    CenterIsTheSolidCentroid(*v);
    RotatedCubeRemainsOneHull(*v);
    LimitedHullKeepsFarthestPoints();
    FaceOrientationSignIsExact();
    v->Release();

    std::printf("%s (%d failure%s)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
