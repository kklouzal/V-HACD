// Contract tests for IVHACD::Compute: typed results, input validation, cancellation, instance reuse and
// degenerate parameters. Build in a configuration with assertions enabled as well as an optimized one;
// the library's internal invariants are checked by its asserts.
#define ENABLE_VHACD_IMPLEMENTATION 1
#include "VHACD.h"

#include <algorithm>
#include <cmath>
#if defined(_MSC_VER) && !defined(NDEBUG)
#include <crtdbg.h>
#include <cstdlib>
#endif
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

// The test meshes are a few units across, so these are the tolerances of a model of that size.
IVHACD::Parameters DefaultParams()
{
    IVHACD::Parameters p;
    p.m_maxConvexHulls = 8;
    p.m_maxNumVerticesPerCH = 32;
    p.m_relativeTolerance = 0.02;
    p.m_minTolerance = 0.02;
    p.m_maxTolerance = 0.1;
    p.m_probeRadius = 0.05;
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
    p.m_relativeTolerance = std::numeric_limits<double>::quiet_NaN();
    CHECK(Run(v, box, p) == Result::InvalidInput);
    p.m_relativeTolerance = 0;
    CHECK(Run(v, box, p) == Result::InvalidInput);

    p = DefaultParams();
    p.m_minTolerance = -1;
    CHECK(Run(v, box, p) == Result::InvalidInput);

    p = DefaultParams();
    p.m_maxTolerance = p.m_minTolerance / 2;
    CHECK(Run(v, box, p) == Result::InvalidInput);

    p = DefaultParams();
    p.m_probeRadius = std::numeric_limits<double>::infinity();
    CHECK(Run(v, box, p) == Result::InvalidInput);
    p.m_probeRadius = -0.5;
    CHECK(Run(v, box, p) == Result::InvalidInput);

    p = DefaultParams();
    p.m_maxVoxels = 8;
    CHECK(Run(v, box, p) == Result::InvalidInput);
}

// A unit cube rotated off every axis is convex, so it must remain one hull. Its voxelized faces are
// staircases whose corner hull always exceeds the voxels, and the notches of that staircase stay within
// the tolerance, so nothing about them makes the cube split.
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
    IVHACD::Parameters p = DefaultParams();
    p.m_maxConvexHulls = 32;
    p.m_maxNumVerticesPerCH = 44;
    CHECK(Run(v, cube, p) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 1);
}

// The tolerance is what decides the count: a concave model splits while a hull would stand too far off
// it, and a tolerance larger than the model itself leaves one hull.
void ToleranceDecidesHullCount(IVHACD& v)
{
    IVHACD::Parameters tight = DefaultParams();
    tight.m_maxConvexHulls = 64;
    CHECK(Run(v, LShape(), tight) == Result::Completed);
    const uint32_t tightHulls = v.GetNConvexHulls();
    CHECK(tightHulls > 1);
    CHECK(!v.GetReport().m_hullBudgetBound);
    CHECK(v.GetReport().m_tolerance >= tight.m_minTolerance);

    IVHACD::Parameters coarse = tight;
    coarse.m_minTolerance = 10;
    coarse.m_maxTolerance = 10;
    CHECK(Run(v, LShape(), coarse) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 1);
}

// Gaps a probe sphere cannot enter are filled, so a slot narrower than the probe costs no extra hull,
// while the same slot keeps its two sides apart once the probe is small enough to enter it.
void ProbeRadiusFillsNarrowGaps(IVHACD& v)
{
    // A pocket 0.1 wide in the top of a block, closed on every side, so nothing outside the pocket can
    // touch it: what decides whether it stays open is only whether the probe fits inside.
    Mesh slotted = Box(1, 1, 0.2);
    const auto append = [&slotted](const Mesh& part, const double dx, const double dy, const double dz)
    {
        const uint32_t offset = uint32_t(slotted.points.size() / 3);
        for (size_t i = 0; i < part.points.size(); i += 3)
        {
            slotted.points.push_back(part.points[i] + dx);
            slotted.points.push_back(part.points[i + 1] + dy);
            slotted.points.push_back(part.points[i + 2] + dz);
        }
        for (const uint32_t index : part.triangles)
        {
            slotted.triangles.push_back(index + offset);
        }
    };
    append(Box(0.45, 1, 1), 0, 0, 0.2);
    append(Box(0.45, 1, 1), 0.55, 0, 0.2);
    append(Box(0.1, 0.2, 1), 0.45, 0, 0.2);
    append(Box(0.1, 0.2, 1), 0.45, 0.8, 0.2);

    IVHACD::Parameters wide = DefaultParams();
    wide.m_maxConvexHulls = 32;
    wide.m_probeRadius = 0.2;
    CHECK(Run(v, slotted, wide) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 1);

    IVHACD::Parameters narrow = wide;
    narrow.m_probeRadius = 0.01;
    CHECK(Run(v, slotted, narrow) == Result::Completed);
    CHECK(v.GetNConvexHulls() > 1);
}

// Hulls may overlap, which is what lets a plus sign be two crossing bars: a decomposition that partitioned
// its voxels would need three pieces for it. The bars are 1 m wide, so the tolerance has to be well under
// that for the two-bar cover to be the tighter answer; at a coarse tolerance the cover is still a cover,
// but hulls bloated by their own voxels split it differently.
void CrossingBarsCoverEachOther(IVHACD& v)
{
    Mesh plus = Box(3, 1, 0.4);
    const Mesh upright = Box(1, 3, 0.4);
    const uint32_t offset = uint32_t(plus.points.size() / 3);
    for (size_t i = 0; i < upright.points.size(); i += 3)
    {
        plus.points.push_back(upright.points[i] + 1.0);
        plus.points.push_back(upright.points[i + 1]);
        plus.points.push_back(upright.points[i + 2]);
    }
    for (const uint32_t index : upright.triangles)
    {
        plus.triangles.push_back(index + offset);
    }
    for (size_t i = 0; i < offset * 3; i += 3)
    {
        plus.points[i + 1] += 1.0;
    }

    IVHACD::Parameters p = DefaultParams();
    p.m_maxConvexHulls = 32;
    p.m_relativeTolerance = 0.01;
    p.m_minTolerance = 0.01;
    p.m_maxTolerance = 0.03;
    p.m_probeRadius = 0.05;
    CHECK(Run(v, plus, p) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 2);

    // Each hull spans one bar, so between them they overlap where the bars cross.
    for (uint32_t i = 0; i < v.GetNConvexHulls(); ++i)
    {
        IVHACD::ConvexHull hull;
        v.GetConvexHull(i, hull);
        double low[3] = {1e30, 1e30, 1e30};
        double high[3] = {-1e30, -1e30, -1e30};
        for (const VHACD::Vertex& q : hull.m_points)
        {
            const double c[3] = {q.mX, q.mY, q.mZ};
            for (int axis = 0; axis < 3; ++axis)
            {
                low[axis] = std::min(low[axis], c[axis]);
                high[axis] = std::max(high[axis], c[axis]);
            }
        }
        const bool spansX = high[0] - low[0] > 2.5 && high[1] - low[1] < 1.5;
        const bool spansY = high[1] - low[1] > 2.5 && high[0] - low[0] < 1.5;
        CHECK(spansX || spansY);
    }
}

// Budgets bound the work rather than the quality, and say so.
void BudgetsAreReported(IVHACD& v)
{
    IVHACD::Parameters p = DefaultParams();
    p.m_maxConvexHulls = 1;
    CHECK(Run(v, LShape(), p) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 1);
    CHECK(v.GetReport().m_hullBudgetBound);

    IVHACD::Parameters coarse = DefaultParams();
    coarse.m_maxVoxels = 1 << 13;
    CHECK(Run(v, LShape(), coarse) == Result::Completed);
    CHECK(v.GetReport().m_voxelBudgetBound);
    CHECK(v.GetReport().m_tolerance > coarse.m_minTolerance);
    CHECK(v.GetReport().m_voxelCount <= coarse.m_maxVoxels);
}

// Every hull carries a bounding box and an id alongside its points. The merge phase rebuilds both, so they
// are checked against the points they claim to describe.
void HullBoxesAndIdsDescribeTheirPoints(IVHACD& v)
{
    IVHACD::Parameters p = DefaultParams();
    p.m_maxConvexHulls = 8;
    CHECK(Run(v, LShape(), p) == Result::Completed);
    CHECK(v.GetNConvexHulls() > 1);

    std::vector<uint32_t> ids;
    for (uint32_t i = 0; i < v.GetNConvexHulls(); ++i)
    {
        IVHACD::ConvexHull hull;
        CHECK(v.GetConvexHull(i, hull));
        CHECK(!hull.m_points.empty());
        for (const VHACD::Vertex& q : hull.m_points)
        {
            CHECK(q.mX >= hull.mBmin.GetX() && q.mX <= hull.mBmax.GetX());
            CHECK(q.mY >= hull.mBmin.GetY() && q.mY <= hull.mBmax.GetY());
            CHECK(q.mZ >= hull.mBmin.GetZ() && q.mZ <= hull.mBmax.GetZ());
        }
        // The box is the points' box, not merely a box containing them.
        for (int axis = 0; axis < 3; ++axis)
        {
            double low = std::numeric_limits<double>::max();
            double high = -std::numeric_limits<double>::max();
            for (const VHACD::Vertex& q : hull.m_points)
            {
                const double c[3] = {q.mX, q.mY, q.mZ};
                low = std::min(low, c[axis]);
                high = std::max(high, c[axis]);
            }
            CHECK(low == hull.mBmin[axis] && high == hull.mBmax[axis]);
        }
        CHECK(hull.m_center.GetX() >= hull.mBmin.GetX() && hull.m_center.GetX() <= hull.mBmax.GetX());
        ids.push_back(hull.m_meshId);
    }
    std::sort(ids.begin(), ids.end());
    CHECK(std::adjacent_find(ids.begin(), ids.end()) == ids.end());

    // A hull past the last one is not a hull, and leaves its argument alone.
    IVHACD::ConvexHull untouched;
    CHECK(!v.GetConvexHull(v.GetNConvexHulls(), untouched));
    CHECK(untouched.m_points.empty() && untouched.m_triangles.empty());
}

// A Compute that produced nothing must not leave the previous one's report behind, where it would read as
// a description of the call that was refused.
void RejectedComputeLeavesNoReport(IVHACD& v)
{
    CHECK(Run(v, LShape(), DefaultParams()) == Result::Completed);
    CHECK(v.GetNConvexHulls() > 0 && v.GetReport().m_voxelCount > 0);

    IVHACD::Parameters bad = DefaultParams();
    bad.m_maxPieces = 1;
    bad.m_maxConvexHulls = 8;
    CHECK(Run(v, LShape(), bad) == Result::InvalidInput);
    CHECK(v.GetNConvexHulls() == 0);
    CHECK(v.GetReport().m_voxelCount == 0);
    CHECK(v.GetReport().m_tolerance == 0 && v.GetReport().m_voxelSize == 0);
    CHECK(v.GetReport().m_pieceCount == 0 && v.GetReport().m_protectedVoxels == 0);
    CHECK(!v.GetReport().m_hullBudgetBound && !v.GetReport().m_voxelBudgetBound
          && !v.GetReport().m_pieceBudgetBound);
}

// The decomposition rests on one question: does this hull contain a voxel it may not? The scan that
// answers it walks columns and clips a line against the faces, so it is checked here against the
// definition itself, evaluated over every voxel centre and every face.
// The probe-radius closing and the far-voxel test both rest on one exact distance transform, computed
// separably per axis as the lower envelope of parabolas. That is the subtle part of the whole design, so it
// is checked against the definition it implements: for every voxel, the squared distance to the nearest seed
// found by looking at all of them.
void DistanceTransformMatchesNearestSeed()
{
    std::mt19937 rng(20260917);
    const VHACD::Vector3<uint32_t> shapes[] = {
        VHACD::Vector3<uint32_t>(1, 1, 1),  VHACD::Vector3<uint32_t>(9, 4, 6),
        VHACD::Vector3<uint32_t>(1, 12, 1), VHACD::Vector3<uint32_t>(7, 7, 7),
        VHACD::Vector3<uint32_t>(13, 2, 5),
    };
    for (const VHACD::Vector3<uint32_t>& dim : shapes)
    {
        const size_t count = size_t(dim[0]) * dim[1] * dim[2];
        // Densities from "one seed" to "almost all seeds", and one grid with no seed at all.
        for (const int percent : {0, 3, 15, 50, 97})
        {
            std::vector<uint8_t> seeds(count, 0);
            std::uniform_int_distribution<int> roll(1, 100);
            for (size_t i = 0; i < count; ++i)
            {
                seeds[i] = roll(rng) <= percent ? 1 : 0;
            }
            std::vector<size_t> seedIndices;
            for (size_t i = 0; i < count; ++i)
            {
                if (seeds[i])
                {
                    seedIndices.push_back(i);
                }
            }

            std::vector<int32_t> field;
            VHACD::SquaredDistanceTransform(dim, seeds, field);
            CHECK(field.size() == count);

            // The grid is stored the way Volume::GetVoxel indexes it: k + j*dimZ + i*dimY*dimZ.
            const auto coords = [&dim](const size_t index, int32_t& x, int32_t& y, int32_t& z)
            {
                z = int32_t(index % dim[2]);
                y = int32_t((index / dim[2]) % dim[1]);
                x = int32_t(index / (size_t(dim[2]) * dim[1]));
            };
            const int32_t span = int32_t(dim[0]) * int32_t(dim[0]) + int32_t(dim[1]) * int32_t(dim[1])
                                 + int32_t(dim[2]) * int32_t(dim[2]);
            for (size_t i = 0; i < count; ++i)
            {
                int32_t x = 0, y = 0, z = 0;
                coords(i, x, y, z);
                int32_t nearest = std::numeric_limits<int32_t>::max();
                for (const size_t s : seedIndices)
                {
                    int32_t sx = 0, sy = 0, sz = 0;
                    coords(s, sx, sy, sz);
                    const int32_t d = (x - sx) * (x - sx) + (y - sy) * (y - sy) + (z - sz) * (z - sz);
                    nearest = std::min(nearest, d);
                }
                if (seedIndices.empty())
                {
                    // Nothing to be near: the distance only has to exceed anything the grid can span.
                    CHECK(field[i] > span);
                }
                else
                {
                    CHECK(field[i] == nearest);
                }
            }
        }
    }
}

void HullSpaceTestMatchesBruteForce()
{
    // A solid slab with a notch cut out of it, voxelized by hand.
    VHACD::Volume volume;
    volume.m_dim = VHACD::Vector3<uint32_t>(24, 20, 16);
    volume.m_scale = 0.25;
    volume.m_bounds = VHACD::BoundsAABB(VHACD::Vect3(0, 0, 0),
                                        VHACD::Vect3(double(volume.m_dim[0]) * volume.m_scale,
                                                     double(volume.m_dim[1]) * volume.m_scale,
                                                     double(volume.m_dim[2]) * volume.m_scale));
    volume.m_data.assign(size_t(volume.m_dim[0]) * volume.m_dim[1] * volume.m_dim[2],
                         VHACD::VoxelValue::PRIMITIVE_OUTSIDE_SURFACE);
    for (uint32_t i = 4; i < 20; ++i)
    {
        for (uint32_t j = 4; j < 16; ++j)
        {
            for (uint32_t k = 4; k < 12; ++k)
            {
                const bool notch = i >= 9 && i < 15 && j >= 9;
                volume.SetVoxel(i, j, k, notch ? VHACD::VoxelValue::PRIMITIVE_OUTSIDE_SURFACE
                                               : VHACD::VoxelValue::PRIMITIVE_INSIDE_SURFACE);
            }
        }
    }

    // The library polls for cancellation between passes; nothing here ever cancels.
    struct NeverCancels final : VHACD::VHACDCallbacks
    {
        void ProgressUpdate(VHACD::Stages, double, const char*) override {}
        bool PollProgress(VHACD::Stages, double, const char*) override { return false; }
        bool IsCanceled() const override { return false; }
    } callbacks;

    VHACD::SpaceModel space;
    space.Build(volume, 1.5, 3.0, callbacks);
    CHECK(space.GetFarVoxelCount() > 0);

    std::mt19937_64 rng(0x5eed);
    std::uniform_real_distribution<double> coordinate(-1.0, 7.0);
    int agreements = 0;
    for (int trial = 0; trial < 200; ++trial)
    {
        // A random box, as a hull of its eight corners.
        std::uniform_real_distribution<double> side(0.2, 3.0);
        const VHACD::Vect3 low(coordinate(rng), coordinate(rng), coordinate(rng));
        const VHACD::Vect3 high(low.GetX() + side(rng), low.GetY() + side(rng), low.GetZ() + side(rng));
        std::vector<VHACD::Vertex> points;
        for (int corner = 0; corner < 8; ++corner)
        {
            points.emplace_back((corner & 1) ? high.GetX() : low.GetX(),
                                (corner & 2) ? high.GetY() : low.GetY(),
                                (corner & 4) ? high.GetZ() : low.GetZ());
        }
        const std::vector<VHACD::Triangle> triangles = {
            {0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
            {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};

        // The definition: any far voxel whose centre lies in the box.
        bool expected = false;
        for (uint32_t i = 0; i < volume.m_dim[0] && !expected; ++i)
        {
            for (uint32_t j = 0; j < volume.m_dim[1] && !expected; ++j)
            {
                for (uint32_t k = 0; k < volume.m_dim[2] && !expected; ++k)
                {
                    // Voxel (i, j, k) is centred on the minimum of the bounds plus i, j, k voxels.
                    const VHACD::Vect3 centre(volume.m_bounds.GetMin().GetX() + double(i) * volume.m_scale,
                                              volume.m_bounds.GetMin().GetY() + double(j) * volume.m_scale,
                                              volume.m_bounds.GetMin().GetZ() + double(k) * volume.m_scale);
                    if (space.IsFarVoxel(int32_t(i), int32_t(j), int32_t(k))
                        && centre.GetX() >= low.GetX() && centre.GetX() <= high.GetX()
                        && centre.GetY() >= low.GetY() && centre.GetY() <= high.GetY()
                        && centre.GetZ() >= low.GetZ() && centre.GetZ() <= high.GetZ())
                    {
                        expected = true;
                    }
                }
            }
        }
        CHECK(space.HullReachesTooFar(points, triangles) == expected);
        agreements += expected ? 1 : 0;
    }
    // The boxes must actually straddle the notch, or the comparison proves nothing.
    CHECK(agreements > 10);
}

// The hull builder refuses a point cloud whose thinnest extent is a small enough fraction of its longest,
// whatever the voxels say, and a rod 500 m long and 1 mm thick is such a cloud. Its collision must not
// disappear for it.
void ThinRodKeepsItsCollision(IVHACD& v)
{
    for (const double thickness : {0.001, 0.1})
    {
        Mesh rod = Box(500.0, thickness, thickness);
        IVHACD::Parameters p = DefaultParams();
        p.m_maxConvexHulls = 32;
        p.m_minTolerance = 1.0;
        p.m_maxTolerance = 5.0;
        p.m_probeRadius = 0.5;
        p.m_maxVoxels = 1 << 16;
        CHECK(Run(v, rod, p) == Result::Completed);
        CHECK(v.GetNConvexHulls() >= 1);

        // The rod's midpoint has to lie inside one of the hulls.
        const double centre[3] = {250.0, 0.5 * thickness, 0.5 * thickness};
        bool enclosed = false;
        for (uint32_t i = 0; i < v.GetNConvexHulls(); ++i)
        {
            IVHACD::ConvexHull hull;
            v.GetConvexHull(i, hull);
            CHECK(hull.m_points.size() >= 4 && !hull.m_triangles.empty());
            double worst = -std::numeric_limits<double>::max();
            for (const VHACD::Triangle& t : hull.m_triangles)
            {
                const VHACD::Vect3 a(hull.m_points[t.mI0]);
                const VHACD::Vect3 b(hull.m_points[t.mI1]);
                const VHACD::Vect3 c(hull.m_points[t.mI2]);
                VHACD::Vect3 normal = (b - a).Cross(c - a);
                const double length = normal.GetNorm();
                if (length > 1.0e-12)
                {
                    normal = normal / length;
                    worst = std::max(worst, normal.Dot(VHACD::Vect3(centre[0], centre[1], centre[2]) - a));
                }
            }
            enclosed = enclosed || worst <= 1.0e-9;
        }
        CHECK(enclosed);
    }
}

// A mesh whose vertices all sit at one point describes no volume and no grid; it must come back empty
// rather than with a report full of arithmetic that has no meaning.
void MeshWithoutExtentIsHandled(IVHACD& v)
{
    Mesh point = Box(0.0, 0.0, 0.0);
    CHECK(Run(v, point, DefaultParams()) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 0);
    CHECK(v.GetReport().m_voxelCount == 0);
    CHECK(std::isfinite(v.GetReport().m_tolerance) && std::isfinite(v.GetReport().m_voxelSize));
}

// Voxel coordinates are packed into ten bits each, so no grid may exceed 1021 voxels on an axis whatever
// is asked of it, and no setting may leave a model without collision.
void ExtremeSettingsStillProduceAGrid(IVHACD& v)
{
    struct Case
    {
        double m_extent;
        double m_tolerance;
        double m_probe;
        uint32_t m_maxVoxels;
    };
    const Case cases[] = {
        {500.0, 1.0e-6, 1.0e-6, 4096},     // finer than any budget can hold
        {500.0, 1.0e-6, 1.0e-6, 1 << 16},  // and with a budget that cannot help either
        {1.0, 0.01, 1000.0, 1 << 16},      // a probe a thousand times the model
        {0.001, 0.01, 0.05, 1 << 16},      // a model far below its own tolerance
    };
    for (const Case& c : cases)
    {
        IVHACD::Parameters p = DefaultParams();
        p.m_maxConvexHulls = 32;
        p.m_relativeTolerance = 0.01;
        p.m_minTolerance = c.m_tolerance;
        p.m_maxTolerance = std::max(c.m_tolerance, 0.03);
        p.m_probeRadius = c.m_probe;
        p.m_maxVoxels = c.m_maxVoxels;
        CHECK(Run(v, Box(c.m_extent, c.m_extent, c.m_extent), p) == Result::Completed);
        CHECK(v.GetNConvexHulls() >= 1);
        const IVHACD::Report& report = v.GetReport();
        CHECK(report.m_voxelCount > 0 && report.m_voxelCount <= std::max(c.m_maxVoxels, 4096u));
        CHECK(report.m_voxelSize > 0 && std::isfinite(report.m_tolerance));
    }
}

void EmptyMeshCompletesWithoutHulls(IVHACD& v)
{
    CHECK(v.Compute(static_cast<const double*>(nullptr), 0, nullptr, 0, DefaultParams()) == Result::Completed);
    CHECK(v.GetNConvexHulls() == 0);
}

void EveryPieceHasVolume(IVHACD& v)
{
    IVHACD::Parameters p = DefaultParams();
    p.m_maxConvexHulls = 64;
    p.m_minTolerance = 0.01;
    CHECK(Run(v, LShape(), p) == Result::Completed);
    CHECK(v.GetNConvexHulls() > 0);
    for (uint32_t i = 0; i < v.GetNConvexHulls(); ++i)
    {
        IVHACD::ConvexHull ch;
        v.GetConvexHull(i, ch);
        CHECK(std::isfinite(ch.m_volume) && ch.m_volume > 0);
    }
}

void CancelFromUpdateStopsAndReleasesResults(IVHACD& v)
{
    // Start from a completed run, so anything left behind is the previous run's and not an empty slate.
    CHECK(Run(v, LShape(), DefaultParams()) == Result::Completed);
    CHECK(v.GetNConvexHulls() > 0 && v.GetReport().m_voxelCount > 0);

    IVHACD::Parameters p = DefaultParams();
    CancelOnFirstUpdate cancel;
    cancel.target = &v;
    p.m_callback = &cancel;
    CHECK(Run(v, LShape(), p) == Result::Canceled);
    CHECK(v.GetNConvexHulls() == 0);
    CHECK(v.GetReport().m_voxelCount == 0 && v.GetReport().m_tolerance == 0);
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
    // An assertion must fail the run, not stop it: a debug build pops a modal dialog by default, which in
    // an unattended suite looks like a test that never finishes.
#if defined(_MSC_VER) && !defined(NDEBUG)
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif

    IVHACD* const v = VHACD::CreateVHACD();
    ValidMeshCompletes(*v);
    InvalidMeshIsRejectedAndReleasesResults(*v);
    InvalidParametersAreRejected(*v);
    EmptyMeshCompletesWithoutHulls(*v);
    EveryPieceHasVolume(*v);
    ThinRodKeepsItsCollision(*v);
    MeshWithoutExtentIsHandled(*v);
    ExtremeSettingsStillProduceAGrid(*v);
    CancelFromUpdateStopsAndReleasesResults(*v);
    ReusedInstanceReproducesResults(*v);
    CenterIsTheSolidCentroid(*v);
    RotatedCubeRemainsOneHull(*v);
    ToleranceDecidesHullCount(*v);
    ProbeRadiusFillsNarrowGaps(*v);
    CrossingBarsCoverEachOther(*v);
    BudgetsAreReported(*v);
    HullBoxesAndIdsDescribeTheirPoints(*v);
    RejectedComputeLeavesNoReport(*v);
    LimitedHullKeepsFarthestPoints();
    FaceOrientationSignIsExact();
    DistanceTransformMatchesNearestSeed();
    HullSpaceTestMatchesBruteForce();
    v->Release();

    std::printf("%s (%d failure%s)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
