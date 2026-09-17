# V-HACD

[![License: BSD 3-Clause](https://img.shields.io/badge/license-BSD%203--Clause-blue.svg)](LICENSE)
![C++17](https://img.shields.io/badge/C%2B%2B-17-informational.svg)
![Header-only](https://img.shields.io/badge/header--only-yes-success.svg)

**Voxelized Hierarchical Approximate Convex Decomposition**: turn any triangle mesh into convex hulls for physics
collision. One header, C++17, no dependencies.

This is a maintained fork of [kmammou/v-hacd](https://github.com/kmammou/v-hacd) 4.1, which is archived. It keeps
the algorithm but not its central assumption: you no longer ask for a number of hulls. You ask for an accuracy,
and the hull count follows from the shape. On a corpus of 611 game models that is **half as many hulls** as
upstream, **less than a quarter as much uncovered surface**, and **6x faster**.

![Approximate convex decomposition of a camel](doc/acd.png)

## At a glance

Upstream 4.1 at its own settings (32 hulls, resolution 100000) and this fork at its defaults (1% of each model's
diagonal, between 1 and 3 cm, with a 5 cm probe), on the same 611 inputs, one thread. See
[Benchmark details](#benchmark-details).

|                                                        | Upstream 4.1 | This fork    |
| ------------------------------------------------------ | -----------: | -----------: |
| Hulls per model                                        | 30.1         | **14.5**     |
| Hull faces per model                                   | 789          | **307**      |
| Models that came out as one hull                       | 10           | **81**       |
| Models that hit the 32-hull cap                        | 550          | **116**      |
| Source surface more than 1 cm outside every hull       | 3.37%        | **0.77%**    |
| Source surface more than 5 cm outside every hull       | 2.24%        | **0.36%**    |
| Hull surface standing off the source, 95th percentile  | 2.99 m       | **1.69 m**   |
| Models decomposed into no hulls at all                 | 3            | **0**        |
| Total decomposition time                               | 203 s        | **34 s**     |
| Median model                                           | 195 ms       | **42 ms**    |
| 95th percentile model                                  | 815 ms       | **163 ms**   |
| Slowest model                                          | 4.26 s       | **250 ms**   |

Every model's uncovered surface stays inside the accuracy it was asked for, at the 99th percentile, on all 611.
The corpus is mostly large environment geometry, which is why those distances are in metres; on 61 props (glTF
sample assets and game props under 10 m) the same comparison against the fork's own previous, count-driven
revision is 15.3 hulls per model instead of 26.4, with hull surfaces standing 92 mm off the source at the 95th
percentile instead of 152 mm.

## How it decides

Three settings describe what the collision has to be, and nothing describes how to get there.

- **A tolerance.** How far a hull may stand off the surface: `clamp(m_relativeTolerance * diagonal,
  m_minTolerance, m_maxTolerance)`. It also sets the voxel size, so a model is resolved to the accuracy asked of
  it rather than to a fixed grid, and a large model is no longer coarse for being large.
- **A probe radius.** Gaps, slots and pockets that a sphere of `m_probeRadius` cannot enter from outside are
  filled, along with sealed cavities: nothing that size can occupy them, so hulls may cover them, and covering
  them costs far fewer hulls. Slatted crates, handles, wheel wells and bottle necks stop being concavities.
- **Budgets.** `m_maxConvexHulls`, `m_maxVoxels` and `m_maxPieces` bound the work rather than the quality.
  `IVHACD::GetReport` says which of them, if any, decided the result, and what tolerance was really used.

A piece splits while its hull would cover space no hull may cover, and two hulls merge while the merged hull
would not. What is left when no pair may merge is the hull count.

## What changed

### Hulls follow the shape

Upstream splits a piece until its hull is within a percentage of its voxel volume, then merges pieces back until
a count is met. Volume is the wrong measure for collision: a deep narrow hole has little volume and blocks
everything, while a wide shallow dish has plenty and blocks nothing. A count is the wrong control: on this
corpus, models under 10 m needed a median of 2 hulls to reach the accuracy their grid could offer and were given
21.8, while 14 of 159 needed more than the 32 they were allowed.

A hull the builder refuses, which it does for a cloud too thin for its length whatever its voxels say, takes the
box around its voxels rather than disappearing: a rod 500 m long and 1 mm thick keeps its collision.

The fork measures distance instead, on the voxel grid it already builds:

1. The solid is closed by the probe: three exact integer distance transforms (Meijster, Roerdink and Hesselink)
   give the space the probe can reach from outside, the solid that closing leaves behind, and the distance from
   it. Every voxel farther than the tolerance from that closed solid is one no hull may cover.
2. A piece whose hull covers one of those voxels splits. Nothing else makes it split, so a rotated cube, a
   tilted quad and a single triangle are one hull each; upstream produces 32, 32 and 17.
3. Pairs of hulls merge, cheapest added volume first, while the merged hull covers none of them.

### Faster than upstream

The speed came first, from work that left every output byte for byte identical, and holds up under the new
decisions: 203 s to 34 s over the corpus, and no model slower than 250 ms.

| Change | Speed-up |
| --- | ---: |
| Build each piece's hull straight from its voxel corners. Upstream built a triangle mesh and an AABB tree for every piece, which only the experimental `m_findBestPlane` search ever read. | 3.55x |
| Keep hull faces in vectors instead of linked lists, and stop allocating a 147 KB node pool for every hull. | 1.24x |
| Collect voxel corners with a bitmap, already sorted, instead of a hash map. | 1.19x |
| Sum exact coordinate differences as floating-point expansions before falling back to 256-bit arithmetic. | 1.14x |
| Skip extended precision for determinants with a zero row or column. | 1.11x |
| Reuse one workspace for every hull build. | 1.08x |
| Reserve face storage, drop a repeated visibility test, and compute tree statistics per axis. | 1.08x |
| Merge pre-sorted point lists and compute merge costs without building hull objects. | 1.04x |
| Skip voxels already on the surface while voxelizing. | 1.03x |
| Compute tree statistics with SSE2. | 1.02x |

Merging prices only the pairs that can merge. Hulls farther apart than a filled gap, hulls whose bounds hold
nothing that may not be covered, and hulls with a forbidden voxel on the line between them are all settled
without building anything; without those three tests a passenger coach takes 17.4 s instead of 1.2 s.

### Better hulls

- **Small parts of large models survive.** Upstream welded every vertex within 0.1% of the model's largest
  dimension and dropped the triangles that collapsed, which deleted real geometry: 18 cm lamps spread over a
  270 m airfield and narrow slivers of 600 m terrain tiles produced no hulls at all. The fork welds only vertices
  at exactly the same position.
- **The voxel staircase is not concavity.** A hull of voxel corners always exceeds the voxels of a surface that is
  not axis-aligned. Upstream counted that as concavity and split every oblique solid to its recursion limit; the
  probe's closing swallows those notches, so nothing about them makes a piece split.
- **Vertex caps cost less.** When a hull has more vertices than `m_maxNumVerticesPerCH`, the fork adds the
  farthest remaining point first instead of taking faces in creation order.

### Correct and predictable

- `Compute` returns `ComputeResult` (`Completed`, `Canceled` or `InvalidInput`) and validates its input first.
  Non-finite coordinates, out-of-range indices and out-of-range parameters are rejected, with the reason sent to
  your logger, instead of reading out of bounds.
- Merge order no longer depends on `std::unordered_map` iteration order. Exactly equal merge costs, which are common
  between voxel-aligned pieces, break ties by hull id, so a model decomposes the same way with any standard library.
  Repeated runs produce identical output on all 611 corpus models.
- The space model is integer work throughout: squared distances, and lower envelopes compared by
  cross-multiplication, so the field is exact and reproduces bit for bit.
- Latent undefined behavior is fixed: out-of-range pointer arithmetic in the flood fill, a tree of unbounded depth
  walked with a fixed-size stack, a data race on a static counter shared by all instances, and a quicksort whose
  stack was guarded only by an assert. A later audit found more of its own: a mesh with no extent converting NaN
  to an unsigned integer, a grid able to exceed the ten bits a voxel coordinate has, and a distance transform
  writing to an empty buffer.
- The whole corpus decomposes identically whether assertions and the standard library's hardening are on or off,
  and no assertion fires on any of its 611 models.
- `ConvexHull::m_center` is the hull's center of mass as a solid.
- A contract test suite (`test/ContractTests.cpp`) covers typed results, validation, cancellation, instance reuse,
  centroids, hull reduction, what the tolerance, probe radius and budgets promise, a rod too thin for the hull
  builder keeping its collision, a mesh with no extent, settings no grid can satisfy, the orientation test against
  exact integer arithmetic, and the space test against its own definition over every voxel of a grid.

### Built for engines

- `Compute` runs on the calling thread and owns no threads, so it fits into a job system without oversubscribing
  the CPU. Instances share no mutable state; run one per worker.
- `Cancel()` works from any thread, including from your progress callback, and takes effect promptly: `Compute`
  reports progress after at most 10 ms of work plus its longest indivisible step. Over the benchmark corpus the
  longest gap between reports is 14 ms for the median model and 17 ms at the 95th percentile; the worst, 129 ms,
  is the 267,000-triangle warehouse, whose AABB tree is built in one step.

## Quick start

Add `include/VHACD.h` to your project and define `ENABLE_VHACD_IMPLEMENTATION` in exactly one source file before
including it.

```cpp
#define ENABLE_VHACD_IMPLEMENTATION 1
#include "VHACD.h"

#include <cstdint>
#include <cstdio>
#include <vector>

int main()
{
    // A unit cube: 8 corners and 12 triangles.
    const std::vector<float> positions = {
        0, 0, 0,  1, 0, 0,  0, 1, 0,  1, 1, 0,
        0, 0, 1,  1, 0, 1,  0, 1, 1,  1, 1, 1,
    };
    const std::vector<uint32_t> triangles = {
        0, 2, 1,  1, 2, 3,  4, 5, 6,  5, 7, 6,  0, 1, 4,  1, 5, 4,
        2, 6, 3,  3, 6, 7,  0, 4, 2,  2, 4, 6,  1, 3, 5,  3, 7, 5,
    };

    // Collision may stand off the surface by 1% of the cube's diagonal, and openings a 5 cm sphere
    // cannot enter may be filled. How many hulls that takes is the library's business.
    VHACD::IVHACD::Parameters params;
    params.m_relativeTolerance = 0.01;
    params.m_minTolerance = 0.01;
    params.m_maxTolerance = 0.03;
    params.m_probeRadius = 0.05;
    params.m_maxConvexHulls = 32;
    params.m_maxNumVerticesPerCH = 44;

    VHACD::IVHACD* const vhacd = VHACD::CreateVHACD();
    const VHACD::IVHACD::ComputeResult result = vhacd->Compute(positions.data(),
                                                               uint32_t(positions.size() / 3),
                                                               triangles.data(),
                                                               uint32_t(triangles.size() / 3),
                                                               params);
    if (result == VHACD::IVHACD::ComputeResult::Completed)
    {
        for (uint32_t i = 0; i < vhacd->GetNConvexHulls(); ++i)
        {
            VHACD::IVHACD::ConvexHull hull;
            vhacd->GetConvexHull(i, hull);
            std::printf("hull %u: %zu vertices, volume %g\n", i, hull.m_points.size(), hull.m_volume);
        }
    }
    vhacd->Release();
    return 0;
}
```

`Compute` also accepts `double` positions. Hulls come back as double-precision vertices and triangles in the
input's coordinate space, with their volume and center of mass. `GetReport()` says what the decomposition used:

```cpp
const VHACD::IVHACD::Report& report = vhacd->GetReport();
std::printf("tolerance %.1f mm, voxel %.1f mm, %u pieces%s\n",
            report.m_tolerance * 1000.0,
            report.m_voxelSize * 1000.0,
            report.m_pieceCount,
            report.m_voxelBudgetBound ? " (voxel budget)" : "");
```

### Cancellation

```cpp
#include <atomic>

class CancelOnRequest final : public VHACD::IVHACD::IUserCallback
{
public:
    CancelOnRequest(VHACD::IVHACD& vhacd, const std::atomic<bool>& requested)
        : m_vhacd(vhacd), m_requested(requested) {}

    void Update(double, double, const char*, const char*) override
    {
        if (m_requested.load(std::memory_order_relaxed))
        {
            m_vhacd.Cancel(); // Compute returns ComputeResult::Canceled
        }
    }

private:
    VHACD::IVHACD& m_vhacd;
    const std::atomic<bool>& m_requested;
};
```

Point `params.m_callback` at an instance. You can also call `Cancel()` directly from another thread while
`Compute` runs; a request made before `Compute` starts is discarded.

## Parameters

Lengths are in the units of the input, so a mesh in metres takes metres.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `m_relativeTolerance` | 0.01 | How far a hull may stand off the surface, as a fraction of the model's bounding-box diagonal. |
| `m_minTolerance` | 0.01 | Floor on that tolerance. |
| `m_maxTolerance` | 0.03 | Ceiling on it, so large models do not demand grids they cannot afford. |
| `m_probeRadius` | 0.05 | Gaps, openings and pockets a sphere of this radius cannot enter from outside are filled, as are sealed cavities. 0 keeps every reachable pocket. |
| `m_maxVoxels` | 524288 | Voxel budget. Where it binds, the voxel size and the tolerance are coarsened together and the report says so. |
| `m_maxConvexHulls` | 64 | Hull budget. Hulls merge past the tolerance only to meet it, which the report records. |
| `m_maxPieces` | 512 | Piece budget: splitting stops here even where a piece still reaches too far. |
| `m_maxNumVerticesPerCH` | 32 | Maximum vertices per hull, at least 4. Larger hulls keep their farthest points first. |
| `m_shrinkWrap` | true | Moves hull vertices within one voxel of the source mesh onto it. |
| `m_fillMode` | `FLOOD_FILL` | How the interior is found: `FLOOD_FILL` for closed meshes, `RAYCAST_FILL` for meshes with holes, `SURFACE_ONLY` for hollow results. |
| `m_callback` | null | Receives progress on the thread running `Compute`. |
| `m_logger` | null | Receives warnings and the reason for `InvalidInput`. |

The tolerance is the dial worth turning. A finer one costs voxels, and where the voxel budget cannot follow, the
report's `m_tolerance` tells you what you actually got. The probe radius is the second: it is the size of the
smallest thing that has to fit into an object's openings, and raising it is the cheapest way to fewer hulls.

## Requirements and building

- A C++17 compiler. The library, tests and examples are built with MSVC (Visual Studio 2026) and Clang on Windows.
- An x86-64 CPU. The implementation uses SSE2 and stops with an `#error` on targets without it.
- Nothing else: there is no library to build, because the header contains the implementation.

The command line tool and the contract tests build with CMake:

```sh
cmake -S app -B build/app
cmake --build build/app --config Release

cmake -S test -B build/test
cmake --build build/test --config Release
ctest --test-dir build/test -C Release
```

## TestVHACD

`TestVHACD` decomposes a Wavefront OBJ file and writes the hulls to `decomp.obj` and `decomp.stl` in the working
directory. `app/meshes` has sample meshes to try.

```sh
TestVHACD app/meshes/bunny.obj -e 0.01 -t 0.05 -h 32 -v 44 -o obj
```

| Option | Meaning (default) |
| --- | --- |
| `-e <fraction>` | Tolerance as a fraction of the model's diagonal (0.01) |
| `-n <length>` | Smallest tolerance, in model units (0.01) |
| `-x <length>` | Largest tolerance, in model units (0.03) |
| `-t <radius>` | Probe radius: gaps this sphere cannot enter are filled (0.05) |
| `-h <n>` | Hull budget (64) |
| `-r <n>` | Voxel budget (524288) |
| `-d <n>` | Piece budget (512) |
| `-v <n>` | Maximum vertices per hull (32) |
| `-s true/false` | Shrink wrap (true) |
| `-f flood/raycast/surface` | Fill mode (flood) |
| `-o obj/stl/usda` | Also write one OBJ or STL file per hull, or a single `decomp.usda` |
| `-g true/false` | Logging (true) |

## Migrating from upstream 4.x

- `Compute` returns `IVHACD::ComputeResult`. Compare with `ComputeResult::Completed` where you tested the `bool`.
- Asynchronous computation is gone: `CreateVHACD_ASYNC`, `IsReady`, `IUserCallback::NotifyVHACDComplete`,
  `IUserTaskRunner`, `m_taskRunner` and `m_asyncACD`. Call `Compute` from your own worker thread.
- Also removed: `m_findBestPlane` (experimental and off by default), `ComputeCenterOfMass` (it always returned
  false) and `findNearestConvexHull`.
- The hull count is an outcome, not an input. `m_resolution`, `m_minimumVolumePercentErrorAllowed`,
  `m_maxRecursionDepth` and `m_minEdgeLength` are gone; `m_relativeTolerance`, `m_minTolerance`, `m_maxTolerance`,
  `m_probeRadius`, `m_maxVoxels` and `m_maxPieces` replace them. `m_maxConvexHulls` stays, as a budget.
- Tolerances are lengths in the units of your mesh. A mesh normalized to a unit cube wants a tolerance around
  0.01; a mesh in metres wants metres.
- `IVHACD::GetReport()` is new, and worth logging: it reports the tolerance and voxel size used and which budget,
  if any, decided the result.
- `ConvexHull::m_center` is the center of mass of the hull as a solid. Upstream returned the area centroid of its
  surface.
- Only exactly coincident vertices are welded.
- `TestVHACD` lost `-a` (asynchronous), `-p` (best plane) and `-l`, and its `-e`, `-r`, `-d` and `-t` now mean
  tolerance, voxel budget, piece budget and probe radius.
- The library needs C++17 and SSE2.

## Limitations

- A model far larger than its tolerance hits the voxel budget, and then the grid decides the accuracy rather than
  the tolerance. The report says when that happened; raising `m_maxVoxels` costs cook time roughly in proportion.
- Detail smaller than a voxel is approximated at voxel scale, and a feature far below the tolerance is covered
  rather than resolved.
- A probe pressed against the mouth of a narrow channel reaches into it, so a channel open at its ends keeps a
  thin reachable region near each opening even when the probe cannot travel down it.
- The interior still comes from a flood fill, so a mesh with a hole large enough to leak is treated as the shell
  it looks like. Generalized winding numbers would settle that, and would replace the fill modes.
- Merging is greedy, so a decomposition is not the smallest set of hulls that meets the tolerance, only a small
  one reached cheaply.

## Benchmark details

- **Corpus**: 611 inputs. 606 are glTF models from a game project: buildings and city blocks, terrain tiles up to
  several hundred meters across, props and vehicles. 5 are synthetic: a triangle, a quad, a tilted quad, a unit
  cube and a rotated unit cube.
- **Settings**: upstream at its own, 32 hulls and resolution 100000 with recursion depth 8, 1% volume error and
  minimum edge length 2; the fork at its defaults, 1% of the diagonal clamped to 1 to 3 cm with a 5 cm probe and
  half a million voxels. Both with a 32-hull budget, 44 vertices per hull, shrink wrap on, flood fill, and
  upstream's `m_asyncACD` off, so both use one thread.
- **Machine**: AMD Ryzen 7 9800X3D, Windows 11, MSVC 14.51 toolset (Visual Studio 2026), x64, `/O2 /GL /fp:precise`.
- **Timing**: wall time of `Compute` for each model, one run each; repeated runs produced identical output on
  every model.
- **Coverage**: 20,000 area-weighted samples on each model's surface. A sample counts as uncovered if it lies more
  than 1 cm or 5 cm outside the nearest hull, measured to that hull's face planes. The table shows the mean over
  all models; a model with no hulls counts as fully uncovered. "Inside the accuracy it was asked for" compares
  each model's 99th-percentile uncovered distance against the tolerance its own report records.
- **Standing off**: 20,000 area-weighted samples on the hull surfaces, excluding samples buried inside another
  hull, measured to the nearest source triangle. The table shows the mean of each model's 95th percentile.

## How it works

1. **Voxelize.** The mesh is normalized and voxelized on a grid whose voxel is a fraction of the tolerance, and
   the interior is filled by flood fill, by ray casts, or not at all.
2. **Close.** Distance transforms give the space the probe sphere reaches from outside, and what it cannot reach
   is added to the solid. Every voxel farther than the tolerance from that closed solid is marked as one no hull
   may cover.
3. **Split.** Each piece's convex hull is built from the corners of its surface voxels. A piece whose hull covers
   a marked voxel splits at the middle of its longest axis.
4. **Merge.** Pairs of hulls merge, the pair adding the least volume first, while the merged hull covers no marked
   voxel. What remains when no pair may merge is the result; `m_maxConvexHulls` bounds it.
5. **Finish.** Hull vertices near the source mesh move onto it, and each hull is reduced to at most
   `m_maxNumVerticesPerCH` vertices.

A single convex hull fills every concavity, and an exact convex decomposition produces far more parts than a
simulation can afford. An approximate decomposition sits in between:

![A convex hull compared with an approximate convex decomposition](doc/chvsacd.png)

![An approximate convex decomposition compared with an exact one](doc/ecdvsacd.png)

## Credits

- **Khaled Mamou** created HACD and V-HACD.
- **John W. Ratcliff** wrote the original ACD and the header-only V-HACD 4 rewrite.
- **Julio Jerez**, author of the Newton physics engine, wrote the convex hull builder.
- The exact orientation test uses floating-point expansion arithmetic from **Jonathan Richard Shewchuk**'s
  *Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates*.
- The distance fields follow **Meijster, Roerdink and Hesselink**, *A general algorithm for computing distance
  transforms in linear time* (2000), which keeps them exact and reproducible in integers.
- Measuring concavity as a distance rather than a volume follows **Wei et al.**, *Approximate Convex Decomposition
  for 3D Meshes with Collision-Aware Concavity and Tree Search* (SIGGRAPH 2022), and leaving space that nothing
  can reach to the hulls follows **James Andrews**, *Navigation-Driven Approximate Convex Decomposition*
  (SIGGRAPH 2024).

## License

BSD 3-Clause. See [LICENSE](LICENSE).
