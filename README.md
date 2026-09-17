# V-HACD

[![License: BSD 3-Clause](https://img.shields.io/badge/license-BSD%203--Clause-blue.svg)](LICENSE)
![C++17](https://img.shields.io/badge/C%2B%2B-17-informational.svg)
![Header-only](https://img.shields.io/badge/header--only-yes-success.svg)

**Voxelized Hierarchical Approximate Convex Decomposition**: turn any triangle mesh into a small set of convex
hulls for physics collision. One header, C++17, no dependencies.

This is a maintained fork of [kmammou/v-hacd](https://github.com/kmammou/v-hacd) 4.1, which is archived. It keeps
the algorithm and the shape of the API. On a corpus of 611 game models it is **12.6x faster**, leaves **about half
as much of the source surface uncovered**, decomposes the three models upstream returned nothing for, and fits
into an engine's job system.

![Approximate convex decomposition of a camel](doc/acd.png)

## At a glance

Upstream 4.1 and this fork on the same 611 inputs with the same settings: 32 hulls, 32 vertices per hull,
resolution 100000, one thread. See [Benchmark details](#benchmark-details).

|                                                        | Upstream 4.1           | This fork           |
| ------------------------------------------------------ | ---------------------: | ------------------: |
| Total decomposition time                               | 191.3 s                | **15.2 s**          |
| Median model                                           | 194 ms                 | **11.3 ms**         |
| 95th percentile model                                  | 805 ms                 | **81 ms**           |
| Slowest model                                          | 4.13 s                 | **188 ms**          |
| Rotated unit cube                                      | 1.87 s, 32 hulls       | **11 ms, 1 hull**   |
| Models decomposed into no hulls at all                 | 3                      | **0**               |
| Source surface more than 1 cm outside every hull       | 3.4%                   | **1.6%**            |
| Source surface more than 5 cm outside every hull       | 2.3%                   | **0.9%**            |
| Mean hulls per model                                   | 30.1                   | **28.1**            |
| Models using all 32 hulls                              | 550                    | **491**             |
| Mean hull faces per model                              | 761                    | **728**             |

The median model decomposes 11.9x faster. The fork is slower on only three models: the three that upstream
reduced to nothing in under 0.1 ms, which the fork now decomposes properly in 0.2 to 2 ms. Keeping tilted solids
whole leaves a little more hull surface away from the source: the median distance from a hull's surface to the
source mesh is about 4% larger on average, and the 95th percentile about 1%. With `m_tiltedSurfaceAllowance`
set to 0, both are within about 1% of upstream's.

## What changed

### Faster

Each change was measured on the same corpus when it landed, and together they account for nearly all of the
speed-up above. Every change in the table except the tilted-surface allowance leaves all 611 outputs byte for
byte identical.

| Change | Speed-up |
| --- | ---: |
| Build each piece's hull straight from its voxel corners. Upstream built a triangle mesh and an AABB tree for every piece, which only the experimental `m_findBestPlane` search ever read. | 3.55x |
| Stop splitting convex solids over the voxel staircase (`m_tiltedSurfaceAllowance`), so there are fewer pieces to build and merge. | 1.42x |
| Keep hull faces in vectors instead of linked lists, and stop allocating a 147 KB node pool for every hull. | 1.24x |
| Collect voxel corners with a bitmap, already sorted, instead of a hash map. | 1.19x |
| Sum exact coordinate differences as floating-point expansions before falling back to 256-bit arithmetic. | 1.14x |
| Skip extended precision for determinants with a zero row or column. | 1.11x |
| Reuse one workspace for every hull build. | 1.08x |
| Reserve face storage, drop a repeated visibility test, and compute tree statistics per axis. | 1.08x |
| Merge pre-sorted point lists and compute merge costs without building hull objects. | 1.04x |
| Skip voxels already on the surface while voxelizing. | 1.03x |
| Compute tree statistics with SSE2. | 1.02x |

With `m_tiltedSurfaceAllowance` set to 0, which keeps upstream's volume error test, the fork still decomposes the
corpus 9.4x faster (20.3 s).

### Better hulls

- **Convex solids stay whole.** A hull built from voxel corners always exceeds the voxels of a surface that is not
  axis-aligned by a staircase of about half a voxel per surface voxel, even on a flat face. Upstream counted that
  gap as concavity, so every oblique solid split to the recursion limit and merged back up to the hull cap: a
  rotated cube came out as 32 hulls. The fork discounts the gap on tilted surfaces, so a rotated cube is one hull
  and 87 models use fewer hulls.
- **Small parts of large models survive.** Upstream welded every vertex within 0.1% of the model's largest
  dimension and dropped the triangles that collapsed. On large models that deleted real geometry: 18 cm lamps
  spread over a 270 m airfield and narrow slivers of 600 m terrain tiles produced no hulls at all. The fork welds
  only vertices at exactly the same position. The four models that now use more hulls are all ones upstream
  damaged this way: besides those three, a model of small details spread across 138 m went from 2 hulls, which
  left 98% of its surface uncovered, to 15.
- **Vertex caps cost less.** When a hull has more vertices than `m_maxNumVerticesPerCH`, the fork adds the farthest
  remaining point first instead of taking faces in creation order. With that order, 32-vertex hulls leave less of
  the source uncovered than 44-vertex hulls did before.

### Correct and predictable

- `Compute` returns `ComputeResult` (`Completed`, `Canceled` or `InvalidInput`) and validates its input first.
  Non-finite coordinates, out-of-range indices and out-of-range parameters are rejected, with the reason sent to
  your logger, instead of reading out of bounds.
- Merge order no longer depends on `std::unordered_map` iteration order. Exactly equal merge costs, which are common
  between voxel-aligned pieces, break ties by hull id, so a model decomposes the same way with any standard library.
- Latent undefined behavior is fixed: out-of-range pointer arithmetic in the flood fill, a tree of unbounded depth
  walked with a fixed-size stack, a data race on a static counter shared by all instances, and a quicksort whose
  stack was guarded only by an assert.
- `m_maxRecursionDepth` means what it says, and `ConvexHull::m_center` is the hull's center of mass as a solid.
- A contract test suite (`test/ContractTests.cpp`) covers typed results, validation, cancellation, instance reuse,
  centroids, hull reduction, and the orientation test against exact integer arithmetic.

### Built for engines

- `Compute` runs on the calling thread and owns no threads, so it fits into a job system without oversubscribing
  the CPU. Instances share no mutable state; run one per worker.
- `Cancel()` works from any thread, including from your progress callback, and takes effect promptly: `Compute`
  reports progress after at most 10 ms of work plus its longest indivisible step, which stays under 30 ms on the
  benchmark corpus.

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

    VHACD::IVHACD::Parameters params;
    params.m_maxConvexHulls = 32;
    params.m_resolution = 100000;
    params.m_maxNumVerticesPerCH = 32;

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
input's coordinate space, with their volume and center of mass.

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

| Parameter | Default | Meaning |
| --- | --- | --- |
| `m_maxConvexHulls` | 64 | Maximum number of hulls, at least 1. Pieces merge, cheapest added volume first, until this many remain. |
| `m_resolution` | 400000 | Voxel budget. The longest axis gets floor(1.5 × resolution^0.33) voxels, from 32 to 1021: 67 for 100000, 105 for 400000. |
| `m_minimumVolumePercentErrorAllowed` | 1 | A piece stops splitting once its hull is within this percentage of its voxel volume. |
| `m_tiltedSurfaceAllowance` | 0.5 | Voxel volumes per tilted surface voxel that do not count as error. 0 keeps upstream's test. |
| `m_maxRecursionDepth` | 10 | Pieces at this split depth are not split again (the whole model is depth 0), so at most 2^depth pieces precede merging. |
| `m_minEdgeLength` | 2 | A piece spanning at most this many voxels on every axis is not split again. |
| `m_maxNumVerticesPerCH` | 64 | Maximum vertices per hull, at least 4. Larger hulls keep their farthest points first. |
| `m_shrinkWrap` | true | Moves hull vertices within one voxel of the source mesh onto it. |
| `m_fillMode` | `FLOOD_FILL` | How the interior is found: `FLOOD_FILL` for closed meshes, `RAYCAST_FILL` for meshes with holes, `SURFACE_ONLY` for hollow results. |
| `m_callback` | null | Receives progress on the thread running `Compute`. |
| `m_logger` | null | Receives warnings and the reason for `InvalidInput`. |

The benchmark settings (32 hulls, 32 vertices, resolution 100000) suit game collision well. Raise the resolution
for more detail, and lower `m_maxConvexHulls` for cheaper collision.

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
TestVHACD app/meshes/bunny.obj -h 32 -v 32 -r 100000 -o obj
```

| Option | Meaning (default) |
| --- | --- |
| `-h <n>` | Maximum number of hulls (64) |
| `-r <n>` | Voxel budget (400000) |
| `-e <percent>` | Volume error allowed, 0.001 to 10 (1) |
| `-t <voxels>` | Tilted surface allowance (0.5) |
| `-d <n>` | Recursion depth (10) |
| `-v <n>` | Maximum vertices per hull (64) |
| `-l <n>` | Minimum edge length in voxels (2) |
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
- Upstream's `m_maxRecursionDepth` split one level deeper than documented. Pass N + 1 to get upstream's depth N.
- `ConvexHull::m_center` is the center of mass of the hull as a solid. Upstream returned the area centroid of its
  surface.
- `m_tiltedSurfaceAllowance` is new. Set it to 0 to keep upstream's volume error test.
- Only exactly coincident vertices are welded.
- `TestVHACD` lost `-a` (asynchronous) and `-p` (best plane) and gained `-t`.
- The library needs C++17 and SSE2.

## Limitations

- Flat geometry without thickness still splits into many hulls: a single triangle becomes 17 hulls and a tilted
  quad uses all 32, because the hull of a one-voxel-thick slab rarely gets within the volume error.
- Complex models use every hull `m_maxConvexHulls` allows; the count does not adapt to a target accuracy.
- Detail smaller than a voxel is approximated at voxel scale.

## Benchmark details

- **Corpus**: 611 inputs. 606 are glTF models from a game project: buildings and city blocks, terrain tiles up to
  several hundred meters across, props and vehicles. 5 are synthetic: a triangle, a quad, a tilted quad, a unit
  cube and a rotated unit cube.
- **Settings**: 32 hulls, resolution 100000, 32 vertices per hull, 1% volume error, minimum edge length 2, shrink
  wrap on, flood fill. Recursion depth 8 for upstream and 9 for the fork, which split the same way. Upstream ran with
  `m_asyncACD` off, so both use one thread.
- **Machine**: AMD Ryzen 7 9800X3D, Windows 11, MSVC 14.51 toolset (Visual Studio 2026), x64, `/O2 /GL /fp:precise`.
- **Timing**: wall time of `Compute` for each model, the minimum over two alternating runs for upstream and three
  for the fork. Repeated runs produced identical output for both.
- **Coverage**: 20,000 area-weighted samples on each model's surface. A sample counts as uncovered if it lies more
  than 1 cm or 5 cm outside the nearest hull, measured to that hull's face planes. The table shows the mean over
  all models; a model with no hulls counts as fully uncovered.

## How it works

1. **Voxelize.** The mesh is normalized and voxelized, and the interior is filled by flood fill, by ray casts, or
   not at all.
2. **Split.** Each piece's convex hull is built from the corners of its surface voxels. A piece whose hull exceeds
   its voxels by more than the allowed error splits at the middle of its longest axis, down to the recursion
   depth.
3. **Merge.** Pairs of hulls merge, the pair adding the least volume first, until at most `m_maxConvexHulls` remain.
4. **Finish.** Hull vertices near the source mesh move onto it, and each hull is reduced to at most
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

## License

BSD 3-Clause. See [LICENSE](LICENSE).
