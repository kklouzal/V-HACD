/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#ifndef VHACD_H
#    define VHACD_H

// Please view this slide deck which describes usage and how the algorithm works.
// https://docs.google.com/presentation/d/1OZ4mtZYrGEC8qffqb8F7Le2xzufiqvaPpRbLHKKgTIM/edit?usp=sharing

// VHACD is now a header only library.
// In just *one* of your CPP files *before* you include 'VHACD.h' you must declare
// #define ENABLE_VHACD_IMPLEMENTATION 1
// This will compile the implementation code into your project. If you don't
// have this define, you will get link errors since the implementation code will
// not be present. If you define it more than once in your code base, you will get
// link errors due to a duplicate implementation. This is the same pattern used by
// ImGui and StbLib and other popular open source libraries.

#    define VHACD_VERSION_MAJOR 4
#    define VHACD_VERSION_MINOR 1

// Changes for version 4.1
//
// Various minor tweaks mostly to the test application and some default values.

// Changes for version 4.0
//
// * The code has been significantly refactored to be cleaner and easier to maintain
//      * All OpenCL related code removed
//      * All Bullet code removed
//      * All SIMD code removed
//      * Old plane splitting code removed
//
// * The code is now delivered as a single header file 'VHACD.h' which has both the API
// * declaration as well as the implementation.  Simply add '#define ENABLE_VHACD_IMPLEMENTATION 1'
// * to any CPP in your application prior to including 'VHACD.h'. Only do this in one CPP though.
// * If you do not have this define once, you will get link errors since the implementation code
// * will not be compiled in. If you have this define more than once, you are likely to get
// * duplicate symbol link errors.
//
// * Since the library is now delivered as a single header file, we do not provide binaries
// * or build scripts as these are not needed.
//
// * The old DebugView and test code has all been removed and replaced with a much smaller and
// * simpler test console application with some test meshes to work with.
//
// * The convex hull generation code has changed. The previous version came from Bullet.
// * However, the new version is courtesy of Julio Jerez, the author of the Newton
// * physics engine. His new version is faster and more numerically stable.
//
// * The code can now detect if the input mesh is, itself, already a convex object and
// * can early out.
//
// * Significant performance improvements have been made to the code and it is now much
// * faster, stable, and is easier to tune than previous versions.
//
// * A bug was fixed with the shrink wrapping code (project hull vertices) that could
// * sometime produce artifacts in the results. The new version uses a 'closest point'
// * algorithm that is more reliable.
//
// * You can now select which 'fill mode' to use. For perfectly closed meshes, the default
// * behavior using a flood fill generally works fine. However, some meshes have small
// * holes in them and therefore the flood fill will fail, treating the mesh as being
// * hollow. In these cases, you can use the 'raycast' fill option to determine which
// * parts of the voxelized mesh are 'inside' versus being 'outside'. Finally, there
// * are some rare instances where a user might actually want the mesh to be treated as
// * hollow, in which case you can pass in 'surface' only.
// *
// * A new optional virtual interface called 'IUserProfiler' was provided.
// * This allows the user to provide an optional profiling callback interface to assist in
// * diagnosing performance issues. This change was made by Danny Couture at Epic for the UE4 integration.
// * Some profiling macros were also declared in support of this feature.
// *
// * Another new optional virtual interface called 'IUserTaskRunner' was provided.
// * This interface is used to run logical 'tasks' in a background thread. If none is provided
// * then a default implementation using std::thread will be executed.
// * This change was made by Danny Couture at Epic to speed up the voxelization step.
// *



// The history of V-HACD:
//
// The initial version was written by John W. Ratcliff and was called 'ACD'
// This version did not perform CSG operations on the source mesh, so if you
// recursed too deeply it would produce hollow results.
//
// The next version was written by Khaled Mamou and was called 'HACD'
// In this version Khaled tried to perform a CSG operation on the source
// mesh to produce more robust results. However, Khaled learned that the
// CSG library he was using had licensing issues so he started work on the
// next version.
//
// The next version was called 'V-HACD' because Khaled made the observation
// that plane splitting would be far easier to implement working in voxel space.
//
// V-HACD has been integrated into UE4, Blender, and a number of other projects.
// This new release, version4, is a significant refactor of the code to fix
// some bugs, improve performance, and to make the codebase easier to maintain
// going forward.

#include <stdint.h>

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

namespace VHACD {

struct Vertex
{
    double mX;
    double mY;
    double mZ;

    Vertex() = default;
    Vertex(double x, double y, double z) : mX(x), mY(y), mZ(z) {}

    const double& operator[](size_t idx) const
    {
        switch(idx)
        {
            case 0: return mX;
            case 1: return mY;
            case 2: return mZ;
        };
        return mX;
    }
};

struct Triangle
{
    uint32_t mI0;
    uint32_t mI1;
    uint32_t mI2;

    Triangle() = default;
    Triangle(uint32_t i0, uint32_t i1, uint32_t i2) : mI0(i0), mI1(i1), mI2(i2) {}
};

template <typename T>
class Vector3
{
public:
    /*
    * Getters
    */
    T& operator[](size_t i);
    const T& operator[](size_t i) const;
    T& GetX();
    T& GetY();
    T& GetZ();
    const T& GetX() const;
    const T& GetY() const;
    const T& GetZ() const;

    /*
    * Normalize and norming
    */
    T Normalize();
    T GetNorm() const;
    T GetNormSquared() const;
    int LongestAxis() const;

    /*
    * Vector-vector operations
    */
    Vector3& operator=(const Vector3& rhs);
    Vector3& operator+=(const Vector3& rhs);
    Vector3& operator-=(const Vector3& rhs);

    Vector3 CWiseMul(const Vector3& rhs) const;
    Vector3 Cross(const Vector3& rhs) const;
    T Dot(const Vector3& rhs) const;
    Vector3 operator+(const Vector3& rhs) const;
    Vector3 operator-(const Vector3& rhs) const;

    /*
    * Vector-scalar operations
    */
    Vector3& operator-=(T a);
    Vector3& operator+=(T a);
    Vector3& operator/=(T a);
    Vector3& operator*=(T a);

    Vector3 operator*(T rhs) const;
    Vector3 operator/(T rhs) const;

    /*
    * Unary operations
    */
    Vector3 operator-() const;

    /*
    * Comparison operators
    */

    /*
     * Returns true if all elements of *this are greater than or equal to all elements of rhs, coefficient wise
     * LE is less than or equal
     */
    bool CWiseAllGE(const Vector3<T>& rhs) const;
    bool CWiseAllLE(const Vector3<T>& rhs) const;

    Vector3 CWiseMin(const Vector3& rhs) const;
    Vector3 CWiseMax(const Vector3& rhs) const;
    T MaxCoeff() const;

    T MaxCoeff(uint32_t& idx) const;

    /*
    * Constructors
    */
    Vector3() = default;
    Vector3(T a);
    Vector3(T x, T y, T z);
    Vector3(const Vector3& rhs);
    ~Vector3() = default;

    template <typename U>
    Vector3(const Vector3<U>& rhs);

    Vector3(const VHACD::Vertex&);
    Vector3(const VHACD::Triangle&);

    operator VHACD::Vertex() const;

private:
    std::array<T, 3> m_data{ T(0.0) };
};

typedef VHACD::Vector3<double> Vect3;

struct BoundsAABB
{
    BoundsAABB() = default;
    BoundsAABB(const std::vector<VHACD::Vertex>& points);
    BoundsAABB(const Vect3& min,
               const Vect3& max);

    BoundsAABB Union(const BoundsAABB& b);

    bool Intersects(const BoundsAABB& b) const;

    double Volume() const;

    BoundsAABB Inflate(double ratio) const;

    VHACD::Vect3 ClosestPoint(const VHACD::Vect3& p) const;

    VHACD::Vect3& GetMin();
    VHACD::Vect3& GetMax();
    const VHACD::Vect3& GetMin() const;
    const VHACD::Vect3& GetMax() const;

    VHACD::Vect3 GetSize() const;

    VHACD::Vect3 m_min{ double(0.0) };
    VHACD::Vect3 m_max{ double(0.0) };
};

/**
* This enumeration determines how the voxels as filled to create a solid
* object. The default should be 'FLOOD_FILL' which generally works fine
* for closed meshes. However, if the mesh is not watertight, then using
* RAYCAST_FILL may be preferable as it will determine if a voxel is part
* of the interior of the source mesh by raycasting around it.
*
* Finally, there are some cases where you might actually want a convex
* decomposition to treat the source mesh as being hollow. If that is the
* case you can pass in 'SURFACE_ONLY' and then the convex decomposition
* will converge only onto the 'skin' of the surface mesh.
*/
enum class FillMode
{
    FLOOD_FILL, // This is the default behavior, after the voxelization step it uses a flood fill to determine 'inside'
                // from 'outside'. However, meshes with holes can fail and create hollow results.
    SURFACE_ONLY, // Only consider the 'surface', will create 'skins' with hollow centers.
    RAYCAST_FILL, // Uses raycasting to determine inside from outside.
};

class IVHACD
{
public:
    /**
    * This optional pure virtual interface is used to notify the caller of the progress
    * of convex decomposition. It is called on the thread running Compute.
    */
    class IUserCallback
    {
    public:
        virtual ~IUserCallback(){};

        /**
        * Notifies the application of the current state of the convex decomposition operation
        *
        * @param overallProgress : Total progress from 0-100%
        * @param stageProgress : Progress of the current stage 0-100%
        * @param stage : A text description of the current stage we are in
        * @param operation : A text description of what operation is currently being performed.
        */
        virtual void Update(const double overallProgress,
                            const double stageProgress,
                            const char* const stage,
                            const char* operation) = 0;
    };

    /**
    * Optional user provided pure virtual interface to be notified of warning or informational messages
    */
    class IUserLogger
    {
    public:
        virtual ~IUserLogger(){};
        virtual void Log(const char* const msg) = 0;
    };

    /**
    * A simple class that represents a convex hull as a triangle mesh with
    * double precision vertices. Polygons are not currently provided.
    */
    class ConvexHull
    {
    public:
        std::vector<VHACD::Vertex>      m_points;
        std::vector<VHACD::Triangle>    m_triangles;

        double                          m_volume{ 0 };          // The volume of the convex hull
        VHACD::Vect3                    m_center{ 0, 0, 0 };    // Center of mass of the hull as a uniform solid (area centroid for a flat hull)
        uint32_t                        m_meshId{ 0 };          // A unique id for this convex hull
        VHACD::Vect3            mBmin;                  // Bounding box minimum of the AABB
        VHACD::Vect3            mBmax;                  // Bounding box maximum of the AABB
    };

    /**
    * This class provides the parameters controlling the convex decomposition operation
    */
    class Parameters
    {
    public:
        IUserCallback*      m_callback{nullptr};            // Optional user provided callback interface for progress
        IUserLogger*        m_logger{nullptr};              // Optional user provided callback interface for log messages
        // How far a hull may stand off the surface the probe below can reach, in the units of the input.
        // The tolerance used is clamp(m_relativeTolerance * bounding box diagonal, m_minTolerance,
        // m_maxTolerance); it also sets the voxel size, so a model is decomposed to the accuracy asked of
        // it. The hull count follows from it: pieces split while their hull reaches too far, and merge
        // while the merged hull does not. All three must be finite and positive.
        double              m_relativeTolerance{ 0.01 };
        double              m_minTolerance{ 0.01 };
        double              m_maxTolerance{ 0.03 };
        // Gaps, openings and pockets a sphere of this radius cannot enter from outside are filled, so hulls
        // may cover them: nothing of that size can occupy them. 0 keeps every reachable pocket and fills
        // only sealed cavities. Finite and non-negative.
        double              m_probeRadius{ 0.05 };
        // Voxel budget. The voxel size, and with it the tolerance, is coarsened when the tolerance would
        // need a finer grid than this; IVHACD::GetReport records what was used. A grid never exceeds 1021
        // voxels on an axis whatever the budget allows, since voxel coordinates are packed into ten bits.
        uint32_t            m_maxVoxels{ 512u << 10 };
        // Hull budget. The count normally follows from the tolerance; hulls merge past the tolerance only
        // to meet this bound, which the report records. At least 1.
        uint32_t            m_maxConvexHulls{ 64 };
        // Piece budget. Splitting stops here even where a piece still reaches too far, which bounds the
        // work of a model whose detail the tolerance cannot capture. At least m_maxConvexHulls.
        uint32_t            m_maxPieces{ 512 };
        uint32_t            m_maxNumVerticesPerCH{ 32 };    // The maximum number of vertices allowed in any output convex hull; at least 4. A hull over the limit keeps the vertices farthest out first
        bool                m_shrinkWrap{true};             // Whether or not to shrinkwrap the voxel positions to the source mesh on output
        FillMode            m_fillMode{ FillMode::FLOOD_FILL }; // How to fill the interior of the voxelized mesh
    };

    /**
    * What a decomposition used, so a caller can log the models its budgets constrained.
    */
    class Report
    {
    public:
        double      m_tolerance{ 0 };            // Tolerance used, in the units of the input
        double      m_probeRadius{ 0 };          // Probe radius used, in the units of the input
        double      m_voxelSize{ 0 };            // Edge length of one voxel, in the units of the input
        uint32_t    m_voxelCount{ 0 };           // Voxels in the grid
        uint32_t    m_pieceCount{ 0 };           // Pieces the splitting produced, before any merging
        uint64_t    m_protectedVoxels{ 0 };      // Voxels of reachable space no hull may cover
        bool        m_pieceBudgetBound{ false }; // Splitting stopped at m_maxPieces, so a piece may reach too far
        bool        m_voxelBudgetBound{ false }; // m_maxVoxels forced a coarser tolerance than requested
        bool        m_hullBudgetBound{ false };  // Hulls merged past the tolerance to meet m_maxConvexHulls
    };

    /**
    * Outcome of Compute.
    */
    enum class ComputeResult
    {
        Completed,    // Decomposition finished; GetNConvexHulls is zero only when no voxel was produced
        Canceled,     // Cancel was requested while Compute ran; no hulls are available
        InvalidInput, // The mesh or parameters violate the Compute contract; the reason goes to IUserLogger
    };

    /**
    * Requests that a running Compute stop early. It may be called from any thread, including
    * from IUserCallback::Update. The interrupted Compute returns ComputeResult::Canceled.
    * A request made before Compute starts is discarded.
    */
    virtual void Cancel() = 0;

    /**
    * Compute a convex decomposition of a triangle mesh using float vertices and the provided user parameters.
    *
    * @param points : countPoints * 3 finite coordinates in the form X1,Y1,Z1, X2,Y2,Z2, ...; may be null only when countPoints is 0
    * @param countPoints : The number of vertices in the source mesh.
    * @param triangles : countTriangles * 3 vertex indices, each below countPoints; may be null only when countTriangles is 0
    * @param countTriangles : The number of triangles in the source mesh
    * @param params : The convex decomposition parameters to apply
    * @return : Completed, Canceled, or InvalidInput. Previous results are released in every case.
    */
    virtual ComputeResult Compute(const float* const points,
                         const uint32_t countPoints,
                         const uint32_t* const triangles,
                         const uint32_t countTriangles,
                         const Parameters& params) = 0;

    /**
    * Compute a convex decomposition of a triangle mesh using double vertices and the provided user parameters.
    *
    * @param points : countPoints * 3 finite coordinates in the form X1,Y1,Z1, X2,Y2,Z2, ...; may be null only when countPoints is 0
    * @param countPoints : The number of vertices in the source mesh.
    * @param triangles : countTriangles * 3 vertex indices, each below countPoints; may be null only when countTriangles is 0
    * @param countTriangles : The number of triangles in the source mesh
    * @param params : The convex decomposition parameters to apply
    * @return : Completed, Canceled, or InvalidInput. Previous results are released in every case.
    */
    virtual ComputeResult Compute(const double* const points,
                         const uint32_t countPoints,
                         const uint32_t* const triangles,
                         const uint32_t countTriangles,
                         const Parameters& params) = 0;

    /**
    * Returns what the last Compute used: its tolerance, voxel size and whether a budget constrained it.
    */
    virtual const Report& GetReport() const = 0;

    /**
    * Returns the number of convex hulls that were produced.
    *
    * @return : Returns the number of convex hulls produced, or zero if it failed or was canceled
    */
    virtual uint32_t GetNConvexHulls() const = 0;

    /**
    * Retrieves one of the convex hulls in the solution set
    *
    * @param index : Which convex hull to retrieve
    * @param ch : The convex hull descriptor to return
    * @return : Returns true if the convex hull exists and could be retrieved
    */
    virtual bool GetConvexHull(const uint32_t index,
                               ConvexHull& ch) const = 0;

    /**
    * Releases any memory allocated by the V-HACD class
    */
    virtual void Clean() = 0; // release internally allocated memory

    /**
    * Releases this instance of the V-HACD class
    */
    virtual void Release() = 0; // release IVHACD

protected:
    virtual ~IVHACD()
    {
    }
};
/*
 * Out of line definitions
 */

    template <typename T>
    T clamp(const T& v, const T& lo, const T& hi)
    {
        if (v < lo)
        {
            return lo;
        }
        if (v > hi)
        {
            return hi;
        }
        return v ;
    }

/*
 * Getters
 */
    template <typename T>
    inline T& Vector3<T>::operator[](size_t i)
    {
        return m_data[i];
    }

    template <typename T>
    inline const T& Vector3<T>::operator[](size_t i) const
    {
        return m_data[i];
    }

    template <typename T>
    inline T& Vector3<T>::GetX()
    {
        return m_data[0];
    }

    template <typename T>
    inline T& Vector3<T>::GetY()
    {
        return m_data[1];
    }

    template <typename T>
    inline T& Vector3<T>::GetZ()
    {
        return m_data[2];
    }

    template <typename T>
    inline const T& Vector3<T>::GetX() const
    {
        return m_data[0];
    }

    template <typename T>
    inline const T& Vector3<T>::GetY() const
    {
        return m_data[1];
    }

    template <typename T>
    inline const T& Vector3<T>::GetZ() const
    {
        return m_data[2];
    }

/*
 * Normalize and norming
 */
    template <typename T>
    inline T Vector3<T>::Normalize()
    {
        T n = GetNorm();
        if (n != T(0.0)) (*this) /= n;
        return n;
    }

    template <typename T>
    inline T Vector3<T>::GetNorm() const
    {
        return std::sqrt(GetNormSquared());
    }

    template <typename T>
    inline T Vector3<T>::GetNormSquared() const
    {
        return this->Dot(*this);
    }

    template <typename T>
    inline int Vector3<T>::LongestAxis() const
    {
        auto it = std::max_element(m_data.begin(), m_data.end());
        return int(std::distance(m_data.begin(), it));
    }

/*
 * Vector-vector operations
 */
    template <typename T>
    inline Vector3<T>& Vector3<T>::operator=(const Vector3<T>& rhs)
    {
        GetX() = rhs.GetX();
        GetY() = rhs.GetY();
        GetZ() = rhs.GetZ();
        return *this;
    }

    template <typename T>
    inline Vector3<T>& Vector3<T>::operator+=(const Vector3<T>& rhs)
    {
        GetX() += rhs.GetX();
        GetY() += rhs.GetY();
        GetZ() += rhs.GetZ();
        return *this;
    }

    template <typename T>
    inline Vector3<T>& Vector3<T>::operator-=(const Vector3<T>& rhs)
    {
        GetX() -= rhs.GetX();
        GetY() -= rhs.GetY();
        GetZ() -= rhs.GetZ();
        return *this;
    }

    template <typename T>
    inline Vector3<T> Vector3<T>::CWiseMul(const Vector3<T>& rhs) const
    {
        return Vector3<T>(GetX() * rhs.GetX(),
                          GetY() * rhs.GetY(),
                          GetZ() * rhs.GetZ());
    }

    template <typename T>
    inline Vector3<T> Vector3<T>::Cross(const Vector3<T>& rhs) const
    {
        return Vector3<T>(GetY() * rhs.GetZ() - GetZ() * rhs.GetY(),
                          GetZ() * rhs.GetX() - GetX() * rhs.GetZ(),
                          GetX() * rhs.GetY() - GetY() * rhs.GetX());
    }

    template <typename T>
    inline T Vector3<T>::Dot(const Vector3<T>& rhs) const
    {
        return   GetX() * rhs.GetX()
                 + GetY() * rhs.GetY()
                 + GetZ() * rhs.GetZ();
    }

    template <typename T>
    inline Vector3<T> Vector3<T>::operator+(const Vector3<T>& rhs) const
    {
        return Vector3<T>(GetX() + rhs.GetX(),
                          GetY() + rhs.GetY(),
                          GetZ() + rhs.GetZ());
    }

    template <typename T>
    inline Vector3<T> Vector3<T>::operator-(const Vector3<T>& rhs) const
    {
        return Vector3<T>(GetX() - rhs.GetX(),
                          GetY() - rhs.GetY(),
                          GetZ() - rhs.GetZ());
    }

    template <typename T>
    inline Vector3<T> operator*(T lhs, const Vector3<T>& rhs)
    {
        return Vector3<T>(lhs * rhs.GetX(),
                          lhs * rhs.GetY(),
                          lhs * rhs.GetZ());
    }

/*
 * Vector-scalar operations
 */
    template <typename T>
    inline Vector3<T>& Vector3<T>::operator-=(T a)
    {
        GetX() -= a;
        GetY() -= a;
        GetZ() -= a;
        return *this;
    }

    template <typename T>
    inline Vector3<T>& Vector3<T>::operator+=(T a)
    {
        GetX() += a;
        GetY() += a;
        GetZ() += a;
        return *this;
    }

    template <typename T>
    inline Vector3<T>& Vector3<T>::operator/=(T a)
    {
        GetX() /= a;
        GetY() /= a;
        GetZ() /= a;
        return *this;
    }

    template <typename T>
    inline Vector3<T>& Vector3<T>::operator*=(T a)
    {
        GetX() *= a;
        GetY() *= a;
        GetZ() *= a;
        return *this;
    }

    template <typename T>
    inline Vector3<T> Vector3<T>::operator*(T rhs) const
    {
        return Vector3<T>(GetX() * rhs,
                          GetY() * rhs,
                          GetZ() * rhs);
    }

    template <typename T>
    inline Vector3<T> Vector3<T>::operator/(T rhs) const
    {
        return Vector3<T>(GetX() / rhs,
                          GetY() / rhs,
                          GetZ() / rhs);
    }

/*
 * Unary operations
 */
    template <typename T>
    inline Vector3<T> Vector3<T>::operator-() const
    {
        return Vector3<T>(-GetX(),
                          -GetY(),
                          -GetZ());
    }

/*
 * Comparison operators
 */
    template <typename T>
    inline bool Vector3<T>::CWiseAllGE(const Vector3<T>& rhs) const
    {
        return    GetX() >= rhs.GetX()
                  && GetY() >= rhs.GetY()
                  && GetZ() >= rhs.GetZ();
    }

    template <typename T>
    inline bool Vector3<T>::CWiseAllLE(const Vector3<T>& rhs) const
    {
        return    GetX() <= rhs.GetX()
                  && GetY() <= rhs.GetY()
                  && GetZ() <= rhs.GetZ();
    }

    template <typename T>
    inline Vector3<T> Vector3<T>::CWiseMin(const Vector3<T>& rhs) const
    {
        return Vector3<T>(std::min(GetX(), rhs.GetX()),
                          std::min(GetY(), rhs.GetY()),
                          std::min(GetZ(), rhs.GetZ()));
    }

    template <typename T>
    inline Vector3<T> Vector3<T>::CWiseMax(const Vector3<T>& rhs) const
    {
        return Vector3<T>(std::max(GetX(), rhs.GetX()),
                          std::max(GetY(), rhs.GetY()),
                          std::max(GetZ(), rhs.GetZ()));
    }

    template <typename T>
    inline T Vector3<T>::MaxCoeff() const
    {
        return *std::max_element(m_data.begin(), m_data.end());
    }

    template <typename T>
    inline T Vector3<T>::MaxCoeff(uint32_t& idx) const
    {
        auto it = std::max_element(m_data.begin(), m_data.end());
        idx = uint32_t(std::distance(m_data.begin(), it));
        return *it;
    }

/*
 * Constructors
 */
    template <typename T>
    inline Vector3<T>::Vector3(T a)
            : m_data{a, a, a}
    {
    }

    template <typename T>
    inline Vector3<T>::Vector3(T x, T y, T z)
            : m_data{x, y, z}
    {
    }

    template <typename T>
    inline Vector3<T>::Vector3(const Vector3& rhs)
            : m_data{rhs.m_data}
    {
    }

    template <typename T>
    template <typename U>
    inline Vector3<T>::Vector3(const Vector3<U>& rhs)
            : m_data{T(rhs.GetX()), T(rhs.GetY()), T(rhs.GetZ())}
    {
    }

    template <typename T>
    inline Vector3<T>::Vector3(const VHACD::Vertex& rhs)
            : Vector3<T>(rhs.mX, rhs.mY, rhs.mZ)
    {
        static_assert(std::is_same<T, double>::value, "Vertex to Vector3 constructor only enabled for double");
    }

    template <typename T>
    inline Vector3<T>::Vector3(const VHACD::Triangle& rhs)
            : Vector3<T>(rhs.mI0, rhs.mI1, rhs.mI2)
    {
        static_assert(std::is_same<T, uint32_t>::value, "Triangle to Vector3 constructor only enabled for uint32_t");
    }

    template <typename T>
    inline Vector3<T>::operator VHACD::Vertex() const
    {
        static_assert(std::is_same<T, double>::value, "Vector3 to Vertex conversion only enable for double");
        return ::VHACD::Vertex( GetX(), GetY(), GetZ());
    }

IVHACD* CreateVHACD();      // Create a V-HACD instance; Compute runs synchronously on the calling thread

} // namespace VHACD

#if ENABLE_VHACD_IMPLEMENTATION
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <limits.h>

#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef _MSC_VER
#include <intrin.h>
#endif // _MSC_VER

// The implementation uses SSE2, which every x86-64 CPU has.
#if !(defined(__SSE2__) || defined(_M_X64) || defined(_M_AMD64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2))
#error "The V-HACD implementation requires SSE2"
#endif
#include <emmintrin.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4100 4127 4189 4244 4456 4701 4702 4996)
#endif // _MSC_VER

#ifdef __GNUC__
#pragma GCC diagnostic push
// Minimum set of warnings used for cleanup
// #pragma GCC diagnostic warning "-Wall"
// #pragma GCC diagnostic warning "-Wextra"
// #pragma GCC diagnostic warning "-Wpedantic"
// #pragma GCC diagnostic warning "-Wold-style-cast"
// #pragma GCC diagnostic warning "-Wnon-virtual-dtor"
// #pragma GCC diagnostic warning "-Wshadow"
#endif // __GNUC__

namespace VHACD {

// Index of the lowest set bit; bits must be non-zero.
inline unsigned CountTrailingZeros(const uint64_t bits)
{
    assert(bits != 0);
#ifdef _MSC_VER
    unsigned long index;
    _BitScanForward64(&index, bits);
    return unsigned(index);
#else
    return unsigned(__builtin_ctzll(bits));
#endif
}

} // namespace VHACD

// Scoped Timer
namespace VHACD {

class Timer
{
public:
    Timer()
        : m_startTime(std::chrono::high_resolution_clock::now())
    {
    }

    void Reset()
    {
        m_startTime = std::chrono::high_resolution_clock::now();
    }

    double GetElapsedSeconds()
    {
        auto s = PeekElapsedSeconds();
        Reset();
        return s;
    }

    double PeekElapsedSeconds()
    {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = now - m_startTime;
        return diff.count();
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
};

class ScopedTime
{
public:
    ScopedTime(const char* action,
               VHACD::IVHACD::IUserLogger* logger)
        : m_action(action)
        , m_logger(logger)
    {
        m_timer.Reset();
    }

    ~ScopedTime()
    {
        double dtime = m_timer.GetElapsedSeconds();
        if( m_logger )
        {
            char scratch[512];
            snprintf(scratch,
                        sizeof(scratch),"%s took %0.5f seconds",
                        m_action,
                        dtime);
            m_logger->Log(scratch);
        }
    }

    const char* m_action{ nullptr };
    Timer       m_timer;
    VHACD::IVHACD::IUserLogger* m_logger{ nullptr };
};
BoundsAABB::BoundsAABB(const std::vector<VHACD::Vertex>& points)
        : m_min(points[0])
        , m_max(points[0])
{
    for (uint32_t i = 1; i < points.size(); ++i)
    {
        const VHACD::Vertex& p = points[i];
        m_min = m_min.CWiseMin(p);
        m_max = m_max.CWiseMax(p);
    }
}

BoundsAABB::BoundsAABB(const VHACD::Vect3& min,
                              const VHACD::Vect3& max)
        : m_min(min)
        , m_max(max)
{
}

BoundsAABB BoundsAABB::Union(const BoundsAABB& b)
{
    return BoundsAABB(GetMin().CWiseMin(b.GetMin()),
                      GetMax().CWiseMax(b.GetMax()));
}

bool VHACD::BoundsAABB::Intersects(const VHACD::BoundsAABB& b) const
{
    if (   (  GetMin().GetX() > b.GetMax().GetX())
           || (b.GetMin().GetX() >   GetMax().GetX()))
        return false;
    if (   (  GetMin().GetY() > b.GetMax().GetY())
           || (b.GetMin().GetY() >   GetMax().GetY()))
        return false;
    if (   (  GetMin().GetZ() > b.GetMax().GetZ())
           || (b.GetMin().GetZ() >   GetMax().GetZ()))
        return false;
    return true;
}

double VHACD::BoundsAABB::Volume() const
{
    VHACD::Vect3 d = GetMax() - GetMin();
    return d.GetX() * d.GetY() * d.GetZ();
}

BoundsAABB VHACD::BoundsAABB::Inflate(double ratio) const
{
    double inflate = (GetMin() - GetMax()).GetNorm() * double(0.5) * ratio;
    return BoundsAABB(GetMin() - inflate,
                      GetMax() + inflate);
}

VHACD::Vect3 VHACD::BoundsAABB::ClosestPoint(const VHACD::Vect3& p) const
{
    return p.CWiseMax(GetMin()).CWiseMin(GetMax());
}

VHACD::Vect3& VHACD::BoundsAABB::GetMin()
{
    return m_min;
}

VHACD::Vect3& VHACD::BoundsAABB::GetMax()
{
    return m_max;
}

inline const VHACD::Vect3& VHACD::BoundsAABB::GetMin() const
{
    return m_min;
}

const VHACD::Vect3& VHACD::BoundsAABB::GetMax() const
{
    return m_max;
}

VHACD::Vect3 VHACD::BoundsAABB::GetSize() const
{
    return GetMax() - GetMin();
}

// Center of mass of a closed triangle mesh as a uniform solid. A flat mesh has no volume, so its area
// centroid is used instead, and the vertex average when it has no area either.
void ComputeCentroid(const std::vector<VHACD::Vertex>& points,
                     const std::vector<VHACD::Triangle>& indices,
                     VHACD::Vect3& center)
{
    center = VHACD::Vect3(0);
    if ( points.empty() )
    {
        return;
    }
    const VHACD::Vect3 origin(points[0]);
    VHACD::Vect3 volumeMoment(0);
    VHACD::Vect3 areaMoment(0);
    double volume6 = 0; // six times the signed volume
    double area2 = 0;   // twice the area
    for (const VHACD::Triangle& t : indices)
    {
        const VHACD::Vect3 a(points[t.mI0]);
        const VHACD::Vect3 b(points[t.mI1]);
        const VHACD::Vect3 c(points[t.mI2]);
        const double tetra6 = (a - origin).Dot((b - origin).Cross(c - origin));
        volumeMoment += (origin + a + b + c) * tetra6;
        volume6 += tetra6;
        const double triangle2 = (b - a).Cross(c - a).GetNorm();
        areaMoment += (a + b + c) * triangle2;
        area2 += triangle2;
    }
    if ( std::abs(volume6) > 1e-9 * area2 * std::sqrt(area2) )
    {
        center = volumeMoment / (4.0 * volume6);
    }
    else if ( area2 > 0 )
    {
        center = areaMoment / (3.0 * area2);
    }
    else
    {
        for (const VHACD::Vertex& p : points)
        {
            center += VHACD::Vect3(p);
        }
        center /= double(points.size());
    }
}

double Determinant3x3(const std::array<VHACD::Vect3, 3>& matrix,
                      double& error)
{
    double det = double(0.0);
    error = double(0.0);

    double a01xa12 = matrix[0].GetY() * matrix[1].GetZ();
    double a02xa11 = matrix[0].GetZ() * matrix[1].GetY();
    error += (std::abs(a01xa12) + std::abs(a02xa11)) * std::abs(matrix[2].GetX());
    det += (a01xa12 - a02xa11) * matrix[2].GetX();

    double a00xa12 = matrix[0].GetX() * matrix[1].GetZ();
    double a02xa10 = matrix[0].GetZ() * matrix[1].GetX();
    error += (std::abs(a00xa12) + std::abs(a02xa10)) * std::abs(matrix[2].GetY());
    det -= (a00xa12 - a02xa10) * matrix[2].GetY();

    double a00xa11 = matrix[0].GetX() * matrix[1].GetY();
    double a01xa10 = matrix[0].GetY() * matrix[1].GetX();
    error += (std::abs(a00xa11) + std::abs(a01xa10)) * std::abs(matrix[2].GetZ());
    det += (a00xa11 - a01xa10) * matrix[2].GetZ();

    return det;
}

double ComputeMeshVolume(const std::vector<VHACD::Vertex>& vertices,
                         const std::vector<VHACD::Triangle>& indices)
{
    double volume = 0;
    for (uint32_t i = 0; i < indices.size(); i++)
    {
        const std::array<VHACD::Vect3, 3> m = {
            vertices[indices[i].mI0],
            vertices[indices[i].mI1],
            vertices[indices[i].mI2]
        };
        double placeholder;
        volume += Determinant3x3(m,
                                 placeholder);
    }

    volume *= (double(1.0) / double(6.0));
    if (volume < 0)
        volume *= -1;
    return volume;
}


/*
 * Returns index of highest set bit in x
 */
inline int dExp2(int x)
{
    int exp;
    for (exp = -1; x; x >>= 1)
    {
        exp++;
    }
    return exp;
}

/*
 * Reverses the order of the bits in v and returns the result
 * Does not put fill any of the bits higher than the highest bit in v
 * Only used to calculate index of ndNormalMap::m_normal when tessellating a triangle
 */
inline int dBitReversal(int v,
                        int base)
{
    int x = 0;
    int power = dExp2(base) - 1;
    do
    {
        x += (v & 1) << power;
        v >>= 1;
        power--;
    } while (v);
    return x;
}

class Googol
{
    #define VHACD_GOOGOL_SIZE 4
public:
    Googol() = default;
    Googol(double value);

    operator double() const;
    Googol operator+(const Googol &A) const;
    Googol operator-(const Googol &A) const;
    Googol operator*(const Googol &A) const;

    Googol& operator+= (const Googol &A);
    Googol& operator-= (const Googol &A);




private:
    void NegateMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const;
    void CopySignedMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const;
    int NormalizeMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const;
    void ShiftRightMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa,
                            int bits) const;
    uint64_t CheckCarrier(uint64_t a, uint64_t b) const;

    int LeadingZeros(uint64_t a) const;
    void ExtendedMultiply(uint64_t a,
                          uint64_t b,
                          uint64_t& high,
                          uint64_t& low) const;
    void ScaleMantissa(uint64_t* out,
                       uint64_t scale) const;

    int m_sign{ 0 };
    int m_exponent{ 0 };
    std::array<uint64_t, VHACD_GOOGOL_SIZE> m_mantissa{ 0 };
};

Googol::Googol(double value)
{
    int exp;
    double mantissa = fabs(frexp(value, &exp));

    m_exponent = exp;
    m_sign = (value >= 0) ? 0 : 1;

    m_mantissa[0] = uint64_t(double(uint64_t(1) << 62) * mantissa);
}

Googol::operator double() const
{
    double mantissa = (double(1.0) / double(uint64_t(1) << 62)) * double(m_mantissa[0]);
    mantissa = ldexp(mantissa, m_exponent) * (m_sign ? double(-1.0) : double(1.0));
    return mantissa;
}

Googol Googol::operator+(const Googol &A) const
{
    Googol tmp;
    if (m_mantissa[0] && A.m_mantissa[0])
    {
        std::array<uint64_t, VHACD_GOOGOL_SIZE> mantissa0;
        std::array<uint64_t, VHACD_GOOGOL_SIZE> mantissa1;
        std::array<uint64_t, VHACD_GOOGOL_SIZE> mantissa;

        CopySignedMantissa(mantissa0);
        A.CopySignedMantissa(mantissa1);

        int exponentDiff = m_exponent - A.m_exponent;
        int exponent = m_exponent;
        if (exponentDiff > 0)
        {
            ShiftRightMantissa(mantissa1,
                               exponentDiff);
        }
        else if (exponentDiff < 0)
        {
            exponent = A.m_exponent;
            ShiftRightMantissa(mantissa0,
                               -exponentDiff);
        }

        uint64_t carrier = 0;
        for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
        {
            uint64_t m0 = mantissa0[i];
            uint64_t m1 = mantissa1[i];
            mantissa[i] = m0 + m1 + carrier;
            carrier = CheckCarrier(m0, m1) | CheckCarrier(m0 + m1, carrier);
        }

        int sign = 0;
        if (int64_t(mantissa[0]) < 0)
        {
            sign = 1;
            NegateMantissa(mantissa);
        }

        int bits = NormalizeMantissa(mantissa);
        if (bits <= (-64 * VHACD_GOOGOL_SIZE))
        {
            tmp.m_sign = 0;
            tmp.m_exponent = 0;
        }
        else
        {
            tmp.m_sign = sign;
            tmp.m_exponent = int(exponent + bits);
        }

        tmp.m_mantissa = mantissa;
    }
    else if (A.m_mantissa[0])
    {
        tmp = A;
    }
    else
    {
        tmp = *this;
    }

    return tmp;
}

Googol Googol::operator-(const Googol &A) const
{
    Googol tmp(A);
    tmp.m_sign = !tmp.m_sign;
    return *this + tmp;
}

Googol Googol::operator*(const Googol &A) const
{
    if (m_mantissa[0] && A.m_mantissa[0])
    {
        std::array<uint64_t, VHACD_GOOGOL_SIZE * 2> mantissaAcc{ 0 };
        for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
        {
            uint64_t a = m_mantissa[i];
            if (a)
            {
                uint64_t mantissaScale[2 * VHACD_GOOGOL_SIZE] = { 0 };
                A.ScaleMantissa(&mantissaScale[i], a);

                uint64_t carrier = 0;
                for (int j = 0; j < 2 * VHACD_GOOGOL_SIZE; j++)
                {
                    const int k = 2 * VHACD_GOOGOL_SIZE - 1 - j;
                    uint64_t m0 = mantissaAcc[k];
                    uint64_t m1 = mantissaScale[k];
                    mantissaAcc[k] = m0 + m1 + carrier;
                    carrier = CheckCarrier(m0, m1) | CheckCarrier(m0 + m1, carrier);
                }
            }
        }

        uint64_t carrier = 0;
        int bits = LeadingZeros(mantissaAcc[0]) - 2;
        for (int i = 0; i < 2 * VHACD_GOOGOL_SIZE; i++)
        {
            const int k = 2 * VHACD_GOOGOL_SIZE - 1 - i;
            uint64_t a = mantissaAcc[k];
            mantissaAcc[k] = (a << uint64_t(bits)) | carrier;
            carrier = a >> uint64_t(64 - bits);
        }

        int exp = m_exponent + A.m_exponent - (bits - 2);

        Googol tmp;
        tmp.m_sign = m_sign ^ A.m_sign;
        tmp.m_exponent = exp;
        for (std::size_t i = 0; i < tmp.m_mantissa.size(); ++i)
        {
            tmp.m_mantissa[i] = mantissaAcc[i];
        }

        return tmp;
    }
    return Googol(double(0.0));
}

Googol& Googol::operator+=(const Googol &A)
{
    *this = *this + A;
    return *this;
}

Googol& Googol::operator-=(const Googol &A)
{
    *this = *this - A;
    return *this;
}

void Googol::NegateMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const
{
    uint64_t carrier = 1;
    for (size_t i = mantissa.size() - 1; i < mantissa.size(); i--)
    {
        uint64_t a = ~mantissa[i] + carrier;
        if (a)
        {
            carrier = 0;
        }
        mantissa[i] = a;
    }
}

void Googol::CopySignedMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const
{
    mantissa = m_mantissa;
    if (m_sign)
    {
        NegateMantissa(mantissa);
    }
}

int Googol::NormalizeMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa) const
{
    int bits = 0;
    if (int64_t(mantissa[0] * 2) < 0)
    {
        bits = 1;
        ShiftRightMantissa(mantissa, 1);
    }
    else
    {
        while (!mantissa[0] && bits > (-64 * VHACD_GOOGOL_SIZE))
        {
            bits -= 64;
            for (int i = 1; i < VHACD_GOOGOL_SIZE; i++) {
                mantissa[i - 1] = mantissa[i];
            }
            mantissa[VHACD_GOOGOL_SIZE - 1] = 0;
        }

        if (bits > (-64 * VHACD_GOOGOL_SIZE))
        {
            int n = LeadingZeros(mantissa[0]) - 2;
            if (n > 0)
            {
                uint64_t carrier = 0;
                for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
                {
                    uint64_t a = mantissa[i];
                    mantissa[i] = (a << n) | carrier;
                    carrier = a >> (64 - n);
                }
                bits -= n;
            }
            else if (n < 0)
            {
                // this is very rare but it does happens, whee the leading zeros of the mantissa is an exact multiple of 64
                uint64_t carrier = 0;
                int shift = -n;
                for (int i = 0; i < VHACD_GOOGOL_SIZE; i++)
                {
                    uint64_t a = mantissa[i];
                    mantissa[i] = (a >> shift) | carrier;
                    carrier = a << (64 - shift);
                }
                bits -= n;
            }
        }
    }
    return bits;
}

void Googol::ShiftRightMantissa(std::array<uint64_t, VHACD_GOOGOL_SIZE>& mantissa,
                                int bits) const
{
    uint64_t carrier = 0;
    if (int64_t(mantissa[0]) < int64_t(0))
    {
        carrier = uint64_t(-1);
    }

    while (bits >= 64)
    {
        for (int i = VHACD_GOOGOL_SIZE - 2; i >= 0; i--)
        {
            mantissa[i + 1] = mantissa[i];
        }
        mantissa[0] = carrier;
        bits -= 64;
    }

    if (bits > 0)
    {
        carrier <<= (64 - bits);
        for (int i = 0; i < VHACD_GOOGOL_SIZE; i++)
        {
            uint64_t a = mantissa[i];
            mantissa[i] = (a >> bits) | carrier;
            carrier = a << (64 - bits);
        }
    }
}

uint64_t Googol::CheckCarrier(uint64_t a, uint64_t b) const
{
    return ((uint64_t(-1) - b) < a) ? uint64_t(1) : 0;
}

int Googol::LeadingZeros(uint64_t a) const
{
    #define VHACD_COUNTBIT(mask, add)	\
    do {								\
        uint64_t test = a & mask;		\
        n += test ? 0 : add;			\
        a = test ? test : (a & ~mask);	\
    } while (false)

    int n = 0;
    VHACD_COUNTBIT(0xffffffff00000000LL, 32);
    VHACD_COUNTBIT(0xffff0000ffff0000LL, 16);
    VHACD_COUNTBIT(0xff00ff00ff00ff00LL, 8);
    VHACD_COUNTBIT(0xf0f0f0f0f0f0f0f0LL, 4);
    VHACD_COUNTBIT(0xccccccccccccccccLL, 2);
    VHACD_COUNTBIT(0xaaaaaaaaaaaaaaaaLL, 1);

    return n;
}

void Googol::ExtendedMultiply(uint64_t a,
                              uint64_t b,
                              uint64_t& high,
                              uint64_t& low) const
{
    uint64_t bLow = b & 0xffffffff;
    uint64_t bHigh = b >> 32;
    uint64_t aLow = a & 0xffffffff;
    uint64_t aHigh = a >> 32;

    uint64_t l = bLow * aLow;

    uint64_t c1 = bHigh * aLow;
    uint64_t c2 = bLow * aHigh;
    uint64_t m = c1 + c2;
    uint64_t carrier = CheckCarrier(c1, c2) << 32;

    uint64_t h = bHigh * aHigh + carrier;

    uint64_t ml = m << 32;
    uint64_t ll = l + ml;
    uint64_t mh = (m >> 32) + CheckCarrier(l, ml);
    uint64_t hh = h + mh;

    low = ll;
    high = hh;
}

void Googol::ScaleMantissa(uint64_t* dst,
                           uint64_t scale) const
{
    uint64_t carrier = 0;
    for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
    {
        if (m_mantissa[i])
        {
            uint64_t low;
            uint64_t high;
            ExtendedMultiply(scale,
                             m_mantissa[i],
                             high,
                             low);
            uint64_t acc = low + carrier;
            carrier = CheckCarrier(low,
                                   carrier);
            carrier += high;
            dst[i + 1] = acc;
        }
        else
        {
            dst[i + 1] = carrier;
            carrier = 0;
        }

    }
    dst[0] = carrier;
}

Googol Determinant3x3(const std::array<VHACD::Vector3<Googol>, 3>& matrix)
{
    Googol det = double(0.0);

    Googol a01xa12 = matrix[0].GetY() * matrix[1].GetZ();
    Googol a02xa11 = matrix[0].GetZ() * matrix[1].GetY();
    det += (a01xa12 - a02xa11) * matrix[2].GetX();

    Googol a00xa12 = matrix[0].GetX() * matrix[1].GetZ();
    Googol a02xa10 = matrix[0].GetZ() * matrix[1].GetX();
    det -= (a00xa12 - a02xa10) * matrix[2].GetY();

    Googol a00xa11 = matrix[0].GetX() * matrix[1].GetY();
    Googol a01xa10 = matrix[0].GetY() * matrix[1].GetX();
    det += (a00xa11 - a01xa10) * matrix[2].GetZ();
    return det;
}

/*
 * Exact floating-point expansion arithmetic after J. R. Shewchuk, "Adaptive Precision Floating-Point Arithmetic and
 * Fast Robust Geometric Predicates" (1997). An expansion is a sum of doubles in increasing order of magnitude whose
 * nonzero terms do not overlap, so its largest term has the sign of the exact sum and is within a factor of two of
 * it. The operations are exact for round-to-nearest double arithmetic evaluated in double precision
 * (FLT_EVAL_METHOD 0) without reassociation (no fast-math) as long as no intermediate overflows or underflows.
 */
namespace ExactArithmetic
{
inline void TwoSum(const double a, const double b, double& sum, double& error)
{
    sum = a + b;
    const double bVirtual = sum - a;
    const double aVirtual = sum - bVirtual;
    error = (a - aVirtual) + (b - bVirtual);
}

// Requires |a| >= |b|, or a == 0.
inline void FastTwoSum(const double a, const double b, double& sum, double& error)
{
    sum = a + b;
    error = b - (sum - a);
}

// The rounding error of difference = a - b.
inline double TwoDiffTail(const double a, const double b, const double difference)
{
    const double bVirtual = a - difference;
    const double aVirtual = difference + bVirtual;
    return (a - aVirtual) + (bVirtual - b);
}

inline void TwoDiff(const double a, const double b, double& difference, double& error)
{
    difference = a - b;
    error = TwoDiffTail(a, b, difference);
}

#if defined(__FMA__) || (defined(_MSC_VER) && defined(__AVX2__))
// A fused multiply-add returns the product's rounding error exactly. Targets with fused multiply-add are also the
// only ones on which a compiler could fuse the separate operations of the split product below.
inline void TwoProduct(const double a, const double b, double& product, double& error)
{
    product = a * b;
    error = std::fma(a, b, -product);
}
#else
// Dekker's product splits each factor into halves of 26 and 27 bits whose products are exact.
inline void TwoProduct(const double a, const double b, double& product, double& error)
{
    const auto split = [](const double value, double& high, double& low) {
        const double scaled = value * 134217729.0; // 2^27 + 1
        high = scaled - (scaled - value);
        low = value - high;
    };
    product = a * b;
    double aHigh, aLow, bHigh, bLow;
    split(a, aHigh, aLow);
    split(b, bHigh, bLow);
    const double error1 = product - (aHigh * bHigh);
    const double error2 = error1 - (aLow * bHigh);
    const double error3 = error2 - (aHigh * bLow);
    error = (aLow * bLow) - error3;
}
#endif

// (a1 + a0) - (b1 + b0) as the expansion out[0..3]; a and b are two-term expansions, most significant term first.
inline void TwoTwoDiff(const double a1, const double a0, const double b1, const double b0, double out[4])
{
    double i, j, k, l;
    TwoDiff(a0, b0, i, out[0]);
    TwoSum(a1, i, j, k);
    TwoDiff(k, b1, l, out[1]);
    TwoSum(j, l, out[3], out[2]);
}

// h = e * b without zero terms; returns the length of h, at most 2 * length.
inline int ScaleExpansionZeroElim(const int length, const double* e, const double b, double* h)
{
    double q, hh;
    TwoProduct(e[0], b, q, hh);
    int count = 0;
    if (hh != 0.0)
    {
        h[count++] = hh;
    }
    for (int i = 1; i < length; ++i)
    {
        double product, productError, sum;
        TwoProduct(e[i], b, product, productError);
        TwoSum(q, productError, sum, hh);
        if (hh != 0.0)
        {
            h[count++] = hh;
        }
        FastTwoSum(product, sum, q, hh);
        if (hh != 0.0)
        {
            h[count++] = hh;
        }
    }
    if (q != 0.0 || count == 0)
    {
        h[count++] = q;
    }
    return count;
}

// h = e + f without zero terms; returns the length of h, at most eLength + fLength.
inline int FastExpansionSumZeroElim(const int eLength, const double* e, const int fLength, const double* f, double* h)
{
    int ei = 0;
    int fi = 0;
    // Takes the smaller-magnitude head term next.
    const auto takeE = [&]() {
        return fi == fLength || (ei < eLength && ((f[fi] > e[ei]) == (f[fi] > -e[ei])));
    };
    double q = takeE() ? e[ei++] : f[fi++];
    int count = 0;
    double sum, error;
    if (ei < eLength && fi < fLength)
    {
        FastTwoSum(takeE() ? e[ei++] : f[fi++], q, sum, error);
        q = sum;
        if (error != 0.0)
        {
            h[count++] = error;
        }
    }
    while (ei < eLength || fi < fLength)
    {
        TwoSum(q, takeE() ? e[ei++] : f[fi++], sum, error);
        q = sum;
        if (error != 0.0)
        {
            h[count++] = error;
        }
    }
    if (q != 0.0 || count == 0)
    {
        h[count++] = q;
    }
    return count;
}

// The largest term of the exact determinant of rows a, b, c: exact in sign, within a factor of two in magnitude.
inline double Determinant3x3Estimate(const VHACD::Vect3& a, const VHACD::Vect3& b, const VHACD::Vect3& c)
{
    const auto minor = [](const double p, const double q, const double r, const double s, double out[4]) {
        double pq, pqError, rs, rsError;
        TwoProduct(p, q, pq, pqError);
        TwoProduct(r, s, rs, rsError);
        TwoTwoDiff(pq, pqError, rs, rsError, out);
    };
    // det = a.z (b.x c.y - c.x b.y) + b.z (c.x a.y - a.x c.y) + c.z (a.x b.y - b.x a.y)
    double bc[4], ca[4], ab[4];
    minor(b.GetX(), c.GetY(), c.GetX(), b.GetY(), bc);
    minor(c.GetX(), a.GetY(), a.GetX(), c.GetY(), ca);
    minor(a.GetX(), b.GetY(), b.GetX(), a.GetY(), ab);
    double aTerm[8], bTerm[8], cTerm[8], abTerms[16], all[24];
    const int aLength = ScaleExpansionZeroElim(4, bc, a.GetZ(), aTerm);
    const int bLength = ScaleExpansionZeroElim(4, ca, b.GetZ(), bTerm);
    const int cLength = ScaleExpansionZeroElim(4, ab, c.GetZ(), cTerm);
    const int abLength = FastExpansionSumZeroElim(aLength, aTerm, bLength, bTerm, abTerms);
    const int allLength = FastExpansionSumZeroElim(abLength, abTerms, cLength, cTerm, all);
    return all[allLength - 1];
}
} // namespace ExactArithmetic

class HullPlane : public VHACD::Vect3
{
public:
    HullPlane(const HullPlane&) = default;

    HullPlane(const VHACD::Vect3& p,
              double w);

    HullPlane(const VHACD::Vect3& p0,
              const VHACD::Vect3& p1,
              const VHACD::Vect3& p2);

    HullPlane Scale(double s) const;

    HullPlane& operator=(const HullPlane& rhs);

    double Evalue(const VHACD::Vect3 &point) const;

    double& GetW();

private:
    double m_w;
};

HullPlane::HullPlane(const VHACD::Vect3& p,
                     double w)
    : VHACD::Vect3(p)
    , m_w(w)
{
}

HullPlane::HullPlane(const VHACD::Vect3& p0,
                     const VHACD::Vect3& p1,
                     const VHACD::Vect3& p2)
    : VHACD::Vect3((p1 - p0).Cross(p2 - p0))
    , m_w(-Dot(p0))
{
}

HullPlane HullPlane::Scale(double s) const
{
    return HullPlane(*this * s,
                     m_w * s);
}

HullPlane& HullPlane::operator=(const HullPlane& rhs)
{
    GetX() = rhs.GetX();
    GetY() = rhs.GetY();
    GetZ() = rhs.GetZ();
    m_w = rhs.m_w;
    return *this;
}

double HullPlane::Evalue(const VHACD::Vect3& point) const
{
    return Dot(point) + m_w;
}

double& HullPlane::GetW()
{
    return m_w;
}

class ConvexHullFace
{
public:
    ConvexHullFace() = default;
    double Evalue(const std::vector<VHACD::Vect3>& pointArray,
                  const VHACD::Vect3& point) const;
    HullPlane GetPlaneEquation(const std::vector<VHACD::Vect3>& pointArray,
                               bool& isValid) const;

    std::array<int, 3> m_index;
private:
    int m_mark{ 0 };           // Set when a new vertex sees the face, which is then deleted
    bool m_onBoundary{ true }; // Not yet tested against the remaining points
    // m_twin[i] indexes the face across the edge from m_index[i] to m_index[(i + 1) % 3], during construction only.
    std::array<std::size_t, 3> m_twin{ { SIZE_MAX, SIZE_MAX, SIZE_MAX } };

    friend class ConvexHull;
};

double ConvexHullFace::Evalue(const std::vector<VHACD::Vect3>& pointArray,
                              const VHACD::Vect3& point) const
{
    const VHACD::Vect3& p0 = pointArray[m_index[0]];
    const VHACD::Vect3& p1 = pointArray[m_index[1]];
    const VHACD::Vect3& p2 = pointArray[m_index[2]];

    std::array<VHACD::Vect3, 3> matrix = { p2 - p0, p1 - p0, point - p0 };
    double error;
    double det = Determinant3x3(matrix,
                                error);

    // the code use double, however the threshold for accuracy test is the machine precision of a float.
    // by changing this to a smaller number, the code should run faster since many small test will be considered valid
    // the precision must be a power of two no smaller than the machine precision of a double, (1<<48)
    // float64(1<<30) can be a good value

    // double precision	= double (1.0f) / double (1<<30);
    double precision = double(1.0) / double(1 << 24);
    double errbound = error * precision;
    if (fabs(det) > errbound)
    {
        return det;
    }

    // A floating-point difference is zero only when its operands are equal, so a zero row or column here is
    // also zero in the exact matrix below and the determinant is exactly zero. Points sharing an axis-aligned
    // voxel face make up most of the evaluations that reach this point.
    for (int i = 0; i < 3; ++i)
    {
        if (   (matrix[i][0] == double(0.0) && matrix[i][1] == double(0.0) && matrix[i][2] == double(0.0))
            || (matrix[0][i] == double(0.0) && matrix[1][i] == double(0.0) && matrix[2][i] == double(0.0)))
        {
            return double(0.0);
        }
    }

#if defined(FLT_EVAL_METHOD) && FLT_EVAL_METHOD == 0 && !defined(__FAST_MATH__)
    // Hull points are mostly voxel corners, whose coordinate differences are exact in double precision. The
    // determinant of exact differences is then summed exactly as an expansion. On 611 inputs that covers 99% of the
    // evaluations that reach this point; the rest go on to Googol arithmetic. Differences between 2^-200 and 2^200
    // keep every term of the expansion clear of overflow and underflow.
    const VHACD::Vect3* const others[3] = { &p2, &p1, &point };
    bool exactDifferences = true;
    for (int row = 0; row < 3 && exactDifferences; ++row)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            const double difference = matrix[row][axis];
            const double magnitude = fabs(difference);
            exactDifferences = exactDifferences
                               && (difference == 0.0 || (magnitude >= 0x1p-200 && magnitude <= 0x1p200))
                               && ExactArithmetic::TwoDiffTail((*others[row])[axis], p0[axis], difference) == 0.0;
        }
    }
    if (exactDifferences)
    {
        return ExactArithmetic::Determinant3x3Estimate(matrix[0], matrix[1], matrix[2]);
    }
#endif

    const VHACD::Vector3<Googol> p0g = pointArray[m_index[0]];
    const VHACD::Vector3<Googol> p1g = pointArray[m_index[1]];
    const VHACD::Vector3<Googol> p2g = pointArray[m_index[2]];
    const VHACD::Vector3<Googol> pointg = point;
    std::array<VHACD::Vector3<Googol>, 3> exactMatrix = { p2g - p0g, p1g - p0g, pointg - p0g };
    return Determinant3x3(exactMatrix);
}

HullPlane ConvexHullFace::GetPlaneEquation(const std::vector<VHACD::Vect3>& pointArray,
                                           bool& isvalid) const
{
    const VHACD::Vect3& p0 = pointArray[m_index[0]];
    const VHACD::Vect3& p1 = pointArray[m_index[1]];
    const VHACD::Vect3& p2 = pointArray[m_index[2]];
    HullPlane plane(p0, p1, p2);

    isvalid = false;
    double mag2 = plane.Dot(plane);
    if (mag2 > double(1.0e-16))
    {
        isvalid = true;
        plane = plane.Scale(double(1.0) / sqrt(mag2));
    }
    return plane;
}

// Orders points by x, then y, then z. Coordinates are finite (Compute validates its input), so this is a strict weak
// order. ConvexHull sorts its input this way, and inputs already in this order skip the sort.
inline bool LexicographicallyLess(const VHACD::Vect3& a, const VHACD::Vect3& b)
{
    for (int i = 0; i < 3; i++)
    {
        if (a[i] != b[i])
        {
            return a[i] < b[i];
        }
    }
    return false;
}

inline bool LexicographicallyLess(const VHACD::Vertex& a, const VHACD::Vertex& b)
{
    if (a.mX != b.mX)
    {
        return a.mX < b.mX;
    }
    if (a.mY != b.mY)
    {
        return a.mY < b.mY;
    }
    return a.mZ < b.mZ;
}

class ConvexHullVertex : public VHACD::Vect3
{
public:
    ConvexHullVertex() = default;
    ConvexHullVertex(const ConvexHullVertex&) = default;
    ConvexHullVertex& operator=(const ConvexHullVertex& rhs) = default;
    using VHACD::Vect3::operator=;

    int m_mark{ 0 };
};


class ConvexHullAABBTreeNode
{
    #define VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE 8
public:
    // m_count and m_indices are written before they are read, and only for leaves.
    ConvexHullAABBTreeNode() {}

    VHACD::Vect3 m_box[2];
    ConvexHullAABBTreeNode* m_left{ nullptr };
    ConvexHullAABBTreeNode* m_right{ nullptr };
    ConvexHullAABBTreeNode* m_parent{ nullptr };

    size_t m_count;
    std::array<size_t, VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE> m_indices;
};

/*
 * Storage for building convex hulls, kept by the caller so repeated builds reuse the vectors' capacity instead of
 * allocating them again. A workspace serves one build at a time on one thread, and the vertex pool and faces of a
 * hull built on it stay valid only until the next build on the same workspace.
 */
class ConvexHullWorkspace
{
    friend class ConvexHull;

    std::vector<ConvexHullVertex> m_inputPoints;
    std::vector<ConvexHullAABBTreeNode> m_tree;
    std::vector<ConvexHullFace> m_faces;
    std::vector<VHACD::Vect3> m_vertices;
    std::vector<std::size_t> m_boundaryFaces;
    std::vector<std::size_t> m_stack;
    std::vector<std::size_t> m_coneList;
    std::vector<std::size_t> m_deleteList;
    std::vector<double> m_farthestDistance;
};

class ConvexHull
{
    class ndNormalMap;

public:
    // Builds the hull in workspace; its vertex pool and faces live there until the workspace's next build.
    ConvexHull(ConvexHullWorkspace& workspace,
               const std::vector<::VHACD::Vertex>& vertexCloud,
               double distTol,
               int maxVertexCount = 0x7fffffff);
    ConvexHull(const ConvexHull&) = delete;
    ConvexHull& operator=(const ConvexHull&) = delete;
    ~ConvexHull() = default;

    const std::vector<VHACD::Vect3>& GetVertexPool() const;

    // The hull's faces, in creation order.
    const std::vector<ConvexHullFace>& GetFaces() const { return m_faces; }

private:
    void BuildHull(const std::vector<::VHACD::Vertex>& vertexCloud,
                   double distTol,
                   int maxVertexCount);

    void GetUniquePoints(std::vector<ConvexHullVertex>& points);
    int InitVertexArray(std::vector<ConvexHullVertex>& points,
                        std::vector<ConvexHullAABBTreeNode>& memoryPool);

    ConvexHullAABBTreeNode* BuildTreeOld(std::vector<ConvexHullVertex>& points,
                                         std::vector<ConvexHullAABBTreeNode>& memoryPool);
    ConvexHullAABBTreeNode* BuildTreeRecurse(ConvexHullAABBTreeNode* const parent,
                                             ConvexHullVertex* const points,
                                             int count,
                                             int baseIndex,
                                             int depth,
                                             std::vector<ConvexHullAABBTreeNode>& memoryPool) const;

    std::size_t AddFace(int i0,
                        int i1,
                        int i2);

    void CalculateConvexHull3D(ConvexHullAABBTreeNode* vertexTree,
                               std::vector<ConvexHullVertex>& points,
                               int count,
                               double distTol,
                               int maxVertexCount);

    int SupportVertex(ConvexHullAABBTreeNode** const tree,
                      const std::vector<ConvexHullVertex>& points,
                      const VHACD::Vect3& dir,
                      const bool removeEntry = true) const;
    double TetrahedrumVolume(const VHACD::Vect3& p0,
                             const VHACD::Vect3& p1,
                             const VHACD::Vect3& p2,
                             const VHACD::Vect3& p3) const;

    ConvexHullWorkspace& m_workspace;
    std::vector<ConvexHullFace>& m_faces;
    double m_diag{ 0.0 };
    std::vector<VHACD::Vect3>& m_points;
};

class ConvexHull::ndNormalMap
{
public:
    ndNormalMap();

    static const ndNormalMap& GetNormalMap();

    void TessellateTriangle(int level,
                            const VHACD::Vect3& p0,
                            const VHACD::Vect3& p1,
                            const VHACD::Vect3& p2,
                            int& count);

    std::array<VHACD::Vect3, 128> m_normal;
    int m_count{ 128 };
};

const ConvexHull::ndNormalMap& ConvexHull::ndNormalMap::GetNormalMap()
{
    static ndNormalMap normalMap;
    return normalMap;
}

void ConvexHull::ndNormalMap::TessellateTriangle(int level,
                                                 const VHACD::Vect3& p0,
                                                 const VHACD::Vect3& p1,
                                                 const VHACD::Vect3& p2,
                                                 int& count)
{
    if (level)
    {
        assert(fabs(p0.Dot(p0) - double(1.0)) < double(1.0e-4));
        assert(fabs(p1.Dot(p1) - double(1.0)) < double(1.0e-4));
        assert(fabs(p2.Dot(p2) - double(1.0)) < double(1.0e-4));
        VHACD::Vect3 p01(p0 + p1);
        VHACD::Vect3 p12(p1 + p2);
        VHACD::Vect3 p20(p2 + p0);

        p01 = p01 * (double(1.0) / p01.GetNorm());
        p12 = p12 * (double(1.0) / p12.GetNorm());
        p20 = p20 * (double(1.0) / p20.GetNorm());

        assert(fabs(p01.GetNormSquared() - double(1.0)) < double(1.0e-4));
        assert(fabs(p12.GetNormSquared() - double(1.0)) < double(1.0e-4));
        assert(fabs(p20.GetNormSquared() - double(1.0)) < double(1.0e-4));

        TessellateTriangle(level - 1, p0,  p01, p20, count);
        TessellateTriangle(level - 1, p1,  p12, p01, count);
        TessellateTriangle(level - 1, p2,  p20, p12, count);
        TessellateTriangle(level - 1, p01, p12, p20, count);
    }
    else
    {
        /*
         * This is just m_normal[index] = n.Normalized(), but due to tiny floating point errors, causes
         * different outputs, so I'm leaving it
         */
        HullPlane n(p0, p1, p2);
        n = n.Scale(double(1.0) / n.GetNorm());
        n.GetW() = double(0.0);
        int index = dBitReversal(count,
                                 int(m_normal.size()));
        m_normal[index] = n;
        count++;
        assert(count <= int(m_normal.size()));
    }
}

ConvexHull::ndNormalMap::ndNormalMap()
{
    VHACD::Vect3 p0(double( 1.0), double( 0.0), double( 0.0));
    VHACD::Vect3 p1(double(-1.0), double( 0.0), double( 0.0));
    VHACD::Vect3 p2(double( 0.0), double( 1.0), double( 0.0));
    VHACD::Vect3 p3(double( 0.0), double(-1.0), double( 0.0));
    VHACD::Vect3 p4(double( 0.0), double( 0.0), double( 1.0));
    VHACD::Vect3 p5(double( 0.0), double( 0.0), double(-1.0));

    int count = 0;
    int subdivisions = 2;
    TessellateTriangle(subdivisions, p4, p0, p2, count);
    TessellateTriangle(subdivisions, p0, p5, p2, count);
    TessellateTriangle(subdivisions, p5, p1, p2, count);
    TessellateTriangle(subdivisions, p1, p4, p2, count);
    TessellateTriangle(subdivisions, p0, p4, p3, count);
    TessellateTriangle(subdivisions, p5, p0, p3, count);
    TessellateTriangle(subdivisions, p1, p5, p3, count);
    TessellateTriangle(subdivisions, p4, p1, p3, count);
}

ConvexHull::ConvexHull(ConvexHullWorkspace& workspace,
                       const std::vector<::VHACD::Vertex>& vertexCloud,
                       double distTol,
                       int maxVertexCount)
    : m_workspace(workspace)
    , m_faces(workspace.m_faces)
    , m_points(workspace.m_vertices)
{
    m_faces.clear();
    m_points.clear();
    if (vertexCloud.size() >= 4)
    {
        BuildHull(vertexCloud,
                  distTol,
                  maxVertexCount);
    }
}

const std::vector<VHACD::Vect3>& ConvexHull::GetVertexPool() const
{
    return m_points;
}

void ConvexHull::BuildHull(const std::vector<::VHACD::Vertex>& vertexCloud,
                           double distTol,
                           int maxVertexCount)
{
    std::vector<ConvexHullVertex>& points = m_workspace.m_inputPoints;
    points.resize(vertexCloud.size());
    /*
     * treePool holds the AABB tree's nodes, root first
     * Leaf nodes hold up to 8 vertices and have null m_left and m_right, which is how SupportVertex tells them apart
     * Vertices are specified by the m_indices array and are accessed via the points array
     *
     * Nodes point at each other, so BuildTreeOld reserves every node the tree can need before building it
     */
    std::vector<ConvexHullAABBTreeNode>& treePool = m_workspace.m_tree;
    treePool.clear();
    for (size_t i = 0; i < vertexCloud.size(); ++i)
    {
        points[i] = VHACD::Vect3(vertexCloud[i]);
        points[i].m_mark = 0;
    }
    int count = InitVertexArray(points,
                                treePool);

    if (m_points.size() >= 4)
    {
        CalculateConvexHull3D(&treePool.front(),
                              points,
                              count,
                              distTol,
                              maxVertexCount);
    }
}

void ConvexHull::GetUniquePoints(std::vector<ConvexHullVertex>& points)
{
    const auto lexicographicLess = [](const ConvexHullVertex& a, const ConvexHullVertex& b) {
        return LexicographicallyLess(a, b);
    };
    const auto samePosition = [](const ConvexHullVertex& a, const ConvexHullVertex& b) {
        return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
    };
    // Voxel corners arrive sorted and distinct.
    if (!std::is_sorted(points.begin(), points.end(), lexicographicLess))
    {
        std::sort(points.begin(), points.end(), lexicographicLess);
    }
    points.erase(std::unique(points.begin(), points.end(), samePosition), points.end());
}

ConvexHullAABBTreeNode* ConvexHull::BuildTreeRecurse(ConvexHullAABBTreeNode* const parent,
                                                     ConvexHullVertex* const points,
                                                     int count,
                                                     int baseIndex,
                                                     int depth,
                                                     std::vector<ConvexHullAABBTreeNode>& memoryPool) const
{
    ConvexHullAABBTreeNode* tree = nullptr;

    assert(count);
    VHACD::Vect3 minP( double(1.0e15));
    VHACD::Vect3 maxP(-double(1.0e15));
    if (count <= VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE)
    {
        assert(memoryPool.size() < memoryPool.capacity());
        ConvexHullAABBTreeNode& clump = memoryPool.emplace_back();

        clump.m_count = count;
        // x and y share one SSE2 register; _mm_min_pd(value, bound) keeps the bound on ties, as std::min(bound,
        // value) does, so the bounds are those CWiseMin and CWiseMax produced.
        __m128d lowXY = _mm_set_pd(minP[1], minP[0]);
        __m128d highXY = _mm_set_pd(maxP[1], maxP[0]);
        double lowZ = minP[2];
        double highZ = maxP[2];
        for (int i = 0; i < count; ++i)
        {
            clump.m_indices[i] = i + baseIndex;

            const VHACD::Vect3& p = points[i];
            const __m128d xy = _mm_loadu_pd(&p[0]);
            lowXY = _mm_min_pd(xy, lowXY);
            highXY = _mm_max_pd(xy, highXY);
            lowZ = std::min(lowZ, p[2]);
            highZ = std::max(highZ, p[2]);
        }
        double low[2], high[2];
        _mm_storeu_pd(low, lowXY);
        _mm_storeu_pd(high, highXY);
        minP = VHACD::Vect3(low[0], low[1], lowZ);
        maxP = VHACD::Vect3(high[0], high[1], highZ);

        clump.m_left = nullptr;
        clump.m_right = nullptr;
        tree = &clump;
    }
    else
    {
        // x and y run as the two lanes of SSE2 registers and z as scalars. Each axis still accumulates in point
        // order, so the sums and bounds are bit identical to per-axis scalar code.
        double low[3] = { minP[0], minP[1], minP[2] };
        double high[3] = { maxP[0], maxP[1], maxP[2] };
        double sum[3] = { 0.0, 0.0, 0.0 };
        double sumSquares[3] = { 0.0, 0.0, 0.0 };
        __m128d lowXY = _mm_loadu_pd(low);
        __m128d highXY = _mm_loadu_pd(high);
        __m128d sumXY = _mm_setzero_pd();
        __m128d sumSquaresXY = _mm_setzero_pd();
        for (int i = 0; i < count; ++i)
        {
            const VHACD::Vect3& p = points[i];
            const __m128d xy = _mm_loadu_pd(&p[0]);
            lowXY = _mm_min_pd(xy, lowXY);
            highXY = _mm_max_pd(xy, highXY);
            sumXY = _mm_add_pd(sumXY, xy);
            sumSquaresXY = _mm_add_pd(sumSquaresXY, _mm_mul_pd(xy, xy));
            const double z = p[2];
            low[2] = std::min(low[2], z);
            high[2] = std::max(high[2], z);
            sum[2] += z;
            sumSquares[2] += z * z;
        }
        _mm_storeu_pd(low, lowXY);
        _mm_storeu_pd(high, highXY);
        {
            double xySum[2], xySumSquares[2];
            _mm_storeu_pd(xySum, sumXY);
            _mm_storeu_pd(xySumSquares, sumSquaresXY);
            sum[0] = xySum[0];
            sum[1] = xySum[1];
            sumSquares[0] = xySumSquares[0];
            sumSquares[1] = xySumSquares[1];
        }
        minP = VHACD::Vect3(low[0], low[1], low[2]);
        maxP = VHACD::Vect3(high[0], high[1], high[2]);
        VHACD::Vect3 median(sum[0], sum[1], sum[2]);
        VHACD::Vect3 varian(sumSquares[0], sumSquares[1], sumSquares[2]);

        varian = varian * double(count) - median.CWiseMul(median);
        int index = 0;
        double maxVarian = double(-1.0e10);
        for (int i = 0; i < 3; ++i)
        {
            if (varian[i] > maxVarian)
            {
                index = i;
                maxVarian = varian[i];
            }
        }
        VHACD::Vect3 center(median * (double(1.0) / double(count)));

        double test = center[index];

        int i0 = 0;
        int i1 = count - 1;
        do
        {
            for (; i0 <= i1; i0++)
            {
                double val = points[i0][index];
                if (val > test)
                {
                    break;
                }
            }

            for (; i1 >= i0; i1--)
            {
                double val = points[i1][index];
                if (val < test)
                {
                    break;
                }
            }

            if (i0 < i1)
            {
                std::swap(points[i0],
                          points[i1]);
                i0++;
                i1--;
            }
        } while (i0 <= i1);

        // A mean split can peel off a few points per level, so its depth is unbounded in general. From
        // BalancedSplitDepth on, a side holding under a third falls back to the middle index. Each level then
        // keeps at most two thirds of its points plus one, so any int point count finishes within 77 levels,
        // which bounds the recursion and the SupportVertex stack (one entry per level plus one). Voxel and
        // hull point sets reach 17 levels on the engine corpus, so their trees are unchanged.
        static constexpr int BalancedSplitDepth = 24;
        if (i0 == 0 || i0 >= (count - 1) ||
            (depth >= BalancedSplitDepth && (i0 < count / 3 || count - i0 < count / 3)))
        {
            i0 = count / 2;
        }

        assert(memoryPool.size() < memoryPool.capacity());
        tree = &memoryPool.emplace_back();

        assert(i0);
        assert(count - i0);

        tree->m_left = BuildTreeRecurse(tree,
                                        points,
                                        i0,
                                        baseIndex,
                                        depth + 1,
                                        memoryPool);
        tree->m_right = BuildTreeRecurse(tree,
                                         &points[i0],
                                         count - i0,
                                         i0 + baseIndex,
                                         depth + 1,
                                         memoryPool);
    }

    assert(tree);
    tree->m_parent = parent;
    /*
     * WARNING: Changing the compiler conversion of 1.0e-3f changes the results of the convex decomposition
     * Inflate the tree's bounding box slightly
     */
    tree->m_box[0] = minP - VHACD::Vect3(double(1.0e-3f));
    tree->m_box[1] = maxP + VHACD::Vect3(double(1.0e-3f));
    return tree;
}

ConvexHullAABBTreeNode* ConvexHull::BuildTreeOld(std::vector<ConvexHullVertex>& points,
                                                 std::vector<ConvexHullAABBTreeNode>& memoryPool)
{
    GetUniquePoints(points);
    int count = int(points.size());
    if (count < 4)
    {
        return nullptr;
    }
    // Both sides of every split are non-empty, so the tree has at most count leaves and one inner node fewer.
    // Nodes never move once that many are reserved.
    memoryPool.reserve(2 * std::size_t(count) - 1);
    return BuildTreeRecurse(nullptr,
                            points.data(),
                            count,
                            0,
                            0,
                            memoryPool);
}

int ConvexHull::SupportVertex(ConvexHullAABBTreeNode** const treePointer,
                              const std::vector<ConvexHullVertex>& points,
                              const VHACD::Vect3& dirPlane,
                              const bool removeEntry) const
{
#define VHACD_STACK_DEPTH_3D 96 // BuildTreeRecurse bounds the tree to 77 levels
    double aabbProjection[VHACD_STACK_DEPTH_3D];
    ConvexHullAABBTreeNode* stackPool[VHACD_STACK_DEPTH_3D];

    VHACD::Vect3 dir(dirPlane);

    int index = -1;
    int stack = 1;
    stackPool[0] = *treePointer;
    aabbProjection[0] = double(1.0e20);
    double maxProj = double(-1.0e20);
    int ix = (dir[0] > double(0.0)) ? 1 : 0;
    int iy = (dir[1] > double(0.0)) ? 1 : 0;
    int iz = (dir[2] > double(0.0)) ? 1 : 0;
    while (stack)
    {
        stack--;
        double boxSupportValue = aabbProjection[stack];
        if (boxSupportValue > maxProj)
        {
            ConvexHullAABBTreeNode* me = stackPool[stack];

            /*
             * If the node is not a leaf node...
             */
            if (me->m_left && me->m_right)
            {
                const VHACD::Vect3 leftSupportPoint(me->m_left->m_box[ix].GetX(),
                                                    me->m_left->m_box[iy].GetY(),
                                                    me->m_left->m_box[iz].GetZ());
                double leftSupportDist = leftSupportPoint.Dot(dir);

                const VHACD::Vect3 rightSupportPoint(me->m_right->m_box[ix].GetX(),
                                                     me->m_right->m_box[iy].GetY(),
                                                     me->m_right->m_box[iz].GetZ());
                double rightSupportDist = rightSupportPoint.Dot(dir);

                /*
                 * ...push the shorter side first
                 * So we can explore the tree in the larger side first
                 */
                if (rightSupportDist >= leftSupportDist)
                {
                    aabbProjection[stack] = leftSupportDist;
                    stackPool[stack] = me->m_left;
                    stack++;
                    assert(stack < VHACD_STACK_DEPTH_3D);
                    aabbProjection[stack] = rightSupportDist;
                    stackPool[stack] = me->m_right;
                    stack++;
                    assert(stack < VHACD_STACK_DEPTH_3D);
                }
                else
                {
                    aabbProjection[stack] = rightSupportDist;
                    stackPool[stack] = me->m_right;
                    stack++;
                    assert(stack < VHACD_STACK_DEPTH_3D);
                    aabbProjection[stack] = leftSupportDist;
                    stackPool[stack] = me->m_left;
                    stack++;
                    assert(stack < VHACD_STACK_DEPTH_3D);
                }
            }
            /*
             * If it is a node...
             */
            else
            {
                ConvexHullAABBTreeNode* cluster = me;
                for (size_t i = 0; i < cluster->m_count; ++i)
                {
                    const ConvexHullVertex& p = points[cluster->m_indices[i]];
                    assert(p.GetX() >= cluster->m_box[0].GetX());
                    assert(p.GetX() <= cluster->m_box[1].GetX());
                    assert(p.GetY() >= cluster->m_box[0].GetY());
                    assert(p.GetY() <= cluster->m_box[1].GetY());
                    assert(p.GetZ() >= cluster->m_box[0].GetZ());
                    assert(p.GetZ() <= cluster->m_box[1].GetZ());
                    if (!p.m_mark)
                    {
                        //assert(p.m_w == double(0.0f));
                        double dist = p.Dot(dir);
                        if (dist > maxProj)
                        {
                            maxProj = dist;
                            index = cluster->m_indices[i];
                        }
                    }
                    else if (removeEntry)
                    {
                        cluster->m_indices[i] = cluster->m_indices[cluster->m_count - 1];
                        cluster->m_count = cluster->m_count - 1;
                        i--;
                    }
                }

                if (cluster->m_count == 0)
                {
                    ConvexHullAABBTreeNode* const parent = cluster->m_parent;
                    if (parent)
                    {
                        ConvexHullAABBTreeNode* const sibling = (parent->m_left != cluster) ? parent->m_left : parent->m_right;
                        assert(sibling != cluster);
                        ConvexHullAABBTreeNode* const grandParent = parent->m_parent;
                        if (grandParent)
                        {
                            sibling->m_parent = grandParent;
                            if (grandParent->m_right == parent)
                            {
                                grandParent->m_right = sibling;
                            }
                            else
                            {
                                grandParent->m_left = sibling;
                            }
                        }
                        else
                        {
                            sibling->m_parent = nullptr;
                            *treePointer = sibling;
                        }
                    }
                }
            }
        }
    }

    assert(index != -1);
    return index;
}

double ConvexHull::TetrahedrumVolume(const VHACD::Vect3& p0,
                                     const VHACD::Vect3& p1,
                                     const VHACD::Vect3& p2,
                                     const VHACD::Vect3& p3) const
{
    const VHACD::Vect3 p1p0(p1 - p0);
    const VHACD::Vect3 p2p0(p2 - p0);
    const VHACD::Vect3 p3p0(p3 - p0);
    return p3p0.Dot(p1p0.Cross(p2p0));
}

int ConvexHull::InitVertexArray(std::vector<ConvexHullVertex>& points,
                                std::vector<ConvexHullAABBTreeNode>& memoryPool)
{
    ConvexHullAABBTreeNode* tree = BuildTreeOld(points,
                                                memoryPool);
    int count = int(points.size());
    if (count < 4)
    {
        m_points.resize(0);
        return 0;
    }

    m_points.resize(count);

    VHACD::Vect3 boxSize(tree->m_box[1] - tree->m_box[0]);
    m_diag = boxSize.GetNorm();
    const ndNormalMap& normalMap = ndNormalMap::GetNormalMap();

    int index0 = SupportVertex(&tree,
                               points,
                               normalMap.m_normal[0]);
    m_points[0] = points[index0];
    points[index0].m_mark = 1;

    bool validTetrahedrum = false;
    VHACD::Vect3 e1(double(0.0));
    for (int i = 1; i < normalMap.m_count; ++i)
    {
        int index = SupportVertex(&tree,
                                  points,
                                  normalMap.m_normal[i]);
        assert(index >= 0);

        e1 = points[index] - m_points[0];
        double error2 = e1.GetNormSquared();
        if (error2 > (double(1.0e-4) * m_diag * m_diag))
        {
            m_points[1] = points[index];
            points[index].m_mark = 1;
            validTetrahedrum = true;
            break;
        }
    }
    if (!validTetrahedrum)
    {
        // A cloud too flat for its extent has no hull to build. That is a fact about the points, not a
        // broken invariant: callers fall back to the box around them.
        m_points.resize(0);
        return count;
    }

    validTetrahedrum = false;
    VHACD::Vect3 e2(double(0.0));
    VHACD::Vect3 normal(double(0.0));
    for (int i = 2; i < normalMap.m_count; ++i)
    {
        int index = SupportVertex(&tree,
                                  points,
                                  normalMap.m_normal[i]);
        assert(index >= 0);
        e2 = points[index] - m_points[0];
        normal = e1.Cross(e2);
        double error2 = normal.GetNorm();
        if (error2 > (double(1.0e-4) * m_diag * m_diag))
        {
            m_points[2] = points[index];
            points[index].m_mark = 1;
            validTetrahedrum = true;
            break;
        }
    }

    if (!validTetrahedrum)
    {
        // A cloud too flat for its extent has no hull to build. That is a fact about the points, not a
        // broken invariant: callers fall back to the box around them.
        m_points.resize(0);
        return count;
    }

    // find the largest possible tetrahedron
    validTetrahedrum = false;
    VHACD::Vect3 e3(double(0.0));

    index0 = SupportVertex(&tree,
                           points,
                           normal);
    e3 = points[index0] - m_points[0];
    double err2 = normal.Dot(e3);
    if (fabs(err2) > (double(1.0e-6) * m_diag * m_diag))
    {
        // we found a valid tetrahedral, about and start build the hull by adding the rest of the points
        m_points[3] = points[index0];
        points[index0].m_mark = 1;
        validTetrahedrum = true;
    }
    if (!validTetrahedrum)
    {
        VHACD::Vect3 n(-normal);
        int index = SupportVertex(&tree,
                                  points,
                                  n);
        e3 = points[index] - m_points[0];
        double error2 = normal.Dot(e3);
        if (fabs(error2) > (double(1.0e-6) * m_diag * m_diag))
        {
            // we found a valid tetrahedral, about and start build the hull by adding the rest of the points
            m_points[3] = points[index];
            points[index].m_mark = 1;
            validTetrahedrum = true;
        }
    }
    if (!validTetrahedrum)
    {
        for (int i = 3; i < normalMap.m_count; ++i)
        {
            int index = SupportVertex(&tree,
                                      points,
                                      normalMap.m_normal[i]);
            assert(index >= 0);

            //make sure the volume of the fist tetrahedral is no negative
            e3 = points[index] - m_points[0];
            double error2 = normal.Dot(e3);
            if (fabs(error2) > (double(1.0e-6) * m_diag * m_diag))
            {
                // we found a valid tetrahedral, about and start build the hull by adding the rest of the points
                m_points[3] = points[index];
                points[index].m_mark = 1;
                validTetrahedrum = true;
                break;
            }
        }
    }
    if (!validTetrahedrum)
    {
        // the points do not form a convex hull
        m_points.resize(0);
        return count;
    }

    m_points.resize(4);
    double volume = TetrahedrumVolume(m_points[0],
                                      m_points[1],
                                      m_points[2],
                                      m_points[3]);
    if (volume > double(0.0))
    {
        std::swap(m_points[2],
                  m_points[3]);
    }
    assert(TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < double(0.0));
    return count;
}

std::size_t ConvexHull::AddFace(int i0,
                                int i1,
                                int i2)
{
    ConvexHullFace& face = m_faces.emplace_back();
    face.m_index[0] = i0;
    face.m_index[1] = i1;
    face.m_index[2] = i2;
    return m_faces.size() - 1;
}

void ConvexHull::CalculateConvexHull3D(ConvexHullAABBTreeNode* vertexTree,
                                       std::vector<ConvexHullVertex>& points,
                                       int count,
                                       double distTol,
                                       int maxVertexCount)
{
    distTol = fabs(distTol) * m_diag;
    // Builds create one to three faces per input point, so this holds almost all of them without reallocation.
    const std::size_t expectedFaces = std::min<std::size_t>(4 * std::size_t(count), 1024);
    m_faces.reserve(expectedFaces);
    const std::size_t f0 = AddFace(0, 1, 2);
    const std::size_t f1 = AddFace(0, 2, 3);
    const std::size_t f2 = AddFace(2, 1, 3);
    const std::size_t f3 = AddFace(1, 0, 3);

    m_faces[f0].m_twin = { f3, f2, f1 };
    m_faces[f1].m_twin = { f0, f2, f3 };
    m_faces[f2].m_twin = { f0, f3, f1 };
    m_faces[f3].m_twin = { f0, f1, f2 };

    /*
     * Faces wait here to be tested against the remaining points and are taken oldest first, except that the
     * initial four go in reverse. A face leaves when it is tested and kept, or deleted; its entry stays behind
     * for boundaryHead to skip.
     */
    std::vector<std::size_t>& boundaryFaces = m_workspace.m_boundaryFaces;
    boundaryFaces.reserve(expectedFaces);
    boundaryFaces.assign({ f3, f2, f1, f0 });
    std::size_t boundaryHead = 0;
    std::size_t boundaryCount = boundaryFaces.size();

    m_points.resize(count);

    /*
     * A hull of every point does not depend on the order faces are taken in, so those take the oldest face.
     * A hull limited to maxVertexCount vertices keeps only the points added before the limit, so it takes the
     * face whose farthest remaining point is farthest: each added vertex then restores the largest missing
     * piece of the full hull, and a hull at the limit misses less of it.
     */
    const bool limitedVertexCount = maxVertexCount < count;
    // Farthest remaining point distance per face index, DBL_MAX until evaluated and -DBL_MAX for a face without a
    // valid plane. Points only leave the remaining set, so a stored distance never understates the current one.
    std::vector<double>& farthestDistance = m_workspace.m_farthestDistance;
    farthestDistance.clear();
    const auto measureFarthest = [&](const std::size_t face) {
        bool valid;
        const HullPlane plane(m_faces[face].GetPlaneEquation(m_points, valid));
        return valid ? plane.Evalue(points[SupportVertex(&vertexTree, points, plane)]) : -DBL_MAX;
    };

    count -= 4;
    maxVertexCount -= 4;
    int currentIndex = 4;

    std::vector<std::size_t>& stack = m_workspace.m_stack;
    std::vector<std::size_t>& coneList = m_workspace.m_coneList;
    std::vector<std::size_t>& deleteList = m_workspace.m_deleteList;
    stack.clear();

    while (boundaryCount && count && (maxVertexCount > 0))
    {
        // Every face still waiting sits at or after boundaryHead.
        while (!m_faces[boundaryFaces[boundaryHead]].m_onBoundary)
        {
            boundaryHead++;
            assert(boundaryHead < boundaryFaces.size());
        }
        std::size_t faceNode = boundaryFaces[boundaryHead];
        if (limitedVertexCount)
        {
            farthestDistance.resize(m_faces.size(), DBL_MAX);
            for (;;)
            {
                // Ties go to the oldest face.
                std::size_t leader = SIZE_MAX;
                for (std::size_t entry = boundaryHead; entry < boundaryFaces.size(); ++entry)
                {
                    const std::size_t face = boundaryFaces[entry];
                    if (!m_faces[face].m_onBoundary)
                    {
                        continue;
                    }
                    if (farthestDistance[face] == DBL_MAX)
                    {
                        farthestDistance[face] = measureFarthest(face);
                    }
                    if (leader == SIZE_MAX || farthestDistance[face] > farthestDistance[leader])
                    {
                        leader = face;
                    }
                }
                assert(leader != SIZE_MAX);
                // The leader's distance may be stale; it keeps the lead only if the current one still leads.
                const double current = measureFarthest(leader);
                if (current < farthestDistance[leader])
                {
                    farthestDistance[leader] = current;
                    continue;
                }
                faceNode = leader;
                break;
            }
            if (farthestDistance[faceNode] < distTol)
            {
                // No waiting face has a point far enough outside it.
                break;
            }
        }

        bool isvalid;
        HullPlane planeEquation(m_faces[faceNode].GetPlaneEquation(m_points, isvalid));

        int index = 0;
        double dist = 0;
        VHACD::Vect3 p;
        if (isvalid)
        {
            index = SupportVertex(&vertexTree,
                                  points,
                                  planeEquation);
            p = points[index];
            dist = planeEquation.Evalue(p);
        }

        if (   isvalid
            && (dist >= distTol)
            && (m_faces[faceNode].Evalue(m_points, p) < double(0.0)))
        {
            // faceNode sees p, as tested above, so the flood fill starts from its neighbors.
            deleteList.clear();
            deleteList.push_back(faceNode);
            m_faces[faceNode].m_mark = 1;
            for (const std::size_t twinNode : m_faces[faceNode].m_twin)
            {
                assert(twinNode < m_faces.size());
                if (!m_faces[twinNode].m_mark)
                {
                    stack.push_back(twinNode);
                }
            }
            while (stack.size())
            {
                const std::size_t node1 = stack.back();
                ConvexHullFace& face1 = m_faces[node1];

                stack.pop_back();

                if (!face1.m_mark && (face1.Evalue(m_points, p) < double(0.0)))
                {
                    #ifdef _DEBUG
                    for (const std::size_t node : deleteList)
                    {
                        assert(node != node1);
                    }
                    #endif

                    deleteList.push_back(node1);
                    face1.m_mark = 1;
                    for (const std::size_t twinNode : face1.m_twin)
                    {
                        assert(twinNode < m_faces.size());
                        if (!m_faces[twinNode].m_mark)
                        {
                            stack.push_back(twinNode);
                        }
                    }
                }
            }

            m_points[currentIndex] = points[index];
            points[index].m_mark = 1;

            // AddFace can reallocate m_faces, so no face reference is held across it.
            coneList.clear();
            for (const std::size_t node1 : deleteList)
            {
                assert(m_faces[node1].m_mark == 1);
                for (std::size_t j0 = 0; j0 < 3; ++j0)
                {
                    const std::size_t twinNode = m_faces[node1].m_twin[j0];
                    if (!m_faces[twinNode].m_mark)
                    {
                        std::size_t j1 = (j0 == 2) ? 0 : j0 + 1;
                        const std::size_t newNode = AddFace(currentIndex,
                                                            m_faces[node1].m_index[j0],
                                                            m_faces[node1].m_index[j1]);
                        boundaryFaces.push_back(newNode);
                        boundaryCount++;

                        m_faces[newNode].m_twin[1] = twinNode;
                        for (std::size_t& twinOfTwin : m_faces[twinNode].m_twin)
                        {
                            if (twinOfTwin == node1)
                            {
                                twinOfTwin = newNode;
                            }
                        }
                        coneList.push_back(newNode);
                    }
                }
            }

            // An exterior point sees at least one face but never all of them, so the horizon is non-empty.
            assert(!coneList.empty());
            for (std::size_t i = 0; i + 1 < coneList.size(); ++i)
            {
                const std::size_t nodeA = coneList[i];
                ConvexHullFace& faceA = m_faces[nodeA];
                assert(faceA.m_mark == 0);
                for (std::size_t j = i + 1; j < coneList.size(); j++)
                {
                    const std::size_t nodeB = coneList[j];
                    ConvexHullFace& faceB = m_faces[nodeB];
                    assert(faceB.m_mark == 0);
                    if (faceA.m_index[2] == faceB.m_index[1])
                    {
                        faceA.m_twin[2] = nodeB;
                        faceB.m_twin[0] = nodeA;
                        break;
                    }
                }

                for (std::size_t j = i + 1; j < coneList.size(); j++)
                {
                    const std::size_t nodeB = coneList[j];
                    ConvexHullFace& faceB = m_faces[nodeB];
                    assert(faceB.m_mark == 0);
                    if (faceA.m_index[1] == faceB.m_index[2])
                    {
                        faceA.m_twin[0] = nodeB;
                        faceB.m_twin[2] = nodeA;
                        break;
                    }
                }
            }

            for (const std::size_t node : deleteList)
            {
                ConvexHullFace& deleted = m_faces[node];
                if (deleted.m_onBoundary)
                {
                    deleted.m_onBoundary = false;
                    boundaryCount--;
                }
            }

            maxVertexCount--;
            currentIndex++;
            count--;
        }
        else
        {
            m_faces[faceNode].m_onBoundary = false;
            boundaryCount--;
        }
    }
    m_points.resize(currentIndex);

    // Deleted faces stay in m_faces, marked, until here. The survivors keep their creation order.
    m_faces.erase(std::remove_if(m_faces.begin(),
                                 m_faces.end(),
                                 [](const ConvexHullFace& face) { return face.m_mark != 0; }),
                  m_faces.end());
}

//***********************************************************************************************
// End of ConvexHull generation code by Julio Jerez <jerezjulio0@gmail.com>
//***********************************************************************************************

/*
 * A wrapper class for 3 10 bit integers packed into a 32 bit integer.
 * X is bits 29-20, Y is 19-10, and Z is 9-0, so no grid may exceed 1021 voxels on an axis; ChooseGrid
 * settles that before a grid is built.
 */
class Voxel
{
    /*
     * Specify all of them for consistency
     */
    static constexpr int VoxelBitsZStart =  0;
    static constexpr int VoxelBitsYStart = 10;
    static constexpr int VoxelBitsXStart = 20;
    static constexpr int VoxelBitMask = 0x03FF; // bits 0 through 9 inclusive
public:
    Voxel(uint32_t x,
          uint32_t y,
          uint32_t z);

    VHACD::Vector3<uint32_t> GetVoxel() const;

    uint32_t GetX() const;
    uint32_t GetY() const;
    uint32_t GetZ() const;

private:
    uint32_t m_voxel{ 0 };
};

Voxel::Voxel(uint32_t x,
             uint32_t y,
             uint32_t z)
    : m_voxel((x << VoxelBitsXStart) | (y << VoxelBitsYStart) | (z << VoxelBitsZStart))
{
    assert(x < 1024 && "Voxel constructed with X outside of range");
    assert(y < 1024 && "Voxel constructed with Y outside of range");
    assert(z < 1024 && "Voxel constructed with Z outside of range");
}

VHACD::Vector3<uint32_t> Voxel::GetVoxel() const
{
    return VHACD::Vector3<uint32_t>(GetX(), GetY(), GetZ());
}

uint32_t Voxel::GetX() const
{
    return (m_voxel >> VoxelBitsXStart) & VoxelBitMask;
}

uint32_t Voxel::GetY() const
{
    return (m_voxel >> VoxelBitsYStart) & VoxelBitMask;
}

uint32_t Voxel::GetZ() const
{
    return (m_voxel >> VoxelBitsZStart) & VoxelBitMask;
}

struct SimpleMesh
{
    std::vector<VHACD::Vertex> m_vertices;
    std::vector<VHACD::Triangle> m_indices;
};

/*======================== 0-tests ========================*/
inline bool IntersectRayAABB(const VHACD::Vect3& start,
                             const VHACD::Vect3& dir,
                             const VHACD::BoundsAABB& bounds,
                             double& t)
{
    //! calculate candidate plane on each axis
    bool inside = true;
    VHACD::Vect3 ta(double(-1.0));

    //! use unrolled loops
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (start[i] < bounds.GetMin()[i])
        {
            if (dir[i] != double(0.0))
                ta[i] = (bounds.GetMin()[i] - start[i]) / dir[i];
            inside = false;
        }
        else if (start[i] > bounds.GetMax()[i])
        {
            if (dir[i] != double(0.0))
                ta[i] = (bounds.GetMax()[i] - start[i]) / dir[i];
            inside = false;
        }
    }

    //! if point inside all planes
    if (inside)
    {
        t = double(0.0);
        return true;
    }

    //! we now have t values for each of possible intersection planes
    //! find the maximum to get the intersection point
    uint32_t taxis;
    double tmax = ta.MaxCoeff(taxis);

    if (tmax < double(0.0))
        return false;

    //! check that the intersection point lies on the plane we picked
    //! we don't test the axis of closest intersection for precision reasons

    //! no eps for now
    double eps = double(0.0);

    VHACD::Vect3 hit = start + dir * tmax;

    if ((   hit.GetX() < bounds.GetMin().GetX() - eps
         || hit.GetX() > bounds.GetMax().GetX() + eps)
        && taxis != 0)
        return false;
    if ((   hit.GetY() < bounds.GetMin().GetY() - eps
         || hit.GetY() > bounds.GetMax().GetY() + eps)
        && taxis != 1)
        return false;
    if ((   hit.GetZ() < bounds.GetMin().GetZ() - eps
         || hit.GetZ() > bounds.GetMax().GetZ() + eps)
        && taxis != 2)
        return false;

    //! output results
    t = tmax;

    return true;
}

// Moller and Trumbore's method
inline bool IntersectRayTriTwoSided(const VHACD::Vect3& p,
                                    const VHACD::Vect3& dir,
                                    const VHACD::Vect3& a,
                                    const VHACD::Vect3& b,
                                    const VHACD::Vect3& c,
                                    double& t,
                                    double& u,
                                    double& v,
                                    double& w,
                                    double& sign,
                                    VHACD::Vect3* normal)
{
    VHACD::Vect3 ab = b - a;
    VHACD::Vect3 ac = c - a;
    VHACD::Vect3 n = ab.Cross(ac);

    double d = -dir.Dot(n);
    double ood = double(1.0) / d; // No need to check for division by zero here as infinity arithmetic will save us...
    VHACD::Vect3 ap = p - a;

    t = ap.Dot(n) * ood;
    if (t < double(0.0))
    {
        return false;
    }

    VHACD::Vect3 e = -dir.Cross(ap);
    v = ac.Dot(e) * ood;
    if (v < double(0.0) || v > double(1.0)) // ...here...
    {
        return false;
    }
    w = -ab.Dot(e) * ood;
    if (w < double(0.0) || v + w > double(1.0)) // ...and here
    {
        return false;
    }

    u = double(1.0) - v - w;
    if (normal)
    {
        *normal = n;
    }

    sign = d;

    return true;
}

// RTCD 5.1.5, page 142
inline VHACD::Vect3 ClosestPointOnTriangle(const VHACD::Vect3& a,
                                           const VHACD::Vect3& b,
                                           const VHACD::Vect3& c,
                                           const VHACD::Vect3& p,
                                           double& v,
                                           double& w)
{
    VHACD::Vect3 ab = b - a;
    VHACD::Vect3 ac = c - a;
    VHACD::Vect3 ap = p - a;

    double d1 = ab.Dot(ap);
    double d2 = ac.Dot(ap);
    if (   d1 <= double(0.0)
        && d2 <= double(0.0))
    {
        v = double(0.0);
        w = double(0.0);
        return a;
    }

    VHACD::Vect3 bp = p - b;
    double d3 = ab.Dot(bp);
    double d4 = ac.Dot(bp);
    if (   d3 >= double(0.0)
        && d4 <= d3)
    {
        v = double(1.0);
        w = double(0.0);
        return b;
    }

    double vc = d1 * d4 - d3 * d2;
    if (   vc <= double(0.0)
        && d1 >= double(0.0)
        && d3 <= double(0.0))
    {
        v = d1 / (d1 - d3);
        w = double(0.0);
        return a + v * ab;
    }

    VHACD::Vect3 cp = p - c;
    double d5 = ab.Dot(cp);
    double d6 = ac.Dot(cp);
    if (d6 >= double(0.0) && d5 <= d6)
    {
        v = double(0.0);
        w = double(1.0);
        return c;
    }

    double vb = d5 * d2 - d1 * d6;
    if (   vb <= double(0.0)
        && d2 >= double(0.0)
        && d6 <= double(0.0))
    {
        v = double(0.0);
        w = d2 / (d2 - d6);
        return a + w * ac;
    }

    double va = d3 * d6 - d5 * d4;
    if (   va <= double(0.0)
        && (d4 - d3) >= double(0.0)
        && (d5 - d6) >= double(0.0))
    {
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        v = double(1.0) - w;
        return b + w * (c - b);
    }

    double denom = double(1.0) / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    return a + ab * v + ac * w;
}

class AABBTree
{
public:
    AABBTree() = default;
    AABBTree& operator=(AABBTree&&) = default;

    AABBTree(const std::vector<VHACD::Vertex>& vertices,
             const std::vector<VHACD::Triangle>& indices);

    bool TraceRay(const VHACD::Vect3& start,
                  const VHACD::Vect3& dir,
                  uint32_t& insideCount,
                  uint32_t& outsideCount) const;

    bool TraceRay(const VHACD::Vect3& start,
                  const VHACD::Vect3& dir,
                  double& outT,
                  double& u,
                  double& v,
                  double& w,
                  double& faceSign,
                  uint32_t& faceIndex) const;


    bool GetClosestPointWithinDistance(const VHACD::Vect3& point,
                                       double maxDistance,
                                       VHACD::Vect3& closestPoint) const;

private:
    struct Node
    {
        union
        {
            uint32_t m_children;
            uint32_t m_numFaces{ 0 };
        };

        uint32_t* m_faces{ nullptr };
        VHACD::BoundsAABB m_extents;
    };

    struct FaceSorter
    {
        FaceSorter(const std::vector<VHACD::Vertex>& positions,
                   const std::vector<VHACD::Triangle>& indices,
                   uint32_t axis);

        bool operator()(uint32_t lhs, uint32_t rhs) const;

        double GetCentroid(uint32_t face) const;

        const std::vector<VHACD::Vertex>& m_vertices;
        const std::vector<VHACD::Triangle>& m_indices;
        uint32_t m_axis;
    };

    // partition the objects and return the number of objects in the lower partition
    uint32_t PartitionMedian(Node& n,
                             uint32_t* faces,
                             uint32_t numFaces);

    void Build();

    void BuildRecursive(uint32_t nodeIndex,
                        uint32_t* faces,
                        uint32_t numFaces);

    void TraceRecursive(uint32_t nodeIndex,
                        const VHACD::Vect3& start,
                        const VHACD::Vect3& dir,
                        double& outT,
                        double& u,
                        double& v,
                        double& w,
                        double& faceSign,
                        uint32_t& faceIndex) const;


    bool GetClosestPointWithinDistance(const VHACD::Vect3& point,
                                       const double maxDis,
                                       double& dis,
                                       double& v,
                                       double& w,
                                       uint32_t& faceIndex,
                                       VHACD::Vect3& closest) const;

    void GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
                                                  const VHACD::Vect3& point,
                                                  double& outDisSq,
                                                  double& outV,
                                                  double& outW,
                                                  uint32_t& outFaceIndex,
                                                  VHACD::Vect3& closest) const;

    VHACD::BoundsAABB CalculateFaceBounds(uint32_t* faces,
                                          uint32_t numFaces);

    // track the next free node
    uint32_t m_freeNode;

    const std::vector<VHACD::Vertex>* m_vertices{ nullptr };
    const std::vector<VHACD::Triangle>* m_indices{ nullptr };

    std::vector<uint32_t> m_faces;
    std::vector<Node> m_nodes;


};

AABBTree::FaceSorter::FaceSorter(const std::vector<VHACD::Vertex>& positions,
                                 const std::vector<VHACD::Triangle>& indices,
                                 uint32_t axis)
    : m_vertices(positions)
    , m_indices(indices)
    , m_axis(axis)
{
}

inline bool AABBTree::FaceSorter::operator()(uint32_t lhs,
                                             uint32_t rhs) const
{
    double a = GetCentroid(lhs);
    double b = GetCentroid(rhs);

    if (a == b)
    {
        return lhs < rhs;
    }
    else
    {
        return a < b;
    }
}

inline double AABBTree::FaceSorter::GetCentroid(uint32_t face) const
{
    const VHACD::Vect3& a = m_vertices[m_indices[face].mI0];
    const VHACD::Vect3& b = m_vertices[m_indices[face].mI1];
    const VHACD::Vect3& c = m_vertices[m_indices[face].mI2];

    return (a[m_axis] + b[m_axis] + c[m_axis]) / double(3.0);
}

AABBTree::AABBTree(const std::vector<VHACD::Vertex>& vertices,
                   const std::vector<VHACD::Triangle>& indices)
    : m_vertices(&vertices)
    , m_indices(&indices)
{
    Build();
}

bool AABBTree::TraceRay(const VHACD::Vect3& start,
                        const VHACD::Vect3& dir,
                        uint32_t& insideCount,
                        uint32_t& outsideCount) const
{
    double outT, u, v, w, faceSign;
    uint32_t faceIndex;
    bool hit = TraceRay(start,
                        dir,
                        outT,
                        u,
                        v,
                        w,
                        faceSign,
                        faceIndex);
    if (hit)
    {
        if (faceSign >= 0)
        {
            insideCount++;
        }
        else
        {
            outsideCount++;
        }
    }
    return hit;
}

bool AABBTree::TraceRay(const VHACD::Vect3& start,
                        const VHACD::Vect3& dir,
                        double& outT,
                        double& u,
                        double& v,
                        double& w,
                        double& faceSign,
                        uint32_t& faceIndex) const
{
    outT = FLT_MAX;
    TraceRecursive(0,
                   start,
                   dir,
                   outT,
                   u,
                   v,
                   w,
                   faceSign,
                   faceIndex);
    return (outT != FLT_MAX);
}

bool AABBTree::GetClosestPointWithinDistance(const VHACD::Vect3& point,
                                             double maxDistance,
                                             VHACD::Vect3& closestPoint) const
{
    double dis, v, w;
    uint32_t faceIndex;
    bool hit = GetClosestPointWithinDistance(point,
                                             maxDistance,
                                             dis,
                                             v,
                                             w,
                                             faceIndex,
                                             closestPoint);
    return hit;
}

// partition faces around the median face
uint32_t AABBTree::PartitionMedian(Node& n,
                                   uint32_t* faces,
                                   uint32_t numFaces)
{
    FaceSorter predicate(*m_vertices,
                         *m_indices,
                         n.m_extents.GetSize().LongestAxis());
    std::nth_element(faces,
                     faces + numFaces / 2,
                     faces + numFaces,
                     predicate);

    return numFaces / 2;
}

// partition faces based on the surface area heuristic
void AABBTree::Build()
{
    const uint32_t numFaces = uint32_t(m_indices->size());

    // build initial list of faces
    m_faces.reserve(numFaces);

    for (uint32_t i = 0; i < numFaces; ++i)
    {
        m_faces.push_back(i);
    }

    m_nodes.reserve(uint32_t(numFaces * double(1.5)));

    // allocate space for all the nodes
    m_freeNode = 1;

    // start building
    BuildRecursive(0,
                   m_faces.data(),
                   numFaces);

}

void AABBTree::BuildRecursive(uint32_t nodeIndex,
                              uint32_t* faces,
                              uint32_t numFaces)
{
    const uint32_t kMaxFacesPerLeaf = 6;

    // if we've run out of nodes allocate some more
    if (nodeIndex >= m_nodes.size())
    {
        uint32_t s = std::max(uint32_t(double(1.5) * m_nodes.size()), 512U);
        m_nodes.resize(s);
    }

    // a reference to the current node, need to be careful here as this reference may become invalid if array is resized
    Node& n = m_nodes[nodeIndex];

    // track max tree depth

    n.m_extents = CalculateFaceBounds(faces,
                                      numFaces);

    // calculate bounds of faces and add node
    if (numFaces <= kMaxFacesPerLeaf)
    {
        n.m_faces = faces;
        n.m_numFaces = numFaces;

    }
    else
    {

        // face counts for each branch
        const uint32_t leftCount = PartitionMedian(n, faces, numFaces);
        // const uint32_t leftCount = PartitionSAH(n, faces, numFaces);
        const uint32_t rightCount = numFaces - leftCount;

        // alloc 2 nodes
        m_nodes[nodeIndex].m_children = m_freeNode;

        // allocate two nodes
        m_freeNode += 2;

        // split faces in half and build each side recursively
        BuildRecursive(m_nodes[nodeIndex].m_children + 0, faces, leftCount);
        BuildRecursive(m_nodes[nodeIndex].m_children + 1, faces + leftCount, rightCount);
    }

}

void AABBTree::TraceRecursive(uint32_t nodeIndex,
                              const VHACD::Vect3& start,
                              const VHACD::Vect3& dir,
                              double& outT,
                              double& outU,
                              double& outV,
                              double& outW,
                              double& faceSign,
                              uint32_t& faceIndex) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == NULL)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        double dist[2] = { FLT_MAX, FLT_MAX };

        IntersectRayAABB(start,
                         dir,
                         leftChild.m_extents,
                         dist[0]);
        IntersectRayAABB(start,
                         dir,
                         rightChild.m_extents,
                         dist[1]);

        uint32_t closest = 0;
        uint32_t furthest = 1;

        if (dist[1] < dist[0])
        {
            closest = 1;
            furthest = 0;
        }

        if (dist[closest] < outT)
        {
            TraceRecursive(node.m_children + closest,
                           start,
                           dir,
                           outT,
                           outU,
                           outV,
                           outW,
                           faceSign,
                           faceIndex);
        }

        if (dist[furthest] < outT)
        {
            TraceRecursive(node.m_children + furthest,
                           start,
                           dir,
                           outT,
                           outU,
                           outV,
                           outW,
                           faceSign,
                           faceIndex);
        }
    }
    else
    {
        double t, u, v, w, s;

        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i];

            const VHACD::Vect3& a = (*m_vertices)[(*m_indices)[indexStart].mI0];
            const VHACD::Vect3& b = (*m_vertices)[(*m_indices)[indexStart].mI1];
            const VHACD::Vect3& c = (*m_vertices)[(*m_indices)[indexStart].mI2];
            if (IntersectRayTriTwoSided(start, dir, a, b, c, t, u, v, w, s, NULL))
            {
                if (t < outT)
                {
                    outT = t;
                    outU = u;
                    outV = v;
                    outW = w;
                    faceSign = s;
                    faceIndex = node.m_faces[i];
                }
            }
        }
    }
}

bool AABBTree::GetClosestPointWithinDistance(const VHACD::Vect3& point,
                                             const double maxDis,
                                             double& dis,
                                             double& v,
                                             double& w,
                                             uint32_t& faceIndex,
                                             VHACD::Vect3& closest) const
{
    dis = maxDis;
    faceIndex = uint32_t(~0);
    double disSq = dis * dis;

    GetClosestPointWithinDistanceSqRecursive(0,
                                             point,
                                             disSq,
                                             v,
                                             w,
                                             faceIndex,
                                             closest);
    dis = sqrt(disSq);

    return (faceIndex < (~(static_cast<unsigned int>(0))));
}

void AABBTree::GetClosestPointWithinDistanceSqRecursive(uint32_t nodeIndex,
                                                        const VHACD::Vect3& point,
                                                        double& outDisSq,
                                                        double& outV,
                                                        double& outW,
                                                        uint32_t& outFaceIndex,
                                                        VHACD::Vect3& closestPoint) const
{
    const Node& node = m_nodes[nodeIndex];

    if (node.m_faces == nullptr)
    {
        // find closest node
        const Node& leftChild = m_nodes[node.m_children + 0];
        const Node& rightChild = m_nodes[node.m_children + 1];

        // double dist[2] = { FLT_MAX, FLT_MAX };
        VHACD::Vect3 lp = leftChild.m_extents.ClosestPoint(point);
        VHACD::Vect3 rp = rightChild.m_extents.ClosestPoint(point);


        uint32_t closest = 0;
        uint32_t furthest = 1;
        double dcSq = (point - lp).GetNormSquared();
        double dfSq = (point - rp).GetNormSquared();
        if (dfSq < dcSq)
        {
            closest = 1;
            furthest = 0;
            std::swap(dfSq, dcSq);
        }

        if (dcSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(node.m_children + closest,
                                                     point,
                                                     outDisSq,
                                                     outV,
                                                     outW,
                                                     outFaceIndex,
                                                     closestPoint);
        }

        if (dfSq < outDisSq)
        {
            GetClosestPointWithinDistanceSqRecursive(node.m_children + furthest,
                                                     point,
                                                     outDisSq,
                                                     outV,
                                                     outW,
                                                     outFaceIndex,
                                                     closestPoint);
        }
    }
    else
    {

        double v, w;
        for (uint32_t i = 0; i < node.m_numFaces; ++i)
        {
            uint32_t indexStart = node.m_faces[i];

            const VHACD::Vect3& a = (*m_vertices)[(*m_indices)[indexStart].mI0];
            const VHACD::Vect3& b = (*m_vertices)[(*m_indices)[indexStart].mI1];
            const VHACD::Vect3& c = (*m_vertices)[(*m_indices)[indexStart].mI2];

            VHACD::Vect3 cp = ClosestPointOnTriangle(a, b, c, point, v, w);
            double disSq = (cp - point).GetNormSquared();

            if (disSq < outDisSq)
            {
                closestPoint = cp;
                outDisSq = disSq;
                outV = v;
                outW = w;
                outFaceIndex = node.m_faces[i];
            }
        }
    }
}

VHACD::BoundsAABB AABBTree::CalculateFaceBounds(uint32_t* faces,
                                                uint32_t numFaces)
{
    VHACD::Vect3 minExtents( FLT_MAX);
    VHACD::Vect3 maxExtents(-FLT_MAX);

    // calculate face bounds
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        VHACD::Vect3 a = (*m_vertices)[(*m_indices)[faces[i]].mI0];
        VHACD::Vect3 b = (*m_vertices)[(*m_indices)[faces[i]].mI1];
        VHACD::Vect3 c = (*m_vertices)[(*m_indices)[faces[i]].mI2];

        minExtents = a.CWiseMin(minExtents);
        maxExtents = a.CWiseMax(maxExtents);

        minExtents = b.CWiseMin(minExtents);
        maxExtents = b.CWiseMax(maxExtents);

        minExtents = c.CWiseMin(minExtents);
        maxExtents = c.CWiseMax(maxExtents);
    }

    return VHACD::BoundsAABB(minExtents,
                             maxExtents);
}

enum class Stages
{
    COMPUTE_BOUNDS_OF_INPUT_MESH,
    CREATE_RAYCAST_MESH,
    VOXELIZING_INPUT_MESH,
    BUILD_INITIAL_CONVEX_HULL,
    PERFORMING_DECOMPOSITION,
    INITIALIZING_CONVEX_HULLS_FOR_MERGING,
    COMPUTING_COST_MATRIX,
    MERGING_CONVEX_HULLS,
    FINALIZING_RESULTS,
    NUM_STAGES
};

class VHACDCallbacks
{
public:
    virtual void ProgressUpdate(Stages stage,
                                double stageProgress,
                                const char *operation) = 0;
    // Reports progress when the last report is older than the progress interval, so a caller that
    // checks for cancellation inside IUserCallback::Update notices it within a bounded time.
    // Long loops call this once per bounded unit of work. Returns true once Cancel was requested.
    virtual bool PollProgress(Stages stage,
                              double stageProgress,
                              const char *operation) = 0;
    virtual bool IsCanceled() const = 0;

    virtual ~VHACDCallbacks() = default;
};

enum class VoxelValue : uint8_t
{
    PRIMITIVE_UNDEFINED = 0,
    PRIMITIVE_OUTSIDE_SURFACE_TOWALK = 1,
    PRIMITIVE_OUTSIDE_SURFACE = 2,
    PRIMITIVE_INSIDE_SURFACE = 3,
    PRIMITIVE_ON_SURFACE = 4
};

class Volume
{
public:
    // Returns early with a partial volume once callbacks report cancellation.
    void Voxelize(const std::vector<VHACD::Vertex>& points,
                  const std::vector<VHACD::Triangle>& triangles,
                  double voxelSize,
                  uint32_t pad,
                  FillMode fillMode,
                  const AABBTree& aabbTree,
                  VHACDCallbacks& callbacks);

    void RaycastFill(const AABBTree& aabbTree,
                     VHACDCallbacks& callbacks);

    void SetVoxel(const size_t i,
                  const size_t j,
                  const size_t k,
                  VoxelValue value);

    VoxelValue& GetVoxel(const size_t i,
                         const size_t j,
                         const size_t k);


    const std::vector<Voxel>& GetSurfaceVoxels() const;
    const std::vector<Voxel>& GetInteriorVoxels() const;

    double GetScale() const;
    const VHACD::BoundsAABB& GetBounds() const;
    const VHACD::Vector3<uint32_t>& GetDimensions() const;

    VHACD::BoundsAABB m_bounds;
    double m_scale{ 1.0 };
    VHACD::Vector3<uint32_t> m_dim{ 0 };
    std::vector<VoxelValue> m_data;
private:

    void MarkOutsideSurface(const size_t i0,
                            const size_t j0,
                            const size_t k0,
                            const size_t i1,
                            const size_t j1,
                            const size_t k1);
    void FillOutsideSurface(VHACDCallbacks& callbacks);

    void FillInsideSurface(VHACDCallbacks& callbacks);

    std::vector<VHACD::Voxel> m_surfaceVoxels;
    std::vector<VHACD::Voxel> m_interiorVoxels;
};

bool PlaneBoxOverlap(const VHACD::Vect3& normal,
                     const VHACD::Vect3& vert,
                     const VHACD::Vect3& maxbox)
{
    int32_t q;
    VHACD::Vect3 vmin;
    VHACD::Vect3 vmax;
    double v;
    for (q = 0; q < 3; q++)
    {
        v = vert[q];
        if (normal[q] > double(0.0))
        {
            vmin[q] = -maxbox[q] - v;
            vmax[q] =  maxbox[q] - v;
        }
        else
        {
            vmin[q] =  maxbox[q] - v;
            vmax[q] = -maxbox[q] - v;
        }
    }
    if (normal.Dot(vmin) > double(0.0))
        return false;
    if (normal.Dot(vmax) >= double(0.0))
        return true;
    return false;
}

bool AxisTest(double  a, double  b, double fa, double fb,
              double v0, double v1, double v2, double v3,
              double boxHalfSize1,  double boxHalfSize2)
{
    double p0 = a * v0 + b * v1;
    double p1 = a * v2 + b * v3;

    double min = std::min(p0, p1);
    double max = std::max(p0, p1);

    double rad = fa * boxHalfSize1 + fb * boxHalfSize2;
    if (min > rad || max < -rad)
    {
        return false;
    }

    return true;
}

bool TriBoxOverlap(const VHACD::Vect3& boxCenter,
                   const VHACD::Vect3& boxHalfSize,
                   const VHACD::Vect3& triVer0,
                   const VHACD::Vect3& triVer1,
                   const VHACD::Vect3& triVer2)
{
    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-direction) */
    /*       this gives 3x3=9 more tests */

    VHACD::Vect3 v0 = triVer0 - boxCenter;
    VHACD::Vect3 v1 = triVer1 - boxCenter;
    VHACD::Vect3 v2 = triVer2 - boxCenter;
    VHACD::Vect3 e0 = v1 - v0;
    VHACD::Vect3 e1 = v2 - v1;
    VHACD::Vect3 e2 = v0 - v2;

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */

    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */
    double fex = fabs(e0[0]);
    double fey = fabs(e0[1]);
    double fez = fabs(e0[2]);

    /*
     * These should use Get*() instead of subscript for consistency, but the function calls are long enough already
     */
    if (!AxisTest( e0[2], -e0[1], fez, fey, v0[1], v0[2], v2[1], v2[2], boxHalfSize[1], boxHalfSize[2])) return 0; // X01
    if (!AxisTest(-e0[2],  e0[0], fez, fex, v0[0], v0[2], v2[0], v2[2], boxHalfSize[0], boxHalfSize[2])) return 0; // Y02
    if (!AxisTest( e0[1], -e0[0], fey, fex, v1[0], v1[1], v2[0], v2[1], boxHalfSize[0], boxHalfSize[1])) return 0; // Z12

    fex = fabs(e1[0]);
    fey = fabs(e1[1]);
    fez = fabs(e1[2]);

    if (!AxisTest( e1[2], -e1[1], fez, fey, v0[1], v0[2], v2[1], v2[2], boxHalfSize[1], boxHalfSize[2])) return 0; // X01
    if (!AxisTest(-e1[2],  e1[0], fez, fex, v0[0], v0[2], v2[0], v2[2], boxHalfSize[0], boxHalfSize[2])) return 0; // Y02
    if (!AxisTest( e1[1], -e1[0], fey, fex, v0[0], v0[1], v1[0], v1[1], boxHalfSize[0], boxHalfSize[1])) return 0; // Z0

    fex = fabs(e2[0]);
    fey = fabs(e2[1]);
    fez = fabs(e2[2]);

    if (!AxisTest( e2[2], -e2[1], fez, fey, v0[1], v0[2], v1[1], v1[2], boxHalfSize[1], boxHalfSize[2])) return 0; // X2
    if (!AxisTest(-e2[2],  e2[0], fez, fex, v0[0], v0[2], v1[0], v1[2], boxHalfSize[0], boxHalfSize[2])) return 0; // Y1
    if (!AxisTest( e2[1], -e2[0], fey, fex, v1[0], v1[1], v2[0], v2[1], boxHalfSize[0], boxHalfSize[1])) return 0; // Z12

    /* Bullet 1: */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in 0-direction */
    double min = std::min({v0.GetX(), v1.GetX(), v2.GetX()});
    double max = std::max({v0.GetX(), v1.GetX(), v2.GetX()});
    if (min > boxHalfSize[0] || max < -boxHalfSize[0])
        return false;

    /* test in 1-direction */
    min = std::min({v0.GetY(), v1.GetY(), v2.GetY()});
    max = std::max({v0.GetY(), v1.GetY(), v2.GetY()});
    if (min > boxHalfSize[1] || max < -boxHalfSize[1])
        return false;

    /* test in getZ-direction */
    min = std::min({v0.GetZ(), v1.GetZ(), v2.GetZ()});
    max = std::max({v0.GetZ(), v1.GetZ(), v2.GetZ()});
    if (min > boxHalfSize[2] || max < -boxHalfSize[2])
        return false;

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    VHACD::Vect3 normal = e0.Cross(e1);

    if (!PlaneBoxOverlap(normal, v0, boxHalfSize))
        return false;
    return true; /* box and triangle overlaps */
}

// Voxel packs voxel coordinates into 10 bits each. A coordinate on a shorter axis reaches the
// longest-axis count plus one, so that count must stay below 1023.
constexpr size_t MaxVoxelDimension = 1021;

void Volume::Voxelize(const std::vector<VHACD::Vertex>& points,
                      const std::vector<VHACD::Triangle>& indices,
                      const double voxelSize,
                      const uint32_t pad,
                      FillMode fillMode,
                      const AABBTree& aabbTree,
                      VHACDCallbacks& callbacks)
{
    if (points.size() == 0)
    {
        return;
    }

    // The grid holds the mesh plus pad voxels of empty space on every side, which is the room the probe
    // sphere of SpaceModel needs to reach around the mesh.
    const VHACD::BoundsAABB meshBounds(points);
    const VHACD::Vect3 d = meshBounds.GetSize();
    const double padding = double(pad) * voxelSize;
    m_bounds = VHACD::BoundsAABB(meshBounds.GetMin() - padding,
                                 meshBounds.GetMax() + padding);
    m_scale = voxelSize;
    const double invScale = double(1.0) / voxelSize;
    for (int32_t axis = 0; axis < 3; ++axis)
    {
        m_dim[axis] = uint32_t(size_t(d[axis] * invScale) + 1 + 2 * size_t(pad));
        assert(m_dim[axis] <= MaxVoxelDimension);
    }

    m_data = std::vector<VoxelValue>(m_dim[0] * m_dim[1] * m_dim[2],
                                     VoxelValue::PRIMITIVE_UNDEFINED);

    VHACD::Vect3 p[3];
    VHACD::Vect3 boxcenter;
    VHACD::Vect3 pt;
    const VHACD::Vect3 boxhalfsize(double(0.5));
    for (size_t t = 0; t < indices.size(); ++t)
    {
        if ( (t & 1023) == 0 && callbacks.PollProgress(Stages::VOXELIZING_INPUT_MESH, 50.0 * double(t) / double(indices.size()), "Voxelizing triangles") )
        {
            return;
        }
        size_t i0, j0, k0;
        size_t i1, j1, k1;
        VHACD::Vector3<uint32_t> tri = indices[t];
        for (int32_t c = 0; c < 3; ++c)
        {
            pt = points[tri[c]];

            p[c] = (pt - m_bounds.GetMin()) * invScale;

            size_t i = static_cast<size_t>(p[c][0] + double(0.5));
            size_t j = static_cast<size_t>(p[c][1] + double(0.5));
            size_t k = static_cast<size_t>(p[c][2] + double(0.5));

            assert(i < m_dim[0] && j < m_dim[1] && k < m_dim[2]);

            if (c == 0)
            {
                i0 = i1 = i;
                j0 = j1 = j;
                k0 = k1 = k;
            }
            else
            {
                i0 = std::min(i0, i);
                j0 = std::min(j0, j);
                k0 = std::min(k0, k);

                i1 = std::max(i1, i);
                j1 = std::max(j1, j);
                k1 = std::max(k1, k);
            }
        }
        if (i0 > 0)
            --i0;
        if (j0 > 0)
            --j0;
        if (k0 > 0)
            --k0;
        if (i1 < m_dim[0])
            ++i1;
        if (j1 < m_dim[1])
            ++j1;
        if (k1 < m_dim[2])
            ++k1;
        for (size_t i_id = i0; i_id < i1; ++i_id)
        {
            boxcenter[0] = uint32_t(i_id);
            for (size_t j_id = j0; j_id < j1; ++j_id)
            {
                boxcenter[1] = uint32_t(j_id);
                for (size_t k_id = k0; k_id < k1; ++k_id)
                {
                    boxcenter[2] = uint32_t(k_id);
                    // A voxel an earlier triangle already marked keeps its surface flags, so it needs no overlap test.
                    VoxelValue& value = GetVoxel(i_id,
                                                 j_id,
                                                 k_id);
                    if (   value == VoxelValue::PRIMITIVE_UNDEFINED
                        && TriBoxOverlap(boxcenter,
                                         boxhalfsize,
                                         p[0],
                                         p[1],
                                         p[2]))
                    {
                        value = VoxelValue::PRIMITIVE_ON_SURFACE;
                        m_surfaceVoxels.emplace_back(uint32_t(i_id),
                                                     uint32_t(j_id),
                                                     uint32_t(k_id));
                    }
                }
            }
        }
    }

    if (fillMode == FillMode::SURFACE_ONLY)
    {
        const size_t i0_local = m_dim[0];
        const size_t j0_local = m_dim[1];
        const size_t k0_local = m_dim[2];
        for (size_t i_id = 0; i_id < i0_local; ++i_id)
        {
            for (size_t j_id = 0; j_id < j0_local; ++j_id)
            {
                for (size_t k_id = 0; k_id < k0_local; ++k_id)
                {
                    const VoxelValue& voxel = GetVoxel(i_id,
                                                       j_id,
                                                       k_id);
                    if (voxel != VoxelValue::PRIMITIVE_ON_SURFACE)
                    {
                        SetVoxel(i_id,
                                 j_id,
                                 k_id,
                                 VoxelValue::PRIMITIVE_OUTSIDE_SURFACE);
                    }
                }
            }
        }
    }
    else if (fillMode == FillMode::FLOOD_FILL)
    {
        /*
         * Marking the outside edges of the voxel cube to be outside surfaces to walk
         */
        MarkOutsideSurface(0,            0,            0,            m_dim[0], m_dim[1], 1);
        MarkOutsideSurface(0,            0,            m_dim[2] - 1, m_dim[0], m_dim[1], m_dim[2]);
        MarkOutsideSurface(0,            0,            0,            m_dim[0], 1,        m_dim[2]);
        MarkOutsideSurface(0,            m_dim[1] - 1, 0,            m_dim[0], m_dim[1], m_dim[2]);
        MarkOutsideSurface(0,            0,            0,            1,        m_dim[1], m_dim[2]);
        MarkOutsideSurface(m_dim[0] - 1, 0,            0,            m_dim[0], m_dim[1], m_dim[2]);
        FillOutsideSurface(callbacks);
        FillInsideSurface(callbacks);
    }
    else if (fillMode == FillMode::RAYCAST_FILL)
    {
        RaycastFill(aabbTree, callbacks);
    }
}

void Volume::RaycastFill(const AABBTree& aabbTree,
                         VHACDCallbacks& callbacks)
{
    const uint32_t i0 = m_dim[0];
    const uint32_t j0 = m_dim[1];
    const uint32_t k0 = m_dim[2];

    size_t maxSize = i0 * j0 * k0;

    std::vector<Voxel> temp;
    temp.reserve(maxSize);
    uint32_t count{ 0 };
    for (uint32_t i = 0; i < i0; ++i)
    {
        if ( callbacks.PollProgress(Stages::VOXELIZING_INPUT_MESH, 50.0 + 50.0 * double(i) / double(i0), "Raycast fill") )
        {
            return;
        }
        for (uint32_t j = 0; j < j0; ++j)
        {
            for (uint32_t k = 0; k < k0; ++k)
            {
                VoxelValue& voxel = GetVoxel(i, j, k);
                if (voxel != VoxelValue::PRIMITIVE_ON_SURFACE)
                {
                    VHACD::Vect3 start = VHACD::Vect3(i, j, k) * m_scale + m_bounds.GetMin();

                    uint32_t insideCount = 0;
                    uint32_t outsideCount = 0;

                    VHACD::Vect3 directions[6] = {
                        VHACD::Vect3( 1,  0,  0),
                        VHACD::Vect3(-1,  0,  0), // this was 1, 0, 0 in the original code, but looks wrong
                        VHACD::Vect3( 0,  1,  0),
                        VHACD::Vect3( 0, -1,  0),
                        VHACD::Vect3( 0,  0,  1),
                        VHACD::Vect3( 0,  0, -1)
                    };

                    for (uint32_t r = 0; r < 6; r++)
                    {
                        aabbTree.TraceRay(start,
                                          directions[r],
                                          insideCount,
                                          outsideCount);
                        // Early out if we hit the outside of the mesh
                        if (outsideCount)
                        {
                            break;
                        }
                        // Early out if we accumulated 3 inside hits
                        if (insideCount >= 3)
                        {
                            break;
                        }
                    }

                    if (outsideCount == 0 && insideCount >= 3)
                    {
                        voxel = VoxelValue::PRIMITIVE_INSIDE_SURFACE;
                        temp.emplace_back(i, j, k);
                        count++;
                    }
                    else
                    {
                        voxel = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE;
                    }
                }
            }
        }
    }

    if (count)
    {
        m_interiorVoxels = std::move(temp);
    }
}

void Volume::SetVoxel(const size_t i,
                      const size_t j,
                      const size_t k,
                      VoxelValue value)
{
    assert(i < m_dim[0]);
    assert(j < m_dim[1]);
    assert(k < m_dim[2]);

    m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]] = value;
}

VoxelValue& Volume::GetVoxel(const size_t i,
                             const size_t j,
                             const size_t k)
{
    assert(i < m_dim[0]);
    assert(j < m_dim[1]);
    assert(k < m_dim[2]);
    return m_data[k + j * m_dim[2] + i * m_dim[1] * m_dim[2]];
}

const std::vector<Voxel>& Volume::GetSurfaceVoxels() const
{
    return m_surfaceVoxels;
}

const std::vector<Voxel>& Volume::GetInteriorVoxels() const
{
    return m_interiorVoxels;
}

double Volume::GetScale() const
{
    return m_scale;
}

const VHACD::BoundsAABB& Volume::GetBounds() const
{
    return m_bounds;
}

const VHACD::Vector3<uint32_t>& Volume::GetDimensions() const
{
    return m_dim;
}

void Volume::MarkOutsideSurface(const size_t i0,
                                const size_t j0,
                                const size_t k0,
                                const size_t i1,
                                const size_t j1,
                                const size_t k1)
{
    for (size_t i = i0; i < i1; ++i)
    {
        for (size_t j = j0; j < j1; ++j)
        {
            for (size_t k = k0; k < k1; ++k)
            {
                VoxelValue& v = GetVoxel(i, j, k);
                if (v == VoxelValue::PRIMITIVE_UNDEFINED)
                {
                    v = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
                }
            }
        }
    }
}

// Marks up to maxDistance undefined voxels after (direction +1) or before (direction -1) the voxel at
// index along one axis, stopping at the first defined voxel or the grid boundary. coordinate is the
// voxel coordinate on that axis and axisCount the number of voxels along it. Addresses are formed
// only for voxels inside the grid.
inline void WalkAxis(VoxelValue* const data,
                     int64_t index,
                     int64_t coordinate,
                     const int64_t axisCount,
                     const int64_t stride,
                     const int64_t direction,
                     const int64_t maxDistance)
{
    for (int64_t count = 0; count < maxDistance; ++count)
    {
        coordinate += direction;
        if ( coordinate < 0 || coordinate >= axisCount )
        {
            return;
        }
        index += direction * stride;
        if ( data[index] != VoxelValue::PRIMITIVE_UNDEFINED )
        {
            return;
        }
        data[index] = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK;
    }
}

void Volume::FillOutsideSurface(VHACDCallbacks& callbacks)
{
    size_t voxelsWalked = 0;
    const int64_t i0 = m_dim[0];
    const int64_t j0 = m_dim[1];
    const int64_t k0 = m_dim[2];

    // Avoid striding too far in each direction to stay in L1 cache as much as possible.
    // The cache size required for the walk is roughly (4 * walkDistance * 64) since
    // the k direction doesn't count as it's walking byte per byte directly in a cache lines.
    // ~16k is required for a walk distance of 64 in each directions.
    const int64_t walkDistance = 64;

    // Strides of the GetVoxel layout. Walking by stride saves the index multiplications.
    const int64_t kstride = 1;
    const int64_t jstride = k0;
    const int64_t istride = j0 * k0;
    VoxelValue* const data = m_data.data();

    // It might seem counter intuitive to go over the whole voxel range multiple times
    // but since we do the run in memory order, it leaves us with far fewer cache misses
    // than a BFS algorithm and it has the additional benefit of not requiring us to
    // store and manipulate a fifo for recursion that might become huge when the number
    // of voxels is large.
    // This will outperform the BFS algorithm by several orders of magnitude in practice.
    do
    {
        voxelsWalked = 0;
        for (int64_t i = 0; i < i0; ++i)
        {
            if ( callbacks.PollProgress(Stages::VOXELIZING_INPUT_MESH, 50.0 + 25.0 * double(i) / double(i0), "Flood filling outside") )
            {
                return;
            }
            for (int64_t j = 0; j < j0; ++j)
            {
                for (int64_t k = 0; k < k0; ++k)
                {
                    VoxelValue& voxel = GetVoxel(i, j, k);
                    if (voxel == VoxelValue::PRIMITIVE_OUTSIDE_SURFACE_TOWALK)
                    {
                        voxelsWalked++;
                        voxel = VoxelValue::PRIMITIVE_OUTSIDE_SURFACE;

                        // walk in each direction to mark other voxel that should be walked.
                        // this will generate a 3d pattern that will help the overall
                        // algorithm converge faster while remaining cache friendly.
                        const int64_t index = k + j * jstride + i * istride;
                        WalkAxis(data, index, k, k0, kstride, 1, walkDistance);
                        WalkAxis(data, index, k, k0, kstride, -1, walkDistance);

                        WalkAxis(data, index, j, j0, jstride, 1, walkDistance);
                        WalkAxis(data, index, j, j0, jstride, -1, walkDistance);

                        WalkAxis(data, index, i, i0, istride, 1, walkDistance);
                        WalkAxis(data, index, i, i0, istride, -1, walkDistance);
                    }
                }
            }
        }

    } while (voxelsWalked != 0);
}

void Volume::FillInsideSurface(VHACDCallbacks& callbacks)
{
    const uint32_t i0 = uint32_t(m_dim[0]);
    const uint32_t j0 = uint32_t(m_dim[1]);
    const uint32_t k0 = uint32_t(m_dim[2]);

    size_t maxSize = i0 * j0 * k0;

    std::vector<Voxel> temp;
    temp.reserve(maxSize);
    uint32_t count{ 0 };

    for (uint32_t i = 0; i < i0; ++i)
    {
        if ( callbacks.PollProgress(Stages::VOXELIZING_INPUT_MESH, 75.0 + 25.0 * double(i) / double(i0), "Filling inside") )
        {
            return;
        }
        for (uint32_t j = 0; j < j0; ++j)
        {
            for (uint32_t k = 0; k < k0; ++k)
            {
                VoxelValue& v = GetVoxel(i, j, k);
                if (v == VoxelValue::PRIMITIVE_UNDEFINED)
                {
                    v = VoxelValue::PRIMITIVE_INSIDE_SURFACE;
                    temp.emplace_back(i, j, k);
                    count++;
                }
            }
        }
    }

    if ( count )
    {
        m_interiorVoxels = std::move(temp);
    }
}

//******************************************************************************************
//  ShrinkWrap helper class
//******************************************************************************************
// This is a code snippet which 'shrinkwraps' a convex hull
// to a source mesh.
//
// It is a somewhat complicated algorithm. It works as follows:
//
// * Step #1 : Compute the mean unit normal vector for each vertex in the convex hull
// * Step #2 : For each vertex in the conex hull we project is slightly outwards along the mean normal vector
// * Step #3 : We then raycast from this slightly extruded point back into the opposite direction of the mean normal vector
//             resulting in a raycast from slightly beyond the vertex in the hull into the source mesh we are trying
//             to 'shrink wrap' against
// * Step #4 : If the raycast fails we leave the original vertex alone
// * Step #5 : If the raycast hits a backface we leave the original vertex alone
// * Step #6 : If the raycast hits too far away (no more than a certain threshold distance) we live it alone
// * Step #7 : If the point we hit on the source mesh is not still within the convex hull, we reject it.
// * Step #8 : If all of the previous conditions are met, then we take the raycast hit location as the 'new position'
// * Step #9 : Once all points have been projected, if possible, we need to recompute the convex hull again based on these shrinkwrapped points
// * Step #10 : In theory that should work.. let's see...

//***********************************************************************************************
// Reachable space, the closed solid, and the voxels a hull may not contain
//***********************************************************************************************

// Everything below works in voxel index space, where voxel (i, j, k) occupies
// [i, i + 1] x [j, j + 1] x [k, k + 1] and its centre sits at (i + 0.5, j + 0.5, k + 0.5). A point p of
// the voxelized mesh maps to (p - origin) / scale, where origin is the minimum corner of voxel (0, 0, 0),
// which is VoxelHull::GetPoint(0, 0, 0).

// Larger than the longest distance any grid can produce, so a line holding no seed never wins a
// comparison against a line that does.
constexpr int32_t kNoSeedDistance = 4 * int32_t(MaxVoxelDimension);

inline int64_t FloorDiv(const int64_t numerator, const int64_t denominator)
{
    const int64_t quotient = numerator / denominator;
    return (numerator % denominator != 0 && ((numerator < 0) != (denominator < 0))) ? quotient - 1 : quotient;
}

// One pass of the separable distance transform over a line: out[u] = min over v of (u - v)^2 + in[v],
// where in[] already holds squared distances. Lower envelope of parabolas after Meijster, Roerdink and
// Hesselink, "A general algorithm for computing distance transforms in linear time" (2000). Integer
// only, so the field is exact and reproduces bit for bit.
inline void DistanceLinePass(const std::vector<int32_t>& in,
                             std::vector<int32_t>& out,
                             std::vector<int32_t>& vertex,
                             std::vector<int32_t>& boundary)
{
    const int32_t count = int32_t(in.size());
    if (count == 0)
    {
        return;
    }
    auto value = [&in](const int64_t x, const int32_t i)
    {
        const int64_t delta = x - i;
        return delta * delta + in[size_t(i)];
    };
    // The envelope is built in 64 bits and stored in 32: a squared distance cannot reach 2^31.
    auto separator = [&in](const int32_t i, const int32_t u)
    {
        return FloorDiv(int64_t(u) * u - int64_t(i) * i + in[size_t(u)] - in[size_t(i)], 2 * (int64_t(u) - i));
    };

    int32_t top = 0;
    vertex[0] = 0;
    boundary[0] = 0;
    for (int32_t u = 1; u < count; ++u)
    {
        while (top >= 0 && value(boundary[size_t(top)], vertex[size_t(top)]) > value(boundary[size_t(top)], u))
        {
            --top;
        }
        if (top < 0)
        {
            top = 0;
            vertex[0] = u;
        }
        else
        {
            const int64_t crossing = 1 + separator(vertex[size_t(top)], u);
            if (crossing < count)
            {
                ++top;
                vertex[size_t(top)] = u;
                boundary[size_t(top)] = crossing;
            }
        }
    }
    for (int32_t u = count - 1; u >= 0; --u)
    {
        out[size_t(u)] = int32_t(value(u, vertex[size_t(top)]));
        if (u == boundary[size_t(top)])
        {
            --top;
        }
    }
}

// Squared distance, in voxel units, from every voxel centre to the nearest centre of a seed voxel.
// Voxels without any seed in the grid receive a distance larger than the grid can span.
void SquaredDistanceTransform(const VHACD::Vector3<uint32_t>& dim,
                              const std::vector<uint8_t>& seeds,
                              std::vector<int32_t>& distances)
{
    const size_t dimX = dim[0];
    const size_t dimY = dim[1];
    const size_t dimZ = dim[2];
    distances.assign(dimX * dimY * dimZ, 0);
    if (distances.empty())
    {
        return;
    }

    // Pass 1: distance along z inside each (i, j) column, still unsquared.
    for (size_t i = 0; i < dimX; ++i)
    {
        for (size_t j = 0; j < dimY; ++j)
        {
            const size_t base = (i * dimY + j) * dimZ;
            int32_t distance = kNoSeedDistance;
            for (size_t k = 0; k < dimZ; ++k)
            {
                distance = seeds[base + k] ? 0 : std::min(distance + 1, kNoSeedDistance);
                distances[base + k] = distance;
            }
            distance = kNoSeedDistance;
            for (size_t k = dimZ; k-- > 0;)
            {
                distance = seeds[base + k] ? 0 : std::min(distance + 1, kNoSeedDistance);
                distances[base + k] = std::min(distances[base + k], distance);
            }
            for (size_t k = 0; k < dimZ; ++k)
            {
                distances[base + k] *= distances[base + k];
            }
            static_assert(int64_t(kNoSeedDistance) * kNoSeedDistance < INT32_MAX, "squared distances must fit");
        }
    }

    // Pass 2 along y, then pass 3 along x, each gathering a line, transforming it and scattering it back.
    std::vector<int32_t> line;
    std::vector<int32_t> result;
    std::vector<int32_t> vertex;
    std::vector<int32_t> boundary;
    const size_t longest = std::max(dimX, dimY);
    line.reserve(longest);
    result.reserve(longest);
    vertex.resize(longest);
    boundary.resize(longest);

    line.resize(dimY);
    result.resize(dimY);
    for (size_t i = 0; i < dimX; ++i)
    {
        for (size_t k = 0; k < dimZ; ++k)
        {
            for (size_t j = 0; j < dimY; ++j)
            {
                line[j] = distances[(i * dimY + j) * dimZ + k];
            }
            DistanceLinePass(line, result, vertex, boundary);
            for (size_t j = 0; j < dimY; ++j)
            {
                distances[(i * dimY + j) * dimZ + k] = result[j];
            }
        }
    }

    line.resize(dimX);
    result.resize(dimX);
    for (size_t j = 0; j < dimY; ++j)
    {
        for (size_t k = 0; k < dimZ; ++k)
        {
            for (size_t i = 0; i < dimX; ++i)
            {
                line[i] = distances[(i * dimY + j) * dimZ + k];
            }
            DistanceLinePass(line, result, vertex, boundary);
            for (size_t i = 0; i < dimX; ++i)
            {
                distances[(i * dimY + j) * dimZ + k] = result[i];
            }
        }
    }
}

// Voxels per tolerance. A hull face can pass between voxel centres and the far test spares voxels within
// one voxel of the closed solid, so two voxels per tolerance keep the true error near the tolerance.
// The notches of a voxel staircase, at most sqrt(3) voxels from the solid, need one more: a probe closing
// two voxels wide swallows them, and where there is none the third voxel of margin does instead. Tilted
// solids stay whole either way, and at two voxels a model costs 3.4 times fewer voxels.
constexpr double kVoxelsPerToleranceClosed = 2.0;
constexpr double kVoxelsPerToleranceOpen = 3.0;

// Voxels along the longest axis of the smallest model, so even a model smaller than its tolerance keeps a
// recognizable shape.
constexpr uint32_t kMinVoxelsPerModel = 16;

// Below this a grid cannot hold a mesh and the space around it at any useful tolerance.
constexpr uint32_t MinVoxelBudget = 4096;

// The voxel size for a tolerance, coarsened until the padded grid fits the voxel budget and the voxels
// per axis the Voxel packing allows. Extents are in the same units as the tolerance.
struct GridSizing
{
    double      m_voxelSize{ 0 };
    uint32_t    m_pad{ 0 };
    double      m_voxelsPerTolerance{ kVoxelsPerToleranceOpen };
    bool        m_budgetBound{ false };
};

// Voxels along an axis for a voxel size: the mesh, plus the padding the probe needs on both sides.
inline double GridSideFor(const double extent,
                          const double voxelSize,
                          const uint32_t pad)
{
    return std::floor(extent / voxelSize) + 1 + 2 * double(pad);
}

inline uint32_t GridPadFor(const double probeRadius,
                           const double voxelSize)
{
    // The probe needs its radius of empty space, plus a voxel for the far test and one to keep the mesh
    // off the boundary.
    return uint32_t(std::ceil(probeRadius / voxelSize)) + 2;
}

inline GridSizing ChooseGrid(const VHACD::Vect3& extent,
                             const double tolerance,
                             const double probeRadius,
                             const uint32_t maxVoxels)
{
    const double longest = std::max({ extent[0], extent[1], extent[2] });
    // Callers voxelize nothing without extent, and validation rejects a tolerance that is not positive.
    assert(longest > double(0.0) && tolerance > double(0.0) && maxVoxels >= MinVoxelBudget);

    GridSizing sizing;
    sizing.m_voxelSize = std::min(tolerance / kVoxelsPerToleranceClosed, longest / double(kMinVoxelsPerModel));
    if (probeRadius < 2 * sizing.m_voxelSize)
    {
        sizing.m_voxelSize = std::min(tolerance / kVoxelsPerToleranceOpen, longest / double(kMinVoxelsPerModel));
    }

    // Voxel coordinates are packed into ten bits each, so no axis may exceed MaxVoxelDimension. Since
    // floor(e / h) + 1 + 2 * (ceil(r / h) + 2) is at most (e + 2r) / h + 7, this size settles it outright,
    // and every step below only coarsens the grid further.
    const double axisFloor = (longest + 2 * probeRadius) / double(MaxVoxelDimension - 7);
    if (sizing.m_voxelSize < axisFloor)
    {
        sizing.m_voxelSize = axisFloor;
        sizing.m_budgetBound = true;
    }

    // Coarsen until the grid fits the voxel budget. A voxel the size of the model leaves five voxels per
    // axis whatever the padding, which is far inside the smallest budget the contract allows, so this
    // always ends.
    for (uint32_t attempt = 0; attempt < 64; ++attempt)
    {
        sizing.m_pad = GridPadFor(probeRadius, sizing.m_voxelSize);
        double count = 1;
        for (int32_t axis = 0; axis < 3; ++axis)
        {
            count *= GridSideFor(extent[axis], sizing.m_voxelSize, sizing.m_pad);
        }
        if (count <= double(maxVoxels))
        {
            break;
        }
        sizing.m_budgetBound = true;
        sizing.m_voxelSize *= std::max(std::cbrt(count / double(maxVoxels)), double(1.02));
        if (attempt + 1 == 64)
        {
            // Unreachable while the multiplier above holds, and cheap insurance if it ever does not.
            sizing.m_voxelSize = longest;
            sizing.m_pad = GridPadFor(probeRadius, sizing.m_voxelSize);
        }
    }

    // The margin the far test needs depends on whether the grid that was finally chosen resolves the
    // probe, not on the one first asked for.
    sizing.m_voxelsPerTolerance = probeRadius >= 2 * sizing.m_voxelSize ? kVoxelsPerToleranceClosed
                                                                       : kVoxelsPerToleranceOpen;
    assert(GridSideFor(extent[0], sizing.m_voxelSize, sizing.m_pad) <= double(MaxVoxelDimension)
           && GridSideFor(extent[1], sizing.m_voxelSize, sizing.m_pad) <= double(MaxVoxelDimension)
           && GridSideFor(extent[2], sizing.m_voxelSize, sizing.m_pad) <= double(MaxVoxelDimension));
    return sizing;
}

// The space a probe sphere reaches, the solid that closing leaves behind, and the voxels a hull may not
// contain.
//
// A decomposition may fill anything nothing can reach: gaps narrower than the probe, pockets behind a
// narrow neck, and sealed cavities. What it may not do is stand more than the tolerance away from that
// closed solid where a probe can be. Both the split test and the merge test reduce to one question:
// does this hull contain a voxel centre of the far set?
class SpaceModel
{
public:
    // probeRadius and tolerance are in voxels. Returns early, leaving nothing protected, once callbacks
    // report cancellation; Compute discards the decomposition in that case.
    void Build(const Volume& volume,
               double probeRadius,
               double tolerance,
               VHACDCallbacks& callbacks);

    // True when a voxel centre inside the hull lies farther than the tolerance from the closed solid.
    bool HullReachesTooFar(const std::vector<VHACD::Vertex>& points,
                           const std::vector<VHACD::Triangle>& triangles) const;

    // The same test for a hull still held by the builder, which merge candidates are.
    bool HullReachesTooFar(const std::vector<VHACD::Vect3>& points,
                           const std::vector<VHACD::ConvexHullFace>& faces) const;

    // True when the segment between two points passes through a voxel no hull may cover. Any hull holding
    // both points holds the segment, so this rejects a merge without building anything.
    bool SegmentReachesTooFar(const VHACD::Vect3& from,
                              const VHACD::Vect3& to) const;

    // False when nothing inside this box may be covered, which settles a hull inside the box in constant
    // time. It only reads the columns the box spans, so it can answer true for a box that in fact holds
    // no far voxel; every caller treats that as "look closer".
    bool BoxMayHoldFarVoxel(const VHACD::Vect3& low,
                            const VHACD::Vect3& high) const;

    uint64_t GetFarVoxelCount() const
    {
        return m_farVoxelCount;
    }

    // True when no hull may cover this voxel's centre.
    bool IsFarVoxel(int32_t i, int32_t j, int32_t k) const;

private:
    struct Plane
    {
        VHACD::Vect3 m_normal; // Outward, unit length
        double m_offset;       // A point x is inside when m_normal.Dot(x) + m_offset <= 0
    };

    // The test both entry points share: index-space points, and face corners as indices into them.
    bool ReachesTooFar(const std::vector<VHACD::Vect3>& local,
                       const uint32_t* faceIndices,
                       size_t faceStride,
                       size_t faceCount) const;

    // Far voxels in the columns of an index-space rectangle, if any.
    bool RectangleHasFarVoxel(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1) const;
    bool ScanRectangle(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const std::vector<Plane>& planes) const;
    bool ColumnHasFarVoxelInside(uint32_t i, uint32_t j, const std::vector<Plane>& planes) const;

    VHACD::Vector3<uint32_t> m_dim{ 0 };
    VHACD::Vect3 m_origin{ 0, 0, 0 }; // Minimum corner of voxel (0, 0, 0)
    double m_scale{ 1.0 };
    // Far voxels per column as sorted, inclusive z ranges, indexed through m_runStart (one extra entry).
    std::vector<uint32_t> m_runStart;
    std::vector<uint16_t> m_runZ;
    // Summed far-voxel counts over columns, (dimX + 1) by (dimY + 1), for empty-rectangle tests.
    std::vector<uint32_t> m_columnSum;
    uint64_t m_farVoxelCount{ 0 };
};

void SpaceModel::Build(const Volume& volume,
                       const double probeRadius,
                       const double tolerance,
                       VHACDCallbacks& callbacks)
{
    m_dim = volume.GetDimensions();
    m_scale = volume.GetScale();
    m_origin = volume.GetBounds().GetMin() - m_scale * double(0.5);
    m_farVoxelCount = 0;

    const size_t dimX = m_dim[0];
    const size_t dimY = m_dim[1];
    const size_t dimZ = m_dim[2];
    const size_t voxelCount = dimX * dimY * dimZ;

    std::vector<uint8_t> mask(voxelCount, 0);
    std::vector<uint8_t> solid(voxelCount, 0);
    std::vector<int32_t> field(voxelCount, 0);
    m_runStart.assign(dimX * dimY + 1, 0);
    m_columnSum.assign((dimX + 1) * (dimY + 1), 0);

    // Each pass below walks the whole grid, so each is a step a caller waiting to cancel has to sit out.
    // Voxelizing the mesh and filling it report the first three quarters of this stage.
    auto canceled = [&callbacks](const double progress)
    {
        return callbacks.PollProgress(Stages::VOXELIZING_INPUT_MESH, progress, "Measuring reachable space");
    };

    // The solid: everything the voxelizer marked as surface or interior.
    for (size_t index = 0; index < voxelCount; ++index)
    {
        const VoxelValue value = volume.m_data[index];
        solid[index] = (value == VoxelValue::PRIMITIVE_ON_SURFACE || value == VoxelValue::PRIMITIVE_INSIDE_SURFACE) ? 1 : 0;
    }
    SquaredDistanceTransform(m_dim, solid, field);
    if (canceled(80.0))
    {
        return;
    }

    // Centres where a probe sphere fits clear of every solid voxel cube, flood filled from the padded grid
    // boundary: what the probe cannot reach from outside is filled. Centres sit a voxel apart, so a probe
    // narrower than two cannot be placed where it belongs, and taking it at face value would fill gaps it
    // plainly enters. Two voxels is also the smallest probe a grid can express, so a smaller one is taken
    // as that, and the report says which radius was used. The grid is sized to hold the requested probe
    // and only falls back to this where the voxel budget forced a coarser one.
    const double probe = std::max(probeRadius, double(2.0));
    const double clearance = probe + std::sqrt(double(3.0)) * double(0.5);
    const int32_t clearanceSquared = int32_t(std::ceil(clearance * clearance));
    std::vector<uint32_t> stack;
    std::fill(mask.begin(), mask.end(), uint8_t(0));
    auto visit = [&](const size_t index)
    {
        if (!mask[index] && field[index] >= clearanceSquared)
        {
            mask[index] = 1;
            stack.push_back(uint32_t(index));
        }
    };
    for (size_t i = 0; i < dimX; ++i)
    {
        for (size_t j = 0; j < dimY; ++j)
        {
            for (size_t k = 0; k < dimZ; ++k)
            {
                if (i == 0 || j == 0 || k == 0 || i + 1 == dimX || j + 1 == dimY || k + 1 == dimZ)
                {
                    visit((i * dimY + j) * dimZ + k);
                }
            }
        }
    }
    while (!stack.empty())
    {
        const size_t index = stack.back();
        stack.pop_back();
        const size_t k = index % dimZ;
        const size_t j = (index / dimZ) % dimY;
        const size_t i = index / (dimZ * dimY);
        if (i > 0)          visit(index - dimY * dimZ);
        if (i + 1 < dimX)   visit(index + dimY * dimZ);
        if (j > 0)          visit(index - dimZ);
        if (j + 1 < dimY)   visit(index + dimZ);
        if (k > 0)          visit(index - 1);
        if (k + 1 < dimZ)   visit(index + 1);
    }

    if (canceled(85.0))
    {
        return;
    }

    // The closed solid is the solid itself plus everything the probe sphere does not cover from those
    // centres. The solid belongs to it whatever the probe covers: a ball that reaches a solid voxel's
    // centre from a centre exactly its radius away does not make that voxel free space.
    SquaredDistanceTransform(m_dim, mask, field);
    const int32_t probeSquared = int32_t(std::min(std::floor(probe * probe), double(INT32_MAX)));
    for (size_t index = 0; index < voxelCount; ++index)
    {
        mask[index] = (solid[index] || field[index] > probeSquared) ? 1 : 0;
    }

    if (canceled(90.0))
    {
        return;
    }

    // Far voxels: farther than the tolerance from the closed solid. A hull face can pass between voxel
    // centres, so a voxel one step nearer than the tolerance is left alone. The notches of a voxel
    // staircase, at most sqrt(3) voxels from solid, are kept out of the far set either by the closing,
    // whose probe is at least two voxels wide wherever the grid resolves it, or by the third voxel of
    // margin ChooseGrid leaves where it does not. Tilted solids are not split for their staircase either
    // way.
    SquaredDistanceTransform(m_dim, mask, field);
    const double reach = std::max(tolerance - double(1.0), double(0.0));
    const int32_t reachSquared = int32_t(std::min(std::floor(reach * reach), double(INT32_MAX)));

    if (canceled(95.0))
    {
        return;
    }

    m_runZ.clear();
    for (size_t i = 0; i < dimX; ++i)
    {
        for (size_t j = 0; j < dimY; ++j)
        {
            const size_t column = i * dimY + j;
            const size_t base = column * dimZ;
            m_runStart[column] = uint32_t(m_runZ.size());
            uint32_t count = 0;
            size_t k = 0;
            while (k < dimZ)
            {
                if (field[base + k] > reachSquared)
                {
                    const size_t start = k;
                    while (k + 1 < dimZ && field[base + k + 1] > reachSquared)
                    {
                        ++k;
                    }
                    m_runZ.push_back(uint16_t(start));
                    m_runZ.push_back(uint16_t(k));
                    count += uint32_t(k - start + 1);
                }
                ++k;
            }
            m_columnSum[(i + 1) * (dimY + 1) + (j + 1)] = count;
            m_farVoxelCount += count;
        }
    }
    m_runStart[dimX * dimY] = uint32_t(m_runZ.size());

    for (size_t i = 1; i <= dimX; ++i)
    {
        for (size_t j = 1; j <= dimY; ++j)
        {
            m_columnSum[i * (dimY + 1) + j] += m_columnSum[(i - 1) * (dimY + 1) + j]
                                             + m_columnSum[i * (dimY + 1) + (j - 1)]
                                             - m_columnSum[(i - 1) * (dimY + 1) + (j - 1)];
        }
    }
}

bool SpaceModel::IsFarVoxel(const int32_t i,
                            const int32_t j,
                            const int32_t k) const
{
    if (i < 0 || j < 0 || k < 0
        || uint32_t(i) >= m_dim[0] || uint32_t(j) >= m_dim[1] || uint32_t(k) >= m_dim[2])
    {
        return false;
    }
    const size_t column = size_t(i) * m_dim[1] + size_t(j);
    for (uint32_t run = m_runStart[column]; run < m_runStart[column + 1]; run += 2)
    {
        if (uint32_t(k) < m_runZ[run])
        {
            return false;
        }
        if (uint32_t(k) <= m_runZ[run + 1])
        {
            return true;
        }
    }
    return false;
}

bool SpaceModel::BoxMayHoldFarVoxel(const VHACD::Vect3& low,
                                    const VHACD::Vect3& high) const
{
    if (m_farVoxelCount == 0)
    {
        return false;
    }
    const double invScale = double(1.0) / m_scale;
    const VHACD::Vect3 first = (low - m_origin) * invScale;
    const VHACD::Vect3 last = (high - m_origin) * invScale;
    const double x0 = std::ceil(first.GetX() - double(0.5));
    const double y0 = std::ceil(first.GetY() - double(0.5));
    const double x1 = std::floor(last.GetX() - double(0.5));
    const double y1 = std::floor(last.GetY() - double(0.5));
    if (x1 < double(0.0) || y1 < double(0.0)
        || x0 > double(m_dim[0] - 1) || y0 > double(m_dim[1] - 1)
        || x1 < x0 || y1 < y0)
    {
        return false;
    }
    return RectangleHasFarVoxel(uint32_t(std::max(x0, double(0.0))),
                                uint32_t(std::max(y0, double(0.0))),
                                uint32_t(std::min(x1, double(m_dim[0] - 1))),
                                uint32_t(std::min(y1, double(m_dim[1] - 1))));
}

bool SpaceModel::SegmentReachesTooFar(const VHACD::Vect3& from,
                                      const VHACD::Vect3& to) const
{
    if (m_farVoxelCount == 0)
    {
        return false;
    }
    const double invScale = double(1.0) / m_scale;
    const VHACD::Vect3 start = (from - m_origin) * invScale;
    const VHACD::Vect3 end = (to - m_origin) * invScale;
    const VHACD::Vect3 delta = end - start;
    // One sample per voxel of the longest axis keeps every voxel the segment crosses in reach of a sample.
    const double span = std::max({ std::abs(delta.GetX()), std::abs(delta.GetY()), std::abs(delta.GetZ()) });
    const int32_t steps = int32_t(std::min(std::ceil(span), double(3 * MaxVoxelDimension))) + 1;
    for (int32_t step = 0; step <= steps; ++step)
    {
        const VHACD::Vect3 point = start + delta * (double(step) / double(steps));
        if (IsFarVoxel(int32_t(std::floor(point.GetX())),
                       int32_t(std::floor(point.GetY())),
                       int32_t(std::floor(point.GetZ()))))
        {
            return true;
        }
    }
    return false;
}

bool SpaceModel::RectangleHasFarVoxel(const uint32_t x0,
                                      const uint32_t y0,
                                      const uint32_t x1,
                                      const uint32_t y1) const
{
    const size_t stride = size_t(m_dim[1]) + 1;
    const uint32_t total = m_columnSum[size_t(x1 + 1) * stride + (y1 + 1)]
                         - m_columnSum[size_t(x0) * stride + (y1 + 1)]
                         - m_columnSum[size_t(x1 + 1) * stride + y0]
                         + m_columnSum[size_t(x0) * stride + y0];
    return total != 0;
}

bool SpaceModel::ColumnHasFarVoxelInside(const uint32_t i,
                                         const uint32_t j,
                                         const std::vector<Plane>& planes) const
{
    const size_t column = size_t(i) * m_dim[1] + j;
    const uint32_t runBegin = m_runStart[column];
    const uint32_t runEnd = m_runStart[column + 1];
    if (runBegin == runEnd)
    {
        return false;
    }

    // Clip the line through this column's voxel centres against the hull.
    const double centreX = double(i) + double(0.5);
    const double centreY = double(j) + double(0.5);
    double low = -std::numeric_limits<double>::max();
    double high = std::numeric_limits<double>::max();
    for (const Plane& plane : planes)
    {
        const double slope = plane.m_normal.GetZ();
        const double offset = plane.m_normal.GetX() * centreX + plane.m_normal.GetY() * centreY + plane.m_offset;
        if (slope > double(1e-12))
        {
            high = std::min(high, -offset / slope);
        }
        else if (slope < double(-1e-12))
        {
            low = std::max(low, -offset / slope);
        }
        else if (offset > double(0.0))
        {
            return false;
        }
        if (low > high)
        {
            return false;
        }
    }

    // Voxel k has its centre at k + 0.5, so the centres inside the hull are the k in [low, high] - 0.5.
    const double firstExact = std::ceil(low - double(0.5));
    const double lastExact = std::floor(high - double(0.5));
    if (lastExact < double(0.0) || firstExact > double(m_dim[2] - 1) || lastExact < firstExact)
    {
        return false;
    }
    const uint32_t first = uint32_t(std::max(firstExact, double(0.0)));
    const uint32_t last = uint32_t(std::min(lastExact, double(m_dim[2] - 1)));
    for (uint32_t run = runBegin; run < runEnd; run += 2)
    {
        if (m_runZ[run] > last)
        {
            break;
        }
        if (m_runZ[run + 1] >= first)
        {
            return true;
        }
    }
    return false;
}

bool SpaceModel::ScanRectangle(const uint32_t x0,
                               const uint32_t y0,
                               const uint32_t x1,
                               const uint32_t y1,
                               const std::vector<Plane>& planes) const
{
    if (!RectangleHasFarVoxel(x0, y0, x1, y1))
    {
        return false;
    }
    if (x0 == x1 && y0 == y1)
    {
        return ColumnHasFarVoxelInside(x0, y0, planes);
    }
    // Halve the longer side: the summed counts skip empty regions without visiting their columns.
    if (x1 - x0 >= y1 - y0)
    {
        const uint32_t middle = x0 + (x1 - x0) / 2;
        return ScanRectangle(x0, y0, middle, y1, planes) || ScanRectangle(middle + 1, y0, x1, y1, planes);
    }
    const uint32_t middle = y0 + (y1 - y0) / 2;
    return ScanRectangle(x0, y0, x1, middle, planes) || ScanRectangle(x0, middle + 1, x1, y1, planes);
}

bool SpaceModel::HullReachesTooFar(const std::vector<VHACD::Vertex>& points,
                                   const std::vector<VHACD::Triangle>& triangles) const
{
    if (m_farVoxelCount == 0 || points.size() < 4 || triangles.empty())
    {
        return false;
    }
    const double invScale = double(1.0) / m_scale;
    std::vector<VHACD::Vect3> local;
    local.reserve(points.size());
    for (const VHACD::Vertex& point : points)
    {
        local.push_back((VHACD::Vect3(point) - m_origin) * invScale);
    }
    static_assert(sizeof(VHACD::Triangle) == 3 * sizeof(uint32_t), "Triangle must be three indices");
    return ReachesTooFar(local,
                         &triangles[0].mI0,
                         sizeof(VHACD::Triangle) / sizeof(uint32_t),
                         triangles.size());
}

bool SpaceModel::HullReachesTooFar(const std::vector<VHACD::Vect3>& points,
                                   const std::vector<VHACD::ConvexHullFace>& faces) const
{
    if (m_farVoxelCount == 0 || points.size() < 4 || faces.empty())
    {
        return false;
    }
    const double invScale = double(1.0) / m_scale;
    std::vector<VHACD::Vect3> local;
    local.reserve(points.size());
    for (const VHACD::Vect3& point : points)
    {
        local.push_back((point - m_origin) * invScale);
    }
    static_assert(sizeof(VHACD::ConvexHullFace) >= 3 * sizeof(int32_t), "A face must hold three indices");
    std::vector<uint32_t> indices;
    indices.reserve(faces.size() * 3);
    for (const VHACD::ConvexHullFace& face : faces)
    {
        indices.push_back(uint32_t(face.m_index[0]));
        indices.push_back(uint32_t(face.m_index[1]));
        indices.push_back(uint32_t(face.m_index[2]));
    }
    return ReachesTooFar(local, indices.data(), 3, faces.size());
}

bool SpaceModel::ReachesTooFar(const std::vector<VHACD::Vect3>& local,
                               const uint32_t* const faceIndices,
                               const size_t faceStride,
                               const size_t faceCount) const
{
    // Bounds and centroid of the hull in index space.
    VHACD::Vect3 low(std::numeric_limits<double>::max());
    VHACD::Vect3 high(-std::numeric_limits<double>::max());
    VHACD::Vect3 centroid(0, 0, 0);
    for (const VHACD::Vect3& position : local)
    {
        low = low.CWiseMin(position);
        high = high.CWiseMax(position);
        centroid += position;
    }
    centroid /= double(local.size());

    const double firstX = std::ceil(low.GetX() - double(0.5));
    const double lastX = std::floor(high.GetX() - double(0.5));
    const double firstY = std::ceil(low.GetY() - double(0.5));
    const double lastY = std::floor(high.GetY() - double(0.5));
    if (lastX < double(0.0) || lastY < double(0.0)
        || firstX > double(m_dim[0] - 1) || firstY > double(m_dim[1] - 1)
        || lastX < firstX || lastY < firstY)
    {
        return false;
    }
    const uint32_t x0 = uint32_t(std::max(firstX, double(0.0)));
    const uint32_t x1 = uint32_t(std::min(lastX, double(m_dim[0] - 1)));
    const uint32_t y0 = uint32_t(std::max(firstY, double(0.0)));
    const uint32_t y1 = uint32_t(std::min(lastY, double(m_dim[1] - 1)));

    // Face planes, oriented away from the centroid so the test does not depend on winding.
    std::vector<Plane> planes;
    planes.reserve(faceCount);
    for (size_t face = 0; face < faceCount; ++face)
    {
        const uint32_t* const corner = faceIndices + face * faceStride;
        const VHACD::Vect3& a = local[corner[0]];
        const VHACD::Vect3& b = local[corner[1]];
        const VHACD::Vect3& c = local[corner[2]];
        VHACD::Vect3 normal = (b - a).Cross(c - a);
        const double length = normal.GetNorm();
        if (!(length > double(1e-12)))
        {
            continue;
        }
        normal /= length;
        double offset = -normal.Dot(a);
        if (normal.Dot(centroid) + offset > double(0.0))
        {
            normal = -normal;
            offset = -offset;
        }
        planes.push_back(Plane{ normal, offset });
    }
    if (planes.empty())
    {
        return false;
    }

    return ScanRectangle(x0, y0, x1, y1, planes);
}

// The box around a set of points, as a hull. The convex hull builder refuses a point cloud whose
// thinnest extent is a small enough fraction of its longest, however many voxels that cloud spans, and a
// piece is never nothing: it holds voxels, and the box holds them all, no farther out than its own hull
// would reach along the axes.
inline std::unique_ptr<IVHACD::ConvexHull> BoxHullOfPoints(const std::vector<VHACD::Vertex>& points)
{
    if (points.empty())
    {
        return nullptr;
    }
    const VHACD::BoundsAABB bounds(points);
    const VHACD::Vect3 low = bounds.GetMin();
    const VHACD::Vect3 high = bounds.GetMax();

    std::unique_ptr<IVHACD::ConvexHull> hull(new IVHACD::ConvexHull);
    hull->m_points.reserve(8);
    for (uint32_t corner = 0; corner < 8; ++corner)
    {
        hull->m_points.emplace_back((corner & 1) ? high.GetX() : low.GetX(),
                                    (corner & 2) ? high.GetY() : low.GetY(),
                                    (corner & 4) ? high.GetZ() : low.GetZ());
    }
    // Outward winding, as the hull builder produces.
    hull->m_triangles = { { 0, 2, 1 }, { 1, 2, 3 }, { 4, 5, 6 }, { 5, 7, 6 },
                          { 0, 1, 4 }, { 1, 5, 4 }, { 2, 6, 3 }, { 3, 6, 7 },
                          { 0, 4, 2 }, { 2, 4, 6 }, { 1, 3, 5 }, { 3, 7, 5 } };
    hull->m_volume = VHACD::ComputeMeshVolume(hull->m_points,
                                              hull->m_triangles);
    return hull;
}

//***********************************************************************************************
// QuickHull implementation
//***********************************************************************************************

//////////////////////////////////////////////////////////////////////////
// Quickhull base class holding the hull during construction
//////////////////////////////////////////////////////////////////////////
class QuickHull
{
public:
    // distTol is a fraction of the point cloud's diagonal: points within it of the hull are left out.
    uint32_t ComputeConvexHull(ConvexHullWorkspace& workspace,
                               const std::vector<VHACD::Vertex>& vertices,
                               uint32_t maxHullVertices,
                               double distTol = double(0.0001));

    const std::vector<VHACD::Vertex>& GetVertices() const;
    const std::vector<VHACD::Triangle>& GetIndices() const;

private:
    std::vector<VHACD::Vertex>   m_vertices;
    std::vector<VHACD::Triangle> m_indices;
};

uint32_t QuickHull::ComputeConvexHull(ConvexHullWorkspace& workspace,
                                      const std::vector<VHACD::Vertex>& vertices,
                                      uint32_t maxHullVertices,
                                      const double distTol)
{
    m_indices.clear();

    VHACD::ConvexHull ch(workspace,
                         vertices,
                         distTol,
                         maxHullVertices);

    auto& vlist = ch.GetVertexPool();
    if ( !vlist.empty() )
    {
        size_t vcount = vlist.size();
        m_vertices.resize(vcount);
        std::copy(vlist.begin(),
                  vlist.end(),
                  m_vertices.begin());
    }

    m_indices.reserve(ch.GetFaces().size());
    for (const VHACD::ConvexHullFace& face : ch.GetFaces())
    {
        m_indices.emplace_back(face.m_index[0],
                               face.m_index[1],
                               face.m_index[2]);
    }

    return uint32_t(m_indices.size());
}

const std::vector<VHACD::Vertex>& QuickHull::GetVertices() const
{
    return m_vertices;
}

const std::vector<VHACD::Triangle>& QuickHull::GetIndices() const
{
    return m_indices;
}

//******************************************************************************************
// Implementation of the ShrinkWrap function
//******************************************************************************************

void ShrinkWrap(ConvexHullWorkspace& workspace,
                SimpleMesh& sourceConvexHull,
                const AABBTree& aabbTree,
                uint32_t maxHullVertexCount,
                double distanceThreshold,
                bool doShrinkWrap)
{
    // The projected points are kept apart from the hull until the rebuild succeeds: moving its vertices in
    // place would leave them paired with faces that no longer describe them if the rebuild then failed.
    std::vector<VHACD::Vertex> verts; // New verts for the new convex hull
    verts.reserve(sourceConvexHull.m_vertices.size());
    // Examine each vertex and see if it is within the voxel distance.
    // If it is, then replace the point with the shrinkwrapped / projected point
    for (uint32_t j = 0; j < sourceConvexHull.m_vertices.size(); j++)
    {
        const VHACD::Vertex& p = sourceConvexHull.m_vertices[j];
        VHACD::Vect3 projected(p);
        if (doShrinkWrap)
        {
            VHACD::Vect3 closest;
            if (aabbTree.GetClosestPointWithinDistance(p, distanceThreshold, closest))
            {
                projected = closest;
            }
        }
        verts.emplace_back(projected);
    }
    // Final step is to recompute the convex hull
    VHACD::QuickHull qh;
    uint32_t tcount = qh.ComputeConvexHull(workspace,
                                           verts,
                                           maxHullVertexCount);
    if (tcount)
    {
        sourceConvexHull.m_vertices = qh.GetVertices();
        sourceConvexHull.m_indices = qh.GetIndices();
    }
    else if (doShrinkWrap)
    {
        // The projected points sit on the surface, so they describe it better than the ones they came
        // from, but a set that flat has no hull to build. They are kept with the faces of the box around
        // them, which for a flat piece is the piece: points and faces still describe each other, and a
        // consumer sees the plane it should rather than a slab a voxel thick.
        const std::unique_ptr<IVHACD::ConvexHull> box = BoxHullOfPoints(verts);
        if (box)
        {
            sourceConvexHull.m_vertices = box->m_points;
            sourceConvexHull.m_indices = box->m_triangles;
        }
    }
}

enum class SplitAxis
{
    X_AXIS_NEGATIVE,
    X_AXIS_POSITIVE,
    Y_AXIS_NEGATIVE,
    Y_AXIS_POSITIVE,
    Z_AXIS_NEGATIVE,
    Z_AXIS_POSITIVE,
};

// Splitting halves the longest axis of a piece, so a piece reaches one voxel long before this depth.
// It bounds the recursion should a piece ever fail to shrink.
constexpr uint32_t MaxSplitDepth = 64;


// How much of the tolerance the merge phase spends on simplifying its hulls. The output is reduced to the
// vertex cap regardless, and merging prices every candidate pair, so hulls that carry a vertex per step of
// a voxel staircase cost the decomposition dearly; against that, every vertex dropped moves a hull inward
// and leaves that much of the surface uncovered.
constexpr double kSimplifyFraction = 0.25;

// And never more than this share of a hull's own diagonal. A piece can be far smaller than the tolerance,
// as an 18 cm light is on a 270 m airfield, and simplifying such a piece by the model's tolerance would
// collapse it inside the feature it stands for.
constexpr double kSimplifyHullFraction = 0.05;

// This class represents a collection of voxels, the convex hull
// which surrounds them, and a triangle mesh representation of those voxels
class VoxelHull
{
public:

    // This method constructs a new VoxelHull based on a plane split of the parent
    // convex hull
    VoxelHull(const VoxelHull& parent,
              SplitAxis axis,
              uint32_t splitLoc);

    // Here we construct the initial convex hull around the
    // entire voxel set. Its pieces build their hulls in hullWorkspace and test them against space, both
    // of which must outlive them.
    VoxelHull(Volume& voxels,
              ConvexHullWorkspace& hullWorkspace,
              const SpaceModel& space);

    ~VoxelHull() = default;

    // Helper method to refresh the min/max voxel bounding region
    void MinMaxVoxelRegion(const Voxel &v);

    // Computes the convex hull of the corner points gathered by CollectHullPoints
    void ComputeConvexHull();

    // Returns true if this piece is done: its hull stays within the tolerance of everything the probe
    // sphere can reach, or the piece cannot be split any further.
    bool IsComplete() const;


    // Convert a voxel position into it's correct double precision location
    VHACD::Vect3 GetPoint(const int32_t x,
                                 const int32_t y,
                                 const int32_t z,
                                 const double scale,
                                 const VHACD::Vect3& bmin) const;

    // Gathers the distinct corners of every surface voxel (original and split-plane) as hull input points,
    // in lexicographic order of position. Corners, not voxel centers, are required: a hull of the centers
    // would not enclose the voxels. Interior voxels cannot contribute hull vertices and are skipped.
    void CollectHullPoints();

    // Splits at the midpoint of the longest side of the voxel region
    SplitAxis ComputeSplitPlane(uint32_t& location);

    // This operation is performed in a background thread.
    // It splits the voxels by a plane
    void PerformPlaneSplit();

    SplitAxis               m_axis{ SplitAxis::X_AXIS_NEGATIVE };
    Volume*                 m_voxels{ nullptr }; // The voxelized data set
    ConvexHullWorkspace*    m_hullWorkspace{ nullptr }; // Where this piece builds its hull
    const SpaceModel*       m_space{ nullptr };  // What this piece's hull may and may not cover
    double                  m_voxelScale{ 0 };   // Size of a single voxel
    double                  m_voxelScaleHalf{ 0 }; // 1/2 of the size of a single voxel
    VHACD::BoundsAABB       m_voxelBounds;
    VHACD::Vect3            m_voxelAdjust;       // Minimum coordinates of the voxel space, with adjustment
    uint32_t                m_depth{ 0 };        // How deep in the recursion of the binary tree this hull is
    bool                    m_reachesTooFar{ false }; // The hull covers space no hull may cover, so the piece splits

    std::unique_ptr<IVHACD::ConvexHull> m_convexHull{ nullptr }; // The convex hull which encloses this set of voxels.
    std::vector<Voxel>                  m_surfaceVoxels;     // The voxels which are on the surface of the source mesh.
    std::vector<Voxel>                  m_newSurfaceVoxels;  // Voxels which are on the surface as a result of a plane split
    std::vector<Voxel>                  m_interiorVoxels;    // Voxels which are part of the interior of the hull

    std::unique_ptr<VoxelHull>          m_hullA{ nullptr }; // hull resulting from one side of the plane split
    std::unique_ptr<VoxelHull>          m_hullB{ nullptr }; // hull resulting from the other side of the plane split

    // Defines the coordinates this convex hull comprises within the voxel volume
    // of the entire source
    VHACD::Vector3<uint32_t>                    m_1{ 0 };
    VHACD::Vector3<uint32_t>                    m_2{ 0 };

    std::vector<VHACD::Vertex>                  m_vertices;
};

VoxelHull::VoxelHull(const VoxelHull& parent,
                     SplitAxis axis,
                     uint32_t splitLoc)
    : m_axis(axis)
    , m_voxels(parent.m_voxels)
    , m_hullWorkspace(parent.m_hullWorkspace)
    , m_space(parent.m_space)
    , m_voxelScale(m_voxels->GetScale())
    , m_voxelScaleHalf(m_voxelScale * double(0.5))
    , m_voxelBounds(m_voxels->GetBounds())
    , m_voxelAdjust(m_voxelBounds.GetMin() - m_voxelScaleHalf)
    , m_depth(parent.m_depth + 1)
    , m_1(parent.m_1)
    , m_2(parent.m_2)
{
    // Default copy the voxel region from the parent, but values will
    // be adjusted next based on the split axis and location
    switch ( m_axis )
    {
        case SplitAxis::X_AXIS_NEGATIVE:
            m_2.GetX() = splitLoc;
            break;
        case SplitAxis::X_AXIS_POSITIVE:
            m_1.GetX() = splitLoc + 1;
            break;
        case SplitAxis::Y_AXIS_NEGATIVE:
            m_2.GetY() = splitLoc;
            break;
        case SplitAxis::Y_AXIS_POSITIVE:
            m_1.GetY() = splitLoc + 1;
            break;
        case SplitAxis::Z_AXIS_NEGATIVE:
            m_2.GetZ() = splitLoc;
            break;
        case SplitAxis::Z_AXIS_POSITIVE:
            m_1.GetZ() = splitLoc + 1;
            break;
    }

    // First, we copy all of the interior voxels from our parent
    // which intersect our region
    for (auto& i : parent.m_interiorVoxels)
    {
        VHACD::Vector3<uint32_t> v = i.GetVoxel();
        if (v.CWiseAllGE(m_1) && v.CWiseAllLE(m_2))
        {
            bool newSurface = false;
            switch ( m_axis )
            {
                case SplitAxis::X_AXIS_NEGATIVE:
                    if ( v.GetX() == splitLoc )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::X_AXIS_POSITIVE:
                    if ( v.GetX() == m_1.GetX() )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::Y_AXIS_NEGATIVE:
                    if ( v.GetY() == splitLoc )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::Y_AXIS_POSITIVE:
                    if ( v.GetY() == m_1.GetY() )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::Z_AXIS_NEGATIVE:
                    if ( v.GetZ() == splitLoc )
                    {
                        newSurface = true;
                    }
                    break;
                case SplitAxis::Z_AXIS_POSITIVE:
                    if ( v.GetZ() == m_1.GetZ() )
                    {
                        newSurface = true;
                    }
                    break;
            }
            // If his interior voxels lie directly on the split plane then
            // these become new surface voxels for our patch
            if ( newSurface )
            {
                m_newSurfaceVoxels.push_back(i);
            }
            else
            {
                m_interiorVoxels.push_back(i);
            }
        }
    }
    // Next we copy all of the surface voxels which intersect our region
    for (auto& i : parent.m_surfaceVoxels)
    {
        VHACD::Vector3<uint32_t> v = i.GetVoxel();
        if (v.CWiseAllGE(m_1) && v.CWiseAllLE(m_2))
        {
            m_surfaceVoxels.push_back(i);
        }
    }
    // Our parent's new surface voxels become our new surface voxels so long as they intersect our region
    for (auto& i : parent.m_newSurfaceVoxels)
    {
        VHACD::Vector3<uint32_t> v = i.GetVoxel();
        if (v.CWiseAllGE(m_1) && v.CWiseAllLE(m_2))
        {
            m_newSurfaceVoxels.push_back(i);
        }
    }

    // Recompute the min-max bounding box which would be different after the split occurs
    m_1 = VHACD::Vector3<uint32_t>(0x7FFFFFFF);
    m_2 = VHACD::Vector3<uint32_t>(0);
    for (auto& i : m_surfaceVoxels)
    {
        MinMaxVoxelRegion(i);
    }
    for (auto& i : m_newSurfaceVoxels)
    {
        MinMaxVoxelRegion(i);
    }
    for (auto& i : m_interiorVoxels)
    {
        MinMaxVoxelRegion(i);
    }

    CollectHullPoints();
    ComputeConvexHull();
}

VoxelHull::VoxelHull(Volume& voxels,
                     ConvexHullWorkspace& hullWorkspace,
                     const SpaceModel& space)
    : m_voxels(&voxels)
    , m_hullWorkspace(&hullWorkspace)
    , m_space(&space)
    , m_voxelScale(m_voxels->GetScale())
    , m_voxelScaleHalf(m_voxelScale * double(0.5))
    , m_voxelBounds(m_voxels->GetBounds())
    , m_voxelAdjust(m_voxelBounds.GetMin() - m_voxelScaleHalf)
    // Here we get a copy of all voxels which lie on the surface mesh
    , m_surfaceVoxels(m_voxels->GetSurfaceVoxels())
    // Now we get a copy of all voxels which are considered part of the 'interior' of the source mesh
    , m_interiorVoxels(m_voxels->GetInteriorVoxels())
    , m_2(m_voxels->GetDimensions() - 1)
{
    CollectHullPoints();
    ComputeConvexHull();
}

void VoxelHull::MinMaxVoxelRegion(const Voxel& v)
{
    VHACD::Vector3<uint32_t> x = v.GetVoxel();
    m_1 = m_1.CWiseMin(x);
    m_2 = m_2.CWiseMax(x);
}

void VoxelHull::ComputeConvexHull()
{
    if ( !m_vertices.empty() )
    {
        // we compute the convex hull as follows...
        VHACD::QuickHull qh;
        uint32_t tcount = qh.ComputeConvexHull(*m_hullWorkspace,
                                               m_vertices,
                                               uint32_t(m_vertices.size()));
        if ( tcount )
        {
            m_convexHull = std::unique_ptr<IVHACD::ConvexHull>(new IVHACD::ConvexHull);

            m_convexHull->m_points = qh.GetVertices();
            m_convexHull->m_triangles = qh.GetIndices();

            m_convexHull->m_volume = VHACD::ComputeMeshVolume(m_convexHull->m_points,
                                                              m_convexHull->m_triangles);
        }
        else
        {
            // The builder calls a cloud too thin for its length degenerate and returns nothing. The piece
            // still holds voxels, and dropping it would drop their surface with it, so it takes the box
            // around them; the far test then judges that box like any other hull.
            m_convexHull = BoxHullOfPoints(m_vertices);
        }
    }
    // Whether this hull covers space it may not: farther than the tolerance from anything the probe
    // sphere cannot reach. That, and nothing about volumes, is what makes a piece split.
    m_reachesTooFar = m_convexHull
                   && m_space->HullReachesTooFar(m_convexHull->m_points,
                                                 m_convexHull->m_triangles);
}

bool VoxelHull::IsComplete() const
{
    if ( m_convexHull == nullptr || !m_reachesTooFar || m_depth >= MaxSplitDepth )
    {
        return true;
    }
    // A single voxel's hull holds only its own centre, which is solid, so it can never reach too far and
    // the recursion always ends.
    const VHACD::Vector3<uint32_t> d = m_2 - m_1;
    return d.GetX() == 0 && d.GetY() == 0 && d.GetZ() == 0;
}

VHACD::Vect3 VoxelHull::GetPoint(const int32_t x,
                                 const int32_t y,
                                 const int32_t z,
                                 const double scale,
                                 const VHACD::Vect3& bmin) const
{
    return VHACD::Vect3(x * scale + bmin.GetX(),
                        y * scale + bmin.GetY(),
                        z * scale + bmin.GetZ());
}

void VoxelHull::CollectHullPoints()
{
    if ( m_surfaceVoxels.empty() && m_newSurfaceVoxels.empty() )
    {
        return;
    }

    // Voxel (x, y, z) has the corners x..x+1, y..y+1 and z..z+1.
    uint32_t x0 = UINT32_MAX;
    uint32_t y0 = UINT32_MAX;
    uint32_t z0 = UINT32_MAX;
    uint32_t x1 = 0;
    uint32_t y1 = 0;
    uint32_t z1 = 0;
    for (const std::vector<Voxel>* voxels : { &m_surfaceVoxels, &m_newSurfaceVoxels })
    {
        for (const Voxel& v : *voxels)
        {
            x0 = std::min(x0, v.GetX());
            y0 = std::min(y0, v.GetY());
            z0 = std::min(z0, v.GetZ());
            x1 = std::max(x1, v.GetX());
            y1 = std::max(y1, v.GetY());
            z1 = std::max(z1, v.GetZ());
        }
    }
    const size_t wy = size_t(y1 - y0) + 2;
    const size_t wz = size_t(z1 - z0) + 2;
    const size_t cornerCount = (size_t(x1 - x0) + 2) * wy * wz;

    // One bit per corner at ((x - x0) * wy + (y - y0)) * wz + (z - z0), so ascending bit order is lexicographic
    // (x, y, z) order, which GetPoint turns into lexicographic order of position. Pieces at one split depth
    // have disjoint voxel boxes, so together their bitmaps hold little more than one grid of corners.
    std::vector<uint64_t> corners((cornerCount + 63) / 64);
    const size_t planeStep = wy * wz;
    for (const std::vector<Voxel>* voxels : { &m_surfaceVoxels, &m_newSurfaceVoxels })
    {
        for (const Voxel& v : *voxels)
        {
            const size_t base = (size_t(v.GetX() - x0) * wy + size_t(v.GetY() - y0)) * wz + size_t(v.GetZ() - z0);
            for (const size_t offset : { size_t(0), size_t(1), wz, wz + 1, planeStep, planeStep + 1, planeStep + wz, planeStep + wz + 1 })
            {
                const size_t bit = base + offset;
                corners[bit >> 6] |= uint64_t(1) << (bit & 63);
            }
        }
    }

    // The row (x, y) holds the bits [rowStart, rowStart + wz).
    size_t x = 0;
    size_t y = 0;
    size_t rowStart = 0;
    for (size_t word = 0; word < corners.size(); ++word)
    {
        for (uint64_t bits = corners[word]; bits != 0; bits &= bits - 1)
        {
            const size_t bit = word * 64 + CountTrailingZeros(bits);
            if ( bit >= rowStart + wz )
            {
                const size_t row = bit / wz;
                x = row / wy;
                y = row - x * wy;
                rowStart = row * wz;
            }
            m_vertices.emplace_back(GetPoint(int32_t(x0 + x),
                                             int32_t(y0 + y),
                                             int32_t(z0 + (bit - rowStart)),
                                             m_voxelScale,
                                             m_voxelAdjust));
        }
    }
}

SplitAxis VoxelHull::ComputeSplitPlane(uint32_t& location)
{
    const VHACD::Vector3<uint32_t> d = m_2 - m_1;

    // Voxels at or below location go to the negative side, so location must stay below the region
    // maximum or the positive side is empty. The midpoint formula reaches the maximum only for an extent
    // of 1, and IsComplete already ruled out a piece that spans a single voxel.
    uint32_t axis = 2;
    SplitAxis ret = SplitAxis::Z_AXIS_NEGATIVE;
    if ( d.GetX() >= d.GetY() && d.GetX() >= d.GetZ() )
    {
        axis = 0;
        ret = SplitAxis::X_AXIS_NEGATIVE;
    }
    else if ( d.GetY() >= d.GetX() && d.GetY() >= d.GetZ() )
    {
        axis = 1;
        ret = SplitAxis::Y_AXIS_NEGATIVE;
    }
    // IsComplete returns true for a piece that spans one voxel, so the split axis spans at least two.
    assert(d[axis] >= 1);
    location = std::min((m_2[axis] + 1 + m_1[axis]) / 2, m_2[axis] - 1);
    return ret;
}

void VoxelHull::PerformPlaneSplit()
{
    if ( IsComplete() )
    {
    }
    else
    {
        uint32_t splitLoc;
        SplitAxis axis = ComputeSplitPlane(splitLoc);
        switch ( axis )
        {
            case SplitAxis::X_AXIS_NEGATIVE:
            case SplitAxis::X_AXIS_POSITIVE:
                // Split on the getX axis at this split location
                m_hullA = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::X_AXIS_NEGATIVE, splitLoc));
                m_hullB = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::X_AXIS_POSITIVE, splitLoc));
                break;
            case SplitAxis::Y_AXIS_NEGATIVE:
            case SplitAxis::Y_AXIS_POSITIVE:
                // Split on the 1 axis at this split location
                m_hullA = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Y_AXIS_NEGATIVE, splitLoc));
                m_hullB = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Y_AXIS_POSITIVE, splitLoc));
                break;
            case SplitAxis::Z_AXIS_NEGATIVE:
            case SplitAxis::Z_AXIS_POSITIVE:
                // Split on the getZ axis at this split location
                m_hullA = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Z_AXIS_NEGATIVE, splitLoc));
                m_hullB = std::unique_ptr<VoxelHull>(new VoxelHull(*this, SplitAxis::Z_AXIS_POSITIVE, splitLoc));
                break;
        }
    }
}

// One candidate merge: what combining two hulls would add in volume, which orders the merges, and whether
// the combined hull would cover space no hull may cover, which decides whether it happens at all.
class CostTask
{
public:
    IVHACD::ConvexHull* m_hullA{ nullptr };
    IVHACD::ConvexHull* m_hullB{ nullptr };
    double              m_concavity{ 0 };          // concavity of the two combined
    bool                m_reachesTooFar{ false };  // the combined hull covers space no hull may cover
};

class HullPair
{
public:
    HullPair(uint32_t hullA,
             uint32_t hullB,
             double concavity);

    // std::priority_queue pops its greatest element. The greatest pair has the lowest concavity, then
    // the lowest hull ids, so the merge sequence does not depend on the queue's internal ordering.
    // Concavity is finite, so this is a strict total order.
    bool operator<(const HullPair &h) const;

    uint32_t    m_hullA{ 0 };
    uint32_t    m_hullB{ 0 };
    double      m_concavity{ 0 };
};

HullPair::HullPair(uint32_t hullA,
                   uint32_t hullB,
                   double concavity)
    : m_hullA(hullA)
    , m_hullB(hullB)
    , m_concavity(concavity)
{
    assert(std::isfinite(concavity));
}

bool HullPair::operator<(const HullPair &h) const
{
    if ( m_concavity != h.m_concavity )
    {
        return m_concavity > h.m_concavity;
    }
    if ( m_hullA != h.m_hullA )
    {
        return m_hullA > h.m_hullA;
    }
    return m_hullB > h.m_hullB;
}

class VHACDImpl : public IVHACD, public VHACDCallbacks
{
    // Don't consider more than 100,000 convex hulls.
public:
    VHACDImpl() = default;

    /*
     * Overrides VHACD::IVHACD
     */
    ~VHACDImpl() override
    {
        Clean();
    }

    void Cancel() override final;

    ComputeResult Compute(const float* const points,
                          const uint32_t countPoints,
                          const uint32_t* const triangles,
                          const uint32_t countTriangles,
                          const Parameters& params) override final;

    ComputeResult Compute(const double* const points,
                          const uint32_t countPoints,
                          const uint32_t* const triangles,
                          const uint32_t countTriangles,
                          const Parameters& params) override final;

    const Report& GetReport() const override final;

    uint32_t GetNConvexHulls() const override final;

    bool GetConvexHull(const uint32_t index,
                       ConvexHull& ch) const override final;

    void Clean() override final;  // release internally allocated memory

    void Release() override final;

// private:
    // Validates the caller's arrays and parameters, copies the mesh, and decomposes it
    template <typename Coordinate>
    ComputeResult ComputeFromArrays(const Coordinate* const points,
                                    const uint32_t countPoints,
                                    const uint32_t* const triangles,
                                    const uint32_t countTriangles,
                                    const Parameters& params);

    // Returns nullptr when the input satisfies the Compute contract, otherwise the violated rule
    template <typename Coordinate>
    static const char* ValidateInput(const Coordinate* const points,
                                     const uint32_t countPoints,
                                     const uint32_t* const triangles,
                                     const uint32_t countTriangles,
                                     const Parameters& params);

    // Decomposes a validated mesh; the caller has released previous results
    ComputeResult Compute(const std::vector<VHACD::Vertex>& points,
                          const std::vector<VHACD::Triangle>& triangles,
                          const Parameters& params);

    // This copies the input mesh while scaling the input positions
    // to fit into a normalized unit cube. It also re-indexes all of the
    // vertex positions in case they weren't clean coming in.
    void CopyInputMesh(const std::vector<VHACD::Vertex>& points,
                       const std::vector<VHACD::Triangle>& triangles);

    void ScaleOutputConvexHull(ConvexHull &ch);

    void AddCostToPriorityQueue(CostTask& task);

    void PerformConvexDecomposition();

    // Merging either runs until no pair of hulls may merge, or until the hull budget is met.
    enum class MergeLimit
    {
        Tolerance,
        Budget
    };

    void MergeHulls(MergeLimit limit);

    // A hull of the same points within a quarter of the tolerance, and at most the output's vertex count.
    std::unique_ptr<ConvexHull> SimplifyHull(const ConvexHull& source);

    // Prices a pair and queues it, unless the tolerance rules the merge out.
    void EvaluatePair(ConvexHull* first,
                      ConvexHull* second,
                      MergeLimit limit);

    double ComputeConvexHullVolume(const ConvexHull& sm);

    double ComputeVolume4(const VHACD::Vect3& a,
                          const VHACD::Vect3& b,
                          const VHACD::Vect3& c,
                          const VHACD::Vect3& d);

    double ComputeConcavity(double volumeSeparate,
                            double volumeCombined,
                            double volumeMesh);

    // See if we can compute the cost without having to actually merge convex hulls.
    // If the axis aligned bounding boxes (slightly inflated) of the two convex hulls
    // do not intersect, then we don't need to actually compute the merged convex hull
    // volume.
    bool DoFastCost(CostTask& mt);

    void PerformMergeCostTask(CostTask& mt);

    std::unique_ptr<ConvexHull> ComputeReducedConvexHull(const ConvexHull& ch,
                                                         uint32_t maxVerts,
                                                         bool projectHullVertices);

    // Take the points in convex hull A and the points in convex hull B and generate
    // a new convex hull on the combined set of points.
    // Once completed, we create a SimpleMesh instance to hold the triangle mesh
    // and we compute an inflated AABB for it.
    std::unique_ptr<ConvexHull> ComputeCombinedConvexHull(const ConvexHull& sm1,
                                                          const ConvexHull& sm2);

    // The volume of the hull ComputeCombinedConvexHull would build, with the same arithmetic, without building it
    // Also reports, when asked, whether the combined hull covers space no hull may cover.
    double ComputeCombinedConvexHullVolume(const ConvexHull& sm1,
                                           const ConvexHull& sm2,
                                           bool* reachesTooFar = nullptr);

    // The points of both live hulls, merged in lexicographic order
    std::vector<VHACD::Vertex> MergeHullPoints(const ConvexHull& sm1,
                                               const ConvexHull& sm2) const;

    // Returns the live hull with this id, or nullptr once it has been merged away
    ConvexHull* GetHull(uint32_t id);

    // Assigns the next id and takes ownership; returns the stored hull
    ConvexHull* AddHull(std::unique_ptr<ConvexHull> hull);

    void RemoveHull(uint32_t id);

    const char* GetStageName(Stages stage) const;

    /*
     * Overrides VHACD::VHACDCallbacks
     */
    void ProgressUpdate(Stages stage,
                        double stageProgress,
                        const char* operation) override final;

    bool PollProgress(Stages stage,
                      double stageProgress,
                      const char* operation) override final;

    bool IsCanceled() const override final;

    std::atomic<bool>                                   m_canceled{ false };
    Timer                                               m_progressTimer; // Time since the last IUserCallback::Update
    Parameters                                          m_params; // Convex decomposition parameters

    std::vector<std::unique_ptr<IVHACD::ConvexHull>>    m_convexHulls; // Finalized convex hulls
    std::vector<std::unique_ptr<VoxelHull>>             m_voxelHulls; // completed voxel hulls
    std::vector<std::unique_ptr<VoxelHull>>             m_pendingHulls;

    VHACD::AABBTree                                     m_AABBTree;
    VHACD::Volume                                       m_voxelize;
    // What the probe sphere can reach and how far a hull may stand off it: the whole decomposition's
    // split and merge decisions come from this.
    VHACD::SpaceModel                                   m_space;
    Report                                              m_report;
    // Every hull build of a Compute runs in this workspace, one at a time.
    ConvexHullWorkspace                                 m_hullWorkspace;
    VHACD::Vect3                                        m_center;
    double                                              m_scale{ double(1.0) };
    double                                              m_recipScale{ double(1.0) };
    std::vector<VHACD::Vertex>                          m_vertices;
    std::vector<VHACD::Triangle>                        m_indices;

    double                                              m_overallHullVolume{ double(0.0) };
    double                                              m_voxelScale{ double(0.0) };
    // The tolerance in the normalized space the decomposition works in.
    double                                              m_tolerance{ double(0.0) };
    std::priority_queue<HullPair>                       m_hullPairQueue;
    // Hulls being merged, indexed by id. A merged-away hull leaves a null entry, so iterating the
    // table visits live hulls in creation order.
    std::vector<std::unique_ptr<IVHACD::ConvexHull>>    m_hulls;
    // The points of each hull in m_hulls in lexicographic order, so a combined hull's input is a linear merge that
    // ConvexHull does not sort again. Empty for merged-away hulls.
    std::vector<std::vector<VHACD::Vertex>>             m_sortedHullPoints;
    size_t                                              m_liveHullCount{ 0 };
};

void VHACDImpl::Cancel()
{
    m_canceled = true;
}

IVHACD::ComputeResult VHACDImpl::Compute(const float* const points,
                                         const uint32_t countPoints,
                                         const uint32_t* const triangles,
                                         const uint32_t countTriangles,
                                         const Parameters& params)
{
    return ComputeFromArrays(points, countPoints, triangles, countTriangles, params);
}

IVHACD::ComputeResult VHACDImpl::Compute(const double* const points,
                                         const uint32_t countPoints,
                                         const uint32_t* const triangles,
                                         const uint32_t countTriangles,
                                         const Parameters& params)
{
    return ComputeFromArrays(points, countPoints, triangles, countTriangles, params);
}

template <typename Coordinate>
const char* VHACDImpl::ValidateInput(const Coordinate* const points,
                                     const uint32_t countPoints,
                                     const uint32_t* const triangles,
                                     const uint32_t countTriangles,
                                     const Parameters& params)
{
    if ( params.m_maxConvexHulls == 0 )
    {
        return "m_maxConvexHulls must be at least 1";
    }
    if ( params.m_maxNumVerticesPerCH < 4 )
    {
        return "m_maxNumVerticesPerCH must be at least 4";
    }
    if ( !std::isfinite(params.m_relativeTolerance) || params.m_relativeTolerance <= 0 )
    {
        return "m_relativeTolerance must be finite and positive";
    }
    if ( !std::isfinite(params.m_minTolerance) || params.m_minTolerance <= 0 )
    {
        return "m_minTolerance must be finite and positive";
    }
    if ( !std::isfinite(params.m_maxTolerance) || params.m_maxTolerance < params.m_minTolerance )
    {
        return "m_maxTolerance must be finite and at least m_minTolerance";
    }
    if ( !std::isfinite(params.m_probeRadius) || params.m_probeRadius < 0 )
    {
        return "m_probeRadius must be finite and non-negative";
    }
    if ( params.m_maxVoxels < MinVoxelBudget )
    {
        return "m_maxVoxels must be at least 4096";
    }
    if ( params.m_maxPieces < params.m_maxConvexHulls )
    {
        return "m_maxPieces must be at least m_maxConvexHulls";
    }
    if ( (countPoints != 0 && points == nullptr) || (countTriangles != 0 && triangles == nullptr) )
    {
        return "a non-empty point or triangle array is null";
    }
    const size_t coordinateCount = size_t(countPoints) * 3;
    for (size_t i = 0; i < coordinateCount; ++i)
    {
        if ( !std::isfinite(points[i]) )
        {
            return "a point coordinate is not finite";
        }
    }
    const size_t indexCount = size_t(countTriangles) * 3;
    for (size_t i = 0; i < indexCount; ++i)
    {
        if ( triangles[i] >= countPoints )
        {
            return "a triangle index is not below countPoints";
        }
    }
    return nullptr;
}

template <typename Coordinate>
IVHACD::ComputeResult VHACDImpl::ComputeFromArrays(const Coordinate* const points,
                                                   const uint32_t countPoints,
                                                   const uint32_t* const triangles,
                                                   const uint32_t countTriangles,
                                                   const Parameters& params)
{
    Clean(); // release any previous results
    if ( const char* const violation = ValidateInput(points, countPoints, triangles, countTriangles, params) )
    {
        if ( params.m_logger )
        {
            char scratch[256];
            snprintf(scratch, sizeof(scratch), "VHACD input rejected: %s", violation);
            params.m_logger->Log(scratch);
        }
        return ComputeResult::InvalidInput;
    }

    std::vector<VHACD::Vertex> v;
    v.reserve(countPoints);
    for (size_t i = 0; i < countPoints; ++i)
    {
        v.emplace_back(points[i * 3 + 0],
                       points[i * 3 + 1],
                       points[i * 3 + 2]);
    }

    std::vector<VHACD::Triangle> t;
    t.reserve(countTriangles);
    for (size_t i = 0; i < countTriangles; ++i)
    {
        t.emplace_back(triangles[i * 3 + 0],
                       triangles[i * 3 + 1],
                       triangles[i * 3 + 2]);
    }

    return Compute(v, t, params);
}

const IVHACD::Report& VHACDImpl::GetReport() const
{
    return m_report;
}

uint32_t VHACDImpl::GetNConvexHulls() const
{
    return uint32_t(m_convexHulls.size());
}

bool VHACDImpl::GetConvexHull(const uint32_t index,
                              ConvexHull& ch) const
{
    bool ret = false;

    if ( index < uint32_t(m_convexHulls.size() ))
    {
        ch = *m_convexHulls[index];
        ret = true;
    }

    return ret;
}

void VHACDImpl::Clean()
{
    m_convexHulls.clear();
    m_hulls.clear();
    m_sortedHullPoints.clear();
    // The workspace keeps the capacity of the largest build; release it with the results.
    m_hullWorkspace = ConvexHullWorkspace();
    m_liveHullCount = 0;
    // Pairs refer to hull ids, which a later Compute reuses.
    m_hullPairQueue = std::priority_queue<HullPair>();

    m_voxelHulls.clear();

    m_pendingHulls.clear();

    m_vertices.clear();
    m_indices.clear();
    // The grid and its distance fields are the largest allocation of a Compute; release them too.
    m_voxelize = VHACD::Volume();
    m_space = VHACD::SpaceModel();
}

void VHACDImpl::Release()
{
    delete this;
}

IVHACD::ComputeResult VHACDImpl::Compute(const std::vector<VHACD::Vertex>& points,
                                         const std::vector<VHACD::Triangle>& triangles,
                                         const Parameters& params)
{
    m_params = params;
    m_canceled = false;
    m_progressTimer.Reset();
    m_report = Report();

    CopyInputMesh(points,
                  triangles);
    if ( !m_canceled )
    {
        // We now recursively perform convex decomposition until complete
        PerformConvexDecomposition();
    }

    if ( m_canceled )
    {
        Clean();
        if ( m_params.m_logger )
        {
            m_params.m_logger->Log("VHACD operation canceled before it was complete.");
        }
        return ComputeResult::Canceled;
    }
    return ComputeResult::Completed;
}

void VHACDImpl::CopyInputMesh(const std::vector<VHACD::Vertex>& points,
                              const std::vector<VHACD::Triangle>& triangles)
{
    m_vertices.clear();
    m_indices.clear();
    m_indices.reserve(triangles.size());

    // First we must find the bounding box of this input vertices and normalize them into a unit-cube
    VHACD::Vect3 bmin( FLT_MAX);
    VHACD::Vect3 bmax(-FLT_MAX);
    ProgressUpdate(Stages::COMPUTE_BOUNDS_OF_INPUT_MESH,
                   0,
                   "ComputingBounds");
    for (uint32_t i = 0; i < points.size(); i++)
    {
        const VHACD::Vertex& p = points[i];

        bmin = bmin.CWiseMin(p);
        bmax = bmax.CWiseMax(p);
    }
    ProgressUpdate(Stages::COMPUTE_BOUNDS_OF_INPUT_MESH,
                   100,
                   "ComputingBounds");

    m_center = (bmax + bmin) * double(0.5);

    VHACD::Vect3 scale = bmax - bmin;
    m_scale = scale.MaxCoeff();

    m_recipScale = m_scale > double(0.0) ? double(1.0) / m_scale : double(0.0);

    {
        // Only equal normalized positions are welded. A fuzzy weld deletes real geometry: upstream
        // welded within 0.1% of the model extent, which collapsed the 18 cm lamps of a 270 m airfield
        // into degenerate triangles that produced no voxels at all.
        struct PositionKey
        {
            uint64_t x;
            uint64_t y;
            uint64_t z;

            bool operator==(const PositionKey& other) const
            {
                return x == other.x && y == other.y && z == other.z;
            }
        };
        struct PositionKeyHash
        {
            size_t operator()(const PositionKey& key) const
            {
                uint64_t h = key.x * 0x9E3779B97F4A7C15ull;
                h = (h ^ (h >> 31)) + key.y * 0xC2B2AE3D27D4EB4Full;
                h = (h ^ (h >> 29)) + key.z * 0x165667B19E3779F9ull;
                return size_t(h ^ (h >> 32));
            }
        };
        const auto coordinateBits = [](const double value) {
            // Adding +0.0 turns -0.0 into +0.0, so equal coordinates share one key.
            const double canonical = value + 0.0;
            uint64_t bits;
            memcpy(&bits, &canonical, sizeof(bits));
            return bits;
        };

        std::unordered_map<PositionKey, uint32_t, PositionKeyHash> vertexIndex;
        vertexIndex.reserve(points.size());
        m_vertices.reserve(points.size());
        const auto indexOf = [&](const VHACD::Vertex& p) {
            const VHACD::Vect3 pos = (VHACD::Vect3(p) - m_center) * m_recipScale;
            const PositionKey key{ coordinateBits(pos.GetX()), coordinateBits(pos.GetY()), coordinateBits(pos.GetZ()) };
            const auto inserted = vertexIndex.emplace(key, uint32_t(m_vertices.size()));
            if ( inserted.second )
            {
                m_vertices.emplace_back(pos.GetX(), pos.GetY(), pos.GetZ());
            }
            return inserted.first->second;
        };

        uint32_t dcount = 0;

        for (size_t i = 0; i < triangles.size(); ++i)
        {
            if ( (i & 1023) == 0 && PollProgress(Stages::COMPUTE_BOUNDS_OF_INPUT_MESH, 100.0 * double(i) / double(triangles.size()), "Reindexing input mesh") )
            {
                break;
            }
            const VHACD::Triangle& t = triangles[i];
            const uint32_t i1 = indexOf(points[t.mI0]);
            const uint32_t i2 = indexOf(points[t.mI1]);
            const uint32_t i3 = indexOf(points[t.mI2]);

            if ( i1 == i2 || i1 == i3 || i2 == i3 )
            {
                dcount++;
            }
            else
            {
                m_indices.emplace_back(i1, i2, i3);
            }
        }

        if ( dcount )
        {
            if ( m_params.m_logger )
            {
                char scratch[512];
                snprintf(scratch,
                         sizeof(scratch),
                         "Skipped %u degenerate triangles", dcount);
                m_params.m_logger->Log(scratch);
            }
        }
    }

    // Create the raycast mesh
    if ( !m_canceled )
    {
        ProgressUpdate(Stages::CREATE_RAYCAST_MESH,
                       0,
                       "Building RaycastMesh");
        m_AABBTree = VHACD::AABBTree(m_vertices,
                                     m_indices);
        ProgressUpdate(Stages::CREATE_RAYCAST_MESH,
                       100,
                       "RaycastMesh completed");
    }
    // A mesh whose vertices all sit at one point has nothing to voxelize, and no grid describes it.
    if ( !m_canceled && !(m_scale > double(0.0)) )
    {
        if ( m_params.m_logger )
        {
            m_params.m_logger->Log("VHACD input has no extent; no hulls were produced");
        }
        return;
    }

    if ( !m_canceled )
    {
        ProgressUpdate(Stages::VOXELIZING_INPUT_MESH,
                        0,
                        "Voxelizing Input Mesh");
        // The tolerance follows the size of the model, within the bounds the caller allows, and the voxel
        // size follows the tolerance. Both arrive in the units of the input, while everything below works
        // on the mesh normalized into a unit cube, so both are scaled with it.
        const double diagonal = (bmax - bmin).GetNorm();
        const double tolerance = std::min(std::max(m_params.m_relativeTolerance * diagonal,
                                                   m_params.m_minTolerance),
                                          m_params.m_maxTolerance);
        const GridSizing sizing = ChooseGrid((bmax - bmin) * m_recipScale,
                                             tolerance * m_recipScale,
                                             m_params.m_probeRadius * m_recipScale,
                                             m_params.m_maxVoxels);
        m_voxelize = VHACD::Volume();
        m_voxelize.Voxelize(m_vertices,
                            m_indices,
                            sizing.m_voxelSize,
                            sizing.m_pad,
                            m_params.m_fillMode,
                            m_AABBTree,
                            *this);
        m_voxelScale = m_voxelize.GetScale();

        if ( !m_canceled )
        {
            // A voxel grid resolves nothing below its own voxel, so a budget that coarsened the grid
            // coarsens the tolerance with it. The report carries what was really used.
            const double probeVoxels = m_params.m_probeRadius * m_recipScale / sizing.m_voxelSize;
            const double toleranceVoxels = std::max(tolerance * m_recipScale / sizing.m_voxelSize,
                                                    sizing.m_voxelsPerTolerance);
            m_space.Build(m_voxelize,
                          probeVoxels,
                          toleranceVoxels,
                          *this);

            const VHACD::Vector3<uint32_t> dim = m_voxelize.GetDimensions();
            m_tolerance = toleranceVoxels * sizing.m_voxelSize;
            m_report.m_voxelSize = sizing.m_voxelSize * m_scale;
            m_report.m_tolerance = toleranceVoxels * m_report.m_voxelSize;
            // A probe under two voxels wide is taken as two, the smallest a grid can express.
            m_report.m_probeRadius = std::max(m_params.m_probeRadius, 2.0 * m_report.m_voxelSize);
            m_report.m_voxelCount = dim[0] * dim[1] * dim[2];
            m_report.m_protectedVoxels = m_space.GetFarVoxelCount();
            m_report.m_voxelBudgetBound = sizing.m_budgetBound;
        }

        ProgressUpdate(Stages::VOXELIZING_INPUT_MESH,
                       100,
                       "Voxelization complete");
    }

    if ( !m_canceled )
    {
        ProgressUpdate(Stages::BUILD_INITIAL_CONVEX_HULL,
                        0,
                        "Build initial ConvexHull");
        std::unique_ptr<VoxelHull> vh = std::unique_ptr<VoxelHull>(new VoxelHull(m_voxelize,
                                                                                 m_hullWorkspace,
                                                                                 m_space));
        if ( vh->m_convexHull )
        {
            m_overallHullVolume = vh->m_convexHull->m_volume;
        }
        m_pendingHulls.push_back(std::move(vh));
        ProgressUpdate(Stages::BUILD_INITIAL_CONVEX_HULL,
                       100,
                       "Initial ConvexHull complete");
    }
}

void VHACDImpl::ScaleOutputConvexHull(ConvexHull& ch)
{
    for (uint32_t i = 0; i < ch.m_points.size(); i++)
    {
        VHACD::Vect3 p = ch.m_points[i];
        p = (p * m_scale) + m_center;
        ch.m_points[i] = p;
    }
    ch.m_volume = ComputeConvexHullVolume(ch); // get the combined volume
    VHACD::BoundsAABB b(ch.m_points);
    ch.mBmin = b.GetMin();
    ch.mBmax = b.GetMax();
    ComputeCentroid(ch.m_points,
                    ch.m_triangles,
                    ch.m_center);
}

void VHACDImpl::AddCostToPriorityQueue(CostTask& task)
{
    HullPair hp(task.m_hullA->m_meshId,
                task.m_hullB->m_meshId,
                task.m_concavity);
    m_hullPairQueue.push(hp);
}

void VHACDImpl::PerformConvexDecomposition()
{
    {
        ScopedTime st("Convex Decomposition",
                      m_params.m_logger);
        // Pieces split while their hull reaches too far, so the share of pieces already finished is what
        // there is to report.
        while ( !m_pendingHulls.empty() && !m_canceled )
        {
            // Every split turns one piece into two, so the budget counts the pieces that exist.
            size_t count = m_pendingHulls.size() + m_voxelHulls.size();
            // First we make a copy of the hulls we are processing
            std::vector<std::unique_ptr<VoxelHull>> oldList = std::move(m_pendingHulls);
            // Split every hull on this level that is not yet complete
            for (auto& i : oldList)
            {
                if ( PollProgress(Stages::PERFORMING_DECOMPOSITION, 100.0 * double(m_voxelHulls.size()) / double(count), "Performing recursive decomposition of convex hulls") )
                {
                    return;
                }
                if ( !i->IsComplete() && count < m_params.m_maxPieces )
                {
                    i->PerformPlaneSplit();
                    ++count;
                }
            }
            // Now, we rebuild the pending convex hulls list by
            // adding the two children to the output list if
            // we need to recurse them further
            for (auto& vh : oldList)
            {
                if ( vh->IsComplete() || !vh->m_hullA )
                {
                    m_report.m_pieceBudgetBound = m_report.m_pieceBudgetBound || !vh->IsComplete();
                    if ( vh->m_convexHull )
                    {
                        m_voxelHulls.push_back(std::move(vh));
                    }
                }
                else
                {
                    if ( vh->m_hullA )
                    {
                        m_pendingHulls.push_back(std::move(vh->m_hullA));
                    }
                    if ( vh->m_hullB )
                    {
                        m_pendingHulls.push_back(std::move(vh->m_hullB));
                    }
                }
            }
        }
    }

    if ( !m_canceled )
    {
        m_hulls.clear();
        m_sortedHullPoints.clear();
        m_liveHullCount = 0;

        m_report.m_pieceCount = uint32_t(m_voxelHulls.size());
        ProgressUpdate(Stages::INITIALIZING_CONVEX_HULLS_FOR_MERGING,
                       0,
                       "Initializing ConvexHulls");
        for (auto& vh : m_voxelHulls)
        {
            if ( m_canceled )
            {
                break;
            }
            AddHull(SimplifyHull(*vh->m_convexHull));
        }
        ProgressUpdate(Stages::INITIALIZING_CONVEX_HULLS_FOR_MERGING,
                        100,
                        "ConvexHull initialization complete");

        m_voxelHulls.clear();

        // Merging decides the hull count: pairs merge while the merged hull stays within the tolerance
        // of everything the probe sphere can reach, and what is left when no pair does is the result.
        if ( !m_canceled )
        {
            MergeHulls(MergeLimit::Tolerance);
        }

        if ( !m_canceled && m_liveHullCount > m_params.m_maxConvexHulls )
        {
            // The budget is a hard bound, so the cheapest merges continue past the tolerance to meet it.
            m_report.m_hullBudgetBound = true;
            MergeHulls(MergeLimit::Budget);
        }

        if ( !m_canceled )
        {
            // Output hulls follow the hull table, so their order depends only on creation order.
            ProgressUpdate(Stages::FINALIZING_RESULTS,
                           0,
                           "Finalizing results");
            for (std::unique_ptr<ConvexHull>& hull : m_hulls)
            {
                if ( PollProgress(Stages::FINALIZING_RESULTS, 100.0 * double(&hull - m_hulls.data()) / double(m_hulls.size()), "Finalizing results") )
                {
                    return;
                }
                if ( !hull )
                {
                    continue;
                }
                // We now must reduce the convex hull
                if ( hull->m_points.size() > m_params.m_maxNumVerticesPerCH || m_params.m_shrinkWrap )
                {
                    hull = ComputeReducedConvexHull(*hull,
                                                    m_params.m_maxNumVerticesPerCH,
                                                    m_params.m_shrinkWrap);
                }
                ScaleOutputConvexHull(*hull);
                hull->m_meshId = uint32_t(m_convexHulls.size());
                m_convexHulls.push_back(std::move(hull));
            }
            m_hulls.clear();
            m_sortedHullPoints.clear();
            m_liveHullCount = 0;
            ProgressUpdate(Stages::FINALIZING_RESULTS,
                           100,
                           "Finalized results");
        }
    }
}

void VHACDImpl::EvaluatePair(ConvexHull* const first,
                             ConvexHull* const second,
                             const MergeLimit limit)
{
    // Building a hull for every pair of a large model dominates the decomposition, so the two cheap
    // answers come first: a merged hull lies inside the bounds of the two hulls together, and it covers
    // the line between them.
    if ( limit == MergeLimit::Tolerance )
    {
        // Pieces far enough apart can only merge through solid lying between them, and a chain of merges
        // between neighbours reaches that result anyway, so proximity bounds the pairs a model has to
        // price at all. Far enough means past a filled gap and past the smaller piece's own size: a
        // tighter bound than that costs hulls, since a piece often does merge across something its own
        // size, and a looser one costs cook time without finding more. Over 61 props, this bound gives
        // 14.84 hulls per model at 109 ms against 15.26 at 97 ms without the piece's size, and 14.67 at
        // 136 ms with no bound at all; over the corpus, 14.23 hulls and 42.8 s against 14.45 and 34.7 s,
        // and 14.02 and 68.9 s.
        const double smaller = std::min((first->mBmax - first->mBmin).GetNorm(),
                                        (second->mBmax - second->mBmin).GetNorm());
        const double margin = m_params.m_probeRadius * m_recipScale + m_tolerance + smaller;
        for (int32_t axis = 0; axis < 3; ++axis)
        {
            if ( first->mBmin[axis] - margin > second->mBmax[axis]
                 || second->mBmin[axis] - margin > first->mBmax[axis] )
            {
                return;
            }
        }

        const VHACD::Vect3 low = first->mBmin.CWiseMin(second->mBmin);
        const VHACD::Vect3 high = first->mBmax.CWiseMax(second->mBmax);
        if ( m_space.BoxMayHoldFarVoxel(low, high)
             && m_space.SegmentReachesTooFar(first->m_center, second->m_center) )
        {
            return;
        }
    }

    CostTask task;
    task.m_hullA = first;
    task.m_hullB = second;
    if ( limit == MergeLimit::Budget && DoFastCost(task) )
    {
        // Hulls whose bounds do not meet are priced from those bounds, which the budget is free to accept.
        return;
    }
    PerformMergeCostTask(task);
    if ( limit == MergeLimit::Tolerance && task.m_reachesTooFar )
    {
        return;
    }
    AddCostToPriorityQueue(task);
}

void VHACDImpl::MergeHulls(const MergeLimit limit)
{
    ScopedTime stMerging(limit == MergeLimit::Tolerance ? "Merging Convex Hulls"
                                                        : "Merging Convex Hulls To The Budget",
                         m_params.m_logger);
    // Pairs from an earlier pass priced hulls that no longer exist.
    m_hullPairQueue = std::priority_queue<HullPair>();

    std::vector<ConvexHull*> live;
    live.reserve(m_liveHullCount);
    for (const std::unique_ptr<ConvexHull>& hull : m_hulls)
    {
        if ( hull )
        {
            live.push_back(hull.get());
        }
    }

    ProgressUpdate(Stages::COMPUTING_COST_MATRIX,
                   0,
                   "Computing Hull Merge Cost Matrix");
    for (size_t i = 1; i < live.size(); ++i)
    {
        if ( PollProgress(Stages::COMPUTING_COST_MATRIX, 100.0 * double(i) / double(live.size()), "Computing Hull Merge Cost Matrix") )
        {
            return;
        }
        for (size_t j = 0; j < i && !m_canceled; ++j)
        {
            EvaluatePair(live[i],
                         live[j],
                         limit);
        }
    }
    ProgressUpdate(Stages::COMPUTING_COST_MATRIX,
                   100,
                   "Finished cost matrix");

    const uint32_t startCount = uint32_t(m_liveHullCount);
    std::vector<ConvexHull*> partners;
    while ( !m_hullPairQueue.empty() && !m_canceled )
    {
        if ( limit == MergeLimit::Budget && m_liveHullCount <= m_params.m_maxConvexHulls )
        {
            break;
        }
        const double merged = double(startCount - uint32_t(m_liveHullCount));
        if ( PollProgress(Stages::MERGING_CONVEX_HULLS, startCount > 1 ? merged * 100.0 / double(startCount - 1) : 100.0, "Merging Convex Hulls") )
        {
            return;
        }

        const HullPair pair = m_hullPairQueue.top();
        m_hullPairQueue.pop();

        // A pair is stale once a cheaper merge has consumed either of its hulls.
        ConvexHull* const first = GetHull(pair.m_hullA);
        ConvexHull* const second = GetHull(pair.m_hullB);
        if ( first == nullptr || second == nullptr )
        {
            continue;
        }

        std::unique_ptr<ConvexHull> combined = ComputeCombinedConvexHull(*first,
                                                                        *second);
        RemoveHull(pair.m_hullA);
        RemoveHull(pair.m_hullB);
        // Pairs pushed below carry the combined hull's id, which AddHull assigns as the current table size.
        combined->m_meshId = uint32_t(m_hulls.size());
        ConvexHull* const hull = combined.get();

        // The partners of the new hull are the hulls that survive beside it, gathered before it joins them.
        partners.clear();
        partners.reserve(m_liveHullCount);
        for (const std::unique_ptr<ConvexHull>& secondHull : m_hulls)
        {
            if ( secondHull )
            {
                partners.push_back(secondHull.get());
            }
        }
        // AddHull sorts the combined hull's points, which pricing a pair against it needs.
        AddHull(std::move(combined));
        for (ConvexHull* const partner : partners)
        {
            if ( m_canceled )
            {
                break;
            }
            EvaluatePair(hull,
                         partner,
                         limit);
        }
    }
}

double VHACDImpl::ComputeConvexHullVolume(const ConvexHull& sm)
{
    double totalVolume = 0;
    VHACD::Vect3 bary(0, 0, 0);
    for (uint32_t i = 0; i < sm.m_points.size(); i++)
    {
        VHACD::Vect3 p(sm.m_points[i]);
        bary += p;
    }
    bary /= double(sm.m_points.size());

    for (uint32_t i = 0; i < sm.m_triangles.size(); i++)
    {
        uint32_t i1 = sm.m_triangles[i].mI0;
        uint32_t i2 = sm.m_triangles[i].mI1;
        uint32_t i3 = sm.m_triangles[i].mI2;

        VHACD::Vect3 ver0(sm.m_points[i1]);
        VHACD::Vect3 ver1(sm.m_points[i2]);
        VHACD::Vect3 ver2(sm.m_points[i3]);

        totalVolume += ComputeVolume4(ver0,
                                      ver1,
                                      ver2,
                                      bary);

    }
    totalVolume = totalVolume / double(6.0);
    return totalVolume;
}

double VHACDImpl::ComputeVolume4(const VHACD::Vect3& a,
                                 const VHACD::Vect3& b,
                                 const VHACD::Vect3& c,
                                 const VHACD::Vect3& d)
{
    VHACD::Vect3 ad = a - d;
    VHACD::Vect3 bd = b - d;
    VHACD::Vect3 cd = c - d;
    VHACD::Vect3 bcd = bd.Cross(cd);
    double dot = ad.Dot(bcd);
    return dot;
}

double VHACDImpl::ComputeConcavity(double volumeSeparate,
                                   double volumeCombined,
                                   double volumeMesh)
{
    // Merge order is the only thing this feeds, and a hull of voxel corners always encloses a voxel, so
    // the whole model's hull has volume. Should one ever not, an unranked but finite cost keeps the merge
    // queue's ordering total rather than filling it with NaN.
    return volumeMesh > double(0.0) ? fabs(volumeSeparate - volumeCombined) / volumeMesh : double(0.0);
}

bool VHACDImpl::DoFastCost(CostTask& mt)
{
    bool ret = false;

    ConvexHull* ch1 = mt.m_hullA;
    ConvexHull* ch2 = mt.m_hullB;

    VHACD::BoundsAABB ch1b(ch1->mBmin,
                           ch1->mBmax);
    VHACD::BoundsAABB ch2b(ch2->mBmin,
                           ch2->mBmax);
    if (!ch1b.Intersects(ch2b))
    {
        VHACD::BoundsAABB b = ch1b.Union(ch2b);

        double combinedVolume = b.Volume();
        double concavity = ComputeConcavity(ch1->m_volume + ch2->m_volume,
                                            combinedVolume,
                                            m_overallHullVolume);
        HullPair hp(ch1->m_meshId,
                    ch2->m_meshId,
                    concavity);
        m_hullPairQueue.push(hp);
        ret = true;
    }
    return ret;
}

void VHACDImpl::PerformMergeCostTask(CostTask& mt)
{
    ConvexHull* ch1 = mt.m_hullA;
    ConvexHull* ch2 = mt.m_hullB;

    double volume1 = ch1->m_volume;
    double volume2 = ch2->m_volume;

    double combinedVolume = ComputeCombinedConvexHullVolume(*ch1,
                                                            *ch2,
                                                            &mt.m_reachesTooFar);
    mt.m_concavity = ComputeConcavity(volume1 + volume2,
                                      combinedVolume,
                                      m_overallHullVolume);
}

std::unique_ptr<IVHACD::ConvexHull> VHACDImpl::ComputeReducedConvexHull(const ConvexHull& ch,
                                                                        uint32_t maxVerts,
                                                                        bool projectHullVertices)
{
    SimpleMesh sourceConvexHull;

    sourceConvexHull.m_vertices = ch.m_points;
    sourceConvexHull.m_indices = ch.m_triangles;

    ShrinkWrap(m_hullWorkspace,
               sourceConvexHull,
               m_AABBTree,
               maxVerts,
               m_voxelScale,
               projectHullVertices);

    std::unique_ptr<ConvexHull> ret(new ConvexHull);

    ret->m_points = sourceConvexHull.m_vertices;
    ret->m_triangles = sourceConvexHull.m_indices;

    VHACD::BoundsAABB b = VHACD::BoundsAABB(ret->m_points).Inflate(double(0.1));
    ret->mBmin = b.GetMin();
    ret->mBmax = b.GetMax();

    ret->m_volume = ComputeConvexHullVolume(*ret);

    // Return the convex hull
    return ret;
}

std::vector<VHACD::Vertex> VHACDImpl::MergeHullPoints(const ConvexHull& sm1,
                                                      const ConvexHull& sm2) const
{
    const std::vector<VHACD::Vertex>& points1 = m_sortedHullPoints[sm1.m_meshId];
    const std::vector<VHACD::Vertex>& points2 = m_sortedHullPoints[sm2.m_meshId];
    assert(points1.size() == sm1.m_points.size() && points2.size() == sm2.m_points.size());
    std::vector<VHACD::Vertex> vertices(points1.size() + points2.size());
    std::merge(points1.begin(),
               points1.end(),
               points2.begin(),
               points2.end(),
               vertices.begin(),
               [](const VHACD::Vertex& a, const VHACD::Vertex& b) { return LexicographicallyLess(a, b); });
    return vertices;
}

double VHACDImpl::ComputeCombinedConvexHullVolume(const ConvexHull& sm1,
                                                  const ConvexHull& sm2,
                                                  bool* const reachesTooFar)
{
    const std::vector<VHACD::Vertex> vertices = MergeHullPoints(sm1,
                                                                sm2);
    const VHACD::ConvexHull hull(m_hullWorkspace,
                                 vertices,
                                 double(0.0001),
                                 int(vertices.size()));

    if ( reachesTooFar )
    {
        // A cloud too thin for its length leaves the builder with nothing, and a merge whose shape cannot
        // be built cannot be tested against the space either; refusing it leaves both hulls as they are.
        *reachesTooFar = hull.GetFaces().empty()
                      || m_space.HullReachesTooFar(hull.GetVertexPool(),
                                                   hull.GetFaces());
    }

    // ComputeConvexHullVolume over the points and faces QuickHull would copy out, in the same order.
    const std::vector<VHACD::Vect3>& points = hull.GetVertexPool();
    VHACD::Vect3 bary(0, 0, 0);
    for (const VHACD::Vect3& p : points)
    {
        bary += p;
    }
    bary /= double(points.size());

    double totalVolume = 0;
    for (const VHACD::ConvexHullFace& face : hull.GetFaces())
    {
        totalVolume += ComputeVolume4(points[face.m_index[0]],
                                      points[face.m_index[1]],
                                      points[face.m_index[2]],
                                      bary);
    }
    return totalVolume / double(6.0);
}

std::unique_ptr<IVHACD::ConvexHull> VHACDImpl::SimplifyHull(const ConvexHull& source)
{
    assert(!source.m_points.empty());
    if ( source.m_points.size() < 4 )
    {
        return source.m_triangles.empty() ? BoxHullOfPoints(source.m_points)
                                          : std::unique_ptr<ConvexHull>(new ConvexHull(source));
    }
    // The builder takes its tolerance as a fraction of the cloud's diagonal, and the hull it leaves is
    // inside the one it was given, so a hull never reaches farther for having been simplified.
    const VHACD::BoundsAABB bounds(source.m_points);
    const double diagonal = (bounds.GetMax() - bounds.GetMin()).GetNorm();
    const double relative = diagonal > double(0.0) ? (m_tolerance * kSimplifyFraction) / diagonal : double(0.0001);

    VHACD::QuickHull qh;
    if ( qh.ComputeConvexHull(m_hullWorkspace,
                              source.m_points,
                              m_params.m_maxNumVerticesPerCH,
                              std::clamp(relative, double(0.0001), kSimplifyHullFraction)) == 0 )
    {
        // Merge candidates arrive as points alone, so a refused cloud has no hull to fall back on.
        std::unique_ptr<ConvexHull> box = source.m_triangles.empty()
                                              ? BoxHullOfPoints(source.m_points)
                                              : std::unique_ptr<ConvexHull>(new ConvexHull(source));
        if ( box )
        {
            const VHACD::BoundsAABB b = VHACD::BoundsAABB(box->m_points).Inflate(double(0.1));
            box->mBmin = b.GetMin();
            box->mBmax = b.GetMax();
        }
        return box;
    }

    std::unique_ptr<ConvexHull> ret(new ConvexHull);
    ret->m_points = qh.GetVertices();
    ret->m_triangles = qh.GetIndices();
    ret->m_volume = ComputeConvexHullVolume(*ret);
    const VHACD::BoundsAABB b = VHACD::BoundsAABB(ret->m_points).Inflate(double(0.1));
    ret->mBmin = b.GetMin();
    ret->mBmax = b.GetMax();
    return ret;
}

std::unique_ptr<IVHACD::ConvexHull> VHACDImpl::ComputeCombinedConvexHull(const ConvexHull& sm1,
                                                                         const ConvexHull& sm2)
{
    std::vector<VHACD::Vertex> vertices = MergeHullPoints(sm1,
                                                          sm2);
    ConvexHull combined;
    combined.m_points = std::move(vertices);
    return SimplifyHull(combined);
}

IVHACD::ConvexHull* VHACDImpl::GetHull(uint32_t id)
{
    return id < m_hulls.size() ? m_hulls[id].get() : nullptr;
}

IVHACD::ConvexHull* VHACDImpl::AddHull(std::unique_ptr<ConvexHull> hull)
{
    hull->m_meshId = uint32_t(m_hulls.size());
    // Until ScaleOutputConvexHull replaces it with the centre of mass, m_center holds the average of the
    // hull's points, which lies inside the hull and is what the merge phase tests segments between.
    VHACD::Vect3 centroid(0, 0, 0);
    for (const VHACD::Vertex& point : hull->m_points)
    {
        centroid += VHACD::Vect3(point);
    }
    hull->m_center = hull->m_points.empty() ? centroid : centroid / double(hull->m_points.size());
    std::vector<VHACD::Vertex> sorted = hull->m_points;
    std::sort(sorted.begin(),
              sorted.end(),
              [](const VHACD::Vertex& a, const VHACD::Vertex& b) { return LexicographicallyLess(a, b); });
    m_sortedHullPoints.push_back(std::move(sorted));
    m_hulls.push_back(std::move(hull));
    ++m_liveHullCount;
    return m_hulls.back().get();
}

void VHACDImpl::RemoveHull(uint32_t id)
{
    assert(id < m_hulls.size() && m_hulls[id]);
    m_hulls[id].reset();
    std::vector<VHACD::Vertex>().swap(m_sortedHullPoints[id]);
    --m_liveHullCount;
}

const char* VHACDImpl::GetStageName(Stages stage) const
{
    const char *ret = "unknown";
    switch ( stage )
    {
        case Stages::COMPUTE_BOUNDS_OF_INPUT_MESH:
            ret = "COMPUTE_BOUNDS_OF_INPUT_MESH";
            break;
        case Stages::CREATE_RAYCAST_MESH:
            ret = "CREATE_RAYCAST_MESH";
            break;
        case Stages::VOXELIZING_INPUT_MESH:
            ret = "VOXELIZING_INPUT_MESH";
            break;
        case Stages::BUILD_INITIAL_CONVEX_HULL:
            ret = "BUILD_INITIAL_CONVEX_HULL";
            break;
        case Stages::PERFORMING_DECOMPOSITION:
            ret = "PERFORMING_DECOMPOSITION";
            break;
        case Stages::INITIALIZING_CONVEX_HULLS_FOR_MERGING:
            ret = "INITIALIZING_CONVEX_HULLS_FOR_MERGING";
            break;
        case Stages::COMPUTING_COST_MATRIX:
            ret = "COMPUTING_COST_MATRIX";
            break;
        case Stages::MERGING_CONVEX_HULLS:
            ret = "MERGING_CONVEX_HULLS";
            break;
        case Stages::FINALIZING_RESULTS:
            ret = "FINALIZING_RESULTS";
            break;
        case Stages::NUM_STAGES:
            // Should be unreachable, here to silence enumeration value not handled in switch warnings
            // GCC/Clang's -Wswitch
            break;
    }
    return ret;
}

void VHACDImpl::ProgressUpdate(Stages stage,
                               double stageProgress,
                               const char* operation)
{
    m_progressTimer.Reset();
    if ( m_params.m_callback )
    {
        double overallProgress = (double(stage) * 100) / double(Stages::NUM_STAGES);
        const char *s = GetStageName(stage);
        m_params.m_callback->Update(overallProgress,
                                    stageProgress,
                                    s,
                                    operation);
    }
}

bool VHACDImpl::PollProgress(Stages stage,
                             double stageProgress,
                             const char* operation)
{
    // 10 ms bounds how stale a cancellation request can be before a caller observing it in Update
    // sees it, beyond the longest single unit of work between polls.
    static constexpr double ProgressIntervalSeconds = 0.01;
    if ( m_progressTimer.PeekElapsedSeconds() >= ProgressIntervalSeconds )
    {
        ProgressUpdate(stage, stageProgress, operation);
    }
    return m_canceled;
}

bool VHACDImpl::IsCanceled() const
{
    return m_canceled;
}

IVHACD* CreateVHACD(void)
{
    VHACDImpl *ret = new VHACDImpl;
    return static_cast< IVHACD *>(ret);
}

} // namespace VHACD

#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif // __GNUC__

#endif // ENABLE_VHACD_IMPLEMENTATION

#endif // VHACD_H
