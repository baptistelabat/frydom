// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#ifndef FRYDOM_DICE_DMESH_H
#define FRYDOM_DICE_DMESH_H

#include "MathUtils/MathUtils.h"

#include "frydom/core/math/FrVector.h"

#include "FrCache.h"
#include "FrMeshTraits.h"
#include "FrPolygon.h"
#include "FrIncrementalMeshWriter.h"


using namespace OpenMesh;


namespace frydom {

  class FrInertiaTensor;
    class FrTriangleMeshConnected;

  namespace mesh {

    class FrClippingPlane;

    // TODO : ce genre de classe devrait etre placeee dans un namespace "_<class=DMesh>internal" puis typedefee
    // dans la classe utilisatrice (DMesh).
    // Reporter cette regle lors de l'elaboration du guide de bonnes pratiques de programmation FRyDoM
    /**
     * Class for an Axis Aligned Bounding Box (AABB)
     */
    struct FrAABoundingBox { // TODO: placer cette classe ailleurs
      double xmin = 0.;
      double xmax = 0.;
      double ymin = 0.;
      double ymax = 0.;
      double zmin = 0.;
      double zmax = 0.;
    };


    typedef std::vector<FrPolygon> PolygonSet;

    /**
     * Class for a triangular mesh, derived from OpenMesh::TriMesh_ArrayKernelT templated with FrMeshTraits
     */
    class FrMesh : public TriMesh_ArrayKernelT<FrMeshTraits> { // FrMesh must be a triangular mesh.

     private:

      meshutils::FrIncrementalMeshWriter m_writer;    ///< mesh writer

      mutable FrCache<PolygonSet> m_polygonSet;       ///< set of boundary polygons delimiting the surface mesh

     public:

      /// Constructor of the class.
      FrMesh() = default;

      /// Constructor of the class.
      explicit FrMesh(std::string meshfile);

      /// This function loads the mesh file.
      void Load(std::string meshfile);

      /// Create a meshed box
      /// \param Lx length
      /// \param Ly width
      /// \param Lz height
      void CreateBox(double Lx, double Ly, double Lz);

      /// This function translates the mesh.
      void Translate(const Point t);

      /// This function rotates the mesh, based on Cardan angles
      void Rotate(double phi, double theta, double psi);

      /// Rotates the mesh, based on a rotation matrix
      /// \param Rot_matrix
      void Rotate(const mathutils::Matrix33<double> &Rot_matrix);

      /// Write the mesh in an output file
      /// \param meshfile name of the output file
      void Write(std::string meshfile) const;

      /// Write the file in an output file, by incrementing it
      /// \param meshfile name of the output file
      /// \param i indice of the increment
      void WriteInc(std::string meshfile, int i);

      /// Write the file in an output file, by incrementing it
      void WriteInc();

            std::shared_ptr<frydom::FrTriangleMeshConnected> ConvertToTriangleMeshConnected();

      /// This function updates all properties of faces and vertices (normals, centroids, surface integrals).
      void UpdateAllProperties();


      /// Get the set of boundary polygons delimiting the surface mesh
      /// \returnset of boundary polygons
      PolygonSet GetBoundaryPolygonSet();

      /// Get the Axis Aligned Bounding Box (AABB) of the mesh
      /// \return AABB
      FrAABoundingBox GetBoundingBox() const;


      /// Get the value of a surface integral over the meshed surface : iint_S type ds
      /// \param type integrand type
      /// \return surface integral of the integrand over the mesh surface
      double GetMeshedSurfaceIntegral(IntegrandType type);

      /// Get the value of a surface integral over the meshed surface : iint_S type . n(iNormal) ds
      /// \param iNormal indice of the normal component [0:2]
      /// \param type integrand type
      /// \return surface integral of the integrand over the mesh
      double GetMeshedSurfaceIntegral(int iNormal, IntegrandType type);

      /// Get the area of the triangular element fh
      /// \param fh triangular element
      /// \return area (m^2)
      const double GetArea(const FaceHandle &fh) const;
            /// Get the area of the meshed surface
            /// \return area (m^2)
            const double GetMeshedSurfaceArea();

      /// Get the area of the surface mesh, closed by the set of boundary polygons
      /// \return area (m^2)
      const double GetArea();

      /// Get the volume delimited by the surface mesh and the set of boundary polygons
      /// \return volume (m^3)
      const double GetVolume();

      /// Get the position of the center of gravity of the domain delimited by the surface mesh and the set of
      /// boundary polygons, under the assumption of uniform weight distribution.
      /// \return center of gravity (m)
      const Position GetCOG();
            const FrInertiaTensor GetPlainInertiaTensor(double density);

            const FrInertiaTensor GetShellInertiaTensor(double density, double thickness);


      /// Check if the surface mesh has boundaries
      /// \return true if the surface mesh has boundaries
      bool HasBoundaries() const;

      /// Check if the domain delimited by the surface mesh and the set of boundary polygons is water tight
      /// \return true if the domain is water tight
      bool IsWatertight() const;


      //            bool AreBoundariesClipping() {
//                // TODO
//                // On verifie que les polygones frontieres ont ete generees
//
//
//                // S'il n'y en a aucune (watertight), on retourne false
//
//                // Pour chaque polygone, on appelle la methode de verif prenant en entree la FrClippingSurface
//
//            }

     private:


      /// This function computes the normal vectors everywhere and the centroid of faces.
      void UpdateBaseProperties();

      /// This function updates the computations of the polynomial surface integrals.
      void UpdateFacesPolynomialIntegrals();

      /// Computes triangular faces surface integration of some polynomial integrands using analytical formulas
      /// established by transforming surface integrals into contour integrals and deriving analytical expressions.
      /// Extended from Eberly... https://d-ice.gitlab.host/common/technical_reports/mesh-integrals
      void CalcFacePolynomialIntegrals(const FrMesh::FaceHandle &fh);

      /// Find the first untagged boundary halfedge
      /// \return first untagged boundary halfedge
      HalfedgeHandle FindFirstUntaggedBoundaryHalfedge() const;

      /// Compute the set of boundary polygons
      void CalcBoundaryPolygonSet();

      /// Check that the set of boundary polygons is located on the given plane
      /// \param plane reference plane
      /// \return true if the set of polygons is on the plane
      bool CheckBoundaryPolygon(FrClippingPlane *plane);


    };

//        double GetMeshedSurfaceIntegral(const FrMesh &mesh, int iNormal, IntegrandType type);
//
//        double CalcVolumeIntegrals(FrMesh &mesh, IntegrandType type);
//
//        FrInertiaTensor CalcPlainInertiaProperties(const FrMesh &mesh, double density);
//
//        FrInertiaTensor CalcPlainEqInertiaProperties(const FrMesh &mesh, double mass);
//
//        FrInertiaTensor CalcShellInertiaProperties(const FrMesh &mesh, double density, double thickness);
//
//        FrInertiaTensor CalcShellEqInertiaProperties(const FrMesh &mesh, double mass, double thickness);

    /// Convert an OpenMesh point into a FRyDoM Vector
    template<class Vector>
    inline Vector OpenMeshPointToVector3d(const mesh::FrMesh::Point &point) {
      return Vector(point[0], point[1], point[2]); // Always gives a FRyDoM vector expressed in NWU
    }

    /// Convert a mathutils Vector3d into an OpenMesh point
    inline mesh::FrMesh::Point Vector3dToOpenMeshPoint(const mathutils::Vector3d<double> &vector3d) {
      return {vector3d[0], vector3d[1], vector3d[2]};
    }

  }  // end namespace mesh




}  // end namespace frydom

#endif //FRYDOM_DICE_DMESH_H
