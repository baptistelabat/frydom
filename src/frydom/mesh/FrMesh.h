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

//#include <iostream>
//#include <list>
//#include <memory>


//#include "fmt/format.h"

//#include "OpenMesh/Core/IO/MeshIO.hh"
//#include "OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh"

#include "MathUtils/MathUtils.h"

#include "frydom/core/math/FrVector.h"

#include "FrCache.h"
#include "FrMeshTraits.h"
#include "FrPolygon.h"
#include "FrIncrementalMeshWriter.h"


using namespace OpenMesh;


namespace frydom {

    class FrInertiaTensor;

    namespace mesh {

        class FrClippingPlane;

        // TODO : ce genre de classe devrait etre placeee dans un namespace "_<class=DMesh>internal" puis typedefee
        // dans la classe utilisatrice (DMesh).
        // Reporter cette regle lors de l'elaboration du guide de bonnes pratiques de programmation FRyDoM
        struct BoundingBox { // TODO: placer cette classe ailleurs
            double xmin = 0.;
            double xmax = 0.;
            double ymin = 0.;
            double ymax = 0.;
            double zmin = 0.;
            double zmax = 0.;
        };


        typedef std::vector<FrPolygon> PolygonSet;

        class FrMesh : public TriMesh_ArrayKernelT<FrMeshTraits> { // FrMesh must be a triangular mesh.

        private:

            meshutils::FrIncrementalMeshWriter m_writer;

            mutable FrCache<PolygonSet> m_polygonSet;

        public:

            /// Constructor of the class.
            FrMesh() = default;

            /// Constructor of the class.
            explicit FrMesh(std::string meshfile);

            /// This function loads the mesh file.
            void Load(std::string meshfile);

            void CreateBox(double Lx, double Ly, double Lz);

            /// This function translates the mesh.
            void Translate(const Point t);

            /// This function rotates the mesh.
            void Rotate(double phi, double theta, double psi);


            void Write(std::string meshfile) const;

            void WriteInc(std::string meshfile, int i);

            void WriteInc();




            /// This function updates all properties of faces and vertices (normals, centroids, surface integrals).
            void UpdateAllProperties();



            BoundingBox GetBoundingBox() const;


            /// This function gives the value of a surface integral over the meshed surface
            double GetMeshedSurfaceIntegral(IntegrandType type);

            double GetMeshedSurfaceIntegral(int iNormal, IntegrandType type);

            const double GetArea(const FaceHandle &fh) const;

            const double GetArea();

            const double GetVolume();

            const Position GetCOG();;



            bool HasBoundaries() const;

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

            PolygonSet GetBoundaryPolygonSet();

        private:


            /// This function computes the normal vectors everywhere and the centroid of faces.
            void UpdateBaseProperties();

            /// This function updates the computations of the polynomial surface integrals.
            void UpdateFacesPolynomialIntegrals();

            // Computes triangular faces surface integration of some polynomial integrands using analytical formulas
            // established by transforming surface integrals into contour integrals and deriving analytical expressions.
            // Extended from Eberly... TODO: mettre la reference
            void CalcFacePolynomialIntegrals(const FrMesh::FaceHandle &fh);


            HalfedgeHandle FindFirstUntaggedBoundaryHalfedge() const;

            void CalcBoundaryPolygonSet();
            // TODO: check pour voir si les polygones obtenus sont bien inscrits dans la surface de coupe


            bool CheckBoundaryPolygon(FrClippingPlane* plane);


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
        template <class Vector>
        inline Vector OpenMeshPointToVector3d(const mesh::FrMesh::Point &point) {
            return Vector(point[0], point[1], point[2]); // Always gives a FRyDoM vector expressed in NWU
        }

        /// Convert a mathutils Vector3d into an OpenMesh point
        inline mesh::FrMesh::Point Vector3dToOpenMeshPoint(const mathutils::Vector3d<double>& vector3d) {
            return {vector3d[0], vector3d[1], vector3d[2]};
        }

    }  // end namespace mesh




}  // end namespace frydom

#endif //FRYDOM_DICE_DMESH_H
