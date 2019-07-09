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

        class FrMesh : public TriMesh_ArrayKernelT<FrMeshTraits> { // FrMesh must be a triangular mesh.

        private:
            meshutils::FrIncrementalMeshWriter m_writer;

            typedef std::vector<HalfedgeHandle> Polygon;
            typedef std::vector<Polygon> PolygonSet;

            mutable FrCache<PolygonSet> m_polygonSet;

            class BoundaryPolygonSurfaceIntegrals {

                private:
                    double m_Int_1  = 0.0;
                    double m_Int_x  = 0.0;
                    double m_Int_y  = 0.0;
                    double m_Int_xy = 0.0;
                    double m_Int_x2 = 0.0;
                    double m_Int_y2 = 0.0;

                public:

                    BoundaryPolygonSurfaceIntegrals() = default;

                    BoundaryPolygonSurfaceIntegrals(double Int_1, double Int_x, double Int_y, double Int_xy, double Int_x2, double Int_y2) {
                        m_Int_1 = Int_1;
                        m_Int_x = Int_x;
                        m_Int_y = Int_y;
                        m_Int_xy = Int_xy;
                        m_Int_x2 = Int_x2;
                        m_Int_y2 = Int_y2;
                    }


                    BoundaryPolygonSurfaceIntegrals(BoundaryPolygonSurfaceIntegrals& integrals) {
                        m_Int_1 = integrals.m_Int_1;
                        m_Int_x = integrals.m_Int_x;
                        m_Int_y = integrals.m_Int_y;
                        m_Int_xy = integrals.m_Int_xy;
                        m_Int_x2 = integrals.m_Int_x2;
                        m_Int_y2 = integrals.m_Int_y2;
                    }

                    /// This function gives the surface integral of a mesh.
                    double GetSurfaceIntegral(IntegrandType type) const {
                        switch (type) {
                            case POLY_1:
                                return m_Int_1;
                            case POLY_X:
                                return m_Int_x;
                            case POLY_Y:
                                return m_Int_y;
                            case POLY_XY:
                                return m_Int_xy;
                            case POLY_X2:
                                return m_Int_x2;
                            case POLY_Y2:
                                return m_Int_y2;
                            default:
                                std::cerr << "No integration rule for integrand of type " << type << " for polygons" << std::endl;
                                exit(1);
                        }

                    }

            };
            mutable FrCache<BoundaryPolygonSurfaceIntegrals> c_polygonSurfaceIntegrals;


            class VolumeIntegrals {

            private:
                double m_Int_1  = 0.0;
                double m_Int_x  = 0.0;
                double m_Int_y  = 0.0;
                double m_Int_z  = 0.0;
                double m_Int_xy = 0.0;
                double m_Int_xz = 0.0;
                double m_Int_yz = 0.0;
                double m_Int_x2 = 0.0;
                double m_Int_y2 = 0.0;
                double m_Int_z2 = 0.0;
                double m_Int_x3 = 0.0;
                double m_Int_y3 = 0.0;
                double m_Int_z3 = 0.0;
                double m_Int_x2y = 0.0;
                double m_Int_y2z = 0.0;
                double m_Int_z2x = 0.0;

            public:

                VolumeIntegrals() = default;

                VolumeIntegrals(double Int_1, double Int_x, double Int_y, double Int_z, double Int_xy, double Int_xz, double Int_yz,
                                double Int_x2, double Int_y2, double Int_z2, double Int_x3, double Int_y3, double Int_z3,
                                double Int_x2y, double Int_y2z, double Int_z2x) {
                    m_Int_1 = Int_1;
                    m_Int_x = Int_x;
                    m_Int_y = Int_y;
                    m_Int_z = Int_z;
                    m_Int_xy = Int_xy;
                    m_Int_xz = Int_xz;
                    m_Int_yz = Int_yz;
                    m_Int_x2 = Int_x2;
                    m_Int_y2 = Int_y2;
                    m_Int_z2 = Int_z2;
                    m_Int_x2 = Int_x3;
                    m_Int_y2 = Int_y3;
                    m_Int_z2 = Int_z3;
                    m_Int_x2y = Int_x2y;
                    m_Int_y2z = Int_y2z;
                    m_Int_z2x = Int_z2x;
                }


                VolumeIntegrals(VolumeIntegrals& integrals) {
                    m_Int_1 = integrals.m_Int_1;
                    m_Int_x = integrals.m_Int_x;
                    m_Int_y = integrals.m_Int_y;
                    m_Int_z = integrals.m_Int_z;
                    m_Int_xy = integrals.m_Int_xy;
                    m_Int_xz = integrals.m_Int_xz;
                    m_Int_yz = integrals.m_Int_yz;
                    m_Int_x2 = integrals.m_Int_x2;
                    m_Int_y2 = integrals.m_Int_y2;
                    m_Int_z2 = integrals.m_Int_z2;
                    m_Int_x2 = integrals.m_Int_x3;
                    m_Int_y2 = integrals.m_Int_y3;
                    m_Int_z2 = integrals.m_Int_z3;
                    m_Int_x2y = integrals.m_Int_x2y;
                    m_Int_y2z = integrals.m_Int_y2z;
                    m_Int_z2x = integrals.m_Int_z2x;
                }

                /// This function gives the surface integral of a mesh.
                double GetSurfaceIntegral(IntegrandType type) const {
                    switch (type) {
                        case POLY_1:
                            return m_Int_1;
                        case POLY_X:
                            return m_Int_x;
                        case POLY_Y:
                            return m_Int_y;
                        case POLY_Z:
                            return m_Int_z;
                        case POLY_XY:
                            return m_Int_xy;
                        case POLY_XZ:
                            return m_Int_xz;
                        case POLY_YZ:
                            return m_Int_yz;
                        case POLY_X2:
                            return m_Int_x2;
                        case POLY_Y2:
                            return m_Int_y2;
                        case POLY_Z2:
                            return m_Int_z2;
                        case POLY_X3:
                            return m_Int_x3;
                        case POLY_Y3:
                            return m_Int_y3;
                        case POLY_Z3:
                            return m_Int_z3;
                        case POLY_X2Y:
                            return m_Int_x2y;
                        case POLY_Y2Z:
                            return m_Int_y2z;
                        case POLY_Z2X:
                            return m_Int_z2x;
                        default:
                            std::cerr << "No integration rule for integrand of type " << type << " for polygons" << std::endl;
                            exit(1);
                    }

                }

            };
            mutable FrCache<VolumeIntegrals> c_volumeIntegrals;

            mutable FrCache<double> c_meshArea;

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

            /// This function updates the computations of the polynomial surface integrals.
            void UpdateFacesPolynomialIntegrals();

            // Computes triangular faces surface integration of some polynomial integrands using analytical formulas
            // established by transforming surface integrals into contour integrals and deriving analytical expressions.
            // Extended from Eberly... TODO: mettre la referece
            void CalcFacePolynomialIntegrals(const FrMesh::FaceHandle &fh);




            double GetVolumeIntegral(IntegrandType type) const;

            /// This function gives the value of a surface integral over the waterline area.
            double GetBoundaryPolygonsSurfaceIntegral(IntegrandType type);




            BoundingBox GetBoundingBox() const;

            const double GetArea() const;

            const double GetArea(const FaceHandle &fh) const;

            const double GetVolume() const;

            const Position GetCOG() const;

            const Position GetCOG(FrClippingPlane* plane);




            bool HasBoundaries() const;

            bool IsWatertight() const;

            HalfedgeHandle FindFirstUntaggedBoundaryHalfedge() const;


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


            void CalcBoundaryPolygonSet();
            // TODO: check pour voir si les polygones obtenus sont bien inscrits dans la surface de coupe


            bool CheckBoundaryPolygon(FrClippingPlane* plane);

            /// This function computes the normal vectors everywhere and the centroid of faces.
            void UpdateBaseProperties();

            void UpdateBoundariesSurfacePolynomialIntegrals();

            void UpdateVolumeIntegrals();


        };


        FrInertiaTensor CalcPlainInertiaProperties(const FrMesh &mesh, double density);

        FrInertiaTensor CalcPlainEqInertiaProperties(const FrMesh &mesh, double mass);

        FrInertiaTensor CalcShellInertiaProperties(const FrMesh &mesh, double density, double thickness);

        FrInertiaTensor CalcShellEqInertiaProperties(const FrMesh &mesh, double mass, double thickness);

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
