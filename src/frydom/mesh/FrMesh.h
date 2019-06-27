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

#include <iostream>
#include <list>
#include <memory>


#include "fmt/format.h"

#include "OpenMesh/Core/IO/MeshIO.hh"
#include "OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh"

#include "MathUtils/MathUtils.h"

#include "frydom/core/math/FrVector.h"

#include "FrCache.h"


using namespace OpenMesh;

//#define MeshTraits \
//    template <class Base, class Refs> struct MeshT : public Base


namespace frydom {

    class FrInertiaTensor;

    namespace mesh {


        enum VertexPosition {  // N'a a priori de sens que lors de la decoupe... Mettre dans FrMeshClipper ?
            // On pourrait du coup plutot utiliser les fonctions d'ajout dynamique de proprietes !!
            VP_ABOVE_SURFACE = 0,
            VP_ON_SURFACE = 1,
            VP_UNDER_SURFACE = 2,
            VP_UNDEFINED = -1
        };

        enum FacePositionType {
            // Position code is composed of 3 digit AOU where A is the number of vertices above the clipping surface,
            // O the number of vertices on and U the number of vertices under.
            FPT_003 = 3,  // TODO : simplifier la representation de cas en enum, on peut certainemet nommer de maniere unique les cas entierement mouille, sec ou a couper
            //  -----------  totally wet
            //       *
            //      / \
            //     /   \
            //    *-----*
            FPT_012 = 12,
            //   ------*------ totally wet
            //        / \
            //       /   \
            //      *-----*
            FPT_021 = 21,
            //  ----*-----*---- totally wet
            //       \   /
            //        \ /
            //         *
            FPT_030 = 30,
            //  ----*----*----*  Lying on the clipping surface, should be removed

            FPT_102 = 102,
            //         *    Face to clip
            //        / \
            //    ---o---o---
            //      /     \
            //     *-------*
            FPT_111 = 111,
            //          *               *    Face to clip
            //         /|               |\
            //        / |               | \
            //    ---*--o---    or   ---o--*---
            //        \ |               | /
            //         \|               |/
            //          *               *
            FPT_120 = 120,
            //          *  totally dry
            //         / \
            //        /   \
            //   ----*-----*----

            FPT_201 = 201,
            //       *-------*  Face to clip
            //        \     /
            //      ---o---o---
            //          \ /
            //           *
            FPT_210 = 210,
            //        *-----*  totally dry
            //         \   /
            //          \ /
            //       ----*----
            FPT_300 = 300,
            //           *  totally dry
            //          / \
            //         /   \
            //        *-----*
            //
            //   ----------------

            FPT_UNDEFINED = -1
        };

        enum IntegrandType {
            // TODO: voir http://www.drdobbs.com/when-enum-just-isnt-enough-enumeration-c/184403955 pour une meilleure
            // gestion des enums
            UNDEFINED_INTEGRAND,
            POLY_1,
            POLY_X,
            POLY_Y,
            POLY_Z,
            POLY_YZ,
            POLY_XZ,
            POLY_XY,
            POLY_X2,
            POLY_Y2,
            POLY_Z2,
            POLY_X3,
            POLY_Y3,
            POLY_Z3,
            POLY_X2Y,
            POLY_Y2Z,
            POLY_Z2X,
            INFINITE_DEPTH_GREEN_FUNCTION, // TODO : en parler avec Camille et Lucas
            FINITE_DEPTH_GREEN_FUNCTION
        };

        struct FrMeshTraits : public DefaultTraits {
            typedef Vec3d Point;

            VertexAttributes(Attributes::Normal |
                             Attributes::Status
            );

            FaceAttributes(Attributes::Normal |
                           Attributes::TAGGED |
                           Attributes::FEATURE |
                           Attributes::Status
            );

            HalfedgeAttributes(Attributes::PrevHalfedge |
                               Attributes::Status |
                               Attributes::TAGGED
            );

            EdgeAttributes(Attributes::Status);

            EdgeTraits
            {
            private:
                double m_length = 0.0;

            public:
                EdgeT() {}

                const double GetLength() const {
                    return m_length;
                }

                void SetLength(double length) {
                    m_length = length;
                }

            };

            FaceTraits
            {
            private:
                Point m_center = {0.0, 0.0, 0.0};

                struct SurfaceIntegrals {
                    double m_int_1 = 0.; // TODO: calculer int_1, pas fait encore

                    double m_int_x = 0.;
                    double m_int_y = 0.;
                    double m_int_z = 0.;

                    double m_int_yz = 0.;
                    double m_int_xz = 0.;
                    double m_int_xy = 0.;

                    double m_int_x2 = 0.;
                    double m_int_y2 = 0.;
                    double m_int_z2 = 0.;

                    double m_int_x3 = 0.;
                    double m_int_y3 = 0.;
                    double m_int_z3 = 0.;

                    double m_int_x2y = 0.;
                    double m_int_y2z = 0.;
                    double m_int_z2x = 0.;
                };

                SurfaceIntegrals m_integrals;

            public:
                FaceT() {}

                const Point &Center() const { return m_center; }

                /// This function sets the position of the face centroid.
                void SetCenter(const Point &center) { m_center = center; }

                /// This function gives the surface integral of a face.
                const double GetSurfaceIntegral(IntegrandType type) const { // TODO: abandonner les enums pour les integrandes et preferer les accessors voir mieux, des fonctors...
                    switch (type) {
                        case POLY_1:
                            return m_integrals.m_int_1; // This is the surface area...
                        case POLY_X:
                            return m_integrals.m_int_x;
                        case POLY_Y:
                            return m_integrals.m_int_y;
                        case POLY_Z:
                            return m_integrals.m_int_z;
                        case POLY_YZ:
                            return m_integrals.m_int_yz;
                        case POLY_XZ:
                            return m_integrals.m_int_xz;
                        case POLY_XY:
                            return m_integrals.m_int_xy;
                        case POLY_X2:
                            return m_integrals.m_int_x2;
                        case POLY_Y2:
                            return m_integrals.m_int_y2;
                        case POLY_Z2:
                            return m_integrals.m_int_z2;
                        case POLY_X3:
                            return m_integrals.m_int_x3;
                        case POLY_Y3:
                            return m_integrals.m_int_y3;
                        case POLY_Z3:
                            return m_integrals.m_int_z3;
                        case POLY_X2Y:
                            return m_integrals.m_int_x2y;
                        case POLY_Y2Z:
                            return m_integrals.m_int_y2z;
                        case POLY_Z2X:
                            return m_integrals.m_int_z2x;
                    }
                }

                void SetSurfaceIntegral(IntegrandType type, const double &val) {
                    switch (type) {
                        case POLY_1:
                            m_integrals.m_int_1 = val;
                            break;
                        case POLY_X:
                            m_integrals.m_int_x = val;
                            break;
                        case POLY_Y:
                            m_integrals.m_int_y = val;
                            break;
                        case POLY_Z:
                            m_integrals.m_int_z = val;
                            break;
                        case POLY_YZ:
                            m_integrals.m_int_yz = val;
                            break;
                        case POLY_XZ:
                            m_integrals.m_int_xz = val;
                            break;
                        case POLY_XY:
                            m_integrals.m_int_xy = val;
                            break;
                        case POLY_X2:
                            m_integrals.m_int_x2 = val;
                            break;
                        case POLY_Y2:
                            m_integrals.m_int_y2 = val;
                            break;
                        case POLY_Z2:
                            m_integrals.m_int_z2 = val;
                            break;
                        case POLY_X3:
                            m_integrals.m_int_x3 = val;
                            break;
                        case POLY_Y3:
                            m_integrals.m_int_y3 = val;
                            break;
                        case POLY_Z3:
                            m_integrals.m_int_z3 = val;
                            break;
                        case POLY_X2Y:
                            m_integrals.m_int_x2y = val;
                            break;
                        case POLY_Y2Z:
                            m_integrals.m_int_y2z = val;
                            break;
                        case POLY_Z2X:
                            m_integrals.m_int_z2x = val;
                            break;
                        case UNDEFINED_INTEGRAND:
                            std::cerr << "Cannot return value of an UNDEFINED_INTEGRAND" << std::endl;
                            break;
                    }
                }

            };

            VertexTraits
            {
            private:
                VertexPosition m_position = VP_UNDEFINED;
                // Remarque: pour les doubles noeuds, on utilisera les attributs de feature sur les elements
                // Il faut etre capable de specifier les features du maillage de maniere externe ou iterne
                // VTK propose un filre permettat a priori de trouver les features d'un maillage

            public:
                VertexT() {}

                const VertexPosition &Position() const { return m_position; }

                /// This function gives the type of position of the vertex (above/below/on the clipping surface).
                void SetPositionType(VertexPosition vPos) { m_position = vPos; }

                /// This function
                void SetAbove() { m_position = VP_ABOVE_SURFACE; }

                void SetOn() { m_position = VP_ON_SURFACE; }

                void SetUnder() { m_position = VP_UNDER_SURFACE; }

                void SetUndefined() { m_position = VP_UNDEFINED; }

                inline bool IsAbove() const {
                    return m_position == VP_ABOVE_SURFACE;
                }

                inline bool IsOn() const {
                    return m_position == VP_ON_SURFACE;
                }

                inline bool IsUnder() const {
                    return m_position == VP_UNDER_SURFACE;
                }

                inline bool IsUndefined() const {
                    return m_position == VP_UNDEFINED;
                }
            };

        };

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

        class FrMesh;

        namespace meshutils {

            class IncrementalMeshWriter {

            private:
                std::string m_meshFileBase = "output";
                std::string m_extension = ".obj";
                int m_counter = 0;

            public:
                IncrementalMeshWriter() = default;

                void SetFileBase(std::string base);

                void SetFileType(std::string fileType);

                void Reinit(int i);

                void Reinit();

                void operator()(const FrMesh &mesh);
		
                void Write(const FrMesh &mesh);

            private:
                std::string GetFilename() const;

            };

        };

        class FrMesh : public TriMesh_ArrayKernelT<FrMeshTraits> { // FrMesh must be a triangular mesh.

        private:
            meshutils::IncrementalMeshWriter m_writer;

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
                /// This function gives the surface integral of a mesh.
                double GetSurfaceIntegral(IntegrandType type) {
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

                friend class FrMesh;

            };

            mutable FrCache<BoundaryPolygonSurfaceIntegrals> c_polygonSurfaceIntegrals;

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

            /// This function computes the normal vectors everywhere and the centroid of faces.
            void UpdateBaseProperties();

            /// This function updates the computations of the polynomial surface integrals.
            void UpdateFacesPolynomialIntegrals();

            // Computes triangular faces surface integration of some polynomial integrands using analytical formulas
            // established by transforming surface integrals into contour integrals and deriving analytical expressions.
            // Extended from Eberly... TODO: mettre la referece
            void CalcFacePolynomialIntegrals(const FrMesh::FaceHandle &fh);

            double GetVolumeIntegral(IntegrandType type) const;

            BoundingBox GetBoundingBox() const;

            const double GetArea() const;

            const double GetArea(const FaceHandle &fh) const;

            const double GetVolume() const;

            const Position GetCOG() const;

            const Position GetShellCOG() const;

//            const FrInertiaTensor GetPlainInertiaTensorAtCOG(double density) const;
//
//            const FrInertiaTensor GetPlainEqInertiaTensorAtCOG(double mass) const;
//
//            const FrInertiaTensor GetShellInertiaTensorAtCOG(double density, double thickness) const;
//
//            const FrInertiaTensor GetShellEqInertiaTensorAtCOG(double mass, double thickness) const;



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

            /// This function gives the value of a surface integral over the waterline area.
            double GetBoundaryPolygonsSurfaceIntegral(IntegrandType type);

            void UpdateBoundariesSurfacePolynomialIntegrals();

            PolygonSet GetBoundaryPolygonSet();

            void CalcBoundaryPolygonSet();
            // TODO: check pour voir si les polygones obtenus sont bien inscrit dans la surface de coupe
        };

        //TODO: mettre le code portant sur les inerties dans un fichier a part

        struct InertiaTensor {
            mathutils::Vector3d<double> calcPoint = {0., 0., 0.};

            double Ixx = 0.;
            double Iyy = 0.;
            double Izz = 0.;
            double Ixy = 0.;
            double Ixz = 0.;
            double Iyz = 0.;

            std::string ReportString() const;

            void Transport(FrMesh::Point A);

            const mathutils::MatrixMN<double> GetTensorMatrix() const;

        private:

        };

        //TODO : Merge with FrInertia
        struct InertialProperties {

            double m_mass = 0.;
            mathutils::Vector3d<double> m_cog = {0., 0., 0.};
            InertiaTensor m_inertiaTensor;

            std::string ReportString() const;

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
