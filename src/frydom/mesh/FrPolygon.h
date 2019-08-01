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

#ifndef FRYDOM_FRPOLYGONSET_H
#define FRYDOM_FRPOLYGONSET_H

#include "FrMeshTraits.h"
#include "FrCache.h"
#include "FrPlane.h"

namespace frydom {

    // Forward Declarations
    class Position;

    namespace mesh {

        typedef std::vector<HalfedgeHandle> Polygon;

        class PolygonSurfaceIntegrals {

        private:
            double m_Int_1  = 0.0;
            double m_Int_x  = 0.0;
            double m_Int_y  = 0.0;
            double m_Int_xy = 0.0;
            double m_Int_x2 = 0.0;
            double m_Int_y2 = 0.0;

        public:

            PolygonSurfaceIntegrals() = default;

            PolygonSurfaceIntegrals(double Int_1, double Int_x, double Int_y, double Int_xy, double Int_x2, double Int_y2) {
                m_Int_1 = Int_1;
                m_Int_x = Int_x;
                m_Int_y = Int_y;
                m_Int_xy = Int_xy;
                m_Int_x2 = Int_x2;
                m_Int_y2 = Int_y2;
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


        /**
         * Class for a 2D Axis Aligned Bounding Box (AABB)
         */
        struct Fr2DAABB { // TODO: placer cette classe ailleurs
            double xmin = 0.;
            double xmax = 0.;
            double ymin = 0.;
            double ymax = 0.;
        };

        /**
         * Class for a polygon, based on a set of vertices.
         * Surface integrals over the surface delimited by the polygon can be computed
         */
        class FrPolygon {

        public:

            /// Polygon constructor based on a set of vertices
            /// \param vertexList list of vertices
            /// \param fc frame convention (NED/NWU)
            FrPolygon(const std::vector<Position>& vertexList, FRAME_CONVENTION fc);

            /// Check if the polygon is planar
            /// \return true if the polygon is planar
            bool IsPlanar() const;

            /// Get the set of vertices
            /// \param fc frame convention (NED/NWU)
            /// \return set of vertices
            std::vector<Position> GetVertexList(FRAME_CONVENTION fc) const;

            /// Get the area of the surface delimited by the polygon
            /// \return area (m^2)
            double GetArea() const;

            /// Get all the surface integrals computed
            /// \return surface integrals
            PolygonSurfaceIntegrals GetSurfaceIntegrals() const;

            /// Get the surface integral on the surface delimited by the polygon of the integrand : iint_S type dS
            /// \param type integrand
            /// \return surface integral
            double GetSurfaceIntegral(IntegrandType type) const;

            /// Get the plane passing through all the vertices, (be careful to check that your polygon is planar)
            /// \return plane related to the polygon
            geom::FrPlane GetPlane() const;

            /// Get the 2D Axis Aligned Bounding Box of the polygon
            /// \return
            Fr2DAABB GetBoundingBox() const;

        private:

            /// Compute the surface integrals over the surface delimited by the polygon
            void ComputeSurfacePolynomialIntegrals();

            /// Check if the polygon is planar
            /// \return true if the polygon is planar
            bool CheckPlanar() const;

            /// Get the position of the vertices in the plane reference frame
            /// \return vertices in the plane reference frame
            std::vector<Position> GetVertexInPlane() const;

            PolygonSurfaceIntegrals c_surfaceIntegrals; ///< cached values of the surface integrals over the surface
                                                        ///< delimited by the polygon

            bool c_planar;                              ///< cached boolean to check if the polygon is planar

            std::vector<Position> m_vertexList;         ///> set of vertices defining the polygon

        };

    } // end namespace frydom::mesh

} // end namespace frydom
#endif //FRYDOM_FRPOLYGONSET_H
