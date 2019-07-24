#include <utility>

//
// Created by lletourn on 16/07/19.
//

#include "FrPolygon.h"
#include "FrMesh.h"
#include "FrMeshClipper.h"

namespace frydom {

    namespace mesh {

        FrPolygon::FrPolygon(const std::vector<Position>& vertexList, FRAME_CONVENTION fc) {

            m_vertexList = vertexList;
            if (IsNED(fc)) {
                for (auto& vertex:m_vertexList)
                    internal::SwapFrameConvention(vertex);
            }

            c_planar = CheckPlanar();

            UpdateBoundariesSurfacePolynomialIntegrals();
        }

        std::vector<Position> FrPolygon::GetVertexList(FRAME_CONVENTION fc) const {

            auto vertexList = m_vertexList;
            if (IsNED(fc)) {
                for (auto& vertex:vertexList)
                    internal::SwapFrameConvention(vertex);
            }

            return vertexList;

//            std::vector<Position> vertexList;
//
//            auto vertex = OpenMeshPointToVector3d<Position>(m_mesh->point(m_mesh->from_vertex_handle(m_polygon[0])));
//            vertexList.push_back(vertex);
//
//            for (auto& heh : m_polygon) {
//                vertex = OpenMeshPointToVector3d<Position>(m_mesh->point(m_mesh->to_vertex_handle(heh)));
//                vertexList.push_back(vertex);
//            }
//
//            return vertexList;
        }

        double FrPolygon::GetArea() const {
            return GetSurfaceIntegral(POLY_1);
        }

        void FrPolygon::UpdateBoundariesSurfacePolynomialIntegrals() {

            assert(IsPlanar());

            double Int1, IntX, IntY, IntXY, IntX2, IntY2;
            Int1 = IntX = IntY = IntXY = IntX2 = IntY2 = 0;

            Position pos0, pos1;
            double x0, x1, y0, y1;
            double dx, dy, px, py, a, b;

            auto vertexInPlane = GetVertexInPlane();

            pos0 = vertexInPlane[0];

            for (int i = 1; i < vertexInPlane.size(); i++) {

                pos1 = vertexInPlane[i];
                x0 = pos0[0];
                y0 = pos0[1];

                x1 = pos1[0];
                y1 = pos1[1];

                dx = x1 - x0;
                dy = y1 - y0;
                px = x0 + x1;
                py = y0 + y1;
                a = x0 * x0 + x1 * x1;
                b = y0 * y0 + y1 * y1;

                Int1 += dy * px;
                IntX += dy * (px * px - x0 * x1);
                IntY += dx * (py * py - y0 * y1);
//                    IntXY += dy * (py * a + 2 * px * (x0 * y0 + x1 * y1));
                IntXY += dy * (py * px*px + y0*x0*x0 + y1*x1*x1);
                IntX2 += dy * a * px;
                IntY2 += dx * b * py;

                pos0 = pos1;

            }

            Int1  /= 2.;
            IntX  /= 6.;
            IntY  /= -6.;
            IntXY /= 24.;
            IntX2 /= 12.;
            IntY2 /= -12.;

            c_surfaceIntegrals = BoundaryPolygonSurfaceIntegrals(Int1, IntX, IntY, IntXY, IntX2, IntY2);

        }

        BoundaryPolygonSurfaceIntegrals FrPolygon::GetSurfaceIntegrals() const {
            return c_surfaceIntegrals;
        }

        double FrPolygon::GetSurfaceIntegral(IntegrandType type) const {
            return c_surfaceIntegrals.GetSurfaceIntegral(type);
        }

//        bool FrPolygon::CheckBoundaryPolygon(FrClippingPlane *plane) const {
//
//            auto valid = !m_polygon.empty();
//
//            for (auto& heh : m_polygon) {
//                auto P1 = m_mesh->point(m_mesh->from_vertex_handle(heh));
//                auto distance = plane->GetDistance(P1);
//                valid &= (distance<1E-8);
//            }
//
//            return false;
//        }

        bool FrPolygon::CheckPlanar() const {

            auto plane = GetPlane();

            bool planar = true;

            for (auto& vertex : m_vertexList){
                auto distance = plane.GetDistanceToPoint(vertex, NWU);
                planar &= (distance<1E-4);
            }

            return planar;
        }

        bool FrPolygon::IsPlanar() const {
            return c_planar;
        }

        std::vector<Position> FrPolygon::GetVertexInPlane() const {
            std::vector<Position> vertexInPlane;

            auto plane = GetPlane();

            for (auto& vertex : m_vertexList){
                auto pos = plane.GetFrame().GetPointPositionInFrame(vertex, NWU);
                vertexInPlane.push_back(pos);
            }

            return vertexInPlane;
        }

        geom::FrPlane FrPolygon::GetPlane() const {
            return geom::FrPlane(m_vertexList, NWU);
        }


    } // end namespace frydom::mesh

} // end namespace frydom