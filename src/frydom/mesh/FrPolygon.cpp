#include <utility>

//
// Created by lletourn on 16/07/19.
//

#include "FrPolygon.h"
#include "FrMesh.h"
#include "FrMeshClipper.h"
#include "FrPlane.h"

namespace frydom {

    namespace mesh {

        FrPolygon::FrPolygon(FrMesh_ *mesh, Polygon polygon) : m_mesh(mesh), m_polygon(std::move(polygon)) {
            c_surfaceIntegrals = BoundaryPolygonSurfaceIntegrals(0,0,0,0,0,0);
            UpdateBoundariesSurfacePolynomialIntegrals();
        }

        Polygon FrPolygon::GetPolygon() const {
            return m_polygon;
        }

        std::vector<Position> FrPolygon::GetVertexList() const {
            std::vector<Position> vertexList;

            for (auto& heh : m_polygon) {
                auto vertex = OpenMeshPointToVector3d<Position>(m_mesh->point(m_mesh->from_vertex_handle(heh)));
                vertexList.push_back(vertex);
            }

            return vertexList;
        }

        double FrPolygon::GetArea() const {

            CheckPlanar();

            return GetSurfaceIntegral(POLY_1);

        }

        void FrPolygon::UpdateBoundariesSurfacePolynomialIntegrals() {

            CheckPlanar();

            double Int1, IntX, IntY, IntXY, IntX2, IntY2;
            Int1 = IntX = IntY = IntXY = IntX2 = IntY2 = 0;

            Polygon polygonSet = m_polygon;

            FrMesh_::Point P0, P1;
            double x0, x1, y0, y1;
            double dx, dy, px, py, a, b;

            P0 = m_mesh->point(m_mesh->from_vertex_handle(m_polygon[0]));

            for (const HalfedgeHandle &heh : m_polygon) {

                P1 = m_mesh->point(m_mesh->to_vertex_handle(heh));

                x0 = P0[0];
                y0 = P0[1];

                x1 = P1[0];
                y1 = P1[1];

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

                P0 = P1;

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

        bool FrPolygon::CheckBoundaryPolygon(FrClippingPlane *plane) const {

            auto valid = !m_polygon.empty();

            for (auto& heh : m_polygon) {
                auto P1 = m_mesh->point(m_mesh->from_vertex_handle(heh));
                auto distance = plane->GetDistance(P1);
                valid &= (distance<1E-8);
            }

            return false;
        }

        bool FrPolygon::CheckPlanar() const {

            std::vector<Position> vertexList = GetVertexList();

            geom::FrPlane plane(vertexList, NWU);

            bool planar = true;

            for (auto& vertex : vertexList){
                auto distance = plane.GetDistanceToPoint(vertex, NWU);
                planar &= (distance<1E-8);
            }

            return planar;
        }

        bool FrPolygon::IsPlanar() const {
            return c_planar;
        }


    } // end namespace frydom::mesh

} // end namespace frydom