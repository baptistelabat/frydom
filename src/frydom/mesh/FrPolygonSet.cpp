//
// Created by lletourn on 16/07/19.
//

#include "FrPolygonSet.h"
#include "FrMesh.h"

namespace frydom {

    namespace mesh {

        FrPolygonSet::FrPolygonSet(FrMesh *mesh) : m_mesh(mesh){

        }

        PolygonSet FrPolygonSet::Get() const {
            return m_polygonSet;
        }

        bool FrPolygonSet::IsValid() const {
            return m_polygonSet.IsValid();
        }

        void FrPolygonSet::UpdateBoundariesSurfacePolynomialIntegrals() {

            double Int1, IntX, IntY, IntXY, IntX2, IntY2;
            Int1 = IntX = IntY = IntXY = IntX2 = IntY2 = 0;

            PolygonSet polygonSet = m_polygonSet.Get();

            FrMesh::Point P0, P1;
            double x0, x1, y0, y1;
            double dx, dy, px, py, a, b;

            for (auto& polygon : polygonSet) {

                P0 = m_mesh->point(m_mesh->from_vertex_handle(polygon[0]));

                for (const HalfedgeHandle &heh : polygon) {

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

            }

            Int1  /= 2.;
            IntX  /= 6.;
            IntY  /= -6.;
            IntXY /= 24.;
            IntX2 /= 12.;
            IntY2 /= -12.;

            c_polygonSurfaceIntegrals.Set(BoundaryPolygonSurfaceIntegrals(Int1, IntX, IntY, IntXY, IntX2, IntY2));

        }



    } // end namespace frydom::mesh

} // end namespace frydom