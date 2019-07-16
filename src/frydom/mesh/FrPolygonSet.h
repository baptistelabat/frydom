//
// Created by lletourn on 16/07/19.
//

#ifndef FRYDOM_FRPOLYGONSET_H
#define FRYDOM_FRPOLYGONSET_H

#include "FrMeshTraits.h"
#include "FrCache.h"

namespace frydom {

    namespace mesh {

        typedef std::vector<HalfedgeHandle> Polygon;
        typedef std::vector<Polygon> PolygonSet;

        // Forward Declarations
        class FrMesh;

        class FrPolygonSet {

        public:

            explicit FrPolygonSet(FrMesh* mesh);

            explicit FrPolygonSet(PolygonSet set);

            PolygonSet Get() const;

            bool IsValid() const;

        private:

            FrMesh* m_mesh;

            void UpdateBoundariesSurfacePolynomialIntegrals();

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

        };

    } // end namespace frydom::mesh

} // end namespace frydom
#endif //FRYDOM_FRPOLYGONSET_H
