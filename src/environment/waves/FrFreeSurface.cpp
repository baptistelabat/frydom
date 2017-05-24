// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// Base class for a water free surface system
//
// =============================================================================

//#include <math.h>
#include "FrFreeSurface.h"

namespace frydom {

    FrFreeSurface::FrFreeSurface(double p_mean_height) : m_mean_height(p_mean_height){
        plane.pos[1] = p_mean_height;  // The free surface plane reference has the altitude the mean FS height
    }

    double FrFreeSurface::getMeanHeight() const {
       return m_mean_height;
    }

    void FrFreeSurface::Initialize(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

        // Making sure the mesh is clean
        mesh.Clear();

        uint nv_x = uint((xmax - xmin) / dx) + 1;
        uint nv_y = uint((ymax - ymin) / dy) + 1;

        uint nb_vertices = nv_x * nv_y;

        // TODO: test nb_vertices against size of data wrt memory usage
        // FIXME: devrait etre alloue dans la heap et non dans la stack (utiliser des pointeurs...)
        std::vector<chrono::ChVector<>> vertices;
        vertices.reserve(nb_vertices);  // preallocation for vertices array

        double xi = xmin;
        double yi = ymin;

        // Creating vertices list on the grid
        for (uint iy =0; iy < nv_y; iy++) {
            for (uint ix = 0; ix < nv_x; ix++) {

                chrono::ChVector<> vertex;
                vertex = chrono::ChVector<>(xi, yi, m_mean_height);
                vertices.push_back(vertex);

                xi += dx;
            }
            yi += dy;
            xi = xmin;
        }

        // Adding faces to the mesh based on vertices generated above
        uint nxm1 = nv_x-1;
        uint nym1 = nv_y-1;
        uint nb_faces = 2* nxm1 * nym1;

        uint ip0, ip1, ip2, ip3, idiv;
        for (uint iface = 0; iface < nb_faces; iface++){  // FIXME: semble qu'on ait un stack overflow si la grille est trop importante
            idiv = (int)iface / nxm1;

            ip0 = iface + idiv;
            ip1 = ip0 + 1;
            ip3 = ip0 + nv_x;
            ip2 = ip3 + 1;

            // Registering faces by alternating between left and right splitting of quadrangles
            if (iface % 2 == 0){
                mesh.addTriangle(vertices[ip0],
                                 vertices[ip1],
                                 vertices[ip2]
                                );
                mesh.addTriangle(vertices[ip0],
                                 vertices[ip2],
                                 vertices[ip3]
                                );

            } else {
                mesh.addTriangle(vertices[ip0],
                                 vertices[ip1],
                                 vertices[ip3]
                                );
                mesh.addTriangle(vertices[ip1],
                                 vertices[ip2],
                                 vertices[ip3]
                                );
            }
        }
    }

    void FrFreeSurface::Initialize(double lmin, double lmax, double dl){
        FrFreeSurface::Initialize(lmin, lmax, dl, lmin, lmax, dl);
    }


}  // end namespace frydom