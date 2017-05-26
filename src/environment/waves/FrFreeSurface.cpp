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
#include <chrono/assets/ChTriangleMeshShape.h>
#include "FrFreeSurface.h"

namespace frydom {
namespace environment{

    FrFreeSurface::FrFreeSurface(double p_mean_height) : m_mean_height(p_mean_height){
        plane.pos[1] = p_mean_height;  // The free surface plane reference has the altitude the mean FS height
    }

    double FrFreeSurface::getMeanHeight() const {
       return m_mean_height;
    }

    void FrFreeSurface::Initialize(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

        // Making sure the mesh is clean
        m_mesh.Clear();

        const std::size_t nv_x = std::size_t((xmax - xmin) / dx) + 1;
        const std::size_t nv_y = std::size_t((ymax - ymin) / dy) + 1;

        uint nb_vertices = uint(nv_x * nv_y);

        std::cout << "Nb vertices: " << nb_vertices << std::endl;

        std::vector<std::vector<chrono::ChVector<>>>
                grid(nv_y, std::vector<chrono::ChVector<>>(nv_x, chrono::ChVector<>()));

        double xi = xmin, yj = ymin;
        for (auto& row: grid){
            for (auto& vertex: row){
                vertex.Set(xi, yj, m_mean_height);
                xi += dx;
            }
            yj += dy;
            xi = xmin;
        }


        // Adding faces to the mesh based on vertices generated above
        std::size_t nb_faces = 2* (nv_x-1) * (nv_y-1);

        std::cout << "Nb faces: " << nb_faces << std::endl;

        for (int iy = 0; iy < nv_y-1; iy++){
            for (int ix = 0; ix < nv_x-1; ix++){
                // TODO: voir si on ne peut pas creer des pointeurs et pas reinstancier les vertex a chaque fois
                auto v0 = grid[iy][ix];
                auto v1 = grid[iy][ix+1];
                auto v2 = grid[iy+1][ix+1];
                auto v3 = grid[iy+1][ix];

                m_mesh.addTriangle(chrono::geometry::ChTriangle(v0, v1, v2));
                m_mesh.addTriangle(chrono::geometry::ChTriangle(v0, v2, v3));

                // TODO: voir si on calcule les normales
            }
        }

        if (m_vis_enabled) {
            auto mesh_shape = std::make_shared<chrono::ChTriangleMeshShape>();
            mesh_shape->SetMesh(m_mesh);
            mesh_shape->SetName("Free surface");
        }

    }

    void FrFreeSurface::Initialize(double lmin, double lmax, double dl){
        FrFreeSurface::Initialize(lmin, lmax, dl, lmin, lmax, dl);
    }

    chrono::geometry::ChTriangleMeshConnected FrFreeSurface::getMesh(void) const {
        return m_mesh;
    }


}  // end namespace environment
}  // end namespace frydom