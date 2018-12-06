//
// Created by Lucas Letournel on 05/12/18.
//

#include "FrFreeSurfaceGridAsset.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom{



    void FrFreeSurfaceGridAsset::SetGridType(GRID_TYPE gridType) {
        m_gridType = gridType;
    }

    void FrFreeSurfaceGridAsset::SetGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

        m_xmin = xmin;
        m_xmax = xmax;
        m_dx = dx;
        m_ymin = ymin;
        m_ymax = ymax;
        m_dy = dy;

        m_gridType = CARTESIAN;
    }

    void FrFreeSurfaceGridAsset::SetGrid(double lmin, double lmax, double dl){
        SetGrid(lmin, lmax, dl, lmin, lmax, dl);
    }

    void FrFreeSurfaceGridAsset::SetGrid(double xc0, double yc0, double diameter, int nbR, int nbTheta) {

        m_xc0 = xc0;
        m_yc0 = yc0;
        m_diameter = diameter;
        m_nbR = nbR;
        m_nbTheta = nbTheta;

        m_gridType = POLAR;

    }

    void FrFreeSurfaceGridAsset::UpdateAssetON() { m_updateAsset = true; }

    void FrFreeSurfaceGridAsset::UpdateAssetOFF() { m_updateAsset = false; }

    std::shared_ptr<FrTriangleMeshConnected>
    FrFreeSurfaceGridAsset::BuildRectangularMeshGrid(double xmin, double xmax, double dx,
                                             double ymin, double ymax, double dy) {

        auto mesh = std::make_shared<FrTriangleMeshConnected>();

        int nvx(int((xmax - xmin) / dx) + 1);
        int nvy(int((ymax - ymin) / dy) + 1);

        // Building the vertices list
        std::vector<chrono::ChVector<double>> vertices;
        double xi = xmin, yi = ymin;

        for (int iy = 0; iy < nvy; iy++) {
            for (int ix = 0; ix < nvx; ix++) {
                chrono::ChVector<double> vertex(xi, yi, 0.);
                vertices.push_back(vertex);
                xi += dx;
            }
            yi += dy;
            xi = xmin;
        }
        // Adding the vertices list to the mesh
        mesh->addVertex(vertices);

        // Building faces of the cartesian grid
        std::vector<chrono::ChVector<int>> triangles;
        for (int iy = 0; iy < nvy - 1; iy++) {
            bool reverse(false);
            for (int ix = 0; ix < nvx - 1; ix++) {
                int i0(iy * nvx + ix);
                int i1(i0 + 1);
                int i2(i1 + nvx);
                int i3(i2 - 1);

                chrono::ChVector<int> triangle_1, triangle_2;

                if (reverse) {
                    triangle_1 = chrono::ChVector<int>(i0, i1, i2);
                    triangle_2 = chrono::ChVector<int>(i0, i2, i3);
                    reverse = false;
                }
                else {
                    triangle_1 = chrono::ChVector<int>(i0, i1, i3);
                    triangle_2 = chrono::ChVector<int>(i1, i2, i3);
                    reverse = true;
                }

                triangles.push_back(triangle_1);
                triangles.push_back(triangle_2);
            }
        }
        // Adding the triangle list to the mesh
        mesh->addTriangle(triangles);

        // TODO: initialiser les normales et autres champs de ChTriangleMeshConnected
        return mesh;
    }

    std::shared_ptr<FrTriangleMeshConnected>
    FrFreeSurfaceGridAsset::BuildPolarMeshGrid(double xc0, double yc0,
                                       double diameter,
                                       unsigned int nbR, unsigned int nbTheta) {

        auto mesh = std::make_shared<FrTriangleMeshConnected>();

        auto angles = linspace(0., MU_2PI, nbTheta);

        std::vector<chrono::ChVector<double>> vertices;
        vertices.reserve((nbR-1) * (nbTheta-1) + 1);

        double radius = diameter * 0.5;
        auto distances = linspace<double>(0, radius, nbR);

        for (const auto& distance : distances) {
            vertices.emplace_back(chrono::ChVector<double>(xc0 + distance, yc0, 0.));
        }

        std::vector<chrono::ChVector<int>> faces;  // TODO: reserver l'espace

        int i0, i1, i2, i3;
        double angle, distance;
        for (unsigned int iangle=1; iangle<angles.size(); ++iangle) {
            angle = angles[iangle];

            // Adding new vertices
            for (unsigned int idist=1; idist<nbR; ++idist) {
                distance = distances[idist];
                vertices.emplace_back(
                        chrono::ChVector<double>(
                                xc0 + distance * cos(angle),
                                yc0 + distance * sin(angle),
                                0.
                        )
                );
            }

            // Building center triangle
            i0 = 0;
            i1 = (iangle-1) * (nbR-1) + 1;
            i2 = iangle * (nbR-1) + 1;
            faces.emplace_back(chrono::ChVector<int>(i0, i1, i2));

            // Building next triangles
            for (unsigned int idist=2; idist<nbR; ++idist) {

                i0 = iangle * (nbR-1) + idist -1;
                i1 = (iangle-1) * (nbR-1) + idist -1;
                i2 = (iangle-1) * (nbR-1) + idist;
                i3 = iangle * (nbR-1) + idist;

                faces.emplace_back(chrono::ChVector<int>(i0, i1, i3));
                faces.emplace_back(chrono::ChVector<int>(i3, i1, i2));

            }
        }

        mesh->addVertex(vertices);
        mesh->addTriangle(faces);

        return mesh;
    }

    void FrFreeSurfaceGridAsset::Initialize() {
        // Building the asset
        std::shared_ptr<FrTriangleMeshConnected> mesh;
        switch (m_gridType) {
            case CARTESIAN:
                mesh = BuildRectangularMeshGrid(m_xmin, m_xmax, m_dx, m_ymin, m_ymax, m_dy);
                break;
            case POLAR:
                mesh = BuildPolarMeshGrid(m_xc0, m_yc0, m_diameter, m_nbR, m_nbTheta);
                break;
            case NONE:
                break;
        }
        m_meshAsset = std::make_shared<chrono::ChTriangleMeshShape>();
        m_meshAsset->SetMesh(*mesh);
        m_chronoPhysicsItem->AddAsset(m_meshAsset);

        SetColor(DodgerBlue);
    }

    void FrFreeSurfaceGridAsset::Update(double time) {

        if (m_updateAsset) {
            // getting the tidal wave height
            double tidalHeight = m_freeSurface->GetTidal()->GetWaterHeight();

            auto& mesh = m_meshAsset->GetMesh();
            auto nbNodes = mesh.m_vertices.size();
            for (unsigned int inode = 0; inode < nbNodes; ++inode) {
                mesh.m_vertices[inode].z() = tidalHeight +
                                                     m_freeSurface->GetElevation(mesh.m_vertices[inode].x(),
                                                                                 mesh.m_vertices[inode].y());
            }
        }

    }

}