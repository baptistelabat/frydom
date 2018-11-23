//
// Created by frongere on 10/10/17.
//

#include <thread>
#include "chrono/assets/ChTriangleMeshShape.h"
#include <chrono/assets/ChColorAsset.h>
#include "FrSeabed.h"
#include <frydom/core/FrOffshoreSystem.h>
#include "frydom/environment/FrEnvironment.h"
#include "frydom/core/FrBody.h"


namespace frydom {


    void FrSeabed::SetGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

        m_xmin = xmin;
        m_xmax = xmax;
        m_dx = dx;
        m_ymin = ymin;
        m_ymax = ymax;
        m_dy = dy;

        m_gridType = CARTESIAN;
    }

    void FrSeabed::SetGrid(double lmin, double lmax, double dl){
        FrSeabed::SetGrid(lmin, lmax, dl, lmin, lmax, dl);
    }

    void FrSeabed::SetGrid(double xc0,
                                double yc0,
                                double diameter,
                                int nbR,
                                int nbTheta) {

        m_xc0 = xc0;
        m_yc0 = yc0;
        m_diameter = diameter;
        m_nbR = nbR;
        m_nbTheta = nbTheta;

        m_gridType = POLAR;

    }

    void FrSeabed::Initialize() {

        // Building the asset
        FrTriangleMeshConnected mesh;
        switch (m_gridType) {
            case CARTESIAN:
                mesh = BuildRectangularMeshGrid(m_xmin, m_xmax, m_dx, m_ymin, m_ymax, m_dy);
                break;
            case POLAR:
                mesh = BuildPolarMeshGrid(m_xc0, m_yc0, m_diameter, m_nbR, m_nbTheta);
                break;
        }

        m_meshAsset = std::make_shared<chrono::ChTriangleMeshShape>();
        m_meshAsset->SetMesh(mesh);
        m_meshAsset->SetName("Seabed");
//            mesh_shape->SetFading(0.9);  // Ne fonctionne pas avec Irrlicht...
        GetBody()->AddAsset(m_meshAsset);

        // Providing color
        std::shared_ptr<chrono::ChColorAsset> color;
        color = std::make_shared<chrono::ChColorAsset>();
        //color->SetColor(chrono::ChColor(49, 33, 6, 1));  // TODO: permettre de changer la couleur
        color->SetColor(chrono::ChColor(255-95, 255-66, 255-16, 1));  // TODO: permettre de changer la couleur
        //color->SetColor(chrono::ChColor(51, 153, 255, 0.));  // TODO: permettre de changer la couleur
        GetBody()->AddAsset(color);

    }


    FrTriangleMeshConnected FrSeabed::BuildRectangularMeshGrid(double xmin, double xmax, double dx,
                                                                    double ymin, double ymax, double dy) {

        FrTriangleMeshConnected mesh;

        int nvx(int((xmax - xmin) / dx) + 1);
        int nvy(int((ymax - ymin) / dy) + 1);

        // Building the vertices list
        std::vector<chrono::ChVector<double>> vertices;
        double xi = xmin, yi = ymin;

        for (int iy = 0; iy < nvy; iy++) {
            for (int ix = 0; ix < nvx; ix++) {
                chrono::ChVector<double> vertex(xi, yi, m_depth);
                vertices.push_back(vertex);
                xi += dx;
            }
            yi += dy;
            xi = xmin;
        }
        // Adding the vertices list to the mesh
        mesh.addVertex(vertices);

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
        mesh.addTriangle(triangles);

        // TODO: initialiser les normales et autres champs de ChTriangleMeshConnected
        return mesh;
    }

    FrTriangleMeshConnected FrSeabed::BuildPolarMeshGrid(double xc0, double yc0,
                                                              double diameter,
                                                              unsigned int nbR, unsigned int nbTheta) {

        FrTriangleMeshConnected mesh;

        auto angles = linspace(0., MU_2PI, nbTheta);

        std::vector<chrono::ChVector<double>> vertices;
        vertices.reserve((nbR-1) * (nbTheta-1) + 1);

        double radius = diameter * 0.5;
        auto distances = linspace<double>(0, radius, nbR);

        for (const auto& distance : distances) {
            vertices.emplace_back(chrono::ChVector<double>(xc0 + distance, yc0, m_depth));
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

        mesh.addVertex(vertices);
        mesh.addTriangle(faces);

        return mesh;
    }

    FrSeabed::FrSeabed() {

    }

    std::shared_ptr<FrBody> FrSeabed::GetBody() {return m_environment->GetSystem()->GetWorldBody();}

    void FrSeabed::SetEnvironment(FrEnvironment *environment) {
        m_environment = environment;
    }






    /// REFACTORING ----------->>>>>>>>



    FrSeabed_::FrSeabed_(FrOcean_ *ocean) :m_ocean(ocean){}


    FrOcean_ *FrSeabed_::GetOcean() const {return m_ocean;}






    FrTriangleMeshConnected
    FrSeabed_::BuildRectangularMeshGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {
        return FrTriangleMeshConnected();
    }

    FrTriangleMeshConnected
    FrSeabed_::BuildPolarMeshGrid(double xc0, double yc0, double diameter, unsigned int nbR, unsigned int nbTheta) {
        return FrTriangleMeshConnected();
    }


    void FrSeabed_::SetDepth(double depth) {m_depth = depth;}

    double FrSeabed_::GetDepth() {return m_depth;}  // TODO : prevoir le fond a bathymetrie variable... (GetDepth(x, y))

    void FrSeabed_::UpdateAsset(bool update) {

    }

    void FrSeabed_::SetGridType(FrSeabed_::GRID_TYPE gridType) {
        m_gridType = gridType;
    }

    void FrSeabed_::SetGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

    }

    void FrSeabed_::SetGrid(double lmin, double lmax, double dl) {

    }

    void FrSeabed_::SetGrid(double xc0, double yc0, double diameter, int nbR, int nbTheta) {

    }

    void FrSeabed_::Update(double time) {}

    void FrSeabed_::Initialize() {

    }

    void FrSeabed_::StepFinalize() {}

}  // end namespace frydom