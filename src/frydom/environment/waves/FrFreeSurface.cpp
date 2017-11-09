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

#include <thread>

#include <chrono/assets/ChTriangleMeshShape.h>
#include <chrono/assets/ChColorAsset.h>
#include "chrono/assets/ChTexture.h"


#include "FrFreeSurface.h"
#include "frydom/core/FrOffshoreSystem.h"


namespace frydom {

    FrFreeSurface::FrFreeSurface() {
        // Creating a waveField and a tidal model
        m_waveField = std::make_shared<FrNullWaveField>();
        m_tidal = std::make_unique<FrTidal>();

        // Creating the free_surface's body
        m_Body = std::make_shared<chrono::ChBody>();
        m_Body->SetIdentifier(-1);
        m_Body->SetName("FreeSurface");
        m_Body->SetPos(chrono::ChVector<>(0, 0, 0));
        m_Body->SetBodyFixed(true);  // Important, however we could add a ChFunction-like to emulate tidal height

        m_Body->SetCollide(false);  // set to false !!!

        // Providing color
        std::shared_ptr<chrono::ChColorAsset> color;
        color = std::make_shared<chrono::ChColorAsset>();
        color->SetColor(chrono::ChColor(0, 41, 58, 0));  // TODO: permettre de changer la couleur
//        color->SetColor(chrono::ChColor(51, 153, 255, 0));  // TODO: permettre de changer la couleur
        m_Body->AddAsset(color);

    }

    void FrFreeSurface::Initialize(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

        // Building the grid
        auto mesh = build_mesh_grid(xmin, xmax, dx, ymin, ymax, dy);

        m_meshAsset = std::make_shared<chrono::ChTriangleMeshShape>();
        m_meshAsset->SetMesh(mesh);
        m_meshAsset->SetName("FreeSurface");
//            mesh_shape->SetFading(0.9);  // Ne fonctionne pas avec Irrlicht...
        m_Body->AddAsset(m_meshAsset);

    }

    void FrFreeSurface::Initialize(double lmin, double lmax, double dl){

        FrFreeSurface::Initialize(lmin, lmax, dl, lmin, lmax, dl);
    }

    FrTriangleMeshConnected FrFreeSurface::build_mesh_grid(double xmin, double xmax, double dx,
                                                           double ymin, double ymax, double dy) {

        FrTriangleMeshConnected mesh;

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

    void FrFreeSurface::UpdateGrid() {

        auto& mesh = m_meshAsset->GetMesh();
        auto nbNodes = mesh.m_vertices.size();

        // Tentative de calcul d'une balance de charge sur des threads

        // Load balancing on a vector over all available threads
        auto nbThread = std::thread::hardware_concurrency();

        auto meanSize = nbNodes / nbThread;
        auto remainder = nbNodes - nbThread * meanSize;


        std::vector<std::pair<unsigned int, unsigned int>> ranges;
        ranges.reserve(nbThread);
        std::pair<unsigned int, unsigned int> range;
        unsigned int lastIndex = 0;
        for (unsigned int iThread=0; iThread<nbThread; ++iThread) {
            range.first = lastIndex;
            range.second = (unsigned int)(lastIndex + meanSize);

            if (iThread < remainder) {
                range.second += 1;
            }

            lastIndex = range.second;
            ranges.push_back(range);
        }

        std::vector<std::thread> threads;
        threads.reserve(nbThread);
        for (unsigned int iThread=0; iThread<nbThread; ++iThread) {
            std::thread thread(&FrFreeSurface::UpdateGridRange, this, ranges[iThread]);
            threads.push_back(thread);
        }

//        for (auto thread : threads) {
//            thread.join();
//        }


        // Fin tentative


//        std::pair<unsigned int, unsigned int> totalRange(0, nbNodes);
//        UpdateGridRange(totalRange);



    }

    void FrFreeSurface::UpdateGridRange(std::pair<unsigned int, unsigned int> range) {

        auto& mesh = this->m_meshAsset->GetMesh();
        auto nbNodes = mesh.m_vertices.size();

        // getting the tidal wave height
        double tidalHeight = this->m_tidal->GetWaterHeight();


        FrLinearWaveProbe* waveProbe;
        chrono::ChVector<double>* vertex;
        for (unsigned int inode=0; inode<nbNodes; ++inode) {
            waveProbe = this->m_waveProbeGrid[inode].get();
            mesh.m_vertices[inode].z() = tidalHeight + waveProbe->GetElevation(m_time);
        }

        // TODO: garder pour faire la demo du speedup...
//        for (auto& vertex: mesh.m_vertices) {
//            vertex.z() = m_waveField->GetElevation(vertex.x(), vertex.y());  // TODO: ne pas utiliser GetElevation mais des waveProbe
//        }
    }


    void FrFreeSurface::UpdateAssetON() {  // TODO: faire la creation des waveProbe ailleurs car ca oblige a definir dire UpdateAssetON apres la creation de waveField...
        m_updateAsset = true;

        // Creating the array of wave probes
        auto nbVertices = m_meshAsset->GetMesh().getCoordsVertices().size();
        m_waveProbeGrid.reserve(nbVertices);

        auto waveField = std::static_pointer_cast<FrLinearWaveField>(m_waveField);

        for (auto& vertex : m_meshAsset->GetMesh().getCoordsVertices()) {
            auto waveProbe = waveField->NewWaveProbe(vertex.x(), vertex.y());
            waveProbe->Initialize();
            m_waveProbeGrid.push_back(waveProbe);
        }
    }



}  // end namespace frydom