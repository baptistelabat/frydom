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


//#include <thread>

#include "MathUtils/MathUtils.h"

#include "FrFreeSurface.h"
#include "frydom/core/FrOffshoreSystem.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"
#include "frydom/asset/FrFreeSurfaceGridAsset.h"

#include "frydom/mesh/FrTriangleMeshConnected.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean_.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveProbe.h"
#include "frydom/environment/ocean/freeSurface/waves/airy/FrAiryRegularWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/airy/FrAiryRegularOptimWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/airy/FrAiryIrregularWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/airy/FrAiryIrregularOptimWaveField.h"


#include "frydom/core/body/FrBody.h"


namespace frydom {


    FrFreeSurface::FrFreeSurface() {

        // Creating a waveField and a tidal model
        m_waveField = std::make_unique<FrNullWaveField>(this);
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
        color->SetColor(chrono::ChColor(255, 145, 94, 0));  // TODO: permettre de changer la couleur
        //color->SetColor(chrono::ChColor(0, 41, 58, 0));  // TODO: permettre de changer la couleur
        //color->SetColor(chrono::ChColor(51, 153, 255, 0));  // TODO: permettre de changer la couleur
        m_Body->AddAsset(color);

    }

    FrFreeSurface::~FrFreeSurface() = default;

    void FrFreeSurface::SetGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

        m_xmin = xmin;
        m_xmax = xmax;
        m_dx = dx;
        m_ymin = ymin;
        m_ymax = ymax;
        m_dy = dy;

        m_gridType = CARTESIAN;
    }

    void FrFreeSurface::SetGrid(double lmin, double lmax, double dl){
        FrFreeSurface::SetGrid(lmin, lmax, dl, lmin, lmax, dl);
    }

    void FrFreeSurface::SetGrid(double xc0,
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

    void FrFreeSurface::Initialize() {

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
        m_meshAsset->SetName("FreeSurface");
//            mesh_shape->SetFading(0.9);  // Ne fonctionne pas avec Irrlicht...
        m_Body->AddAsset(m_meshAsset);

        // If the mesh is being to be animated
        // TODO : remettre en place l'utilisation de l'asset !!!

//        if (m_updateAsset) {
//            // Creating the array of wave probes
//            auto nbVertices = m_meshAsset->GetMesh().getCoordsVertices().size();
//            m_waveProbeGrid.reserve(nbVertices);
//
//            // FIXME: le fait que le wavefield soit requis pour initialiser le maillage de surface libre fait qu'on est
//            // oblige de definir le wavefield avant d'activer l'asset --> pas flex du tout. Utiliser plutot un flag pour
//            // etablir que l'asset est initialise ou pas et initialiser la gille  lors de l'appel a UpdateGrid si le flag est false
//            auto waveField = std::static_pointer_cast<FrLinearWaveField>(m_waveField);
//
//            for (auto& vertex : m_meshAsset->GetMesh().getCoordsVertices()) {
//                auto waveProbe = waveField->NewWaveProbe(vertex.x(), vertex.y());
//                waveProbe->Initialize();
//                m_waveProbeGrid.push_back(waveProbe);
//            }
//        }

    }

    FrTriangleMeshConnected FrFreeSurface::BuildRectangularMeshGrid(double xmin, double xmax, double dx,
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

    FrTriangleMeshConnected FrFreeSurface::BuildPolarMeshGrid(double xc0, double yc0,
                                                              double diameter,
                                                              unsigned int nbR, unsigned int nbTheta) {

        FrTriangleMeshConnected mesh;

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

        mesh.addVertex(vertices);
        mesh.addTriangle(faces);

        return mesh;
    }


    void FrFreeSurface::UpdateGrid() {

//        auto& mesh = m_meshAsset->GetMesh();
//        auto nbNodes = mesh.m_vertices.size();
//
//        // Tentative de calcul d'une balance de charge sur des threads
//
//        // Load balancing on a vector over all available threads
//        auto nbThread = std::thread::hardware_concurrency();
//
////        nbThread = 4;
//
//        auto meanSize = nbNodes / nbThread;
//        auto remainder = nbNodes - nbThread * meanSize;
//
//
//        std::vector<std::pair<unsigned int, unsigned int>> ranges;
//        ranges.reserve(nbThread);
//        std::pair<unsigned int, unsigned int> range;
//        unsigned int lastIndex = 0;
//        for (unsigned int iThread=0; iThread<nbThread; ++iThread) {
//            range.first = lastIndex;
//            range.second = (unsigned int)(lastIndex + meanSize);
//
//            if (iThread < remainder) {
//                range.second += 1;
//            }
//
//            lastIndex = range.second;
//            ranges.push_back(range);
//        }
//
//        std::vector<std::thread> threads(nbThread);
////        threads.reserve(nbThread);
//        for (unsigned int iThread=0; iThread<nbThread; ++iThread) {
//            threads.at(iThread) = std::thread(&FrFreeSurface::UpdateGridRange, this, ranges[iThread]);
////            std::thread thread(&FrFreeSurface::UpdateGridRange, this, ranges[iThread]);
////            threads.push_back(thread);
//        }
//
//        for (unsigned int iThread=0; iThread<nbThread; ++iThread) {
//            threads.at(iThread).join();
//        }
//
//
//        // Fin tentative
//        m_Body->Update(true); // Necessary to update the asset
//
////        std::pair<unsigned int, unsigned int> totalRange(0, nbNodes);
////        UpdateGridRange(totalRange);


    }

    void FrFreeSurface::UpdateGridRange(std::pair<unsigned int, unsigned int> range) {

        auto& mesh = this->m_meshAsset->GetMesh();
        auto nbNodes = mesh.m_vertices.size();

        // getting the tidal wave height
        double tidalHeight = this->m_tidal->GetWaterHeight();

//        std::cout << std::this_thread::get_id() << "\n";

        FrLinearWaveProbe* waveProbe;
        chrono::ChVector<double>* vertex;
        for (unsigned int inode=0; inode<nbNodes; ++inode) {
            waveProbe = this->m_waveProbeGrid[inode].get();
            mesh.m_vertices[inode].z() = tidalHeight + waveProbe->GetElevation(m_time);
        }

        // TODO: garder pour faire la demo du speedup...
//        for (auto& vertex: mesh.m_vertices) {
//            vertex.z() = tidalHeight + m_waveField->GetHeight(vertex.x(), vertex.y());  // TODO: ne pas utiliser GetHeight mais des waveProbe
//        }
    }

//    const chrono::ChFrame<double>* FrFreeSurface::GetFrame() const {
//        return m_tidal->GetTidalFrame();
//    }

    void FrFreeSurface::NoWaves() {
        m_waveModel = NO_WAVES;
//            m_waveField.reset(nullptr);
    }

    void FrFreeSurface::SetLinearWaveField(LINEAR_WAVE_TYPE waveType) {
        m_waveModel = LINEAR_WAVES;
        m_waveField = std::make_unique<FrLinearWaveField>(this, waveType);
    }

    FrLinearWaveField *FrFreeSurface::GetLinearWaveField() const {
        return dynamic_cast<FrLinearWaveField*>(m_waveField.get());
    }

    double FrFreeSurface::GetMeanHeight(double x, double y) {
        return m_tidal->GetWaterHeight();
    }

    double FrFreeSurface::GetHeight(double x, double y) {
        return m_tidal->GetWaterHeight() + m_waveField->GetElevation(x, y);
    }

    void FrFreeSurface::SetGridType(FrFreeSurface::GRID_TYPE gridType) {
        m_gridType = gridType;
    }

    void FrFreeSurface::Update(double time) {
        m_time = time;
        m_tidal->Update(time);
        m_waveField->Update(time);

        if (m_updateAsset) {
            // Updating the free surface grid for visualization
            UpdateGrid();
        }

    }

    FrTidal *FrFreeSurface::GetTidal() const {
        return m_tidal.get();
    }

    std::shared_ptr<chrono::ChBody> FrFreeSurface::GetBody() {return m_Body;}

    FrWaveField* FrFreeSurface::GetWaveField() const { return m_waveField.get(); }




























    // REFACTORING ---------->>>>>>>>>>


    FrFreeSurface_::~FrFreeSurface_() = default;

    FrFreeSurface_::FrFreeSurface_(FrOcean_* ocean) : m_ocean(ocean) {

        // Creating a waveField and a tidal model
        m_waveField         = std::make_unique<FrNullWaveField_>(this);
        m_tidal             = std::make_unique<FrTidal_>(this);
        m_freeSurfaceGridAsset    = std::make_shared<FrFreeSurfaceGridAsset>(ocean->GetEnvironment()->GetSystem()->GetWorldBody().get(),this);

//        CreateFreeSurfaceBody();
    }

//    void FrFreeSurface_::CreateFreeSurfaceBody() {
//
//        m_body = std::make_shared<FrBody_>();
//        m_body->SetName("FreeSurface");
//        m_body->SetPosition(Position(0., 0., 0.), NWU);
//        m_body->SetFixedInWorld(true);
//        m_body->AllowCollision(false);
//
//        m_ocean->GetEnvironment()->GetSystem()->AddBody(m_body);
//
//    }

    FrAtmosphere_ *FrFreeSurface_::GetAtmosphere() const { return m_ocean->GetEnvironment()->GetAtmosphere(); }

    FrOcean_ *FrFreeSurface_::GetOcean() const { return m_ocean; }

    FrTidal_ *FrFreeSurface_::GetTidal() const { return m_tidal.get(); }

    FrWaveField_ * FrFreeSurface_::GetWaveField() const { return m_waveField.get(); }

    double FrFreeSurface_::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
        m_waveField->GetElevation(x,y, fc);
    }

    FrFreeSurfaceGridAsset *FrFreeSurface_::GetFreeSurfaceGridAsset() const {return m_freeSurfaceGridAsset.get();}

    double FrFreeSurface_::GetPosition(FRAME_CONVENTION fc) const {
        return GetPosition(0.,0.,fc);
    }

    double FrFreeSurface_::GetPosition(double x, double y, FRAME_CONVENTION fc) const {
        return m_tidal->GetHeight(fc) + m_waveField->GetElevation(x, y, fc);
    }

    double FrFreeSurface_::GetPosition(const Position worldPos, FRAME_CONVENTION fc) const {
        return GetPosition(worldPos[0],worldPos[1],fc);
    }

    void FrFreeSurface_::GetPosition(Position& worldPos, FRAME_CONVENTION fc) const {
        worldPos[2] = GetPosition(worldPos[0],worldPos[1],fc);
    }


    void FrFreeSurface_::NoWaves() {
        m_waveField = std::make_unique<FrNullWaveField_>(this);
    }

//    void FrFreeSurface_::SetLinearWaveField(LINEAR_WAVE_TYPE waveType) {
////        m_waveModel = LINEAR_WAVES;
//        m_waveField = std::make_unique<FrLinearWaveField_>(this, waveType);
//    }
//
//    FrLinearWaveField *FrFreeSurface_::GetLinearWaveField() const {
//        return dynamic_cast<FrLinearWaveField*>(m_waveField.get());
//    }


    FrAiryRegularWaveField*
    FrFreeSurface_::SetAiryRegularWaveField() {
        m_waveField = std::make_unique<FrAiryRegularWaveField>(this);
        return dynamic_cast<FrAiryRegularWaveField*>(m_waveField.get());
    }

    FrAiryRegularWaveField*
    FrFreeSurface_::SetAiryRegularWaveField(double waveHeight, double wavePeriod, double waveDirAngle, ANGLE_UNIT unit,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirAngle, unit, fc, dc);
        return dynamic_cast<FrAiryRegularWaveField*>(m_waveField.get());
    }


    FrAiryRegularWaveField*
    FrFreeSurface_::SetAiryRegularWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirection, fc, dc);
        return waveField;
    }


    FrAiryRegularOptimWaveField*
    FrFreeSurface_::SetAiryRegularOptimWaveField() {
        m_waveField = std::make_unique<FrAiryRegularOptimWaveField>(this);
        return dynamic_cast<FrAiryRegularOptimWaveField*>(m_waveField.get());
    }

    FrAiryRegularOptimWaveField*
    FrFreeSurface_::SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, double waveDirAngle, ANGLE_UNIT unit,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularOptimWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirAngle, unit, fc, dc);
        return waveField;
    }

    FrAiryRegularOptimWaveField*
    FrFreeSurface_::SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
                                            FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        auto waveField = SetAiryRegularOptimWaveField();
        waveField->SetWaveHeight(waveHeight);
        waveField->SetWavePeriod(wavePeriod);
        waveField->SetDirection(waveDirection, fc, dc);
        return waveField;
    }


    FrAiryIrregularWaveField*
    FrFreeSurface_::SetAiryIrregularWaveField() {
        m_waveField = std::make_unique<FrAiryIrregularWaveField>(this);
        return dynamic_cast<FrAiryIrregularWaveField*>(m_waveField.get());
    }

    FrAiryIrregularOptimWaveField*
    FrFreeSurface_::SetAiryIrregularOptimWaveField() {
        m_waveField = std::make_unique<FrAiryIrregularOptimWaveField>(this);
        return dynamic_cast<FrAiryIrregularOptimWaveField*>(m_waveField.get());
    }

    void FrFreeSurface_::Initialize() {
        if (m_showFreeSurface) {
            m_tidal->Initialize();
            m_waveField->Initialize();
            m_freeSurfaceGridAsset->Initialize();
            m_ocean->GetEnvironment()->GetSystem()->GetWorldBody()->AddAsset(m_freeSurfaceGridAsset);
        }
    }

    void FrFreeSurface_::Update(double time) {
        if (m_showFreeSurface) {
            m_tidal->Update(time);
            m_waveField->Update(time);
        }
    }

    void FrFreeSurface_::ShowFreeSurface(bool showFreeSurface) {
        if (showFreeSurface && m_showFreeSurface!=showFreeSurface) {
            std::cout<< "Be careful to set new free surface grid, wave field and tidal model"<<std::endl;
        }
        m_showFreeSurface = showFreeSurface;
        if (!showFreeSurface) {
            m_waveField = std::make_unique<FrNullWaveField_>(this);
            m_tidal->SetNoTidal();
            m_freeSurfaceGridAsset->SetNoGrid();
        }
    }

    void FrFreeSurface_::StepFinalize() {
        if (m_showFreeSurface) {
            m_waveField->StepFinalize();
            m_freeSurfaceGridAsset->StepFinalize();
        }
    }


}  // end namespace frydom
