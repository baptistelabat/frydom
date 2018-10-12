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

//#include <thread>

#include "MathUtils/MathUtils.h"

#include "FrFreeSurface.h"
#include "frydom/core/FrOffshoreSystem.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "frydom/mesh/FrTriangleMeshConnected.h"
#include "frydom/environment/tidal/FrTidalModel.h"

#include "FrWaveField.h"
#include "FrWaveProbe.h"


#include "frydom/environment/FrEnvironment.h"
#include "frydom/core/FrBody.h"


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
//            vertex.z() = tidalHeight + m_waveField->GetElevation(vertex.x(), vertex.y());  // TODO: ne pas utiliser GetElevation mais des waveProbe
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




























    ////// REFACTORING ---------->>>>>>>>>>



    FrFreeSurface_::FrFreeSurface_(FrEnvironment_* environment) : m_environment(environment) {

        // Creating a waveField and a tidal model
        m_waveField = std::make_shared<FrNullWaveField_>(this);
        m_tidal     = std::make_unique<FrTidal_>(this);

        CreateFreeSurfaceBody();
    }

    FrFreeSurface_::~FrFreeSurface_() = default;

    FrOffshoreSystem_* FrFreeSurface_::GetSystem() {
        return m_environment->GetSystem();
    }

    void FrFreeSurface_::SetGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy) {

        m_xmin = xmin;
        m_xmax = xmax;
        m_dx = dx;
        m_ymin = ymin;
        m_ymax = ymax;
        m_dy = dy;

        m_gridType = CARTESIAN;
    }

    void FrFreeSurface_::SetGrid(double lmin, double lmax, double dl){
        FrFreeSurface_::SetGrid(lmin, lmax, dl, lmin, lmax, dl);
    }

    void FrFreeSurface_::SetGrid(double xc0,
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

    void FrFreeSurface_::CreateFreeSurfaceBody() {
        m_body = std::make_shared<FrBody_>();
        m_body->SetName("FreeSurface");
        m_body->SetAbsPosition(Position(0., 0., 0.), NWU);
        m_body->SetBodyFixed(true);
        m_body->SetCollide(false);

//        m_body->SetColor(FrColor(255, 145, 94));
        m_body->SetColor(DodgerBlue);

    }

    void FrFreeSurface_::Initialize() {

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

//        CreateFreeSurfaceBody();
        m_body->AddMeshAsset(mesh);
        GetSystem()->AddBody(m_body);


        // If the mesh is being to be animated
        if (m_updateAsset) {
            // Creating the array of wave probes
            auto nbVertices = m_meshAsset->getCoordsVertices().size();
            m_waveProbeGrid.reserve(nbVertices);

            // FIXME: le fait que le wavefield soit requis pour initialiser le maillage de surface libre fait qu'on est
            // oblige de definir le wavefield avant d'activer l'asset --> pas flex du tout. Utiliser plutot un flag pour
            // etablir que l'asset est initialise ou pas et initialiser la gille  lors de l'appel a UpdateGrid si le flag est false
            auto waveField = std::static_pointer_cast<FrLinearWaveField_>(m_waveField);

            for (auto& vertex : m_meshAsset->getCoordsVertices()) {
                auto waveProbe = waveField->NewWaveProbe(vertex.x(), vertex.y());
                waveProbe->Initialize();
                m_waveProbeGrid.push_back(waveProbe);
            }
        }

    }

    std::shared_ptr<FrTriangleMeshConnected>
    FrFreeSurface_::BuildRectangularMeshGrid(double xmin, double xmax, double dx,
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
    FrFreeSurface_::BuildPolarMeshGrid(double xc0, double yc0,
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


    void FrFreeSurface_::UpdateGrid() {

        auto nbNodes = m_meshAsset->GetNbVertices();

        // getting the tidal wave height
        double tidalHeight = m_tidal->GetWaterHeight();

        FrLinearWaveProbe_* waveProbe;
        chrono::ChVector<double>* vertex;
        for (unsigned int inode=0; inode<nbNodes; ++inode) {
            waveProbe = this->m_waveProbeGrid[inode].get();
            m_meshAsset->m_vertices[inode].z() = tidalHeight + waveProbe->GetElevation(m_time);
        }

    }

    void FrFreeSurface_::NoWaves() {
        // TODO
//        m_waveModel = NO_WAVES;
////            m_waveField.reset(nullptr);
    }

    void FrFreeSurface_::SetLinearWaveField(LINEAR_WAVE_TYPE waveType) {
        m_waveModel = LINEAR_WAVES;
        m_waveField = std::make_shared<FrLinearWaveField_>(this, waveType);
    }

    FrLinearWaveField *FrFreeSurface_::GetLinearWaveField() const {
        return dynamic_cast<FrLinearWaveField*>(m_waveField.get());
    }

    double FrFreeSurface_::GetMeanHeight(double x, double y) {
        return m_tidal->GetWaterHeight();
    }

    double FrFreeSurface_::GetHeight(double x, double y) {
        return m_tidal->GetWaterHeight() + m_waveField->GetElevation(x, y);
    }

    void FrFreeSurface_::SetGridType(FrFreeSurface_::GRID_TYPE gridType) {
        m_gridType = gridType;
    }

    void FrFreeSurface_::UpdateAssetON() { m_updateAsset = true; }

    void FrFreeSurface_::UpdateAssetOFF() { m_updateAsset = false; }

    void FrFreeSurface_::Update(double time) {
        m_time = time;
        m_tidal->Update(time);
        m_waveField->Update(time);

        if (m_updateAsset) {
            // Updating the free surface grid for visualization
            UpdateGrid();
        }

    }

    FrTidal_ *FrFreeSurface_::GetTidal() const {
        return m_tidal.get();
    }

    FrWaveField_ * FrFreeSurface_::GetWaveField() const { return m_waveField.get(); }


}  // end namespace frydom
