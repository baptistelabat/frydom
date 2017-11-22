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

#ifndef FR_FREE_SURFACE_H
#define FR_FREE_SURFACE_H

//#include "chrono/core/ChFrame.h"

#include "frydom/environment/tidal/FrTidalModel.h"
#include "frydom/misc/FrTriangleMeshConnected.h"
#include "FrWaveField.h"
#include "FrWaveProbe.h"

// Forward declarations
namespace chrono {

    class ChBody;

    class ChTriangleMeshShape;
}


namespace frydom{
    // Forward declaration
    class FrOffshoreSystem;
//    class FrTriangleMeshConnected;

    /// Pure Virtual Base class for a free surface system.
    class FrFreeSurface {

        // FIXME: Il faut que ce soit cette classe qui comprenne un modele de maree !!

    protected:;  // Disallow the default constructor to be used as a public method // TODO: mettre private???

        double m_time = 0.;
        bool m_updateAsset = false;

        std::shared_ptr<chrono::ChBody> m_Body;
        std::unique_ptr<FrTidal> m_tidal;

        WAVE_MODEL m_waveModel = NO_WAVES;
        std::shared_ptr<FrWaveField> m_waveField;

        std::shared_ptr<chrono::ChTriangleMeshShape> m_meshAsset;

        std::vector<std::shared_ptr<FrLinearWaveProbe>> m_waveProbeGrid; // TODO: passer a la classe de base...

        std::vector<double> m_gridHeights; // TODO: preallouer a l'initialisation


    protected:

        /// Private method in charge of the building of the free surface mesh as a rectangular grid.
        FrTriangleMeshConnected BuildRectangularMeshGrid(double xmin, double xmax, double dx,
                                                         double ymin, double ymax, double dy);

        /// Private method in charge of the building of the free surface mesh as a polar grid.
        FrTriangleMeshConnected BuildPolarMeshGrid(double xc0, double yc0, // center
                                                   double diameter,
                                                   unsigned int nbR, unsigned int nbTheta);
    public:

        FrFreeSurface();

        void NoWaves() {
            m_waveModel = NO_WAVES;
//            m_waveField.reset(nullptr);
        }

        void SetLinearWaveField(LINEAR_WAVE_TYPE waveType) {
            m_waveModel = LINEAR_WAVES;
            m_waveField = std::make_shared<FrLinearWaveField>(waveType);
        }

        FrLinearWaveField* GetLinearWaveField() const {
            return dynamic_cast<FrLinearWaveField*>(m_waveField.get());
        }

        double GetMeanHeight(double x, double y) {
            return m_tidal->GetWaterHeight();
        }

        double GetHeight(double x, double y) {
            return m_tidal->GetWaterHeight() + m_waveField->GetElevation(x, y);
        }

        /// Initializes the free surface system
        /// In any case, a mesh grid is used.
        /// this version concerns a rectangular grid
        void Initialize(double xmin,
                        double xmax,
                        double dx,
                        double ymin,
                        double ymax,
                        double dy
                        );

        /// Initializes the free surface system
        /// In any case, a mesh grid is used.
        /// this version concerns a square grid
        void Initialize(double lmin,
                        double lmax,
                        double dl
                        );

        void Initialize(double xc0,
                        double yc0,
                        double diameter,
                        int nbR,
                        int nbTheta
                        );


        /// Get the free surface's mesh
//        FrTriangleMeshConnected getMesh(void) const;

        void UpdateAssetON();

        void UpdateAssetOFF() { m_updateAsset = false; }

        /// Update the state of the free surface
        virtual void Update(double time) {
            m_time = time;
            m_tidal->Update(time);
            m_waveField->Update(time);

            if (m_updateAsset) {
                // Updating the free surface grid for visualization
                UpdateGrid();
            }

        }

        void UpdateGrid();

        void UpdateGridRange(std::pair<unsigned int, unsigned int> range);


        FrTidal* GetTidal() const {
            return m_tidal.get();
        }

        /// get the body that represents the free surface
        std::shared_ptr<chrono::ChBody> GetBody() {return m_Body;}

//        void SetWaveField(std::shared_ptr<FrWaveField> waveField) { m_waveField = waveField; }

        std::shared_ptr<FrWaveField> GetWaveField() const { return m_waveField; }

        const chrono::ChFrame<double>* GetFrame() const;

    };

}  // end namespace frydom

#endif // FR_FREE_SURFACE_H
