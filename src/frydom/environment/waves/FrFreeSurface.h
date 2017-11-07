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

//#include "chrono/physics/ChBody.h"

#include <frydom/environment/tidal/FrTidalModel.h>
#include "frydom/misc/FrTriangleMeshConnected.h"
#include "FrWaveField.h"

// Forward declarations
namespace chrono {
//    template <class Real>
//    class ChCoordsys;

    class ChBody;

//    class ChColorAsset;
}


namespace frydom{
    // Forward declaration
    class FrOffshoreSystem;
//    class FrTriangleMeshConnected;


    /// Pure Virtual Base class for a free surface system.
    class FrFreeSurface {

        // FIXME: Il faut que ce soit cette classe qui comprenne un modele de maree !!

    protected:;  // Disallow the default constructor to be used as a public method

        double m_time = 0.;
//        bool m_vis_enabled;
        bool m_updateAsset = false;

        std::shared_ptr<chrono::ChBody> m_Body;
        FrTriangleMeshConnected m_mesh;

        std::unique_ptr<FrTidal> m_tidal;

        WAVE_MODEL m_waveModel = NO_WAVES;
        std::shared_ptr<FrWaveField> m_waveField;


        /// Private method in charge of the building of the free surface mesh.
        void build_mesh_grid(double xmin, double xmax, double dx,
                             double ymin, double ymax, double dy);

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
            // TODO
            return 0.;
        }

        double GetHeight(double x, double y) {
            // TODO
            // Maree + elevation
            return 0.;
        }

//        std::shared_ptr<FrLinearWaveField> GetLinearWaveField() const {
//            if (m_waveModel != LINEAR_WAVES) {
//                throw "Cannot get the linear wave field as it is not set to a linear model";
//            }
//            return dynamic_cast<std::shared_ptr<FrLinearWaveField>>(m_waveField.get());
//        }


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

        /// Get the free surface's mesh
        FrTriangleMeshConnected getMesh(void) const;

        /// Update the state of the free surface
        virtual void Update(double time) {
            m_time = time;
            m_tidal->Update(time);
            m_waveField->Update(time);

            if (m_updateAsset) {
                // Updating the free surface grid for visualization
            }

        }

        FrTidal* GetTidal() const {
            return m_tidal.get();
        }

        /// get the body that represents the free surface
        std::shared_ptr<chrono::ChBody> GetBody() {return m_Body;}

//        void SetWaveField(std::shared_ptr<FrWaveField> waveField) { m_waveField = waveField; }

        std::shared_ptr<FrWaveField> GetWaveField() const { return m_waveField; }


    };

}  // end namespace frydom

#endif // FR_FREE_SURFACE_H
