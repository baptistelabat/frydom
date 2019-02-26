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


#ifndef FR_FREE_SURFACE_H
#define FR_FREE_SURFACE_H

#include <memory>

#include "MathUtils/Unit.h"

#include "frydom/core/common/FrObject.h"

#include "frydom/core/common/FrConvention.h"
#include "frydom/core/math/FrVector.h"


//#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"


namespace frydom {


    // Forward declarations
    class FrOffshoreSystem_;
    class FrEnvironment_;
    class FrAtmosphere_;
    class FrOcean_;
    class FrWaveField_;
    class FrTidal_;
    class FrBody_;
    class FrAiryRegularWaveField;
    class FrAiryRegularOptimWaveField;
    class FrAiryIrregularWaveField;
    class FrAiryIrregularOptimWaveField;
    class FrFreeSurfaceGridAsset;


    /// Class for a free surface system.
    /**
     * \class FrFreeSurface_
     * \brief Class for defining the free surface.
     */
    class FrFreeSurface_ : public FrObject {

    protected:;  // Disallow the default constructor to be used as a public method // TODO: mettre private???

        FrOcean_* m_ocean;                            ///< Pointer to the ocean containing this free surface
        bool m_showFreeSurface = true;                ///< Boolean testing if the free surface is to be shown/exist

        // Free surface elements
        std::unique_ptr<FrTidal_> m_tidal;            ///< Tidal model
        std::unique_ptr<FrWaveField_> m_waveField;    ///< Wave field model

        // Visualization asset
        std::shared_ptr<FrFreeSurfaceGridAsset> m_freeSurfaceGridAsset;    ///> free surface grid asset, containing also the visualization asset

    public:

        /// Default constructor
        /// \param ocean ocean containing this free surface
        explicit FrFreeSurface_(FrOcean_* ocean);

        /// Default destructor
        ~FrFreeSurface_();

        //---------------------------- Asset ----------------------------//

        /// Set if the free surface is to be shown/exist
        /// \param showFreeSurface showfreesurface true means the free surface exists
        void ShowFreeSurface(bool showFreeSurface);

        /// Get the free surface grid asset
        /// \return free surface grid asset
        FrFreeSurfaceGridAsset* GetFreeSurfaceGridAsset() const;

        //---------------------------- Free surface elements Getters ----------------------------//

        /// Get the ocean containing this free surface
        /// \return ocean containing this free surface
        FrOcean_* GetOcean() const;

        /// Get the atmosphere above the ocean
        /// \return atmosphere above the ocean
        FrAtmosphere_* GetAtmosphere() const;;

        /// Get the tidal model
        /// \return tidal model
        FrTidal_* GetTidal() const;

        /// Get the wave field model
        /// \return wave field model
        FrWaveField_* GetWaveField() const;

        /// Get the wave elevation at the position (x,y,0), given by the wave field model
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return wave elevation, in meters
        double GetElevation(double x, double y, FRAME_CONVENTION fc) const;

        /// Get the vertical position of the free surface at the position (0,0) (tidal height + elevation)
        /// \param fc frame convention (NED/NWU)
        /// \return vertical position of the free surface at the position (0,0) in frame convention
        double GetPosition(FRAME_CONVENTION fc) const;

        /// Get the vertical position of the free surface at the position (x,y) (tidal height + elevation)
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return vertical position of the free surface at the position (x,y) in frame convention
        double GetPosition(double x, double y, FRAME_CONVENTION fc) const;

        /// Get the vertical position of the free surface at the worldPos position (tidal height + elevation)
        /// \param worldPos position in world reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return vertical position of the free surface at the position (x,y) in frame convention
        double GetPosition(const Position worldPos, FRAME_CONVENTION fc) const;

        /// Get the vertical position of the free surface at the worldPos position (tidal height + elevation)
        /// \param worldPos position in world reference frame, (x,y) are used as input and result is returned in z
        /// \param fc frame convention (NED/NWU)
        void GetPosition(Position &worldPos, FRAME_CONVENTION fc) const;

        //---------------------------- Wave field makers ----------------------------//

        /// Set the wave field model to a null wave field
        void NoWaves();

        /// Set the wave field model to an Airy regular wave field
        /// \return Airy regular wave field
        FrAiryRegularWaveField* SetAiryRegularWaveField();

        /// Set the wave field model to an Airy regular wave field
        /// \param waveHeight wave height
        /// \param wavePeriod wave period
        /// \param waveDirAngle wave direction angle
        /// \param unit wave direction angle unit
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        /// \return Airy regular wave field
        FrAiryRegularWaveField* SetAiryRegularWaveField(double waveHeight, double wavePeriod, double waveDirAngle,
                                                        mathutils::ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        /// Set the wave field model to an Airy regular wave field
        /// \param waveHeight wave height
        /// \param wavePeriod wave period
        /// \param waveDirection wave direction
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        /// \return Airy regular wave field
        FrAiryRegularWaveField* SetAiryRegularWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
                                                        FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        /// Set the wave field model to an Airy regular wave field optimized
        /// \return Airy regular wave field
        FrAiryRegularOptimWaveField* SetAiryRegularOptimWaveField();

        /// Set the wave field model to an Airy regular wave field optimized
        /// \param waveHeight wave height
        /// \param wavePeriod wave period
        /// \param waveDirAngle wave direction angle
        /// \param unit wave direction angle unit
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        /// \return Airy regular wave field
        FrAiryRegularOptimWaveField* SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, double waveDirAngle,
                                                                  mathutils::ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        /// Set the wave field model to an Airy regular wave field optimized
        /// \param waveHeight wave height
        /// \param wavePeriod wave period
        /// \param waveDirection wave direction
        /// \param fc frame convention (NED/NWU)
        /// \param dc direction convention (GOTO/COMEFROM)
        /// \return Airy regular wave field
        FrAiryRegularOptimWaveField* SetAiryRegularOptimWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
                                                        FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        /// Set the wave field model to an Airy irregular wave field
        /// \return Airy irregular wave field
        FrAiryIrregularWaveField* SetAiryIrregularWaveField();

        /// Set the wave field model to an Airy irregular wave field optimized
        /// \return Airy irregular wave field
        FrAiryIrregularOptimWaveField* SetAiryIrregularOptimWaveField();

        //---------------------------- Update-Initialize-StepFinalize ----------------------------//

        /// Initialize the state of the free surface
        void Initialize() override;

        /// Update the state of the free surface
        virtual void Update(double time);

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

    };

}  // end namespace frydom

#endif // FR_FREE_SURFACE_H
