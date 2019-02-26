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


#ifndef FRYDOM_FRSEABED_H
#define FRYDOM_FRSEABED_H


#include "frydom/core/common/FrObject.h"
#include "frydom/core/common/FrConvention.h"

namespace frydom {

    // Forward declarations
    class FrOcean;
    class FrSeabedGridAsset;

    /**
     * \class FrSeabed_
     * \brief Class for defining a seabed with either a finite or infinite water depth.
     */
    class FrSeabed_  : public FrObject {
    protected:

        FrOcean *m_ocean;            ///< Pointer to the ocean containing this asset
        bool m_infiniteDepth = false; ///< true if the infinite depth condition is applied

    public:
        /// Default constructor
        /// \param ocean ocean containing this seabed
        explicit FrSeabed_(FrOcean* ocean);

        /// Default destructor
        ~FrSeabed_() = default;

        //---------------------------- Asset ----------------------------//

        /// Get the seabed grid asset
        /// \return seabed grid asset
        virtual FrSeabedGridAsset * GetSeabedGridAsset() = 0;

        //---------------------------- Seabed elements Getters ----------------------------//

        /// Get the ocean containing this seabed
        /// \return ocean containing this seabed
        FrOcean* GetOcean() const;

        /// Set the mean bathymetry of the seabed (negative in NWU/positive in NED)
        /// \param bathymetry mean bathymetry of the seabed
        /// \param fc frame convention (NED/NWU)
        virtual void SetBathymetry(double bathymetry, FRAME_CONVENTION fc) = 0;

        /// Get the mean bathymetry of the seabed
        /// \param fc frame convention (NED/NWU)
        /// \return mean bathymetry of the seabed
        virtual const double GetBathymetry(FRAME_CONVENTION fc) const = 0;

        /// Get the local bathymetry at the position (x,y)
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return local bathymetry
        virtual const double GetBathymetry(double x, double y, FRAME_CONVENTION fc) const = 0;

        /// Check if the infinite depth condition is applied. If true, a FrNullSeabed is instantiated, otherwise it is a
        /// FrMeanSeabed.
        /// \return true if the infinite depth condition is applied.
        bool GetInfiniteDepth();

        //---------------------------- Update-Initialize-StepFinalize ----------------------------//

        /// Update the state of the seabed
        virtual void Update(double time) = 0;

        /// Initialize the state of the seabed
        void Initialize() override = 0;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override = 0;

    };

    /**
     * \class FrNullSeabed_
     * \brief Class for defining a seabed in case of infinite water depth.
     */
    class FrNullSeabed_  : public FrSeabed_ {
    public:

        /// Default constructor
        /// \param ocean ocean containing this seabed
        explicit FrNullSeabed_(FrOcean* ocean);

        //---------------------------- Asset ----------------------------//

        /// Get the seabed grid asset
        /// \return seabed grid asset
        FrSeabedGridAsset * GetSeabedGridAsset() override;

        //---------------------------- Seabed elements Getters ----------------------------//

        /// Set the mean bathymetry of the seabed (negative in NWU/positive in NED)
        /// \param bathymetry mean bathymetry of the seabed
        /// \param fc frame convention (NED/NWU)
        void SetBathymetry(double bathymetry, FRAME_CONVENTION fc) override;

        /// Get the mean bathymetry of the seabed
        /// \param fc frame convention (NED/NWU)
        /// \return mean bathymetry of the seabed
        const double GetBathymetry(FRAME_CONVENTION fc) const override;

        /// Get the local bathymetry at the position (x,y)
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return local bathymetry
        const double GetBathymetry(double x, double y, FRAME_CONVENTION fc) const override;

        //---------------------------- Update-Initialize-StepFinalize ----------------------------//

        /// Update the state of the seabed
        void Update(double time) override;

        /// Initialize the state of the seabed
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;
    };

    /**
     * \class FrMeanSeabed_
     * \brief Class for defining a mean seabed in case of finite and constant water depth.
     */
    class FrMeanSeabed_  : public FrSeabed_ {
    protected:

        bool m_showSeabed = true;     ///< Boolean checking if the seabed is shown/exists
                                      ///< It is turned to false also if the infinite depth condition is enforced.
        std::shared_ptr<FrSeabedGridAsset> m_SeabedGridAsset;    ///> Seabed grid asset, containing also its asset visualization
        double m_bathymetry = -30;    ///< Mean bathymetry, in NWU

        //TODO : consider varying bathymetry

    public:

        /// Default constructor
        /// \param ocean ocean containing this seabed
        explicit FrMeanSeabed_(FrOcean* ocean);

        /// Default destructor
        ~FrMeanSeabed_() = default;

        //---------------------------- Asset ----------------------------//

        /// Get the seabed grid asset
        /// \return seabed grid asset
        FrSeabedGridAsset * GetSeabedGridAsset() override;

        //---------------------------- Seabed elements Getters ----------------------------//

        /// Set the mean bathymetry of the seabed (negative in NWU/positive in NED)
        /// \param bathymetry mean bathymetry of the seabed
        /// \param fc frame convention (NED/NWU)
        void SetBathymetry(double bathymetry, FRAME_CONVENTION fc) override;

        /// Get the mean bathymetry of the seabed
        /// \param fc frame convention (NED/NWU)
        /// \return mean bathymetry of the seabed
        const double GetBathymetry(FRAME_CONVENTION fc) const override;

        /// Get the local bathymetry at the position (x,y)
        /// \param x x position
        /// \param y y position
        /// \param fc frame convention (NED/NWU)
        /// \return local bathymetry
        const double GetBathymetry(double x, double y, FRAME_CONVENTION fc) const override;

        //---------------------------- Update-Initialize-StepFinalize ----------------------------//

        /// Update the state of the seabed
        void Update(double time) override;

        /// Initialize the state of the seabed
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRSEABED_H
