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


// Forward declarations of Chrono classes
namespace chrono {
    class ChBody;
    class ChTriangleMeshShape;
}


namespace frydom {

    class FrOffshoreSystem;
    class FrEnvironment;
    class FrBody;
    class FrTriangleMeshConnected;

    /**
     * \class FrSeabed
     * \brief Class for defining a seabed.
     */
    class FrSeabed  : public FrObject {
    public:

        enum GRID_TYPE {
            CARTESIAN,
            POLAR
        };

    protected:

        bool m_updateAsset = false;

        FrEnvironment* m_environment;

        std::shared_ptr<chrono::ChTriangleMeshShape> m_meshAsset;

        double m_depth = -30;  // FIXME : Pourquoi cette valeur par defaut ? --> plutot utiliser une valeur positive

        GRID_TYPE m_gridType = CARTESIAN;
        double m_xmin = -150.;
        double m_xmax = 150.;
        double m_dx = 3.;
        double m_ymin = -150.;
        double m_ymax = 150.;
        double m_dy = 3.;

        double m_xc0 = 0.;
        double m_yc0 = 0.;
        double m_diameter = 150.;
        int m_nbR = 50;
        int m_nbTheta = 36;

        /// Private method in charge of the building of the seabed mesh as a rectangular grid.
        FrTriangleMeshConnected BuildRectangularMeshGrid(double xmin, double xmax, double dx,
                                                         double ymin, double ymax, double dy);

        /// Private method in charge of the building of the seabed mesh as a polar grid.
        FrTriangleMeshConnected BuildPolarMeshGrid(double xc0, double yc0, // center
                                                   double diameter,
                                                   unsigned int nbR, unsigned int nbTheta);

    public:

        FrSeabed();

        ~FrSeabed() = default;

        void SetEnvironment(FrEnvironment* environment);

        void SetDepth(double depth) {m_depth = depth;}
        double GetDepth() {return m_depth;}

        /// get the body that represents the seabed
        std::shared_ptr<FrBody> GetBody();

        void UpdateAssetON() { m_updateAsset = true; }

        void UpdateAssetOFF() { m_updateAsset = false; }

        void SetGridType(GRID_TYPE gridType) {
            m_gridType = gridType;
        }

        /// Initializes the seabed system
        /// In any case, a mesh grid is used.
        /// this version concerns a rectangular grid
        void SetGrid(double xmin,
                     double xmax,
                     double dx,
                     double ymin,
                     double ymax,
                     double dy
        );

        /// Initializes the seabed system
        /// In any case, a mesh grid is used.
        /// this version concerns a square grid
        void SetGrid(double lmin,
                     double lmax,
                     double dl
        );

        void SetGrid(double xc0,
                     double yc0,
                     double diameter,
                     int nbR,
                     int nbTheta
        );


        /// Update the state of the seabed
        void Update(double time) {}

        void Initialize() override;

        void StepFinalize() override {}

    };














    // REFACTORING ---------->>>>>>>>>>>>>>>



    class FrOcean_;
    class FrSeabedGridAsset;
//    using FrSeabedGridAsset = FrGridAsset;

    /**
     * \class FrSeabed_
     * \brief Class for defining a seabed with either a finite or infinite water depth.
     */
    class FrSeabed_  : public FrObject {
    protected:

        FrOcean_ *m_ocean;            ///< Pointer to the ocean containing this asset
        bool m_infiniteDepth = false; ///< true if the infinite depth condition is applied

    public:
        /// Default constructor
        /// \param ocean ocean containing this seabed
        explicit FrSeabed_(FrOcean_* ocean);

        /// Default destructor
        ~FrSeabed_() = default;

        //---------------------------- Asset ----------------------------//

        /// Get the seabed grid asset
        /// \return seabed grid asset
        virtual FrSeabedGridAsset * GetSeabedGridAsset() = 0;

        //---------------------------- Seabed elements Getters ----------------------------//

        /// Get the ocean containing this seabed
        /// \return ocean containing this seabed
        FrOcean_* GetOcean() const;

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
        explicit FrNullSeabed_(FrOcean_* ocean);

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
        explicit FrMeanSeabed_(FrOcean_* ocean);

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
