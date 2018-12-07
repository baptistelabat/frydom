//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRSEABED_H
#define FRYDOM_FRSEABED_H

#include "frydom/core/FrObject.h"



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














    /// REFACTORING ---------->>>>>>>>>>>>>>>



    class FrOcean_;
    class FrSeabedGridAsset;
//    using FrSeabedGridAsset = FrGridAsset;

    class FrSeabed_  : public FrObject {
    protected:

        /// Pointer to the ocean containing this asset
        FrOcean_* m_ocean;

        /// Boolean checking if the seabed is shown/exists
        bool m_showSeabed = true;

        /// Seabed grid asset, containing also its asset visualization
        std::shared_ptr<FrSeabedGridAsset> m_SeabedGridAsset;

        /// Mean bathymetry
        double m_Bathymetry = -30;

        //TODO : consider varying bathymetry

    public:

        /// Default constructor
        /// \param ocean ocean containing this seabed
        explicit FrSeabed_(FrOcean_* ocean);

        /// Default destructor
        ~FrSeabed_() = default;

        //---------------------------- Asset ----------------------------//
        /// Set if the seabed is to be shown/exist
        /// \param showSeabed showseabed true means the seabed exists
        void ShowSeabed(bool showSeabed);

        /// Get the seabed grid asset
        /// \return seabed grid asset
        FrSeabedGridAsset * GetSeabedGridAsset();

        //---------------------------- Seabed elements Getters ----------------------------//

        /// Get the ocean containing this seabed
        /// \return ocean containing this seabed
        FrOcean_* GetOcean() const;

        /// Set the mean bathymetry of the seabed
        /// \param bathymetry mean bathymetry of the seabed
        void SetBathymetry(double bathymetry);

        /// Get the mean bathymetry of the seabed
        /// \return mean bathymetry of the seabed
        const double GetBathymetry() const;

        /// Get the local bathymetry at the position (x,y)
        /// \param x x position
        /// \param y y position
        /// \return local bathymetry
        const double GetBathymetry(double x, double y) const;

        //---------------------------- Update-Initialize-StepFinalize ----------------------------//

        /// Update the state of the seabed
        void Update(double time);

        /// Initialize the state of the seabed
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRSEABED_H
