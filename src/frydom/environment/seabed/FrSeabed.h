//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRSEABED_H
#define FRYDOM_FRSEABED_H

#include "frydom/core/FrObject.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"



// Forward declarations of Chrono classes
namespace chrono {
    class ChBody;
    class ChTriangleMeshShape;
}


namespace frydom {

    class FrOffshoreSystem;
    class FrEnvironment;
    class FrBody;

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



    class FrEnvironment_;

    class FrSeabed_  : public FrObject {
    public:

        enum GRID_TYPE {
            CARTESIAN,
            POLAR
        };

    protected:

        bool m_updateAsset = false;

        FrEnvironment_* m_environment;

        std::shared_ptr<chrono::ChTriangleMeshShape> m_meshAsset;

        double m_depth = -30;

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

        explicit FrSeabed_(FrEnvironment_* environment);

        ~FrSeabed_() = default;

//        void SetEnvironment();

        void SetDepth(double depth);
        double GetDepth();

        /// get the body that represents the seabed
//        std::shared_ptr<FrBody> GetBody();

//        void UpdateAssetON() { m_updateAsset = true; }
//
//        void UpdateAssetOFF() { m_updateAsset = false; }

        void UpdateAsset(bool update);

        void SetGridType(GRID_TYPE gridType);

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
        void Update(double time);

        void Initialize() override;

        void StepFinalize() override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRSEABED_H
