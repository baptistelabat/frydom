//
// Created by Lucas Letournel on 06/12/18.
//

#ifndef FRYDOM_FRGRIDASSET_H
#define FRYDOM_FRGRIDASSET_H

#include "frydom/core/FrPhysicsItem.h"

namespace frydom {

    class FrFreeSurface_;
    class FrTriangleMeshConnected;

    class FrGridAsset : public FrPhysicsItem_{
    public:
        enum GRID_TYPE {
            NOGRID,  // TODO: utiliser si on ne veut pas montrer la SL
            CARTESIAN,
            POLAR
        };
    protected:
        // Mesh for the asset
        std::shared_ptr<chrono::ChTriangleMeshShape> m_meshAsset;

        bool m_updateAsset = false;

        NAMED_COLOR m_color = Gray;

        double m_gridHeight= 0;

    private:
        GRID_TYPE m_gridType = CARTESIAN;
        double m_xmin = -50.;
        double m_xmax = 50.;
        double m_dx = 1.;
        double m_ymin = -50.;
        double m_ymax = 50.;
        double m_dy = 1.;

        double m_xc0 = 0.;
        double m_yc0 = 0.;
        double m_diameter = 50.;
        int m_nbR = 50;
        int m_nbTheta = 36;


    public:

        FrGridAsset() = default;

        void SetGridHeight(double height);

        double GetGridHeight() const;

        void SetGridType(GRID_TYPE gridType);

        void  SetGridColor(NAMED_COLOR color);

        NAMED_COLOR GetColor() const;

        /// Initializes the free surface system
        /// In any case, a mesh grid is used.
        /// this version concerns a rectangular grid
        void SetGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy);

        /// Initializes the free surface system
        /// In any case, a mesh grid is used.
        /// this version concerns a square grid
        void SetGrid(double lmin, double lmax, double dl);

        void SetGrid(double xc0, double yc0, double diameter, int nbR, int nbTheta);

        void UpdateAssetON();

        void UpdateAssetOFF();

        void Update(double time) override {};

        void Initialize() override;

        void StepFinalize() override {};

    protected:

        /// Private method in charge of the building of the free surface mesh as a rectangular grid.
        std::shared_ptr<FrTriangleMeshConnected>
        BuildRectangularMeshGrid(double xmin, double xmax, double dx,
                                 double ymin, double ymax, double dy);

        /// Private method in charge of the building of the free surface mesh as a polar grid.
        std::shared_ptr<FrTriangleMeshConnected>
        BuildPolarMeshGrid(double xc0, double yc0, // center
                           double diameter,
                           unsigned int nbR, unsigned int nbTheta);
    };
}

#endif //FRYDOM_FRGRIDASSET_H
