//
// Created by Lucas Letournel on 06/12/18.
//

#ifndef FRYDOM_FRGRIDASSET_H
#define FRYDOM_FRGRIDASSET_H

#include <chrono/assets/ChAssetLevel.h>
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "frydom/core/FrObject.h"
#include "frydom/core/FrColors.h"

namespace frydom {

    class FrFreeSurface_;
    class FrTriangleMeshConnected;
    class FrBody_;

    class FrGridAsset : public FrObject  {
    public:
        enum GRID_TYPE {
            NOGRID,  // TODO: utiliser si on ne veut pas montrer la SL
            CARTESIAN,
            POLAR
        };
    protected:
        /// body containing the grid asset
        FrBody_* m_body;
        /// Mesh asset for the visualization
        std::shared_ptr<chrono::ChTriangleMeshShape> m_meshAsset;
        /// Color asset for the visualization
        std::shared_ptr<chrono::ChColorAsset> m_colorAsset;

        /// Boolean to check if the grid asset is updated (in position, color, etc.) during the simulation
//        bool m_updateAsset = false;
        /// The free surface grid asset position is updated every m_updateStep simulation steps
        int m_updateStep = 0;
        int c_currentStep;

        /// Color of the grid asset
        NAMED_COLOR m_color = Gray;
        /// Z position of the grid asset in the world reference frame
        double m_gridHeight= 0;

    private:
        /// Grid type : can be CARTESIAN, POLAR or NOGRID. In this last case, the asset is not visualized
        GRID_TYPE m_gridType = CARTESIAN;
        /// Parameters for the CARTESIAN grid
        double m_xmin = -50.;
        double m_xmax = 50.;
        double m_dx = 1.;
        double m_ymin = -50.;
        double m_ymax = 50.;
        double m_dy = 1.;
        /// Parameters for the POLAR grid
        double m_xc0 = 0.;
        double m_yc0 = 0.;
        double m_diameter = 50.;
        int m_nbR = 50;
        int m_nbTheta = 36;

    public:
        /// Default constructor of the grid asset
        explicit FrGridAsset(FrBody_* body);

        /// grid asset is set to NOGRID and the asset is not visualized or updated
        void SetNoGrid();

        /// Set the Z position of the grid in the world reference frame
        /// \param height Z position in the world reference frame
        void SetGridHeight(double height);

        /// Set the Z position of the grid in the world reference frame
        /// \return Z position in the world reference frame
        virtual double GetGridHeight() const;

        /// Set the grid color
        /// \param color color of the grid
        void  SetGridColor(NAMED_COLOR color);

        /// Get the grid color
        /// \return grid color
        NAMED_COLOR GetGridColor() const;

        /// Set the grid asset to a CARTESIAN grid, with constants x and y discretizations
        /// \param xmin xmin of the grid
        /// \param xmax xmax of the grid
        /// \param dx x discretization of the grid
        /// \param ymin ymin of the grid
        /// \param ymax ymax of the grid
        /// \param dy y discretization of the grid
        void SetGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy);

        /// Set the grid asset to a square CARTESIAN grid, with constants x and y discretizations
        /// \param lmin lmin of the grid
        /// \param lmax lmax of the grid
        /// \param dl discretization of the grid
        void SetGrid(double lmin, double lmax, double dl);

        /// Set the grid asset to a POLAR grid, with constants discretizations in radius and angle
        /// \param xc0 X position of the center of the grid
        /// \param yc0 Y position of the center of the grid
        /// \param diameter diameter of the grid
        /// \param nbR radius discretization of the grid
        /// \param nbTheta angle discretization of the grid
        void SetGrid(double xc0, double yc0, double diameter, int nbR, int nbTheta);

        /// Asset characteristics (position color, etc.) are updated during the simulation
        void UpdateAssetON();

        /// Asset characteristics (position color, etc.) are NOT updated during the simulation
        void UpdateAssetOFF();

        /// Set the update step parameters : the asset characteristics (position, colors, etc.) are then updated
        /// every m_updateStep steps.
        void SetUpdateStep(int nStep);

        /// GridAsset initialization method
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;;

    protected:

        /// Private method in charge of the building of the free surface mesh as a rectangular grid.
        /// \param xmin xmin of the grid
        /// \param xmax xmax of the grid
        /// \param dx x discretization of the grid
        /// \param ymin ymin of the grid
        /// \param ymax ymax of the grid
        /// \param dy y discretization of the grid
        /// \return CARTESIAN grid
        std::shared_ptr<FrTriangleMeshConnected>
        BuildRectangularMeshGrid(double xmin, double xmax, double dx,
                                 double ymin, double ymax, double dy);

        /// Private method in charge of the building of the free surface mesh as a polar grid.
        /// \param xc0 X position of the center of the grid
        /// \param yc0 Y position of the center of the grid
        /// \param diameter diameter of the grid
        /// \param nbR radius discretization of the grid
        /// \param nbTheta angle discretization of the grid
        /// \return POLAR grid
        std::shared_ptr<FrTriangleMeshConnected>
        BuildPolarMeshGrid(double xc0, double yc0, // center
                           double diameter,
                           unsigned int nbR, unsigned int nbTheta);

        /// Set the grid asset type
        /// \param gridType type of the grid asset
        void SetGridType(GRID_TYPE gridType);

    };
}

#endif //FRYDOM_FRGRIDASSET_H
