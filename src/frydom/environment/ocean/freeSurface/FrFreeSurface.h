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

#include "frydom/core/FrObject.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"



// Forward declarations
namespace chrono {

    class ChBody;

    class ChTriangleMeshShape;
}


namespace frydom{

    // Forward declaration
//    class FrOffshoreSystem;
    class FrTidal;
//    class FrWaveField;
//    class FrLinearWaveField;
    class FrLinearWaveProbe;

    class FrTriangleMeshConnected;




    /// Pure Virtual Base class for a free surface system.
    class FrFreeSurface : public FrObject {

    public:

        enum GRID_TYPE {
            CARTESIAN,
            POLAR
        };
        // FIXME: Il faut que ce soit cette classe qui comprenne un modele de maree !!


        std::shared_ptr<chrono::ChBody> m_Body;  // TODO : mettre en protected !!

    protected:;  // Disallow the default constructor to be used as a public method // TODO: mettre private???

        double m_time = 0.;
        bool m_updateAsset = false;


        std::unique_ptr<FrTidal> m_tidal;

        WAVE_MODEL m_waveModel = NO_WAVES;
        std::unique_ptr<FrWaveField> m_waveField;

        std::shared_ptr<chrono::ChTriangleMeshShape> m_meshAsset;

        std::vector<std::shared_ptr<FrLinearWaveProbe>> m_waveProbeGrid; // TODO: passer a la classe de base...

        std::vector<double> m_gridHeights; // TODO: preallouer a l'initialisation

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

        ~FrFreeSurface();

        void NoWaves();

        void SetLinearWaveField(LINEAR_WAVE_TYPE waveType);

        FrLinearWaveField* GetLinearWaveField() const;

        double GetMeanHeight(double x, double y);

        double GetHeight(double x, double y);

        virtual void Initialize() override;

        void SetGridType(GRID_TYPE gridType);

        /// Initializes the free surface system
        /// In any case, a mesh grid is used.
        /// this version concerns a rectangular grid
        void SetGrid(double xmin,
                     double xmax,
                     double dx,
                     double ymin,
                     double ymax,
                     double dy
        );

        /// Initializes the free surface system
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


        /// Get the free surface's mesh
//        FrTriangleMeshConnected getMesh(void) const;

        void UpdateAssetON() { m_updateAsset = true; }

        void UpdateAssetOFF() { m_updateAsset = false; }

        /// Update the state of the free surface
        virtual void Update(double time);

        void UpdateGrid();

        void UpdateGridRange(std::pair<unsigned int, unsigned int> range);


        FrTidal* GetTidal() const;

        /// get the body that represents the free surface
        std::shared_ptr<chrono::ChBody> GetBody();

//        void SetWaveField(std::shared_ptr<FrWaveField> waveField) { m_waveField = waveField; }

        FrWaveField* GetWaveField() const;

//        const chrono::ChFrame<double>* GetFrame() const;

        virtual void StepFinalize() override {}

    };











    //// REFACTORING -------->>>>>>>>>>>






    // Forward declarations
    class FrOffshoreSystem_;
    class FrEnvironment_;
    class FrAtmosphere_;
    class FrOcean_;
    class FrTidal_;
    class FrBody_;
    class FrAiryRegularWaveField;



    class FrFreeSurfaceAsset {

    };


    /// Pure Virtual Base class for a free surface system.
    class FrFreeSurface_ : public FrObject {

    public:

        enum GRID_TYPE {
            NONE,  // TODO: utiliser si on ne veut pas montrer la SL
            CARTESIAN,
            POLAR
        };

        std::shared_ptr<FrBody_> m_body; // TODO : replacer en protected

    protected:;  // Disallow the default constructor to be used as a public method // TODO: mettre private???

        double m_time = 0.;
        bool m_updateAsset = false;

        FrOcean_* m_ocean;

        // Free surface elements

        std::unique_ptr<FrTidal_> m_tidal;

        std::unique_ptr<FrWaveField_> m_waveField;

        // Mesh for the asset
        std::shared_ptr<FrTriangleMeshConnected> m_meshAsset;

        std::vector<std::shared_ptr<FrLinearWaveProbe_>> m_waveProbeGrid; // TODO: passer a la classe de base...

//        std::vector<double> m_gridHeights; // TODO: preallouer a l'initialisation

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

        void CreateFreeSurfaceBody();

    public:

        explicit FrFreeSurface_(FrOcean_* ocean);

        ~FrFreeSurface_();

        double GetTime() const;

        FrOcean_* GetOcean() const;

        FrAtmosphere_* GetAtmosphere() const;;

        FrTidal_* GetTidal() const;

        FrWaveField_ * GetWaveField() const;

        void NoWaves();

        FrAiryRegularWaveField* SetAiryRegularWaveField();
        
        FrAiryRegularWaveField* SetAiryRegularWaveField(double waveHeight, double wavePeriod, double waveDirAngle,
                                                        ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);
        
        FrAiryRegularWaveField* SetAiryRegularWaveField(double waveHeight, double wavePeriod, const Direction& waveDirection,
                                                        FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);


        void SetLinearWaveField(LINEAR_WAVE_TYPE waveType);

        FrLinearWaveField* GetLinearWaveField() const;

        double GetMeanHeight() const;

        double GetHeight(double x, double y) const;

        virtual void Initialize() override;

        void SetGridType(GRID_TYPE gridType);

        /// Initializes the free surface system
        /// In any case, a mesh grid is used.
        /// this version concerns a rectangular grid
        void SetGrid(double xmin, double xmax, double dx, double ymin, double ymax, double dy);

        /// Initializes the free surface system
        /// In any case, a mesh grid is used.
        /// this version concerns a square grid
        void SetGrid(double lmin, double lmax, double dl);

        void SetGrid(double xc0, double yc0, double diameter, int nbR, int nbTheta);


        /// Get the free surface's mesh

        void UpdateAsset(bool update);

        void UpdateAssetON();

        void UpdateAssetOFF();

        /// Update the state of the free surface
        virtual void Update(double time);

        void StepFinalize() override {}

    private:

        void UpdateGrid();

    };


}  // end namespace frydom

#endif // FR_FREE_SURFACE_H
