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

#include "chrono/core/ChCoordsys.h"
#include "../../misc/FrTriangleMeshConnected.h"



namespace frydom{
namespace environment{

    /// Pure Virtual Base class for a free surface system.
    class FrFreeSurface {

      public:
        enum Type {
            FLAT,
            AIRY_REGULAR,
            AIRY_IRREGULAR,
            AIRY_IRREGULAR_DIR,
            NL_RIENECKER_FENTON,
            NL_HOS
        };

        /// Construct a default Free surface
        FrFreeSurface(double p_mean_height);

        virtual ~FrFreeSurface() {};

        /// Update the state of the free surface at the specified time.
        virtual void Synchronize(double time) {};

        /// Advance the state of the free surface by the specified duration.
        virtual void Advance(double step) {};

        /// Get the mean height of the free surface's plane.
        virtual double getMeanHeight() const;

        /// Get the free surface elevation at specified.
        virtual double GetHeight(double x, double y, double t) const = 0;

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

        FrTriangleMeshConnected getMesh(void) const;

      protected:
        FrFreeSurface() {};  // Disallow the default constructor to be used externally

        double m_mean_height;
        chrono::ChCoordsys<> plane;  /// The reference plane of the free surface
        FrTriangleMeshConnected m_mesh;

      private:
        bool m_vis_enabled;

        /// Private method in charge of the building of the free surface mesh.
        void build_mesh_grid(double xmin, double xmax, double dx,
                             double ymin, double ymax, double dy);
    };

}  // end namespace environment
}  // end namespace frydom

#endif // FR_FREE_SURFACE_H
