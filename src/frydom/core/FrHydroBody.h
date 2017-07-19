//Class Name
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRHYDROBODY_H
#define FRYDOM_FRHYDROBODY_H

#include "chrono/physics/ChBodyAuxRef.h"
#include "frydom/misc/FrTriangleMeshConnected.h"
#include "FrConstants.h"

// Forward declaration
namespace chrono {
    class ChBodyAuxRef;
}

namespace frydom {

    class FrHydroBody : public chrono::ChBodyAuxRef,
                        public std::enable_shared_from_this<FrHydroBody> {

    private:
        std::shared_ptr<FrTriangleMeshConnected> hydro_mesh;
        std::shared_ptr<FrTriangleMeshConnected> visu_mesh;

    public:

//        FrHydroBody() : ChBodyAuxRef() {};  // TODO: est-il utile d'appeler le constructeur par defaut de la classe mere ??

        std::shared_ptr<FrHydroBody> GetShared() { return shared_from_this(); };

        /// Set the hydrodynamic mesh from a mesh shared instance
        void SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset=true);

        /// Set the hydrodynamic mesh from a wavefront obj file
        void SetHydroMesh(std::string obj_filename, bool as_asset=true);

        /// Get the current vector flow as seen by the moving body on water
        chrono::ChVector<> GetCurrentRelativeVelocity(FrFrame frame=NWU);

    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROBODY_H
