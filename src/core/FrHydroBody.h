//Class Name
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRHYDROBODY_H
#define FRYDOM_FRHYDROBODY_H

#include "chrono/physics/ChBodyAuxRef.h"
#include "../misc/FrTriangleMeshConnected.h"

namespace frydom {

    class ChTriangleMeshConnected;

    class FrHydroBody : public chrono::ChBodyAuxRef,
                        public std::enable_shared_from_this<FrHydroBody>{

    private:
        std::shared_ptr<FrTriangleMeshConnected> hydro_mesh;
        std::shared_ptr<FrTriangleMeshConnected> visu_mesh;

    public:

        std::shared_ptr<FrHydroBody> GetShared();

        void SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset=true);
        void SetHydroMesh(std::string filename, bool as_asset=true);


    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROBODY_H
