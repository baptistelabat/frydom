//
// Created by frongere on 21/06/17.
//


#include <chrono/assets/ChTriangleMeshShape.h>

#include "FrHydroBody.h"
//#include "../misc/FrTriangleMeshConnected.h"

namespace frydom {


    void FrHydroBody::SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset) {

        hydro_mesh = mesh;

        if (as_asset) {
            // Registering the mesh as an asset
            auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
            shape->SetMesh(*mesh);
            AddAsset(shape);
        }

        // TODO: Ajouter automatiquement un clipper
    }

    void FrHydroBody::SetHydroMesh(std::string filename, bool as_asset) {
        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(filename);
        SetHydroMesh(mesh, as_asset);
    }
}  // end namespace frydom