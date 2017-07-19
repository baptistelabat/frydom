//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBodyAuxRef.h"
#include <chrono/assets/ChTriangleMeshShape.h>

#include "FrHydroBody.h"
#include "FrOffshoreSystem.h"

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

    void FrHydroBody::SetHydroMesh(std::string obj_filename, bool as_asset) {
        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        SetHydroMesh(mesh, as_asset);
    }

    chrono::ChVector<> FrHydroBody::GetCurrentFlow(FrFrame frame) {

        // Get the body's velocity
        auto body_velocity = GetPos_dt();

        // Get the current velocity
        auto mysystem = dynamic_cast<FrOffshoreSystem*>(system);
        auto current_velocity = mysystem->GetCurrent()->GetVelocityVector();


        auto current_flow = chrono::ChVector<>();
        return current_flow;
    }


}  // end namespace frydom