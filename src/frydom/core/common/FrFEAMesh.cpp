//
// Created by lletourn on 06/03/19.
//

#include "FrFEAMesh.h"

#include <chrono/fea/ChMesh.h>

//void frydom::FrFEAMesh::SetupInitial() {
//    GetChronoMesh()->SetupInitial();
//}

void frydom::FrFEAMesh::Initialize() {
    // FIXME
//    GetChronoMesh()->SetupInitial();
}

void frydom::FrFEAMesh::StepFinalize() {

    FrAssetOwner::UpdateAsset();

}
