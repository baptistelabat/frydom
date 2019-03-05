// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#include "FrFreeSurfaceGridAsset.h"

#include "chrono/assets/ChTriangleMeshShape.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"


namespace frydom{


    FrFreeSurfaceGridAsset::FrFreeSurfaceGridAsset(FrFreeSurface* freeSurface) : FrGridAsset() {
        m_freeSurface = freeSurface;
        m_gridHeight = m_freeSurface->GetTidal()->GetHeight(NWU);
        m_color = DodgerBlue;
    }

    void FrFreeSurfaceGridAsset::StepFinalize() {
        FrGridAsset::StepFinalize();
        if (fmod(c_currentStep,m_updateStep)==0) {
            auto mesh = dynamic_cast<chrono::ChTriangleMeshShape*>(m_chronoAsset->GetAssetN(0).get())->GetMesh();
            auto nbNodes = mesh->m_vertices.size();
            for (unsigned int inode = 0; inode < nbNodes; ++inode) {
                mesh->m_vertices[inode].z() = m_freeSurface->GetPosition(mesh->m_vertices[inode].x(),
                                                                                 mesh->m_vertices[inode].y(),NWU);
            }
        }
    }


}  // end namespace frydom
