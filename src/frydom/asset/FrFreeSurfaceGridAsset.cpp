// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#include "FrFreeSurfaceGridAsset.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean_.h"

#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom{


    FrFreeSurfaceGridAsset::FrFreeSurfaceGridAsset(FrBody_ *body, FrFreeSurface_* freeSurface) : FrGridAsset(body) {
        m_freeSurface = freeSurface;
        m_gridHeight = m_freeSurface->GetTidal()->GetHeight(NWU);
        m_color = DodgerBlue;
    }

    void FrFreeSurfaceGridAsset::StepFinalize() {
        FrGridAsset::StepFinalize();
        if (fmod(c_currentStep,m_updateStep)==0) {
            auto& mesh = m_meshAsset->GetMesh();
            auto nbNodes = mesh.m_vertices.size();
            for (unsigned int inode = 0; inode < nbNodes; ++inode) {
                mesh.m_vertices[inode].z() = m_freeSurface->GetPosition(mesh.m_vertices[inode].x(),
                                                                                 mesh.m_vertices[inode].y(),NWU);
            }
        }

    }

}
