//
// Created by Lucas Letournel on 05/12/18.
//

#include "FrFreeSurfaceGridAsset.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"

#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom{


    FrFreeSurfaceGridAsset::FrFreeSurfaceGridAsset(FrFreeSurface_ *freeSurface) {
        m_freeSurface = freeSurface;
        m_gridHeight = m_freeSurface->GetTidal()->GetHeight();
        m_color = DodgerBlue;
    }

    void FrFreeSurfaceGridAsset::Update(double time) {

        if (m_updateAsset) {
            auto& mesh = m_meshAsset->GetMesh();
            auto nbNodes = mesh.m_vertices.size();
            for (unsigned int inode = 0; inode < nbNodes; ++inode) {
                mesh.m_vertices[inode].z() = m_freeSurface->GetPosition(mesh.m_vertices[inode].x(),
                                                                                 mesh.m_vertices[inode].y());
            }
        }

    }

}