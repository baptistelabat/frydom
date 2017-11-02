//
// Created by frongere on 10/10/17.
//

#include "FrEnvironment.h"
#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {


    void FrEnvironment::SetFreeSurface(FrFreeSurface *freeSurface) {
        m_freeSurface = std::unique_ptr<FrFreeSurface>(freeSurface);
        m_system->AddBody(m_freeSurface->GetBody());
    }
}