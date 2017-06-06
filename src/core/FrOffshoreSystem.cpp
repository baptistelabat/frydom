//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem(bool use_material_properties,
                                       unsigned int max_objects,
                                       double scene_size) :

            chrono::ChSystemSMC(use_material_properties, max_objects, scene_size) {

        Set_G_acc(chrono::ChVector<>(0., 0., -9.81));
    }

    std::shared_ptr<FrOffshoreSystem> FrOffshoreSystem::getPtr() {
        return shared_from_this();
    }

    void FrOffshoreSystem::setFreeSurface(environment::FrFreeSurface* freeSurface) {
        m_free_surface.reset(freeSurface);  // TODO: y a t il un moyen de gerer avec la move semantic ???
        // FIXME: Pourquoi dans le debugger, m_free_surface pointe sur un FrFreeSurface alors que dans demo on a fournit un FrFlatFreeSurface ???

        AddBody(m_free_surface->getBody());

    }

    environment::FrFreeSurface* FrOffshoreSystem::getFreeSurface() {
        return m_free_surface.get();  // FIXME: on ne devrait pas avoir besoin d'acceder au raw pointeur...
        // FIXME: comment directement acceder a m_free_surface via des indirections ????
    }

}  // end namespace frydom
