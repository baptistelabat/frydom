//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "../environment/waves/FrFlatFreeSurface.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem(bool use_material_properties,
                                       unsigned int max_objects,
                                       double scene_size) :

            chrono::ChSystemSMC(use_material_properties, max_objects, scene_size),
            m_g_acc_magnitude(9.81),
            m_water_density(1025.),
            NEDframe(chrono::VNULL, M_PI, chrono::VECT_X) {

        Set_G_acc(chrono::ChVector<>(0., 0., -m_g_acc_magnitude));
    }

    std::shared_ptr<FrOffshoreSystem> FrOffshoreSystem::getPtr() {
        return shared_from_this();
    }

    void FrOffshoreSystem::setFreeSurface(environment::FrFreeSurface* freeSurface) {
        m_free_surface.reset(freeSurface);  // TODO: y a t il un moyen de gerer avec la move semantic ???
        // FIXME: Pourquoi dans le debugger, m_free_surface pointe sur un FrFreeSurface alors que dans demo on a fournit un FrFlatFreeSurface ???

        AddBody(m_free_surface->getBody());

    }

    void FrOffshoreSystem::setCurrent(environment::FrCurrent* current_field) {
        m_current.reset(current_field);
    }


    environment::FrFreeSurface* FrOffshoreSystem::getFreeSurface() const {
        return m_free_surface.get();  // FIXME: on ne devrait pas avoir besoin d'acceder au raw pointeur...
        // FIXME: comment directement acceder a m_free_surface via des indirections ????
    }

    environment::FrCurrent* FrOffshoreSystem::GetCurrent() const {
        return m_current.get();
    }

    void FrOffshoreSystem::SetGravityAcceleration(double grav) {
        assert(grav > 0.);
        m_g_acc_magnitude = grav;
        Set_G_acc(chrono::ChVector<>(0., 0., -m_g_acc_magnitude));
    }


    void FrOffshoreSystem::Update(bool update_assets) {
        timer_update.start();  // Timer for profiling

        // TODO: Mettre ici a jour tous les elements de l'environnement
        // Update all environment models (waves, wind, current...)


        // Executes the "forUpdate" in all controls of controlslist
        ExecuteControlsForUpdate();

        // Inherit parent class (recursively update sub objects bodies, links, etc)
        chrono::ChAssembly::Update(update_assets);

        // Update all contacts, if any
        contact_container->Update(ChTime, update_assets);

        timer_update.stop();
    }


}  // end namespace frydom
