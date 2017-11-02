//
// Created by frongere on 29/05/17.
//

#include "FrOffshoreSystem.h"
#include "frydom/environment/FrEnvironment.h"

namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem(bool use_material_properties,
                                       unsigned int max_objects,
                                       double scene_size) :

            chrono::ChSystemSMC(use_material_properties, max_objects, scene_size),
//            m_gravity_acc_magnitude(9.81),
//            m_water_density(1025.),
            NEDframe(chrono::VNULL, M_PI, chrono::VECT_X) {

        // The world body is a virtual body with no mass that is fixed and used to fix nodes in the absolute frame
        world_body = std::make_shared<FrBody>();
        world_body->SetSystem(this);
        world_body->SetBodyFixed(true);
        world_body->SetName("WorldBody");


        // TODO: mettre dans une methode SetEnvironment
        m_environment = std::make_unique<FrEnvironment>();
        m_environment->SetSystem(this);
        AddBody(m_environment->GetFreeSurface()->GetBody());

        // Convention for z orientation is upward
        Set_G_acc(chrono::ChVector<>(0., 0., -m_environment->GetGravityAcceleration()));

    }

//    void FrOffshoreSystem::setFreeSurface(FrFreeSurface* freeSurface) {
//        // TODO: accepter plutot directement le unique_ptr et faire std::move...
//        m_free_surface.reset(freeSurface);  // TODO: y a t il un moyen de gerer avec la move semantic ???
//        // FIXME: Pourquoi dans le debugger, m_free_surface pointe sur un FrFreeSurface alors que dans demo on a fournit un FrFlatFreeSurface ???
//
//        AddBody(m_free_surface->GetBody());
//
//    }
//
//    void FrOffshoreSystem::SetCurrent(FrCurrent *current_field) {
//        m_current.reset(current_field);
//    }
//
//    FrFreeSurface* FrOffshoreSystem::getFreeSurface() const {
//        // TODO: gerer une erreur si la free surface n'est pas renseignee ...
//        return m_free_surface.get();  // FIXME: on ne devrait pas avoir besoin d'acceder au raw pointeur...
//        // FIXME: comment directement acceder a m_free_surface via des indirections ????
//    }
//
//    FrCurrent* FrOffshoreSystem::GetCurrent() const {
////        if (m_current) {
////            return m_current.get();
////        } else {
////            // TODO: creer propre classe d'erreur frydom::no_current_field
////            throw std::runtime_error("Pas de courant");
////        }
//        return m_current.get();
//    }
//
//    void FrOffshoreSystem::SetGravityAcceleration(double grav) {
//        assert(grav > 0.);
//        m_gravity_acc_magnitude = grav;
//        Set_G_acc(chrono::ChVector<>(0., 0., -m_gravity_acc_magnitude));
//    }


    void FrOffshoreSystem::Update(bool update_assets) {
        timer_update.start();  // Timer for profiling

        // TODO: Mettre ici a jour tous les elements de l'environnement
        // Update all environment models (waves, wind, current...)

        // Current model
        m_environment->GetCurrent()->Update(ChTime);

        // Wind model
//        m_environment->GetWind()->Update(ChTime);

        // Wave model
        m_environment->GetFreeSurface()->Update(ChTime);

        // Executes the "forUpdate" in all controls of controlslist
        ExecuteControlsForUpdate();

        // Inherit parent class (recursively update sub objects bodies, links, etc)
        chrono::ChAssembly::Update(update_assets);

        // Update all contacts, if any
        contact_container->Update(ChTime, update_assets);

        timer_update.stop();
    }



}  // end namespace frydom
