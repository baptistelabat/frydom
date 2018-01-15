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



    void FrOffshoreSystem::Update(bool update_assets) {
        timer_update.start();  // Timer for profiling

        // TODO: Mettre ici a jour tous les elements de l'environnement
        // Update all environment models (waves, wind, current...)
        m_environment->Update(ChTime);

        // Executes the "forUpdate" in all controls of controlslist
        ExecuteControlsForUpdate();

        // Inherit parent class (recursively update sub objects bodies, links, etc)
        chrono::ChAssembly::Update(update_assets);

        // Update all contacts, if any
        contact_container->Update(ChTime, update_assets);

        timer_update.stop();
    }

    // From state Y={x,v} to system.
    void FrOffshoreSystem::StateScatter(const chrono::ChState& x, const chrono::ChStateDelta& v, const double T) {

        m_environment->Update(T);  // Updating environment

        IntStateScatter(0, x, 0, v, T);  // TODO: voir pour faire un update de l'environnement juste avant cette ligne ...

//        Update();  //***TODO*** optimize because maybe IntStateScatter above might have already called Update?
    }

    bool FrOffshoreSystem::Integrate_Y() {
        ResetTimers();

        timer_step.start();

        // Executes "forStep" in all controls of controlslist
        ExecuteControlsForStep();  // C'est ici qu'on pourra trigger les calculs de controllers... Voir avec sof si
                                   // c'est le bon endroit / si l'objet control de chrono convient

        stepcount++;
        solvecount = 0;
        setupcount = 0;

        // Compute contacts and create contact constraints
        ComputeCollisions();

        // Counts dofs, statistics, etc. (not needed because already in Advance()...? ) // TODO: voir ce qu'il en est
        Setup();

        // Update everything - and put to sleep bodies that need it (not needed because already in Advance()...? )
        // No need to update visualization assets here.
//        Update(true);  // FIXME : Desactive car redondant avec ce qui est deja fait lors du system::StateScatter()...

        // Re-wake the bodies that cannot sleep because they are in contact with
        // some body that is not in sleep state.
//        ManageSleepingBodies(); // Proposer au chrono group que cette methode soit protected afi de pouvoir
                                  // completement deriver de Integrate_Y()...

        // Prepare lists of variables and constraints.
        DescriptorPrepareInject(*descriptor);
        descriptor->UpdateCountsAndOffsets();

        // Set some settings in timestepper object
        timestepper->SetQcDoClamp(true);
        timestepper->SetQcClamping(max_penetration_recovery_speed);
        // TODO: reactiver les 3 lignes suivantes pour autoriser l'utilisation des solveurs HHT et Newmark
//        if (std::dynamic_pointer_cast<ChTimestepperHHT>(timestepper) ||
//            std::dynamic_pointer_cast<ChTimestepperNewmark>(timestepper))
//            timestepper->SetQcDoClamp(false);

        // PERFORM TIME STEP HERE!
        timestepper->Advance(step);  // Ici, on passe du temps courant au temps suivant en utilisant le shema du timestepper choisi

        // Executes custom processing at the end of step
        CustomEndOfStep();

        // If there are some probe objects in the probe list,
        // tell them to record their variables (ususally x-y couples)
        RecordAllProbes(); // Voir a utiliser les ChProbe pour les capteurs controle. On pourra utiliser pour la radiation et l'enregistremet en buffer circulaire...

        // Call method to gather contact forces/torques in rigid bodies
        contact_container->ComputeContactForces();

        // TODO: ici, appeler une methode sur tout l'assembly permettant de faire une finalisation du pas de temps courant
        // ou alors le faire en tout debut de cette methode pour avoir aussi le t=0 ?
        // Cet appel permettra de trigger le log aux pas de temps fixes.
        // Voir aussi la methode CurtomEndOfStep() ci-dessus qui pourrait faire tout a fait l'affaire...

        // Time elapsed for step..
        timer_step.stop();


        std::cout << "Framerate = " << 1. / timer_step.GetTimeSeconds() << " FPS" << std::endl;

        return true;
    }

    void FrOffshoreSystem::CustomEndOfStep() {
        // TODO : Ici on a bon candidat pour trigger l'emission des donnees des objets...
        std::cout << "End of time step leading to time " << ChTime << std::endl;
    }

    void FrOffshoreSystem::Initialize() {
        // TODO: Ici, on initialise tous les composants de systeme. Ceci implique d'iterer sur ces derniers et qu'ils
        // possedent tous une methode Initialize()
        // On pourra faire deriver tous les objets d'une class FrObject qui apporte a la fois un UUID et a la
        // methode initialize comme methode virtuelle

        std::cout << "Initializing the system to make every component consistent" << std::endl;
        m_environment->Initialize();

        // Initializing physical items
        for (int ibody=0; ibody<bodylist.size(); ibody++) {
            auto body = dynamic_cast<FrBody*>(bodylist[ibody].get());
            if (body) {
                body->Initialize();
            }
        }

        // We can now do an initial Update as everything has been initialized against everything
        Update(true);
    }

    void FrOffshoreSystem::StepFinalize() {

        std::cout << "Finalizing the step at time " << GetChTime() << std::endl;
        m_environment->StepFinalize();

        for (int ibody=0; ibody<bodylist.size(); ibody++) {
            auto body = dynamic_cast<FrBody*>(bodylist[ibody].get());
            if (body) {
                body->StepFinalize();
            }
        }

    }



}  // end namespace frydom
