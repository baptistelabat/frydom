//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

#include <frydom/hydrodynamics/FrHydroDB.h>
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/core/ChMatrixNM.h"
#include "chrono/core/ChMatrix33.h"

#include "FrObject.h"
#include "frydom/environment/waves/FrFreeSurface.h"
#include "frydom/environment/current/FrCurrent.h"
#include "frydom/core/FrBody.h"

#include "frydom/environment/FrEnvironment.h"


// TODO: les objets environnement devront etre mis dans une classe environnement qui encapsule tout l'environnement:
// vent, vagues, courant, fond...


namespace frydom {

    class FrHydroMapper;

    // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
    class FrOffshoreSystem :
            public chrono::ChSystemSMC,
            public std::enable_shared_from_this<FrOffshoreSystem>,
            public FrObject
    {  // TODO: supprimer cette dependance !

    private:

        std::shared_ptr<FrBody> world_body;

        chrono::ChFrame<double> NEDframe;                            ///< Frame that has Z pointing down to have a well defined heading

        std::unique_ptr<FrEnvironment> m_environment;

        int m_nHDB = 0;
        std::vector<std::shared_ptr<FrHydroDB>> m_HDB;
        std::vector<std::shared_ptr<FrHydroMapper>> m_hydroMapper;  // TODO : patch vector hydro map multibody

        int m_NsampleOutput = 1;    ///< number of time sample between two outputs
        int m_NitterOutput = 0;     ///< current iteration between two outputs


    public:
        /// Default constructor
        explicit FrOffshoreSystem(bool use_material_properties = true,
                                  unsigned int max_objects = 16000,
                                  double scene_size = 500);

        /// Default destructor
        ~FrOffshoreSystem() override {}

        /// Copy constructor
        //FrOffshoreSystem(const FrOffshoreSystem& system) {};

        /// Default destructor
        //~FrOffshoreSystem() override {std::cout << "OffshoreSystem deleted" << "\n";};

        inline FrEnvironment* GetEnvironment() const {
            return m_environment.get();
        }

        /// Get NED frame
        chrono::ChFrame<double> GetNEDFrame() const { return NEDframe; }

        /// Get the world body
        chrono::ChBody* GetWorldBodyPtr() const {
            return world_body.get();
        }

        std::shared_ptr<FrBody> GetWorldBody() const {
            return world_body;
        }

        void SetHydroMapper(std::shared_ptr<FrHydroMapper> hydroMapper) {
            m_hydroMapper.push_back(hydroMapper);
        }

        std::shared_ptr<FrHydroMapper> GetHydroMapper(const int id) const {
            return m_hydroMapper[id];
        }

        void SetHydroDB(const std::string filename) {
            m_HDB.push_back( std::make_shared<FrHydroDB>(filename) );
            m_nHDB += 1;
            m_hydroMapper.push_back( m_HDB[m_nHDB-1]->GetMapper() );
        }

        FrHydroDB* GetHydroDB(const int id) const {
            return m_HDB[id].get();
        }

        int GetHydroMapNb() const { return (int) m_hydroMapper.size(); }

        /// Updates all the auxiliary data and children of
        /// bodies, forces, links, given their current state
        /// as well as environment prior to everything.
        void Update(bool update_assets = true) override;

        void StateScatter(const chrono::ChState& x, const chrono::ChStateDelta& v, const double T) override;

        bool Integrate_Y() override;

        void CustomEndOfStep() override;

        void Initialize() override;

        void StepFinalize() override;

        void SetNsampleOutput(const int n) { m_NsampleOutput = n; }

        virtual void IntLoadResidual_Mv(const unsigned int off,
                                        chrono::ChVectorDynamic<>& R,
                                        const chrono::ChVectorDynamic<>& w,
                                        const double c) override;

        virtual void VariablesFbIncrementMq() override;

    };  // class FrOffshoreSystem

} // end namespace frydom

#endif //FRYDOM_FROFFSHORESYSTEM_H
