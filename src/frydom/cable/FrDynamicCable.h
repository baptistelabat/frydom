//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRDYNAMICCABLE_H
#define FRYDOM_FRDYNAMICCABLE_H

#include <memory>
#include "chrono/physics/ChSystem.h"
#include <frydom/core/FrNode.h>
#include <chrono_fea/ChVisualizationFEAmesh.h>
#include <frydom/cable/FrCatenaryLine.h>

#include "chrono_fea/ChBeamSection.h"
#include "chrono_fea/ChElementCableANCF.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"
#include "FrCable.h"




// TODO: changer FrDynamicCable en FrANCFCable... --> pouvoir proposer d'autres modeles dynamiques de cable

// TODO: harmoniser entre le cable catenaire et le cable dynamique !!!
// TODO: faire une classe abstraite de base pour les cables afin d'harmoniser les methodes

// TODO: mettre en place un (de)raffinement automatique...

using namespace chrono::fea;

namespace frydom {

    // Forward declaration
    class FrBody;


    class FrDynamicCable : public ChMesh, public FrCable {

    private:

        std::shared_ptr<ChNodeFEAxyzD> m_starting_node_fea;
        std::shared_ptr<ChNodeFEAxyzD> m_ending_node_fea;

        std::shared_ptr<ChBeamSectionCable> m_section;

        double m_rayleighDamping;   ///< Rayleigh damping
        unsigned int m_nbElements;  ///< Number of elements in the finite element cable model

        bool m_drawCableElements = true;
        bool m_drawCableNodes = true;
        double m_drawCableElementRadius = 0.05;
        double m_drawCableNodeSize = 0.1;

    public:
        FrDynamicCable();
        // TODO: faire un constructeur prenant uniquement les parametres du cable (pas les frontieres)
        // TODO: utiliser l'attribut m_initialized pour eviter de devoir initialiser manuellement (auto lors de l(utilisation
        // du cable si m_initialized est false)

        // TODO: faire constructeur par copie

        void SetRayleighDamping(const double damping);

        double GetRayleighDamping() const;

        void SetNumberOfElements(const unsigned int nbElements);

        unsigned int GetNumberOfElements() const;

        void SetTargetElementLength();

        std::shared_ptr<FrForce> GetStartingForce() const;  // TODO

        std::shared_ptr<FrForce> GetEndingForce() const;  // TODO

        chrono::ChVector<double> GetTension(const double s) const;  // TODO

        chrono::ChVector<double> GetAbsPosition(const double s) const;  // TODO

        chrono::ChVector<double> GetAbsPosition(const int iNode) {
            auto Node = dynamic_cast<ChNodeFEAxyz*>(GetNode(iNode).get());
            return Node->GetPos();
        }

        chrono::ChVector<double> GetStartingNodeTension() const;  // TODO

        chrono::ChVector<double> GetEndingNodeTension() const;  // TODO

        double GetCableLength() {
            double Length = 0;
            for (int i=0; i<GetNnodes()-1;i++) {
                Length += (GetAbsPosition(i+1)-GetAbsPosition(i)).Length();
            }
        }

        std::shared_ptr<ChNodeFEAxyzD> GetStartingNodeFEA() const;
        std::shared_ptr<ChNodeFEAxyzD> GetEndingNodeFEA() const;

        void SetDrawRadius(const double radius);

        void SetDrawNodeSize(const double size);

        void GenerateAssets();

        void InitializeSection();

        void InitializeLinks();

        /// Initialize the cable with given data
        void Initialize();

        virtual void StepFinalize() override;


        /// Update internal time and time step for dynamic behaviour of the cable
        virtual void UpdateTime(const double time);;

        void Update(double time, bool update_assets = true) override;

    };

}

#endif //FRYDOM_FRDYNAMICCABLE_H
