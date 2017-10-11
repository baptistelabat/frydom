//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRDYNAMICCABLE_H
#define FRYDOM_FRDYNAMICCABLE_H

#include <memory>
#include <frydom/core/FrNode.h>
#include <chrono_fea/ChVisualizationFEAmesh.h>
#include <frydom/cable/FrCatenaryLine.h>

#include "chrono_fea/ChBeamSection.h"
#include "chrono_fea/ChElementCableANCF.h"
#include "FrCable.h"


// TODO: harmoniser entre le cable catenaire et le cable dynamique !!!
// TODO: faire une classe abstraite de base pour les cables afin d'harmoniser les methodes

// TODO: mettre en place un (de)raffinement automatique...

using namespace chrono::fea;

namespace frydom {

    class FrDynamicCable : public FrCable {

    private:

        std::shared_ptr<ChNodeFEAxyzD> m_starting_node_fea;
        std::shared_ptr<ChNodeFEAxyzD> m_ending_node_fea;

        std::shared_ptr<ChMesh> m_mesh;
        std::shared_ptr<ChBeamSectionCable> m_section;

        std::shared_ptr<ChVisualizationFEAmesh> m_visu_mesh;

        double m_rayleighDamping; ///< Rayleigh damping
        unsigned int m_nb_elements;



    public:
        FrDynamicCable() = default;

        // TODO: faire constructeur par copie

        void SetRayleighDamping(const double damping) { m_rayleighDamping = damping; }

        double GetRayleighDamping() const { return m_rayleighDamping; }

        void SetNumberOfElements(const unsigned int nb_elements) { m_nb_elements = nb_elements; }

        unsigned int GetNumberOfElements() const { return m_nb_elements; }

        std::shared_ptr<FrForce> GetStartingForce() const {}

        std::shared_ptr<FrForce> GetEndingForce() const {}

        chrono::ChVector<double> GetTension(const double s) const {}

        chrono::ChVector<double> GetAbsPosition(const double s) const {}

        chrono::ChVector<double> GetStartingNodeTension() const {}

        chrono::ChVector<double> GetEndingNodeTension() const {}


        /// Initialize the cable with given data
        void Initialize() {
            m_mesh = std::make_shared<ChMesh>();

            m_section = std::make_shared<ChBeamSectionCable>();
            m_section->SetArea(m_sectionArea);
            m_section->SetBeamRaleyghDamping(m_rayleighDamping);
            m_section->SetDensity(GetDensity());
            m_section->SetYoungModulus(m_youngModulus);
            m_section->SetI(1.); // TODO: ajouter une inertie !!!

            // First, creating a catenary line to initialize finite element mesh node positions
            // TODO: comment on definit q ???
            double q = 600;
            auto catenary_line = FrCatenaryLine(m_starting_node,
                                                m_ending_node,
                                                true,
                                                m_youngModulus * m_sectionArea,
                                                m_cableLength,
                                                q,
                                                chrono::ChVector<double>(0, 0, -1));

//            std::vector<std::shared_ptr<ChElementCableANCF> > elements;
            std::vector<std::shared_ptr<ChNodeFEAxyzD> > nodes;

            // Now, creating the nodes
            double ds = m_cableLength / m_nb_elements;
            double s = 0.;

            // FIXME: la direction n'est pas celle de la tension mais celle entre 2 noeuds qu'on doit recuperer
            // a partir de la position des points de la ligne catenaire
            auto positionA = catenary_line.GetAbsPosition(0.);
            auto positionB = catenary_line.GetAbsPosition(ds);

            auto direction = (positionB - positionA);
            direction.Normalize();

            auto nodeA = std::make_shared<ChNodeFEAxyzD>(positionA, direction);
            m_mesh->AddNode(nodeA);
            nodes.push_back(nodeA);
            m_starting_node_fea = nodeA;


            // FIXME: attention: l'initialisatio suivante n'est pas fonctionnelle !!!
            chrono::ChVector<double> next_positionB;
            for (uint i = 1; i<= m_nb_elements; ++i) {
                s += ds;

                next_positionB = catenary_line.GetAbsPosition(s+ds);

                // Direction of node B
                direction = (next_positionB - positionB).Normalize();

                auto nodeB = std::make_shared<ChNodeFEAxyzD>(positionB, direction);
                m_mesh->AddNode(nodeB);
                nodes.push_back(nodeB);

                positionB = next_positionB;

                nodeA = nodeB;

                auto element = std::make_shared<ChElementCableANCF>();
                m_mesh->AddElement(element);
//                elements.push_back(element);

                element->SetNodes(nodes[i-1], nodes[i]); // FIXME: indice a changer
                element->SetSection(m_section);

            }
            m_ending_node_fea = nodes[m_nb_elements];

        }

        void UpdateTime(double time) {
            m_time = time;
        }

        void UpdateState() {

        }

    };

}

#endif //FRYDOM_FRDYNAMICCABLE_H
