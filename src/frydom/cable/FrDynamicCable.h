//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRDYNAMICCABLE_H
#define FRYDOM_FRDYNAMICCABLE_H

#include <memory>
#include <frydom/core/FrNode.h>
#include <chrono_fea/ChVisualizationFEAmesh.h>
#include <frydom/catenary/FrCatenaryLine.h>

#include "chrono_fea/ChBeamSection.h"
#include "chrono_fea/ChElementCableANCF.h"


// TODO: harmoniser entre le cable catenaire et le cable dynamique !!!
// TODO: faire une classe abstraite de base pour les cables afin d'harmoniser les methodes



using namespace chrono::fea;

namespace frydom {

    class FrDynamicCable {

    private:
        std::shared_ptr<FrNode> m_starting_node;
        std::shared_ptr<FrNode> m_ending_node;

        std::shared_ptr<ChMesh> m_mesh;
        std::shared_ptr<ChBeamSectionCable> m_section;

        std::shared_ptr<ChVisualizationFEAmesh> m_visu_mesh;

        double m_E;  ///< Young Modulus
        double m_A;  ///< section area
        double c_EA; ///< stiffness coefficient
        double m_rayleighDamping; ///< Rayleigh damping

        double m_cableLength;

        std::unique_ptr<FrCatenaryLine> m_catenary_line;

        unsigned int m_nb_elements;



    public:
        FrDynamicCable() = default;

        // TODO: faire constructeur par copie

        void SetDiameter(const double diameter) {}

        void SetYoungModulus(const double E) {}

        void SetRayleighDamping(const double d) {}

        double GetYoungModulus() const {}

        double GetRayleighDamping() const {}

        double GetCrossSectionArea() const {}

        double GetEA() const {}

        void SetLinearDensity(const double lambda) const {}

        double GetLinearDensity() const {}

        void SetCableLength() {}

        double GetCableLength() {}

        void SetNumberOfElements(const unsigned int nb_elements) {}

        unsigned int GetNumberOfElements() const {}

        void SetStartingNode(const std::shared_ptr<FrNode> startingNode) {}

        std::shared_ptr<FrNode> GetStartingNode() const {}

        void SetEndingNode(const std::shared_ptr<FrNode> endingNode) {}

        std::shared_ptr<FrNode> GetEndingNode() const {}

        chrono::ChVector<> GetPosStartingNode() const {}

        chrono::ChVector<> GetPosEndingNode() const {}

        std::shared_ptr<FrForce> GetStartingForce() const {}

        std::shared_ptr<FrForce> GetEndingForce() const {}

        chrono::ChVector<double> GetTension(const double s) const {}

        chrono::ChVector<double> GetStartingNodeTension() const {}

        chrono::ChVector<double> GetEndingNodeTension() const {}




        /// Initialize the cable with given data
        void Initialize() {
            // First, creating a catenary line to initialize finite element mesh node positions
            // TODO: comment on definit q ???
            double q = 600;
            m_catenary_line = std::make_unique<FrCatenaryLine>(m_starting_node,
                                                               m_ending_node,
                                                               true,
                                                               c_EA,
                                                               m_cableLength,
                                                               q,
                                                               chrono::ChVector<double>(0, 0, -1));

            std::vector<std::shared_ptr<ChElementCableANCF> > elements;
            std::vector<std::shared_ptr<ChNodeFEAxyzD> > nodes;

            // Now, creating the nodes
            double s = 0.;
            double ds = m_cableLength / m_nb_elements;

            auto direction = m_catenary_line->GetTension(s).Normalize();
            auto position = m_starting_node->GetAbsPos();
            auto nodeA = std::make_shared<ChNodeFEAxyzD>(position, direction);
            m_mesh->AddNode(nodeA);
            nodes.push_back(nodeA);

            for (uint i = 1; i<= m_nb_elements; ++i) {
                s += ds;

                direction = m_catenary_line->GetTension(s).Normalize();
                position = m_catenary_line->GetPos(s);

                auto nodeB = std::make_shared<ChNodeFEAxyzD>(position, direction);
                m_mesh->AddNode(nodeB);
                nodes.push_back(nodeB);

                auto element = std::make_shared<ChElementCableANCF>();
                m_mesh->AddElement(element);
                elements.push_back(element);

                element->SetNodes(nodes[i-1], nodes[i]);
                element->SetSection(m_section);

            }

        }

        void Update(double time) {

        }

        void UpdateTime(double time) {

        }

        void UpdateState() {

        }

        





    };

}

#endif //FRYDOM_FRDYNAMICCABLE_H
