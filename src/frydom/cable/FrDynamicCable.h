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
        FrDynamicCable() = default;
        // TODO: faire un constructeur prenant uniquement les parametres du cable (pas les frontieres)
        // TODO: utiliser l'attribut m_initialized pour eviter de devoir initialiser manuellement (auto lors de l(utilisation
        // du cable si m_initialized est false)

        // TODO: faire constructeur par copie

        void SetRayleighDamping(const double damping) { m_rayleighDamping = damping; }

        double GetRayleighDamping() const { return m_rayleighDamping; }

        void SetNumberOfElements(const unsigned int nbElements) { m_nbElements = nbElements; }

        unsigned int GetNumberOfElements() const {}

        void SetTargetElementLength() {
            // TODO: on donne une longueur cible d'element fini et ca calcule une discretisation spatiale basee sur la
            // longueur du cable ainsi qu'un nombre subsequent d'elements
        }

        std::shared_ptr<FrForce> GetStartingForce() const {}  // TODO

        std::shared_ptr<FrForce> GetEndingForce() const {}  // TODO

        chrono::ChVector<double> GetTension(const double s) const {}  // TODO

        chrono::ChVector<double> GetAbsPosition(const double s) const {}  // TODO

        chrono::ChVector<double> GetAbsPosition(const int iNode) {
            auto Node = dynamic_cast<ChNodeFEAxyz*>(GetNode(iNode).get());
            return Node->GetPos();
        }

        chrono::ChVector<double> GetStartingNodeTension() const {}  // TODO

        chrono::ChVector<double> GetEndingNodeTension() const {}  // TODO

        double GetCableLength() {
            double Length = 0;
            for (int i=0; i<GetNnodes()-1;i++) {
                Length += (GetAbsPosition(i+1)-GetAbsPosition(i)).Length();
            }
        }


        std::shared_ptr<ChNodeFEAxyzD> GetStartingNodeFEA() const { return m_starting_node_fea; }
        std::shared_ptr<ChNodeFEAxyzD> GetEndingNodeFEA() const { return m_ending_node_fea; }

        void SetDrawRadius(const double radius) {
            m_drawCableElementRadius = radius;
        }

        void SetDrawNodeSize(const double size) {
            m_drawCableNodeSize = size;
        }

        void GenerateAssets() {
            // Assets for the cable visualisation
            if (m_drawCableElements) {
                auto elements_assets = std::make_shared<ChVisualizationFEAmesh>(*this);
                elements_assets->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
                elements_assets->SetColorscaleMinMax(-0.4, 0.4);
                elements_assets->SetSmoothFaces(true);
                elements_assets->SetWireframe(false);
                m_section->SetDrawCircularRadius(m_drawCableElementRadius);
                ChMesh::AddAsset(elements_assets);
            }

            // Assets for the nodes
            if (m_drawCableNodes) {
                auto node_assets = std::make_shared<ChVisualizationFEAmesh>(*this);
                node_assets->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS); // E_GLYPH_NODE_CSYS
                node_assets->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
                node_assets->SetSymbolsThickness(m_drawCableNodeSize);
                node_assets->SetSymbolsScale(0.01);
                node_assets->SetZbufferHide(false);
                ChMesh::AddAsset(node_assets);
            }

        }

        void InitializeSection() {  // TODO: mettre en private
            m_section = std::make_shared<ChBeamSectionCable>();  // Voir si on utilise pas directement la classe mere pour avoir de la torsion...
            m_section->SetArea(m_sectionArea);
            m_section->SetDiameter(GetDiameter());
            m_section->SetBeamRaleyghDamping(m_rayleighDamping);
            m_section->SetDensity(GetDensity());
            m_section->SetYoungModulus(m_youngModulus);
        }

        void InitializeLinks() {  // TODO: mettre en private
            auto system = m_startingNode->GetBody()->GetSystem();  // FIXME: il faut que le corps ou le noeud soit deja enregistre...

            // Attach starting FEA node to body
//            chrono::ChCoordsys<double> starting_relative_pos = m_startingNode->GetRelPos();

            auto hinge_starting_node = std::make_shared<chrono::fea::ChLinkPointFrame>();
            auto starting_body = dynamic_cast<FrBody*>(m_startingNode->GetBody())->GetSharedPtr();
//            hinge_starting_node->Initialize(m_starting_node_fea, starting_body, &starting_relative_pos.pos);
            hinge_starting_node->Initialize(m_starting_node_fea, starting_body);
            system->Add(hinge_starting_node);

            // Attach starting FEA node to body
//            chrono::ChCoordsys<double> ending_relative_pos = m_endingNode->GetRelPos();

            auto hinge_ending_node = std::make_shared<chrono::fea::ChLinkPointFrame>();
            auto ending_body = dynamic_cast<FrBody*>(m_endingNode->GetBody())->GetSharedPtr();
//            hinge_ending_node->Initialize(m_ending_node_fea, ending_body, &ending_relative_pos.pos);
            hinge_ending_node->Initialize(m_ending_node_fea, ending_body);
            system->Add(hinge_ending_node);

        }

        /// Initialize the cable with given data
        void Initialize() {

            InitializeSection();


            // First, creating a catenary line to initialize finite element mesh node positions
            // TODO: comment on definit q ???
            double q = GetLinearDensity();

            // TODO: avoir un constructeur pour juste specifier les parametres du cable, pas les frontieres  -->  degager le constructeur par defaut
            auto catenary_line = FrCatenaryLine(m_startingNode,
                                                m_endingNode,
                                                false, // Elasticity is set to false to build the finite element model // FIXME: la simu ne fonctionne pas si on met false (objectif) --> absolument regler ca !!!!
                                                       // at the specified length
                                                m_youngModulus,
                                                m_sectionArea,
                                                m_cableLength,
                                                q,
                                                chrono::ChVector<double>(0, 0, -1));

            // Initializing the finite element model so that it fits the catenary line to get close from the
            // equilibrium solution
            double s = 0.;
            double ds = m_cableLength / m_nbElements;

            auto direction = catenary_line.GetTension(s);
            direction.Normalize();
            auto position = m_startingNode->GetAbsPos();
            auto nodeA = std::make_shared<ChNodeFEAxyzD>(position, direction);
            AddNode(nodeA);
            m_starting_node_fea = nodeA;

            // Creating the specified number of ANCF Cable elements
            for (uint i = 1; i<= m_nbElements; ++i) {
                s += ds;

                direction = catenary_line.GetTension(s);
                direction.Normalize();
                position = catenary_line.GetAbsPosition(s);

                auto nodeB = std::make_shared<ChNodeFEAxyzD>(position, direction);
                AddNode(nodeB);

                auto element = std::make_shared<ChElementCableANCF>();
                element->SetNodes(nodeA, nodeB);
                element->SetSection(m_section);
                AddElement(element);

                nodeA = nodeB;

            }
            m_ending_node_fea = nodeA;

            // Removing forces from catenary line that have been automatically created at instanciation
            m_startingNode->GetBody()->RemoveForce(catenary_line.GetStartingForce());
            m_endingNode->GetBody()->RemoveForce(catenary_line.GetEndingForce());

            // Generate constraints between boundaries and bodies
            // FIXME: suivant qu'on utilise cette methode pour creer les contraintes ou qu'on specifie directement les contraintes directement sur les noeuds, on a une violation des liaisons
            InitializeLinks();

            // Generate assets for the cable
            GenerateAssets();

        }

        virtual void StepFinalize() override {}


        /// Update internal time and time step for dynamic behaviour of the cable
        virtual void UpdateTime(const double time) {
            m_time_step = time - m_time;
            m_time = time;
        };

        void Update(double time, bool update_assets = true) override {
            ChMesh::Update(time, update_assets);
            UpdateTime(time);
        }

    };

}

#endif //FRYDOM_FRDYNAMICCABLE_H
