//
// Created by lletourn on 05/03/19.
//

#include <chrono/fea/ChBeamSection.h>
#include <chrono/fea/ChLinkPointFrame.h>
#include <chrono/fea/ChNodeFEAxyzD.h>
#include <chrono/fea/ChElementCableANCF.h>
#include <chrono/fea/ChVisualizationFEAmesh.h>
#include "FrANCFCable.h"

#include "frydom/cable/FrCatenaryLine.h"
#include "frydom/cable/FrCatenaryForce.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {

    namespace internal {

        void FrANCFCableBase::InitializeSection() {  // TODO: mettre en private
            m_section = std::make_shared<chrono::fea::ChBeamSectionCable>();  // Voir si on utilise pas directement la classe mere pour avoir de la torsion...
            m_section->SetArea(m_frydomCable->GetSectionArea());
            m_section->SetDiameter(m_frydomCable->GetDiameter());
            m_section->SetBeamRaleyghDamping(m_frydomCable->GetRayleighDamping());
            m_section->SetDensity(m_frydomCable->GetDensity());
            m_section->SetYoungModulus(m_frydomCable->GetYoungModulus());
        }



//        void FrANCFCableBase::InitializeLinks() {  // TODO: mettre en private
//            auto system = m_frydomCable->GetSystem();  // FIXME: il faut que le corps ou le noeud soit deja enregistre...
//
//            m_startingHinge = std::make_shared<chrono::fea::ChLinkPointFrame>();
//            auto starting_body = m_frydomCable->GetStartingNode()->GetBody()->m_chronoBody;
//
//            m_startingHinge->Initialize(m_starting_node_fea, starting_body);
//
//            m_endingHinge = std::make_shared<chrono::fea::ChLinkPointFrame>();
//            auto ending_body = m_frydomCable->GetEndingNode()->GetBody()->m_chronoBody;
//
//            m_endingHinge->Initialize(m_ending_node_fea, ending_body);
//
//        }

        void FrANCFCableBase::GenerateAssets() {
            // Assets for the cable visualisation
            if (m_drawCableElements) {
                auto elements_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
                elements_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
                elements_assets->SetColorscaleMinMax(-0.4, 0.4);
                elements_assets->SetSmoothFaces(true);
                elements_assets->SetWireframe(false);
                m_section->SetDrawCircularRadius(m_frydomCable->GetDrawNodeSize());
                ChMesh::AddAsset(elements_assets);
            }

            // Assets for the nodes
            if (m_drawCableNodes) {
                auto node_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
                node_assets->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS); // E_GLYPH_NODE_CSYS
                node_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
                node_assets->SetSymbolsThickness(m_frydomCable->GetDrawNodeSize());
                node_assets->SetSymbolsScale(0.01);
                node_assets->SetZbufferHide(false);
                ChMesh::AddAsset(node_assets);
            }

        }

        void FrANCFCableBase::SetStartingNode(Position position, Direction direction) {
            auto ChPos = internal::Vector3dToChVector(position);
            auto ChDir = internal::Vector3dToChVector(direction);
            m_starting_node_fea = std::make_shared<chrono::fea::ChNodeFEAxyzD>(ChPos, ChDir);
        }

        void FrANCFCableBase::SetEndingNode(Position position, Direction direction) {
            auto ChPos = internal::Vector3dToChVector(position);
            auto ChDir = internal::Vector3dToChVector(direction);
            m_ending_node_fea = std::make_shared<chrono::fea::ChNodeFEAxyzD>(ChPos, ChDir);
        }

        void FrANCFCableBase::Initialize() {

            InitializeSection();

            // First, creating a catenary line to initialize finite element mesh node positions
            // TODO: avoir un constructeur pour juste specifier les parametres du cable, pas les frontieres  -->  degager le constructeur par defaut
            Position distanceBetweenNodes = (m_frydomCable->GetEndingNode()->GetPositionInWorld(NWU) - m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU));
            // TODO : check if the distance is greater than the length !!
            bool elastic = distanceBetweenNodes.norm() < m_frydomCable->GetUnstretchedLength();

            auto catenaryLine = make_catenary_line(m_frydomCable->GetStartingNode(),
                                                   m_frydomCable->GetEndingNode(),
                                                   m_frydomCable->GetSystem(),
                                                   elastic,
                                                   m_frydomCable->GetYoungModulus(),
                                                   m_frydomCable->GetSectionArea(),
                                                   m_frydomCable->GetUnstretchedLength(),
                                                   m_frydomCable->GetLinearDensity(),
                                                   WATER);

            catenaryLine->Initialize();


            // Initializing the finite element model so that it fits the catenary line to get close from the
            // equilibrium solution
            double s = 0.;
            double ds = m_frydomCable->GetUnstretchedLength() / m_frydomCable->GetNumberOfElements();

            auto direction = catenaryLine->GetTension(s, NWU);
            direction.Normalize();
            auto position = m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU);

            auto ChPos = internal::Vector3dToChVector(position);
            auto ChDir = internal::Vector3dToChVector(direction);
            auto nodeA = std::make_shared<chrono::fea::ChNodeFEAxyzD>(ChPos, ChDir);
            m_starting_node_fea = nodeA;

            // Add the node to the ChMesh
            AddNode(m_starting_node_fea);

            // Creating the specified number of ANCF Cable elements
            for (uint i = 1; i<= m_frydomCable->GetNumberOfElements(); ++i) {
                s += ds;

                // Get the position and direction of the line for the curvilinear coord s
                position = catenaryLine->GetAbsPosition(s, NWU);
                ChPos = internal::Vector3dToChVector(position);
                direction = catenaryLine->GetTension(s, NWU);    direction.Normalize();
                ChDir = internal::Vector3dToChVector(direction);

                // Create a node and add it to the ChMesh
                auto nodeB = std::make_shared<chrono::fea::ChNodeFEAxyzD>(ChPos, ChDir);
                AddNode(nodeB);

                // Create a cable element between the nodes A and B, and add it to the ChMesh
                auto element = std::make_shared<chrono::fea::ChElementCableANCF>();
                element->SetNodes(nodeA, nodeB);
                element->SetSection(m_section);
                AddElement(element);

                //
                nodeA = nodeB;

            }
            // Add the ending node to the ChMesh
            m_ending_node_fea = nodeA; // nodeB is destroyed after the loop
            AddNode(m_ending_node_fea);

            // Removing forces from catenary line that have been automatically created at instanciation
            m_frydomCable->GetStartingNode()->GetBody()->RemoveExternalForce(catenaryLine->GetStartingForce());
            m_frydomCable->GetEndingNode()->GetBody()->RemoveExternalForce(catenaryLine->GetEndingForce());

            // Generate constraints between boundaries and bodies
            // FIXME: suivant qu'on utilise cette methode pour creer les contraintes ou qu'on specifie directement les contraintes directement sur les noeuds, on a une violation des liaisons
//            InitializeLinks();

            // Generate assets for the cable
            GenerateAssets();

        }

        void FrANCFCableBase::Update(double time, bool update_assets) {

            chrono::fea::ChMesh::Update(time, update_assets);
            m_frydomCable->Update(time);

        }


    }




    FrANCFCable::FrANCFCable(const std::shared_ptr<frydom::FrNode> startingNode,
                                     const std::shared_ptr<frydom::FrNode> endingNode, double cableLength,
                                     double youngModulus, double sectionArea, double linearDensity) : FrCable(
            startingNode,
            endingNode,
            cableLength,
            youngModulus,
            sectionArea,
            linearDensity) {

    }


    void FrANCFCable::SetRayleighDamping(const double damping) {
        m_rayleighDamping = damping;
    }

    double FrANCFCable::GetRayleighDamping() const {
        return m_rayleighDamping;
    }

    void FrANCFCable::SetNumberOfElements(const unsigned int nbElements) {
        m_nbElements = nbElements;
    }

    unsigned int FrANCFCable::GetNumberOfElements() const {
        return m_nbElements;
    }

    void FrANCFCable::SetTargetElementLength() {
        // TODO: on donne une longueur cible d'element fini et ca calcule une discretisation spatiale basee sur la
        // longueur du cable ainsi qu'un nombre subsequent d'elements
    }

    void FrANCFCable::SetDrawRadius(const double radius) {
        m_drawCableElementRadius = radius;
    }

    void FrANCFCable::SetDrawNodeSize(const double size) {
        m_drawCableNodeSize = size;
    }

    double FrANCFCable::GetDrawNodeSize() const {
        return m_drawCableElementRadius;
    }



    void FrANCFCable::Initialize() {

        m_chronoCable->Initialize();

    }

//    void FrANCFCable::StepFinalize() {
//
//
//    }

} // end namespace frydom