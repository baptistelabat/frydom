//
// Created by lletourn on 05/03/19.
//

#include <chrono/fea/ChBeamSection.h>
#include <chrono/fea/ChLinkPointFrame.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/fea/ChNodeFEAxyzD.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>
#include <chrono/fea/ChElementCableANCF.h>
#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/fea/ChVisualizationFEAmesh.h>
#include "chrono/fea/ChBuilderBeam.h"
#include "FrANCFCable.h"

#include "frydom/cable/FrCatenaryLine.h"
#include "frydom/cable/FrCatenaryForce.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {

    namespace internal {

        FrANCFCableBase::FrANCFCableBase(FrANCFCable* cable) : chrono::fea::ChMesh(), m_frydomCable(cable) {

//            m_startingHinge = std::make_shared<chrono::fea::ChLinkPointFrame>();
//            m_endingHinge = std::make_shared<chrono::fea::ChLinkPointFrame>();
            m_startingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
            m_endingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
//            m_section = std::make_shared<chrono::fea::ChBeamSectionCable>();  // Voir si on utilise pas directement la classe mere pour avoir de la torsion...
            m_section = std::make_shared<chrono::fea::ChBeamSectionAdvanced>();  // Voir si on utilise pas directement la classe mere pour avoir de la torsion...

        }

        void FrANCFCableBase::InitializeSection() {  // TODO: mettre en private
//            m_section->SetDiameter(m_frydomCable->GetDiameter());
            m_section->SetAsCircularSection(m_frydomCable->GetDiameter());
            m_section->SetBeamRaleyghDamping(m_frydomCable->GetRayleighDamping());
            m_section->SetDensity(m_frydomCable->GetDensity());
            m_section->SetYoungModulus(m_frydomCable->GetYoungModulus());
        }



        void FrANCFCableBase::InitializeLinks() {  // TODO: mettre en private

            auto starting_body = m_frydomCable->GetStartingNode()->GetBody()->m_chronoBody;

            auto Pos = internal::Vector3dToChVector(m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU));

            chrono::ChFrame<double> frame(Pos);

            m_startingHinge->Initialize(m_starting_node_fea, starting_body, frame);

            auto ending_body = m_frydomCable->GetEndingNode()->GetBody()->m_chronoBody;

            Pos = internal::Vector3dToChVector(m_frydomCable->GetEndingNode()->GetPositionInWorld(NWU));

            frame.SetPos(Pos);

//            Pos = m_ending_node_fea->GetPos();

            m_endingHinge->Initialize(m_ending_node_fea, ending_body, frame);

            m_startingHinge->SetConstrainedCoords(true, true, true, false, false, false);
            m_endingHinge->SetConstrainedCoords(true, true, true, false, false, false);

        }

        void FrANCFCableBase::GenerateAssets() {
            // Assets for the cable visualisation
            if (m_drawCableElements) {
                auto elements_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
                elements_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ANCF_BEAM_AX);
//                elements_assets->SetColorscaleMinMax(-0.4, 0.4);
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
//            m_starting_node_fea = std::make_shared<chrono::fea::ChNodeFEAxyzD>(ChPos, ChDir);
            m_starting_node_fea = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(chrono::ChFrame<double>(ChPos, ChDir));
        }

        void FrANCFCableBase::SetEndingNode(Position position, Direction direction) {
            auto ChPos = internal::Vector3dToChVector(position);
            auto ChDir = internal::Vector3dToChVector(direction);
//            m_ending_node_fea = std::make_shared<chrono::fea::ChNodeFEAxyzD>(ChPos, ChDir);
            m_ending_node_fea = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(chrono::ChFrame<double>(ChPos, ChDir));
        }

        void FrANCFCableBase::Initialize() {

            if (m_starting_node_fea == nullptr) {

                InitializeSection();

                // TODO: avoir un constructeur pour juste specifier les parametres du cable, pas les frontieres  -->  degager le constructeur par defaut
                Position distanceBetweenNodes = (m_frydomCable->GetEndingNode()->GetPositionInWorld(NWU) -
                                                 m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU));

                // check if the distance is greater than the length
                bool elastic = distanceBetweenNodes.norm() > m_frydomCable->GetUnstretchedLength();

                // First, creating a catenary line to initialize finite element mesh node positions
                std::shared_ptr<FrCatenaryLine> catenaryLine;

                if (!elastic) { // distance between the nodes is smaller thant the unstretched length of the line
                    // Initializing the finite element model so that it fits the catenary line to get close from the
                    // equilibrium solution
                    catenaryLine = make_catenary_line(m_frydomCable->GetStartingNode(), m_frydomCable->GetEndingNode(),
                                                      m_frydomCable->GetSystem(), elastic,
                                                      m_frydomCable->GetYoungModulus(), m_frydomCable->GetSectionArea(),
                                                      m_frydomCable->GetUnstretchedLength(),
                                                      m_frydomCable->GetLinearDensity(), AIR);
                    catenaryLine->Initialize();
                }

                double s = 0.;
                double ds = m_frydomCable->GetUnstretchedLength() / m_frydomCable->GetNumberOfElements();

                Direction direction = (m_frydomCable->GetEndingNode()->GetPositionInWorld(NWU) -
                                       m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU));
                direction.Normalize();
                auto position = m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU);

                if (!elastic) {
                    position = catenaryLine->GetAbsPosition(s, NWU);
                    direction = catenaryLine->GetTension(s, NWU);
                    direction.Normalize();
                }

                auto ChPos = internal::Vector3dToChVector(position);
                auto ChDir = internal::Vector3dToChVector(direction);
//                auto nodeA = std::make_shared<chrono::fea::ChNodeFEAxyzD>(ChPos, ChDir);
                auto nodeA = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(chrono::ChFrame<double>(ChPos));
                m_starting_node_fea = nodeA;

                // Add the node to the ChMesh
                AddNode(m_starting_node_fea);

                // Creating the specified number of ANCF Cable elements
                for (uint i = 1; i <= m_frydomCable->GetNumberOfElements(); ++i) {
                    s += ds;

                    // Get the position and direction of the line for the curvilinear coord s
                    if (elastic) {
                        position = m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU) + s * direction;
                    } else {
                        position = catenaryLine->GetAbsPosition(s, NWU);
                        direction = catenaryLine->GetTension(s, NWU);
                        direction.Normalize();
                        ChDir = internal::Vector3dToChVector(direction);
                    }
                    ChPos = internal::Vector3dToChVector(position);

                    // Create a node and add it to the ChMesh
//                    auto nodeB = std::make_shared<chrono::fea::ChNodeFEAxyzD>(ChPos, ChDir);
                    auto nodeB = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(chrono::ChFrame<double>(ChPos));
                    AddNode(nodeB);

                    // Create a cable element between the nodes A and B, and add it to the ChMesh
//                    auto element = std::make_shared<chrono::fea::ChElementCableANCF>();
                    auto element = std::make_shared<chrono::fea::ChElementBeamEuler>();
                    element->SetNodes(nodeA, nodeB);
                    element->SetSection(m_section);
                    AddElement(element);

                    //
                    nodeA = nodeB;

                }
                // Add the ending node to the ChMesh
                m_ending_node_fea = nodeA; // nodeB is destroyed after the loop
                AddNode(m_ending_node_fea);

                // Generate constraints between boundaries and bodies
                InitializeLinks();

                // Generate assets for the cable
                GenerateAssets();

                if (!elastic) {
                    // Remove the catenary line used for initialization
                    m_frydomCable->GetSystem()->RemovePhysicsItem(catenaryLine);
                    m_frydomCable->GetStartingNode()->GetBody()->RemoveExternalForce(catenaryLine->GetStartingForce());
                    m_frydomCable->GetStartingNode()->GetBody()->RemoveExternalForce(catenaryLine->GetEndingForce());
                }

            }

            // Absolutely necessary for finite elements !
            SetupInitial();

        }

        void FrANCFCableBase::Update(double time, bool update_assets) {

            chrono::fea::ChMesh::Update(time, update_assets);
            m_frydomCable->Update(time);

        }

        Position FrANCFCableBase::GetAbsPosition(int index, double eta) {

            chrono::ChVector<double> Pos; chrono::ChQuaternion<double> Rot;

//            dynamic_cast<chrono::fea::ChElementCableANCF*>(GetElement(index).get())->EvaluateSectionFrame(eta, Pos, Rot);
            dynamic_cast<chrono::fea::ChElementBeamEuler*>(GetElement(index).get())->EvaluateSectionFrame(eta, Pos, Rot);

            return internal::ChVectorToVector3d<Position>(Pos);
        }

        Force FrANCFCableBase::GetTension(int index, double eta) {

            chrono::ChVector<double> Tension, Torque;

//            auto element = dynamic_cast<chrono::fea::ChElementCableANCF*>(GetElement(index).get());
            auto element = dynamic_cast<chrono::fea::ChElementBeamEuler*>(GetElement(index).get());

            // FIXME : NEED Chrono to complete this method
            element->EvaluateSectionForceTorque(eta, Tension, Torque);

            return {Tension.x(),0.,0.};

//            auto dir = internal::ChVectorToVector3d<Position>(element->GetNodeA()->Get());
//            dir = internal::ChVectorToVector3d<Position>(element->GetNodeB()->GetD());

//            return dir * Tension.x();
        }


    }




    FrANCFCable::FrANCFCable(const std::shared_ptr<frydom::FrNode> startingNode,
                                     const std::shared_ptr<frydom::FrNode> endingNode, double cableLength,
                                     double youngModulus, double sectionArea, double linearDensity,
                                     double rayleighDamping, unsigned int nbElements) : FrCable(
            startingNode,
            endingNode,
            cableLength,
            youngModulus,
            sectionArea,
            linearDensity), m_rayleighDamping(rayleighDamping), m_nbElements(nbElements) {
            m_chronoCable = std::make_shared<internal::FrANCFCableBase>(this);
            SetLogged(true);
    }


    void FrANCFCable::SetRayleighDamping(double damping) {
        m_rayleighDamping = damping;
    }

    double FrANCFCable::GetRayleighDamping() const {
        return m_rayleighDamping;
    }

    void FrANCFCable::SetNumberOfElements(unsigned int nbElements) {
        m_nbElements = nbElements;
    }

    unsigned int FrANCFCable::GetNumberOfElements() const {
        return m_nbElements;
    }

    void FrANCFCable::SetTargetElementLength() {
        // TODO: on donne une longueur cible d'element fini et ca calcule une discretisation spatiale basee sur la
        // longueur du cable ainsi qu'un nombre subsequent d'elements
    }

    void FrANCFCable::SetDrawRadius(double radius) {
        m_drawCableElementRadius = radius;
    }

    void FrANCFCable::SetDrawNodeSize(double size) {
        m_drawCableNodeSize = size;
    }

    double FrANCFCable::GetDrawNodeSize() const {
        return m_drawCableElementRadius;
    }



    void FrANCFCable::Initialize() {

         m_chronoCable->Initialize();

         GetTension(0.,NWU);
    }

    Force FrANCFCable::GetTension(double s, FRAME_CONVENTION fc) const {

        assert(s<=GetUnstretchedLength());

        double ds = GetUnstretchedLength() / GetNumberOfElements();
        double a = s/ds;
        auto index = int(floor(a));
        double eta = 2.*(a - index) - 1.;

        if (s == GetUnstretchedLength()) {
            index = GetNumberOfElements()-1;
            eta = 1;
        }
//        Force Tension;
        // FIXME : NEED Chrono to complete this method
        auto Tension = m_chronoCable->GetTension(index, eta);

        if (IsNED(fc)) internal::SwapFrameConvention(Tension);

        return Tension;



    }

    Position FrANCFCable::GetAbsPosition(double s, FRAME_CONVENTION fc) const {

        assert(s<=GetUnstretchedLength());

        double ds = GetUnstretchedLength() / GetNumberOfElements();
        double a = s/ds;
        auto index = int(floor(a));
        double eta = 2.*(a - index) - 1.;

        if (s == GetUnstretchedLength()) {
            index = GetNumberOfElements()-1;
            eta = 1;
        }

        auto Pos = m_chronoCable->GetAbsPosition(index, eta);

        if (IsNED(fc)) internal::SwapFrameConvention(Pos);

        return Pos;

    }

    double FrANCFCable::GetStretchedLength() const {
        double cl = 0.;
        int n = 1000;

        double ds = GetUnstretchedLength() / (n-1);
        auto pos_prev = GetAbsPosition(0., NWU);

        for (uint i=0; i<n; ++i) {
            auto s = i*ds;
            auto pos = GetAbsPosition(s, NWU);
            cl += (pos - pos_prev).norm();
            pos_prev = pos;
        }
        return cl;
    }

    void FrANCFCable::InitializeLog() {
        if (IsLogged()) {

            // Build the path to the catenary line log
            auto logPath = m_system->GetPathManager()->BuildPath(this, fmt::format("{}_{}.csv",GetTypeName(),GetShortenUUID()));

            // Add the fields to be logged here
            m_message->AddField<double>("time", "s", "Current time of the simulation",
                                        [this]() { return m_system->GetTime(); });

            m_message->AddField<double>("Stretched Length", "m", "Stretched length of the catenary line",
                                        [this]() { return GetStretchedLength(); });

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("Starting Node Tension","N", fmt::format("Starting node tension in world reference frame in {}",c_logFrameConvention),
                     [this]() {return GetTension(0.,c_logFrameConvention);});

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
                    ("Ending Node Tension","N", fmt::format("Ending node tension in world reference frame in {}",c_logFrameConvention),
                     [this]() {return GetTension(GetUnstretchedLength(), c_logFrameConvention);});

            //TODO : logger la position de la ligne pour un ensemble d'abscisses curvilignes?

            // Initialize the message
            FrObject::InitializeLog(logPath);

        }
    }

    void FrANCFCable::StepFinalize() {
        FrFEAMesh::StepFinalize();

        // Serialize and send the log message
        FrObject::SendLog();
    }

//    void FrANCFCable::StepFinalize() {
//
//
//    }

} // end namespace frydom