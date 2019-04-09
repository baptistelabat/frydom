//
// Created by lletourn on 05/03/19.
//

#include <chrono/fea/ChBeamSection.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>
#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/fea/ChVisualizationFEAmesh.h>
#include "chrono/fea/ChBuilderBeam.h"

#include "FrDynamicCable.h"

#include "frydom/cable/FrCatenaryLine.h"
#include "frydom/cable/FrCatenaryForce.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"

#include <type_traits>

namespace frydom {

    namespace internal {

        FrDynamicCableBase::FrDynamicCableBase(FrDynamicCable* cable) : chrono::fea::ChMesh(), m_frydomCable(cable) {

            m_startingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
            m_endingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
            m_section = std::make_shared<chrono::fea::ChBeamSectionAdvanced>();

        }

        void FrDynamicCableBase::InitializeSection() {

            m_section->SetAsCircularSection(m_frydomCable->GetDiameter());
            m_section->SetBeamRaleyghDamping(m_frydomCable->GetRayleighDamping());
            m_section->SetDensity(m_frydomCable->GetDensity());
            m_section->SetYoungModulus(m_frydomCable->GetYoungModulus());
        }



        void FrDynamicCableBase::InitializeLinks() {

            // Starting hinge
            auto starting_body = m_frydomCable->GetStartingNode()->GetBody()->m_chronoBody;

            auto ChronoFrame = internal::FrFrame2ChFrame(m_frydomCable->GetStartingNode()->GetFrameInWorld());

            m_startingHinge->Initialize(m_starting_node_fea, starting_body, ChronoFrame);

            // Ending hinge
            auto ending_body = m_frydomCable->GetEndingNode()->GetBody()->m_chronoBody;

            // FYI : Initialize of the hinge with false is supposed to work with ChronoFrame, as for the starting hinge, above.
            // In practice, there's a bug somewhere inside chrono, so we need to get the relative position of the node anyway...
            ChronoFrame = internal::FrFrame2ChFrame(m_frydomCable->GetEndingNode()->GetFrameInWorld());
            chrono::ChFrame<double> frame(ChronoFrame);
            frame.SetPos(m_ending_node_fea->GetPos() - internal::Vector3dToChVector(m_frydomCable->GetEndingNode()->GetPositionInWorld(NWU)));

            m_endingHinge->Initialize(m_ending_node_fea, ending_body, false, frame, ChronoFrame);

        }

        void FrDynamicCableBase::GenerateAssets() {
            // Assets for the cable visualisation
            if (m_drawCableElements) {
                auto elements_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
                elements_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_TX);
                elements_assets->SetColorscaleMinMax(-m_frydomCable->GetBreakingTension(), m_frydomCable->GetBreakingTension()); //1799620
//                elements_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ANCF_BEAM_AX);
//                elements_assets->SetColorscaleMinMax(-0.4, 0.4);
                elements_assets->SetSmoothFaces(true);
                elements_assets->SetWireframe(false);
                m_section->SetDrawCircularRadius(m_frydomCable->GetDrawElementRadius());
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

        void FrDynamicCableBase::Initialize() {

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

                // Compute the normal to the plan containing the cable
                auto AB = internal::Vector3dToChVector(m_frydomCable->GetEndingNode()->GetPositionInWorld(NWU) -
                                  m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU)); AB.Normalize();

                // Init with the starting node
                auto ChronoFrame = internal::FrFrame2ChFrame(m_frydomCable->GetStartingNode()->GetFrameInWorld());

                chrono::ChVector<double> e1, e2, e3;
                chrono::ChMatrix33<double> RotMat;

                if (!elastic) {
                    e1 = internal::Vector3dToChVector(catenaryLine->GetTension(0., NWU)); e1.Normalize();
                    e3 = e1.Cross(AB); e3.Normalize();
                    e2 = e3.Cross(e1); e2.Normalize();

                    RotMat.Set_A_axis(e1, e2, e3);

                    ChronoFrame.SetPos(
                            internal::Vector3dToChVector(catenaryLine->GetStartingNode()->GetPositionInWorld(NWU)));
                    ChronoFrame.SetRot(RotMat);
                }

                auto nodeA = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(ChronoFrame);
                m_starting_node_fea = nodeA;

                // Add the node to the ChMesh
                AddNode(m_starting_node_fea);

                // Creating the specified number of ANCF Cable elements
                for (uint i = 1; i <= m_frydomCable->GetNumberOfElements(); ++i) {
                    s += ds;

                    // Get the position and direction of the line for the curvilinear coord s
                    if (elastic) {
                        ChronoFrame.SetPos(ChronoFrame.GetPos() + ds*AB);
                    } else {
                        ChronoFrame.SetPos(internal::Vector3dToChVector(catenaryLine->GetNodePositionInWorld(s, NWU)));
                        e1 = internal::Vector3dToChVector(catenaryLine->GetTension(s, NWU)); e1.Normalize();
                        e2 = e3.Cross(e1); e2.Normalize();
                        RotMat.Set_A_axis(e1, e2, e3);
                        ChronoFrame.SetRot(RotMat);
                    }

                    // Create a node and add it to the ChMesh
                    auto nodeB = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(ChronoFrame);
                    AddNode(nodeB);

                    // Create a cable element between the nodes A and B, and add it to the ChMesh
                    auto element = std::make_shared<chrono::fea::ChElementBeamEuler>();
                    element->SetNodes(nodeA, nodeB);
                    element->SetSection(m_section);
                    AddElement(element);

                    //
                    nodeA = nodeB;

                }
                // Add the ending node to the ChMesh
                m_ending_node_fea = nodeA; // nodeB is destroyed after the loop
//                AddNode(m_ending_node_fea);

                // Generate constraints between boundaries and bodies
                InitializeLinks();

                if (m_frydomCable->GetBreakingTension() == 0.) {

                    if (elastic){
                        double tensionMax = (distanceBetweenNodes.norm() - m_frydomCable->GetUnstretchedLength()) * m_frydomCable->GetYoungModulus()*m_frydomCable->GetSectionArea()/m_frydomCable->GetUnstretchedLength();
                        m_frydomCable->SetBreakingTension(1.2*tensionMax);
                    } else{
                        m_frydomCable->SetBreakingTension(1.2*catenaryLine->GetBreakingTension());
                    }

                }

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

        void FrDynamicCableBase::Update(double time, bool update_assets) {

            chrono::fea::ChMesh::Update(time, update_assets);
            m_frydomCable->Update(time);

        }

        Position FrDynamicCableBase::GetNodePositionInWorld(int index, double eta) {

            chrono::ChVector<double> Pos; chrono::ChQuaternion<double> Rot;

            dynamic_cast<chrono::fea::ChElementBeamEuler*>(GetElement(index).get())->EvaluateSectionFrame(eta, Pos, Rot);

            return internal::ChVectorToVector3d<Position>(Pos);
        }

        Force FrDynamicCableBase::GetTension(int index, double eta) {

            chrono::ChVector<double> Tension, Torque;

            auto element = dynamic_cast<chrono::fea::ChElementBeamEuler*>(GetElement(index).get());

            element->EvaluateSectionForceTorque(eta, Tension, Torque);

            auto dir = element->GetNodeB()->GetPos() - element->GetNodeA()->GetPos();
            dir.Normalize();

            // only the traction is kept (no bending, etc.)
            return internal::ChVectorToVector3d<Force>(dir * Tension.x());

        }


    }




    FrDynamicCable::FrDynamicCable(const std::shared_ptr<frydom::FrNode> startingNode,
                                     const std::shared_ptr<frydom::FrNode> endingNode, double cableLength,
                                     double youngModulus, double sectionArea, double linearDensity,
                                     double rayleighDamping, unsigned int nbElements) : FrCable(
            startingNode,
            endingNode,
            cableLength,
            youngModulus,
            sectionArea,
            linearDensity), m_rayleighDamping(rayleighDamping), m_nbElements(nbElements) {
            m_chronoCable = std::make_shared<internal::FrDynamicCableBase>(this);
            SetLogged(true);
    }


    void FrDynamicCable::SetRayleighDamping(double damping) {
        m_rayleighDamping = damping;
    }

    double FrDynamicCable::GetRayleighDamping() const {
        return m_rayleighDamping;
    }

    void FrDynamicCable::SetNumberOfElements(unsigned int nbElements) {
        m_nbElements = nbElements;
    }

    unsigned int FrDynamicCable::GetNumberOfElements() const {
        return m_nbElements;
    }

    void FrDynamicCable::SetTargetElementLength(double elementLength) {
        assert(elementLength>0. && elementLength<GetUnstretchedLength());
        m_nbElements = static_cast<unsigned int>(int(floor(GetUnstretchedLength() / elementLength)));
    }

    void FrDynamicCable::SetDrawElementRadius(double radius) {
        m_drawCableElementRadius = radius;
    }

    double FrDynamicCable::GetDrawElementRadius() {
        return m_drawCableElementRadius;
    }

    void FrDynamicCable::SetDrawNodeSize(double size) {
        m_drawCableNodeSize = size;
    }

    double FrDynamicCable::GetDrawNodeSize() const {
        return m_drawCableElementRadius;
    }



    void FrDynamicCable::Initialize() {

         m_chronoCable->Initialize();

         GetTension(0.,NWU);
    }

    Force FrDynamicCable::GetTension(double s, FRAME_CONVENTION fc) const {

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
        auto Tension = m_chronoCable->GetTension(index, eta);

        if (IsNED(fc)) internal::SwapFrameConvention(Tension);

        return Tension;

    }

    Position FrDynamicCable::GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const {

        assert(s<=GetUnstretchedLength());

        double ds = GetUnstretchedLength() / GetNumberOfElements();
        double a = s/ds;
        auto index = int(floor(a));
        double eta = 2.*(a - index) - 1.;

        if (s == GetUnstretchedLength()) {
            index = GetNumberOfElements()-1;
            eta = 1;
        }

        auto Pos = m_chronoCable->GetNodePositionInWorld(index, eta);

        if (IsNED(fc)) internal::SwapFrameConvention(Pos);

        return Pos;

    }

    double FrDynamicCable::GetStretchedLength() const {
        double cl = 0.;
        int n = 1000;

        double ds = GetUnstretchedLength() / (n-1);
        auto pos_prev = GetNodePositionInWorld(0., NWU);

        for (uint i=0; i<n; ++i) {
            auto s = i*ds;
            auto pos = GetNodePositionInWorld(s, NWU);
            cl += (pos - pos_prev).norm();
            pos_prev = pos;
        }
        return cl;
    }

    void FrDynamicCable::InitializeLog() {
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
                     [this]() { Eigen::Matrix<double, 3, 1> temp = -GetTension(GetUnstretchedLength(), c_logFrameConvention);
                        return temp;});

            //TODO : logger la position de la ligne pour un ensemble d'abscisses curvilignes?

            // Initialize the message
            FrObject::InitializeLog(logPath);

        }
    }

    void FrDynamicCable::StepFinalize() {
        FrFEAMesh::StepFinalize();

        // Serialize and send the log message
        FrObject::SendLog();
    }

//    void FrANCFCable::StepFinalize() {
//
//
//    }

} // end namespace frydom