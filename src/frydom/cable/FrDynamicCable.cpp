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

        FrDynamicCableBase::FrDynamicCableBase(FrDynamicCable *cable) : chrono::fea::ChMesh(), m_frydomCable(cable) {

          m_startingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
          m_endingHinge = std::make_shared<chrono::ChLinkMateGeneric>();
          m_section = std::make_shared<chrono::fea::ChBeamSectionAdvanced>();

        }

        void FrDynamicCableBase::InitializeSection() {

          m_section->SetAsCircularSection(m_frydomCable->GetCableProperties()->GetDiameter());
          m_section->SetBeamRaleyghDamping(m_frydomCable->GetRayleighDamping());
          m_section->SetDensity(m_frydomCable->GetCableProperties()->GetDensity());
          m_section->SetYoungModulus(m_frydomCable->GetCableProperties()->GetYoungModulus());
        }


        void FrDynamicCableBase::InitializeLinks() {

          // Starting hinge
          auto starting_body = m_frydomCable->GetStartingNode()->GetBody()->m_chronoBody;
          auto ChronoFrame = internal::FrFrame2ChFrame(m_frydomCable->GetStartingNode()->GetFrameInBody());

          m_startingHinge->Initialize(m_starting_node_fea, starting_body, true, chrono::ChFrame<double>(), ChronoFrame);

          // Ending hinge
          auto ending_body = m_frydomCable->GetEndingNode()->GetBody()->m_chronoBody;

          ChronoFrame = internal::FrFrame2ChFrame(m_frydomCable->GetEndingNode()->GetFrameInBody());
          FrFrame feaFrame;
          feaFrame.RotZ_RADIANS(MU_PI, NWU, false); // ending_node_fea comes from the opposite direction

          m_endingHinge->Initialize(m_ending_node_fea, ending_body, true, internal::FrFrame2ChFrame(feaFrame),
                                    ChronoFrame);

          // Define the constraints on the hinges.
          HingesConstraints();

        }

        void FrDynamicCableBase::HingesConstraints() {

          switch (m_frydomCable->GetStartingHingeType()) {
            case FrDynamicCable::NONE:
              m_startingHinge->SetConstrainedCoords(false, false, false, false, false, false);
              break;
            case FrDynamicCable::SPHERICAL:
              m_startingHinge->SetConstrainedCoords(true, true, true, false, false, false);
              break;
            case FrDynamicCable::CONSTRAINED:
              m_startingHinge->SetConstrainedCoords(true, true, true, true, true, true);
              break;
          }

          switch (m_frydomCable->GetEndingHingeType()) {
            case FrDynamicCable::NONE:
              m_endingHinge->SetConstrainedCoords(false, false, false, false, false, false);
              break;
            case FrDynamicCable::SPHERICAL:
              m_endingHinge->SetConstrainedCoords(true, true, true, false, false, false);
              break;
            case FrDynamicCable::CONSTRAINED:
              m_endingHinge->SetConstrainedCoords(true, true, true, true, true, true);
              break;
          }

        }

        void FrDynamicCableBase::GenerateAssets() {
          // Assets for the cable visualisation
          if (m_drawCableElements) {
            auto elements_assets = std::make_shared<chrono::fea::ChVisualizationFEAmesh>(*this);
            elements_assets->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_TX);
            elements_assets->SetColorscaleMinMax(-m_frydomCable->GetBreakingTension(),
                                                 m_frydomCable->GetBreakingTension()); //1799620
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
            node_assets->SetFEMglyphType(
                chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS); // E_GLYPH_NODE_CSYS
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
            bool elastic = distanceBetweenNodes.norm() >= m_frydomCable->GetUnstrainedLength();

            // First, creating a catenary line to initialize finite element mesh node positions
            std::shared_ptr<FrCatenaryLine> catenaryLine;

            if (!elastic) { // distance between the nodes is smaller thant the unstrained length of the line
              // Initializing the finite element model so that it fits the catenary line to get close from the
              // equilibrium solution
              catenaryLine = make_catenary_line("", m_frydomCable->GetStartingNode(), m_frydomCable->GetEndingNode(),
                                                m_frydomCable->GetSystem(), m_frydomCable->GetCableProperties(),
                                                elastic,
                                                m_frydomCable->GetUnstrainedLength(), AIR);
              catenaryLine->Initialize();
            }

            double s = 0.;
            double ds = m_frydomCable->GetUnstrainedLength() / m_frydomCable->GetNumberOfElements();

            // Compute the normal to the plan containing the cable
            auto AB = internal::Vector3dToChVector(m_frydomCable->GetEndingNode()->GetPositionInWorld(NWU) -
                                                   m_frydomCable->GetStartingNode()->GetPositionInWorld(NWU));
            AB.Normalize();

            // Init with the starting node
            auto ChronoFrame = internal::FrFrame2ChFrame(m_frydomCable->GetStartingNode()->GetFrameInWorld());

            chrono::ChVector<double> e1, e2, e3;
            chrono::ChMatrix33<double> RotMat;

            if (!elastic) {
              e1 = internal::Vector3dToChVector(catenaryLine->GetTension(0., NWU));
              e1.Normalize();
              e3 = e1.Cross(AB);
              e3.Normalize();
              e2 = e3.Cross(e1);
              e2.Normalize();

              RotMat.Set_A_axis(e1, e2, e3);

              ChronoFrame.SetPos(
                  internal::Vector3dToChVector(catenaryLine->GetStartingNode()->GetPositionInWorld(NWU)));
              ChronoFrame.SetRot(RotMat);
            }

            auto nodeA = std::make_shared<chrono::fea::ChNodeFEAxyzrot>(ChronoFrame);
            m_starting_node_fea = nodeA;

            // Add the node to the ChMesh
            AddNode(m_starting_node_fea);

            // Creating the specified number of Cable elements
            for (uint i = 1; i <= m_frydomCable->GetNumberOfElements(); ++i) {
              s += ds;

              // Get the position and direction of the line for the curvilinear coord s
              if (elastic) {
                ChronoFrame.SetPos(ChronoFrame.GetPos() + ds * AB);
              } else {
                ChronoFrame.SetPos(internal::Vector3dToChVector(catenaryLine->GetNodePositionInWorld(s, NWU)));
                e1 = internal::Vector3dToChVector(catenaryLine->GetTension(s, NWU));
                e1.Normalize();
                e2 = e3.Cross(e1);
                e2.Normalize();
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

              if (elastic) {
                double tensionMax = (distanceBetweenNodes.norm() - m_frydomCable->GetUnstrainedLength()) *
                                    m_frydomCable->GetCableProperties()->GetEA() / m_frydomCable->GetUnstrainedLength();
                m_frydomCable->SetBreakingTension(1.2 * tensionMax);
              } else {
                m_frydomCable->SetBreakingTension(1.2 * catenaryLine->GetMaxTension());
              }

            }

            // Generate assets for the cable
            GenerateAssets();

            if (!elastic) {
              // Remove the catenary line used for initialization
              m_frydomCable->GetSystem()->RemovePhysicsItem(catenaryLine);
              m_frydomCable->GetStartingNode()->GetBody()->RemoveExternalForce(catenaryLine->GetStartingForce());
              m_frydomCable->GetEndingNode()->GetBody()->RemoveExternalForce(catenaryLine->GetEndingForce());
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

          chrono::ChVector<double> Pos;
          chrono::ChQuaternion<double> Rot;

          dynamic_cast<chrono::fea::ChElementBeamEuler *>(GetElement(index).get())->EvaluateSectionFrame(eta, Pos, Rot);

          return internal::ChVectorToVector3d<Position>(Pos);
        }

        Force FrDynamicCableBase::GetTension(int index, double eta) {

          chrono::ChVector<double> Tension, Torque;

          auto element = dynamic_cast<chrono::fea::ChElementBeamEuler *>(GetElement(index).get());

          element->EvaluateSectionForceTorque(eta, Tension, Torque);

          auto dir = element->GetNodeB()->GetPos() - element->GetNodeA()->GetPos();
          dir.Normalize();

          // only the traction is kept (no bending, etc.)
          return internal::ChVectorToVector3d<Force>(dir * Tension.x());

        }


    }


    FrDynamicCable::FrDynamicCable(const std::string &&name,
                                   const std::shared_ptr<frydom::FrNode> &startingNode,
                                   const std::shared_ptr<frydom::FrNode> &endingNode,
                                   const std::shared_ptr<FrCableProperties> &properties,
                                   double unstrainedLength, double rayleighDamping, unsigned int nbElements) :
        FrCable(std::move(name), startingNode, endingNode, properties, unstrainedLength),
        m_rayleighDamping(rayleighDamping), m_nbElements(nbElements) {

      m_chronoCable = std::make_shared<internal::FrDynamicCableBase>(this);
//        SetLogged(true);

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
      assert(elementLength > 0. && elementLength < GetUnstrainedLength());
      m_nbElements = static_cast<unsigned int>(int(floor(GetUnstrainedLength() / elementLength)));
    }

    void FrDynamicCable::SetBreakingTension(double tension) {
      m_maxTension = tension;
    }

    double FrDynamicCable::GetBreakingTension() const {
      return m_maxTension;
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

      GetTension(0., NWU);
    }

    Force FrDynamicCable::GetTension(double s, FRAME_CONVENTION fc) const {

      assert(s <= GetUnstrainedLength());

      if (s > GetUnstrainedLength()) s = GetUnstrainedLength();

      double ds = GetUnstrainedLength() / GetNumberOfElements();
      double a = s / ds;
      auto index = int(floor(a));
      double eta = 2. * (a - index) - 1.;

      if (s == GetUnstrainedLength()) {
        index = GetNumberOfElements() - 1;
        eta = 1;
      }
//        Force Tension;
      auto Tension = m_chronoCable->GetTension(index, eta);

      if (IsNED(fc)) internal::SwapFrameConvention(Tension);

      return Tension;

    }

    Position FrDynamicCable::GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const {

      assert(s <= GetUnstrainedLength());

      double ds = GetUnstrainedLength() / GetNumberOfElements();
      double a = s / ds;
      auto index = int(floor(a));
      double eta = 2. * (a - index) - 1.;

      if (s == GetUnstrainedLength()) {
        index = GetNumberOfElements() - 1;
        eta = 1;
      }

      auto Pos = m_chronoCable->GetNodePositionInWorld(index, eta);

      if (IsNED(fc)) internal::SwapFrameConvention(Pos);

      return Pos;

    }

    void FrDynamicCable::AddFields() {
//        if (IsLogged()) {
//
//            // Add the fields to be logged here
//            m_message->AddField<double>("time", "s", "Current time of the simulation",
//                                        [this]() { return m_system->GetTime(); });
//
//            m_message->AddField<double>("StrainedLength", "m", "Strained length of the catenary line",
//                                        [this]() { return GetStrainedLength(); });
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("StartingNodeTension","N", fmt::format("Starting node tension in world reference frame in {}",GetLogFrameConvention()),
//                     [this]() {return GetTension(0.,GetLogFrameConvention());});
//
//            m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                    ("EndingNodeTension","N", fmt::format("Ending node tension in world reference frame in {}",GetLogFrameConvention()),
//                     [this]() { Eigen::Matrix<double, 3, 1> temp = -GetTension(GetUnstrainedLength(), GetLogFrameConvention());
//                        return temp;});
//
//            //TODO : logger la position de la ligne pour un ensemble d'abscisses curvilignes?
//
//        }
    }

    void FrDynamicCable::SetStartingHingeType(FrDynamicCable::HingeType type) {
      m_startingHingeType = type;
    }

    FrDynamicCable::HingeType FrDynamicCable::GetStartingHingeType() const {
      return m_startingHingeType;
    }

    void FrDynamicCable::SetEndingHingeType(FrDynamicCable::HingeType type) {
      m_endingHingeType = type;
    }

    FrDynamicCable::HingeType FrDynamicCable::GetEndingHingeType() const {
      return m_endingHingeType;
    }

    double FrDynamicCable::GetStaticResidual() {

      double residual = 0;

      for (auto &node : m_chronoCable->GetNodes()) {
        residual += dynamic_cast<chrono::fea::ChNodeFEAxyzrot *>(node.get())->GetPos_dt().Length();
      }

      return residual;
    }

    void FrDynamicCable::Relax() {

      m_chronoCable->SetNoSpeedNoAcceleration();

    }

    std::shared_ptr<FrDynamicCable>
    make_dynamic_cable(const std::string &&name,
                       const std::shared_ptr<FrNode> &startingNode,
                       const std::shared_ptr<FrNode> &endingNode,
                       FrOffshoreSystem *system,
                       const std::shared_ptr<FrCableProperties> &properties,
                       double unstrainedLength,
                       double rayleighDamping,
                       unsigned int nbElements) {

      auto Cable = std::make_shared<FrDynamicCable>(std::move(name),
                                                    startingNode, endingNode, properties, unstrainedLength,
                                                    rayleighDamping, nbElements);

      system->Add(Cable);
      return Cable;

    }

} // end namespace frydom
