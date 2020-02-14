//
// Created by frongere on 30/01/2020.
//


#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/logging/FrTypeNames.h"
#include "frydom/environment/FrEnvironmentInc.h"

#include "FrCatenaryLine.h"

#include "FrLumpedMassCable.h"


namespace frydom {

  namespace internal {

    FrLMBoundaryNode::FrLMBoundaryNode(const std::shared_ptr<FrNode> &node, TYPE type) :
        m_frydom_node(node),
        m_type(type) {}

//    void FrLMBoundaryNode::SetElement(const std::shared_ptr<FrLMElement> &element) {
//      m_element = element;
//    }
    Position FrLMBoundaryNode::GetPosition() const {
      return m_frydom_node->GetPositionInWorld(NWU);
    }

    double FrLMBoundaryNode::GetTension() const {
      // TODO
    }

    Direction FrLMBoundaryNode::GetTensionDirection() const {
      // TODO
    }

    Force FrLMBoundaryNode::GetTensionVector() const {
      // TODO
    }

    chrono::ChMarker *FrLMBoundaryNode::GetMarker() const {
      return m_frydom_node->m_chronoMarker.get();
    }

    FrLMNodeForceBase::FrLMNodeForceBase(frydom::internal::FrLMNode *node) : m_node(node) {}

    FrLMNodeBuoyancyForce::FrLMNodeBuoyancyForce(frydom::internal::FrLMNode *node) : FrLMNodeForceBase(node) {}

    void FrLMNodeBuoyancyForce::UpdateState() {
      force = 0.5 * (m_node->left_element()->GetVolume() + m_node->right_element()->GetVolume()) *
          m_node->GetFluidDensityAtCurrentPosition();
    }

    FrLMNodeMorisonForce::FrLMNodeMorisonForce(frydom::internal::FrLMNode *node) : FrLMNodeForceBase(node) {}

    void FrLMNodeMorisonForce::UpdateState() {
      // TODO



    }

    FrLMNode::FrLMNode(FrLumpedMassCable *cable, const Position &position) :
        FrTreeNodeBase(""),
        m_cable(cable),
        m_body(std::make_shared<chrono::ChBody>()),
        m_marker(std::make_shared<chrono::ChMarker>()) {

      m_body->AddMarker(m_marker);
      m_body->SetPos(internal::Vector3dToChVector(position));
      m_body->UpdateMarkers(0.);

      // Collision model: for the moment, we only use spheres for nodes but in the future, we should play with capsules
      // ideally placed on elements better than on nodes... Is it possible to place collision shapes on links ???
      auto collision_model = m_body->GetCollisionModel();
      collision_model->ClearModel();
      collision_model->AddSphere(m_cable->GetCableProperties()->GetRadius(), {0., 0., 0.});
      collision_model->BuildModel();
      m_body->SetCollide(true);
      m_body->SetMaterialSurface(std::make_shared<chrono::ChMaterialSurfaceSMC>());

      // Adding hydro force
      m_body->AddForce(std::make_shared<FrLMNodeBuoyancyForce>(this));
      m_body->AddForce(std::make_shared<FrLMNodeMorisonForce>(this));

    }

    Position FrLMNode::GetPosition() const {
      return internal::ChVectorToVector3d<Position>(m_body->GetPos());
    }

    Velocity FrLMNode::GetVelocity() const {
      return internal::ChVectorToVector3d<Velocity>(m_body->GetCoord_dt().pos);
    }

    Acceleration FrLMNode::GetAcceleration() const {
      return internal::ChVectorToVector3d<Acceleration>(m_body->GetCoord_dtdt().pos);
    }

    bool FrLMNode::IsInWater() const {
      return m_cable->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->IsInWater(GetPosition(), NWU);
    }

    double FrLMNode::GetFluidDensityAtCurrentPosition() const {
      auto environment = m_cable->GetSystem()->GetEnvironment();
      auto fluid_type = environment->GetFluidTypeAtPointInWorld(GetPosition(), NWU, true);
      return environment->GetFluidDensity(fluid_type);
    }

    void FrLMNode::UpdateMass() {
      m_body->SetMass(0.5 * (m_left_element->GetMass() + m_right_element->GetMass()));
    }

    Direction FrLMNode::GetTangentDirection() const {
      return (m_right_element->right_node()->GetPosition() - m_left_element->left_node()->GetPosition()).normalized();
    }

    Velocity FrLMNode::GetRelativeVelocityOfFluid() const {
      // TODO: mettre cette valeur en cache !!

      auto node_position = GetPosition();
      Velocity fluid_relative_velocity;


      // TODO: mettre en place cet arbitrage directement dans environnement...
      if (IsInWater()) { // WATER
        auto ocean = m_cable->GetSystem()->GetEnvironment()->GetOcean();

        // Current
        fluid_relative_velocity += ocean->GetCurrent()->GetFluxVelocityInWorld(node_position, NWU);

        // Wave orbital velocities
        fluid_relative_velocity += ocean->GetFreeSurface()->GetWaveField()->GetVelocity(node_position, NWU);
      } else { // AIR
        fluid_relative_velocity += m_cable->GetSystem()->GetEnvironment()->GetAtmosphere()->GetWind()->GetFluxVelocityInWorld(node_position, NWU);
      }

      fluid_relative_velocity -= GetVelocity();

      return fluid_relative_velocity;
    }

    Direction FrLMNode::GetTransverseDirection() const {
      auto velocity = GetRelativeVelocityOfFluid();

      // TODO: terminer

    }

    double FrLMNode::GetTension() const {
      // TODO
    }

    Direction FrLMNode::GetTensionDirection() const {
      // TODO
    }

    Force FrLMNode::GetTensionVector() const {
      // TODO
    }

    std::shared_ptr<chrono::ChMarker> FrLMNode::GetMarker() {
      return m_marker;
    }

    std::shared_ptr<chrono::ChBody> FrLMNode::GetBody() {
      return m_body;
    }

    FrLMElement* FrLMNode::left_element() const {
      return m_left_element.get();
    }

    FrLMElement* FrLMNode::right_element() const {
      return m_right_element.get();
    }

    double FrLMNode::GetMass() {
      return m_body->GetMass();
    }

    chrono::ChMarker *FrLMNode::GetMarker() const {
      return m_marker.get();
    }

    double FrLMForceFunctor::operator()(double time,
                                        double rest_length,
                                        double length,
                                        double vel,
                                        chrono::ChLinkSpringCB *link) {
      // TODO: c'est ici qu'on calcule la tension !!!
    }

    FrLMElement::FrLMElement(FrLumpedMassCable *cable,
                             const std::shared_ptr<FrLMNodeBase> &left_node,
                             const std::shared_ptr<FrLMNodeBase> &right_node,
                             const double &rest_length) :
        FrTreeNodeBase(""),
        m_cable(cable),
        m_left_node(left_node),
        m_right_node(right_node),
        m_link(std::make_shared<chrono::ChLinkSpringCB>()),
        m_force_functor(std::make_unique<FrLMForceFunctor>()) {

      m_link->SetSpringRestLength(rest_length);
      m_link->ReferenceMarkers(left_node->GetMarker(), right_node->GetMarker());
      m_link->RegisterForceFunctor(m_force_functor.get());

    }

    std::shared_ptr<chrono::ChLinkSpringCB> FrLMElement::GetLink() {
      return m_link;
    }

    double FrLMElement::GetMass() const {
      return m_cable->GetCableProperties()->GetLinearDensity() * m_link->GetSpringRestLength();
    }

    double FrLMElement::GetVolume() const {
      return m_cable->GetCableProperties()->GetSectionArea() * m_link->GetSpringRestLength();
    }

    FrLMNodeBase* FrLMElement::left_node() {
      return m_left_node.get();
    }

    FrLMNodeBase* FrLMElement::right_node() {
      return m_right_node.get();
    }

  }  // end namespace frydom::internal

  FrLumpedMassCable::FrLumpedMassCable(const std::string &name,
                                       const std::shared_ptr<FrNode> &startingNode,
                                       const std::shared_ptr<FrNode> &endingNode,
                                       const std::shared_ptr<FrCableProperties> &properties,
                                       double unstretchedLength,
                                       unsigned int nbElements) :
      FrCable(startingNode, endingNode, properties, unstretchedLength),
      FrLoggable<FrOffshoreSystem>(name, TypeToString(this), startingNode->GetSystem()) {

    double node_dist = (startingNode->GetPositionInWorld(NWU) - endingNode->GetPositionInWorld(NWU)).norm();

    if (node_dist <= unstretchedLength) {
      BuildSlackCable(nbElements);
    } else {
      BuildTautCable(nbElements);
    }

  }

  void FrLumpedMassCable::BuildSlackCable(unsigned int nbElements) {

    auto system = m_startingNode->GetSystem();

    // Building the static catenary solution to initialize positions
    // FIXME: voir pour une detection auto du fluide...
    FrCatenaryLine catenaryLine("catline",
                                m_startingNode,
                                m_endingNode,
                                m_properties,
                                false,
                                m_unstrainedLength,
                                WATER);
    catenaryLine.Initialize();

    double element_rest_length = m_unstrainedLength / nbElements;

    m_nodes.emplace_back(
        std::make_shared<internal::FrLMBoundaryNode>(m_startingNode, internal::FrLMBoundaryNode::TYPE::START));

    double s = 0.;
    for (unsigned int i = 0; i < nbElements - 1; i++) {
      s += element_rest_length;

      auto new_node = std::make_shared<internal::FrLMNode>(this, catenaryLine.GetNodePositionInWorld(s, NWU));
      system->Add(new_node);
      m_nodes.push_back(new_node);

    }

    m_nodes.emplace_back(
        std::make_shared<internal::FrLMBoundaryNode>(m_endingNode, internal::FrLMBoundaryNode::TYPE::END));


    // Creating the elements
    for (unsigned int i = 0; i < nbElements; i++) {
      m_elements.emplace_back(
          std::make_shared<internal::FrLMElement>(this, m_nodes[i], m_nodes[i + 1], element_rest_length));
      system->Add(m_elements.back());
    }

    // Telling the nodes their elements
    m_nodes.front()->SetElements(m_elements.front(), m_elements.front());
    m_nodes.back()->SetElements(m_elements.back(), m_elements.back());

    for (unsigned int i = 1; i < nbElements; i++) {
      m_nodes[i]->SetElements(m_elements[i - 1], m_elements[i]);
    }

    // Updating mass of elements
    UpdateNodesMasses();

    // INFO TERMINER

  }


  Force FrLumpedMassCable::GetTension(double s, FRAME_CONVENTION fc) const {
    // TODO
  }

  Position FrLumpedMassCable::GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const {
    // TODO
  }

  void FrLumpedMassCable::DefineLogMessages() {
    // TODO

  }

  void FrLumpedMassCable::BuildTautCable(unsigned int nbElements) {
    // TODO
  }

  void FrLumpedMassCable::UpdateNodesMasses() {
    for (auto &node: m_nodes) {
      node->UpdateMass();
    }
  }


}  // end namespace frydom
