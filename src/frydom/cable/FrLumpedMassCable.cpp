//
// Created by frongere on 30/01/2020.
//


#include <chrono/assets/ChSphereShape.h>
#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/logging/FrTypeNames.h"
#include "frydom/environment/FrEnvironmentInc.h"

#include "FrCatenaryLine.h"
#include "FrCableShapeInitializer.h"

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
//      return (m_type == START) ? m_right_element->GetTension() : - m_left_element->GetTension(); // FIXME: verifier signes et implementer les GetTension sur les elements
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

    Velocity FrLMBoundaryNode::GetVelocity() const {
      // TODO
    }

    Acceleration FrLMBoundaryNode::GetAcceleration() const {
      // TODO
    }

    FrLMNodeForceBase::FrLMNodeForceBase(frydom::internal::FrLMNode *node) : m_node(node) {}

    FrLMNodeBuoyancyForce::FrLMNodeBuoyancyForce(frydom::internal::FrLMNode *node) : FrLMNodeForceBase(node) {}

    void FrLMNodeBuoyancyForce::UpdateState() {
      force.SetNull();
      force.z() = 0.5 * (m_node->left_element()->GetVolume() + m_node->right_element()->GetVolume()) *
                  m_node->GetFluidDensityAtCurrentPosition();
    }

    FrLMNodeMorisonForce::FrLMNodeMorisonForce(frydom::internal::FrLMNode *node) : FrLMNodeForceBase(node) {}

    void FrLMNodeMorisonForce::UpdateState() {

      double rho_fluid = m_node->GetFluidDensityAtCurrentPosition();

      auto cable_properties = m_node->GetCableProperties();

      double d = cable_properties->GetHydrodynamicDiameter();
      double l =
          0.5 * (m_node->left_element()->GetUnstretchedLength() + m_node->right_element()->GetUnstretchedLength());

      Force morison_force;

      // TODO: verifier les signes...

      Velocity tangential_velocity;
      Velocity transverse_velocity;

      m_node->GetRelativeVelocityOfFluid(tangential_velocity, transverse_velocity);

      // Transverse drag force
      morison_force =
          0.5 * rho_fluid * cable_properties->GetTransverseDragCoefficient() * d * l
          * transverse_velocity.norm() * transverse_velocity;

      // Tangential drag force
      morison_force +=
          0.5 * rho_fluid * cable_properties->GetTangentialDragCoefficient() * d * l
          * tangential_velocity.norm() * tangential_velocity;


      Acceleration tangential_acceleration;
      Acceleration transverse_acceleration;

      m_node->GetRelativeAccelerationOfFluid(tangential_acceleration, transverse_acceleration);

//      // Transverse added mass
//      morison_force -=
//          rho_fluid * cable_properties->GetTransverseAddedMassCoefficient() * cable_properties->GetSectionArea() * l
//          * transverse_acceleration.norm() * transverse_acceleration;
//
//      // Tangential added mass
//      morison_force -=
//          rho_fluid * cable_properties->GetTangentialAddedMassCoefficient() * cable_properties->GetSectionArea() * l
//          * tangential_acceleration.norm() * tangential_acceleration;

      force = internal::Vector3dToChVector(morison_force);
//      force.SetNull(); // FIXME : a retirer

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
//      auto collision_model = m_body->GetCollisionModel();
//      collision_model->ClearModel();
//      collision_model->AddSphere(m_cable->GetCableProperties()->GetRadius(), {0., 0., 0.});
//      collision_model->BuildModel();
//      m_body->SetCollide(true);
//      m_body->SetMaterialSurface(std::make_shared<chrono::ChMaterialSurfaceSMC>()); // FIXME: it will not work when going into NSC !


      auto sphere_shape = std::make_shared<chrono::ChSphereShape>();
//      sphere_shape->GetSphereGeometry().center = internal::Vector3dToChVector(position);
      sphere_shape->GetSphereGeometry().rad = cable->GetCableProperties()->GetRadius() * 10;
      sphere_shape->SetColor(chrono::ChColor(0, 0, 0));

      m_body->AddAsset(sphere_shape);

//      m_body->SetBodyFixed(true);

      // Adding hydro force
      m_body->AddForce(std::make_shared<FrLMNodeBuoyancyForce>(this));
      m_body->AddForce(std::make_shared<FrLMNodeMorisonForce>(this));

    }

    void FrLMNode::ActivateSpeedLimit(bool val) {
      m_body->SetLimitSpeed(val);
    }

    void FrLMNode::SetSpeedLimit(const double &speed_limit) {
      m_body->SetMaxSpeed((float) speed_limit);
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

    Force FrLMNode::GetTotalForce() const {
      return internal::ChVectorToVector3d<Force>(
          m_body->Get_Xforce()); // FIXME: n'inclue a priori pas les tension d'element car pas exprime comme force exterieure sur le noeud !
    }

    bool FrLMNode::IsInWater() const {
      return m_cable->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->IsInWater(GetPosition(), NWU);
    }

    double FrLMNode::GetFluidDensityAtCurrentPosition() const {
      auto environment = m_cable->GetSystem()->GetEnvironment(); // TODO: dans environnement, avoir direct la methode GetFluidDensity(position)...
      auto fluid_type = environment->GetFluidTypeAtPointInWorld(GetPosition(), NWU, true);
      return environment->GetFluidDensity(fluid_type);
    }

    void FrLMNode::UpdateMass() {
      m_body->SetMass(0.5 * (m_left_element->GetMass() + m_right_element->GetMass()));
    }

    Direction FrLMNode::GetTangentDirection() const {
      return (m_right_element->right_node()->GetPosition() - m_left_element->left_node()->GetPosition()).normalized();
    }

    Velocity
    FrLMNode::GetRelativeVelocityOfFluid() const { // TODO: voir si on ne met pas cette methode sur FrNode, on est forcement dans un fluide...
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

        // Wind
        fluid_relative_velocity += m_cable->GetSystem()->GetEnvironment()->GetAtmosphere()->GetWind()->GetFluxVelocityInWorld(
            node_position, NWU);
      }

      // Body velocity induced fluid flux
      fluid_relative_velocity -= GetVelocity();

      return fluid_relative_velocity;
    }

    void FrLMNode::GetRelativeVelocityOfFluid(Velocity &tangential_velocity, Velocity &transverse_velocity) const {
      auto tangent_direction = GetTangentDirection(); // FIXME: faire un cache !!!

      auto fluid_relative_velocity = GetRelativeVelocityOfFluid();

      tangential_velocity = (fluid_relative_velocity.transpose() * tangent_direction) * tangent_direction;
      transverse_velocity = fluid_relative_velocity - tangential_velocity;
    }

    Acceleration FrLMNode::GetRelativeAccelerationOfFluid() const {

      auto node_position = GetPosition();
      Acceleration fluid_relative_acceleration;


      // TODO: mettre en place cet arbitrage directement dans environnement...
      if (IsInWater()) { // WATER
        auto ocean = m_cable->GetSystem()->GetEnvironment()->GetOcean();

        // Current
//        fluid_relative_acceleration += ocean->GetCurrent()->GetFluxAccelerationInWorld(node_position, NWU);

        // Wave orbital velocities
        fluid_relative_acceleration += ocean->GetFreeSurface()->GetWaveField()->GetAcceleration(node_position, NWU);

      } else { // AIR
//        fluid_relative_acceleration +=
//            m_cable->GetSystem()->GetEnvironment()->GetAtmosphere()->GetWind()->GetFluxAccelerationInworld(
//                node_position, NWU);
      }

      fluid_relative_acceleration -= GetAcceleration();

      return fluid_relative_acceleration;

    }

    void FrLMNode::GetRelativeAccelerationOfFluid(Acceleration &tangential_acceleration,
                                                  Acceleration &transverse_acceleration) const {
      auto tangent_direction = GetTangentDirection(); // FIXME: faire un cache !!!

      auto fluid_relative_acceleration = GetRelativeAccelerationOfFluid();

      tangential_acceleration = (fluid_relative_acceleration.transpose() * tangent_direction) * tangent_direction;
      transverse_acceleration = fluid_relative_acceleration - tangential_acceleration;
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

    FrLMElement *FrLMNode::left_element() const {
      return m_left_element.get();
    }

    FrLMElement *FrLMNode::right_element() const {
      return m_right_element.get();
    }

    FrCableProperties *FrLMNode::GetCableProperties() const {
      return m_cable->GetCableProperties().get();
    }

    double FrLMNode::GetMass() const {
      return m_body->GetMass();
    }

    chrono::ChMarker *FrLMNode::GetMarker() const {
      return m_marker.get();
    }

    double FrLMCableTensionForceFunctor::operator()(double time,
                                                    double rest_length,
                                                    double length,
                                                    double vel,
                                                    chrono::ChLinkSpringCB *link) {
      double tension = 0.;

      // Stiffness part
      if (length > rest_length) {
        tension = - m_cable_properties->GetEA() * (length / rest_length - 1.);
      }

      // Damping part
      tension -= m_cable_properties->GetRayleighDamping() * m_cable_properties->GetSectionArea() * vel;

      return tension;
    }

    FrLMCableTensionForceFunctor::FrLMCableTensionForceFunctor(FrCableProperties *properties) :
        chrono::ChLinkSpringCB::ForceFunctor(),
        m_cable_properties(properties) {}

    FrLMElement::FrLMElement(FrLumpedMassCable *cable,
                             const std::shared_ptr<FrLMNodeBase> &left_node,
                             const std::shared_ptr<FrLMNodeBase> &right_node,
                             const double &rest_length) :
        FrTreeNodeBase(""),
        m_cable(cable),
        m_left_node(left_node),
        m_right_node(right_node),
        m_link(std::make_shared<chrono::ChLinkSpringCB>()),
        m_force_functor(std::make_unique<FrLMCableTensionForceFunctor>(cable->GetCableProperties().get())) {

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

    double FrLMElement::GetUnstretchedLength() const {
      return m_link->GetSpringRestLength();
    }

    FrLMNodeBase *FrLMElement::left_node() {
      return m_left_node.get();
    }

    FrLMNodeBase *FrLMElement::right_node() {
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


    auto shape_initializer = FrCableShapeInitializer::Create(this, GetSystem()->GetEnvironment());

    double element_rest_length = m_unstretchedLength / nbElements;

    m_nodes.emplace_back(
        std::make_shared<internal::FrLMBoundaryNode>(m_startingNode, internal::FrLMBoundaryNode::TYPE::START));

    auto system = m_startingNode->GetSystem();
    double s = 0.;
    for (unsigned int i = 0; i < nbElements - 1; i++) {
      s += element_rest_length;

      auto new_node = std::make_shared<internal::FrLMNode>(this, shape_initializer->GetPosition(s, NWU));
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


  }

  void FrLumpedMassCable::ActivateSpeedLimit(bool val) {
    for (auto &node : m_nodes) {
      node->ActivateSpeedLimit(val);
    }
  }

  void FrLumpedMassCable::SetSpeedLimit(const double &speed_limit) {
    for (auto &node : m_nodes) {
      node->SetSpeedLimit(speed_limit);
    }
  }


  Force FrLumpedMassCable::GetTension(double s, FRAME_CONVENTION fc) const {
    assert(0. <= s <= GetUnstretchedLength());

    // Determining the element where the coordinate s lies
    Force tension;
    double stot = 0.;
    for (const auto &element : m_elements) {
      stot += element->GetUnstretchedLength();
      if (s < stot) {
//        tension = element->GetTension(); // FIXME : implementer !!!
        break;
      }
    }
    return tension;
  }

  double FrLumpedMassCable::GetMass() const {
    double mass = 0.;
    for (const auto &node : m_nodes) {
      mass += node->GetMass();
    }
  }

  Position FrLumpedMassCable::GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const {
    // TODO
  }

  void FrLumpedMassCable::DefineLogMessages() {
    auto msg = NewMessage("State", "State messages");

    msg->AddField<double>("time", "s", "Current time of the simulation",
                          [this]() { return GetSystem()->GetTime(); });

    msg->AddField<double>("StrainedLength", "m", "Strained length of the catenary line",
                          [this]() { return GetStrainedLength(); });

//    msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("StartingNodeTension", "N", fmt::format("Starting node tension in world reference frame in {}", GetLogFC()),
//         [this]() { return GetStartingNode()(GetLogFC()); });
//
//    msg->AddField<Eigen::Matrix<double, 3, 1>>
//        ("EndingNodeTension", "N", fmt::format("Ending node tension in world reference frame in {}", GetLogFC()),
//         [this]() { return GetEndingNodeTension(GetLogFC()); });

  }

//  void FrLumpedMassCable::BuildTautCable(unsigned int nbElements) {
//    // TODO
//  }

  void FrLumpedMassCable::UpdateNodesMasses() {
    for (auto &node: m_nodes) {
      node->UpdateMass();
    }
  }


}  // end namespace frydom
