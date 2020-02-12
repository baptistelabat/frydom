//
// Created by frongere on 30/01/2020.
//


#include "frydom/core/common/FrNode.h"
//#include "frydom/core/body/FrBody.h"
#include "FrCatenaryLine.h"

#include "FrLumpedMassCable.h"


namespace frydom {

  namespace internal {

    FrLumpedElement::FrLumpedElement(FrLumpedMassCable *cable,
                                     const std::shared_ptr<FrNode> &node1,
                                     const std::shared_ptr<FrNode> &node2,
                                     const double &unstretchedLength)
        : m_cable(cable), m_node1(node1), m_node2() {

      Initialize();
      m_link->SetSpringRestLength(unstretchedLength);

    }

    void FrLumpedElement::Initialize() {
      m_link = std::make_shared<chrono::ChLinkSpringCB>();
//      m_link->ReferenceMarkers(m_node2->m_chronoMarker.get(), m_node1->m_chronoMarker.get());
    }

    Direction FrLumpedElement::GetDirection() const {
      return internal::ChVectorToVector3d<Direction>(
          (m_link->GetEndPoint2Abs() - m_link->GetEndPoint2Abs()).GetNormalized()
          );
    }

    double FrLumpedElement::GetRestLength() const {
      return m_link->GetSpringRestLength();
    }

    double FrLumpedElement::GetLength() const {
      return m_link->GetSpringLength();
    }


  }  // end namespace frydom::internal


  FrLumpedMassCable::FrLumpedMassCable(const std::string &name,
                                       const std::shared_ptr<FrNode> &startingNode,
                                       const std::shared_ptr<FrNode> &endingNode,
                                       const std::shared_ptr<FrCableProperties> &properties, double unstretchedLength,
                                       unsigned int nbElements) :
      FrCable(startingNode, endingNode, properties, unstretchedLength),
      FrLoggable<FrOffshoreSystem>(name, GetTypeName(), startingNode->GetBody()->GetSystem()),
      m_nb_elements(nbElements) {

  }

  Position FrLumpedMassCable::GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const {
    return Position();
  }

  Force FrLumpedMassCable::GetTension(double s, FRAME_CONVENTION fc) const {
    return Force();
  }

  void FrLumpedMassCable::DefineLogMessages() {

  }

  void FrLumpedMassCable::Initialize() {

    auto system = GetSystem();

    // Creating a catenary line to get initialization
    auto catenary_line = FrCatenaryLine("init_line", m_startingNode, m_endingNode, m_properties, true,
                                        m_unstrainedLength, WATER); // FIXME: permettre de changer !!!
    catenary_line.Initialize();

    // We now have nb_elements - 1 bodies to create

    double diameter = m_properties->GetDiameter();

    std::vector<std::shared_ptr<FrNode>> right_nodes;
    right_nodes.reserve(m_nb_elements);

    double s = 0.;
    double ds = m_unstrainedLength / m_nb_elements;
    for (uint i = 0; i < m_nb_elements - 1; i++) {
      s += ds;

      // Creating the body
      std::string body_name = GetName() + "_body_" + std::to_string(i);
      auto body = system->NewBody(body_name);
      body->LogThis(true); // TODO: mettre a false

      // Set the initial position based on Catenary position
      auto position = catenary_line.GetNodePositionInWorld(s, NWU);
      body->SetPosition(position, NWU);

      // Creating the body node
      auto node = body->NewNode(body_name + "_node");

//      // The mass is then set too:
//      double mass = 2.;
//      mass = 0.5 * (0.25 * MU_PI * diameter*diameter * length
//      body->SetInertiaTensor(FrInertiaTensor(mass, 0., 0., 0., 0., 0., 0., {0., 0., 0.}, NWU));

      right_nodes.push_back(node);

    }
    right_nodes.push_back(m_endingNode);


    // Creating elements between nodes
    std::vector<internal::FrLumpedElement> elements;
    elements.reserve(m_nb_elements);

    std::shared_ptr<FrNode> left_node, right_node, my_node;
    left_node = m_startingNode;

    for (unsigned int ielt = 0; ielt < m_nb_elements; ielt++) {
      right_node = right_nodes[ielt];

      internal::FrLumpedElement element(this, left_node, right_node, ds);



      left_node = right_node;
    }


  }

}  // end namespace frydom
