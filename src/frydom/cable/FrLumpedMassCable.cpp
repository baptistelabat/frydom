//
// Created by frongere on 30/01/2020.
//


#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "FrCatenaryLine.h"

#include "FrLumpedMassCable.h"


namespace frydom {


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
    auto catenary_line = FrCatenaryLine("init_line", m_startingNode, m_endingNode, m_properties, true, m_unstrainedLength, WATER); // FIXME: permettre de changer !!!
    catenary_line.Initialize();

    // We now have nb_elements - 1 bodies to create

    double diameter = m_properties->GetDiameter();

    std::vector<std::shared_ptr<FrBody>> bodies;
    double s = 0.;
    double ds = m_unstrainedLength / m_nb_elements;
    for (uint i=0; i<m_nb_elements-1; i++) {
      s += ds;

      // Creating the body
      auto body = system->NewBody(GetName() + "_body_" + std::to_string(i));
      body->LogThis(true); // TODO: mettre a false

      // Set the initial position based on Catenary position
      auto position = catenary_line.GetNodePositionInWorld(s, NWU);
      body->SetPosition(position, NWU);

//      // The mass is then set too:
//      double mass = 2.;
//      mass = 0.5 * (0.25 * MU_PI * diameter*diameter * length
//      body->SetInertiaTensor(FrInertiaTensor(mass, 0., 0., 0., 0., 0., 0., {0., 0., 0.}, NWU));

      bodies.push_back(body);

    }

    // Now, s
    std::shared_ptr<FrNode> left_node, right_node, my_node;
    left_node = m_startingNode;
    auto body_iter = bodies.begin();
    for (; body_iter != bodies.end(); body_iter++) {

      




    }






  }

}  // end namespace frydom
