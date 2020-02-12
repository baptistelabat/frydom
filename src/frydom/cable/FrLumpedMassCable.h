//
// Created by frongere on 30/01/2020.
//

#ifndef FRYDOM_FRLUMPEDMASSCABLE_H
#define FRYDOM_FRLUMPEDMASSCABLE_H

#include "FrCable.h"
#include "frydom/core/FrOffshoreSystem.h"

// TODO: voir lequel !
#include "chrono/physics/ChLinkSpring.h"
#include "chrono/physics/ChLinkSpringCB.h"


// TODO: avoir une classe element et une classe lumped node ??


namespace frydom {

  class FrLumpedMassCable;

  namespace internal {

   class LumpedMassElementForce : public chrono::ChLinkSpringCB::ForceFunctor {
    public:
     double operator()(double time, double rest_length, double length, double vel, chrono::ChLinkSpringCB* link) override {
       // Here we compute the force in the link.





     }
   };



    class FrLumpedElement {

     public:
      FrLumpedElement(FrLumpedMassCable* cable,
          const std::shared_ptr<FrNode>& node1,
          const std::shared_ptr<FrNode>& node2,
          const double& unstretchedLength);

      void Initialize();

      Direction GetDirection() const;

      double GetRestLength() const;

      double GetLength() const;

      double GetDeform() const;

      double GetVelocity() const;

      double GetReact() const;

     private:
      FrLumpedMassCable* m_cable;

      std::shared_ptr<FrNode> m_node1;
      std::shared_ptr<FrNode> m_node2;
      std::shared_ptr<chrono::ChLinkSpringCB> m_link;
    };

  }  // end namespace frydom::internal

  // Forward declaration
  class FrNode;

  class FrLumpedMassCable : public FrLoggable<FrOffshoreSystem>, public FrCable {

   public:

    FrLumpedMassCable(const std::string &name,
                      const std::shared_ptr<FrNode> &startingNode,
                      const std::shared_ptr<FrNode> &endingNode,
                      const std::shared_ptr<FrCableProperties> &properties,
                      double unstretchedLength,
                      unsigned int nbElements);

    void Initialize();

    Force GetTension(double s, FRAME_CONVENTION fc) const override;

    Position GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const override;

    void DefineLogMessages() override;


   private:
    unsigned int m_nb_elements;

  };

}  // end namespace frydom



#endif //FRYDOM_FRLUMPEDMASSCABLE_H
