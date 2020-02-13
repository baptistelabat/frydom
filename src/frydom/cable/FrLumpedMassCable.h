//
// Created by frongere on 30/01/2020.
//

#ifndef FRYDOM_FRLUMPEDMASSCABLE_H
#define FRYDOM_FRLUMPEDMASSCABLE_H

#include <memory>
#include <vector>
#include <deque>

#include "FrCable.h"
#include "frydom/core/FrOffshoreSystem.h"

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkSpringCB.h>


// TODO: avoir une classe element et une classe lumped node ??


namespace frydom {

  class FrLumpedMassCable;

  namespace internal {

    class FrLMElement;

    class FrLMNodeBase {
     public:
      virtual double GetTension() const {}

      virtual Direction GetTensionDirection() const {}

      virtual Force GetTensionVector() const {}

      virtual void UpdateMass() {}

      virtual void SetElements(std::shared_ptr<FrLMElement> left_element, std::shared_ptr<FrLMElement> right_element) {
        m_left_element = left_element;
        m_right_element = right_element;
      }

      virtual chrono::ChMarker *GetMarker() const {}

     protected:
      std::shared_ptr<FrLMElement> m_left_element;
      std::shared_ptr<FrLMElement> m_right_element;
    };

    class FrLMBoundaryNode : public FrLMNodeBase {
     public:

      enum TYPE {
        START,
        END
      };

      explicit FrLMBoundaryNode(const std::shared_ptr<FrNode> &node, TYPE type);

      double GetTension() const override;

      Direction GetTensionDirection() const override;

      Force GetTensionVector() const override;

      chrono::ChMarker *GetMarker() const override;

//      void SetElements(std::shared_ptr<FrLMElement> left_element, std::shared_ptr<FrLMElement> right_element) override;

     private:
      std::shared_ptr<FrNode> m_frydom_node;
//      std::shared_ptr<FrLMElement> m_element;
      TYPE m_type;

    };

    class FrLMNode : public FrLMNodeBase, public FrTreeNodeBase {

     public:
      explicit FrLMNode(const Position &position);

      double GetMass();

      void UpdateMass() override;

      double GetTension() const override;

      Direction GetTensionDirection() const override;

      Force GetTensionVector() const override;


      chrono::ChMarker *GetMarker() const override;


//      void
//      SetElements(const std::shared_ptr<FrLMElement> &left_element, const std::shared_ptr<FrLMElement> &right_element);

      std::shared_ptr<chrono::ChMarker> GetMarker();

      std::shared_ptr<chrono::ChBody> GetBody();

     private:
      std::shared_ptr<chrono::ChBody> m_body;
      std::shared_ptr<chrono::ChMarker> m_marker;

    };

    class FrLMForceFunctor : public chrono::ChLinkSpringCB::ForceFunctor {
     public:
      double operator()(double time,                  ///< current time
                        double rest_length,           ///< undeformed length
                        double length,                ///< current length
                        double vel,                   ///< current velocity (positive when extending)
                        chrono::ChLinkSpringCB *link  ///< back-pointer to associated link
      ) override;
    };


    class FrLMElement : public FrTreeNodeBase {
     public:
      FrLMElement(FrLumpedMassCable *cable,
                  const std::shared_ptr<FrLMNodeBase> &left_node,
                  const std::shared_ptr<FrLMNodeBase> &right_node,
                  const double &rest_length);

      std::shared_ptr<chrono::ChLinkSpringCB> GetLink();

      double GetMass() const;

     private:
      FrLumpedMassCable *m_cable;
      std::shared_ptr<FrLMNodeBase> m_left_node;
      std::shared_ptr<FrLMNodeBase> m_right_node;
      std::shared_ptr<chrono::ChLinkSpringCB> m_link;

      std::unique_ptr<FrLMForceFunctor> m_force_functor;

    };

  }  // end namespace internal


  class FrLumpedMassCable : public FrLoggable<FrOffshoreSystem>, public FrCable {

   public:

    FrLumpedMassCable(const std::string &name,
                      const std::shared_ptr<FrNode> &startingNode,
                      const std::shared_ptr<FrNode> &endingNode,
                      const std::shared_ptr<FrCableProperties> &properties,
                      double unstretchedLength,
                      unsigned int nbElements);

//    void Initialize();

    Force GetTension(double s, FRAME_CONVENTION fc) const override;

    Position GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const override;

    void DefineLogMessages() override;


   private:
    void BuildSlackCable(unsigned int nbElements);

    void BuildTautCable(unsigned int nbElements);

    void UpdateNodeMasses();


   private:
    using NodeContainer = std::deque<std::shared_ptr<internal::FrLMNodeBase>>;
    using ElementContainer = std::deque<std::shared_ptr<internal::FrLMElement>>;

    NodeContainer m_nodes;
    ElementContainer m_elements;

  };

}  // end namespace frydom



#endif //FRYDOM_FRLUMPEDMASSCABLE_H
