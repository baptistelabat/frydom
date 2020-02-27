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

  // Forward declaration
  class FrLumpedMassCable;

  namespace internal {
    // Forward declaration
    class FrLMElement;

    class FrLMNodeBase {
     public:

      virtual void ActivateSpeedLimit(bool val) {}

      virtual void SetSpeedLimit(const double &speed_limit) {}

      virtual double GetTension() const {} // FIXME: quelle est la signification de la tension sur un noeud ??? Pour un boundary node ok mais sinon ???

      virtual double GetMass() const { return 0.; }

      virtual Position GetPosition() const {}  // FIXME: tout doit avoir un FRAME_CONVENTION !!!

      virtual Velocity GetVelocity() const {}

      virtual Acceleration GetAcceleration() const {}

      virtual Force GetTotalForce() const { return {}; }

      virtual Direction GetTensionDirection() const {}

      virtual Force GetTensionVector() const {}

      virtual void UpdateMass() {}

      virtual double GetFluidDensityAtCurrentPosition() const {
        assert(false);
      }

      virtual void SetElements(std::shared_ptr<FrLMElement> left_element,
                               std::shared_ptr<FrLMElement> right_element) {
        m_left_element = left_element;
        m_right_element = right_element;
      }

      virtual chrono::ChMarker *GetMarker() const {}

      FrLMElement *left_element() { return m_left_element.get(); }

      FrLMElement *right_element() { return m_right_element.get(); }


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

//      double GetFluidDensityAtCurrentPosition() const override;
      Position GetPosition() const override;

      Velocity GetVelocity() const override;

      Acceleration GetAcceleration() const override;

      double GetTension() const override;

      Direction GetTensionDirection() const override;

      Force GetTensionVector() const override;

      chrono::ChMarker *GetMarker() const override;

     private: // TODO: voir comment faire pour que ce type de noeud n'ait qu'un seul type d'element...
      std::shared_ptr<FrNode> m_frydom_node;
      TYPE m_type;

    };

    class FrLMNodeForceBase : public chrono::ChForce {
     public:
      explicit FrLMNodeForceBase(FrLMNode *node);

     protected:
      FrLMNode *m_node;
    };

    class FrLMNodeBuoyancyForce : public FrLMNodeForceBase {
     public:
      explicit FrLMNodeBuoyancyForce(FrLMNode *node);

      void UpdateState() override;
    };

    class FrLMNodeMorisonForce : public FrLMNodeForceBase {
     public:
      explicit FrLMNodeMorisonForce(FrLMNode *node);

      void UpdateState() override;
    };

    class FrLMNode : public FrLMNodeBase, public FrTreeNodeBase {

     public:
      FrLMNode(FrLumpedMassCable *cable, const Position &position);

      void ActivateSpeedLimit(bool val) override;

      void SetSpeedLimit(const double &speed_limit) override;

      double GetMass() const override;

      Position GetPosition() const override;

      Velocity GetVelocity() const override;

      Acceleration GetAcceleration() const override;

      Force GetTotalForce() const override;

      bool IsInWater() const;

      Velocity GetRelativeVelocityOfFluid() const;

      void GetRelativeVelocityOfFluid(Velocity &tangential_velocity, Velocity &transverse_velocity) const;

      Acceleration GetRelativeAccelerationOfFluid() const;

      void GetRelativeAccelerationOfFluid(Acceleration &tangential_acceleration,
                                          Acceleration &transverse_acceleration) const;

      double GetFluidDensityAtCurrentPosition() const override;

      void UpdateMass() override;

      Direction GetTangentDirection() const;

//      Direction GetTransverseDirection() const;

      double GetTension() const override;

      Direction GetTensionDirection() const override;

      Force GetTensionVector() const override;

      chrono::ChMarker *GetMarker() const override;

      std::shared_ptr<chrono::ChMarker> GetMarker();

      std::shared_ptr<chrono::ChBody> GetBody();

      FrLMElement *left_element() const;

      FrLMElement *right_element() const;

//     private:
      FrCableProperties *GetCableProperties() const;

     private:
      FrLumpedMassCable *m_cable;
      std::shared_ptr<chrono::ChBody> m_body;
      std::shared_ptr<chrono::ChMarker> m_marker;

    };

    class FrLMCableTensionForceFunctor : public chrono::ChLinkSpringCB::ForceFunctor {
     public:
      explicit FrLMCableTensionForceFunctor(FrCableProperties *properties);

      double operator()(double time,                  ///< current time
                        double rest_length,           ///< unstretched length
                        double length,                ///< current length
                        double vel,                   ///< current velocity (positive when extending)
                        chrono::ChLinkSpringCB *link  ///< back-pointer to associated link
      ) override;

     private:
      FrCableProperties *m_cable_properties;
    };

    class FrLMElement : public FrTreeNodeBase {
     public:
      FrLMElement(FrLumpedMassCable *cable,
                  const std::shared_ptr<FrLMNodeBase> &left_node,
                  const std::shared_ptr<FrLMNodeBase> &right_node,
                  const double &rest_length);

      std::shared_ptr<chrono::ChLinkSpringCB> GetLink();

      double GetMass() const;

      double GetVolume() const;

      double GetUnstretchedLength() const;

      FrLMNodeBase *left_node();

      FrLMNodeBase *right_node();

     private:
      FrLumpedMassCable *m_cable;
      std::shared_ptr<FrLMNodeBase> m_left_node;
      std::shared_ptr<FrLMNodeBase> m_right_node;
      std::shared_ptr<chrono::ChLinkSpringCB> m_link;

      std::unique_ptr<FrLMCableTensionForceFunctor> m_force_functor;

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

    void ActivateSpeedLimit(bool val);

    void SetSpeedLimit(const double &speed_limit);


    Force GetTension(double s, FRAME_CONVENTION fc) const override;

    double GetMass() const;


    Position GetPositionInWorld(double s, FRAME_CONVENTION fc) const override;

    void DefineLogMessages() override;

    double GetUnstretchedLengthFromElements() const {
      double length = 0.;
      for (const auto &element: m_elements) {
        length += element->GetUnstretchedLength();
      }
      return length;
    }

//   private:
    void UpdateNodesMasses();


   private:
    using NodeContainer = std::deque<std::shared_ptr<internal::FrLMNodeBase>>;
    using ElementContainer = std::deque<std::shared_ptr<internal::FrLMElement>>;

    NodeContainer m_nodes;
    ElementContainer m_elements;

  };

}  // end namespace frydom



#endif //FRYDOM_FRLUMPEDMASSCABLE_H
