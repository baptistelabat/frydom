// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#include "FrMorisonModel.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironmentInc.h"


namespace frydom {

    template<typename OffshoreSystemType>
    std::shared_ptr<FrMorisonCompositeElement<OffshoreSystemType>>
    make_morison_model(FrBody<OffshoreSystemType> *body) {
      return std::make_shared<FrMorisonCompositeElement>(body);
    }

    // -----------------------------------------------------------------
    // MORISON MODEL
    // -----------------------------------------------------------------
    template<typename OffshoreSystemType>
    void FrMorisonElement<OffshoreSystemType>::SetFrame(FrBody<OffshoreSystemType> *body, Position posA, Position posB,
                                                        Direction vect) {

      Position position = 0.5 * (posA + posB);

      Direction e3 = posB - posA;
      e3.normalize();
      Direction e1 = vect.cross(e3);

      if (std::abs(e1.norm()) > FLT_EPSILON) {
        e1.normalize();
      } else {
        e1 = Direction(1., 0., 0.);
      }

      Direction e2 = e3.cross(e1);
      e2.normalize();

//        m_node = std::make_shared<FrNode<OffshoreSystemType>>(body, position, FrRotation(e1, e2, e3, NWU));
      m_node = std::make_shared<FrNode<OffshoreSystemType>>(body);  // TODO : doit etre gere par la classe de base !!
      m_node->SetFrameInBody(FrFrame(position, FrRotation(e1, e2, e3, NWU), NWU));
    }

    template<typename OffshoreSystemType>
    void FrMorisonElement<OffshoreSystemType>::SetFrame(FrBody<OffshoreSystemType> *body, const FrFrame &frame) {
      m_node = std::make_shared<FrNode<OffshoreSystemType>>(body);
      m_node->SetFrameInBody(frame);
    }

    template<typename OffshoreSystemType>
    Force FrMorisonElement<OffshoreSystemType>::GetForceInWorld(FRAME_CONVENTION fc) const {
      auto force = m_force;
      if (IsNED(fc)) internal::SwapFrameConvention(force);
      return force;
    }

    template<typename OffshoreSystemType>
    Torque FrMorisonElement<OffshoreSystemType>::GetTorqueInBody() const {
      return m_torque;
    }

    template<typename OffshoreSystemType>
    FrFrame FrMorisonElement<OffshoreSystemType>::GetFrame() const { return this->m_node->GetFrameInWorld(); }


    // ---------------------------------------------------------------------
    // MORISON SINGLE ELEMENT
    // ---------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrMorisonSingleElement<OffshoreSystemType>::FrMorisonSingleElement(FrBody<OffshoreSystemType> *body) {
      this->m_node = std::make_shared<FrNode<OffshoreSystemType>>(body);
    }

    template<typename OffshoreSystemType>
    FrMorisonSingleElement<OffshoreSystemType>::FrMorisonSingleElement(FrBody<OffshoreSystemType> *body, Position posA,
                                                                       Position posB, double diameter,
                                                                       MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                                       Direction perpendicular) {
      SetAddedMass(ca);
      SetDragCoeff(cd);
      SetFrictionCoeff(cf);

      //this->m_node = std::make_shared<FrNode<OffshoreSystemType>>(body);
      SetFrame(body, posA, posB, perpendicular);

      SetDiameter(diameter);
      SetLength(posA, posB);
      SetVolume();
    }

    template<typename OffshoreSystemType>
    FrMorisonSingleElement<OffshoreSystemType>::FrMorisonSingleElement(
        std::shared_ptr<FrNode<OffshoreSystemType>> &nodeA,
        std::shared_ptr<FrNode<OffshoreSystemType>> &nodeB,
        double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
        Direction perpendicular) {
      SetNodes(nodeA, nodeB);

      SetAddedMass(ca);
      SetDragCoeff(cd);
      SetFrictionCoeff(cf);

      //this->m_node = std::make_shared<FrNode<OffshoreSystemType>>(nodeA->GetBody());
      SetFrame(nodeA->GetBody(), nodeA->GetNodePositionInBody(NWU), nodeB->GetNodePositionInBody(NWU), perpendicular);

      SetDiameter(diameter);
      SetLength(nodeA->GetPositionInWorld(NWU), nodeB->GetPositionInWorld(NWU));
      SetVolume();
    }

    template<typename OffshoreSystemType>
    FrMorisonSingleElement<OffshoreSystemType>::FrMorisonSingleElement(FrBody<OffshoreSystemType> *body, FrFrame frame,
                                                                       double diameter, double length,
                                                                       MorisonCoeff ca, MorisonCoeff cd, double cf) {
      SetAddedMass(ca);
      SetDragCoeff(cd);
      SetFrictionCoeff(cf);

      SetFrame(body, frame);

      SetDiameter(diameter);
      SetLength(length);
      SetVolume();
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::SetNodes(std::shared_ptr<FrNode<OffshoreSystemType>> &nodeA,
                                                              std::shared_ptr<FrNode<OffshoreSystemType>> &nodeB) {
      m_nodeA = nodeA;
      m_nodeB = nodeB;
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::SetNodes(FrBody<OffshoreSystemType> *body, Position posA,
                                                              Position posB) {
      m_nodeA = std::make_shared<FrNode<OffshoreSystemType>>(body);
      m_nodeA->SetPositionInBody(posA, NWU);
      m_nodeB = std::make_shared<FrNode<OffshoreSystemType>>(body);
      m_nodeB->SetPositionInBody(posB, NWU);
      SetLength(m_nodeA->GetPositionInWorld(NWU), m_nodeB->GetPositionInWorld(NWU));
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::SetAddedMass(MorisonCoeff ca) {
      assert(ca.x >= -FLT_EPSILON or std::abs(ca.x) <= FLT_EPSILON);
      assert(ca.y >= -FLT_EPSILON or std::abs(ca.y) <= FLT_EPSILON);
      m_property.ca = ca;
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::SetDragCoeff(MorisonCoeff cd) {
      assert(cd.x >= -FLT_EPSILON or std::abs(cd.x) <= FLT_EPSILON);
      assert(cd.y >= -FLT_EPSILON or std::abs(cd.y) <= FLT_EPSILON);
      m_property.cd = cd;
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::SetFrictionCoeff(double cf) {
      assert(cf >= -FLT_EPSILON or std::abs(cf) <= FLT_EPSILON);
      m_property.cf = cf;
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::SetDiameter(const double diameter) {
      assert(diameter >= -FLT_EPSILON or std::abs(diameter) <= FLT_EPSILON);
      m_property.diameter = diameter;
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::SetLength(Position posA, Position posB) {
      m_property.length = (posB - posA).norm();
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::SetVolume() {
      m_property.volume = MU_PI_4 * GetDiameter() * GetDiameter() * GetLength();
    }

    template<typename OffshoreSystemType>
    Velocity FrMorisonSingleElement<OffshoreSystemType>::GetFlowVelocity() {

      Velocity velocity;
      Position worldPos = this->m_node->GetPositionInWorld(NWU);
      auto body = this->m_node->GetBody();

      auto waveField = body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

      velocity = waveField->GetVelocity(worldPos, NWU);
      velocity -= this->m_node->GetVelocityInWorld(NWU);

      if (this->m_includeCurrent) {
        velocity += body->GetSystem()->GetEnvironment()->GetOcean()->GetCurrent()->GetFluxVelocityInWorld(worldPos,
                                                                                                          NWU);
      }

      Velocity velocityBody = body->GetFrame().ProjectVectorParentInFrame(velocity, NWU);
      return this->m_node->GetFrameInWorld().ProjectVectorParentInFrame(velocityBody, NWU);
    }

    template<typename OffshoreSystemType>
    Acceleration FrMorisonSingleElement<OffshoreSystemType>::GetFlowAcceleration() {

      Acceleration acceleration;
      Position worldPos = this->m_node->GetPositionInWorld(NWU);
      auto body = this->m_node->GetBody();

      auto waveField = body->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetWaveField();

      acceleration = waveField->GetAcceleration(worldPos, NWU);
      acceleration -= this->m_node->GetAccelerationInWorld(NWU);

      Acceleration accBody = body->GetFrame().ProjectVectorParentInFrame(acceleration, NWU);
      return this->m_node->GetFrameInWorld().ProjectVectorParentInFrame(accBody, NWU);
    }

    //
    // UPDATE
    //
    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::Update(double time) {

      Force localForce;

      auto body = this->m_node->GetBody();
      auto rho = body->GetSystem()->GetEnvironment()->GetOcean()->GetDensity();

      Velocity velocity = GetFlowVelocity();
      localForce.x() =
          0.5 * m_property.cd.x * rho * m_property.diameter * m_property.length * velocity.x() * std::abs(velocity.x());
      localForce.y() =
          0.5 * m_property.cd.y * rho * m_property.diameter * m_property.length * velocity.y() * std::abs(velocity.y());

      if (this->m_extendedModel) {
        Acceleration acceleration = GetFlowAcceleration();
        localForce.x() += rho * (m_property.ca.x + 1.) * GetVolume() * acceleration.x();
        localForce.y() += rho * (m_property.ca.y + 1.) * GetVolume() * acceleration.y();
      }

      localForce.z() = 0.5 * m_property.cf * rho * M_PI * m_property.diameter * m_property.length * velocity.z() *
                       std::abs(velocity.z());

      // Project force in world at COG
      auto forceBody = this->m_node->GetFrameInWorld().ProjectVectorFrameInParent(localForce, NWU);
      this->m_force = body->GetFrame().ProjectVectorFrameInParent(forceBody, NWU);

      //Project torque in body at COG
      Position relPos = this->m_node->GetNodePositionInBody(NWU) - body->GetCOG(NWU);
      this->m_torque = relPos.cross(forceBody);
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::Initialize() {
      assert(this->m_node);
      assert(this->m_node->GetBody());
      assert(m_property.length > FLT_EPSILON);
      assert(m_property.diameter > FLT_EPSILON);
      SetVolume();
    }

    template<typename OffshoreSystemType>
    void FrMorisonSingleElement<OffshoreSystemType>::StepFinalize() {

    }


    // -------------------------------------------------------------------
    // MORISON COMPOSITE FORCE MODEL
    // -------------------------------------------------------------------
    template<typename OffshoreSystemType>
    FrMorisonCompositeElement<OffshoreSystemType>::FrMorisonCompositeElement(FrBody<OffshoreSystemType> *body) {
      this->m_node = std::make_shared<FrNode<OffshoreSystemType>>(body);
    }

    template<typename OffshoreSystemType>
    FrMorisonCompositeElement<OffshoreSystemType>::FrMorisonCompositeElement(FrBody<OffshoreSystemType> *body,
                                                                             FrFrame &frame) {
      this->m_node = std::make_shared<FrNode<OffshoreSystemType>>(
          body); // TODO : Devrait etre instancie dans la classe de base
      this->m_node->SetFrameInBody(frame);
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::AddElement(std::shared_ptr<FrNode<OffshoreSystemType>> &nodeA,
                                                                   std::shared_ptr<FrNode<OffshoreSystemType>> &nodeB,
                                                                   double diameter,
                                                                   MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                                   Direction perpendicular) {
      m_morison.push_back(std::make_unique<FrMorisonSingleElement>(nodeA, nodeB, diameter, ca, cd, cf, perpendicular));
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::AddElement(std::shared_ptr<FrNode<OffshoreSystemType>> &nodeA,
                                                                   std::shared_ptr<FrNode<OffshoreSystemType>> &nodeB,
                                                                   Direction perpendicular) {
      m_morison.push_back(std::make_unique<FrMorisonSingleElement>(nodeA, nodeB, m_property.diameter,
                                                                   m_property.ca, m_property.cd,
                                                                   m_property.cf, perpendicular));
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::AddElement(Position posA, Position posB, double diameter,
                                                                   MorisonCoeff ca, MorisonCoeff cd, double cf,
                                                                   unsigned int n,
                                                                   Direction perpendicular) {
      Direction dV = (posB - posA) / n;

      Position pos;
      for (unsigned int i = 0; i < n; ++i) {
        pos = posA + dV * i;
        m_morison.push_back(std::make_unique<FrMorisonSingleElement>(this->m_node->GetBody(), pos, pos + dV, diameter,
                                                                     ca, cd, cf, perpendicular));
      }
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::AddElement(Position posA, Position posB, unsigned int n,
                                                                   Direction perpendicular) {
      AddElement(posA, posB, m_property.diameter, m_property.ca, m_property.cd, m_property.cf, n, perpendicular);
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::AddElement(FrFrame frame, double length, double diameter,
                                                                   MorisonCoeff ca, MorisonCoeff cd, double cf) {
      m_morison.push_back(std::make_unique<FrMorisonSingleElement>(this->m_node->GetBody(), frame, diameter,
                                                                   length, ca, cd, cf));
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::AddElement(FrFrame frame, double length) {
      AddElement(frame, length, m_property.diameter, m_property.ca, m_property.cd, m_property.cf);
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::SetDragCoeff(MorisonCoeff cd) {
      m_property.cd = cd;
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::SetFrictionCoeff(double cf) {
      m_property.cf = cf;
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::SetAddedMass(MorisonCoeff ca) {
      m_property.ca = ca;
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::SetDiameter(double diameter) {
      m_property.diameter = diameter;
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::Initialize() {

      for (auto &element: m_morison) {
        element->Initialize();
        element->SetExtendedModel(this->m_extendedModel);
      }
    }

    template<typename OffshoreSystemType>
    void FrMorisonCompositeElement<OffshoreSystemType>::Update(double time) {

      m_force.SetNull();
      m_torque.SetNull();

      for (auto &element : m_morison) {
        element->Update(time);
        m_force += element->GetForceInWorld(NWU);
        m_torque += element->GetTorqueInBody();
      }
    }

}  // end namespace frydom
