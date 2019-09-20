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


#include "FrManoeuvringDamping.h"

#include "frydom/core/body/FrBody.h"


namespace frydom {

    template<typename OffshoreSystemType>
    typename FrManDampingTaylorExpansion<OffshoreSystemType>::TypeParam_
    FrManDampingTaylorExpansion<OffshoreSystemType>::SetParams(double val, int m, int n, int p) {
      TypeParam_ param;
      if (m > 0) {
        param.cm = true;
        param.m.first = int(m / 2);
        param.m.second = m - param.m.first;
      }
      if (n > 0) {
        param.cn = true;
        param.n.first = int(n / 2);
        param.n.second = n - param.n.first;
      }
      if (p > 0) {
        param.cp = true;
        param.p.first = int(p / 2);
        param.p.second = p - param.p.first;
      }
      param.val = val;
      return param;
    }

    template<typename OffshoreSystemType>
    typename FrManDampingTaylorExpansion<OffshoreSystemType>::TypeParam_
    FrManDampingTaylorExpansion<OffshoreSystemType>::SetParams(std::string tag, double val) {
      auto m = int(std::count(tag.begin(), tag.end(), 'u'));
      auto n = int(std::count(tag.begin(), tag.end(), 'v'));
      auto p = int(std::count(tag.begin(), tag.end(), 'w'));
      return SetParams(val, m, n, p);
    }

    template<typename OffshoreSystemType>
    double FrManDampingTaylorExpansion<OffshoreSystemType>::ForceComponent(const TypeParam_ param, double vx, double vy,
                                                                           double vrz) const {
      double res;
      res = param.val;
      if (param.cm) {
        res *= std::pow(std::abs(vx), param.m.first) * std::pow(vx, param.m.second);
      }
      if (param.cn) {
        res *= std::pow(std::abs(vy), param.n.first) * std::pow(vy, param.n.second);
      }
      if (param.cp) {
        res *= std::pow(std::abs(vrz), param.p.first) * std::pow(vrz, param.p.second);
      }
      return res;
    }

    template<typename OffshoreSystemType>
    void FrManDampingTaylorExpansion<OffshoreSystemType>::Set(std::string tag, double val) {

      if (tag.at(0) == 'X') {
        SetX(tag, val);
      } else if (tag.at(0) == 'Y') {
        SetY(tag, val);
      } else if (tag.at(0) == 'N') {
        SetN(tag, val);
      } else {
        std::cout << "warning : invalid coefficient definition" << std::endl;
      }
    }

    template<typename OffshoreSystemType>
    void FrManDampingTaylorExpansion<OffshoreSystemType>::SetX(std::string tag, double val) {
      m_cx.push_back(SetParams(tag, val));
    }

    template<typename OffshoreSystemType>
    void FrManDampingTaylorExpansion<OffshoreSystemType>::SetY(std::string tag, double val) {
      m_cy.push_back(SetParams(tag, val));
    }

    template<typename OffshoreSystemType>
    void FrManDampingTaylorExpansion<OffshoreSystemType>::SetN(std::string tag, double val) {
      m_cn.push_back(SetParams(tag, val));
    }

    template<typename OffshoreSystemType>
    void FrManDampingTaylorExpansion<OffshoreSystemType>::SetX(double val, int m, int n, int p) {
      m_cx.push_back(SetParams(val, m, n, p));
    }

    template<typename OffshoreSystemType>
    void FrManDampingTaylorExpansion<OffshoreSystemType>::SetY(double val, int m, int n, int p) {
      m_cy.push_back(SetParams(val, m, n, p));
    }

    template<typename OffshoreSystemType>
    void FrManDampingTaylorExpansion<OffshoreSystemType>::SetN(double val, int m, int n, int p) {
      m_cn.push_back(SetParams(val, m, n, p));
    }

    template<typename OffshoreSystemType>
    void FrManDampingTaylorExpansion<OffshoreSystemType>::ClearAll() {
      m_cx.clear();
      m_cy.clear();
      m_cn.clear();
    }

    template<typename OffshoreSystemType>
    void FrManDampingTaylorExpansion<OffshoreSystemType>::Compute(double time) {

      auto force = Force();
      auto torque = Torque();

      auto vx = this->m_body->GetVelocityInBody(NWU).GetVx();
      auto vy = this->m_body->GetVelocityInBody(NWU).GetVy();
      auto vrz = this->m_body->GetAngularVelocityInBody(NWU).GetWz();

      for (auto &cx: m_cx) {
        force.GetFx() -= ForceComponent(cx, vx, vy, vrz);
      }
      for (auto &cy: m_cy) {
        force.GetFy() -= ForceComponent(cy, vx, vy, vrz);
      }
      for (auto &cn: m_cn) {
        torque.GetMz() -= ForceComponent(cn, vx, vy, vrz);
      }

      this->SetForceTorqueInBodyAtCOG(force, torque, NWU);
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrManDampingTaylorExpansion<OffshoreSystemType>>
    make_manoeuvring_model(const std::shared_ptr<FrBody<OffshoreSystemType>> &body) {

      auto manoeuvring = std::make_shared<FrManDampingTaylorExpansion<OffshoreSystemType>>();
      manoeuvring->ClearAll();

      body->AddExternalForce(manoeuvring);

    }

}  // end namespace frydom
