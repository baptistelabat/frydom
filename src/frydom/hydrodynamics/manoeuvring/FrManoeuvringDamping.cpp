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


#include <iostream>
#include <algorithm>
#include "FrManoeuvringDamping.h"
#include "chrono/physics/ChBody.h"
#include "MathUtils/Maths.h"

#include "frydom/core/body/FrBody.h"

namespace frydom {

    FrManDampingTaylorExpansion_::TypeParam_ FrManDampingTaylorExpansion_::SetParams(double val, int m, int n, int p) {
        TypeParam_ param;
        if (m > 0) {
            param.cm = true;
            param.m.first = int(m / 2);
            param.m.second = m - param.m.first;
        }
        if (n > 0) {
            param.cn = true;
            param.n.first = int(n/2);
            param.n.second = n - param.n.first;
        }
        if (p > 0) {
            param.cp = true;
            param.p.first = int(p/2);
            param.p.second = p - param.p.first;
        }
        param.val = val;
        return param;
    }

    FrManDampingTaylorExpansion_::TypeParam_ FrManDampingTaylorExpansion_::SetParams(std::string tag, double val) {
        auto m = int(std::count(tag.begin(), tag.end(), 'u'));
        auto n = int(std::count(tag.begin(), tag.end(), 'v'));
        auto p = int(std::count(tag.begin(), tag.end(), 'w'));
        return SetParams(val, m, n, p);
    }

    double FrManDampingTaylorExpansion_::ForceComponent(const TypeParam_ param, double vx, double vy, double vrz) const {
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

   void FrManDampingTaylorExpansion_::Set(std::string tag, double val) {

       if (tag.at(0) == 'X') {
           SetX(tag, val);
       } else if (tag.at(0) == 'Y') {
           SetY(tag , val);
       } else if (tag.at(0) == 'N') {
           SetN(tag, val);
       } else {
           std::cout << "warning : invalid coefficient definition" << std::endl;
       }
   }

    void FrManDampingTaylorExpansion_::SetX(std::string tag, double val) {
        m_cx.push_back(SetParams(tag, val));
    }

    void FrManDampingTaylorExpansion_::SetY(std::string tag, double val) {
        m_cy.push_back(SetParams(tag, val));
    }

    void FrManDampingTaylorExpansion_::SetN(std::string tag, double val) {
        m_cn.push_back(SetParams(tag, val));
    }

    void FrManDampingTaylorExpansion_::SetX(double val, int m, int n, int p) {
        m_cx.push_back(SetParams(val, m, n, p));
    }

    void FrManDampingTaylorExpansion_::SetY(double val, int m, int n, int p) {
        m_cy.push_back(SetParams(val, m, n, p));
    }

    void FrManDampingTaylorExpansion_::SetN(double val, int m, int n, int p) {
        m_cn.push_back(SetParams(val, m, n, p));
    }

    void FrManDampingTaylorExpansion_::ClearAll() {
        m_cx.clear();
        m_cy.clear();
        m_cn.clear();
    }

    void FrManDampingTaylorExpansion_::Update(double time) {

        auto force = Force();
        auto torque = Torque();

        auto vx = m_body->GetVelocityInBody(NWU).GetVx();
        auto vy = m_body->GetVelocityInBody(NWU).GetVy();
        auto vrz = m_body->GetAngularVelocityInBody(NWU).GetWz();

        for (auto& cx: m_cx) {
            force.GetFx() -= ForceComponent(cx, vx, vy, vrz);
        }
        for (auto& cy: m_cy) {
            force.GetFy() -= ForceComponent(cy, vx, vy, vrz);
        }
        for (auto& cn: m_cn) {
            torque.GetMz() -= ForceComponent(cn, vx, vy, vrz);
        }

        SetForceTorqueInBodyAtCOG(force ,torque, NWU);
    }

    void FrManDampingTaylorExpansion_::Initialize() {
        FrForce_::Initialize();
    }

    void FrManDampingTaylorExpansion_::StepFinalize() {
        FrForce_::StepFinalize();
    }

}  // end namespace frydom
