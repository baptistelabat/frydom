// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#include <iostream>
#include <algorithm>
#include "FrManoeuvringDamping.h"
#include "chrono/physics/ChBody.h"
#include "MathUtils/Maths.h"

#include "frydom/core/body/FrBody.h"

namespace frydom {

    template <typename T>
    int sgn_bis(T val) {
        int sgn = (T(0) < val) - (val < T(0));
        if (std::abs(sgn) < 0.5) { sgn = 1;}
        return sgn;
    }

    FrTaylorManDamping::TypeCoeff FrTaylorManDamping::SetCoeff(const std::string tag, const double val) {
        auto m = int(std::count(tag.begin(), tag.end(), 'u'));
        auto n = int(std::count(tag.begin(), tag.end(), 'v'));
        auto p = int(std::count(tag.begin(), tag.end(), 'r'));
        return SetCoeff(val, m, n, p);
    }


    FrTaylorManDamping::TypeCoeff FrTaylorManDamping::SetCoeff(const double val, const int m, const int n, const int p) {
        TypeCoeff new_coeff;
        if (m > 0) {
            new_coeff.cm = 1;
            new_coeff.m1 = int(m / 2);
            new_coeff.m2 = m - new_coeff.m1;
        }
        if (n > 0) {
            new_coeff.cn = 1;
            new_coeff.n1 = int(n / 2);
            new_coeff.n2 = n - new_coeff.n1;
        }
        if (p > 0) {
            new_coeff.cp = 1;
            new_coeff.p1 = int(p / 2);
            new_coeff.p2 = p - new_coeff.p1;
        }
        new_coeff.val = val;
        return new_coeff;
    }

    double FrTaylorManDamping::ForceComponent(const TypeCoeff coeff, const double vx, const double vy, const double vrz) const {
        double res;
        res = coeff.val;
        if (coeff.cm == 1) {
            res *= std::pow(std::abs(vx), coeff.m1) * std::pow(vx, coeff.m2);
        }
        if (coeff.cn == 1) {
            res *= std::pow(std::abs(vy), coeff.n1) * std::pow(vy, coeff.n2);
        }
        if (coeff.cp == 1) {
            res *= std::pow(std::abs(vrz),coeff.p1) * std::pow(vrz, coeff.p2);
        }
        return res;
        }

    void FrTaylorManDamping::Set(const std::string tag, const double val) {

        if (tag.at(0) == 'X') {
            SetX(tag, val);
        } else if (tag.at(0) == 'Y') {
            SetY(tag, val);
        } else if (tag.at(0) == 'N') {
            SetN(tag, val);
        } else {
            std::cout << "Warning : invalid coefficient definition" << std::endl;
        }
    }

    void FrTaylorManDamping::SetX(const std::string tag, const double val) {
        m_cx.push_back(SetCoeff(tag, val));
    }

    void FrTaylorManDamping::SetY(const std::string tag, const double val) {
        m_cy.push_back(SetCoeff(tag, val));
    }

    void FrTaylorManDamping::SetN(const std::string tag, const double val) {
        m_cn.push_back(SetCoeff(tag, val));
    }

    void FrTaylorManDamping::SetX(const double val, const int m, const int n, const int p) {
        m_cx.push_back(SetCoeff(val, m, n, p));
    }

    void FrTaylorManDamping::SetY(const double val, const int m, const int n, const int p) {
        m_cy.push_back(SetCoeff(val, m, n, p));
    }

    void FrTaylorManDamping::SetN(const double val, const int m, const int n, const int p) {
        m_cn.push_back(SetCoeff(val, m, n, p));
    }

    void FrTaylorManDamping::UpdateState() {

        auto force_temp = chrono::ChVector<double>();
        auto moment_temp = chrono::ChVector<double>();

        auto body_lin_velocity = Body->GetPos_dt();
        auto body_angular_velocity = Body->GetWvel_par();
        body_lin_velocity = Body->TransformDirectionParentToLocal(body_lin_velocity);

        auto vx = body_lin_velocity.x();
        auto vy = body_lin_velocity.y();
        auto vrz = body_angular_velocity.z();

        for (auto& cx: m_cx) {
            force_temp.x() -= ForceComponent(cx, vx, vy ,vrz);
        }

        for (auto& cy: m_cy) {
            force_temp.y() -= ForceComponent(cy, vx, vy, vrz);
        }

        for (auto& cn: m_cn) {
            moment_temp.z() -= ForceComponent(cn, vx, vy, vrz);
        }

        force = Body->TransformDirectionLocalToParent(force_temp);
        moment = Body->TransformDirectionParentToLocal(moment_temp);

    }
























    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

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
}
