//
// Created by camille on 28/05/18.
//

#include <iostream>
#include <algorithm>
#include "FrManoeuvringDamping.h"
#include "chrono/physics/ChBody.h"
#include "MathUtils/Maths.h"

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

}
