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
        TypeCoeff new_coeff;
        new_coeff.m = std::count(tag.begin(), tag.end(), 'u');
        new_coeff.n = std::count(tag.begin(), tag.end(), 'v');
        new_coeff.p = std::count(tag.begin(), tag.end(), 'r');
        new_coeff.val = val;
        m_cx.push_back(new_coeff);
    }

    void FrTaylorManDamping::SetY(const std::string tag, const double val) {
        TypeCoeff new_coeff;
        new_coeff.m = std::count(tag.begin(), tag.end(), 'u');
        new_coeff.n = std::count(tag.begin(), tag.end(), 'v');
        new_coeff.p = std::count(tag.begin(), tag.end(), 'r');
        new_coeff.val = val;
        m_cy.push_back(new_coeff);
    }

    void FrTaylorManDamping::SetN(const std::string tag, const double val) {
        TypeCoeff new_coeff;
        new_coeff.m = std::count(tag.begin(), tag.end(), 'u');
        new_coeff.n = std::count(tag.begin(), tag.end(), 'v');
        new_coeff.p = std::count(tag.begin(), tag.end(), 'r');
        new_coeff.val = val;
        m_cn.push_back(new_coeff);
    }

    void FrTaylorManDamping::SetX(const double val, const double m, const double n, const double p) {
        TypeCoeff new_coeff;
        new_coeff.m = m;
        new_coeff.n = n;
        new_coeff.p = p;
        new_coeff.val = val;
        m_cx.push_back(new_coeff);
    }

    void FrTaylorManDamping::SetY(const double val, const double m, const double n, const double p) {
        TypeCoeff new_coeff;
        new_coeff.m = m;
        new_coeff.n = n;
        new_coeff.p = p;
        new_coeff.val = val;
        m_cy.push_back(new_coeff);
    }

    void FrTaylorManDamping::SetN(const double val, const double m, const double n, const double p) {
        TypeCoeff new_coeff;
        new_coeff.m = m;
        new_coeff.n = n;
        new_coeff.p = p;
        new_coeff.val = val;
        m_cn.push_back(new_coeff);
    }

    void FrTaylorManDamping::UpdateState() {

        auto force_temp = chrono::ChVector<double>();
        auto moment_temp = chrono::ChVector<double>();
        auto body_lin_velocity = Body->GetPos_dt();
        auto body_angular_velocity = Body->GetWvel_par();

        auto vxa = std::abs(body_lin_velocity.x());
        auto vya = std::abs(body_lin_velocity.y());
        auto vrza = std::abs(body_angular_velocity.z());

        for (auto& cx: m_cx) {
            force_temp.x() -= cx.val * std::pow(vxa, cx.m) * std::pow(vya, cx.n) * std::pow(vrza, cx.p);
        }

        for (auto& cy: m_cy) {
            force_temp.y() -= cy.val * std::pow(vxa, cy.m) * std::pow(vya, cy.n) * std::pow(vrza, cy.p);
        }

        for (auto& cn: m_cn) {
            moment_temp.z() -= cn.val * std::pow(vxa, cn.m) * std::pow(vya, cn.n) * std::pow(vrza, cn.p);
        }

        force = force_temp;
        moment = moment_temp;

        // Transform
    }

}