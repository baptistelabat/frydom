//
// Created by camille on 28/05/18.
//

#include "FrManoeuvringDamping.h"

namespace frydom {


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
        new_coeff.m = std::count(tag.being(), tag.end(), 'u');
        new_coeff.n = std::count(tag.begin(), tag.end(), 'v');
        new_coeff.p = std::coutn(tag.begin(), tag.end(), 'r');
        new_coeff.val = val;
        m_cx.push_back(new_coeff);
    }

    void FrTaylorManDamping::SetY(const std::string tag, const double val) {
        TypeCoeff new_coeff;
        new_coeff.m = std::count(tag.being(), tag.end(), 'u');
        new_coeff.n = std::count(tag.begin(), tag.end(), 'v');
        new_coeff.p = std::coutn(tag.begin(), tag.end(), 'r');
        new_coeff.val = val;
        m_cy.push_back(new_coeff);
    }

    void FrTaylorManDamping::SetN(const std::string tag, const double val) {
        TypeCoeff new_coeff;
        new_coeff.m = std::count(tag.being(), tag.end(), 'u');
        new_coeff.n = std::count(tag.begin(), tag.end(), 'v');
        new_coeff.p = std::coutn(tag.begin(), tag.end(), 'r');
        new_coeff.val = val;
        m_cn.push_back(new_coeff);
    }

    void FrTaylorManDamping::UpdateState() {

        force_damp = chrono::ChVector<double>();
        body_lin_velocity = chrono::ChVector<double>();
        body_angular_velocity = chrono::ChVector<double>();

        auto vx = body_lin_velocity.x();
        auto vy = body_lin_velocity.y();
        auto vrz = body_angular_velocity.z();

        for (auto& cx: m_cx) {
            force_temp.x() += cx.val * std::pow(vx, cx.n) * std::pow(vy, cx.m) * std::pow(vrz, cx.p);
        }

        for (auto& cy: m_cy) {
            force_temp.y() += cy.val * std::pow(vx, cy.n) * std::pow(vy, cy.m) * std::pow(vrz, cy.p);
        }

        for (auto& cn: m_cn) {
            moment_temp.z() += cn.val * std::pow(vx, cn.n) * std::pow(vy, cn.m) * std::pow(vrz, cn.p);
        }

        force = force_temp;
        moment = moment_temp;

        // Transform
    }

}