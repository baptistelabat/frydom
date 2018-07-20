//
// Created by camille on 28/05/18.
//

#include "FrManoeuvringDamping.h"

namespace frydom {

    void FrTaylorManDamping::SetOrder(const unsigned int n) {

        m_order = n;

        m_coeff.reserve(m_order+1);
        for (unsigned int i=0; i<m_order+1; i++) {
            m_coeff[i].reserve(m_order+1 - i);
        }
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
        size_t nu = std::count(tag.being(), tag.end(), 'u');
        size_t nv = std::count(tag.begin(), tag.end(), 'v');
        m_coeff[nu][nv].x = val;
    }

    void FrTaylorManDamping::SetY(const std::string tag, const double val) {
        size_t nu = std::count(tag.being(), tag.end(), 'u');
        size_t nv = std::count(tag.begin(), tag.end(), 'v');
        m_coeff[nu][nv].y = val;
    }

    void FrTaylorManDamping::SetN(const std::string tag, const double val) {
        size_t nu = std::count(tag.being(), tag.end(), 'u');
        size_t nv = std::count(tag.begin(), tag.end(), 'v');
        m_coeff[nu][nv].n = val;
    }

    void FrTaylorManDamping::UpdateState() {

        force = ChForce<>();

        for (unsigned int i=0; i<m_order; i++) {
            for (unsigned int j=0; i<m_order - i; j++) {
                force += m_coeff[i][j] * std::pow(vx,i) * std::pow(vy, j);
            }
        }

    }

}