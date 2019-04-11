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


#ifndef FRYDOM_FREXCITATIONFORCEBASE_H
#define FRYDOM_FREXCITATIONFORCEBASE_H

#include "frydom/core/math/FrVector.h"
#include "MathUtils/Interp1d.h"
#include "frydom/core/force/FrForce.h"

namespace frydom {

    // Forward declarations.
    class FrHydroDB;
    class FrEquilibriumFrame;

    /**
     * \class FrExcitationForce
     * \brief Class for defining an excitation force (linear or nonlinear).
     */
    class FrExcitationForceBase : public FrForce {

    protected:

        /// Interpolator in waves frequencies and directions.
        std::vector<std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators;

        /// Hydrodynamic database.
        std::shared_ptr<FrHydroDB> m_HDB;

        /// Excitation loads (linear excitation) or diffraction loads (nonlinear excitation).
        std::vector<Eigen::MatrixXcd> m_Fhdb;

        //TODO: passed the raw to shared ptr, need some modif in the mapper.
        FrEquilibriumFrame* m_equilibriumFrame;

        /// Excitation or diffraction force.
        Force m_WorldForce = Force();

        /// Excitation or diffraction torque.
        Torque m_WorldTorque = Torque();


    public:

        /// Constructor.
        explicit FrExcitationForceBase(std::shared_ptr<FrHydroDB> HDB) : m_HDB(HDB) {};

        virtual Eigen::MatrixXcd GetHDBData(const unsigned int iangle) const = 0;

        virtual Eigen::VectorXcd GetHDBData(const unsigned int iangle, const unsigned iforce) const = 0;

        /// This function interpolates the excitation loads (linear excitation) or the diffraction loads (nonlinear excitation) with respect to the wave frequencies and directions.
        virtual void BuildHDBInterpolators();

        /// This function return the excitation force (linear excitation) or the diffraction force (nonlinear excitation) form the interpolator.
        std::vector<Eigen::MatrixXcd> GetHDBInterp(std::vector<double> waveFrequencies,
                                                          std::vector<double> waveDirections,
                                                          mathutils::ANGLE_UNIT angleUnit);

        void Initialize() override;

        /// This function computes the excitation loads (linear excitation) or the diffraction loads (nonlinear excitation).
        void Compute_F_HDB();

    };


} // end namespace frydom

#endif //FRYDOM_FREXCITATIONFORCEBASE_H