//
// Created by camille on 14/12/17.
//

#ifndef FRYDOM_FRWAVEDRIFTFORCE_H
#define FRYDOM_FRWAVEDRIFTFORCE_H

#include <frydom/core/FrForce.h>
#include <frydom/environment/waves/FrWaveProbe.h>
#include <frydom/core/FrHydroBody.h>
#include <MathUtils.h>

namespace frydom {

    class FrDriftMode {

    public:
        enum TYPE {
            LINEAR,
            ANGULAR,
        };

    private:
        TYPE m_type;                    ///< Type linear or angular of the mode
        Eigen::Vector3d m_direction;    ///< Direction of the mode
        Eigen::Vector3d m_point;        ///< Application point of the mode

        bool m_active = true;           ///< Status of the mode

    public:

        FrDriftMode() = default;

        void SetTypeLINEAR() { m_type = LINEAR; }
        void SetTypeAngular() { m_type = ANGULAR; }

        TYPE GetType() const { return m_type; }

        void SetDirection(Eigen::Vector3d& direction) { m_direction = direction; }
        Eigen::Vector3d GetDirection() const { return m_direction; }

        void SetPoint(Eigen::Vector3d& point) { m_point = point; }
        Eigen::Vector3d GetPoint() const { return m_point; }

        void Activate() { m_active = true; }
        void Deactivate() { m_active = false; }
        bool IsActive() const { return m_active; }

    };


    class FrWaveDriftForce : public FrForce {

    public:

        /// Construct a new force model from drift table coefficients
        FrWaveDriftForce(const std::string hdf5_file);

        /// Wave probe attached to the force at the application point location
        void SetWaveProbe(std::shared_ptr<FrLinearWaveProbe>& waveProbe) { m_waveProbe = waveProbe; }

        /// Definition of the complex amplitude at the wave probe
        void SetCmplxElevation();

        /// Definition of the body where the force is applied
        void SetBody(FrHydroBody* body) { m_body = body;}

        /// Building the wave drift coefficient interpolator
        void BuildDriftCoefficientInterpolators();

        /// Update procedure containing the Wave Drift Force definition
        void UpdateState() override;

    private:

        int m_NbModes;                                     ///< Number of modes representd in the database
        std::vector<FrDriftMode> m_modes;                  ///< Definition of the mode
        std::shared_ptr<FrLinearWaveProbe> m_waveProbe;    ///< Wave probe for local wave field characteristics
        std::vector<std::vector<std::complex<double>>> m_CmplxElevation;  ///< Wave complex elevation
        std::vector<mathutils::LookupTable2d<>> m_table;                 ///< Lookup table 2D depending on freq and heading (for each mode)
        FrHydroBody*    m_body;                            ///< Hydro body to which the force is applied

    };

// end namespace frydom
}

#endif //FRYDOM_FRWAVEDRIFTFORCE_H
