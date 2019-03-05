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


#include "FrWaveField.h"

#include "frydom/core/math/functions/ramp/FrLinearRampFunction.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"


namespace frydom {

    FrWaveField::FrWaveField(FrFreeSurface *freeSurface) : m_freeSurface(freeSurface) {
        m_infinite_depth = freeSurface->GetOcean()->GetSeabed()->GetInfiniteDepth();
    }

    FrWaveField::WAVE_MODEL FrWaveField::GetWaveModel() const { return m_waveModel; }

    Velocity FrWaveField::GetVelocity(double x, double y, double z, bool cutoff, FRAME_CONVENTION fc) const {

        if (cutoff) {
            auto wave_elevation = GetElevation(x, y, fc);
            if (wave_elevation < z) {
                return {0.,0.,0.};
            }
        }
        return GetVelocity(x, y, z, fc);
    }

    Velocity FrWaveField::GetVelocity(const Position& worldPos, FRAME_CONVENTION fc) const {
        return GetVelocity(worldPos.GetX(), worldPos.GetY(), worldPos.GetZ(), fc);
    }

    Acceleration FrWaveField::GetAcceleration(double x, double y, double z, bool cutoff, FRAME_CONVENTION fc) const {

        if (cutoff) {
            auto wave_elevation = GetElevation(x, y, fc);
            if (wave_elevation < z) {
                return {0.,0.,0.};
            }
        }
        return GetAcceleration(x, y, z, fc);
    }

    Acceleration FrWaveField::GetAcceleration(const Position& worldPos, FRAME_CONVENTION fc) const {
        return GetAcceleration(worldPos.GetX(), worldPos.GetY(), worldPos.GetZ(), fc);
    }

    void FrWaveField::Initialize() {
        m_infinite_depth = m_freeSurface->GetOcean()->GetSeabed()->GetInfiniteDepth();
        if (!m_infinite_depth) {c_depth = m_freeSurface->GetOcean()->GetDepth(NWU);};
    }

    void FrWaveField::StepFinalize() {
    }

    std::vector<std::vector<double>>
    FrWaveField::GetElevation(const std::vector<double> &xVect, const std::vector<double> &yVect, FRAME_CONVENTION fc) const {
        auto nx = xVect.size();
        auto ny = yVect.size();

        std::vector<std::vector<double>> elevations;
        std::vector<double> elev;

        elevations.reserve(nx);
        elev.reserve(ny);

        double eta, x, y;
        for (unsigned int ix=0; ix<nx; ++ix) {
            elev.clear();
            x = xVect[ix];
            for (unsigned int iy=0; iy<ny; ++iy) {
                y = yVect[iy];
                eta = GetElevation(x, y, fc);
                elev.push_back(eta);
            }
            elevations.push_back(elev);
        }
        return elevations;
    }

    std::vector<std::vector<std::vector<Velocity>>>
    FrWaveField::GetVelocity(const std::vector<double> &xvect, const std::vector<double> &yvect,
                              const std::vector<double> &zvect, FRAME_CONVENTION fc) const {
        auto nx = xvect.size();
        auto ny = yvect.size();
        auto nz = zvect.size();

        std::vector<std::vector<std::vector<Velocity>>> velocity;
        std::vector<std::vector<Velocity>> velocity_x;
        std::vector<Velocity> velocity_y;
        Velocity velocity_z;

        double x, y, z;

        for (unsigned int ix=0; ix<nx; ++ix ) {
            velocity_x.clear();
            x = xvect[ix];
            for (unsigned int iy=0; iy<ny; ++iy) {
                velocity_y.clear();
                y = yvect[iy];
                for (unsigned int iz=0; iz<nz; ++iz) {
                    z = zvect[iz];
                    velocity_z = GetVelocity(x, y, z, fc);
                    velocity_y.push_back(velocity_z);
                }
                velocity_x.push_back(velocity_y);
            }
            velocity.push_back(velocity_x);
        }
        return velocity;
    }

    void FrWaveField::Update(double time) {
        c_time = time;
        if (m_freeSurface->GetOcean()->GetEnvironment()->GetTimeRamp()->IsActive()) {
            c_ramp = m_freeSurface->GetOcean()->GetEnvironment()->GetTimeRamp()->Get_y(c_time);
        }
        if (!m_infinite_depth) {c_depth = m_freeSurface->GetOcean()->GetDepth(NWU);};
    }


    // FrNullWaveField definitions

    FrNullWaveField::FrNullWaveField(FrFreeSurface* freeSurface) : FrWaveField(freeSurface) {
        m_waveModel = NO_WAVES;
    }

    double FrNullWaveField::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
        return 0.;
    }

    Velocity FrNullWaveField::GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
        return {0.,0.,0.};
    }

    Acceleration FrNullWaveField::GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const {
        return {0.,0.,0.};
    }

    std::vector<double> FrNullWaveField::GetWaveFrequencies(FREQUENCY_UNIT unit) const {
        return std::vector<double>(1, 0.);
    }

    std::vector<double> FrNullWaveField::GetWaveNumbers() const {
        return std::vector<double>(1, 0.);
    }

    std::vector<std::vector<double>> FrNullWaveField::GetWaveAmplitudes() const {
        return std::vector<std::vector<double>>(1, std::vector<double>(1, 0.));
    }

    std::vector<double> FrNullWaveField::GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                            DIRECTION_CONVENTION dc) const {
        return std::vector<double>(1, 0.);
    }

    std::vector<std::vector<Complex>> FrNullWaveField::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {
        return std::vector<std::vector<Complex>>(1, std::vector<Complex>(1, 0.));
    }

}  // end namespace frydom
