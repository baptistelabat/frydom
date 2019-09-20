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

#include "frydom/core/math/functions/ramp/FrCosRampFunction.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrWaveField<OffshoreSystemType>::FrWaveField(FrFreeSurface<OffshoreSystemType> *freeSurface) : m_freeSurface(freeSurface) {
      m_infinite_depth = freeSurface->GetOcean()->GetSeabed()->GetInfiniteDepth();
    }

//    FrWaveField<OffshoreSystemType>::WAVE_MODEL FrWaveField<OffshoreSystemType>::GetWaveModel() const { return m_waveModel; }
    template<typename OffshoreSystemType>
    Velocity FrWaveField<OffshoreSystemType>::GetVelocity(double x, double y, double z, bool cutoff, FRAME_CONVENTION fc) const {

      if (cutoff) {
        auto wave_elevation = GetElevation(x, y, fc);
        if (wave_elevation < z) {
          return {0., 0., 0.};
        }
      }
      return GetVelocity(x, y, z, fc);
    }

    template<typename OffshoreSystemType>
    Velocity FrWaveField<OffshoreSystemType>::GetVelocity(const Position &worldPos, FRAME_CONVENTION fc) const {
      return GetVelocity(worldPos.GetX(), worldPos.GetY(), worldPos.GetZ(), fc);
    }

    template<typename OffshoreSystemType>
    Acceleration FrWaveField<OffshoreSystemType>::GetAcceleration(double x, double y, double z, bool cutoff, FRAME_CONVENTION fc) const {

      if (cutoff) {
        auto wave_elevation = GetElevation(x, y, fc);
        if (wave_elevation < z) {
          return {0., 0., 0.};
        }
      }
      return GetAcceleration(x, y, z, fc);
    }

    template<typename OffshoreSystemType>
    Acceleration FrWaveField<OffshoreSystemType>::GetAcceleration(const Position &worldPos, FRAME_CONVENTION fc) const {
      return GetAcceleration(worldPos.GetX(), worldPos.GetY(), worldPos.GetZ(), fc);
    }

    template<typename OffshoreSystemType>
    void FrWaveField<OffshoreSystemType>::Initialize() {
      m_infinite_depth = m_freeSurface->GetOcean()->GetSeabed()->GetInfiniteDepth();
      if (!m_infinite_depth) { c_depth = m_freeSurface->GetOcean()->GetDepth(NWU); };
      c_density = m_freeSurface->GetOcean()->GetDensity();
      c_gravity = m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();
    }

    template<typename OffshoreSystemType>
    void FrWaveField<OffshoreSystemType>::StepFinalize() {
    }

    template<typename OffshoreSystemType>
    std::vector<std::vector<double>>
    FrWaveField<OffshoreSystemType>::GetElevation(const std::vector<double> &xVect, const std::vector<double> &yVect,
                              FRAME_CONVENTION fc) const {
      auto nx = xVect.size();
      auto ny = yVect.size();

      std::vector<std::vector<double>> elevations;
      std::vector<double> elev;

      elevations.reserve(nx);
      elev.reserve(ny);

      double eta, x, y;
      for (unsigned int ix = 0; ix < nx; ++ix) {
        elev.clear();
        x = xVect[ix];
        for (unsigned int iy = 0; iy < ny; ++iy) {
          y = yVect[iy];
          eta = GetElevation(x, y, fc);
          elev.push_back(eta);
        }
        elevations.push_back(elev);
      }
      return elevations;
    }

    template<typename OffshoreSystemType>
    std::vector<std::vector<std::vector<Velocity>>>
    FrWaveField<OffshoreSystemType>::GetVelocity(const std::vector<double> &xvect, const std::vector<double> &yvect,
                             const std::vector<double> &zvect, FRAME_CONVENTION fc) const {
      auto nx = xvect.size();
      auto ny = yvect.size();
      auto nz = zvect.size();

      std::vector<std::vector<std::vector<Velocity>>> velocity;
      std::vector<std::vector<Velocity>> velocity_x;
      std::vector<Velocity> velocity_y;
      Velocity velocity_z;

      double x, y, z;

      for (unsigned int ix = 0; ix < nx; ++ix) {
        velocity_x.clear();
        x = xvect[ix];
        for (unsigned int iy = 0; iy < ny; ++iy) {
          velocity_y.clear();
          y = yvect[iy];
          for (unsigned int iz = 0; iz < nz; ++iz) {
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

//    Velocity FrWaveField<OffshoreSystemType>::GetVelocity(double x, double y, double z, bool cutoff, FRAME_CONVENTION fc) const {
//
//        if (cutoff) {
//            auto wave_elevation = GetElevation(x, y, fc);
//            if (wave_elevation < z) {
//                return {0.,0.,0.};
//            }
//        }
//        return GetVelocity(x, y, z, fc);
//    }
    template<typename OffshoreSystemType>
    void FrWaveField<OffshoreSystemType>::Update(double time) {
      c_time = time;
      if (m_freeSurface->GetOcean()->GetEnvironment()->GetTimeRamp()->IsActive()) {
        c_ramp = m_freeSurface->GetOcean()->GetEnvironment()->GetTimeRamp()->Get_y(c_time);
      }
      if (!m_infinite_depth) { c_depth = m_freeSurface->GetOcean()->GetDepth(NWU); };
    }

    template<typename OffshoreSystemType>
    double FrWaveField<OffshoreSystemType>::GetPressure(Position position, FRAME_CONVENTION fc) const {
      return GetPressure(position.GetX(), position.GetY(), position.GetZ(), fc);
    }


    // FrNullWaveField definitions
    template<typename OffshoreSystemType>
    FrNullWaveField<OffshoreSystemType>::FrNullWaveField(FrFreeSurface<OffshoreSystemType> *freeSurface) : FrWaveField<OffshoreSystemType>(freeSurface) {
//        m_waveModel = NO_WAVES;
    }

    template<typename OffshoreSystemType>
    double FrNullWaveField<OffshoreSystemType>::GetElevation(double x, double y, FRAME_CONVENTION fc) const {
      return 0.;
    }

    template<typename OffshoreSystemType>
    Velocity FrNullWaveField<OffshoreSystemType>::GetVelocity(double x, double y, double z, FRAME_CONVENTION fc) const {
      return {0., 0., 0.};
    }

    template<typename OffshoreSystemType>
    Acceleration FrNullWaveField<OffshoreSystemType>::GetAcceleration(double x, double y, double z, FRAME_CONVENTION fc) const {
      return {0., 0., 0.};
    }

    template<typename OffshoreSystemType>
    std::vector<double> FrNullWaveField<OffshoreSystemType>::GetWaveFrequencies(FREQUENCY_UNIT unit) const {
      return std::vector<double>(1, 0.);
    }

    template<typename OffshoreSystemType>
    std::vector<double> FrNullWaveField<OffshoreSystemType>::GetWaveNumbers() const {
      return std::vector<double>(1, 0.);
    }

    template<typename OffshoreSystemType>
    std::vector<std::vector<double>> FrNullWaveField<OffshoreSystemType>::GetWaveAmplitudes() const {
      return std::vector<std::vector<double>>(1, std::vector<double>(1, 0.));
    }

    template<typename OffshoreSystemType>
    std::vector<double> FrNullWaveField<OffshoreSystemType>::GetWaveDirections(ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                                           DIRECTION_CONVENTION dc) const {
      return std::vector<double>(1, 0.);
    }

    template<typename OffshoreSystemType>
    std::vector<std::vector<Complex>>
    FrNullWaveField<OffshoreSystemType>::GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const {
      return std::vector<std::vector<Complex>>(1, std::vector<Complex>(1, 0.));
    }

    template<typename OffshoreSystemType>
    double FrNullWaveField<OffshoreSystemType>::GetPressure(double x, double y, double z, FRAME_CONVENTION fc) const {

      // Get the pressure at the position (x,y,z) for a null wave field.

      return 0.;

    }

}  // end namespace frydom
