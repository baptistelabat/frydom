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


#ifndef FRYDOM_FRAIRYREGULAROPTIMWAVEFIELD_H
#define FRYDOM_FRAIRYREGULAROPTIMWAVEFIELD_H

#include "FrAiryRegularWaveField.h"

namespace frydom {

    /**
     * \class FrAiryRegularOptimWaveField
     * \brief Class which deals with regular wave fields with optimization (cos(theta), etc.), from FrAiryRegularWaveField.
     */
    template<class OffshoreSystemType, class StretchingType>
    class FrAiryRegularOptimWaveField : public FrAiryRegularWaveField<OffshoreSystemType, StretchingType> {
     private:

      Complex c_expJwt;    ///< cache attribute for the time dependant coefficient (H.exp(-j.w.t)
      double c_cosTheta;   ///< cache attribute for the cos(theta) value
      double c_sinTheta;   ///< cache attribute for the sin(theta) value

     public:
      explicit FrAiryRegularOptimWaveField(FrFreeSurface<OffshoreSystemType> *freeSurface);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "AiryRegularOptimWaveField"; }

      /// Get the complex wave elevation at the position (x,y,0), of the regular Airy wave field
      /// \param x x position
      /// \param y y position
      /// \param fc frame convention (NED/NWU)
      /// \return complex wave elevation, in meters
      std::vector<std::vector<Complex>> GetComplexElevation(double x, double y, FRAME_CONVENTION fc) const final;

      /// Return the complex eulerian fluid particule velocity in global reference frame (implemented in child)
      /// \param x x position
      /// \param y y position
      /// \param z z position
      /// \param fc frame convention (NED/NWU)
      /// \return complex eulerian fluid particule velocity, in m/s
      mathutils::Vector3d<Complex> GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const final;

      /// Initialize the state of the wave field
      void Initialize() override;

      /// Method called at the send of a time step.
      void StepFinalize() override;

      /// Update of the internal cache attributes
      void InternalUpdate();

    };

} // end namespace frydom

#endif //FRYDOM_FRAIRYREGULAROPTIMWAVEFIELD_H
