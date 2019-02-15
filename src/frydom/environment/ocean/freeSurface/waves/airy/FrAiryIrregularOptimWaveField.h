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


#ifndef FRYDOM_FRAIRYIRREGULAROPTIMWAVEFIELD_H
#define FRYDOM_FRAIRYIRREGULAROPTIMWAVEFIELD_H

#include "FrAiryIrregularWaveField.h"

namespace frydom {

    /**
     * \class FrAiryIrregularOptimWaveField
     * \brief Class which deals with irregular wave field, from FrAiryIrregularWaveField.
     */
    class FrAiryIrregularOptimWaveField : public FrAiryIrregularWaveField{
    private:

        std::vector<Complex> c_expJwt;    ///< Cache value of exp(-j.w_m.t) for the different w_m frequencies
        std::vector<double> c_cosTheta;   ///< Cache value of cos(theta_n) for the different theta_n directions
        std::vector<double> c_sinTheta;   ///< Cache value of sin(theta_n) for the different theta_n directions
        std::vector<std::vector<Complex>> c_AExpJphi;    ///< Cache value of A(w_m,theta_n).exp[-j.phase(w_m,theta_n)]
                                                         ///< for the different w_m frequencies and theta_j directions
    public:

        /// Default constructor
        /// \param freeSurface free surface containing this wave field
        explicit FrAiryIrregularOptimWaveField(FrFreeSurface_* freeSurface);

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
        std::vector<mathutils::Vector3d<Complex>> GetComplexVelocity(double x, double y, double z, FRAME_CONVENTION fc) const final;

        /// Initialize the state of the wave field
        void Initialize() override;

        /// Method called at the send of a time step.
        void StepFinalize() override;

        /// Update of the internal cache attributes
        void InternalUpdate();
    };
} //end namespace frydom

#endif //FRYDOM_FRAIRYIRREGULAROPTIMWAVEFIELD_H
