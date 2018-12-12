//
// Created by Lucas Letournel on 11/12/18.
//

#ifndef FRYDOM_FRAIRYREGULAROPTIMWAVEFIELD_H
#define FRYDOM_FRAIRYREGULAROPTIMWAVEFIELD_H

#include "FrAiryRegularWaveField.h"

namespace frydom {

    class FrAiryRegularOptimWaveField : public FrAiryRegularWaveField {
    private:
        /// cache attribute for the time dependant coefficient (H.exp(-j.w.t)
        Complex c_expJwt;
        /// cache attribute for the cos(theta) value
        double c_cosTheta;
        /// cache attribute for the sin(theta) value
        double c_sinTheta;

    public:
        explicit FrAiryRegularOptimWaveField(FrFreeSurface_* freeSurface);

        /// Get the complex wave elevation at the position (x,y,0), of the regular Airy wave field
        /// \param x x position
        /// \param y y position
        /// \return complex wave elevation, in meters
        Complex GetComplexElevation(double x, double y) const final;

        /// Return the complex eulerian fluid particule velocity in global reference frame (implemented in child)
        /// \param x x position
        /// \param y y position
        /// \param z z position
        /// \return complex eulerian fluid particule velocity, in m/s
        mathutils::Vector3d<Complex> GetComplexVelocity(double x, double y, double z) const final;

        /// Initialize the state of the wave field
        void Initialize() override;

        /// Method called at the send of a time step.
        void StepFinalize() override;

        /// Update of the internal cache attributes
        void InternalUpdate();

    };

} // end namespace frydom
#endif //FRYDOM_FRAIRYREGULAROPTIMWAVEFIELD_H
