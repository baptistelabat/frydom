//
// Created by Lucas Letournel on 11/12/18.
//

#ifndef FRYDOM_FRAIRYIRREGULAROPTIMWAVEFIELD_H
#define FRYDOM_FRAIRYIRREGULAROPTIMWAVEFIELD_H

#include "FrAiryIrregularWaveField.h"

namespace frydom {
    class FrAiryIrregularOptimWaveField : public FrAiryIrregularWaveField{
    private:
        std::vector<Complex> c_expJwt;
        std::vector<double> c_cosTheta;
        std::vector<double> c_sinTheta;
        std::vector<std::vector<Complex>> c_AExpJphi;
    public:
        explicit FrAiryIrregularOptimWaveField(FrFreeSurface_* freeSurface);

        std::vector<std::vector<Complex>> GetComplexElevation(double x, double y) const final;

        std::vector<mathutils::Vector3d<Complex>> GetComplexVelocity(double x, double y, double z) const final;

        /// Initialize the state of the wave field
        void Initialize() override;

        /// Method called at the send of a time step.
        void StepFinalize() override;

        /// Update of the internal cache attributes
        void InternalUpdate();
    };
} //end namespace frydom

#endif //FRYDOM_FRAIRYIRREGULAROPTIMWAVEFIELD_H
