//
// Created by frongere on 22/11/18.
//

#ifndef FRYDOM_FRRADIATIONCONVOLUTIONMODEL_H
#define FRYDOM_FRRADIATIONCONVOLUTIONMODEL_H


#include "FrRadiationModel.h"

#include "frydom/core/FrVector.h"


namespace frydom {

    // Forward declarations
    template <class T>
    class FrRecorder_;



    class FrRadiationConvolutionModel_ : public FrRadiationModel_ {

    private:

        using VelocityRecorder = std::pair<const FrBody_*, std::unique_ptr<FrRecorder_<GeneralizedVelocity>>>;
        using RecorderContainer = std::vector<VelocityRecorder>;
        RecorderContainer m_velocityRecorders;


    public:

        explicit FrRadiationConvolutionModel_(std::shared_ptr<FrHydroDB_> hdb);

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;


    private:

        void GatherBodiesGeneralizedVelocities();

        void ComputeConvolutions();

        using RecorderIter = RecorderContainer::iterator;
        RecorderIter begin_recorder();
        RecorderIter end_recorder();


    };

}  // end namespace frydom




#endif //FRYDOM_FRRADIATIONCONVOLUTIONMODEL_H
