//
// Created by frongere on 22/11/18.
//

#include "FrRadiationConvolutionModel.h"

#include "frydom/utils/FrRecorder.h"




namespace frydom   {

    FrRadiationConvolutionModel_::FrRadiationConvolutionModel_(std::shared_ptr<FrHydroDB_> hdb) :
            FrRadiationModel_(hdb) {}

    void FrRadiationConvolutionModel_::Initialize() {

        // Initializing velocity recorders

        unsigned long nbBodies = m_HDB->GetNbBodies();
        m_velocityRecorders.reserve(nbBodies);

        FrBEMBody_* bemBody;

        auto bodyIter = m_HDB->begin_body();
        for (; bodyIter != m_HDB->end_body(); bodyIter++) {

            bemBody = bodyIter.GetBEMBody();


            // FIXME : il faut recuperer le timeStep de system ainsi que le temps de persistance depuis la HDB pour chaque corps...
            double timePersistance, timeStep;

            auto newRecorder = std::make_unique<FrRecorder_<GeneralizedVelocity>>(timePersistance, timeStep);
            VelocityRecorder velocityRecorder = std::make_pair(m_HDB->GetFrydomBody(bemBody), std::move(newRecorder));
            m_velocityRecorders.push_back(std::move(velocityRecorder));


            /// TERMINER !!!!!!!!!!!



        }



    }


    void FrRadiationConvolutionModel_::Update(double time) {
        assert(time == GetTime());  // to be sure we are going to gather veolicities at the correct time

        GatherBodiesGeneralizedVelocities();
        ComputeConvolutions();

    }

    void FrRadiationConvolutionModel_::StepFinalize() {
        // TODO
    }


    void FrRadiationConvolutionModel_::GatherBodiesGeneralizedVelocities() {

        const FrBody_* frydomBody;
        GeneralizedVelocity generalizedVelocity;

        auto recorderIter = begin_recorder();
        for (; recorderIter!= end_recorder(); recorderIter++) {

            frydomBody = (*recorderIter).first;

            generalizedVelocity.SetVelocity(frydomBody->GetCOGVelocityInWorld(NWU));
            generalizedVelocity.SetAngularVelocity(frydomBody->GetAngularVelocityInWorld(NWU));

            (*recorderIter).second->Record(GetTime(), generalizedVelocity);

        }

    }

    void FrRadiationConvolutionModel_::ComputeConvolutions() {
        // TODO
    }


    FrRadiationConvolutionModel_::RecorderIter FrRadiationConvolutionModel_::begin_recorder() {
        return m_velocityRecorders.begin();
    }

    FrRadiationConvolutionModel_::RecorderIter FrRadiationConvolutionModel_::end_recorder() {
        return m_velocityRecorders.end();
    }






}  // end namespace