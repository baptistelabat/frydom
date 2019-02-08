//
// Created by frongere on 07/02/19.
//

#ifndef FRYDOM_FRSINFUNCTION_H
#define FRYDOM_FRSINFUNCTION_H

#include "FrFunction.h"

namespace frydom {

    class FrSinFunction : public FrFunction_ {

    private:
        double m_amplitude = 1.;
        double m_angularFrequency = 1.;
        double m_phase = 0.;
        double m_YOffset = 0.;

    public:

        FrSinFunction();

        void SetAmplitude(double amplitude);

        double GetAmplitude() const;

        void SetHeight(double height);

        double GetHeight() const;

        void SetAngularFrequency(double w);

        double GetAngularFrequency() const;

        void SetFrequency(double f);

        double GetFrequency() const;

        void SetPeriod(double T);

        double GetPeriod() const;

        void SetPhase(double phase);

        double GetPhase() const;

        void SetYOffset(double yOffset);

        double GetYOffset() const;

    protected:
        void Eval(double x) const override;


    };

}  // end namespace frydom

#endif //FRYDOM_FRSINFUNCTION_H
