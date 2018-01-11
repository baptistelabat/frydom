//
// Created by frongere on 30/10/17.
//

#ifndef FRYDOM_FRWAVEPROBE_H
#define FRYDOM_FRWAVEPROBE_H

#include <memory>
#include <complex>

#include "chrono/core/ChVector.h"

#include "frydom/core/FrObject.h"
#include "FrWaveField.h"

namespace frydom {

    // =================================================================================================================
    // Forward declaration
    class FrWaveField;

    /// Class to make a measurement of the wave elevation with respect to the mean water height (tidal)
    /// It is mainly used at a fixed absolute position in the local horizontal plane
    class FrWaveProbe : public FrObject {
    protected:

        double m_x = 0;
        double m_y = 0;

        FrWaveField* m_waveField;

        std::shared_ptr<FrRamp> m_waveRamp;

    public:
        FrWaveProbe(double x, double y) : m_x(x), m_y(y) {}

//        void SetWaveField(FrWaveField* waveField) = 0;
//        virtual FrWaveField* GetWaveField() const { return m_waveField; }

        void SetX(double x) { m_x = x; }

        double GetX() const { return m_x; }

        void SetY(double y) { m_y = y; }

        double GetY() const { return m_y; }

        virtual FrWaveField* GetWaveField() const = 0;

    };

    // =================================================================================================================
    // Forward declaration
    class FrLinearWaveField;

    class FrLinearWaveProbe : public FrWaveProbe {  // TODO: mettre en cache la steady elevation pour les params x, y

    private:
        FrLinearWaveField* m_waveField = nullptr;

        std::vector<std::complex<double>> m_steadyElevation;

    public:
        FrLinearWaveProbe(double x, double y) : FrWaveProbe(x, y) {}

        void SetWaveField(FrLinearWaveField* waveField) { m_waveField = waveField; }

        FrLinearWaveField* GetWaveField() const override { return m_waveField; }

        void Initialize() {
            m_steadyElevation = m_waveField->GetSteadyElevation(m_x, m_y);
        }

        double GetElevation(double time) const {

            auto emjwt = m_waveField->GetTimeCoeffs(); // FIXME: tres couteux a l'appel...
            std::complex<double> elev = 0.;
            for (unsigned int ifreq=0; ifreq<emjwt.size(); ++ifreq) {
                elev += m_steadyElevation[ifreq] * emjwt[ifreq];
            }
            double realElev = imag(elev);

            // Applying the wave ramp
            auto waveRamp = m_waveField->GetWaveRamp();
            if (waveRamp && waveRamp->IsActive()) {
                m_waveField->GetWaveRamp()->Apply(
                        m_waveField->GetTime(),
                        realElev
                );
            }
            return realElev;
        }

    };


}  // end namespace frydom


#endif //FRYDOM_FRWAVEPROBE_H

