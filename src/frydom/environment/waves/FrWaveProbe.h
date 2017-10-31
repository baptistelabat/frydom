//
// Created by frongere on 30/10/17.
//

#ifndef FRYDOM_FRWAVEPROBE_H
#define FRYDOM_FRWAVEPROBE_H

#include <memory>
#include <complex>

#include "chrono/core/ChVector.h"

namespace frydom {

    // Forward declaration
    class FrWaveField;

    /// Class to make a measurement of the wave elevation with respect to the mean water height (tidal)
    /// It is mainly used at a fixed absolute position in the local horizontal plane
    class FrWaveProbe {
    protected:

        double m_x = 0;
        double m_y = 0;

        FrWaveField* m_waveField;

    public:
        FrWaveProbe() = default;

        FrWaveProbe(double x, double y)
                : m_x(x), m_y(y) {};

//        virtual void SetWaveField(FrWaveField* waveField) = 0;

        virtual void SetPosition(double x, double y) = 0;

        chrono::ChVector<double> GetPosition() const {
            return chrono::ChVector<double>(m_x, m_y, 0.);
        }

        virtual void Initialize() = 0;

        virtual double GetElevation() const = 0;

        virtual std::vector<std::complex<double>> GetCmplxElevation() const = 0;
    };


    class FrRegularLinearWaveField;

    class FrLinearRegularWaveProbe : public FrWaveProbe {

    private:
        std::complex<double> m_steadyElevation;

        FrRegularLinearWaveField* m_waveField;

    public:
        FrLinearRegularWaveProbe() = default;

        FrLinearRegularWaveProbe(double x, double y) : FrWaveProbe(x, y) {}

        void SetWaveField(FrRegularLinearWaveField* waveField) { m_waveField = waveField; }

        void SetPosition(double x, double y) override { // TODO; mettre dans la classe de base
            m_x = x;
            m_y = y;
            Initialize();
        }

        void Initialize() override;

        double GetElevation() const override;

        std::vector<std::complex<double>> GetCmplxElevation() const override;

    };

    class FrIrregularLinearWaveField;

    class FrLinearIrregularWaveProbe : public FrWaveProbe {

    private:
        std::vector<std::complex<double>> m_steadyElevation;

        FrIrregularLinearWaveField* m_waveField;

    public:
        FrLinearIrregularWaveProbe(double x, double y) : FrWaveProbe(x, y) {}

        void SetWaveField(FrIrregularLinearWaveField* waveField) { m_waveField = waveField; }

        void SetPosition(double x, double y) override { // TODO; mettre dans la classe de base
            m_x = x;
            m_y = y;
            Initialize();
        }

        void Initialize() override;

        double GetElevation() const override;

        std::vector<std::complex<double>> GetCmplxElevation() const override;

    };






}  // end namespace frydom


#endif //FRYDOM_FRWAVEPROBE_H

