//
// Created by frongere on 30/10/17.
//

#ifndef FRYDOM_FRWAVEPROBE_H
#define FRYDOM_FRWAVEPROBE_H

#include <memory>
#include <complex>

#include "chrono/core/ChVector.h"
#include "FrWaveField.h"

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
        FrWaveProbe(double x, double y) : m_x(x), m_y(y) {}
//
//        void SetWaveField(FrWaveField* waveField) = 0;
//        virtual FrWaveField* GetWaveField() const { return m_waveField; }

        void SetX(double x) { m_x = x; }

        double GetX() const { return m_x; }

        void SetY(double y) { m_y = y; }

        double GetY() const { return m_y; }

    };

    class FrLinearWaveField;

    class FrLinearWaveProbe : public FrWaveProbe {

    private:
        FrLinearWaveField* m_waveField;

    public:
        FrLinearWaveProbe(double x, double y) : FrWaveProbe(x, y) {}

        void SetWaveField(FrLinearWaveField* waveField) { m_waveField = waveField; }
        FrLinearWaveField* GetWaveField() const { return m_waveField; }

    };










}  // end namespace frydom


#endif //FRYDOM_FRWAVEPROBE_H

