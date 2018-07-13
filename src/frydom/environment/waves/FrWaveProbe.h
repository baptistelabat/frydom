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
#include "frydom/core/FrNode.h"
#include "chrono/core/ChFrameMoving.h"
#include "frydom/core/FrEulerAngles.h"

namespace frydom {

    // =================================================================================================================
    // Forward declaration
    class FrWaveField;

    /// Class to make a measurement of the wave elevation with respect to the mean water height (tidal)
    /// It is mainly used at a fixed absolute position in the local horizontal plane
    class FrWaveProbe : public FrObject {

    protected:
        double m_x = 0;                                     ///< Abs position in x of the sensor (m)
        double m_y = 0;                                     ///< Abs position in y of the sensor (m)
        FrWaveField* m_waveField;                           ///< Wave field
        std::shared_ptr<chrono::ChFrameMoving<double>> m_node;      ///< Node to which the wave probe is attached
        std::shared_ptr<FrRamp> m_waveRamp;                 ///< Ramp applied at the initial stage

    public:
        /// Default constructor
        FrWaveProbe() {}

        /// New constructor from sensor position in the horizontal plane
        FrWaveProbe(double x, double y) : m_x(x), m_y(y) {
            m_node = std::make_shared<chrono::ChFrameMoving<double>>();
            m_node->GetPos().x() = x;
            m_node->GetPos().y() = y;
        }

        /// Set Abs position in X
        void SetX(double x) {
            m_x = x;
            m_node->GetPos().x() = x;
        }

        /// Return the abs position in X of the sensor
        double& GetX() const { return m_node->GetPos().x(); }

        /// Set Abs position in Y
        void SetY(double y) {
            m_y = y;
            m_node->GetPos().y() = y;
        }

        /// Return the abs position in Y of the sensor
        double& GetY() const { return m_node->GetPos().y(); }

        /// Return the wave field
        virtual FrWaveField* GetWaveField() const = 0;

        /// Initialization of the wave probe sensor
        virtual void Initialize() override {};

        /// Method run after each time step
        virtual void StepFinalize() override {}

        /// Attached a node to the wave probe
        void AttachedNode(std::shared_ptr<chrono::ChFrameMoving<double>> node) {
            m_node = node;
            m_x = GetX();
            m_y = GetY();}


        /// Return the free surface elevation at the sensor position
        virtual double GetElevation(double time) const = 0;

    };

    // =================================================================================================================
    // Forward declaration
    class FrLinearWaveField;

    /// Specialization of the wave probe for linear wave field and moving frame.
    /// Assumed moving frame and encounter frequency

    class FrLinearWaveProbe : public FrWaveProbe {

    public:
        FrLinearWaveProbe() : FrWaveProbe() {}

        FrLinearWaveProbe(double x, double y) : FrWaveProbe(x, y) {}

        /// Set the linear wave field linked to the wave probe
        void SetWaveField(FrLinearWaveField *waveField) { m_waveField = waveField; }

        /// Return the linear wave field to which the wave probe is linked
        FrLinearWaveField *GetWaveField() const override {
            return dynamic_cast<FrLinearWaveField *>(m_waveField);
        }

        /// Compute complex elevation depending on the position and frequency
        std::vector<std::vector<std::complex<double>>> GetCmplxElevation() const;

        /// Return the wave elevation at the wave probe position
        virtual double GetElevation(double time) const override;

    };

    /// Specialization of the wave probe to optimize cpu time for linear wave field
    /// Assumed linear wave field and fixe position if the global reference frame.
    /// Assumed only time frequency dependance.

    class FrLinearWaveProbeSteady : public FrLinearWaveProbe {  // TODO: mettre en cache la steady elevation pour les params x, y

    private:
        std::vector<std::complex<double>> m_steadyElevation;            ///< steady part of the wave elevation

    public:
        FrLinearWaveProbeSteady(): FrLinearWaveProbe() {}

        /// Constructor of the wave sensor with position in the horizontal plane
        FrLinearWaveProbeSteady(double x, double y) : FrLinearWaveProbe(x, y) {}

        /// Set the steady part of the wave elevation
        virtual void Initialize() override;

        /// Return the wave elevation at the sensor position
        virtual double GetElevation(double time) const override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRWAVEPROBE_H

