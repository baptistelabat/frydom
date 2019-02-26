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


#ifndef FRYDOM_FRHYDRODB_H
#define FRYDOM_FRHYDRODB_H

#include <vector>
#include <memory>

#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrUnits.h"


namespace frydom {

    // Forward declarations
    class FrHydroMapper;
    class FrBEMBody;
    class FrBody;
    class FrHDF5Reader;
    class FrEquilibriumFrame;

    // ----------------------------------------------------------
    // FrDiscretization1D
    // ----------------------------------------------------------

    /**
     * \class FrDiscretization1D_
     * \brief Class for the linear discretization (frequency and angle).
     */
    class FrDiscretization1D {
    private:
        double m_xmin = 0.;
        double m_xmax = 0.;
        unsigned int m_nx = 0;

    public:

        FrDiscretization1D() = default;

        FrDiscretization1D(double xmin, double xmax, unsigned int nx)
                : m_xmin(xmin), m_xmax(xmax), m_nx(nx) {}

        double GetMin() const { return m_xmin; }
        void SetMin(double xmin) { m_xmin = xmin; }

        double GetMax() const { return m_xmax; }
        void SetMax(double xmax) { m_xmax = xmax; }

        unsigned int GetNbSample() const { return m_nx; }
        void SetNbSample(unsigned int nx) { m_nx = nx; }

        std::vector<double> GetVector() const;

        void SetStep(double delta);

        double GetStep() const;

    };

    // --------------------------------------------------------
    // FrHydroDB
    // --------------------------------------------------------

    /**
     * \class FrHydroDB_
     * \brief Class for dealling with hydrodynamic databases.
     */
    class FrHydroDB {

    private:

        double m_gravityAcc;
        double m_waterDensity;
        double m_waterDepth;
        double m_normalizationLength;
        int m_nbody;

        std::vector<std::unique_ptr<FrBEMBody>> m_bodies;
        std::unique_ptr<FrHydroMapper> m_mapper;

        FrDiscretization1D m_frequencyDiscretization;
        FrDiscretization1D m_waveDirectionDiscretization;
        FrDiscretization1D m_timeDiscretization;

    public:

        /// Constructor of the class.
        FrHydroDB() = default;

        /// Constructor of the class.
        explicit FrHydroDB(std::string h5file);

        /// This subroutine reads the modes of a body.
        void ModeReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        /// This subroutine reads the excitation loads from the *.HDB5 input file.
        void ExcitationReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        /// This subroutine reads the radiation coefficients from the *.HDB5 input file.
        void RadiationReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        /// This subroutine reads the wave drift coefficients.
        void WaveDriftReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        /// This subroutine reads the hydrostatic matrix.
        void HydrostaticReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        /// This subroutine gives the number of bodies.
        unsigned int GetNbBodies() const { return (uint)m_bodies.size(); };

        void SetWaveDirectionDiscretization(const double minAngle, const double maxAngle, const unsigned int nbAngle);

        void SetTimeDiscretization(const double finalTime, const unsigned int nbTimeSamples);

        void SetFrequencyDiscretization(const double minFreq, const double maxFreq, const unsigned int nbFreq);

        std::vector<double> GetFrequencies() const { return m_frequencyDiscretization.GetVector(); }

        std::vector<double> GetWaveDirections(ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const;

        unsigned int GetNbFrequencies() const { return m_frequencyDiscretization.GetNbSample(); }

        double GetMaxFrequency() const { return m_frequencyDiscretization.GetMax(); }

        double GetMinFrequency() const { return m_frequencyDiscretization.GetMin(); }

        double GetStepFrequency() const {return m_frequencyDiscretization.GetStep(); }

        unsigned int GetNbWaveDirections() const { return m_waveDirectionDiscretization.GetNbSample(); }

        unsigned int GetNbTimeSamples() const { return m_timeDiscretization.GetNbSample(); }

        double GetFinalTime() const { return m_timeDiscretization.GetMax(); }

        double GetTimeStep() const { return m_timeDiscretization.GetStep(); }

        std::vector<double> GetTimeDiscretization() const { return m_timeDiscretization.GetVector(); }

        FrBEMBody* NewBody(std::string bodyName);

        FrBEMBody* GetBody(int ibody);

        FrBEMBody* GetBody(std::shared_ptr<FrBody> body);

        FrBEMBody* GetBody(FrBody* body);

        FrBody* GetBody(FrBEMBody* body);

        FrHydroMapper* GetMapper();

        void Map(FrBEMBody* BEMBody, FrBody* body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

        void Map(int iBEMBody, FrBody* body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

        std::vector<std::unique_ptr<FrBEMBody>>::iterator begin() { return m_bodies.begin(); }

        std::vector<std::unique_ptr<FrBEMBody>>::iterator end() { return m_bodies.end(); }
    };


    std::shared_ptr<FrHydroDB> make_hydrodynamic_database(std::string h5file);


}  // end namespace frydom


#endif //FRYDOM_FRHYDRODB_H
