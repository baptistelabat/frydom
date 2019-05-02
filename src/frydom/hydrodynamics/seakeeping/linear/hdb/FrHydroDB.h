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
#include <unordered_map>
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
     * \class FrDiscretization1D
     * \brief Class for the linear discretization (frequency and angle).
     */
    class FrDiscretization1D {
    private:
        double m_xmin = 0.;         ///< Min value of the discretization
        double m_xmax = 0.;         ///< Max value of the discretization
        unsigned int m_nx = 0;      ///< Number of discrete value

    public:

        /// Default constructor
        FrDiscretization1D() = default;

        /// Constructor with specified parameters
        /// \param xmin Min abscisse
        /// \param xmax Max abscisse
        /// \param nx Number of discrete value
        FrDiscretization1D(double xmin, double xmax, unsigned int nx)
                : m_xmin(xmin), m_xmax(xmax), m_nx(nx) {}

        /// Set / Get the min value of the discretization vector
        double GetMin() const { return m_xmin; }
        void SetMin(double xmin) { m_xmin = xmin; }

        /// Set / Get the max value of the discretization vector
        double GetMax() const { return m_xmax; }
        void SetMax(double xmax) { m_xmax = xmax; }

        /// Set / Get the number of discrete value
        unsigned int GetNbSample() const { return m_nx; }
        void SetNbSample(unsigned int nx) { m_nx = nx; }

        /// Return the discrete vector
        /// \return
        std::vector<double> GetVector() const;

        /// Define the step the discretization vector
        /// \param delta Step value
        void SetStep(double delta);

        /// Return the step of the discretization vector
        /// \return Step value
        double GetStep() const;
    };

    // --------------------------------------------------------
    // FrHydroDB
    // --------------------------------------------------------

    /**
     * \class FrHydroDB
     * \brief Class for dealling with hydrodynamic databases.
     */
    class FrHydroDB {

    private:

        double m_gravityAcc;                ///< gravity coming from the HDB
        double m_waterDensity;              ///< water density coming from the HDB
        double m_waterDepth;                ///< water depth coming from the HDB
        double m_normalizationLength;       ///< Normalization length coming from the HDB
        int m_nbody;                        ///< Number of bodies in interaction considered in the HDB

        std::vector<std::unique_ptr<FrBEMBody>> m_bodies;   ///< List of BEM body database
        std::unique_ptr<FrHydroMapper> m_mapper;            ///< Mapper between bodies and BEM body database

        FrDiscretization1D m_frequencyDiscretization;       ///< Wave frequency discretization
        FrDiscretization1D m_waveDirectionDiscretization;   ///< Wave direction discretization
        FrDiscretization1D m_timeDiscretization;            ///< Time samples

    public:

        /// Constructor of the hydrodynamic database HDB
        FrHydroDB() = default;

        /// Constructor of the hydrodynamic database with specified HDF5 filename
        explicit FrHydroDB(std::string h5file);

        /// Readers of the different items present in the HDF5 database
        /// reader : HDF5 reader
        /// path : path of the item in the HDF5 file
        /// BEMBody : BEM database of the corresponding body
        void MaskReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        void ModeReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        void ExcitationReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        void RadiationReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        void WaveDriftReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        void HydrostaticReader(FrHDF5Reader& reader, std::string path, FrBEMBody* BEMBody);

        /// Return the number of bodies in the HDB
        /// \return Number of bodies
        unsigned int GetNbBodies() const { return (uint)m_bodies.size(); };

        /// Set the wave direction time and frequency discretization as discribed in the database
        void SetWaveDirectionDiscretization(const double minAngle, const double maxAngle, const unsigned int nbAngle);

        void SetTimeDiscretization(const double finalTime, const unsigned int nbTimeSamples);

        void SetFrequencyDiscretization(const double minFreq, const double maxFreq, const unsigned int nbFreq);

        /// Return the wave frequencies discretization of the hydrodynamic database
        /// \return Wae frequency discretization
        std::vector<double> GetFrequencies() const { return m_frequencyDiscretization.GetVector(); }

        /// Return the wave direction discretization of the hydrodynamic database
        /// \param angleUnit Unit of the direction angle
        /// \param fc Frame convention
        /// \return Wave direction discretization
        std::vector<double> GetWaveDirections(ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const;

        /// Return the number of discrete wave frequencies in the database
        /// \return Number of discrete wave frequencies
        unsigned int GetNbFrequencies() const { return m_frequencyDiscretization.GetNbSample(); }

        /// Return the maximum value of the discrete wave frequency in the database
        /// \return Maximum value of the discrete wave frequency
        double GetMaxFrequency() const { return m_frequencyDiscretization.GetMax(); }

        /// Return the minimum value of the discrete wave frequency in the database
        /// \return Minimum value of the discrete wave frequency
        double GetMinFrequency() const { return m_frequencyDiscretization.GetMin(); }

        /// Return the frequency step of the database
        /// \return Frequency step
        double GetStepFrequency() const {return m_frequencyDiscretization.GetStep(); }

        /// Return the number of discrete wave direction in the database
        /// \return Number of directe wave direction
        unsigned int GetNbWaveDirections() const { return m_waveDirectionDiscretization.GetNbSample(); }

        /// Return the number of time samples in the database
        /// \return Number of time samples
        unsigned int GetNbTimeSamples() const { return m_timeDiscretization.GetNbSample(); }

        /// Return the maximum time in the database
        /// \return Maximum time
        double GetFinalTime() const { return m_timeDiscretization.GetMax(); }

        /// Return the time step of the database
        /// \return Time step
        double GetTimeStep() const { return m_timeDiscretization.GetStep(); }

        /// Return the discrete time vector of the database
        /// \return Time vector
        std::vector<double> GetTimeDiscretization() const { return m_timeDiscretization.GetVector(); }

        /// Add a new BEM body database to HDB
        /// \param bodyName Name of the BEM body
        /// \return BEM body pointer
        FrBEMBody* NewBody(std::string bodyName);

        /// Return the BEM body database
        /// \param ibody Index of the BEM body
        /// \return BEM Body database
        FrBEMBody* GetBody(int ibody);

        /// Return the BEM body database
        /// \param body frydom body object
        /// \return BEM body database
        FrBEMBody* GetBody(std::shared_ptr<FrBody> body);

        /// Return the BEM body database
        /// \param body frydom body object
        /// \return BEM body database
        FrBEMBody* GetBody(FrBody* body);

        /// Return the frydom body object
        /// \param body BEM body database
        /// \return frydom body
        FrBody* GetBody(FrBEMBody* body);

        /// Return the mapper between frydom body and BEM body database
        /// \return Mapper
        FrHydroMapper* GetMapper();

        /// Define a map between a BEM body database and a body
        /// \param BEMBody BEM body database
        /// \param body body (frydom object)
        /// \param eqFrame Equilibrium frame of the corresponding body
        void Map(FrBEMBody* BEMBody, FrBody* body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

        /// Define a map between a BEM body database and a body
        /// \param iBEMBody Index of the BEM body database in the HDB
        /// \param body body (frydom object)
        /// \param eqFrame Equilibrium frame of the corresponding body
        void Map(int iBEMBody, FrBody* body, std::shared_ptr<FrEquilibriumFrame> eqFrame);

        //std::vector<std::unique_ptr<FrBEMBody>>::iterator begin() { return m_bodies.begin(); }

        //std::vector<std::unique_ptr<FrBEMBody>>::iterator end() { return m_bodies.end(); }

        std::unordered_map<FrBEMBody*, FrBody*>::iterator begin();

        std::unordered_map<FrBEMBody*, FrBody*>::iterator end();

        /// Return the water density stored in the HDB
        /// \return Water density
        double GetWaterDensity(){return m_waterDensity;};

        /// Return the gravity acceleration stored in the HDB
        /// \return Gravity acceleration
        double GetGravityAcc(){return m_gravityAcc;};

    };

    std::shared_ptr<FrHydroDB> make_hydrodynamic_database(std::string h5file);

}  // end namespace frydom


#endif //FRYDOM_FRHYDRODB_H
