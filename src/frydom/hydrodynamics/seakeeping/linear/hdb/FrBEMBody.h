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


#ifndef FRYDOM_FRBEMBODY_H
#define FRYDOM_FRBEMBODY_H


#include "frydom/core/math/FrVector.h"

#include "MathUtils/LookupTable2D.h"
#include "MathUtils/Interp1d.h"
#include "MathUtils/Matrix.h"


namespace frydom {

    // Forward declaration
    class FrHydroDB;

    /**
     * \class FrBEMMode
     * \brief
     */
    class FrBEMMode {

    public:
        enum TYPE {
            LINEAR,
            ANGULAR
        };

    private:
        TYPE m_type;
        Direction m_direction;
        Position m_position;
        bool m_active = true;

    public:
        FrBEMMode() = default;

        void SetTypeLINEAR() { m_type = LINEAR; }
        void SetTypeANGULAR() { m_type = ANGULAR; }
        TYPE GetType() const { return m_type; }

        void SetDirection(Direction& direction) { m_direction = direction; }
        Direction GetDirection() const { return m_direction; }

        void SetPointPosition(Position& position) { m_position = position; }
        Position GetPointPosition() const { return m_position; }

        // TODO : flag pour le couplage FrBody/FrBEMBody (a voir si conservation necessaire)
        void Activate() { m_active = true; }
        void Deactivate() { m_active = false; }
        bool IsActive() const { return m_active; }

    };

    typedef FrBEMMode FrBEMForceMode;
    typedef FrBEMMode FrBEMMotionMode;


    class FrDOFMask {

    private:
        unsigned int m_nbDOF;
        std::vector<int> m_listDOF;
        mathutils::Vector6d<bool> m_mask;
        mathutils::MatrixMN<double> m_matrix;

    public:
        FrDOFMask() {}

        void SetMask(mathutils::Vector6d<int> mask);

        mathutils::Vector6d<bool> GetMask() const;

        bool GetMask(unsigned int imode) const { return m_mask[imode]; }

        mathutils::MatrixMN<double> GetMatrix() const;

        unsigned int GetNbMode() const { return m_nbDOF; }

        std::vector<int> GetListDOF() const { return m_listDOF; }
    };


    struct FrWaveDriftPolarData {
        std::vector<double> m_angles;
        std::vector<double> m_freqs;
        std::vector<double> m_data;
        std::unique_ptr<mathutils::LookupTable2d<double>> m_table;

        FrWaveDriftPolarData();

        void SetAngles(const std::vector<double>& angles);

        void SetFrequencies(const std::vector<double>& freqs);

        void AddData(std::string& name, std::vector<double> coeffs);

        double Eval(const std::string name, double x, double y) const;

        bool HasSurge() const;

        bool HasSway() const;

        bool HasHeave() const;

        bool HasPitch() const;

        bool HasRoll() const;

        bool HasYaw() const;
    };

    /**
     * \class FrBEMBody
     * \brief Class for defining a body subject to hydrodynamic loads using a BEM solver.
     */
    class FrBEMBody {

    private:

        FrHydroDB* m_HDB;
        unsigned int m_id;
        std::string m_name;
        Position m_position;

        FrDOFMask m_forceMask;
        FrDOFMask m_motionMask;

        std::vector<FrBEMForceMode> m_forceModes;
        std::vector<FrBEMMotionMode> m_motionModes;

        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> m_excitationMask;
        std::vector<Eigen::MatrixXcd> m_excitation;
        std::vector<Eigen::MatrixXcd> m_froudeKrylov;
        std::vector<Eigen::MatrixXcd> m_diffraction;

        std::vector<Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> m_radiationMask;
        std::unordered_map<FrBEMBody*, mathutils::Matrix66<double>> m_infiniteAddedMass;
        std::unordered_map<FrBEMBody*, std::vector< std::shared_ptr<mathutils::Interp1d<double, mathutils::Vector6d<double>>> >> m_interpK;
        std::unordered_map<FrBEMBody*, std::vector< std::shared_ptr<mathutils::Interp1d<double, mathutils::Vector6d<double>>> >> m_interpKu;

        std::vector<std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators;

        std::shared_ptr<FrWaveDriftPolarData> m_waveDrift;

        mathutils::Matrix33<double> m_hydrostaticStiffnessMatrix;

    public:
        FrBEMBody(unsigned int id, std::string name,  FrHydroDB* HDB)
                : m_id(id), m_name(name), m_HDB(HDB) { }

        std::string GetName() const { return m_name; }

        unsigned int GetID() const { return m_id; }

        unsigned int GetNbFrequencies() const;

        std::vector<double> GetFrequencies() const;

        unsigned int GetNbWaveDirections() const;

        std::vector<double> GetWaveDirections(mathutils::ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const;

        unsigned int GetNbBodies() const;

        unsigned int GetNbTimeSamples() const;

        /// This subroutine allocates the arrays for the hdb.
        void Initialize();

        /// This subroutine runs the interpolators.
        void Finalize();

        //
        // Mask
        //

        void SetForceMask(mathutils::Vector6d<int> mask);

        void SetMotionMask(mathutils::Vector6d<int> mask);

        mathutils::MatrixMN<double> GetForceMaskMatrix() const { return m_forceMask.GetMatrix(); }

        mathutils::MatrixMN<double> GetMotionMaskMatrix() const { return m_motionMask.GetMatrix(); }

        unsigned int GetNbForceMode() const;

        unsigned int GetNbMotionMode() const;

        bool GetMotionMask(unsigned int imotion) const { return m_motionMask.GetMask(imotion); }

        bool GetForceMask(unsigned int iforce) const { return m_forceMask.GetMask(iforce); }

        std::vector<int> GetListDOF() const { return m_motionMask.GetListDOF(); }

        //
        // Generalized modes
        //

        //unsigned int GetNbForceMode() const { return (uint)m_forceModes.size(); }

        //unsigned int GetNbMotionMode() const { return (uint)m_motionModes.size(); }

        FrBEMForceMode* GetForceMode(unsigned int imode);

        FrBEMMotionMode* GetMotionMode(unsigned int imode);

        void AddForceMode(FrBEMForceMode& mode);

        void AddMotionMode(FrBEMMotionMode& mode);

        //
        // Setters
        //

        void SetPosition(Position position) { m_position = position; }

        void SetDiffraction(unsigned int iangle, const Eigen::MatrixXcd& diffractionMatrix);

        void SetFroudeKrylov(unsigned int iangle, const Eigen::MatrixXcd& froudeKrylovMatrix);

        void SetExcitation(unsigned int iangle, const Eigen::MatrixXcd& excitationMatrix);

        /// This subroutine computes the excitation loads from the diffraction loads and the Froude-Krylov loads.
        void ComputeExcitation();

        void SetInfiniteAddedMass(FrBEMBody* BEMBodyMotion, const Eigen::MatrixXd& CMInf);

        void SetImpulseResponseFunctionK(FrBEMBody* BEMBodyMotion, const std::vector<Eigen::MatrixXd>& listIRF);

        void SetImpulseResponseFunctionKu(FrBEMBody* BEMBodyMotion, const std::vector<Eigen::MatrixXd>& listIRF);

        void SetStiffnessMatrix(const mathutils::Matrix33<double>& hydrostaticStiffnessMatrix);

        void SetStiffnessMatrix(const mathutils::Matrix66<double>& hydrostaticStiffnessMatrix);

        void SetWaveDrift();
        //
        // Getters
        //

        Eigen::MatrixXcd GetDiffraction(unsigned int iangle) const;

        Eigen::VectorXcd GetDiffraction(unsigned int iangle, unsigned int iforce) const;

        Eigen::MatrixXcd GetFroudeKrylov(unsigned int iangle) const;

        Eigen::VectorXcd GetFroudeKrylov(unsigned int iangle, unsigned int iforce) const;

        Eigen::MatrixXcd GetExcitation(unsigned int iangle) const;

        Eigen::VectorXcd GetExcitation(unsigned int iangle, unsigned int iforce) const;

        mathutils::Matrix66<double> GetInfiniteAddedMass(FrBEMBody* BEMBodyMotion) const;

        mathutils::Matrix66<double> GetSelfInfiniteAddedMass();

        mathutils::Interp1d<double, mathutils::Vector6d<double>>* GetIRFInterpolatorK(FrBEMBody* BEMBodyMotion, unsigned int idof);

        mathutils::Interp1d<double, mathutils::Vector6d<double>>* GetIRFInterpolatorKu(FrBEMBody* BEMBodyMotion, unsigned int idof);

        mathutils::Matrix33<double> GetHydrostaticStiffnessMatrix() const { return m_hydrostaticStiffnessMatrix; }

        std::shared_ptr<FrWaveDriftPolarData> GetWaveDrift() const;
        //
        // Interpolators
        //

        /// This function interpolates the excitation loads with respect to the wave direction.
        void BuildWaveExcitationInterpolators();

        std::vector<Eigen::MatrixXcd> GetExcitationInterp(std::vector<double> waveFrequencies,
                                                          std::vector<double> waveDirections,
                                                          mathutils::ANGLE_UNIT angleUnit);

        /// This function interpolates the diffraction loads with respect to the wave direction.
        void BuildDiffractionInterpolators();

    };

}  // end namespace frydom


#endif //FRYDOM_FRBEMBODY_H
