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


#include "MathUtils/MathUtils.h"
#include "chrono/core/ChVectorDynamic.h"

/// <<<<<<<<<<<<<<<<<<<< ADDITIONAL INCLUDE

#include "frydom/core/math/FrVector.h"

using namespace mathutils;

namespace frydom {

//    /**
//     * \class FrDiscretization1D
//     * \brief Class for the linear discretization (frequency and angle).
//     */
//    class FrDiscretization1D {
//    private:
//        double m_xmin = 0.;
//        double m_xmax = 0.;
//        unsigned int m_nx = 0;
//
//    public:
//
//        FrDiscretization1D() = default;
//
//        FrDiscretization1D(double xmin, double xmax, unsigned int nx)
//                : m_xmin(xmin), m_xmax(xmax), m_nx(nx) {}
//
//        double GetMin() const { return m_xmin; }
//        void SetMin(double xmin) { m_xmin = xmin; }
//
//        double GetMax() const { return m_xmax; }
//        void SetMax(double xmax) { m_xmax = xmax; }
//
//        unsigned int GetNbSample() const { return m_nx; }
//        void SetNbSample(unsigned int nx) { m_nx = nx; }
//
//        std::vector<double> GetVector() const;
//
//        void SetStep(double delta) {
//            m_nx = 1 + (unsigned int)((m_xmax - m_xmin) / delta);
//        }
//
//        double GetStep() const {
//            return (m_xmax-m_xmin) / double(m_nx-1);
//        }
//
//    };
//
//    /**
//     * \class FrBEMMode
//     * \brief
//     */
//    class FrBEMMode {
//    public:
//        enum TYPE {
//            LINEAR,
//            ANGULAR
//        };
//
//    private:
//        TYPE m_type;
//        Eigen::Vector3d m_direction;
//        Eigen::Vector3d m_point;
//
//        bool m_active = true;
//
//    public:
//        FrBEMMode() = default;
//
//        void SetTypeLINEAR() { m_type = LINEAR; }
//        void SetTypeANGULAR() { m_type = ANGULAR; }
//
//        TYPE GetType() const { return m_type; }
//
//        void SetDirection(Eigen::Vector3d& direction) { m_direction = direction; }
//        Eigen::Vector3d GetDirection() const { return m_direction; }
//
//        void SetPoint(Eigen::Vector3d& point) { m_point = point; }
//        Eigen::Vector3d GetPoint() const {return m_point; }
//
//        // TODO: Ces fonctions sont la pour permettre lors du linkage des corps BEM avec les corps hydro
//        // de frydom de supprimer les couplages lorsque par exemple on impose une liaison a un corps
//        void Activate() { m_active = true; }
//        void Deactivate() { m_active = false; }
//        bool IsActive() const { return m_active; }
//
//    };
//
//
//    typedef FrBEMMode FrBEMForceMode; /// Force modes
//    typedef FrBEMMode FrBEMMotionMode; /// Motion modes
//
//
//    class FrHydroDB;
//    class FrHydroBody;
//
//    /**
//     * \class FrBEMBody
//     * \brief Class for defining a body subject to hydrodynamic loads using a BEM solver.
//     */
//    class FrBEMBody {
//
//        // =================================================================================================================
//
//    private:
//        FrHydroBody* m_hydroBody = nullptr;  // TODO: est-ce qu'on utilise un pointeur vers un hydrobody ou bien une bimap dans la HDB ??
//
//        FrHydroDB* m_HDB = nullptr;
//
//        unsigned int m_ID;
//        std::string m_BodyName;
//        Eigen::Vector3d m_BodyPosition;
//
////        std::shared_ptr<FrMesh> m_HydrodynamicMesh;
//
//        std::vector<FrBEMForceMode> m_ForceModes;
//        std::vector<FrBEMMotionMode> m_MotionModes;
//
//        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> m_ExcitationMask;
//        std::vector<Eigen::MatrixXcd> m_Diffraction;
//        std::vector<Eigen::MatrixXcd> m_FroudeKrylov;
//        std::vector<Eigen::MatrixXcd> m_Excitation;
//
//        std::vector<Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> m_RadiationMask;
//        std::vector<Eigen::MatrixXd> m_InfiniteAddedMass;
//        std::vector<std::vector<Eigen::MatrixXd>> m_AddedMass;
//        std::vector<std::vector<Eigen::MatrixXd>> m_RadiationDamping;
//        std::vector<std::vector<Eigen::MatrixXd>> m_ImpulseResponseFunction;
//        std::vector<std::vector<Eigen::MatrixXd>> m_SpeedDependentIRF;
//
//        std::vector<std::vector<Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators;
//
//        bool m_radiation_active;
//
//    public:
//        FrBEMBody() = default;
//        FrBEMBody(unsigned int ID, std::string& BodyName) : m_ID(ID), m_BodyName(BodyName) {}
//
//        void SetHDB(FrHydroDB* HDB) { m_HDB = HDB; }
//
//        FrHydroDB* GetHDB() const { return m_HDB; }
//
//        void SetHydroBody(FrHydroBody* hydroBody) { m_hydroBody = hydroBody; }  // TODO: Ne serait-il pas mieux que ce soit la HDB qui gere les relations BEMBody/HydroBody ???
//        FrHydroBody* GetHydroBody() { return m_hydroBody; }
//
//        void SetName(const std::string& BodyName) { m_BodyName = BodyName; }
//        void SetBodyPosition(const Eigen::Vector3d& BodyPosition) { m_BodyPosition = BodyPosition; }
//
//        unsigned int GetNbForceMode() const { return (uint)m_ForceModes.size(); }
//
//        unsigned int GetNbMotionMode() const { return (uint)m_MotionModes.size(); }
//
//
//        FrBEMForceMode* GetForceMode(unsigned int imode) {
//            assert(imode < GetNbForceMode());
//            return &m_ForceModes[imode];
//        }
//
//        FrBEMMotionMode* GetMotionMode(unsigned int imode) {
//            assert(imode < GetNbMotionMode());
//            return &m_MotionModes[imode];
//        }
//
//        unsigned int GetID() const { return m_ID; }
//
//        void AddForceMode(FrBEMForceMode& mode) {
//            m_ForceModes.push_back(mode);
//        }
//
//        void AddMotionMode(FrBEMMotionMode& mode) {
//            m_MotionModes.push_back(mode);
//        }
//
//        unsigned int GetNbFrequencies() const;
//
//        std::vector<double> GetFrequencies() const;
//
//        std::vector<std::vector<double>>
//        GetEncounterFrequencies(std::vector<double> waveFrequencies,
//                                std::vector<double> waveDirections,
//                                std::vector<double> waveNumbers,
//                                chrono::ChVector<double> frame_velocity,
//                                ANGLE_UNIT angleUnit);
//
//        unsigned int GetNbWaveDirections() const;
//
//        std::vector<double> GetWaveDirections() const;
//
//        void FilterRadiation();
//
//        void FilterExcitation();
//
//        void Initialize();
//
//        void Finalize() {
//            ComputeExcitation();
//            FilterExcitation();
//            FilterRadiation();
//
//            // TODO: Ici, on construit les interpolateurs
//            BuildInterpolators();
//        }
//
//        void BuildInterpolators();
//
//        void BuildWaveExcitationInterpolators();  // TODO: voir si on construit les interpolateurs pour la diffraction et Froude-Krylov
//
//        std::vector<Eigen::MatrixXcd>
//        GetExcitationInterp(std::vector<double> waveFrequencies, std::vector<double> waveDirections, ANGLE_UNIT angleUnit=DEG);
//
//        std::vector<Eigen::MatrixXcd>
//        GetExcitationInterp(std::vector<double> waveFrequencies, std::vector<double> waveDirection,
//                            std::vector<double> waveNumbers, chrono::ChVector<double> frame_velocity,
//                            ANGLE_UNIT angleUnit=DEG);
//
//        void SetDiffraction(unsigned int iangle, const Eigen::MatrixXcd& diffractionMatrix);
//
//        void SetFroudeKrylov(unsigned int iangle, const Eigen::MatrixXcd& froudeKrylovMatrix);
//
//        void ComputeExcitation();
//
//        void SetInfiniteAddedMass(unsigned int ibody, const Eigen::MatrixXd& CMInf);
//
//        void SetAddedMass(unsigned int ibody, unsigned int idof, const Eigen::MatrixXd& CM);
//
//        void SetRadiationDamping(unsigned int ibody, unsigned int idof, const Eigen::MatrixXd& CA);
//
//        void SetImpulseResponseFunction(unsigned int ibody, unsigned int idof, const Eigen::MatrixXd& IRF);
//
//        void SetSpeedDependentIRF(const unsigned int ibody, const unsigned int idof, const Eigen::MatrixXd& IRF);
//
//        // FIXME: les GetDiffraction etc ne doivent pas specialement etre accessible en dehors des interpolations...
//        // On utilisera donc plutot GetExcitation(omega, angles) ...
//
//        Eigen::MatrixXcd GetDiffraction(unsigned int iangle) const;
//
//        Eigen::VectorXcd GetDiffraction(unsigned int iangle, unsigned int iforce) const;
//
//        Eigen::MatrixXcd GetFroudeKrylov(unsigned int iangle) const;
//
//        Eigen::VectorXcd GetFroudeKrylov(unsigned int iangle, unsigned int iforce) const;
//
//        Eigen::MatrixXcd GetExcitation(unsigned int iangle) const;
//
//        Eigen::VectorXcd GetExcitation(unsigned int iangle, unsigned int iforce) const;
//
//        Eigen::MatrixXd GetInfiniteAddedMass(unsigned int ibody) const;
//
//        Eigen::MatrixXd GetSelfInfiniteAddedMass() const;
//
//        Eigen::MatrixXd GetAddedMass(unsigned int ibody, unsigned int idof) const;
//
//        Eigen::VectorXd GetAddedMass(unsigned int ibody, unsigned int idof, unsigned int iforce) const;
//
//        Eigen::MatrixXd GetSelfAddedMass(unsigned int idof) const;
//
//        Eigen::VectorXd GetSelfAddedMass(unsigned int idof, unsigned int iforce) const;
//
//        Eigen::MatrixXd GetRadiationDamping(unsigned int ibody, unsigned int idof) const;
//
//        Eigen::VectorXd GetRadiationDamping(unsigned int ibody, unsigned int idof, unsigned int iforce) const;
//
//        Eigen::MatrixXd GetSelfRadiationDamping(unsigned int idof) const;
//
//        Eigen::VectorXd GetselfRadiationDamping(unsigned int idof, unsigned int iforce) const;
//
//        std::vector<Eigen::MatrixXd> GetImpulseResponseFunction(unsigned int ibody) const;
//
//        Eigen::MatrixXd GetImpulseResponseFunction(unsigned int ibody, unsigned int idof) const;
//
//        Eigen::VectorXd GetImpulseResponseFunction(unsigned int ibody, unsigned int idof, unsigned int iforce) const;
//
//        Eigen::MatrixXd GetSelfImpulseResponseFunction(unsigned int idof) const;
//
//        Eigen::VectorXd GetSelfImpulseResponseFunction(unsigned int idof, unsigned int iforce) const;
//
//        std::vector<Eigen::MatrixXd> GetSpeedDependentIRF(unsigned int ibody) const;
//
//        Eigen::MatrixXd GetSpeedDependentIRF(unsigned int ibody, unsigned int idof) const;
//
//        Eigen::VectorXd GetSpeedDependentIRF(unsigned int ibody, unsigned int idof, unsigned int iforce) const;
//
//        void GenerateImpulseResponseFunctions();
//
//        void GenerateSpeedDependentIRF();
//
//        virtual void IntLoadResidual_Mv(const unsigned int off,
//                                        chrono::ChVectorDynamic<>& R,
//                                        const chrono::ChVectorDynamic<>& w,
//                                        const double c);
//
//        void SetRadiationActive(const bool is_active) { m_radiation_active = is_active; }
//
//        bool GetRadiationActive() const { return m_radiation_active; }
//
//        //void VariablesFbIncrementMq();
//
//        //void Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const;
//
//        void SetBEMVariables();
//    };
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

    class FrHydroDB_;

    /**
     * \class FrBEMBody_
     * \brief
     */
    class FrBEMMode_ {

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
        FrBEMMode_() = default;

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

    typedef FrBEMMode_ FrBEMForceMode_;
    typedef FrBEMMode_ FrBEMMotionMode_;


    struct FrWaveDriftPolarData {
        std::vector<double> m_angles;
        std::vector<double> m_freqs;
        std::vector<double> m_data;
        std::unique_ptr<mathutils::LookupTable2d<>> m_table;

        FrWaveDriftPolarData();

        void SetAngles(const std::vector<double>& angles);

        void SetFrequencies(const std::vector<double>& freqs);

        void AddData(std::string& name, std::vector<double> coeffs);

        double Eval(const std::string name, double x, double y) const;

        bool HasSurge() const { return m_table->HasSerie("surge"); }

        bool HasSway() const { return m_table->HasSerie("sway"); }

        bool HasHeave() const { return m_table->HasSerie("heave"); }

        bool HasPitch() const { return m_table->HasSerie("pitch"); }

        bool HasRoll() const { return m_table->HasSerie("roll"); }

        bool HasYaw() const { return m_table->HasSerie("yaw"); }
    };

    /**
     * \class FrBEMBody_
     * \brief Class for defining a body subject to hydrodynamic loads using a BEM solver.
     */
    class FrBEMBody_ {

    private:

        FrHydroDB_* m_HDB;
        unsigned int m_id;
        std::string m_name;
        Position m_position;

        std::vector<FrBEMForceMode_> m_forceModes;
        std::vector<FrBEMMotionMode_> m_motionModes;

        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> m_excitationMask;
        std::vector<Eigen::MatrixXcd> m_excitation;
        std::vector<Eigen::MatrixXcd> m_froudeKrylov;
        std::vector<Eigen::MatrixXcd> m_diffraction;

        std::vector<Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> m_radiationMask;
        std::unordered_map<FrBEMBody_*, mathutils::Matrix66<double>> m_infiniteAddedMass;
        std::unordered_map<FrBEMBody_*, std::vector< std::shared_ptr<Interp1d<double, Vector6d<double>>> >> m_interpK;
        std::unordered_map<FrBEMBody_*, std::vector< std::shared_ptr<Interp1d<double, Vector6d<double>>> >> m_interpKu;

        std::vector<std::vector<Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators;

        std::shared_ptr<FrWaveDriftPolarData> m_waveDrift;

        mathutils::Matrix33<double> m_hydrostaticStiffnessMatrix;

    public:
        FrBEMBody_(unsigned int id, std::string name,  FrHydroDB_* HDB)
                : m_id(id), m_name(name), m_HDB(HDB) { }

        std::string GetName() const { return m_name; }

        unsigned int GetID() const { return m_id; }

        unsigned int GetNbFrequencies() const;

        std::vector<double> GetFrequencies() const;

        unsigned int GetNbWaveDirections() const;

        std::vector<double> GetWaveDirections(ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const;

        unsigned int GetNbBodies() const;

        unsigned int GetNbTimeSamples() const;

        /// This subroutine allocates the arrays for the hdb.
        void Initialize();

        /// This subroutine runs the interpolators.
        void Finalize();

        //
        // Generalized modes
        //

        unsigned int GetNbForceMode() const { return (uint)m_forceModes.size(); }

        unsigned int GetNbMotionMode() const { return (uint)m_motionModes.size(); }

        FrBEMForceMode_* GetForceMode(unsigned int imode);

        FrBEMMotionMode_* GetMotionMode(unsigned int imode);

        void AddForceMode(FrBEMForceMode_& mode);

        void AddMotionMode(FrBEMMotionMode_& mode);

        //
        // Setters
        //

        void SetPosition(Position position) { m_position = position; }

        void SetDiffraction(unsigned int iangle, const Eigen::MatrixXcd& diffractionMatrix);

        void SetFroudeKrylov(unsigned int iangle, const Eigen::MatrixXcd& froudeKrylovMatrix);

        void SetExcitation(unsigned int iangle, const Eigen::MatrixXcd& excitationMatrix);

        /// This subroutine computes the excitation loads from the diffraction loads and the Froude-Krylov loads.
        void ComputeExcitation();

        void SetInfiniteAddedMass(FrBEMBody_* BEMBodyMotion, const Eigen::MatrixXd& CMInf);

        void SetImpulseResponseFunctionK(FrBEMBody_* BEMBodyMotion, const std::vector<Eigen::MatrixXd>& listIRF);

        void SetImpulseResponseFunctionKu(FrBEMBody_* BEMBodyMotion, const std::vector<Eigen::MatrixXd>& listIRF);

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

        mathutils::Matrix66<double> GetInfiniteAddedMass(FrBEMBody_* BEMBodyMotion) const;

        mathutils::Matrix66<double> GetSelfInfiniteAddedMass();

        Interp1d<double, Vector6d<double>>* GetIRFInterpolatorK(FrBEMBody_* BEMBodyMotion, unsigned int idof);

        Interp1d<double, Vector6d<double>>* GetIRFInterpolatorKu(FrBEMBody_* BEMBodyMotion, unsigned int idof);

        mathutils::Matrix33<double> GetHydrostaticStiffnessMatrix() const { return m_hydrostaticStiffnessMatrix; }

        std::shared_ptr<FrWaveDriftPolarData> GetWaveDrift() const;
        //
        // Interpolators
        //

        /// This subroutine interpolates the excitation loads with respect to the wave direction.
        void BuildWaveExcitationInterpolators();

        std::vector<Eigen::MatrixXcd> GetExcitationInterp(std::vector<double> waveFrequencies,
                                                          std::vector<double> waveDirections,
                                                          ANGLE_UNIT angleUnit);
    };

}  // end namespace frydom


#endif //FRYDOM_FRBEMBODY_H
