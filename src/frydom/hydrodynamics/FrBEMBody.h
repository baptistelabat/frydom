//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRBEMBODY_H
#define FRYDOM_FRBEMBODY_H


#include "MathUtils.h"

using namespace mathutils;

namespace frydom {

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

        void SetStep(double delta) {
            m_nx = 1 + (unsigned int)((m_xmax - m_xmin) / delta);
        }

        double GetStep() const {
            return (m_xmax-m_xmin) / double(m_nx-1);
        }

    };


    class FrBEMMode {
    public:
        enum TYPE {
            LINEAR,
            ANGULAR
        };

    private:
        TYPE m_type;
        Eigen::Vector3d m_direction;
        Eigen::Vector3d m_point;

        bool m_active = true;

    public:
        FrBEMMode() = default;

        void SetTypeLINEAR() { m_type = LINEAR; }
        void SetTypeANGULAR() { m_type = ANGULAR; }

        TYPE GetType() const { return m_type; }

        void SetDirection(Eigen::Vector3d& direction) { m_direction = direction; }
        Eigen::Vector3d GetDirection() const { return m_direction; }

        void SetPoint(Eigen::Vector3d& point) { m_point = point; }
        Eigen::Vector3d GetPoint() const {return m_point; }

        // TODO: Ces fonctions sont la pour permettre lors du linkage des corps BEM avec les corps hydro
        // de frydom de supprimer les couplages lorsque par exemple on impose une liaison a un corps
        void Activate() { m_active = true; }
        void Deactivate() { m_active = false; }
        bool IsActive() const { return m_active; }

    };


    typedef FrBEMMode FrBEMForceMode; /// Force modes
    typedef FrBEMMode FrBEMMotionMode; /// Motion modes


    class FrHydroDB;
    class FrHydroBody;

    class FrBEMBody {

        // =================================================================================================================

    private:
        FrHydroBody* m_hydroBody = nullptr;  // TODO: est-ce qu'on utilise un pointeur vers un hydrobody ou bien une bimap dans la HDB ??

        FrHydroDB* m_HDB = nullptr;

        unsigned int m_ID;
        std::string m_BodyName;
        Eigen::Vector3d m_BodyPosition;

//        std::shared_ptr<FrMesh> m_HydrodynamicMesh;

        std::vector<FrBEMForceMode> m_ForceModes;
        std::vector<FrBEMMotionMode> m_MotionModes;

        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> m_ExcitationMask;
        std::vector<Eigen::MatrixXcd> m_Diffraction;
        std::vector<Eigen::MatrixXcd> m_FroudeKrylov;
        std::vector<Eigen::MatrixXcd> m_Excitation;

        std::vector<Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> m_RadiationMask;
        std::vector<Eigen::MatrixXd> m_InfiniteAddedMass;
        std::vector<std::vector<Eigen::MatrixXd>> m_AddedMass;
        std::vector<std::vector<Eigen::MatrixXd>> m_RadiationDamping;
        std::vector<std::vector<Eigen::MatrixXd>> m_ImpulseResponseFunction;

        std::vector<std::vector<Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators;

    public:
        FrBEMBody() = default;
        FrBEMBody(unsigned int ID, std::string& BodyName) : m_ID(ID), m_BodyName(BodyName) {}

        void SetHDB(FrHydroDB* HDB) { m_HDB = HDB; }

        FrHydroDB* GetHDB() const { return m_HDB; }

        void SetHydroBody(FrHydroBody* hydroBody) { m_hydroBody = hydroBody; }  // TODO: Ne serait-il pas mieux que ce soit la HDB qui gere les relations BEMBody/HydroBody ???

        void SetName(const std::string& BodyName) { m_BodyName = BodyName; }
        void SetBodyPosition(const Eigen::Vector3d& BodyPosition) { m_BodyPosition = BodyPosition; }

        unsigned int GetNbForceMode() const { return (uint)m_ForceModes.size(); }

        unsigned int GetNbMotionMode() const { return (uint)m_MotionModes.size(); }


        FrBEMForceMode* GetForceMode(unsigned int imode) {
            assert(imode < GetNbForceMode());
            return &m_ForceModes[imode];
        }

        FrBEMMotionMode* GetMotionMode(unsigned int imode) {
            assert(imode < GetNbMotionMode());
            return &m_MotionModes[imode];
        }

        unsigned int GetID() const { return m_ID; }

        void AddForceMode(FrBEMForceMode& mode) {
            m_ForceModes.push_back(mode);
        }

        void AddMotionMode(FrBEMMotionMode& mode) {
            m_MotionModes.push_back(mode);
        }

        unsigned int GetNbFrequencies() const;

        std::vector<double> GetFrequencies() const;

        unsigned int GetNbWaveDirections() const;

        std::vector<double> GetWaveDirections() const;

        void FilterRadiation();

        void FilterExcitation();

        void Initialize();

        void Finalize() {
            ComputeExcitation();
            FilterExcitation();
            FilterRadiation();

            // TODO: Ici, on construit les interpolateurs
            BuildInterpolators();
        }

        void BuildInterpolators();

        void BuildWaveExcitationInterpolators();  // TODO: voir si on construit les interpolateurs pour la diffraction et Froude-Krylov

        std::vector<Eigen::MatrixXcd>
        GetExcitationInterp(std::vector<double> waveFrequencies, std::vector<double> waveDirections, ANGLE_UNIT angleUnit=DEG);

        void SetDiffraction(unsigned int iangle, const Eigen::MatrixXcd& diffractionMatrix);

        void SetFroudeKrylov(unsigned int iangle, const Eigen::MatrixXcd& froudeKrylovMatrix);

        void ComputeExcitation();

        void SetInfiniteAddedMass(unsigned int ibody, const Eigen::MatrixXd& CMInf);

        void SetAddedMass(unsigned int ibody, unsigned int idof, const Eigen::MatrixXd& CM);

        void SetRadiationDamping(unsigned int ibody, unsigned int idof, const Eigen::MatrixXd& CA);

        void SetImpulseResponseFunction(unsigned int ibody, unsigned int idof, const Eigen::MatrixXd& IRF);

        // FIXME: les GetDiffraction etc ne doivent pas specialement etre accessible en dehors des interpolations...
        // On utilisera donc plutot GetExcitation(omega, angles) ...

        Eigen::MatrixXcd GetDiffraction(unsigned int iangle) const;

        Eigen::VectorXcd GetDiffraction(unsigned int iangle, unsigned int iforce) const;

        Eigen::MatrixXcd GetFroudeKrylov(unsigned int iangle) const;

        Eigen::VectorXcd GetFroudeKrylov(unsigned int iangle, unsigned int iforce) const;

        Eigen::MatrixXcd GetExcitation(unsigned int iangle) const;

        Eigen::VectorXcd GetExcitation(unsigned int iangle, unsigned int iforce) const;

        Eigen::MatrixXd GetInfiniteAddedMass(unsigned int ibody) const;

        Eigen::MatrixXd GetSelfInfiniteAddedMass() const;

        Eigen::MatrixXd GetAddedMass(unsigned int ibody, unsigned int idof) const;

        Eigen::VectorXd GetAddedMass(unsigned int ibody, unsigned int idof, unsigned int iforce) const;

        Eigen::MatrixXd GetSelfAddedMass(unsigned int idof) const;

        Eigen::VectorXd GetSelfAddedMass(unsigned int idof, unsigned int iforce) const;

        Eigen::MatrixXd GetRadiationDamping(unsigned int ibody, unsigned int idof) const;

        Eigen::VectorXd GetRadiationDamping(unsigned int ibody, unsigned int idof, unsigned int iforce) const;

        Eigen::MatrixXd GetSelfRadiationDamping(unsigned int idof) const;

        Eigen::VectorXd GetselfRadiationDamping(unsigned int idof, unsigned int iforce) const;

        std::vector<Eigen::MatrixXd> GetImpulseResponseFunction(unsigned int ibody) const;

        Eigen::MatrixXd GetImpulseResponseFunction(unsigned int ibody, unsigned int idof) const;

        Eigen::VectorXd GetImpulseResponseFunction(unsigned int ibody, unsigned int idof, unsigned int iforce) const;

        Eigen::MatrixXd GetSelfImpulseResponseFunction(unsigned int idof) const;

        Eigen::VectorXd GetSelfImpulseResponseFunction(unsigned int idof, unsigned int iforce) const;

        void GenerateImpulseResponseFunctions();

    };

}  // end namespace frydom


#endif //FRYDOM_FRBEMBODY_H
