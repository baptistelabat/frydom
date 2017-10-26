//
// Created by frongere on 17/10/17.
//

#ifndef FRYDOM_FRHYDRODB_H
#define FRYDOM_FRHYDRODB_H

#include <vector>
//#include "Eigen/Dense"

#include "frydom/core/FrHydroBody.h"
#include "frydom/misc/FrLinspace.h"
#include "frydom/misc/FrEigen.h"

#define J std::complex<double>(0, 1)

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

        std::vector<double> GetVector() const {
            return linspace<double>(m_xmin, m_xmax, m_nx);
        }

    };

    class FrBEMMode {

        enum TYPE {
            LINEAR,
            ANGULAR
        };

    private:
        TYPE m_type;
        Eigen::Vector3d m_direction;
        Eigen::Vector3d m_point;
        Eigen::Vector2d vec;

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

    class FrBEMBody {

    private:
        FrHydroBody* HydroBody;

        FrHydroDB* m_HDB = nullptr;

        unsigned int m_ID;
        std::string m_BodyName;
        Eigen::Vector3d m_BodyPosition;

//        std::shared_ptr<FrMesh> mesh;

        std::vector<FrBEMForceMode> m_ForceModes;
        std::vector<FrBEMMotionMode> m_MotionModes;

        std::vector<Eigen::MatrixXcd> m_Diffraction;
        std::vector<Eigen::MatrixXcd> m_FroudeKrylov;

        std::vector<Eigen::MatrixXcd> m_Excitation;

        std::vector<Eigen::MatrixXd> m_InfiniteAddedMass;
        std::vector<std::vector<Eigen::MatrixXd>> m_AddedMass;
        std::vector<std::vector<Eigen::MatrixXd>> m_RadiationDamping;
        std::vector<std::vector<Eigen::MatrixXd>> m_ImpulseResponseFunction;

    public:
        FrBEMBody() = default;
        FrBEMBody(unsigned int ID, std::string& BodyName) : m_ID(ID), m_BodyName(BodyName) {}

        void SetName(const std::string& BodyName) { m_BodyName = BodyName; }
        void SetBodyPosition(const Eigen::Vector3d& BodyPosition) { m_BodyPosition = BodyPosition; }

        void SetHDB(FrHydroDB* HDB) { m_HDB = HDB; }

        unsigned int GetNbForceMode() const { return (uint)m_ForceModes.size(); }
        unsigned int GetNbMotionMode() const { return (uint)m_MotionModes.size(); }

        unsigned int GetID() const { return m_ID; }

        void AddForceMode(FrBEMForceMode& mode) {
            m_ForceModes.push_back(mode);
        }
        void AddMotionMode(FrBEMMotionMode& mode) {
            m_MotionModes.push_back(mode);
        }

        void Initialize();

        void Finalize() {
            ComputeExcitation();
        }

        void SetDiffraction(const unsigned int iangle, const Eigen::MatrixXcd& diffractionMatrix) {
            m_Diffraction[iangle] = diffractionMatrix;
        }

        void SetFroudeKrylov(const unsigned int iangle, const Eigen::MatrixXcd& froudeKrylovMatrix) {
            m_FroudeKrylov[iangle] = froudeKrylovMatrix;
        }

        void ComputeExcitation();

        void SetInfiniteAddedMass(const unsigned int ibody, const Eigen::MatrixXd& CMInf) {
            m_InfiniteAddedMass[ibody] = CMInf;
        }

        void SetAddedMass(const unsigned int ibody, const unsigned int idof, const Eigen::MatrixXd& CM) {
            m_AddedMass[ibody][idof] = CM;
        }

        void SetRadiationDamping(const unsigned int ibody, const unsigned int idof, const Eigen::MatrixXd& CA) {
            m_RadiationDamping[ibody][idof] = CA;
        }

        void SetImpulseResponseFunction(const unsigned int ibody, const unsigned int idof, const Eigen::MatrixXd& IRF) {
            m_ImpulseResponseFunction[ibody][idof] = IRF;
        }

        Eigen::MatrixXcd GetDiffraction(const unsigned int iangle) const {
            return m_Diffraction[iangle];
        }

        Eigen::VectorXcd GetDiffraction(const unsigned int iangle, const unsigned iforce) const {
            assert(iforce < GetNbForceMode());
            return m_Diffraction[iangle].row(iforce);
        }

        Eigen::MatrixXcd GetFroudeKrylov(const unsigned int iangle) const {
            return m_FroudeKrylov[iangle];
        }

        Eigen::VectorXcd GetFroudeKrylov(const unsigned int iangle, const unsigned iforce) const {
            assert(iforce < GetNbForceMode());
            return m_FroudeKrylov[iangle].row(iforce);
        }

        Eigen::MatrixXcd GetExcitation(const unsigned int iangle) const {
            return m_Excitation[iangle];
        }

        Eigen::VectorXcd GetExcitation(const unsigned int iangle, const unsigned iforce) const {
            assert(iforce < GetNbForceMode());
            return m_Excitation[iangle].row(iforce);
        }

        Eigen::MatrixXd GetInfiniteAddedMass(const unsigned int ibody) const {
            return m_InfiniteAddedMass[ibody];
        }

        Eigen::MatrixXd GetSelfInfiniteAddedMass() const {
            return m_InfiniteAddedMass[m_ID];
        }

        Eigen::MatrixXd GetAddedMass(const unsigned int ibody, const unsigned int idof) const {
            return m_AddedMass[ibody][idof];
        }

        Eigen::VectorXd GetAddedMass(const unsigned int ibody, const unsigned int idof, const unsigned int iforce) const {
            return m_AddedMass[ibody][idof].row(iforce);
        }

        Eigen::MatrixXd GetSelfAddedMass(const unsigned int idof) const {
            return GetAddedMass(m_ID, idof);
        }

        Eigen::VectorXd GetSelfAddedMass(const unsigned int idof, const unsigned int iforce) const {
            return GetAddedMass(m_ID, idof, iforce);
        }

        Eigen::MatrixXd GetRadiationDamping(const unsigned int ibody, const unsigned int idof) const {
            return m_RadiationDamping[ibody][idof];
        }

        Eigen::VectorXd GetRadiationDamping(const unsigned int ibody, const unsigned int idof, const unsigned int iforce) const {
            return m_RadiationDamping[ibody][idof].row(iforce);
        }

        Eigen::MatrixXd GetSelfRadiationDamping(const unsigned int idof) const {
            return GetAddedMass(m_ID, idof);
        }

        Eigen::VectorXd GetselfRadiationDamping(const unsigned int idof, const unsigned int iforce) const {
            return GetAddedMass(m_ID, idof, iforce);
        }

        Eigen::MatrixXd GetImpulseResponseFunction(const unsigned int ibody, const unsigned int idof) const {
            return m_ImpulseResponseFunction[ibody][idof];
        }

        Eigen::VectorXd GetImpulseResponseFunction(const unsigned int ibody, const unsigned int idof, const unsigned int iforce) const {
            return m_ImpulseResponseFunction[ibody][idof].row(iforce);
        }

        Eigen::MatrixXd GetSelfImpulseResponseFunction(const unsigned int idof) const {
            return GetAddedMass(m_ID, idof);
        }

        Eigen::VectorXd GetSelfImpulseResponseFunction(const unsigned int idof, const unsigned int iforce) const {
            return GetAddedMass(m_ID, idof, iforce);
        }




    };

    class FrHydroDB {
    private:
        double m_GravityAcc;
//        unsigned int m_NbBodies=0;
        double m_NormalizationLength;
        double m_WaterDensity;
        double m_WaterDepth;
        std::vector<std::shared_ptr<FrBEMBody>> m_Bodies;
        FrDiscretization1D m_FrequencyDiscretization;
        FrDiscretization1D m_WaveDirectionDiscretization;
        FrDiscretization1D m_TimeDiscretization;

    public:
        FrHydroDB() {};

        unsigned int GetNbBodies() const { return (unsigned int)m_Bodies.size(); }

        void SetGravityAcc(const double GravityAcc) { m_GravityAcc = GravityAcc; }
        void SetNormalizationLength(const double NormalizationLength) { m_NormalizationLength = NormalizationLength; }
        void SetWaterDensity(const double WaterDensity) { m_WaterDensity = WaterDensity; }
        void SetWaterDepth(const double WaterDepth) { m_WaterDepth = WaterDepth; }

        void SetFrequencyDiscretization(const double MinFreq, const double MaxFreq, const unsigned int NbFreq) {
            m_FrequencyDiscretization.SetMin(MinFreq);
            m_FrequencyDiscretization.SetMax(MaxFreq);
            m_FrequencyDiscretization.SetNbSample(NbFreq);
        }

        void SetWaveDirectionDiscretization(const double MinAngle, const double MaxAngle, const unsigned int NbAngle) {
            m_WaveDirectionDiscretization.SetMin(MinAngle);
            m_WaveDirectionDiscretization.SetMax(MaxAngle);
            m_WaveDirectionDiscretization.SetNbSample(NbAngle);
        }

        void SetTimeDiscretization(const double FinalTime, const unsigned int NbTimeSamples) {
            m_TimeDiscretization.SetMin(0.);
            m_TimeDiscretization.SetMax(FinalTime);
            m_TimeDiscretization.SetNbSample(NbTimeSamples);
        }

        unsigned int GetNbFrequencies() const { return m_FrequencyDiscretization.GetNbSample(); }
        unsigned int GetNbWaveDirections() const { return m_WaveDirectionDiscretization.GetNbSample(); }
        unsigned int GetNbTimeSamples() const { return m_TimeDiscretization.GetNbSample(); }

        std::shared_ptr<FrBEMBody> NewBody(std::string BodyName) {
            auto body = std::make_shared<FrBEMBody>(GetNbBodies(), BodyName);
            body->SetHDB(this);
            m_Bodies.push_back(body);
            return body;
        }

        std::shared_ptr<FrBEMBody> GetBody(unsigned int ibody) { return m_Bodies[ibody]; }


    };








    FrHydroDB LoadHDB5(std::string h5file);



}  // end namespace frydom


#endif //FRYDOM_FRHYDRODB_H
