//
// Created by camille on 05/06/18.
//

#ifndef FRYDOM_FRNODEDYNAMIC_H
#define FRYDOM_FRNODEDYNAMIC_H

#include "frydom/core/FrHydroBody.h"
#include "frydom/core/FrNode.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/core/ChFrameMoving.h"

#include "frydom/hydrodynamics/FrVelocityRecorder.h"
#include "frydom/hydrodynamics/FrPositionRecorder.h"

namespace frydom {

    /// Node affected by a mass. Allow to attached force to a node.
    /// The position and velocity of this node is determined by body motion equation
    /// By default a spring / damping force is used

    class FrNodeDynamic :   public FrHydroBody {

    private:
        std::shared_ptr<FrForce> m_force;

    public:
        /// Default constructor
        FrNodeDynamic();

        /// Define the motion of the node from the mass spring system with damping
        void SetSpringDamping(chrono::ChFrameMoving<>* ref_node,
                        const double T0=60.,
                        const double psi = 0.5);

        /// Define the motion of the node with steady velocity
        void SetSteadyMotion(chrono::ChVector<double> velocity);

    };

    /// Node for which its velocity and motion are equal to the mean value of the body
    /// to which the node is attached

    class FrNodeMeanMotion : public FrHydroBody {

    private:
        chrono::ChFrameMoving<>* m_node = nullptr;              ///<
        FrBody* m_body = nullptr;                               ///< Body to which the node is attached
        std::unique_ptr<FrVelocityRecorder> m_velocitiesREC;    ///< Velocity recorder
        std::unique_ptr<FrPositionRecorder> m_positionsREC;     ///< Position recorder
        unsigned int m_size = 0;                                ///< size of the recorders
        double m_tmax = -1;                                     ///< Time length of the recorder (s)

    public:
        /// Default constructor
        FrNodeMeanMotion();

        /// Define the node to follow
        void SetNodeRef(chrono::ChFrameMoving<>* node) { m_node = node; }

        void AttachedBody(FrBody* body) { m_body = body; }

        /// Set size of the recorders
        void SetSize(const unsigned int size) { m_size = size; }

        /// Return the size of the recorders
        unsigned int GetSize() const { return m_size; }

        /// Set the time length of the recorder
        void SetTmax(const double tmax) { m_tmax = tmax; }

        /// Return the time length of the recorder
        double GetTmax() const { return m_tmax; }

        /// Initialize recorders
        void Initialize() override;

        /// Update position and velocity of the node
        void Update(bool update_asset = true) override;

        void StepFinalize() override {}

    };

    FrNodeMeanMotion::FrNodeMeanMotion() : FrHydroBody() {
        m_velocitiesREC = std::make_unique<FrVelocityRecorder>();
        m_positionsREC = std::make_unique<FrPositionRecorder>();
    }

    void FrNodeMeanMotion::Initialize() {

        auto dt = GetSystem()->GetStep();

        if (m_tmax > DBL_EPSILON && dt > DBL_EPSILON) {
            m_size = uint(m_tmax/dt) + 1;
        } else if (m_size > 0 && m_tmax < DBL_EPSILON) {
            m_tmax = (m_size -1) * dt;
        }

        m_velocitiesREC->SetBody(m_body);
        m_velocitiesREC->SetSize(m_size);
        m_velocitiesREC->Initialize();

        m_positionsREC->SetBody(m_body);
        m_positionsREC->SetSize(m_size);
        m_positionsREC->Initialize();

        FrHydroBody::Initialize();

    }

    void FrNodeMeanMotion::Update(bool update_asset) {

        m_velocitiesREC->RecordVelocity();
        m_positionsREC->RecordPosition();

        auto vx = m_velocitiesREC->GetRecordOnDOF(0);
        auto vy = m_velocitiesREC->GetRecordOnDOF(1);
        auto x = m_positionsREC->GetRecordOnDOF(0);
        auto y = m_positionsREC->GetRecordOnDOF(1);

        //auto mean_vx = std::accumulate(vx.begin(), vx.end(), 0)/vx.size();
        //auto mean_vy = std::accumulate(vy.begin(), vy.end(), 0)/vy.size();
        //auto mean_x = std::accumulate(x.begin(), x.end(), 0)/m_size;
        //auto mean_y = std::accumulate(y.begin(), y.end(), 0)/m_size;

        double mean_vx = 0.;
        double mean_vy = 0.;
        double mean_x = 0.;
        double mean_y = 0.;
        for (unsigned int i=0; i<m_size; ++i) {
            mean_vx += vx[i];
            mean_vy += vy[i];
            mean_x += x[i];
            mean_y += y[i];
        }
        mean_vx = mean_vx/m_size;
        mean_vy = mean_vy/m_size;
        mean_x = mean_x/m_size;
        mean_y = mean_y/m_size;

        SetPos( chrono::ChVector<double>(mean_x, mean_y, 0.));
        SetPos_dt( chrono::ChVector<double>( mean_vx, mean_vy, 0.));

        FrHydroBody::Update(update_asset);
    }


}

#endif //FRYDOM_FRNODEDYNAMIC_H
