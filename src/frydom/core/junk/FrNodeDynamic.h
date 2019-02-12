// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRNODEDYNAMIC_H
#define FRYDOM_FRNODEDYNAMIC_H

#include "frydom/core/junk/FrHydroBody.h"
#include "frydom/core/common/FrNode.h"
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

        /// Constructor with damping string force definition
        FrNodeDynamic(chrono::ChFrameMoving<>* ref_node, double T0, double psi);

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
        unsigned int m_size = 0;                                ///< full size of the recorders
        unsigned int m_nstep = 0;                               ///< number of element stored
        double m_tmax = -1;                                     ///< Time length of the recorder (s)

    public:
        /// Default constructor
        FrNodeMeanMotion();

        FrNodeMeanMotion(FrBody* body, double tmax);

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

        //void StepFinalize() override {}

    };


}

#endif //FRYDOM_FRNODEDYNAMIC_H
