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


#ifndef FRYDOM_FRSPRINGDAMPINGFORCE_H
#define FRYDOM_FRSPRINGDAMPINGFORCE_H

#include "chrono/core/ChFrameMoving.h"
#include "frydom/core/force/FrForce.h"

namespace frydom {

    // Forward declaration
    class FrNode;

    /**
     * \class FrSpringDampingForce
     * \brief Class for computing spring-damper loads.
     */
    class FrSpringDampingForce : public FrForce {

    protected:
        double m_w0;
        double m_psi;
        double m_damping;           ///< Damping of the force
        double m_stiffness;         ///< Spring stiffness
        chrono::ChFrameMoving<>* m_node;             ///< Node to which the force is attached

    public:
        /// Default constructor
        FrSpringDampingForce(chrono::ChFrameMoving<>* ref_node, const double T0, const double psi);

        /// Definition of the reference node to follow
        void SetNode(chrono::ChFrameMoving<>* ref_node) { m_node = ref_node; }

        /// Definition of the damping
        void SetDamping(const double damp) { m_damping = damp; }

        /// Return the damping of the force
        double GetDamping() const { return m_damping; }

        /// Definition of the stiffness
        void SetStiffness(const double stiff) { m_stiffness = stiff; }

        /// Return the stiffness of the force
        double GetStiffness() const { return m_stiffness; }

        /// Definition of the spring/damping parameters
        void SetParam(const double T0, const double psi);

        /// Initialize parameters
        void Initialize() override;

        /// Update force
        void UpdateState() override;

    };

}

#endif //FRYDOM_FRSPRINGDAMPINGFORCE_H
