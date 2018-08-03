//
// Created by frongere on 28/07/17.
//

#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H

#include "chrono/physics/ChMarker.h"
#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "FrCatenaryForce.h"
#include "frydom/core/FrNode.h"
#include "FrCable.h"


// TODO: prevoir une discretisation automatique pour laquelle on precise la taille cible d'un element
// Servira dans la visu et pour l'application de forces comme Morrison.


namespace frydom {


    class FrCatenaryLine : public FrCable,
                           public chrono::ChLink {

    private:

        bool m_elastic = true;

        chrono::ChVector<double> m_t0;

        double m_q;
        chrono::ChVector<double> m_u;
        chrono::ChVector<double> c_qvec;

        chrono::ChMatrix33<double> c_Umat;

        // Forces to apply to bodies
        std::shared_ptr<FrCatenaryForce> m_startingForce;
        std::shared_ptr<FrCatenaryForce> m_endingForce;

        // Data for Newton-Raphson solver
        const double Lmin            = 1e-10;
        double m_tolerance     = 1e-6;
        unsigned int m_itermax = 100;
        double m_relax           = 0.1;

        bool m_drawCableElements = true;
        int m_nbDrawnElements = 21;
        std::vector<std::shared_ptr<chrono::ChLineShape>> m_cableElements;

    public:

        FrCatenaryLine(const std::shared_ptr<FrNode>& startingNode,
                       const std::shared_ptr<FrNode>& endingNode,
                       bool elastic,
                       const double youngModulus,
                       const double sectionArea,
                       const double cableLength,
                       const double q,  // TODO: pour q, lier a la densite du cable !!!
                       const chrono::ChVector<double> u
        );

        // TODO: avoir une methode pour detacher d'un noeud ou d'un corps. Dans ce cas, un nouveau oeud fixe est cree a
        // la position courante de la ligne.

        /// Get the starting force of the line
        std::shared_ptr<FrCatenaryForce> GetStartingForce() { return m_startingForce; }

        /// Get the ending force of the line
        std::shared_ptr<FrCatenaryForce> GetEndingForce() { return m_endingForce; }

        /// Get the current absolute position of the line's starting node
        chrono::ChVector<> GetPosStartingNode() const { return m_startingNode->GetAbsPos(); }

        /// Get the current absolute position of the line's ending node
        chrono::ChVector<> GetPosEndingNode() const { return m_endingNode->GetAbsPos(); }

        /// Guess the line tension from line boundary positions
        /// Used to initialize the Newton-Raphson algorithm (see solve method) TODO: mettre un see a facon doxygen...
        void guess_tension();

        // TODO: accessors pour le champ de force distribue

        chrono::ChVector<double> GetTension(const double s) const;

        /// Returns the cartesian tension at the start of the line.
        /// This tension is applied by the line on its node
        chrono::ChVector<double> getStartingNodeTension() const { return m_t0; }

        /// Returns the cartensian tension at the end of the line.
        /// This tension is applied by the end node's parent body ON the line
        chrono::ChVector<double> GetEndingNodeTension() const;

        // TODO: supprimer a terme cette methode et coder en dur a chaque fois qu'on en a besoin
        double _rho(const double s) const {
            // FIXME: cette fonction calcule le tension en s mais generalement, cette derniere doit etre accessible ailleurs...
            auto t0_qS = GetTension(s);
            return t0_qS.Length() - chrono::Vdot(m_u, t0_qS);
        }

        /// Get the current chord at lagrangian coordinate s
        /// This is the position of the line if there is no elasticity.
        /// This is given by the catenary equation
        chrono::ChVector<double> GetUnstrainedChord(const double s) const;

        /// Get the current elastic increment at lagrangian coordinate s
        chrono::ChVector<double> GetElasticIncrement(const double s) const;

        /// Get the line position at lagrangian coordinate s
        chrono::ChVector<double> GetAbsPosition(const double s) const override;

        /// Get the position residual.
        /// This is the difference between the end line position calculated using catenary equation and the effective
        /// geometrical position (position of the ending node)
        chrono::ChVector<double> get_residual() const;

//        /// Compute the jacobian matrix with respect to tension using finite difference method
//        chrono::ChMatrix33<double> numerical_jacobian() const {
//            auto jac = chrono::ChMatrix33<double>();
//            // TODO
//            return jac;
//        }

        /// Compute the jacobian matrix with respect to tension using its analytical expression
        chrono::ChMatrix33<double> analytical_jacobian() const;

        /// Accessor relative to the embedded Newton-Raphson solver

        void SetSolverTolerance(const double tol) { m_tolerance = tol; }
        void SetSolverMaxIter(unsigned int maxiter) { m_itermax = maxiter; }
        void SetSolverInitialRelaxFactor(const double relax) { m_relax = relax; }

        /// Solve the nonlinear catenary equation for line tension using a Relaxed Newton-Raphson solver
        void solve();

        void Initialize() override {
            solve();
            // Generate assets for the cable
            GenerateAssets();
        }

        /// Returns the current cable length by line discretization
        double GetCableLength(const double n = 1000) const;

        //void UpdateTime(const double time) {
        //    m_time = time;
        //}

        /// Update time and state of the cable
        virtual void Update(const double time, bool update_assets = true) override {
            UpdateTime(time);
            UpdateState();
            UpdateAsset();
            ChPhysicsItem::Update(time, update_assets);
        }
//
        /// Update internal time and time step for dynamic behaviour of the cable
        virtual void UpdateTime(const double time) override {
            m_time_step = time - m_time;
            m_time = time;
        };
//
        /// Update the length of the cable if unrolling speed is defined.
        virtual void UpdateState() {
            if (std::abs(m_unrollingSpeed) > DBL_EPSILON and std::abs(m_time_step) > DBL_EPSILON) {
                m_cableLength += m_unrollingSpeed * m_time_step;
            }
            solve();
        }

        /// Update the assets (position of the discretized cable for the visualization)
        virtual void UpdateAsset();

        /// Generate the assets (discretized cable for the visualization)
        void GenerateAssets();
    };


}// end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
