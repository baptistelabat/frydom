//
// Created by frongere on 28/07/17.
//

#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H

#include "chrono/physics/ChMarker.h"
#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChMatrix33.h"
#include "FrCatenaryForce.h"
#include "frydom/core/FrNode.h"
#include "FrCable.h"

//#include <limits>

//#define SQRT_EPS sqrt(std::numeric_limits<double>::epsilon())

// TODO: prevoir une discretisation automatique pour laquelle on precise la taille cible d'un element
// Servira dans la visu et pour l'application de forces comme Morrison.


namespace frydom {


    class FrCatenaryLine : public FrCable {

    private:

        std::shared_ptr<FrNode> m_starting_node; // FIXME: retirer les attributs apparaissant deja dans la classe de base !!!
        std::shared_ptr<FrNode> m_ending_node;

        bool m_elastic = true;

        double m_Lu = 0.;                 ///> unstretched length
        double m_E = 1e12;                ///> young_modulus (default is near from infinite)
        double m_A = 0.15;                ///> line section area
        double c_EA = 0.15e12;

        chrono::ChVector<double> m_t0;

        double m_q;
        chrono::ChVector<double> m_u;
        chrono::ChVector<double> c_qvec;

        chrono::ChMatrix33<double> c_Umat;

        // Forces to apply to bodies
        std::shared_ptr<FrCatenaryForce> m_starting_force;
        std::shared_ptr<FrCatenaryForce> m_ending_force;

        // Data for Newton-Raphson solver
        double Lmin = 1e-10;
        double m_tolerance = 1e-6;
        unsigned int m_itermax = 100;
        double relax = 0.1;

    public:

        FrCatenaryLine(std::shared_ptr<FrNode>& starting_node,
                       std::shared_ptr<FrNode>& ending_node,
                       bool elastic,
                       double EA,
                       double L,
                       double q,
                       chrono::ChVector<double> u
        );

        void GenerateDiscretization(const double elt_length=1.) {
            // TODO
        }
        /// Split the line into two parts at lagrangian coordinate s
        /// The current line now lies between 0 and s
        /// A new line is created and lies between s and L
        /// A new free node is created at s
        FrCatenaryLine SplitLine(const double s) {
            // TODO
        }

        /// Set a new starting node for the line
        /// The new node may attached to another body
        void SetStartingNode(const std::shared_ptr<FrNode> node) {
            // First removing the line from the current node


            // Removing the force form the current node parent body

            // Then adding the new node to the line and the force to the support body


        }

        void SetEndingNode (const std::shared_ptr<FrNode> node) {
            //TODO
        }

        // TODO: avoir une methode pour detacher d'un noeud ou d'un corps. Dans ce cas, un nouveau oeud fixe est cree a
        // la position courante de la ligne.

        /// Get the starting node of the line
        std::shared_ptr<FrNode> GetStartingNode() const { return m_starting_node; }

        /// Get the ending node of the line
        std::shared_ptr<FrNode> GetEndingNode() const { return m_ending_node; }

        /// Get the starting force of the line
        std::shared_ptr<FrCatenaryForce> GetStartingForce() { return m_starting_force; }

        /// Get the ending force of the line
        std::shared_ptr<FrCatenaryForce> GetEndingForce() { return m_ending_force; }

        /// Get the current absolute position of the line's starting node
        chrono::ChVector<> GetPosStartingNode() const { return m_starting_node->GetAbsPos(); }

        /// Get the current absolute position of the line's ending node
        chrono::ChVector<> GetPosEndingNode() const { return m_ending_node->GetAbsPos(); }

        /// Guess the line tension from line boundary positions
        /// Used to initialize the Newton-Raphson algorithm (see solve method) TODO: mettre un see a facon doxygen...
        void guess_tension();

        /// Set the stiffness coefficient of the line
        void SetEA(const double E, const double A);

        /// Get the stiffness coefficient of the line
        void SetEA(const double EA) { c_EA = EA; }

        /// Get the Young Modulus of the line
        double get_young_modulus() const { return m_E; }

        /// Get the cross section area of the line
        double get_cross_section_area() const { return m_A; }

        /// Get the stiffness coefficient of the line
        double GetEA() const { return c_EA; }

        /// Set the linear density of the line (in kg/m)
        void SetLinearDensity(const double lambda) {
            // TODO
        }

        /// Get the linear density of the line (in kg/m)
        double GetLinearDensity() const {
            // TODO
        }

        // TODO: accessors pour le champ de force distribue

        chrono::ChVector<double> GetTension(const double s) const;

        /// Returns the cartesian tension at the start of the line.
        /// This tension is applied by the line on its node
        chrono::ChVector<double> get_starting_node_tension() const { return m_t0; }

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
        chrono::ChVector<double> GetAbsPosition(const double s) const;

        /// Get the position residual.
        /// This is the difference between the end line position calculated using catenary equation and the effective
        /// geometrical position (position of the ending node)
        chrono::ChVector<double> get_residual() const;

        /// Compute the jacobian matrix with respect to tension using finite difference method
        chrono::ChMatrix33<double> numerical_jacobian() const {
            auto jac = chrono::ChMatrix33<double>();
            // TODO
            return jac;
        }

        /// Compute the jacobian matrix with respect to tension using its analytical expression
        chrono::ChMatrix33<double> analytical_jacobian() const;

        /// Solve the nonlinear catenary equation for line tension using a Relaxed Newton-Raphson method
        void solve();

        /// Returns the current cable length by line discretization
        double get_cable_length(const double n=1000) const;

        void UpdateTime(const double time) {
            m_time = time;
        }

        void UpdateState() {
            // TODO: solve ?
        }
    };


}// end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
