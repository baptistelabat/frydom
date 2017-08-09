//
// Created by frongere on 28/07/17.
//

#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H

#include <chrono/physics/ChMarker.h>
#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChMatrix33.h"
#include "FrCatenaryNode.h"

namespace frydom {


    class FrCatenaryLine {

    private:

        std::shared_ptr<FrCatenaryNode> m_starting_node;
        std::shared_ptr<FrCatenaryNode> m_ending_node;

        bool m_elastic = true;

        double m_Lu = 0.;                 ///> unstretched length
        double m_E = 1e12;                ///> young_modulus (default is near from infinite)
        double m_A = 0.15;                ///> line section area

        // Field data
        double m_qs = 9.81;  // FIXME: NON, ce n'est pas la bonne expression de la force de gravite !!!
        chrono::ChVector<double> m_u = chrono::ChVector<double>(0, 0, -1);
        chrono::ChMatrix33<double> m_Umat; // Mettre a jour lors d'une modif de champ

        // Cache data
        double c_EA = 1e12; /// Axial stiffness // TODO: mettre a jour les 2 pptes suivantes lors d'une modif de E ou A
        double c_L_EA = 0.; // L/EA

        chrono::ChVector<double> m_t0;  // Tension at starting node (this is the main unknown !!)
        chrono::ChVector<double> c_tL;    // Tension at ending node (tL = t0 - q*L)

        chrono::ChVector<double> c_t0_qs; // t0 / qs
        chrono::ChVector<double> c_Ut0_qs; // U t0 / qs

        double c_t0_norm; // ||t0||
        double c_tL_norm; // ||tL||

        chrono::ChVector<double> c_t0_unit; // t0 / ||t0||
        chrono::ChVector<double> c_tL_unit; // tL / ||tL||

        double c_rho0 = 0.; // ||t0|| - u' t0
        double c_rhoL = 0.; // ||tL|| - u' tL
        double c_one_rho0 = 0.; // 1 / rho0
        double c_one_rhoL = 0.; // 1 / rhoL
        chrono::ChMatrix33<double> c_UxlnrhoL_rho0; // U log(rhoL / rho0)



        // Data for Newton-Raphson solver
        chrono::ChVector<double> m_residual;
        chrono::ChMatrix33<double> m_jacobian;
        double m_tolerance = 1e-6;
        unsigned int m_itermax = 100;

    public:

        FrCatenaryLine() {

            UpdateConstantField();
        }

        FrCatenaryLine(std::shared_ptr<FrCatenaryNode>& starting_node,
                       std::shared_ptr<FrCatenaryNode>& ending_node)
                : m_starting_node(starting_node), m_ending_node(ending_node) {

            // TODO: voir a regler automatiquement une longueur L de cable ie la distance entre les 2 marker
            // + 10%...


            // Computing a default value for the line length
            auto node_distance = (m_starting_node->GetPos() - m_ending_node->GetPos()).Length();
            m_Lu = 1.1 * node_distance;

            // First guess on the line tension
//            m_t0 = 0.5 * m_qs * m_Lu * m_u;  // TODO: voir si on met ca dans une methode de first guess...

            // Updating the constant field
            UpdateConstantField();


        };

        /// Set the line axial stiffness from material stiffness and diameter
//        void SetAxialStiffness(double E, double A) { m_EA = E*A; }

        void SetLineSectionArea(double A) {};

        void SetYoungModulus(double E) {};

        /// Set the line axial stiffness from section stiffness
//        void SetAxialStiffness(double p_EA) {m_EA = p_EA; }

        /// Set the unstretched length of the line element
        void SetUnstretchedLength(double length) { m_Lu = length; }

        /// Set the line density (kg/m)
        void SetLineLinearDensity(double linear_density) {};

        /// Set the line material density
        void SetLineDensity(double density) {};


        void SetStartingNode(std::shared_ptr<FrCatenaryNode>& starting_node) { m_starting_node = starting_node; };
        void SetEndingNode(std::shared_ptr<FrCatenaryNode>& ending_node) { m_ending_node = ending_node; };

        void SetNodes(std::shared_ptr<FrCatenaryNode>& starting_node,
                      std::shared_ptr<FrCatenaryNode>& ending_node) {
            m_starting_node = starting_node;
            m_ending_node = ending_node;
        };

        void SetElastic(const bool flag) { m_elastic = flag; }
        void SetElasticOn() { m_elastic = true; }
        void SetElasticOff() { m_elastic = false; }


        void ResetConstantFieldToGravityOnly() {}


        void AddConstantField(chrono::ChVector<double> qvect) {

            // Ici on construit le champ q = qs * u avec qs > 0 et u direction unitaire. Ces donnees sont utilisees

            // Adding a constant field to be merged with gravity and eventually water buoyancy... (mettre un flag pour ca !! inWater=true)


        }

        void UpdateConstantField() {

            // Update de la matrice U = I - uu' et mise en cache

            // Voir pour d'autre updates a faire ici et impactes par le champ de force distribue sur la ligne

            m_Umat.Set33Identity();
            m_Umat -= chrono::TensorProduct(m_u, m_u);

        }



        // Real computations :

        chrono::ChVector<double> GetTension(double s) {
            return m_t0 - s * m_qs * m_u;  // TODO: voir a stocker un vecteur complet q...
        }


        chrono::ChVector<double> ComputeUnstrainedSolution(double s) {

            auto t0mqS = GetTension(s);
            auto t0mqS_norm = t0mqS.Length();
            auto rho_s = t0mqS_norm - m_u.Dot(t0mqS);

            return m_Umat.Matr_x_Vect(c_t0_qs) * log(rho_s/c_rho0) - (m_u/m_qs) * (t0mqS_norm - c_t0_norm);

        }

        chrono::ChVector<double> ComputeElasticIncrement(double s) {
            return (m_qs * s / c_EA) * (c_t0_qs - 0.5 * s * m_u);
        }

        chrono::ChVector<double> GetUnstrainedChord() { // Return pc(L)
            // The computation is made quick by caching many values that are used both by evaluation and
            // jacobian computation
            return c_UxlnrhoL_rho0.Matr_x_Vect(c_t0_qs) - (c_tL_norm - c_t0_norm) / m_qs * m_u;
        }

        chrono::ChVector<double> GetEndingNodePosition() {

            // Position of the starting node
            auto pL = m_starting_node->GetPos();

            // Adding line chord
            pL += GetUnstrainedChord();

            // Adding elasticity increment
            if (m_elastic) {
                pL += ComputeElasticIncrement(m_Lu);
            }

            return pL;

        }

        // TODO: donner la possibilite de rentrer aussi un vecteur
        chrono::ChVector<double> ComputePosition(double s) {

            // Position of the starting node
            auto pS = m_starting_node->GetPos();

            // Adding the unstrained solution
            pS += ComputeUnstrainedSolution(s);

            // If the line is elastic, adding the elastic increment
            if (m_elastic) {
                pS += ComputeElasticIncrement(s);
            }

            return pS;

        }

        double GetStrainedLineLength() { // TODO
            // L + dl

            // with dl = \int_0^L \frac{t(s)}{EA} ds a calculer par trapeze... eq (18)


        }

        chrono::ChVector<double> GetCatenaryResidual() {
            // On evalue C(t, p) = ComputePosition(L) - position recuperee depuis le noeud ending (guess de position)

            return GetEndingNodePosition() - m_ending_node->GetPos();
        };



        chrono::ChMatrix33<double> GetCatenaryJacobian() {
            // TODO: utiliser la fonction TensorProduct de ChMatrix33 !! --> outer product

            auto jac = c_UxlnrhoL_rho0 * (1./m_qs);

            auto tmp = c_one_rhoL * c_tL_unit - c_one_rho0 * c_t0_unit + (c_one_rho0-c_one_rhoL)*m_u;
            jac += chrono::TensorProduct(c_Ut0_qs, tmp);
            jac += chrono::TensorProduct(m_u, c_t0_unit-c_tL_unit);

            if (m_elastic) {
                jac.Element(0, 0) += c_L_EA;
                jac.Element(1, 1) += c_L_EA;
                jac.Element(2, 2) += c_L_EA;
            }

            return jac;

        };


        void SolveCatenaryEquation() {
            // Using Newton-Raphson algorithm to solve for t0, considering position of both ending nodes
            // are given. This is a one line solution. For solvng some free node, consider using a
            // FrCatenaryNet object to enclose lines.

            unsigned int iter = 1;
            int code;
            double maxresidual;

            chrono::ChMatrixNM<double, 3, 1> e;
            chrono::ChMatrixNM<double, 3, 1> delta;


            while (true) {

                // Updating the cache to make it up to date with respect to tension.
                UpdateCache();

                // Computing the residual
                m_residual = GetCatenaryResidual();


                if (iter == m_itermax) {
                    code = 1; // itermax reached, no convergence
                    break;
                }

                // Computing the max residual (Infinite norm of the residual)
                // TODO
                maxresidual = m_residual.LengthInf();

                if (maxresidual < m_tolerance) {
                    code = 0; // Convergence reached
                    break;
                }

                // Computing the jacobian
                m_jacobian = GetCatenaryJacobian();

                // Solving the system J dX = -E(X)
                e.Element(0, 0) = -m_residual.x();
                e.Element(1, 0) = -m_residual.y();
                e.Element(2, 0) = -m_residual.z();

                chrono::ChLinearAlgebra::Solve_LinSys(m_jacobian, &e, &delta);

                // Updating the solution
                m_t0.x() += delta.Element(0, 0);
                m_t0.y() += delta.Element(1, 0);
                m_t0.z() += delta.Element(2, 0);


                iter++;

            }


        }


        void UpdateCache() {

            // Attention ici. Si la tension t0 est dans le meme sens que q = qs u, on a une singularité et le câble ne
            // peut être que droit et seul t0 ou tL possede une tension egale


            // Updating every cache variables
            c_tL = m_t0 - m_Lu * m_qs * m_u;

            c_t0_qs = m_t0 / m_qs;

            c_Ut0_qs = m_Umat.Matr_x_Vect(m_t0);

            c_t0_norm = m_t0.Length();
            c_tL_norm = c_tL.Length();

            c_t0_unit = m_t0 / c_t0_norm;
            c_tL_unit = c_tL / c_tL_norm;

            c_rho0 = c_t0_norm - m_u.Dot(m_t0);
            c_rhoL = c_tL_norm - m_u.Dot(c_tL);

            c_one_rho0 = 1. / c_rho0;
            c_one_rhoL = 1. / c_rhoL;

            c_UxlnrhoL_rho0 = m_Umat * log(c_rhoL/c_rho0);


        }







    };





}// end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
