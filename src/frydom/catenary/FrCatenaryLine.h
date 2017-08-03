//
// Created by frongere on 28/07/17.
//

#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H

#include <chrono/physics/ChMarker.h>
//#include <chrono/physics/ChNlsolver.h>
#include "chrono/core/ChLinearAlgebra.h"
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
//        double rho_line = 0.;           ///> density of the line (kg/m**3)


        // Cache data
        double m_EA; /// Axial stiffness


        chrono::ChVector<double> starting_tension;  // t(0)
        chrono::ChVector<double> ending_tension;    // t(L)



//        // Data for Newton-Raphson solver
//        chrono::ChMatrixNM<double, 2, 1> solution;
//        chrono::ChMatrixNM<double, 2, 1> residual;
//        chrono::ChMatrixNM<double, 2, 2> jacobian;
//        chrono::ChMatrixNM<double, 2, 1> delta;
//        double tolerance = 1e-6;
//        unsigned int maxiter = 100;

    public:

        FrCatenaryLine() = default;

        FrCatenaryLine(std::shared_ptr<FrCatenaryNode>& starting_node,
                       std::shared_ptr<FrCatenaryNode>& ending_node) {};

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



        void AddConstantField(chrono::ChVector<double> qvect) {

            // Ici on construit le champ q = qs * u avec qs > 0 et u direction unitaire. Ces donnees sont utilisees

            // Adding a constant field to be merged with gravity and eventually water buoyancy... (mettre un flag pour ca !! inWater=true)


        }

        void UpdateConstantField() {

            // Update de la matrice U = I - uu' et mise en cache

            // Voir pour d'autre updates a faire ici et impactes par le champ de force distribue sur la ligne



        }



        // Real computations :

        chrono::ChVector<double> GetTension(double s) {
            // t(s)
        }


        chrono::ChVector<double> ComputeUnstrainedSolution(double s) {
            // On calcule pc(s)
        }

        chrono::ChVector<double> ComputeElasticIncrement(double s) {
            // pe(s)
        }

        chrono::ChVector<double> ComputePosition(double s) {
            // p(s)
        }

        double GetStrainedLineLength() {
            // L + dl

            // with dl = \int_0^L \frac{t(s)}{EA} ds a calculer par trapeze...


        }

        chrono::ChVector<double> GetCatenaryResidual() {
            // On evalue C(t, p) = ComputePosition(L) - position recuperee depuis le noeud ending (guess de position)

        };



        chrono::ChMatrix33<double> GetCatenaryJacobian() {};


        void SolveCatenaryEquation() {};








    };





}// end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
