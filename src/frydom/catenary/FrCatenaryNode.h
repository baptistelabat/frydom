//
// Created by frongere on 03/08/17.
//

#ifndef FRYDOM_FRCATENARYNODE_H
#define FRYDOM_FRCATENARYNODE_H

#include <frydom/core/FrForce.h>
#include <chrono/physics/ChMarker.h>

namespace frydom {

    class FrCatenaryLine;

    enum NodeType {
        ANCHOR,
        FAIRLEAD,
        FREE
    };

    // Remarque: si on ajoute un noeud a un corps, alors il doit avoir une reference a un ChMarker qui est dependant du corps

    // Un noeud ancre est fixe dans l'espace absolu, il ne doit donc reposer que sur un ChCoordsys

    // Un noeut doit etre declare comme un noeud shared_ptr car il peut

    // Avoir une fabrique make_catenary_node et les fonctions make_anchor, make_free_node et make_body_node



    // TODO: creer une classe Anchor : FrCatenaryNode::ANCHOR ou un truc de ce type

    /// Abstract base class for FryDom Node involved in catenary cable modeling
    class FrCatenaryNode : public chrono::ChCoordsys<double> {

    private:
        NodeType m_node_type = ANCHOR;
        std::shared_ptr<chrono::ChMarker> m_body_marker = nullptr;

        FrForce m_force;  // Force additionnelle sur le noeud (a updater !!!)
        chrono::ChVector<double> m_TotalNodeForce = chrono::VNULL;  ///> Sum of forces on the node, exspressed in the absolute frame


    public:
        virtual chrono::ChVector<double> GetPos() = 0;

        /// Attach the node to a body
        void AttachToBody(std::shared_ptr<chrono::ChBody>& body) {
            // On est alors automatiquement attache a un corps et donc en mode FAIRLEAD
            // Ca cree alors un shared_ptr vers un ChMarker qui est garder en attribut de classe node et qui est ajoute
            // a body

            m_node_type = FAIRLEAD;

        }

        /// The node is released from the body and becomes a free node
        // TODO: voir comment ca se comporte quand une ligne a un noeud declare CONNECT mais qui n'a aucune autre ligne
        // de connectee...
        void Release() {

            // Si le noeud est en mode FREE, on retourne direct, rien a faire

            // Si le noeud est en mode FAIRLEAD
                // 1- On supprime le ChMarker du corps ainsi que de la classe + les efforts declares...

            // Si le noeud est en mode fixed, on passe juste en mode free...

            // 2- On declare le noeud comme etant FREE
            m_node_type = FREE;


        }

        /// Fix at the current position
        void FixOnEarth() {

            // Si le noeud est actuellement en mode FAIRLEAD, il faut faire un Release()

            m_node_type = ANCHOR;
        }

        /// Fix at the specified position
        void FixOnEarth(chrono::ChVector<double>& position) {

            m_node_type = ANCHOR;
        }

        void MakeFreeConnection() {
            // Make the node a free node that connects lines

            m_node_type = FREE;
        }





    };


}  // end namespace frydom

#endif //FRYDOM_FRCATENARYNODE_H
