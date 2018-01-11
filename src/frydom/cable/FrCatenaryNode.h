//
// Created by frongere on 03/08/17.
//

#ifndef FRYDOM_FRCATENARYNODE_H
#define FRYDOM_FRCATENARYNODE_H

#include "frydom/core/FrObject.h"
#include "frydom/core/FrForce.h"
#include <chrono/physics/ChMarker.h>
#include "chrono/core/ChVector.h"


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



    /// Abstract base class for FryDom Node involved in catenary cable modeling
    class FrCatenaryNode : public chrono::ChMarker, FrObject {  // FIXME: faire deriver de ChMarker de maniere a ce que la position soit update automatiquement ???

    private:
        NodeType m_node_type = ANCHOR;
        chrono::ChVector<double> m_position;



    public:

        FrCatenaryNode() = default;

        FrCatenaryNode(const double x, const double y, const double z)
                : m_position(chrono::ChVector<double>(x, y, z)){}

        chrono::ChVector<double> GetPos() {
            // Different wether the type of node
            switch (m_node_type) {
                case ANCHOR:
                    return m_position;
            }
        };

        void SetPos(double x, double y, double z) {
            m_position.x() = x;
            m_position.y() = y;
            m_position.z() = z;
        }

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
