//
// Created by frongere on 24/05/18.
//

#ifndef FRYDOM_DICE_DTRIANGLEINTEGRATIONS_H
#define FRYDOM_DICE_DTRIANGLEINTEGRATIONS_H

#include "FrMesh.h"

namespace frydom {

    /* Ici, on regroupe les differentes implemetations d'integration surfacique de champs scalaires
     * et vectoriels sur une facette triangulaire
     *
     * Ces outils doivent avoir connaissance de l'API OpenMesh afin d'iterer sur les facettes mais
     * egalement d'attribuer les proprietes des facettes, principalement les integrales de surface
     *
     * Les fonctions fournies modifient le maillage en entree qui doit etre passe par reference.
     *
     * Cette classe s'occupe de calculer des noyaux d'integrales, ie ces derniers doivent maximiser
     * leur reutilisabilite, ce qui s'obtient par l'extraction tous les scalaires multiplicateurs du
     * signe integral et par l'eclatement des sommes des integrales '\int{a+b} = \int{a} + \int{b})
     * Voir comment cela est fait actuellement dans les calculs d'itegrale de surface de DMesh...
    */


    class FrTriangleIntegrations {

    public:

    };

}

#endif //FRYDOM_DICE_DTRIANGLEINTEGRATIONS_H
