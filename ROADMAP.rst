FRyDoM development Roadmap
==========================


Premiers objectifs
------------------

Pour le moment, on ne considère pas de vitesse d'avance (on prends U=0). On pourra le faire plus tard en introduisant
l'unification  man/seakeeping développée par Fossen ("A nonlinear unified state-space model for ship maneuvering and
control in a seaway").

On doit introduire en priorité les modèles suivants:

* Propulsion:
    T = Kt * n**2 * rho * D**4
On prendra pour le moment un Kt constant

* Aéro:
    Champs de vent constant inclu dans le FrOffshoreSystem
    modèle en 0.5 * rho * S_xy * v**2 * C_xyz(alpha_w)

    On pourra prendre des coeffs C constants en première approche

* Courant:
    Idem vent

Remarque: pour le vent et le courant en terme de direction, on prendra comme convention un angle exprimé dans le repère
NED et cet angle indiquera que le flux "viennent de". Expl: un vent au 90 est un vent qui vient de l'est et qui fait donc
dériver les structures vers l'ouest.

* Radiation:

    Tant qu'on aura pas mis en place une unification man/seakeeping, on considérera uniquement un coeff de masse ajoutée
constant choisi en w=0 et issu d'un code de tenue à la mer (Margot ?). Il s'agira en première instance de ne pas coupler
les ddl et de surcharger la méthode chrono qui va bien (dans ChBody, chercher GetMass()) afin qu'elle permette d'introduire
des termes de masses d'eau ajoutée, tant en mouvement linéaire qu'angulaire.

* Dérive due aux vagues:

    On considère qu'on nous fournit une base de données hydro des efforts constants du second ordre en domaine fréquentiel
pour chaque ddl. On fera une simple sommation sur la bande de fréquence du spectre de houle discrétisé.

* Amortissements:

    Pour le moment, un modèle linéaire de type Dv avec D matrice diagonale suffira.

