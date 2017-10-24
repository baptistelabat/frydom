
Specification pour le stockage de base de données hydrodynamiques au format HDF5
================================================================================


Preambule
---------

Les bases de données hydrodynamiques concernent les coefficients hydrodynamiques 
utilisés dans les du premier (et à terme du second) ordre des efforts de vague 
en théorie hydrodynamique potentielle.

Dans ces bases de données hydrodynamiques, on trouve pour le premier ordre 
(le second ordre devra à terme aussi être traité) en terme de données::

	* Date du calcul 
	
	* Nom du calcul (optionnel)
	
	* Acceleration de la gravite (m/s**2)
	
	* Densité de l'eau (kg/m**3)
	
	* Longueur de normalisation
	* Hauteur d'eau (0 pour infini, positif pour une hauteur finie)
	* Nombre de corps en interaction (nbBody)
	
	* Données de calcul BEM
		+ Discrétisation en fréquence:
			- nombre de pulsations (nw)
			- pulsation min (rad/s) (wmin)
			- pulsation max (rad/s) (wmax)
		+ Discrétisation en angles de propagation de vague (pour la 
		diffraction et Froude Krylov)
			- nombre d'angles (ntheta)
			- angle min (deg) (theta_min)
			- angle max (deg) (theta_max)
		+ Discrétisation temporelle (réponses impulsionnelles de radiation)
			- temps final (s) (tf)
			- Nombre de pas de temps (nt)
	
	* for body_force_i in bodies:
		+ Nom du corps (optionnel)
		+ ID du corps (entier défini de manière unique)
		+ Position du corps = position absolue du repère du maillage du corps 
		(Point 000 du corps dans sa description) par raport au repère absolu
		du monde situé sur la surface libre au repos (3 x 1)
		+ Nombre de modes de forces (nbForce)
		+ Description des modes force/moment:
			- mode force (liste):
				- axe(3 x 1)
			- mode moment (liste):
				- axe (3 x 1)
				- point d'application (3 x 1)
		+ Nombre de degrés de liberté (nbDOF)
		+ Description des modes de degrés de liberté:
			- mode translation (liste):
				- axe (3 x 1)
			- mode rotation (liste):
				- axe (3x1)
				- centre de rotation (3x1)
		+ Matrice de raideur hydrostatique (6x6)
		+ Nom du fichier de maillage utilisé pour le calculs BEM
		+ Données du maillage utilisé par les calculs BEM
			- Nombre de vertex (nv) du maillage
			- Tableau des coordonnées catésiennes des vertex (nv x 3)
			- Nombre de facettes du maillage
			- Tableau des connectivités facette du maillage (nf x 4). 
			Supporte les quadrangles et les triangles. Les quadrangles
			sont décrits par les 4 identifiants de vertex/ La numérotation 
			commence à zéro) présentés dans le sens anti-horaire afin 
			d'assurer une normale sortante. Les triangles sont représentés 
			en répétant l'identifiant du premier vertex en fin de ligne.
		+ Coefficients d'excitation de Froude-Kylov (FroudeKrylovCoeffs)
			- for [wave_dir[iwd] for iwd in range(ntheta)] (syntaxe Python)
				- FroudeKrylovCoeffs_real (nbForce x nw)
				- FroudeKrylovCoeffs_imag (nbForce x nw)
		+ Coefficients d'excitation de diffraction
			- for [wave_dir[iwd] for iwd in range(ntheta)] (syntaxe Python)
				- DiffractionCoeffs_real (nbForce x nw)
				- DiffractionCoeffs_imag (nbForce x nw)
		+ Coefficients de masse ajoutée en fréquence infinie:
			- for body_motion_j in bodies:
				for dof_k in body_motion_j.dofs:
				- AddedMassInfFreq (nbForce x nw)
		+ Coefficients de masse ajoutée en fréquence nulle (on considère
		pour cette fréquence qu'on ne s'intéresse pas à l'interaction entre
		flotteurs car ces données sont utilisées en manoeuvrabilité
		sur eau calme.):
			- for body_motion_j in bodies:
				- AddedMassZeroreq (nbForce x nb_dof_j)
		+ Coefficients de radiation fréquentielle:
			- for body_motion_j in bodies:
				- for dof_k in body_motion_j.dofs:
					- AddedMass_j_k (nbForce x nw)
					- WaveDamping_j_k (nbForce x nw)
		+ Réponse impulsionnelle:
			- for body_motion_j in bodies:
				- for dof_k in body_motion_j.dofs:
					- ImpulseResponseFunction_j_k (nbForce x nt)
		



