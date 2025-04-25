# DISPLACEMENT NODE

Package du Displacement Node.

Gère l'évitement et exécute le pathfinder.

Le fichier `disp_node.py` contient principalement la définition des publishers et subscriptions ROS.
L'essentiel de la logique d'évitement est implémentée dans `disp_manager.py` 

L'implémentation du pathfinder (A*) est en C++ dans le package `pathfinder`. Le fichier pathfinder.py est juste un "wrapper".

A FAIRE : recalage (idée : effectuer un premier déplacement pour se rapprocher + se mettre dos au mur, puis envoyer une commande de vitesse faible jusqu'à toucher le mur)