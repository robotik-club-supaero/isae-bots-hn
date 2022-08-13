#!/bin/bash
#     ____                                                  
#    / ___| _   _ _ __   __ _  ___ _ __ ___                 
#    \___ \| | | | '_ \ / _` |/ _ \ '__/ _ \                
#     ___) | |_| | |_) | (_| |  __/ | | (_) |               
#    |____/ \__,_| .__/ \__,_|\___|_|  \___/                
#   ____       _ |_|       _   _ _       ____ _       _     
#  |  _ \ ___ | |__   ___ | |_(_) | __  / ___| |_   _| |__  
#  | |_) / _ \| '_ \ / _ \| __| | |/ / | |   | | | | | '_ \ 
#  |  _ < (_) | |_) | (_) | |_| |   <  | |___| | |_| | |_) |
#  |_| \_\___/|_.__/ \___/ \__|_|_|\_\  \____|_|\__,_|_.__/ 
#

# On crée quelques binds dans le .bashrc utiles pour la simulation (que l'on va désactiver après il vaut mieux)
echo 'control_bindkeys ' >> ~/.bashrc  # on crée un appel à une fonction dans le bashrc pour appliquer les bindkeys

# On se place en racine pour lancer Terminator
WDIR=$PWD
cd ~ 
terminator -mu -l simulation --working-directory $WDIR

 # lancement d'une session de terminator avec la layout simulation
 # l'option -u permet de désactiver le dBus pour que plusieurs sessions de terminator n'interfèrent pas entre elles
 # l'option -m permet d'avoir déjà une fenêtre maximisée à l'ouverture

sed -i "/^control_bindkeys/d" ~/.bashrc # on enlève l'appel à la fonction des bindkeys dans le bashrc

docker kill $(docker ps -a -q)

# La commande stty quit \[STOP_KEY] permet de bind temporairement une touche à la commande quit (par défaut Ctrl-\)
# Par défaut on l'utilise sur la touche "suppr" (de symbole ^[[3~ par défaut, pour avoir le symbole sur votre ordi faites dans un terminal Ctrl-V + [KEY])
# Il faut la relancer pour chaque terminal, et du coup elle ne perturbe pas les terminaux en dehors de la simulation


# On a ensuite un bind dans le bashrc qui permet de clear et de relancer en une seule touche avec bind -x '"[RESTART_KEY]":"reset && exit"'

# Et avec ça on a deux petits keybindings pour arrêter et relancer rapidement la simulation

