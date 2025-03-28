![logo isae-robotics-club](../.media/isae_robotics_logo.png)

# Tutoriel installation HN

## Prérequis :

- ## Le tutoriel "Téléchargement du code"

- ## Un environnement Linux

Si vous n'avez pas un ordinateur sous Linux (ce qui est probable), vous pouvez utiliser WSL, une VM, voire un dual boot pour les plus courageux. Si vous utilisez WSL, vous devez le configurer de sorte à pouvoir y exécuter des applications graphiques.

Cela peut sembler redondant étant donné que Docker existe aussi sous Windows ou MacOS, mais d'expérience, c'est une mauvaise idée d'essayer. De plus, un environnement Linux est nécessaire car certaines applications à exécuter ont une interface graphique.

Si votre distribution Linux n'utilise pas `apt`, vous devrez adapter les commandes données en exemple.

Vous avez besoin des droits d'administration pour installer certaines dépendances.

- ### Docker

La procédure dépend de votre distribution Linux. Voir https://docs.docker.com/engine/install/.
La méthode ci-dessous est pour Ubuntu.

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Finally, install Docker:
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

**Optionnel (mais recommandé)**

Si par la suite, vous souhaitez utiliser Docker sans avoir à utiliser `sudo` :

```
sudo groupadd docker
sudo usermod -aG docker $USER
```

Vous devez fermer la session et vous reconnecter (ou redémarrer WSL) pour que les modifications prennent effet.

Source : https://docs.docker.com/engine/install/linux-postinstall/

- ### Make

Il est probablement pré-installé, mais si jamais il ne l'est pas :

```
sudo apt install make
```

## Créer l'image Docker

Cette étape est potentiellement longue et nécessite une bonne connexion à internet. Vous devez vous placer dans le répertoire où vous avez téléchargé le code, par exemple : `cd ~/Documents/isae-bots-hn-2024`.

```
make build-core
make build-base
```

## Créer le conteneur Docker
```
make create-container
```

Si le conteneur existe déjà mais que vous souhaitiez le réinitialiser (par exemple parce que l'image Docker a changé), vous devez remplacer la commande par :
```
make clear-container
```

Vous pouvez sortir du Docker en tapant `exit`.