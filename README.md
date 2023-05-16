# ISAE-BOTS-HN 2023

![logo isae-robotics-club](.media/isae_robotics_logo.png)

## Description

## Utilisation

Pour une utilisation propre du Git, se référer au tuto Git. (sur Gitlab dans **/isae-robotics/others/tuto**)

### Détail des branches

Pour un détail des branches, voir ci-dessous:

* La branche `main` : contient les codes stables (qui compilent et qui marchent).

* La branche `pi_release` : contient les codes dernièrement flashés sur la pi (ou en cas de hotfix ceux modifiés directement sur la pi).

* Autres branches : pour chaque modifs ou nouveauté.

### Remarques

> ATTENTION : On **ne commit pas dans la branche `main`** directement !

Créez une nouvelle branche pour travailler. Une fois le travail fini :

* faite un **merge** de `main` dans votre branche (pour la mettre à jour) et résolvez les conflits de merge.

* vérifiez que vous n'avez rien cassé de ce qu'il y avait dans `main`.

* faite un **merge** de votre branche dans `main`. Revérifiez que tout marche bien.

> REMARQUE : Evitez de faire des `git pull` ...

Oui alors quand même, avant chaque session de travail, si vous travaillez à plusieur sur une branche ou bien que vous récupérez le travail d'une autre branche, il faut la mettre à jour.

Pour cela, on vous demande de remplacer la commande

```bash
git pull
```

par les commandes

```bash
git fetch
git rebase
```
ou (équivalent)

```bash
git pull --rebase
```

Explication : le `git pull` squeeze tous les commits de modifs récupérés, ce qui entraine une perte de l'historique des commits
