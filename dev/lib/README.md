Ne placez pas les programmes du HN ici ; ils ne seront pas copiés sur le Docker. Utilisez le dossier dev/src à la place.

# Mettre à jour la BR

Le dossier "br" est un miroir de la branche "br" du GitHub du BN. (Voir documentation sur les sous-modules Git.)

Vous pouvez faire des modifications locales non-persistentes à la BR ici, mais ne faites pas de "commit" ou de "push" des modifications. Si vous souhaitez apporter des modifications persistentes, faites-les directement sur le GitHub du BN.

Lorsque la BR est modifiée sur le GitHub du BN, les modifications ne sont PAS répercutées automatiquement sur le HN parce que le miroir est lié à un commit spécifique.

## Mise à jour par quelqu'un d'autre

Si des commits distants ont affecté la version de la BR liée au HN, vous devez faire un "pull" _spécial_ :
```
git pull --recurse-submodules
```

## Mise à jour par vous
Si la BR a été modifiée sur le GitHub BN, vous pouvez répercuter les modifications sur le HN :

-  Mettez à jour la BR dans l'espace de travail local
```bash 
cd dev/lib/br
git fetch
# Utilisez toujours un commit spécifique de la branche "br" pour être sûr d'avoir la bonne version de la BR (évitez `git checkout br`)
git checkout <hash_du_commit>
```  

-  Indexez le changement
```bash
# Retour à la racine du git HN
cd ../../..
git add dev/lib/br
```

- Faites un "commit"

Ajoutez les autres fichiers modifiés si nécessaire, puis faites un commit comme d'habitude :

```
git commit -m "Updated BR"
git push
```

## Recompilation

La BR doit être recompilée à chaque fois qu'elle est modifiée. 

Pour recompiler :
```
make main CMD="colcon build --symlink-install"
```
