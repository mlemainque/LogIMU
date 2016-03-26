/*

A FAIRE :

Vérifier les mesures


2013.08.22
Je n'avais en fait pas résolu totalement le problème : pour ce faire, j'ai dû enlever un bout de code dans
"pcd8544.h". Mais je n'ai pas compris pourquoi il y avait une erreur ...

2013.08.21
J'ai résolu une erreur inconnue qui est arrivée lorsque j'ai commencé à écrire la classe USER qui gère l'écran
lcd ; au début je pensais que ça venait de la classe pcd8544, mais au final je me suis rendu compte qu'il ne
fallait en aucun cas définir de variables dans le header. Pourtant c'est ce que je fais pour CDR dans maths.h !

2013.08.19
J'ai écrit une classe FLASH qui gère la lecture/écriture de tableaux de données : permet d'enregistrer les
masques et des vecteurs (zéros)

2013.08.18
J'ai abandonné l'idée de coder de beaux buffers pour la classe LOG : ceci aurait permis de faire des "listen"
sans que l'utilisateur s'en rende compte, mais ça ne vaut pas le coup
J'ai codé le calcul des mesures absolues d'une meilleure manière qu'auparavant

2013.08.17
J'ai écrit la classe LOG qui devrait à priori permettre d'envoyer tout type de paquet (avec un auxiliaire)
J'ai enlevé le type Vector et l'ait remplacé par des tableaux : j'ai gagné en place pour les conversions

2013.08.16
J'ai créé une classe SENSORS regroupant tous les capteurs, ce qui permet d'économiser de la mémoire en code
Je suis désormais le schéma classique de 2 fichiers par classe : un .h et un .cpp


*/

#include "wirish.h"
#include "SdFat.h"
#include "pcd8544.h"

