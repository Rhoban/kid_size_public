# Referee

* Détecter + utiliser les ball out (reset de la balle au centre du terrain du côté où elle est sortie
* Head hint

# Approach

* **Centrage de la balle**
* **Prendre de la marge par rapport aux limites des cages quand proche**
* Excentrage de la balle en défense? (à discuter)


# Search

* Tuner les patrouilles (attaquants plus en avant)

# Localisation
- Reset border filter (trancher au magneto) -> fixé, testé mais pas en match
- Haut niveau FP

# Mouvements
- Scan occasionnel en mode tracking

#GrosBan (pas forcément que grosban en fait)
- Refaire odometrie

#Vision
- 1 voit son épaule? (Check grosban model)
- Fausse balle vue par le gardien dans ses cages
- Logger des infos et crasher quand la vision a des comportements étranges (timestamp négatif etc)

#Com entre robots
- Distribution position balle

* Parametres à chequer:
  - Le compas maxAngle?

# Startup

- Tester les jauges de pression au démarrage
- Verifier parametres kicker

# A verifier

* SCRIPT DE DEMARRAGE A PARTAGER!!!!

#Pour Ludo (non urgent)
- [Benchmarker] Réduire le nombre d'espace
- [Benchmarker] Flag no color
- [Benchmarker] Nombre réels à coté des résultats. Exemple : 90.00% (18/20)
- [Benchmarker] Avoir un résultat global (résumé des N logs)
