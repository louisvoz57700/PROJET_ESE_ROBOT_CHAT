# 🛠️ Hardware – README

Bienvenue dans ce README dédié à **l’ensemble de la partie hardware** du projet.

---

## 🧩 Architecture matérielle

Pour notre projet, nous avons conçu **deux PCB distincts**, chacun répondant à un objectif précis :

- **[Bluetooth WB55](./WB55)**  
  Carte principale intégrant le microcontrôleur **STM32WB55** et l’ensemble de la chaîne Bluetooth Low Energy.

- **[Mainboard G431](./G431)**  
  Carte secondaire conçue comme solution de secours, destinée à prendre le relais en cas de défaillance du PCB Bluetooth.

Notre stratégie de développement s’est appuyée sur la conception d’un **module Bluetooth dédié**, incluant les éléments les plus critiques du design RF, notamment l’antenne et ses réseaux d’adaptation.  
En parallèle, par souci de robustesse et de gestion des risques, une **Mainboard alternative** a été développée pour garantir la continuité du projet.

Le module Bluetooth s’étant révélé **fonctionnel et performant dès la première mise sous tension**, nous avons validé cette architecture et décidé de **ne pas retenir la solution de repli** basée sur la Mainboard.

---
