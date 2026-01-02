# 🤖 PROJET_ESE_ROBOT_CHAT

Bienvenue dans le repository de notre projet de 3ème année.  
Ce projet a pour but de concevoir et développer un **robot intelligent** capable d’interagir et d’échanger de manière autonome.  

---

## 👥 Membres de l’équipe

- [Kenny Saint Fleur](https://github.com/Kennystflr)  
- [Louis Vozzola](https://github.com/louisvoz57700)  
- [Antoine LEMARIGNIER](https://github.com/LEMARIGNIER-Antoine)  
- [Thomas TERLINDEN](https://github.com/TTrld)  


---

## 📁 Structure du projet

Ce dépôt est organisé de la manière suivante :
## 📁 Arborescence du projet

- `Datasheet/` : Contient les **documents techniques** des composants électroniques (datasheets, notes d’application, documents de référence).  

- `Hardware/` : Contient la **conception matérielle**, incluant les projets KiCad (schémas, PCB, librairies locales) et fichiers liés à la fabrication (impression 3D).  

- `IMG/` : Contient toutes les **images et illustrations** du projet (schémas explicatifs, rendus PCB, photos du prototype, captures pour la documentation).  

- `Software/` : Contient le **code source / firmware**, y compris les projets STM32CubeIDE, le code embarqué (C/C++) et les scripts utilitaires éventuels.


---

## 🎯 Objectifs du projet

- Concevoir un **robot fonctionnel** (hardware + software).  
- Développer un **système de communication** entre le robot et un utilisateur.  
- Intégrer des **fonctionnalités d’autonomie et de dialogue**.  
- Documenter toutes les étapes du projet pour assurer sa **reproductibilité**.  

---
## Schéma de Principe
<img width="1336" height="904" alt="image" src="https://github.com/user-attachments/assets/7335ae36-fff2-46ec-bc82-de067b597700" />

## Organigramme de décision
<img width="1081" height="369" alt="Organigramme de decision(1)" src="https://github.com/user-attachments/assets/13309a05-b155-4094-a117-cbdac6cd5ec0" />


## 🛠️ Technologies utilisées (prévisionnel)

- **Langages** : Python / C  
- **Matériel** : STM32 / VL053 / LIDAR
- **Fabrication** : JLCPCB / Imprimante 3D FDM / Fusion 360

---

## 📅 Organisation

- **Phase 1** : Étude et conception  
- **Phase 2** : Développement hardware & software  
- **Phase 3** : Intégration et tests  
- **Phase 4** : Présentation finale  

---

## ✅ To-Do List

- [x] algo de prise de decision
- [x] LIDAR
- [ ] app bluetooth
- [x] Compréhension de tout les capteurs


---

## Résumé des séances :
### Séance du 19 / 09 :

- Nous avons continuer les PCB (G431 et WB55) et fait le schéma de principe
- Prise en main du capteur LIDAR

### Séance du 23 / 09 :

- Nous avons finis les PCB (Routage + Schematic) avec vérification du professeur
- Fais la 3D sur le robot avec intégration des capteurs TOF

### Séance du 24 / 09 :

- Nous avons implémenté une connexion bluetooth entre la stm32WB15CC et notre téléphone avec ST BLE Toolbox

