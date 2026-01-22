
## 📡 PCB Bluetooth

Pour cette carte électronique, nous avons privilégié l’utilisation des composants recommandés et fournis par notre professeur, afin d’assurer **fiabilité et cohérence pédagogique**.  

Les principaux composants intégrés sont :

- **Microcontrôleur** :  
  *STM32WB55CGU*, intégrant un second cœur **Cortex-M0+** dédié à la gestion de la communication **Bluetooth Low Energy (BLE)**.

- **Drivers moteurs** :  
  *ZXBM5210*, permettant un pilotage simple et efficace des moteurs.

- **Régulateur LDO (5 V → 3,3 V)** :  
  *BU33SD5WG-TR*, utilisé pour l’alimentation du microcontrôleur.

- **Convertisseur Buck (Vbat → 5 V)** :  
  *MP1475SGJ-P*, destiné à l’alimentation des capteurs (LiDAR, ToF, mesure de courant).

- **Multiplexeur I²C** :  
  *TCA9548AMRGER*, permettant de sélectionner dynamiquement le capteur ToF avec lequel communiquer.

- **Accéléromètre** :  
  *ADXL343*, utilisé pour la détection des taps et des mouvements.

---

<p align="center">
  <img src="../IMG/PCB_BLE_Kicad_3D_Up.png" width="32%" />
  <img src="../IMG/PCB_BLE_Kicad.png" width="32%" />
  <img src="../IMG/PCB_BLE_Kicad_3D_Down.png" width="32%" />
</p>

---

## 🔹 Description du PCB Bluetooth

### Section RF
La **partie droite du PCB** est dédiée à la **section RF**, avec notamment :

- Une **antenne conçue et réalisée directement sur le PCB**.  
- Un **filtre passe-bande centré sur 2,4 GHz**.  
- Un **réseau d’adaptation d’impédance** pour optimiser les performances radio et le débit maximal.

### Connecteur Bluetooth externe
Un **connecteur dédié au Bluetooth externe** permet de disposer d’une **solution de secours** si le module intégré ne fonctionne pas ou ne répond pas aux performances attendues, augmentant ainsi la **robustesse et la flexibilité** du système.

### Capteurs et moteurs
Le PCB intègre **quatre connecteurs pour les capteurs ToF**, pour détecter les obstacles à l’avant, à l’arrière et sur les côtés du robot.  
Les **deux sorties moteurs** et les **signaux des encodeurs** sont regroupés sur le **côté gauche**, facilitant l’intégration mécanique et optimisant le **routage des câbles**.

### Indicateurs de tension
Des **indicateurs 3,3 V et 5 V** permettent de vérifier rapidement le bon fonctionnement des régulateurs et simplifient les phases de **mise au point et de diagnostic**.

### Test points et conversion de tension
- La **partie droite** regroupe les **points de test**, donnant accès aux signaux principaux pour le **débogage**.  
- La **conversion de tension** est placée sur l’**autre face du PCB** (côté « chat-souris »), isolant les étages de puissance des circuits sensibles et améliorant la **lisibilité globale** de la carte.
