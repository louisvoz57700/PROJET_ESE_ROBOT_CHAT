## PCB_A : version WB55
### Partie Bluetooth
La section Software/PCB_A/Preuve_BLE/BLE est la section de notre code de preuve de notre antenne Bluetooth intégrée à notre PCB. Tout fonctionne ! Notre code est ici :[code](PCB_A/Preuve_BLE/BLE)

Ce qu'on a fait : on a suivi la vidéo suivante : STM32 Bluetooth Firmware Tutorial (Bring-Up) - Phil's Lab #129

Les étapes : aller sur stm32CubeProgrammer et on upgrade les firmware, on n'oublie pas de start le wireless stack sinon on ne verra pas notre carte lors de la connexion bluetooth.

Puis sur stm32CubeIde, on va aller créer un service qui peut écrire et lire.

On rajoute une fonction pour interpréter les notifications (la fonction est ici : [custom_app.c](PCB_A/Preuve_BLE/BLE/STM32_WPAN/App/custom_app.c))

Rajoutons un message : "Hello Laurent Fiack from antonio, Louis, Kenny et thomas!", qu'on va lire sur notre téléphone pour tester la connexion.

Une fois que c'est fait, on installe sur notre téléphone l'application ST BLE toolbox. 

On se connecte sur notre carte :
<img width="369" height="700" alt="image" src="https://github.com/user-attachments/assets/e325896c-31f6-43f3-a811-aec1771d1442" />

puis on rejoins notre service : 

![WhatsApp Image 2026-01-18 at 17 41 05 (1)](https://github.com/user-attachments/assets/b9da0ccb-70c2-45dc-b032-6d7ef8b4ab2d)

On regarde le message reçu :

![WhatsApp Image 2026-01-18 at 17 40 56](https://github.com/user-attachments/assets/966d239f-08e3-4702-9761-caf4a4909d6f)

La valeur hexadécimale affichée est : 

4865-6C6C-6F20-4C61-7572-656E-7420-4669-6163-6B20-6672-6F6D-2061-6E74-6F6E-69-6F2C-204C-6F75-69-732C-204B-656E-6E79-2065-7420-7468-6F6D-6173-2100

Ce qui donne en (ASCII) :

Hello Laurent Fiack from antonio, Louis, Kenny et thomas!

C'est bien le message qu'on avait envoyé !

### Partie FreeRTOS
La section se trouve ici : [code](PCB_A/jeu_chat/PCB_BLE)

#### Fonctionnement : 
Notre code fonctionne avec FreeRtos et nous avons décidé de le faire fonctionner grâce à 4 tâches : [disponible ici](PCB_A/jeu_chat/PCB_BLE/Core/Src/tasks)

 * task_FSM.c : c'est notre machine à état.   (attr.priority   = BASE_PRIO + 4;)
 * task_comm.c : utilisé pour l'écran, l'écran passe de l'image d'une souris à celle d'un chat suivant le mode de jeu choisis. (attr.priority   = BASE_PRIO +  1;)
 * task_control.c : mode chat, définit la façon pour attraper la souris. (Mode souris ici fonctionne, est décrite dans task_FSM, on lui fait faire seulement un carré par manque de temps). (attr.priority   = BASE_PRIO + 5;)
 * task_sensor.c : Lidar + tof ( lidar : attr.priority   = BASE_PRIO + 3; et tof : attr.priority   = BASE_PRIO + 2;)


## Organigramme de décision
<img width="1081" height="369" alt="Organigramme de decision(1)" src="https://github.com/user-attachments/assets/13309a05-b155-4094-a117-cbdac6cd5ec0" />


#### Capteurs et actionneurs : 
La logique de tous nos capteurs se retrouvent ici : [disponible ici](PCB_A/jeu_chat/PCB_BLE/Core/Src/capteurs)

La logique de tous nos actionneurs se retrouvent ici : [disponible ici](PCB_A/jeu_chat/PCB_BLE/Core/Src/actionneurs)

Notre odométrie et notre PID se trouvent dans le fichier suivant : [odométrie + PID](PCB_A/jeu_chat/PCB_BLE/Core/Src/actionneurs/moteur.c)
Dans le même fichier, on y retrouvera 2 fonctions utiles à nos déplacements Robot_Translation et Robot_Rotation qui peuvent être appelées et qui nous servent pour la chasse ainsi que pour la fuite.

Test du carré sur odométrie suffisante : 
https://github.com/user-attachments/assets/7094cc9a-a563-437a-8e9e-69ce3a43d382

mode chat : 
https://github.com/user-attachments/assets/37d389ab-2c22-427e-961a-3d2abfaa119d


