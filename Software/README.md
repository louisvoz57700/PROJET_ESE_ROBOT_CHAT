## PCB_A : version WB55
La section Software/PCB_A/Preuve_BLE/BLE est la section de notre code de preuve de notre antenne Bluetooth intégrée à notre PCB. Tout fonctionne !

Ce qu'on a fait : on a suivi la vidéo suivante : STM32 Bluetooth Firmware Tutorial (Bring-Up) - Phil's Lab #129

Les étapes : aller sur stm32CubeProgrammer et on upgrade les firmware, on n'oublie pas de start le wireless stack sinon on ne verra pas notre carte lors de la connexion bluetooth.

Puis sur stm32CubeIde, on va aller créer un service qui peut écrire et lire.

On rajoute une fonction pour interpréter les notifications (la fonction est ici : [custom_app.c](Software/PCB_A/Preuve_BLE/BLE/STM32_WPAN/App/custom_app.c))

Rajoutons un message : "Hello Laurent Fiack from antonio, Louis, Kenny et thomas!", qu'on va lire sur notre téléphone pour tester la connexion.

Une fois que c'est fait, on installe sur notre téléphone l'application ST BLE toolbox. 

On se connecte sur notre carte :

![WhatsApp Image 2026-01-18 at 17 41 05](https://github.com/user-attachments/assets/aea99bb4-5753-4f7f-946c-b81bd6db133d)

puis on rejoins notre service : 

![WhatsApp Image 2026-01-18 at 17 41 05 (1)](https://github.com/user-attachments/assets/b9da0ccb-70c2-45dc-b032-6d7ef8b4ab2d)

On regarde le message reçu :

![WhatsApp Image 2026-01-18 at 17 40 56](https://github.com/user-attachments/assets/966d239f-08e3-4702-9761-caf4a4909d6f)

La valeur hexadécimale affichée est : 

4865-6C6C-6F20-4C61-7572-656E-7420-4669-6163-6B20-6672-6F6D-2061-6E74-6F6E-69-6F2C-204C-6F75-69-732C-204B-656E-6E79-2065-7420-7468-6F6D-6173-2100

Ce qui donne en (ASCII) :

Hello Laurent Fiack from antonio, Louis, Kenny et thomas!

C'est bien le message qu'on avait envoyé !

## PCB_B : version G431 + module bluetooth externe
