# Paquet ROS2 pour la communication CAN avec le robot

Ce paquet est utilisé pour la communication CAN avec le robot. 

## Installation

### Prérequis

- Avoir un environnement de développement ROS2 (voir [ROS2 dev environment](https://clubrobotinsat.github.io/doc/informatique/mise_en_place/ros2_dev_container_setup.html))
- Avoir une Raspberry Pi connectée au robot (voir [environnement Raspberry](docs/environnement_raspi.md))
- Avoir des connaissances en Python
- Avoir des connaissances en ROS2
- Avoir des connaissances en communication CAN
- Avoir des connaissances en électronique

Installer les dépendances suivantes:

```bash
sudo apt update
sudo apt install can-utils # Pour les outils de communication CAN
pip install python-can # Pour la communication CAN en Python
```

### Configuration de l'interface CAN

/!\ Il est important de configurer l'interface CAN sur la Raspberry Pi pour pouvoir communiquer avec le robot. /!\

Suivre les étapes suivantes pour configurer l'interface CAN sur la Raspberry Pi: [environnement Raspberry](../docs/environnement_raspi.md#set-up-can-interface)


## Format des messages CAN

:warning: Les messages CAN sont envoyés en utilisant le format standard 11bits. /!\ 

/!\ Il faut s'assurer que le même format est utilisé dans les STM32. /!\

- Le répertoire d'élec soft: [https://github.com/ClubRobotInsat/robot-2023/tree/main/elec-soft](https://github.com/ClubRobotInsat/robot-2023/tree/main/elec-soft)
- Le répertoire de la bibliothèque des drivers: [https://github.com/ClubRobotInsat/librobot2023](https://github.com/ClubRobotInsat/librobot2023)

La structure des messages CAN est définie de cette manière:

<table>
  <tr>
    <td colspan="11" align="center">Header (11bits)</td>
    <td colspan="8" align="center">Data (8 bytes)</td>
  </tr>
  <tr align="center">
    <td>1</td>
    <td>2</td>
    <td>3</td>
    <td>4</td>
    <td>5</td>
    <td>6</td>
    <td>7</td>
    <td>8</td>
    <td>9</td>
    <td>10</td>
    <td>11</td>
    <td>1</td>
    <td>2</td>
    <td>3</td>
    <td>4</td>
    <td>5</td>
    <td>6</td>
    <td>7</td>
    <td>8</td>
  </tr>
  <tr align="center">
    <td colspan="3">Priorité <br/> (3bits)</td>
    <td colspan="4">ID destinataire <br/> (4bits)</td>
    <td colspan="4">ID origine <br/>  (4bits)</td>
    <td>ID Commande <br/>  (1 Byte)</td>
    <td>1st parameter<br/>  (1 Byte)</td>
    <td colspan="6">Paramètres optionnels <br/>  (6 Bytes)</td>

  </tr>
</table>

### Base roulante
| ID commande | 1st param | Opt params | Description                         |
|:-----------:|:---------:|:----------:|:------------------------------------|
|      0      |           |            | stop()                              |
|      1      |           |            | ping()                              |
|      2      |   idMot   |   speed    | setSpeed(idMot, speed)              |
|      3      |   idMot   | direction  | setMotorDirection(idMot, direction) |
|      4      |   idMot   |            | getPosition(idMotor)                |
|      5      |   idMot   |  position  | getPositionACK(idMot, position)     |

### Bras

| ID commande | 1st param | Opt params | Description                 |
|:-----------:|:---------:|:----------:|:----------------------------|
|      0      |           |            | stop()                      |
|      1      |           |            | ping()                      |
|      2      |  idServo  |   angle    | setAngle(idServo, angle)    |
|      3      |  idServo  |            | getAngle(idServo)           |
|      4      |  idServo  |   angle    | getAngleACK(idServo, angle) |
|      5      |   angle   |            | setPliersAngle(angle)       |
|      6      |           |            | getPliersAngle()            |
|      7      |   angle   |            | getPliersAngleACK(angle)    |

### Stockage
| ID commande | 1st param | Opt params | Description                 |
|:-----------:|:---------:|:----------:|:----------------------------|
|      0      |           |            | stop()                      |
|      1      |           |            | ping                        |
|      2      |  idServo  |   angle    | setAngle(idServo, angle)    |
|      3      |  idServo  |            | getAngle(idServo)           |
|      4      |  idServo  |   angle    | getAngleACK(idServo, angle) |
|      5      |  idServo  |   speed    | setSpeed(idServo, speed)    |
|      6      |  idServo  |            | getSpeed(idServo)           |
|      7      |  idServo  |   speed    | getSpeedACK(idServo, speed) |

=> Améliorer pour avoir un truc du style:
- tourner chaine de "tant"
- sensChaine

### Urgence
| ID commande | 1st param | Opt params | Description |
|:-----------:|:---------:|:----------:|:------------|
|      0      |           |            | stop()      |

### Raspi
En fonction des interfaces de chacun => peut interpréter les messages correctement.



## Utilisation

### Lancer le noeud de réception (en développement)

Pour lancer le noeud de réception, exécuter la commande suivante:

```bash
ros2 run can_robot can_rx
```

### Lancer le noeud d'envoi (pas encore développé)

Pour lancer le noeud d'envoi, exécuter la commande suivante:

```bash
ros2 run can_robot can_tx
```