# Paquet ROS2 pour la communication CAN avec le robot

Ce paquet est utilisé pour la communication CAN avec le robot. 

## Table des matières (manuelle)

- [Installation](#installation)
  - [Prérequis](#prérequis)
  - [Configuration de l'interface CAN](#configuration-de-linterface-can)
- [Architecture](#architecture)
- [Liste des messages](#liste-des-messages)
  - [`CanRaw.msg`](#canrawmsg)
- [Utilisation](#utilisation)
  - [Noeud `can_raw_rx`](#noeud-can_raw_rx)
    - [Lancement](#lancement)
    - [Protocole de test](#protocole-de-test)
  - [Noeud `can_rx_decoder` (en cours de développement)](#noeud-can_rx_decoder-en-cours-de-développement)
  - [Noeud `can_raw_tx` (TODO)](#noeud-can_raw_tx-todo)
  - [Noeud `can_tx_encoder` (TODO)](#noeud-can_tx_encoder-todo)
- [Format des messages CAN](#format-des-messages-can)
  - [Base roulante](#base-roulante)
  - [Bras](#bras)
  - [Stockage](#stockage)
  - [Urgence](#urgence)
  - [Raspi](#raspi)
- [Auteurs](#auteurs)

## Installation

### Prérequis

- Avoir un environnement de développement ROS2 (voir [ROS2 dev environment](https://clubrobotinsat.github.io/doc/informatique/mise_en_place/ros2_dev_container_setup.html))
- Avoir une Raspberry Pi connectée au robot (voir [environnement Raspberry](../../docs/environnement_raspi.md))
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

Suivre les étapes suivantes pour configurer l'interface CAN sur la Raspberry Pi: [environnement Raspberry](../../docs/environnement_raspi.md#set-up-can-interface)


## Architecture

Il y a 4 noeuds dans ce paquet:
- `can_raw_rx`: Noeud qui reçoit les messages CAN bruts et les publie sur un topic
- `can_rx_decoder`: Noeud qui reçoit les messages CAN et les interprète
- `can_raw_tx`: Noeud qui envoie les messages CAN bruts
- `can_tx_encoder`: Noeud qui reçoit des commandes et les transforme en message CAN brut

Il y a plusieurs topics dans ce paquet:
- `/can_raw_rx`: Topic pour les messages CAN bruts reçus
- `/can_raw_tx`: Topic pour les messages CAN bruts à envoyer
- TODO: Ajouter les topics pour les messages interprétés

Il y a une interface de communication CAN pour ce paquet:
- `can_interface`: Interface pour la communication CAN

## Liste des messages

### `CanRaw.msg`
```txt
int32 arbitration_id
uint8[8] data
uint8 err_flag
uint8 rtr_flag
uint8 eff_flag

## CanRaw.msg
# @brief A message containing raw CAN data.
# @param arbitration_id The CAN ID of the message
# @param data The raw data of the message
# @param err_flag The error flag of the message (0 = data frame, 1 = error message)
# @param rtr_flag The RTR flag of the message (0 = data frame, 1 = remote frame)
# @param eff_flag The EFF flag of the message (0 = standard frame 11 bits, 1 = extended frame 29 bits)
```

##  Utilisation

### Noeud `can_raw_rx`

#### Lancement
Pour lancer `can_raw_rx`, exécuter la commande suivante:

```bash
ros2 run can_robot can_raw_rx
```

Ne pas oublier les étapes suivantes:
- Configurer l'interface CAN sur la Raspberry Pi (voir [environnement Raspberry](../../docs/environnement_raspi.md#set-up-can-interface))
- Setup ROS2 environment 
- Compiler avec `colcon build --packages-select can_robot` (dans le workspace)
- Source le workspace avec `source install/setup.bash` (dans le workspace)
- Lancer le noeud avec `ros2 run can_robot can_raw_rx`

#### Protocole de test

1. Lancer le noeud `can_raw_rx` avec la commande `ros2 run can_robot can_raw_rx`
2. Ecouter le topic `can_raw_rx` avec la commande `ros2 topic echo /can_raw_rx`
3. Envoyer un message CAN avec la commande `cansend can0 013#02.01.60.00.00.00.00.00`
4. Vérifier que le message est bien reçu par le noeud `can_raw_rx` 
5. Vérifier que le message est bien affiché dans le terminal où le topic est écouté

### Noeud `can_rx_decoder` (en cours de développement)

Pour lancer le noeud de réception, exécuter la commande suivante:

```bash
ros2 run can_robot can_rx
```

### Noeud `can_raw_tx` 

#### Lancement

#### Protocole de test 

1. Lancer le noeud `can_raw_tx` avec la commande `ros2 run can_robot can_raw_tx`
2. Lancer sur un autre terminal ou en arrière plan `candump can0`
3. Lancer sur un autre terminal ou en arrière plan `ros2 topic pub -r 1 /can_raw_tx can_raw_interfaces/msg/CanRaw '{arbitration_id: 19,data: [2,1,96,0,0,0,0,0], err_flag: 0, rtr_flag: 0, eff_flag: 0}'` pour envoyer des données sur le topic /can_raw_tx
4. candump devrait afficher les données captées sur le bus CAN

#### Astuces

Pour tester un node, on peut par exemple, lancer ce node, puis faire `ctrl+z` + `bg` pour faire tourner le processus en arrière plan. Ensuite, il est possible de publier une donnée directement sur le terminal, en utilisant une commande comme la suivante : 
`ros2 topic pub </nom_du_topic> <nom de l'interface message> <données à publier sur le topic>`
Le données doivent être sous format YAML. Prenez exemple sur la commande suivante : `ros2 topic pub /can_raw_tx can_raw_interfaces/msg/CanRaw '{arbitration_id: 0,data: [0,0,0,0,0,0,0,0], err_flag: 0, rtr_flag: 0, eff_flag: 0}'`

### Noeud `can_tx`

#### Lancement

#### Protocole de test

1. Lancer le noeud `can_tx` avec la commande `ros2 run can_robot can_tx`
2. Ecouter le topic `<Nom du topic sur lequel can_raw_tx envoie les trames CAN>` avec la commande `ros2 topic echo <Nom du topic sur lequel can_raw_tx envoie les trames CAN>` 
3. Lancer un script de test qui envoie un ensemble de messages corrects et de messages erronés
4. Vérifier que les messages en sortie de can_tx correspondent aux messages envoyés sur les topics en entrée (Il y en a 4, /storage_cmd, /arm_cmd, /motor_cmd, /sensor_cmd), et que les messages erronés n'ont pas été convertis en trame CAN.
5. Un script de test est en train d'être élaboré, il se nomme test_can_tx.sh [Où le mettre de préférence]

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

### Capteurs

| ID commande | 1st param | Opt params | Description                 |
|:-----------:|:---------:|:----------:|:----------------------------|
|      0      |           |            | stop()                      |
|      1      |           |            | ping                        |


=> Améliorer pour avoir un truc du style:
- tourner chaine de "tant"
- sensChaine

### Urgence
| ID commande | 1st param | Opt params | Description |
|:-----------:|:---------:|:----------:|:------------|
|      0      |           |            | stop()      |

### Raspi
En fonction des interfaces de chacun => peut interpréter les messages correctement.

## Auteurs

- [Ronan Bonnet](https://github.com/BloodFutur)
- [Liam Chrisment](https://github.com/LiamKaist)