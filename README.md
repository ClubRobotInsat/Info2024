# Info 2024

Ce répertoire sert à contenir tous le code et les informations nécessaires qui sera dans la Raspberry Pi du robot pour la Coupe de France de Robotique 2024.

## Table des matières (manuelle)

- [Prérequis](#prérequis)
- [Architecture du robot](#architecture-du-robot)
  - [Paquets ROS2](#paquets-ros2)
- [Comment utiliser ce répertoire](#comment-utiliser-ce-répertoire)
- [Comment avoir un environnement de travail de qualité?](#comment-avoir-un-environnement-de-travail-de-qualité)
- [Comment utiliser ROS2 dans ce répertoire?](#comment-utiliser-ros2-dans-ce-répertoire)
- [Erreurs bizarres](#erreurs-bizarres)
  - [Setup.py is deprecated](#setuppy-is-deprecated)
  - [No modules named 'blablabla'](#no-modules-named-blablabla)
- [Ressources potentielles](#ressources-potentielles)
- [Auteurs](#auteurs)

## Prérequis

- Docker
- Git
- Des connaissances en Python
- Les bases de ROS2
- Connaitre l'architecture du robot

## Architecture du robot

TODO: Ajouter un schéma de l'architecture du robot

### Paquets ROS2

- [x] can_robot: Paquet ROS2 pour la communication CAN avec le robot. (Voir [can_robot/README.md](src/can_robot/README.md))
- [ ] Simulation
- [x] Téléopération (Voir [teleop/README.md](src/teleop_twist_keyboard/README.md)
- [ ] Navigation du robot
- [ ] Déplacement du bras
- [ ] Localisation
- [ ] Cartographie
- [ ] Etat du robot
- [ ] Évitement d'obstacles
- [ ] Vision
- [x] Lidar (Voir [lidar/README.md](docs/lidar.md) et [lidar/urg_node2/README.md](src/urg_node2/README.md))
- [ ] Intelligence artificielle & Stratégie


## Comment utiliser ce répertoire

1. Télécharger le répertoire : `git clone git@github.com:ClubRobotInsat/Info2024.git`.
2. Créer un container de développement,  [voir documentation 'ROS2 dev environment'](https://clubrobotinsat.github.io/doc/informatique/mise_en_place/ros2_dev_container_setup.html)
3. Lancer le container
4. Vérifier qu'il est bien lancé `docker container ls`
5. Se connecter au container `docker exec -it 491a065c09de /bin/bash` (491a065c09de est l'ID de container récupéré avant)
6. Vous êtes près à développer du ROS2

Les fichiers entre le container et le repository sont partagés.

## Comment avoir un environnement de travail de qualité?

- Avoir un Raspberry Pi connecté au robot ou à defaut, un setup minimal pour le developpement de la fonctionnalité
- Lire la [documentation pour environnement Raspberry](docs/config_environnement_raspi.md)

## Comment utiliser ROS2 dans ce répertoire?

- Ouvrir son containeur et aller dans le répertoire suivant: `cd /home/ws/src/`.
- Compiler avec Colcon: `colcon build`.
- Sourcer: `source install/local_setup.bash`.
- Lancer vos nodes ROS2: `ros2 run can_robot can_rx` (Exemple)

## Erreurs bizarres

### Setup.py is deprecated

TODO: Corriger ça directement dans le DockerFile et Tester

Installer ce package python dans le container: `pip install setuptools==58.2.0`
L'erreur devrait disparaitre. 

### No modules named 'blablabla'

Cette erreur peut arriver de différentes manières.

1. Le module n'est pas installé, l'installer avec pip: `pip install python-can`
2. Tu as créé un package ROS qui a le même nom qu'un module python que tu utilises, exemple "can" : utilisé par python-can. => Change le nom de ton package ROS2.

### Package à installer

- python-can
- ros-humble-teleop-twist-keyboard
- ros-humble-gazebo-ros-pkgs
- ros-humble-gazebo-ros2-control
- ros-humble-gazebo-dev
- ros-humble-xacro

## Ressources potentielles

- Documentation ROS2 Humble: https://docs.ros.org/en/humble/index.html
- Subramanian, R. (2023). Robot Simulation and Visualization. In: Build Autonomous Mobile Robot from Scratch using ROS. Maker Innovations Series. Apress, Berkeley, CA. https://doi.org/10.1007/978-1-4842-9645-5_5
- LinoRobot2, 4WD ROS2 package, https://github.com/linorobot/linorobot2
- Nav2, ROS2 Navigation package, https://navigation.ros.org
- PyMoveIt2, (Package for Robotic Arms), https://github.com/AndrejOrsula/pymoveit2

## Auteurs

- [Ronan Bonnet](https://github.com/BloodFutur)