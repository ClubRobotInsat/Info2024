# Environnement Raspberry Pi 4

**Table des matières (manuelle)**

- [Les identifiants](#les-identifiants)
- [Set up une Raspberry vierge](#set-up-une-raspberry-vierge)
  - [Création de l'image à flasher sur la carte SD](#création-de-limage-à-flasher-sur-la-carte-sd)
  - [Se connecter à la raspberry avec ton PC](#se-connecter-à-la-raspberry-avec-ton-pc)
  - [Mise à jour de l'OS et installation d'outils](#mise-à-jour-de-los-et-installation-doutils)
  - [Set up CAN interface](#set-up-can-interface)
  - [Démarrage automatique de l'interface CAN](#démarrage-automatique-de-linterface-can)
  - [Comment accélerer le démarrage (3minutes -> 30 secondes)](#comment-accélerer-le-démarrage-3minutes---30-secondes)
  - [Installation de ROS2](#installation-de-ros2)
- [Utilisation d'une Raspberry déjà configurée](#utilisation-dune-raspberry-déjà-configurée)
  - [Se connecter à la Raspberry par le réseau](#se-connecter-à-la-raspberry-par-le-réseau)
  - [Comment transférer des données?](#comment-transférer-des-données)


## Les identifiants

- username: pi
- password: robotcdf

## Set up une Raspberry vierge

### Création de l'image à flasher sur la carte SD

1. Sur votre PC installer rpi-imager: `sudo snap install rpi-imager`.
2. Lancer `rpi-imager`
3. Sélectionner Ubuntu 22.04 (Server pour avoir + de ressources)
4. Configurer pour avoir le SSH
5. Configurer votre Réseau Wi-Fi (utiliser votre partage de connexion téléphone)
6. Démarrer la raspberry et vous devriez pouvoir vous connecter et avoir une connexion Internet

### Se connecter à la raspberry avec ton PC

1. Activer le partage de connexion sur le téléphone
2. Se connecter à eduroam sur le téléphone (pour éviter l'utilisation de donnée perso)
3. Chercher l'adresse IP de la raspberry (Courage!)
4. Se connecter au partage de connexion sur ton PC
5. Se connecter à la Raspberry en SSH: `ssh pi@192.168.43.143` **Changer l'adresse IP avec la bonne**

### Mise à jour de l'OS et installation d'outils

1. Mettre à jour: `sudo apt update && sudo apt upgrade`
2. Installer outil de réseau: `sudo apt install -y net-tools`

### Set up CAN interface

1. Mettre à jour: `sudo apt update && sudo apt upgrade`
2. Installer can-utils: `sudo apt install -y can-utils`
3. Mettre à jour la configuration firmware:

    Ajouter les lignes suivantes à la fin de `/boot/firmware/config.txt`:
```txt
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
```

Le paramètre oscillator correspond au quartz du CAN Transceiver (peut-être différent de 8000000, 16000000 pour PiHat)
Le paramètre interrupt correspond au PIN sur la Raspberry qui gère l'interruption CAN

4. Tester si l'interface peut-être activée: `sudo ip link set can0 up type can bitrate 125000`

En faisant `ip a`, vous devriez voir l'interface can0 'UP'.
Le bitrate correspond au débit du bus CAN, ici 125kB/s

5. Déboguer l'interface CAN

Regarder si le module est bien chargé: `lsmod | grep can`
`sudo dmesg | grep can` ou `sudo dmesg | grep mcp`

Si vous avez l'erreur `mcp251x spi0.0: Cannot initialize MCP2515. Error=110`. Alors il y a un problème électronique.

### Démarrage automatique de l'interface CAN

Source: https://askubuntu.com/questions/1337826/18-04-server-with-can-bus-and-netplan-configuration

1. Ajouter le module can dans ce fichier: `/etc/modules`

Modifier le fichier avec votre éditeur préféré: `sudo nano /etc/modules`.
Ajouter les lignes suivantes:
```txt
# /etc/modules: kernel modules to load at boot time.
#
# This file contains the names of kernel modules that should be loaded
# at boot time, one per line. Lines beginning with "#" are ignored.
can
can-dev
```

2. Créer un fichier sous `/etc/systemd/network` nommé `can.network` avec le contenu suivant:
```txt
[Match]
Name=can0

[CAN]
BitRate=125000

[Link]
ActivationPolicy=up
```

3. Redémarrer et vérifier que l'interface est 'UP':
```bash
sudo reboot
```

```bash
ip a
```

4. Tester l'interface en envoyant et recevant des données CAN

Dans un terminal: `candump can0`, puis dans un autre: `cansend can0 123#1122334455667788`.

Vous devriez voir le message apparaitre dans le terminal avec candump.

Si ça ne marche pas, assurer vous que l'électronique est correcte. [Documentation/Communication avec le hardware](https://clubrobotinsat.github.io/doc/informatique/communication.html).


### Comment accélerer le démarrage (3minutes -> 30 secondes)

On désactive certains services pas très utiles:
```bash
sudo systemctl disable systemd-networkd-wait-online.service
sudo systemctl mask systemd-networkd-wait-online.service
```

### Installation de ROS2

Je te conseille de suivre le tuto.
Source: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

#### Choisir la locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

#### Setup sources

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Installation des paquets ROS2

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

#### Setup de l'environnement

```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```
Je te conseille de rajouter cette ligne dans le `~/.bashrc` à la fin.

#### Création d'un espace de travail ROS2

1. Source ROS2 environment
```bash
source /opt/ros/humble/setup.bash
```
2. Créer un espace de travail
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

3. Cloner un example pour tester
```bash
git clone https://github.com/ros/ros_tutorials.git -b humble`
```

4. Resoudre les dependances
```bash
cd .. 
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
```

5. Compiler l'espace de travail avec colcon
From the root of the workspace (~/ros2_ws), run the following command to build the workspace:
```bash
colcon build
```

**Je te conseille de compiler sur ton PC puis mettre les fichier compiler sur la Raspberry Pi.**
Si t'es pas convaincu, tu changeras rapidement d'avis face au temps de compilation sur la Raspi.

## Utilisation d'une Raspberry déjà configurée

### Se connecter à la Raspberry par le réseau

**Si tu connais le réseau sur lequel la Raspberry a été configurée**
1. Allumer le partage de connexion configuré pour la Raspi (si téléphone, se connecter en même temps à un wifi pour sauvegarder des données mobiles)
2. S'y connecter avec son PC
3. Chercher l'adresse IP de la raspberry (Courage!)
4. Se connecter à la Raspberry en SSH: ssh pi@192.168.43.143 Changer l'adresse IP avec la bonne

### Comment transférer des données

On utilise la commande `scp`. Exemple:
`scp -r ROS_playground pi@192.168.43.143:/home/pi/ros2_ws/`.

Cette commande envoie le dossier ROS_playground de manière récursive à la Raspi dans le dossier `/home/pi/ros2_ws/`.


On peut également utilisé `rsync` pour synchroniser des dossiers. Exemple:
`rsync -urltv --delete -e ssh /home/ronan/ClubRobot/Coupe_de_France/Info2024/* pi@192.168.43.115:/home/pi/ros2_ws`

