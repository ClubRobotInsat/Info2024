# Lidar Hokuyo URG-04LX-UG01

Pour utiliser le lidar Hokuyo URG-04LX-UG01 avec ROS2, on peut utiliser le package `urg_node2` qui est une adaptation du package `urg_node` pour ROS2.
Elle est donnée par Hokuyo Automation.
https://github.com/Hokuyo-aut/urg_node2

Elle permet de lire les données du lidar et de les publier sur un topic de type `sensor_msgs/LaserScan`.
En l'occurence, le topic est `/scan` et le type de message est `sensor_msgs/LaserScan`.
https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html


## TroubleShooting test du lidar sur Docker et RViz

Pour connecter le lidar au Docker(Si ça ne marche pas):
https://answers.ros.org/question/286646/error-connecting-to-hokuyo-could-not-open-serial-hokuyo/
```shell
sudo od /dev/ttyACM0
sudo adduser $USER dialout
```

Pour tester le lidar sur RViz:

Ne pas oublier de build, source etc..
Lancer le node du lidar:
```shell
ros2 launch urg_node2 urg_node2.launch.py 
```
Lancer RViz:
```shell
ros2 run rviz2 rviz2
```

Dans RViz, ajouter un topic de type LaserScan et choisir le topic `/scan` pour voir les données du lidar.
Choisissez un frame de référence (ex: laser) pour voir les données du lidar par rapport à ce frame.
N'hesiter pas à changer les paramètres pour voir les données du lidar sous différents angles, distances etc..

Pour voir les données du lidar en ligne de commande:
```shell
ros2 topic echo /scan
```