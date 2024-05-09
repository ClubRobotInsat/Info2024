# Paquet ROS2 pour valider les fonctionnalités du code du paquet can_robot  

Création d'un node can_test permettant de prendre un fichier de config yaml pour le transformer en messages à envoyer sur les topics de commande,afin de pouvoir déceler des bugs et faire une batterie de tests

# Pour tuer tous les processus ros2 
 
ps aux | grep -ie ros2 | awk '{print $2}' | xargs kill -9

# Pour lancer le node de validation en utilisant le fichier de configuration yaml   

ros2 run can_robot can_tx
ros2 topic echo /can_raw_tx > test_output.txt
ros2 run can_test can_tx_validation --ros-args -p file_name:=/<ROS_WS>/src/can_test/test/test_can_tx.yaml

Il faut s'assurer de temporiser les envois , sinon ça sature et mène à des comportements indéterminés 

# [TODO]

Améliorer la rapidité des nodes et de la vitesse en général en analysant plus finement les temps d'envoi , de calcul etc ...
