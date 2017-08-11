Présentation
============

Ceci est le projet de Joris Tillet, stagiaire de juin à août 2017 au MRASL à l'École Polytechnique de Montréal, encadré par David Saussié.
L'objectif est d'avoir un drone "follower" autonome, qui suit le "leader".
Le leader est soit commandé manuellement, soit a une consigne et la suit de manière autonome.
Le follower se place derrière le leader (aligné par rapport à son cap), à une certaine distance, et le suit.

Environnement de travail utilisé
--------------------------------

J'ai uniquement travaillé sur ROS kinetic (desktop full), sur la version 16.04 d'Ubuntu.

Packets utilisés et packets créés
---------------------------------

J'utilise plusieurs packets ROS qui existaient déjà :

* crazyflie\_ros [wiki](http://wiki.ros.org/crazyflie) [github](https://github.com/whoenig/crazyflie_ros)
* falkor\_ardrone [github](https://github.com/FalkorSystems/falkor_ardrone)
* hector\_quadrotor [wiki](http://wiki.ros.org/hector_quadrotor) [github](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor)
* imu\_tools [wiki](http://wiki.ros.org/imu_filter_madgwick) [github](https://github.com/ccny-ros-pkg/imu_tools)
* vicon\_bridge [wiki](http://wiki.ros.org/vicon_bridge) [github](https://github.com/ethz-asl/vicon_bridge) _attention : a été modifié localement_

Ces packets n'ont pas été modifié, vous pouvez les retélécharger tels quels.
Seul vicon_bridge a été modifier pour se connecter correctement à l'installation présente au MRASL.

J'ai créé les packets suivants :

* simulation
* crazyflie2
* ardrone


Installation
============

Ce package utilise le packet `ardrone_autonomy`, il faut donc l'installer avec la commande :

		sudo apt install ros-kinetic-ardrone-autonomy 

Puis se placer à la racine du workspace (leader-follower), et lancer la commande `catkin_make`.
Si des erreurs apparaissent, se référer au fichier "erreurs_rencontrees.md"

Attention : il faut modifier le fichier `leader-follower/src/simulation/launch/labo_slam_gazebo_2_uav.launch` : il faut mettre le bon chemin à la ligne 6.
Remplacer `/home/joristillen/Desktop` par le dossier où se trouve le workspace.


Simulation
==========

Dans un terminal lancé dans le workspace (ou `cd .../leader-follower`) lancer cette commande pour sourcer le terminal (nécessaire à chaque fois que vous relancez un terminal) :

		source devel/setup.bash

Puis, lancer les 2 commandes suivantes :

		roslaunch leader_follower labo_slam_gazebo_2_uav.launch 
		roslaunch leader_follower leader_follower_auto.launch 


La premiere lance gazebo dans le world de la voliere du labo, avec 2 drones et 2 boules. La boule blanche représente l'objectif du leader, tandis que la grise représente celui du follower.
La deuxieme commande déplace les objectifs (boules blanche et grise) et lance les commandes pour que les drones suivent leurs objectifs.

Vous pouvez modifier le fichier commande\_leader.cpp dans src/simulation/src/ pour changer la consigne du leader (ne pas oublier de recompiler avec `catkin_make` à la racine du workspace avant de relancer le programme). Actuellement, la consigne suit une courbe de Lemniscate (symbole infini). Une courbe de Lissajoux ou un cercle sont proposés, il suffit de décommenter la courbe souhaitée.

Dans le fichier commade\_follower.cpp, au même endroit, vous pouvez changer la valeur de `d` qui correspond à la distance à laquelle le follower doit être par rapport au leader.


Crazyflie2
==========



Ardrone
=======






























