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

Je n'ai jamais eu l'occasion de faire voler simultanément 2 Crazyflies dans la volière (par manque de boules réfléchissantes du VICON). Il n'y a donc pas de fichier .launch tout prêt pour lancer cette expérience, mais avec tout le code de l'archive, on peut le faire relativement simplement.

En revanche, on peut faire voler un Crazyflie seul, ou un Crazyflie en tant que leader et un AR drone en tant que follower. Le drone doit avoir son modèle enregistré dans le système VICON sous le nom de `cf_leader`, ou alors modifier le code en conséquence.

Dans tous les cas, il faut lancer un terminal, faire le `source devel/setup.bash` à la racine du workspace, et lancer la commande suivante :

		roslaunch crazyflie_driver crazyflie_server.launch

On peut rajouter à la fin de cette commande `uri:=radio://0/80/2M` pour préciser à quel drone on souhaite se connecter, si ambiguïté il y a.

* Ensuite, si l'on souhaite juste faire voler le Crazyflie seul, et le faire suivre sa commande (en tant que leader donc), il faut lancer la commande suivante (toujours dans un terminal avec la commande `source devel/setup.bash`) :

		roslaunch crazyflie2 all.launch

Puis, lancer rqt, et publier quelques messages 'cmd\_vel' pour qu'il se mette en route (je ne sais pas pourquoi cela est necessaire). Ne pas oublier d'arrêter cette publication rapidement.

* Pour rajouter un AR Drone en follower, il y a 3 étapes :

Vérifier dans le fichier `src/simulation/src/commande_follower.cpp`qu'à la ligne 34, colone 42, le topic écouté est bien `/vicon/cf_leader/cf_leader` (ou celui du modèle VICON). Sinon corriger la ligne et recompiler (`catkin_make` à la racine du workspace).

Lancer la commande suivante :

		roslaunch ardrone lf_ar_cf.launch

Enfin, lancer rqt, et publier quelques messages 'cmd\_vel' pour que le Crazyflie se mette en route (je ne sais pas pourquoi cela est necessaire). Ne pas oublier d'arrêter cette publication rapidement. Et publier un `empty_msg` sur le topic `/uav2/ardrone/takeoff` pour faire décoller l'AR drone.


Ardrone
=======

Ce packet permet d'appliquer le projet leader-follower sur des Parrot AR Drones (1.0 et/ou 2.0). Il faut donc prendre 2 drones, et faire en sorte qu'ils se connectent à un routeur commun. Au MRASL, les 3 drones actuellement fonctionnels ont déjà été hacké pour pouvoir se connecter au réseau `mrasl_2.4`. Sinon, possibilité de suivre [ce tuto](https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones).

Dans mes tests, le ardrone 2.0 était le leader, et se connectait avec l'adresse ip fixe 192.168.1.16. Les 2 autres drones se connectaient avec les adresses 192.168.1.14 et 192.168.1.15. Actuellement, le programme cherche à communiquer avec les adresses en .15 et .16, si la configuration est différente, il faut modifier le code dans le .launch.

Sur le VICON Tracker, il faut que le système suive les 2 modèles : `ardrone_leader` et `ardrone_follower`. Il faut également que le système VICON soit connecté au même routeur que les drones, ou en tout cas, que la machine sur laquelle les codes seront exécutés soit connectée au routeur et au système VICON.

Vérifier dans le fichier `src/simulation/src/commande_follower.cpp`qu'à la ligne 34, colone 42, le topic écouté est bien `/vicon/ardrone_leader/ardrone_leader` (ou celui du modèle VICON). Sinon corriger la ligne et recompiler (`catkin_make` à la racine du workspace).

En premier lieu, sur votre machine, se connecter successivement aux drones qui vont être utilisés et lancer cette commande sur les deux pour qu'ils se connectent au routeur :

		echo "./data/wifi.sh" | telnet 192.168.1.1

Puis se reconnecter au routeur. On peut vérifier qu'ils se sont bien connectés en envoyant des ping vers leurs adresses :

		ping 192.168.1.15

Lorsque tout est prêt, on peut lancer le fichier launch dans un terminal (avec la commande `source devel/setup.bash` à la racine du workspace avant tout) :

		roslaunch ardrone leader_follower.launch

Les objectifs sont publiés, et les commandes sont envoyées. Il ne reste plus qu'à faire décoller les drones : il faut publier un empty message sur les topics `/uav1/ardrone/takeoff` et `/uav2/ardrone/takeoff`. `uav1` correspond au leader et `uav2` au follower. À tout moment, vous pouvez les faire atterrir avec le même message sur le topic `/uavX/ardrone/land` ou couper les moteurs en cas d'urgence grâce au topic `/uavX/ardrone/reset`. Le plus fonctionnel est d'utiliser `rqt` pour publier ces messages.

Ne pas oublier de vérifier le niveau de batterie des drones régulièrement : le pourcentage restant est publié sur le topic `uavX/ardrone/navdata`.
