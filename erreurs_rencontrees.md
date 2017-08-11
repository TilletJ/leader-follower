Ce fichier présente les différentes erreurs auxquelles j'ai eu affaire pendant mon projet. J'indique ici les solutions qui ont marché pour moi.

* _EnableMotors.h : no such file or directory_

		sudo apt install ros-kinetic-hector-nav-msgs

* _Markers.h : no such file or directory_

Modifier le CMakeLists.txt dans leader\_follower/src/vicon\_bridge. Commenter les lignes 70 à 73 (inclusivement) avec des # au début des lignes. Recompiler (catkin\_make), puis redécommenter ces lignes. Enfin, recompiler une dernière fois.


* Ne pas hésiter à supprimer le dossier "build" et réessayer ensuite.

* Voici les packets ROS que j'ai pu installer pendant mon travail, je ne saurai en revanche pas dire s'ils sont nécessaires au bon fonctionnement de tous les programmes.

		sudo apt install ros-kinetic-cv-bridge 
		sudo apt install ros-kinetic-opencv3
		sudo apt install ros-kinetic-cv-camera 
		sudo apt install ros-kinetic-hardware-interface 
		sudo apt install ros-kinetic-controller-interface 
		sudo apt install ros-kinetic-gazebo-ros-control 
		sudo apt install python-cv-bridge 
		sudo apt install python-opencv
		sudo apt install libopencv-dev python-numpy python-dev
		sudo -H pip install opencv
