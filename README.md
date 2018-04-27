# cis497

For CIS497 (DMD senior project) at University of Pennsylvlania

Working from <a href="http://github.com/mattoverby/admm-elastic" target="_blank">ADMM code</a> with some modifications to allow collision objects to move.

Note I built the code on a Linux machine (build dir in admm-elastic).
The files I wrote to run are in the cis497 folder; I also added code to src/Collider.hpp (to add variables controlling motion to a passive collision object), cis497/utils/Application.hpp (to update passive object meshes every time step), and a few other files (primarily to be able to access the motion variables given a PassiveMesh, as in Application.hpp).
The motion variables are initially set to a vector of three 0's attached to a PassiveCollision; they are also public so to change them one would access the desired variable through the collider.
Everything else is mostly the same from the original code.
