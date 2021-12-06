Ho caricato due versioni dell'homework all'interno della cartella tutorials.

La prima versione prevede tre server distinti:
1. tutorial_draw_circle_server.cpp
2. tutorial_delete_circle_server.cpp
3. tutorial_get_circle_server.cpp
e i due client:
1. tutorial_draw_circle.cpp
2. tutorial_delete_circle.cpp
Questa versione non prevedeva il client di getCircles in quanto tale servizio è chiamato
all'interno di tutorial_delete_circle. Usando tre server distinti avevo il problema di
dover comunicare le modifiche apportate a circles (l'array di Circle) da un nodo all'altro,
perciò ho realizzato una seconda versione dell'homework.

La seconda versione, senz'altro migliore della precedente, prevede un singolo server, che
fornisce tutti e tre i servizi, e tre client distinti. Questa versione permette di avere un
array di circles globale che mi permette di sapere quali sono ad ogni istante le tartarughe
ancora in vita, accessibile sia dalla draw_circle (che vi aggiungerà le tartarughe man mano
che le spawna) che dalla delete_circle (che cancellerà le tartarughe dall'array man mano che
la tartaruga iniziale le colpisce).

Per testare l'homework è quindi necessario compilare i file, usando catkin build, e aprire:
1. Un terminale sul quale eseguiamo il comando 
	>>roscore
2. Un terminale sul quale eseguiamo il comando 
	>>rosrun turtlesim turtlesim_node
3. Un terminale sul quale avviamo il server con il comando 
	>>rosrun turtlesim tutorial_single_server
4. Un terminale sul quale eseguiamo 
	>>rosrun turtlesim turtle_teleop_key
   che ci permetterà di muoverci con la tartaruga per testare la delete.
5. Un terminale sul quale eseguire i diversi client mediante i comandi:
	>>rosrun turtlesim tutorial_draw_circle
	>>rosrun turtlesim tutorial_get_circle
	>>rosrun turtlesim tutorial_delete_circle
