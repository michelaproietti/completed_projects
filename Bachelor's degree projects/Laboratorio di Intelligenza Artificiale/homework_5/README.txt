All'interno della cartella src c'è il file move_base_client, che salva la posizione iniziale del robot, aspetta che l'action server sia pronto per soddisfare una sua richiesta, dopodiché invia un goal all'action server, aspetta 30 secondi e poi cancella la propria richiesta. Dopodiché, invia un nuovo goal con le coordinate della posizione iniziale che avevamo salvato all'inizio e aspetta di ricevere il risultato della propria richiesta dall'action server, dopodiché termina.

Per poter eseguire il move_base_client è necessario:
1. Avviare roscore
2. Avviare stage_ros eseguendo il comando rosrun stage_ros stageros worlds/willow-erratic.world
3. Avere a disposizione una mappa del mondo da condividere con tutti i nodi ROS tramite il
   comando rosrun map_server map_server mappa.yaml
4. Avviare il localizer e il planner tramite:
	rosrun thin_navigation thin_localizer_node
	rosrun thin_navigation thin_planner_node
5. Avviare rviz e aggiungere la mappa, poseArray e path alla configurazione
6. Scrivere il comando "rostopic pub /initialpose tab tab" e completare la struttura che si ottiene
 mettendo:
	position: x=0.01, y=0.01
	orientation: w=6.12323399574e-17 (valore ottenuto tramite rostopic echo /base_pose_ground_truth)
   In questo modo settiamo correttamente la posizione iniziale del robot su rviz.
7. Avviare il nodo con rosrun actionlib_tutorials move_base_client

Il robot riceverà quindi un goal, inizierà a spostarsi verso di esso, poi si fermerà e riceverà un nuovo goal per tornare alla sua posizione iniziale. 
