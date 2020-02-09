# super-duper-couscous

Ce projet n'a rien à voir avec le couscous. Ah si, quand on fait cuire du couscous ça consomme de l'énergie !

Un loggeur de la consommation électrique.

# Principe

# Hardware

# Firmware ESP32-wroom32

# Serveur
On utilise divers container Docker pour faire tourner tout les services. Ces containers ont chacun leur volume de données persistées.
On peut déployer tout les containers en se plaçaant dans le dossier `server`, puis avec un `docker-compose up`.

La chaîne de données est :
1. l'ESP32 envoit une mesure au broker MQTT
2. le broker MQTT envoit la mesure au subscriber Node-Red
3. Node-Red inscrit la mesure dans InfluxDB
4. Grafana ira chercher les mesures dans InfluxDB pour afficher des graphes lorsque demandé
