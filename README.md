docker build -t mqtt_server .
docker run -d -p 1883:1883 --name mqtt_container mqtt_server
