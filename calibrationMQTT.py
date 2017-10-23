import paho.mqtt.client as mqtt
import time
import sys

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("/gyroscope/mesures/Calibration")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    try:
        print(msg.topic+" "+str(msg.payload))

        with open("calibrationData.txt", "w") as myfile:
            myfile.write(msg.payload)
    except UnicodeDecodeError:
        print("oups");

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("pil-01.univlehavre.lan", 1883, 60)

if sys.argv[1] == "set" :
    while(True):
        with open("calibrationData.txt", "r") as myfile:
            message=myfile.readlines()
        print(message)
        client.publish("/gyroscope/mesures/Calibration", payload=message[0], qos=0, retain=False)
        time.sleep(1)
elif sys.argv[1] == "get" :
    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    client.loop_forever()
else :
    print("erreur args")
