#ifndef MQTT_MANAGER
#define MQTT_MANAGER

#include <PubSubClient.h>
#include "WiFi_Manager.hpp"

// Add your MQTT Broker IP address, example:
const char *mqtt_server = "broker.fkainka.de";
PubSubClient client(espClient);

const char *messTopic = "NodeESP/LoRaMess/values";

void callback(char *topic, byte *message, unsigned int length)
{
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;

    for (int i = 0; i < length; i++)
    {
        Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }
    Serial.println();
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32Mess"))
        {
            Serial.println("mqtt connected");
            // Subscribe
            // client.subscribe("esp32/output");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 1 seconds");
            // Wait 5 seconds before retrying
            delay(1000);
        }
    }
}

void mqtt_loop()
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();
}

void mqtt_init()
{
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    mqtt_loop();
}

void mqtt_send(char *topic, char *msg, bool retain)
{
    client.publish(topic, msg);
}

void mqtt_send(char *msg )
{
    client.publish(messTopic, msg, true);
}
#endif