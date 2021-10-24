# *Deep Dive in Internet of Things*

# ![1](DPIoT/20211019_172732000_iOS.jpg)
# ![2](DPIoT/20211019_172744000_iOS.jpg)
# ![3](DPIoT/20211019_172807000_iOS.jpg)
# ![4](DPIoT/20211019_172819000_iOS.jpg)
# ![5](DPIoT/20211019_172833000_iOS.jpg)
# ![6](DPIoT/20211019_172847000_iOS.jpg)
# ![7](DPIoT/20211019_172916000_iOS.jpg)
# ![8](DPIoT/20211019_172930000_iOS.jpg)
# ![9](DPIoT/20211019_172942000_iOS.jpg)
# ![10](DPIoT/20211019_172954000_iOS.jpg)
# ![12](DPIoT/20211019_173305000_iOS.jpg)
# ![13](DPIoT/20211019_175430000_iOS.jpg)
# ![14](DPIoT/20211019_175446000_iOS.jpg)
# ![15](DPIoT/20211019_175508000_iOS.jpg)
# ![16](DPIoT/20211019_175522000_iOS.jpg)
# ![17](DPIoT/20211019_175535000_iOS.jpg)
# ![18](DPIoT/20211019_175553000_iOS.jpg)
# ![19](DPIoT/20211019_175615000_iOS.jpg)
# ![20](DPIoT/20211019_175630000_iOS.jpg)
# ![21](DPIoT/20211019_175641000_iOS.jpg)
# ![21](DPIoT/20211019_175651000_iOS.jpg)
# ![22](DPIoT/20211019_175704000_iOS.jpg)
# ![23](DPIoT/20211019_175715000_iOS.jpg)
# ![24](DPIoT/20211019_175729000_iOS.jpg)
# ![25](DPIoT/20211019_175740000_iOS.jpg)
# ![26](DPIoT/20211019_175753000_iOS.jpg)
# ![27](DPIoT/20211019_175805000_iOS.jpg)
# ![28](DPIoT/20211019_175821000_iOS.jpg)
# ![29](DPIoT/20211019_175833000_iOS.jpg)
# ![30](DPIoT/20211019_175852000_iOS.jpg)
# ![31](DPIoT/20211019_175906000_iOS.jpg)
# ![32](DPIoT/20211019_175928000_iOS.jpg)
# ![33](DPIoT/20211019_175940000_iOS.jpg)
# ![34](DPIoT/20211019_180001000_iOS.jpg)
# ![35](DPIoT/20211019_180043000_iOS.jpg)

# Arduino - Pi - Azure 

## MQTT Protocol MQ Telemetry Transport Protocol (Port 1883)
Is a lightweight protocol to publish and subscribe system where you can publish and receive messages as a client. It is capable of connecting millions of devices and it is know for its scalability and its simple use. It is know in the field of IoT given the distribution of data in low-bandwidth environments.
# ![36](DPIoT/MQTT-Broker-1-2.png)

## Node-RED
An open source tool for building Internet of Things (IoT), making use of low code environment by making use of visual programming and connecting blocks to performs a task.
[Node RED Main Page](https://randomnerdtutorials.com/getting-started-with-node-red-on-raspberry-pi/)
### Installing and Upgrading Node-RED 
[Installation Guide](https://nodered.org/docs/getting-started/raspberrypi)
```bash
bash <(curl -sL https://raw.githubusercontent.com/node-red/linux-installers/master/deb/update-nodejs-and-nodered)
```
# ![37](DPIoT/Node-RED-overview.png)

## Mosquito Broker
In an MQTT protocol the broker is responsible for receving, filtering, and then decidng who is responsible for pbuilching messages to subscribe clinets. Mosquitto is one broker that is installed in the Raspberry Pi.
# ![38](DPIoT/Installing-and-Testing-MQTT-Broker-on-Raspberry-Pi-for-IoT-Communication.png)

### Installing Mosquitto Broker
```bash
pi@raspberry:~ $ sudo apt update
pi@raspberry:~ $ sudo apt install -y mosquitto mosquitto-clients

pi@raspberry:~ $ sudo systemctl enable mosquitto.service
```

### Starting it!
```bash
pi@raspberry:~ $ mosquitto -v
```
### What is my IP?
```bash
pi@raspberry:~ $ hostname -I 
```
# Arduino IDE
## Installing Libraries 
### PubSubClient Library - publish/subscribe messaging with a server that supports MQTT [PubSub Library](https://github.com/knolleary/pubsubclient)
### DHT Sensor Library - an easy way of using any DHT sensor to read temperature and humidity [DHT Library](https://github.com/adafruit/DHT-sensor-library)

## Pin Outs
### ESP8266 Pin Out
# ![38](DPIoT/ESP8266-NodeMCU-kit-12-E-pinout-gpio-pin.png)
### Project Basic Pin Out
# ![39](DPIoT/ESP8266_MQTT_Node-RED_bb-1.png)
```C++
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

/*
 ESP8266 Pin out  https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
 DHT11 Pin out/Basic Code https://create.arduino.cc/projecthub/arcaegecengiz/using-dht11-b0f365
 CCS811 Pin out/Basic Code https://wiki.keyestudio.com/KS2002_EASY_Plug_CCS811_CO2_Air_Quality_Sensor(Black_and_Eco-friendly)
 I2C Scanner https://playground.arduino.cc/Main/I2cScanner/

    **** MQQTT Commands *****

    - MQTT_Public Have Passowrd
      mosquitto_pub -h hostlocal -u mqtt_username -P mqtt_password -t "topic" -m "Hello"
    - MQTT_Subscribe Have Passowrd
      mosquitto_sub -h hostlocal  -u mqtt_username -P mqtt_password -t "topic"
    ---------------------------------------------------------------
    - MQTT_Public No Passowrd
      mosquitto_pub -h hostlocal -t "topic" -m "Hello"
    - MQTT_Subscribe No Passowrd
      mosquitto_sub -h hostlocal -t "topic"
    ================================================================
    -h hostlocal (IP adress raspberry pi)
    -u username MQTT
    -P password MQTT
    -m message (message you want to sent)


    **** ESP8211 Connections - ESP8266 PIN out as reference *****

    - DHT11
    DATA: ESP D0
    -CCS811
    SCL: ESP D1
    SDA: ESP D2
    WAKE: GND
    - Fan
    DATA: ESP D5
      > TRANSISTOR CONNECTION https://hacksterio.s3.amazonaws.com/uploads/attachments/527211/dsc_0280_qGPUj4c7cU.JPG
      Collector: GND of FAN
      Base: ESP D5
      Emmiter: GND to ESP
    - Servo
    DATA: ESP D6
*/


#define DHTTYPE DHT11
#define DHT11_PIN 16 // PIN ESP8266 = D0

DHT dht(DHT11_PIN, DHTTYPE);
float tempSense;
float humSense;

const char* ssid = "ATT6zya9Az";        // wifi ssid
const char* password = "5vb#n=6huf8t";  // wifi password
const char* mqttServer = "192.168.1.209"; // IP address Raspberry Pi
const int mqttPort = 1883;                // MQTT Port number
const char* mqttUser = "CARBON";         // MQQTT User
const char* mqttPassword = "CARBON";     // MQTT Password

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long previousMillis = 0;
unsigned long interval = 10000;

void setup() {


  Serial.begin(115200); // Baud Rate
  Serial.println("System Checks: ");
  
  dht.begin();

  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);
  
  WiFi.begin(ssid, password); // WiFi connection

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    Serial.print(++i); Serial.print(' ');
  }
  Serial.println("Connected to the WiFi network");
  Serial.println('\n');
  Serial.println("Connection established!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
    Serial.println("connected!");
    Serial.println(" ");
    delay(3000);

    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Assurance of message sent to topic in MQTT
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

void loop() {
    tempSense = dht.readTemperature(); // Reading temp
    humSense = dht.readHumidity(); // Reading Hum
    
      client.loop();
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis >= interval) {
      // save the last time you updated the DHT values
      previousMillis = currentMillis;

      if(!isnan(tempSense)&&!isnan(humSense)){
        // Display Serial Monitor
        Serial.println("Temperature (C): " +String(tempSense));
        Serial.println("Humidity: (%): " +String(humSense));;
        Serial.println(" ");

        // LED ACTUATION START

        if(tempSense>=28) { // Paramaters to change Actuation
            Serial.println("High Temperature Dectected detected!!!");
            digitalWrite(2, HIGH);
            digitalWrite(0, LOW);  // 0 = green ; 2 = red
            Serial.println(" ");
            delay(15);
            }

              else {
            Serial.println("Normal Temperature");
            digitalWrite(2, LOW);
            digitalWrite(0, HIGH);  // 0 = green ; 2 = red
            Serial.println(" ");  
              }
            } else {
              Serial.println("Data is not ready!");
              delay(5000);
              }
              
        // LED ACTUATION END

        // Display on MQTT Server
        String toSend = "Temperature(C): " + String(tempSense) + ", Humidity(%) :" +String(humSense);
        client.publish("room1",toSend.c_str()); // Sends Message
        client.subscribe("room1"); // Receives Message
        delay (1000);
      }

} // End of Program
```
