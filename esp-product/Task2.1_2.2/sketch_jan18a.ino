#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include<DFRobot_DHT11.h>
#include<ArduinoJson.h>
DFRobot_DHT11 DHT;
#define DHT_PIN 5

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
/*char ssid[] ="xty2005";    // your network SSID (name)
char pass[] = "987654321";    // your network password (use for WPA, or use as key for WEP)
*/
// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate 
//    flashed in the WiFi module.

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
char ssid[] = "ysWIFI";    // your network SSID (name)
char pass[] = "123qweYQH";    // your network password (use for WPA, or use as key for WEP)

const char inTopic[]   = "/sys/k28djpfWciO/esp8266_dev/thing/service/property/set";
const char outTopic[]  = "/sys/k28djpfWciO/esp8266_dev/thing/event/property/post";

const char willTopic[] = "arduino/will";
const char broker[]    = "iot-06z00e5muyhgvpk.mqtt.iothub.aliyuncs.com";
int        port        = 1883;

const long interval = 10000;
unsigned long previousMillis = 0;

int count = 0;
String inputString="";
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // Set pin 4 as OUTPUT
  pinMode(4, OUTPUT);

  /*Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(1000);
  }*/
  
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    
    Serial.println("Connecting to WiFi...");
  }






  Serial.println("You're connected to the network");
  Serial.println();
  mqttClient.setId("k28djpfWciO.esp8266-dev|securemode=2,signmethod=hmacsha256,timestamp=1736948161960|");                    //mqtt 连接客户端id
  mqttClient.setUsernamePassword("esp8266-dev&k28djpfWciO", "548c124a9518c036f409b15978e9f93e2d8270aa73072b7a329b20d27858a33e");    


  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(inTopic);
  Serial.println();

  // subscribe to a topic
  // the second parameter sets the QoS of the subscription,
  // the the library supports subscribing at QoS 0, 1, or 2
  int subscribeQos = 1;

  mqttClient.subscribe(inTopic, subscribeQos);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(inTopic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(inTopic);
  Serial.println();
}

void loop() {
  // call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  mqttClient.poll();

  // to avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
  // see: File -> Examples -> 02.Digital -> BlinkWithoutDelay for more info
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    String payload;

    payload="{\"params\":\"temp\":55,\"humi\":66},\"version\":\"1,0,0\"}";//读取真实的那里要按下面改。//读取真实的那里要按下面改。
    
    
   /* #include<ArduinoJson.h>//放到前面
    DHT.read(DHT11_PIN);
    DynamicJsonDocument json_msg(512);
    DynamicJsonDocument json_data(512);
    json_data["temp"]=DHT.temperature;
    json_data["humi"]=DHT.humidity;
    json_msg["params"]=json_data;
    json_msg["version"]="1,0,0";
    serializejson(json_msg,payload);*/

  





    Serial.print("Sending message to topic: ");
    Serial.println(outTopic);
    Serial.println(payload);

    // send message, the Print interface can be used to set the message contents
    // in this case we know the size ahead of time, so the message payload can be streamed

    bool retained = false;
    int qos = 1;
    bool dup = false;

    mqttClient.beginMessage(outTopic, payload.length(), retained, qos, dup);
    mqttClient.print(payload);
    mqttClient.endMessage();

    Serial.println();

    count++;
  }
}

void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', duplicate = ");
  Serial.print(mqttClient.messageDup() ? "true" : "false");
  Serial.print(", QoS = ");
  Serial.print(mqttClient.messageQoS());
  Serial.print(", retained = ");
  Serial.print(mqttClient.messageRetain() ? "true" : "false");
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    char inChar = (char)mqttClient.read();
    inputString+=inChar;
    if(inputString.length()==messageSize){
      DynamicJsonDocument json_msg(1024);
      DynamicJsonDocument json_item(1024);
      DynamicJsonDocument json_value(1024);
     deserializeJson(json_msg,inputString);
     String items=json_msg["items"];
    deserializeJson(json_item,items);
     String led=json_msg["led"];
    deserializeJson(json_value,led);
   bool value=json_msg["value"];
   if(value==0){
    Serial.println("off");
    digitalWrite(4,LOW);
   }
   else{
    Serial.println("on");
    digitalWrite(4,HIGH);
   }
   inputString="";
    }
  }
  Serial.println();

  Serial.println();
}
