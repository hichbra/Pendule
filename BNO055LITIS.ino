
/*
 * AVERTISSEMENT - ATTENTION
 * Ceci est en cours de développement
 * Il faut rajouter la température et le magnétomètre
 * 
 * Il est possible de configurer le capteur pour ne pas avoir de la fusion de données et dans ce cas obtenir des mesures "brutes"
 * 
 */


#define NODEBUG

// Def WIFI
#define AILEG//#define HICHAM//#define NOTAILEG

#define NOTESTPENDULE

// Def MQTT
#define NOTSERVEURAILEG

// Calibrage
#define NOCALIBRAGE

// Set 2G
//#define BNO055_ACC_CONFIG_ADDR 0x08 

// Pour dater les mesures
//#include <time.h>

// Pour le bus I2C
#include <Wire.h>

// Pour le gyroscope
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Pour l'ESP8266
#include <ESP8266WiFi.h>

// Pour publier les info via MQTT (MQ Telemetry Transport)
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <utility/imumaths.h>

#ifdef DEBUG
 #define DEBUG_BEGIN(x)        Serial.begin (x)
 #define DEBUG_PRINT(x)        Serial.print (x)
 #define DEBUG_PRINTLN(x)      Serial.println (x)
#else
 #define DEBUG_BEGIN(x)       
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

/************************* Pour le WIFI  *********************************/
#ifdef AILEG
const char* SSID = "CTheory";
const char* PASSWORD = "TheButterflyEffect";
#elif defined HICHAM
const char* SSID = "FREEBOX_MOHAMMED_X3";
const char* PASSWORD = "F4CAE55AF478";
#else
const char* SSID = "PENDULE-4G-2017";
const char* PASSWORD = "PanDule2Foucault";
#endif

WiFiClient client;
#ifdef DEBUG
byte mac[6];
#endif 

/************************* Pour MQTT      ********************************/
#define DELAI_RECONNECTION 5000          // em ms

#ifdef SERVEURAILEG
  #define SERVER "192.168.1.4"
  #define SERVERPORT 8883
  #define USERNAME "PenduleDeFoucault"
  #define MQTTPASSWORD "Gyro2017"
#elif defined HICHAMQTT
  #define SERVER "test.mosquitto.org"
  #define SERVERPORT 1883
  #define USERNAME ""
  #define MQTTPASSWORD "" 
#else
  #define SERVER "pil-01.univlehavre.lan"
  #define SERVERPORT 1883
  #define USERNAME ""
  #define MQTTPASSWORD ""  
#endif


Adafruit_MQTT_Client mqtt(&client, SERVER, SERVERPORT, USERNAME, MQTTPASSWORD);
Adafruit_MQTT_Publish anglesEuler = Adafruit_MQTT_Publish(&mqtt, USERNAME "/gyroscope/mesures/Euler");
Adafruit_MQTT_Publish calibrationMQTT = Adafruit_MQTT_Publish(&mqtt, USERNAME "/gyroscope/mesures/setCalibration");
Adafruit_MQTT_Publish setCalibrationMQTT = Adafruit_MQTT_Publish(&mqtt, USERNAME "/gyroscope/mesures/Calibration");
Adafruit_MQTT_Subscribe getCalibrationMQTT = Adafruit_MQTT_Subscribe(&mqtt, USERNAME "/gyroscope/mesures/Calibration");

#ifdef TESTPENDULE
Adafruit_MQTT_Publish surX = Adafruit_MQTT_Publish(&mqtt, USERNAME "/gyroscope/mesures/Euler/x");
Adafruit_MQTT_Publish surY = Adafruit_MQTT_Publish(&mqtt, USERNAME "/gyroscope/mesures/Euler/y");
Adafruit_MQTT_Publish surZ = Adafruit_MQTT_Publish(&mqtt, USERNAME "/gyroscope/mesures/Euler/z");
#endif


/************************* Le capteur    *********************************/
Adafruit_BNO055 bno = Adafruit_BNO055(55);



/************************* Les mesures    *********************************/
/** Delai entre deux prise de mesures */
const unsigned long DELAI_ENTRE_MESURES = 500;
// Mesure du temps pour horodater
static unsigned long previousMillis = 0;  


void setup_wifi(void) {
  delay(10);
  // Connexion au Wi-fi
  //DEBUG_PRINT("Connexion à : ");
  //DEBUG_PRINTLN(SSID);
  WiFi.begin(SSID, PASSWORD);
  //DEBUG_PRINTLN(PASSWORD);
  DEBUG_PRINT("C'est en cours");
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_PRINT(".");
  }
 
  DEBUG_PRINTLN("");
  DEBUG_PRINT("ESP connecté au Wi-fi ");
  DEBUG_PRINTLN(SSID);  
  DEBUG_PRINT("Adresse IP : ");
  DEBUG_PRINTLN(WiFi.localIP());
#ifdef DEBUG  
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
#endif
}


void setup_bno()
{
  if (!bno.begin())
  {
    /* Echec sur la connection du BNO05 */
    DEBUG_PRINT("[ECHEC] abscence de BNO05, vérifiez le cablage");
    while (1);
  }
  delay(1000);
    bno.setExtCrystalUse(false); // ?? True ou false
#ifdef DEBUG  
  displaySensorDetails();
#endif 
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  DEBUG_PRINTLN("------------------------------------");
  DEBUG_PRINT  ("Capteur           : "); DEBUG_PRINTLN (sensor.name);
  DEBUG_PRINT  ("Version du driver : "); DEBUG_PRINTLN (sensor.version);
  DEBUG_PRINT  ("Identificateur    : "); DEBUG_PRINTLN (sensor.sensor_id);
  DEBUG_PRINT  ("Valeur maximale   : "); DEBUG_PRINT(sensor.max_value); DEBUG_PRINTLN (" xxx");
  DEBUG_PRINT  ("Valeur minimale   : "); DEBUG_PRINT(sensor.min_value); DEBUG_PRINTLN (" xxx");
  DEBUG_PRINT  ("Resolution        : "); DEBUG_PRINT(sensor.resolution); DEBUG_PRINTLN (" xxx");
  DEBUG_PRINTLN("------------------------------------");
  DEBUG_PRINTLN("");
  delay(500);
}

void MQTT_connect() {
  int8_t ret;

  
  DEBUG_PRINT("Connection à MQTT... ");

  uint8_t retries = 3;
  // Vérifions si c'est déjà connecté
  while ((ret = mqtt.connect()) != 0) { // O c'est connecté
       DEBUG_PRINTLN(mqtt.connectErrorString(ret));
       DEBUG_PRINTLN("On va attendre ...");
       mqtt.disconnect();
       delay(DELAI_RECONNECTION);  // Attendons 
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  DEBUG_PRINTLN("MQTT connecté !");
}


void setup(void)
{
  DEBUG_BEGIN(115200);
  Wire.begin(13, 12);
 
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Test de BNO055, ESP8266 et MQTT\n\r");
  DEBUG_PRINTLN("Configuration du wifi");
  setup_wifi();
  DEBUG_PRINTLN("=========================================================");

  mqtt.subscribe(&getCalibrationMQTT);

  MQTT_connect();
  
  /* Initialisation du capteur */
  DEBUG_PRINTLN("Configuration du capteur");
  setup_bno();

#ifdef CALIBRAGE
  calibration(); // Mettre le client python en GET
#else
  loadCalibration(); // Mettre le client python en SET puis le désactiver apres la recuperation des donnees
#endif
}

void calibration()
{
  int cpt = 0 ;
  while (cpt < 10  || !bno.isFullyCalibrated()) {
    
    String valeurCalib;
    unsigned int taille;
    
    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
    DEBUG_PRINT("CALIBRATION: Sys=");
    DEBUG_PRINT(String(system, DEC));
    valeurCalib = "Systeme:"+ String(system, DEC) + " ; ";
   
    DEBUG_PRINT(" Gyro=");
    DEBUG_PRINT(String(gyro, DEC));
    valeurCalib = valeurCalib+ "Gyro:" +String(gyro, DEC) + " ; ";
 
    DEBUG_PRINT(" Accel=");
    DEBUG_PRINT(String(accel, DEC));
    valeurCalib = valeurCalib+ "Accel:" +String(accel, DEC) + " ; ";
 
    DEBUG_PRINT(" Mag=");
    DEBUG_PRINTLN(String(mag, DEC));
    valeurCalib = valeurCalib+ "Magneto:" +String(mag, DEC) + " ; ";
    
  
    if (!mqtt.connected()) {
      MQTT_connect();
    }
    char valeurMQTT[valeurCalib.length()+1];
    valeurCalib.toCharArray(valeurMQTT, valeurCalib.length()+1);
    if (! calibrationMQTT.publish(valeurMQTT)) {
      DEBUG_PRINTLN("Echec calibration");
    } 
    

    if (gyro == 3 && accel == 3 && mag == 3){
      cpt++;
    }
    else {
      cpt = 0 ;
    }
    delay(100);
  }

  // Envoie des données de calibration au client MQTT pour qu'il les enregistre
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);

  saveCalibration(newCalib);
}

void saveCalibration(const adafruit_bno055_offsets_t &calibData)
{
  // Envoyer ces donnees sur le serveur
  // Unsigned short
  
  String valeurCalib = calibData.accel_offset_x + String(";")+ calibData.accel_offset_y +String(";")+ calibData.accel_offset_z +String(";")
    + calibData.gyro_offset_x +String(";")+ calibData.gyro_offset_y +String(";")+ calibData.gyro_offset_z +String(";")
    + calibData.mag_offset_x +String(";")+ calibData.mag_offset_y +String(";")+ calibData.mag_offset_z +String(";")
    + calibData.accel_radius +String(";")
    + calibData.mag_radius ;
  
  /*
  uint16_t xR = x ;
  uint16_t yR = y ;
  uint16_t zR = z ;
  
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" int = ");Serial.print(x);Serial.print(" le retour = ");Serial.print(xR);Serial.println(" | ");
  Serial.print(calibData.accel_offset_y); Serial.print(" int = ");Serial.print(y);Serial.print(" le retour = ");Serial.print(yR);Serial.println(" | ");
  Serial.print(calibData.accel_offset_z); Serial.print(" int = ");Serial.print(z);Serial.print(" le retour = ");Serial.print(zR);Serial.println(" | ");
  
  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
  */
  if (!mqtt.connected()) {
     MQTT_connect();
  }
  Serial.print("calibration = ");Serial.println(valeurCalib);
  char valeurMQTT[valeurCalib.length()+1];
  valeurCalib.toCharArray(valeurMQTT, valeurCalib.length()+1);
  
  if (! setCalibrationMQTT.publish(valeurMQTT)) {
    DEBUG_PRINTLN("Echec calibration");
  } 
}

void loadCalibration()
{
  String trame = "";
  
  Adafruit_MQTT_Subscribe *subscription;  

  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &getCalibrationMQTT) {
      trame = (char *)getCalibrationMQTT.lastread;
    }
  }
  
  DEBUG_PRINT("Trame récupéré: ");DEBUG_PRINTLN(trame);
  // Recuperer les donnees du serveur
  uint16_t accelX = getValue(trame, ';', 0).toInt(); 
  uint16_t accelY = getValue(trame, ';', 1).toInt();
  uint16_t accelZ = getValue(trame, ';', 2).toInt();
  
  uint16_t gyroX = getValue(trame, ';', 3).toInt();
  uint16_t gyroY = getValue(trame, ';', 4).toInt();
  uint16_t gyroZ = getValue(trame, ';', 5).toInt();
  
  uint16_t magX = getValue(trame, ';', 6).toInt();
  uint16_t magY = getValue(trame, ';', 7).toInt();
  uint16_t magZ = getValue(trame, ';', 8).toInt();
  
  uint16_t accelRadius = getValue(trame, ';', 9).toInt();
  uint16_t magRadius = getValue(trame, ';', 10).toInt();

  adafruit_bno055_offsets_t calibrationData;

  calibrationData.accel_offset_x = accelX ;
  calibrationData.accel_offset_y = accelY ;
  calibrationData.accel_offset_z = accelZ ;

  calibrationData.gyro_offset_x = gyroX ;
  calibrationData.gyro_offset_y = gyroY ;
  calibrationData.gyro_offset_z = gyroZ ;
  
  calibrationData.mag_offset_x = magX ;
  calibrationData.mag_offset_y = magY ;
  calibrationData.mag_offset_z = magZ ;
  
  calibrationData.accel_radius = accelRadius ;
  calibrationData.mag_radius = magRadius ;

  bno.setSensorOffsets(calibrationData);
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void mesure(sensors_event_t &event, char valeur[40])
{
  unsigned long currentMillis = millis();
  //if (currentMillis - previousMillis >= DELAI_ENTRE_MESURES) { // On essaye de prendre des mesures à intervalle régulier
 
    // Lecture du capteur
    bno.getEvent(&event);
    
    DEBUG_PRINT("tps: ");
    DEBUG_PRINTLN(currentMillis - previousMillis); 
    previousMillis = currentMillis;
    
    // Construction de la chaine de caractère temps;x;y;z
    unsigned int taille;
    snprintf(valeur,250 * sizeof(char),"%lu;", currentMillis);
    dtostrf(event.orientation.x,10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0'; 
    dtostrf(event.orientation.y,10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(event.orientation.z,10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';

    // Temperature
    int8_t temp = bno.getTemp();
    dtostrf(temp,10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    
    // Gyroscope
    /*imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    dtostrf(gyro.x(),10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(gyro.y(),10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(gyro.z(),10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';*/
    
    // Accelerometre
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    dtostrf(acc.x(),10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(acc.y(),10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(acc.z(),10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';

    // Magnetometre
    /*imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    dtostrf(mag.x(),10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(mag.y(),10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(mag.z(),10, 5, valeur + strlen(valeur));
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    */
    // Angle d'Euler
    /*imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(euler.x(),10, 5, valeur + strlen(valeur));   
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(euler.y(),10, 5, valeur + strlen(valeur));   
    taille = strlen(valeur); valeur[taille] = ';'; valeur[taille + 1] = '\0';
    dtostrf(euler.z(),10, 5, valeur + strlen(valeur));   

    */
    DEBUG_PRINT("Euler ");
    DEBUG_PRINTLN(valeur); 

#ifdef TESTPENDULE    
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    dtostrf(euler.x(),7, 4, valeur);
  if (! surX.publish(valeur)) {
    DEBUG_PRINTLN("Echec X");
  }
  dtostrf(euler.y(),7, 4, valeur);
  if (! surY.publish(valeur)) {
    DEBUG_PRINTLN("Echec Y");
  }
  dtostrf(euler.z(),7, 4, valeur);
  if (! surZ.publish(valeur)) {
    DEBUG_PRINTLN("Echec Z");
  }   
 
  //Display the floating point data 
  /*
  DEBUG_PRINT("OrientX: ");
  DEBUG_PRINT(event.orientation.x, 4);
  DEBUG_PRINT("\tOrientY: ");
  DEBUG_PRINT(event.orientation.y, 4);
  DEBUG_PRINT("\tOrientZ: ");
  DEBUG_PRINT(event.orientation.z, 4);

  DEBUG_PRINT("accelerationX: ");
  DEBUG_PRINT(event.acceleration.x, 4);
  DEBUG_PRINT("\taccelerationY: ");
  DEBUG_PRINT(event.acceleration.y, 4);
  DEBUG_PRINT("\taccelerationZ: ");
  DEBUG_PRINT(event.acceleration.z, 4);
  
  DEBUG_PRINT("magneticX: ");
  DEBUG_PRINT(event.magnetic.x, 4);
  DEBUG_PRINT("\tmagneticY: ");
  DEBUG_PRINT(event.magnetic.y, 4);
  DEBUG_PRINT("\tmagneticZ: ");
  DEBUG_PRINT(event.magnetic.z, 4);

  DEBUG_PRINT("gyroX: ");
  DEBUG_PRINT(event.gyro.x, 4);
  DEBUG_PRINT("\tyroY: ");
  DEBUG_PRINT(event.gyro.y, 4);
  DEBUG_PRINT("\tyroZ: ");
  DEBUG_PRINT(event.gyro.z, 4);
  */
  DEBUG_PRINT("");

  //test temperature
  /*
  Serial.print("\tTemp: ");
  Serial.print(event.temperature, 4);
  Serial.print("\tMagnetoX: ");
  Serial.print(event.magnetic.x, 4);
  Serial.print("\tMagnetoY: ");
  Serial.print(event.magnetic.y, 4);
  Serial.print("\tMagnetoZ: ");
  Serial.print(event.magnetic.z, 4);
  */
#endif
  //}
 // else delay(DELAI_ENTRE_MESURES - (currentMillis - previousMillis) - 10); // Pas terrible au dépassement de capacité
} 

void publier(char valeur[])
{
  if (!mqtt.connected()) {
    MQTT_connect();
  }
  if (! anglesEuler.publish(valeur)) {
    DEBUG_PRINTLN("Echec angles d'EULER : ");
  } 
}



void loop(void)
{ 
  sensors_event_t event;
  char valeur[250];

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= DELAI_ENTRE_MESURES) {
    mesure(event, valeur);
    publier(valeur);
  }
  
  //delay(100);
}
