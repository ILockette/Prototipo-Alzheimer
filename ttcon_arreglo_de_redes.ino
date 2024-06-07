#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <TinyGPS.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoJson.h> // Asegúrate de incluir la biblioteca ArduinoJson

// Configuración de MQTT y Wi-Fi
//const char* ssid = "LuisRood2";
//const char* pass = "ocholetras";
const char* mqtt_server = "52.146.12.184";
const int mqtt_port = 1883;

const char* redesWiFi[] = {"Red1", "LuisRood2", "mqtt"}; // Arreglo de nombres de redes
const char* contrasenasWiFi[] = {"pass1", "ocholetras", "ocholetras"}; // Arreglo de contraseñas de redes
const int numRedes = sizeof(redesWiFi) / sizeof(redesWiFi[0]); // Número de redes en el arreglo

//const char* mac="12:34:56:78:9A:BC";

WiFiClient wifiClient;
PubSubClient client(wifiClient);
TinyGPS gps; // Objeto GPS

// Variable Global  
char macAddress[18]; // Al menos 18 caracteres para incluir la dirección MAC y el terminador nulo
float umbral_aceleracion = 0.5;  // Umbral para caída libre
float umbral_rotacion = 200;  // Umbral para rotación brusca
int tiempo_umbral = 100;  // Tiempo en ms para considerar como caída
int contador_tiempo = 0;  // Contador para tiempo bajo umbral


// Conectar a Wi-Fi
/*void connectToWiFi() {
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" Conectado a Wi-Fi");
}*/
void connectToWiFi() {
    bool conectado = false;
    while (!conectado) {
        for (int intento = 0; intento < numRedes; intento++) {
            WiFi.begin(redesWiFi[intento], contrasenasWiFi[intento]);
            Serial.print("Conectando a ");
            Serial.print(redesWiFi[intento]);
            Serial.print("...");
            unsigned long tiempoInicio = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - tiempoInicio < 10000) {
                delay(500); // Espera un tiempo para intentar conectarse
            }
            if (WiFi.status() == WL_CONNECTED) {
                conectado = true;
                Serial.println("Conectado a Wi-Fi");
                break; // Salir del bucle for
            } else {
                Serial.println("No se pudo conectar a esta red");
            }
        }
        if (!conectado) {
            Serial.println("No se pudo conectar a ninguna red Wi-Fi. Volviendo a intentar...");
            delay(5000); // Espera antes de volver a intentar
        }
    }
}

// Conectar a MQTT
void connectToMQTT() {
    if (!client.connected()) {
        Serial.println("Conectando al servidor MQTT...");
        while (!client.connect("ArduinoClient")) {
            Serial.print("Error de conexión: ");
            Serial.print(client.state()); // Obtener el estado de la conexión
            Serial.println(" Reintentando...");
            delay(2000); // Esperar antes de reintentar
        }
    }
     client.subscribe(macAddress); // Asegúrate de suscribirte al tópico correcto
     publishMacJson(); // Publicar el JSON con la MAC al reconectar
   
}
// Obtener y Almacenar la Dirección MAC
void obtenerMAC() {
    byte mac[6];
    WiFi.macAddress(mac);

    sprintf(macAddress, "%02X:%02X:%02X:%02X:%02X:%02X", 
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Publicar la MAC en formato JSON
void publishMacJson() {
    StaticJsonDocument<128> jsonDoc;
    jsonDoc["Mac"] = macAddress;
    jsonDoc["Caida"] = nullptr;
    jsonDoc["Error"] = nullptr;
    jsonDoc["Latitud"] = nullptr;
    jsonDoc["Longitud"] = nullptr;
    jsonDoc["Fecha"] = nullptr;
    jsonDoc["Hora"] = nullptr;

    char jsonBuffer[128];
    serializeJson(jsonDoc, jsonBuffer);
    Serial.println(jsonBuffer);
    if (client.publish("mi/tema", jsonBuffer)) {
        Serial.println("MAC publicada con éxito.");
    } else {
        Serial.println("Error al publicar la MAC.");
    }
}



void setup() {//inicio del disp
    Serial.begin(9600);
    Serial1.begin(9600);  // Iniciar el puerto serie para el GPS
    // Conectar a Wi-Fi y MQTT
    connectToWiFi();
    obtenerMAC(); // Obtener la dirección MAC y almacenarla
    Serial.println("MAC obtenida: " + String(macAddress));

    if (!IMU.begin()) {
        Serial.println("No se pudo inicializar el IMU");
        while (1); // Detiene el programa si el IMU no se inicia
    }
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback); // Configurar el callback para mensajes entrantes

    connectToMQTT(); // Conectar al servidor MQTT

    // Publicar la MAC
    /*StaticJsonDocument<128> jsonDoc;
    jsonDoc["Mac"] = macAddress;
    //jsonDoc["mac"] = "12:34:56:78:9A:BC"; // Cambia por tu MAC real
    jsonDoc["Caida"] = NULL;
    jsonDoc["Error"] = NULL;
    jsonDoc["Latitud"] = NULL;
    jsonDoc["Longitud"] = NULL;
    jsonDoc["Fecha"] = NULL;
    jsonDoc["Hora"] = NULL;

    char jsonBuffer[128];
    serializeJson(jsonDoc, jsonBuffer);
    Serial.println(jsonBuffer);
    if (client.publish("mi/tema", jsonBuffer)) {
        Serial.println("MAC publicada con éxito.");
    } else {
        Serial.println("Error al publicar la MAC.");
    }
    client.subscribe(macAddress);*/

    // Publicar la MAC inicialmente
    publishMacJson();
    client.subscribe(macAddress);
}

void loop() {//esta activo todo el tiempo
    //revisar la conexion
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Conexión Wi-Fi perdida. Intentando reconectar...");
        connectToWiFi();
        connectToMQTT();
    }

    if (!client.connected()) {
        Serial.println("Conexión MQTT perdida. Intentando reconectar...");
        connectToMQTT();
    }
    client.loop(); // Procesar mensajes entrantes
     float latitud, longitud;
    gps.f_get_position(&latitud, &longitud);
    int year;
    byte month, day, hour, minute, second, hundredths;
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);
    StaticJsonDocument<256> dispositivo;
    float aceleracion_x = 0.0;
    float aceleracion_y = 0.0;
    float aceleracion_z = 0.0;
    float giro_x = 0.0;
    float giro_y = 0.0;
    float giro_z = 0.0;

     while (Serial1.available() > 0) {
            gps.encode(Serial1.read());
        }
    // Leer desde la terminal serie para simular caida
    if (Serial.available() > 0) { // Verificar si hay datos disponibles
        String input = Serial.readStringUntil('\n'); // Leer hasta un salto de línea
        input.trim(); // Eliminar espacios en blanco

        if (input.equalsIgnoreCase("caida")) { // Verificar si el mensaje es "caida"
            dispositivo["Mac"] =macAddress;
            Serial.println("Se ha detectado 'caida'. Enviando mensaje...");
            dispositivo["Caida"] = true; // Indicar que se detectó una caída
            //obtener gps
          if (latitud == TinyGPS::GPS_INVALID_F_ANGLE || longitud == TinyGPS::GPS_INVALID_F_ANGLE) {
                  Serial.println("Datos GPS inválidos. No se puede crear el JSON.");
                  dispositivo["error"] = "Datos GPS inválidos.";
                  dispositivo["Latitud"] = nullptr;
                  dispositivo["Longitud"] = nullptr; 
                  dispositivo["Fecha"] = nullptr;
                  //dispositivo["Hora"] = nullptr;
                  }

                  if (latitud != TinyGPS::GPS_INVALID_F_ANGLE) {
                      dispositivo["error"]=nullptr;
                      dispositivo["Latitud"] = latitud;
                      dispositivo["Longitud"] = longitud;
                      if (year > 2000) {
                              dispositivo["Fecha"] = String(day) + "/" + String(month) + "/" + String(year);
                              //dispositivo["Hora"] = String(hour) + ":" + String(minute) + ":" + String(second);
                          }
                  }
                  char jsonBuffer[128];
                  serializeJson(dispositivo, jsonBuffer); // Serializar el JSON
                  client.publish("mi/tema", jsonBuffer); // Publicar el mensaje al tópico correcto
                  Serial.println("Mensaje de 'caida' enviado: " + String(jsonBuffer));

        }
    }
    //detectar caida
    // Leer valores del acelerómetro y giroscopio
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(aceleracion_x, aceleracion_y, aceleracion_z);
    }

    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(giro_x, giro_y, giro_z);
    }

     // Calcula la magnitud total de la aceleración
    float magnitud_aceleracion = sqrt(aceleracion_x * aceleracion_x + aceleracion_y * aceleracion_y + aceleracion_z * aceleracion_z);

    // Verifica si la aceleración está por debajo del umbral
    if (magnitud_aceleracion < umbral_aceleracion) {
      contador_tiempo += 10;  // Incrementa el contador
    } else {
      contador_tiempo = 0;  // Reinicia si la aceleración es normal
    }

    // Verifica si hay rotación brusca
    bool rotacion_alta = (abs(giro_x) > umbral_rotacion || abs(giro_y) > umbral_rotacion || abs(giro_z) > umbral_rotacion);

    // Detecta si la aceleración está baja por suficiente tiempo y hay rotación alta
    if (contador_tiempo >= tiempo_umbral && rotacion_alta) {
      Serial.println("Caída detectada!");
        Serial.println("Se ha detectado 'caida'. Enviando mensaje...");
            dispositivo["Mac"] =macAddress;
            dispositivo["Caida"] = true; // Indicar que se detectó una caída
            if (latitud == TinyGPS::GPS_INVALID_F_ANGLE || longitud == TinyGPS::GPS_INVALID_F_ANGLE) {
                  Serial.println("Datos GPS inválidos. No se puede crear el JSON.");
                  dispositivo["error"] = "Datos GPS inválidos.";
                  dispositivo["Latitud"] = nullptr;
                  dispositivo["Longitud"] = nullptr;
                  dispositivo["Fecha"] =nullptr;
                  //dispositivo["Hora"] = nullptr;
                  }

                  if (latitud != TinyGPS::GPS_INVALID_F_ANGLE) {
                      dispositivo["Error"]=nullptr;
                      dispositivo["Latitud"] = latitud;
                      dispositivo["Longitud"] = longitud;
                      if (year > 2000) {
                              dispositivo["Fecha"] = String(day) + "/" + String(month) + "/" + String(year);
                              //dispositivo["Hora"] = String(hour) + ":" + String(minute) + ":" + String(second);
                          }
                  }
                  char jsonBuffer[128];
                  serializeJson(dispositivo, jsonBuffer); // Serializar el JSON
                  client.publish("mi/tema", jsonBuffer); // Publicar el mensaje al tópico correcto
                  Serial.println("Mensaje de 'caida' enviado: " + String(jsonBuffer));
      contador_tiempo = 0;  // Reinicia el contador
    }

    delay(10);  // Ajusta según el tiempo de ciclo necesario
}
void mqttCallback(char* topic, byte* payload, unsigned int length) { //procesa los mensajes
    String message = "";

    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.println("Mensaje recibido: " + message);

    // Verificar si el mensaje es correcto
    if (message == "obtener_datos") {
      float latitud, longitud;
        gps.f_get_position(&latitud, &longitud);
        int year;
        byte month, day, hour, minute, second, hundredths;
        gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);
        StaticJsonDocument<256> dispositivo;
        dispositivo["Mac"] =macAddress;
        dispositivo["Caida"] = nullptr;

        

                // Verificar si los datos del GPS son válidos
        if (latitud == TinyGPS::GPS_INVALID_F_ANGLE || longitud == TinyGPS::GPS_INVALID_F_ANGLE) {
            Serial.println("Datos GPS inválidos. No se puede crear el JSON.");
            dispositivo["Error"] = "Datos GPS inválidos.";
            dispositivo["Latitud"] = nullptr;
            dispositivo["Longitud"] = nullptr;
            dispositivo["Fecha"] =nullptr;
            dispositivo["Hora"] = nullptr;
        }

        if (latitud != TinyGPS::GPS_INVALID_F_ANGLE) {
           dispositivo["error"]=nullptr;
            dispositivo["Latitud"] = latitud;
            dispositivo["Longitud"] = longitud;
            if (year > 2000) {
                    dispositivo["Fecha"] = String(day) + "/" + String(month) + "/" + String(year);
                    dispositivo["Hora"] = String(hour) + ":" + String(minute) + ":" + String(second);
                }
        }
        char jsonBuffer[256];
        serializeJson(dispositivo, jsonBuffer); // Serializar JSON correctamente
        Serial.println(jsonBuffer);
        client.publish("mi/tema", jsonBuffer); // Publicar datos de ubicación
        
    }
}
