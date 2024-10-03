#include "painlessMesh.h"
#include <Arduino_JSON.h>
#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#define MESH_PREFIX     "joserafaelparra"
#define MESH_PASSWORD   "planetavegetta777"
#define MESH_PORT       5555

/**
 * Class: RGBNode.
 * Clase para manejar el nodo RGB y subir información a la base de datos.
 *
 * Attributes:
 * - redPin (int): El pin GPIO al que está conectado el led rojo el RGB.
 * - greenPin (int): El pin GPIO al que está conectado el led verde el RGB.
 * - bluePin (int): El pin GPIO al que está conectado el led azul el RGB.
 * - mesh (painlessMesh): Objeto de la red MESH para manejar la comunicación entre nodos.
 * - userScheduler (Scheduler): scheduler para manejar las tareas del nodo.
 *
 * Methods:
 * - setColor(int red, int green, int blue): Modifica el color del RGB.
 * - blinkErrorLED(): Parpadea el led rojo del RGB en caso de errores.
 * - receivedCallback(uint32_t from, String &msg): Función para procesar los mensajes provenientes de la red.
* - estadoLed(): Regresa el color actual del LED RGB basado en la temperatura actual.
 * - setup(): Inicializa los pines del LED RGB, la red MESH y configura la conexión WiFi y el cliente InfluxDB.
 * - loop(): Actualiza la red MESH y envía datos a InfluxDB si los valores de temperatura y humedad son válidos.
 * - disconnectFromMesh(): Desconecta al nodo de la red MESH para permitir que este se conecte a la base de datos.
 * - connectToWiFiAndSendData(): Conecta el nodo a la red WiFi, envía los datos a InfluxDB y se desconecta del WiFi.
 * - reconnectToMesh(): Reconecta el nodo a la red MESH después de desconectarse temporalmente para subir los datos.
 */

// Se define la red a la que se conectará el nodo y sus parametros como URL, Token, etc.
#define WIFI_SSID "Redmi Note 13 5G"
#define WIFI_PASSWORD "iw2mate9k6d6ryf"
#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "QLNQNCCK6oXS4eJGbRTzwuRjbIqbJmLnuU9wb5Vedrb77_36gkwQPcjBHRkxoTOHGO3HE0y7fSwn7N3Kx47EKQ=="
#define INFLUXDB_ORG "9bd44237850ebc00"
#define INFLUXDB_BUCKET "ESP32"

// Zona horaria en la que se trabajará, en el caso de Colombia es UTC-5.
#define TZ_INFO "UTC-5"

InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

Scheduler userScheduler;
painlessMesh mesh;

WiFiMulti wifiMulti;

// Pins for the RGB LED
const int redPin = 13;
const int greenPin = 12;
const int bluePin = 14;

// Data point for InfluxDB
Point sensor("temperature_data");
Point sensor2("humidity_data");
Point sensor3("state_data");
Point sensor4("time_data");

// Last known temperature and humidity
float lastTemperature = -999;
float lastHumidity = -999;
String estado_led = "desconocido";

// Function prototypes
void disconnectFromMesh();
void connectToWiFiAndSendData();
void reconnectToMesh();
String estadoLed(); // Prototipo de la nueva función

/**
 * Define los colores para el RGB.
 *
 * Parametros:
 * - red (int): La intensidad del color rojo (0-255).
 * - green (int): La intensidad del color verde (0-255).
 * - blue (int): La intensidad del color azul (0-255).
 */
void setColor(int red, int green, int blue) {
  ledcWrite(0, red);   // channel 0 controls Red
  ledcWrite(1, green); // channel 1 controls Green
  ledcWrite(2, blue);  // channel 2 controls Blue
}

/**
 * Función para hacer parpadear el LED durante 3 segundos en caso de error.
 */
void blinkErrorLED() {
  for (int i = 0; i < 6; i++) {
    setColor(255, 0, 0);  // Red (error)
    delay(500);
    setColor(0, 0, 0);    // Off
    delay(500);
  }
}

/**
 * Función para manejar mensahes provenientes de la red MESH, los cuales se espera estén en un formato Json.
 * 
 * Parametros:
 * - from (uint32_t): ID del nodo que envía el mensaje.
 * - msg (String): El mensaje recibido.
 */
void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("Message received from node %u: %s\n", from, msg.c_str());

  // Parse JSON
  DynamicJsonDocument doc(2048); // Aumentar el tamaño del buffer para datos más grandes
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    Serial.print("Error deserializing JSON: ");
    Serial.println(error.c_str());
    blinkErrorLED();
    return;
  }

  // Extract temperature and humidity
  float temperature = doc["temp"];
  float humidity = doc["hum"];

  // Verifica si los valores de temperatura y humedad están presentes
  if (!doc.containsKey("temp") || !doc.containsKey("hum")) {
    Serial.println("Error: faltan los campos de temperatura o humedad en el JSON.");
    return;
  }

  Serial.printf("Received temperature: %.2f, humidity: %.2f\n", temperature, humidity);

  // Validate temperature range
  if (temperature < -10 || temperature > 60) {
    Serial.println("Temperature out of range. Blinking error LED.");
    blinkErrorLED();
    return;
  }

  // Update last known values
  lastTemperature = temperature;
  lastHumidity = humidity;

  // Change RGB LED color based on temperature and provide color description
  if (temperature < 20) {
    setColor(0, 0, 255);  // Blue for temperatures < 20
    estado_led = "Azul: temperatura confortable";
    Serial.println("LED Color: Azul (Temperatura confortable)");
  } else if (temperature >= 20 && temperature <= 24) {
    setColor(0, 255, 0);  // Green for temperatures 20-24
    estado_led = "Verde: temperatura tibia";
    Serial.println("LED Color: Verde (Temperatura tibia)");
  } else {
    setColor(255, 0, 0);  // Red for temperatures > 24
    estado_led = "Rojo: temperatura cálida, vigilar zona";
    Serial.println("LED Color: Rojo (Temperatura cálida, vigilar zona)");
  }

  // Update estado_led
  Serial.printf("Estado actual del LED: %s\n", estado_led.c_str());
}

/**
 * Función la cual retorna el color actual del LED RGB.
 * 
 * Regresa:
 * - (String): El estado actual del LED ("Rojo", "Verde", "Azul", o "Desconocido").
 */
String estadoLed() {
  if (ledcRead(0) == 255 && ledcRead(1) == 0 && ledcRead(2) == 0) {
    return "Rojo: temperatura cálida, vigilar zona";
  } else if (ledcRead(0) == 0 && ledcRead(1) == 255 && ledcRead(2) == 0) {
    return "Verde: temperatura tibia";
  } else if (ledcRead(0) == 0 && ledcRead(1) == 0 && ledcRead(2) == 255) {
    return "Azul: temperatura confortable";
  } else {
    return "Desconocido";
  }
}

/**
 * Inicializa los pines del RGB, el objeto de la red MESH y las funciones.
*/
void setup() {
  Serial.begin(115200);

  // Setup RGB LED pins
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(redPin, 0);
  ledcAttachPin(greenPin, 1);
  ledcAttachPin(bluePin, 2);

  // Initialize the mesh network
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(receivedCallback);  // Set the callback function for receiving messages

  // Initialize WiFi (for later use)
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  // Set up InfluxDB client
  
  sensor.addTag("device", "RGBNode");
  sensor2.addTag("device", "RGBNode2");
  sensor3.addTag("device", "RGBNode3");
  sensor4.addTag("device", "RGBNode4");
  
  //sensor.addTag("SSID", WiFi.SSID());

  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
}

/**
 * Actualiza la red mesh.
*/
void loop() {
  // Update the mesh network
  mesh.update();

  // Check if it's time to send data (every 10 seconds)
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime >= 10000) {
    if (lastTemperature != -999 && lastHumidity != -999) {  // Make sure valid values are available
      disconnectFromMesh();         // Disconnect from the mesh network
      connectToWiFiAndSendData();    // Connect to WiFi and send data to InfluxDB
      // Delay is added inside connectToWiFiAndSendData
    }
    lastSendTime = millis();
  }
}

/**
 * Función que desconecta el nodo de la red MESH, ya que no es posible estar conectado a la red Mesh y a la base de datos al mismo tiempo.
 */
void disconnectFromMesh() {
  Serial.println("Disconnecting from mesh network...");
  mesh.stop();
}

/**
 * Función que conecta el nodo a la red WiFi, envía los datos a la base de datos en InfluxDB, y luego se desconecta el WiFi para permitir la reconexión a la red Mesh.
 */
void connectToWiFiAndSendData() {
  Serial.println("Connecting to WiFi...");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected.");

  // Enviar datos a InfluxDB.
  sensor.clearFields();
  sensor2.clearFields();
  sensor3.clearFields();
  sensor4.clearFields();
  sensor.addField("temperature", lastTemperature);
  sensor2.addField("humidity", lastHumidity);
  sensor3.addField("estado_led", estado_led);  // Añadir el estado del LED
  sensor4.addField("timestamp", String(mesh.getNodeTime())); // Añadir timestamp

  if (client.validateConnection()) {
    Serial.println("Connected to influxDb");
    if (client.writePoint(sensor)) {
      Serial.println("Data written to InfluxDB.");
    } else {
      Serial.print("Failed to write to InfluxDB: ");
      Serial.println(client.getLastErrorMessage());
    }
        if (client.writePoint(sensor2)) {
      Serial.println("Data written to InfluxDB.");
    } else {
      Serial.print("Failed to write to InfluxDB: ");
      Serial.println(client.getLastErrorMessage());
    }
        if (client.writePoint(sensor3)) {
      Serial.println("Data written to InfluxDB.");
    } else {
      Serial.print("Failed to write to InfluxDB: ");
      Serial.println(client.getLastErrorMessage());
    }
        if (client.writePoint(sensor4)) {
      Serial.println("Data written to InfluxDB.");
    } else {
      Serial.print("Failed to write to InfluxDB: ");
      Serial.println(client.getLastErrorMessage());
    }
  } else {
    Serial.println("Failed to connect to InfluxDB");
  }

  // Desconectar de la red Wifi.
  WiFi.disconnect();
  Serial.println("WiFi disconnected.");
  
  // Esperar 5 segundos antes de reconectarse a la red Mesh.
  delay(5000);

  // reconectarse a la red Mesh.
  reconnectToMesh();
}

/**
 * Función encargada de reconectar el nodo a la red MESH después de desconectarse temporalmente para enviar datos a la base en InfluxDB.
 */
void reconnectToMesh() {
  Serial.println("Reconnecting to mesh network...");
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  Serial.println("Mesh network reconnected.");
}