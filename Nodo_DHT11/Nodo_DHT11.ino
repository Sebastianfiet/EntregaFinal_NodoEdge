#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "painlessMesh.h"
#include <Arduino_JSON.h>

/**
 * @brief A class to represent the node handling DHT11 sensor readings and sending data via a mesh network.
 * 
 * Atributos:
 * - MESH_PREFIX (str): Contiene el nombre de la red MESH.
 * - MESH_PASSWORD (str): contiene la contraseña de la red.
 * - MESH_PORT (int): El puerto por el cual se comunicarán los nodos en la red.
 * - DHTPIN (int): El pin al que se conecta el sensor DTH11.
 * - DHTTYPE (constant): El tipo de sensor DHT que se maneja, en este caso DHT11.
 * - dht (object): Una instancia del sensor DHT11.
 * - nodeNumber (int): Identificador para el nodo 1.
 * - temp (double): Variable en donde se guardarán las lecturas de temperatura.
 * - hum (double): Variable en donde se guardarán las lecturas de humedad.
 * - lastTemp (double): El último valor registrado de temperatura para propósitos de comparación en la función de calidad de datos.
 * - lastHum (double): El último valor registrado de humedad para propósitos de comparación en la función de calidad de datos.
 * - readings (String): una cadena de texto en formato Json en donde se almacenan las lecturas para su posterior transmisión en la red.
 * - userScheduler (object): Un scheduler para administrar las tareas del nodo.
 * - mesh (object): Una instancia de la red MESH para la comunicación entre nodos.
 * - taskSendMessage (Task object): Tarea para enviar mensajes a los nodos de la red cada 5 segundos.
 * - taskCheckDataQuality (Task object): Tarea para checar la calidad de los datos cada 10 segundos.
 * 
 * Métodos:
 * - setup(): Inicializa el sensor DHT11, la red MESH, y las tareas del nodo.
 * - loop(): Actualiza la red y se encarga de repetir las tareas cada vez que se necesite.
 * - getReadings(): Obtiene las lecturas de temperatura y humedad del sensor y las guarda en un formato Json.
 * - sendMessage(): Envía los datos guardados en el archivo Json a los nodos conectados a la red.
 * - checkDataQuality(): Checha que los datos leídos estén en un rango verosímil y no sean muy similares a los anteriores, lo que podría ser un indicio de errores.
 * - receivedCallback(uint32_t from, String &msg): Procesa los mensajes recibidos en caso otro nodo los envíe.
 * - newConnectionCallback(uint32_t nodeId): Maneja las nuevas conexiones que entren en la red.
 * - changedConnectionCallback(): Maneja los cambios de conexión en la red.
 * - nodeTimeAdjustedCallback(int32_t offset): Maneja la sincronización de los eventos con la red.
 */


// MESH Details
#define MESH_PREFIX     "joserafaelparra" // Nombre de la red mesh
#define MESH_PASSWORD   "planetavegetta777" // Contraseña de la red mesh
#define MESH_PORT       5555 // Puerto de comunicación mesh

// Pines para el sensor DHT y tipo de sensor (DHT11, DHT22, etc.)
#define DHTPIN 4     // Pin al que está conectado el sensor DHT
#define DHTTYPE DHT11 // Tipo de sensor: DHT11 o DHT22

// Crear una instancia del sensor DHT
DHT dht(DHTPIN, DHTTYPE);

// Pines para el LED RGB (Nodo 1 no lo usa, pero lo dejamos por si es necesario)
int redPin = 13;
int greenPin = 12;
int bluePin = 14;

// Número de este nodo
int nodeNumber = 1;  // Este es el nodo 1
double temp = 0;     // Variable para temperatura
double hum = 0;      // Variable para humedad
double lastTemp = -100; // Última temperatura registrada para comparación
double lastHum = -100;  // Última humedad registrada para comparación
String readings;

Scheduler userScheduler; // Scheduler para controlar las tareas
painlessMesh  mesh;

// Prototipos de funciones
void sendMessage(); 
String getReadings();
void checkDataQuality(); // Función para la nueva tarea que valida los datos

// Crear una tarea para enviar mensajes cada 5 segundos
Task taskSendMessage(TASK_SECOND * 5.05 , TASK_FOREVER, &sendMessage);

// Crear una tarea para revisar la calidad de los datos cada 10 segundos
Task taskCheckDataQuality(TASK_SECOND * 10, TASK_FOREVER, &checkDataQuality);

// Función para redondear a 2 decimales
double roundToTwoDecimals(double value) {
  return round(value * 100.0) / 100.0;
}


/**
 * Lee la temperatura y humedad del sensor DHT11 y almacena esos datos en un formato Json.
 * 
 */
String getReadings() {
  JSONVar jsonReadings;

  // Obtener las lecturas del sensor DHT
  temp = dht.readTemperature();  // Leer temperatura en grados Celsius
  hum = dht.readHumidity();      // Leer humedad en porcentaje

  // Comprobar si las lecturas son válidas (ni NaN)
  if (!isnan(temp) && !isnan(hum)) {
    // Redondear a 2 decimales
    temp = roundToTwoDecimals(temp);
    hum = roundToTwoDecimals(hum);

    // Agregar valores al mensaje JSON
    jsonReadings["node"] = nodeNumber;
    jsonReadings["temp"] = temp;  // Enviar temperatura
    jsonReadings["hum"] = hum;    // Enviar humedad

    readings = JSON.stringify(jsonReadings);  // Convertir a formato JSON
  } else {
    Serial.println("Error en las lecturas del sensor.");
  }

  return readings;
}

/**
 * Envía el archivo Json con los datos obtenidos a los nodos de la red MESH.
 */
void sendMessage() {
  String msg = getReadings();  // Obtener lecturas en formato JSON
  Serial.println("Enviando mensaje: " + msg);
  mesh.sendBroadcast(msg);  // Enviar mensaje a través de la red mesh
}

/**
 * Chequea si los valores de temperatura y humedad están en un rango creíble
 * Si los valores no lo están o son muy similares entre sí, se imprime un mensaje de error. 
 */
void checkDataQuality() {
  if ((temp < 0 || temp > 50) || (hum < 20 || hum > 90)) {
    Serial.println("Datos fuera de rango: Reintentando...");
    temp = 0;
    hum = 0;
  }

  // Verificar si los valores son muy similares a los anteriores
  if (abs(temp - lastTemp) < 0.5 && abs(hum - lastHum) < 1) {
    Serial.println("Lecturas sospechosas: datos muy similares a los anteriores.");
  } else {
    lastTemp = temp;
    lastHum = hum;
    Serial.println("Datos válidos.");
  }
}

/**
 * Función para manejar mensajes provenientes de otros nodos.
 * Parametros:
 * - from: ID del nodo que envía el mensaje.
 * - msg: El mensaje del nodo, el cual se espera esté en un formato Json.
 */
void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("Mensaje recibido del nodo %u: %s\n", from, msg.c_str());

  // Parsear el mensaje JSON
  JSONVar myObject = JSON.parse(msg.c_str());
  
  // Si el mensaje tiene un error de formato, salir
  if (JSON.typeof(myObject) == "undefined") {
    Serial.println("Error en el formato del mensaje");
    return;
  }

  // Leer los valores del mensaje
  int node = int(myObject["node"]);
  double receivedTemp = double(myObject["temp"]);
  double receivedHum = double(myObject["hum"]);

  // Imprimir valores recibidos
  Serial.print("Nodo: ");
  Serial.println(node);
  Serial.print("Temperatura recibida: ");
  Serial.print(receivedTemp);
  Serial.println(" °C");
  Serial.print("Humedad recibida: ");
  Serial.print(receivedHum);
  Serial.println(" %");
}

/**
 * Función para manejar nuevas conexones de la red MESH.
 * Parametros:
 * - nodeId: ID del nodo que se conectó a la red.
 */
void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("Nueva conexión, nodeId = %u\n", nodeId);
}

/**
 * Función que se activa cuando una conexión en la red cambia.
 */
void changedConnectionCallback() {
  Serial.printf("Conexiones cambiadas\n");
}

/**
 * Función llamda cuando el tiempo en la red se modifica.
 * Parametros
 * - offset: El tiempo de offset después del ajuste.
 */
void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Tiempo ajustado %u. Offset = %d\n", mesh.getNodeTime(), offset);
}


/**
 * Se Inicializa el sensor DHT11 y la red MESH.
 * Se preparan las tareas para el envío de datos y checar la calidad de los datos.
 */
void setup() {
  Serial.begin(115200);

  // Inicializar el sensor DHT
  dht.begin();
  
  // Configurar los pines como salida para el LED RGB (si se quiere usar)
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Iniciar la red mesh
  mesh.setDebugMsgTypes(ERROR | STARTUP);  // Solo mensajes de error y arranque
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);  // Callback para recibir mensajes
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  // Agregar tarea para enviar mensajes cada 5 segundos
  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

  // Agregar tarea para verificar la calidad de los datos cada 10 segundos
  userScheduler.addTask(taskCheckDataQuality);
  taskCheckDataQuality.enable();
}

/**
 * loop para actualizar la red MESH y mantener activas las tareas del nodo.
 */
void loop() {
  mesh.update();  // Actualizar la red mesh
  userScheduler.execute();  // Ejecutar las tareas del scheduler
}