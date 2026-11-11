#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#include <MPU9250.h> // Instalar: "MPU9250 by Hideaki Tai"

// --- Configuración de Pines ---
#define BUZZER_PIN 15    
#define GPS_RX_PIN 16        
#define GPS_TX_PIN 17        
#define I2C_SDA 21           
#define I2C_SCL 22           

#define UMBRAL_DISTANCIA 3.0 

HardwareSerial gpsSerial(2); 
TinyGPSPlus gps;
BluetoothSerial SerialBT;
MPU9250 mpu;

// --- Máquina de Estados ---
enum EstadoSistema { INICIANDO, ESPERANDO_GPS, MODO_APP, MODO_RUTA_DEFECTO };
EstadoSistema estadoActual = INICIANDO;

// --- Variables de Ruta ---
const int ruta_ida[] = {1, 2, 3}; // IDs a buscar en el archivo nodos.json
const int total_nodos_ida = sizeof(ruta_ida) / sizeof(ruta_ida[0]);
int indice_ruta = 0;

double lat_objetivo = 0.0;
double lon_objetivo = 0.0;
bool nodo_cargado = false;

// --- Temporizadores No Bloqueantes ---
unsigned long ultimoBip = 0;
unsigned long ultimoCalculoVelocidad = 0;
double distanciaAnterior = 0.0;

// --- Funciones de Audio (Low Level Trigger) ---
void encenderBuzzer() {
  digitalWrite(BUZZER_PIN, LOW);  // LOW enciende el buzzer activo
}

void apagarBuzzer() {
  digitalWrite(BUZZER_PIN, HIGH); // HIGH lo apaga
}

// --- Carga de JSON desde LittleFS ---
bool cargarNodoObjetivo(int idBuscado) {
  File file = LittleFS.open("/nodos.json", "r");
  if (!file) {
    Serial.println("Error: Archivo nodos.json no encontrado");
    return false;
  }

  JsonDocument filtro;
  filtro["nodos"][0]["id"] = true;
  filtro["nodos"][0]["lat"] = true;
  filtro["nodos"][0]["lon"] = true;

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file, DeserializationOption::Filter(filtro));
  file.close(); 

  if (error) return false;

  JsonArray arrayNodos = doc["nodos"];
  for (JsonObject nodo : arrayNodos) {
    if (nodo["id"] == idBuscado) {
      lat_objetivo = nodo["lat"];
      lon_objetivo = nodo["lon"];
      Serial.printf("Nodo %d cargado. Destino: %f, %f\n", idBuscado, lat_objetivo, lon_objetivo);
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  
  // Inicialización de Hardware
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Configuración del Buzzer (Low Trigger)
  pinMode(BUZZER_PIN, OUTPUT);
  apagarBuzzer(); // Asegurar que inicie apagado

  if (!LittleFS.begin()) {
    Serial.println("Error crítico: LittleFS no montado");
    while (1); 
  }

  // Inicialización MPU-9250 (Dirección I2C por defecto 0x68)
  if (!mpu.setup(0x68)) {
    Serial.println("Error crítico: MPU-9250 no encontrado");
    while (1);
  }
  // IMPORTANTE: Antes de pruebas reales, usar mpu.calibrateMag() aquí

  SerialBT.begin("Navegador_UES");
  
  // Pitido inicial largo de confirmación de encendido
  encenderBuzzer(); delay(500); apagarBuzzer();
  
  estadoActual = ESPERANDO_GPS;
}

void loop() {
  // 1. Lectura continua de sensores
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (mpu.update()) {
    // El MPU se actualiza continuamente en background
  }

  // 2. Control de Estados
  switch (estadoActual) {
    
    case ESPERANDO_GPS:
      // Bip corto cada 2s para indicar "Buscando Satélites"
      if (millis() - ultimoBip > 2000) {
        encenderBuzzer(); delay(50); apagarBuzzer();
        ultimoBip = millis();
      }

      if (gps.location.isValid() && gps.location.isUpdated()) {
        apagarBuzzer();
        Serial.println("¡Fix GPS obtenido!");
        if (SerialBT.hasClient()) {
          estadoActual = MODO_APP;
        } else {
          estadoActual = MODO_RUTA_DEFECTO;
        }
      }
      break;

    case MODO_APP:
      if (SerialBT.available()) {
        String comando = SerialBT.readStringUntil('\n');
        Serial.print("App dice: "); Serial.println(comando);
      }
      break;

    case MODO_RUTA_DEFECTO:
      if (!nodo_cargado) {
        if (indice_ruta < total_nodos_ida) {
          if (cargarNodoObjetivo(ruta_ida[indice_ruta])) {
            nodo_cargado = true;
            distanciaAnterior = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), lat_objetivo, lon_objetivo);
            ultimoCalculoVelocidad = millis();
          }
        } else {
          apagarBuzzer();
          Serial.println("¡Ruta finalizada!");
          while(true) { yield(); } 
        }
      }

      if (nodo_cargado && gps.location.isValid()) {
        double distancia = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), lat_objetivo, lon_objetivo);
        double rumboObjetivo = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), lat_objetivo, lon_objetivo);
        
        // Brújula digital compensada
        float rumboActual = mpu.getYaw(); 
        
        // Calcular error de rumbo (-180 a 180 grados)
        float errorRumbo = rumboObjetivo - rumboActual;
        if (errorRumbo > 180) errorRumbo -= 360;
        if (errorRumbo < -180) errorRumbo += 360;

        // Línea de depuración para el Monitor Serie
        Serial.printf("Dist: %.2fm | Rumbo Obj: %.1f° | MPU: %.1f° | Error: %.1f°\n", distancia, rumboObjetivo, rumboActual, errorRumbo);

        // Feedback Auditivo Dinámico basado en ritmos (cada 600ms)
        if (millis() - ultimoBip > 600) {
          if (errorRumbo < -20) {
            // Corrección IZQUIERDA: Dos bips rápidos
            encenderBuzzer(); delay(50); apagarBuzzer(); delay(50);
            encenderBuzzer(); delay(50); apagarBuzzer();
            ultimoBip = millis();
          } else if (errorRumbo > 20) {
            // Corrección DERECHA: Un bip largo
            encenderBuzzer(); delay(300); apagarBuzzer();
            ultimoBip = millis();
          } else {
            // ADELANTE (En curso): Silencio
            apagarBuzzer();
          }
        }

        // Evaluar si se alcanzó el nodo
        if (distancia < UMBRAL_DISTANCIA) {
          Serial.println("¡Nodo alcanzado!");
          // Patrón de éxito: 3 bips asimétricos
          encenderBuzzer(); delay(100); apagarBuzzer(); delay(100);
          encenderBuzzer(); delay(100); apagarBuzzer(); delay(100);
          encenderBuzzer(); delay(400); apagarBuzzer(); 
          
          indice_ruta++;
          nodo_cargado = false; 
        }

        // Sistema de seguridad: Detección de paro
        if (millis() - ultimoCalculoVelocidad > 3000) {
          double velocidad = (distanciaAnterior - distancia) / 3.0; // m/s
          if (velocidad < 0.2 && distancia > UMBRAL_DISTANCIA) {
            Serial.println("Alerta: Baja velocidad detectada.");
          }
          distanciaAnterior = distancia;
          ultimoCalculoVelocidad = millis();
        }
      }
      break;
  }
}