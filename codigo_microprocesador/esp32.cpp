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

// Si al girar en sentido horario el yaw BAJA, cambia a -1.0f
#define SIGNO_YAW 1.0f

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
BluetoothSerial SerialBT;
MPU9250 mpu;

// --- Máquina de Estados ---
enum EstadoSistema { INICIANDO, ESPERANDO_GPS, MODO_APP, MODO_RUTA_DEFECTO, RUTA_FINALIZADA };
EstadoSistema estadoActual = INICIANDO;

// --- Variables de Ruta ---
const int ruta_ida[] = {1, 2, 3};
const int total_nodos_ida = sizeof(ruta_ida) / sizeof(ruta_ida[0]);
int indice_ruta = 0;
double latNodoAnterior = 0.0, lonNodoAnterior = 0.0;
bool hayNodoAnterior = false;

double lat_objetivo = 0.0;
double lon_objetivo = 0.0;
bool nodo_cargado = false;

// --- Temporizadores de Navegación ---
unsigned long ultimoBip = 0;
unsigned long ultimoCalculoVelocidad = 0;
unsigned long ultimoPrint = 0;
double distanciaAnterior = 0.0;

// --- Watchdog del MPU (basado en ruido crudo del acelerómetro) ---
float ultimaSumaAcc = 0.0;
unsigned long tUltimoDatoVivo = 0;

// --- Rumbo filtrado y fusión con GPS ---
float filtroSx = 0.0f, filtroSy = 0.0f;
bool filtroInicializado = false;
float offsetMag = 0.0f;

// --- Modo App (Bluetooth) ---
char estadoGuia = 'A';              // L, R o A
unsigned long ultimoMensajeApp = 0;
unsigned long ultimoEnvioRumbo = 0;
char bufferBT[8];
uint8_t bufferBTlen = 0;

// =====================================================================
//  BUZZER NO BLOQUEANTE (Low Level Trigger)
// =====================================================================
void encenderBuzzer() { digitalWrite(BUZZER_PIN, LOW); }
void apagarBuzzer()   { digitalWrite(BUZZER_PIN, HIGH); }

// Patrones: duraciones en ms, alternando ON, OFF, ON, OFF...
const uint16_t PAT_GPS[]    = {50};
const uint16_t PAT_IZQ[]    = {50, 50, 50};
const uint16_t PAT_DER[]    = {300};
const uint16_t PAT_EXITO[]  = {100, 100, 100, 100, 400};
const uint16_t PAT_FINAL[]  = {150, 100, 150, 100, 150, 100, 700}; // 3 cortos + 1 largo
const uint16_t PAT_ALERTA[] = {80, 80, 80, 80, 80};                // 3 muy rápidos

const uint16_t* patronActual = nullptr;
uint8_t patronLen = 0;
uint8_t patronIdx = 0;
unsigned long patronInicio = 0;
bool patronActivo = false;

void iniciarPatron(const uint16_t* pasos, uint8_t n) {
  patronActual = pasos;
  patronLen = n;
  patronIdx = 0;
  patronInicio = millis();
  patronActivo = true;
  encenderBuzzer();
}

void detenerPatron() {
  patronActivo = false;
  apagarBuzzer();
}

// Llamar en cada vuelta del loop
void actualizarBuzzer() {
  if (!patronActivo) return;
  if (millis() - patronInicio >= patronActual[patronIdx]) {
    patronIdx++;
    patronInicio = millis();
    if (patronIdx >= patronLen) {
      detenerPatron();
    } else if (patronIdx % 2 == 0) {
      encenderBuzzer();
    } else {
      apagarBuzzer();
    }
  }
}

// =====================================================================
//  COMANDOS DEL TELÉFONO (BLUETOOTH)
// =====================================================================
bool procesarComando(char c) {
  switch (c) {
    case 'L': case 'R': case 'A':
      estadoGuia = c;
      return true;
    case 'N':
      iniciarPatron(PAT_EXITO, 5);
      return true;
    case 'F':
      iniciarPatron(PAT_FINAL, 7);
      estadoGuia = 'A';
      return true;
    case 'H': // solo latido
      return true;
    default:
      return false;
  }
}

// Lee sin bloquear. Devuelve true si llegó algún comando válido.
bool leerComandosBT() {
  bool hubo = false;
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (bufferBTlen > 0) {
        if (procesarComando(bufferBT[0])) hubo = true;
        bufferBTlen = 0;
      }
    } else if (bufferBTlen < sizeof(bufferBT) - 1) {
      bufferBT[bufferBTlen++] = c;
    }
  }
  return hubo;
}

// =====================================================================
//  CARGA DE JSON DESDE LITTLEFS
// =====================================================================
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

// =====================================================================
//  I2C
// =====================================================================
void destrabarI2C() {
  Serial.println("Forzando liberación del bus I2C...");
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(1);

  if (digitalRead(I2C_SDA) == LOW) {
    pinMode(I2C_SCL, OUTPUT);
    for (int i = 0; i < 16; i++) {
      digitalWrite(I2C_SCL, LOW);
      delayMicroseconds(20);
      digitalWrite(I2C_SCL, HIGH);
      delayMicroseconds(20);
    }
  }
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);
}

void iniciarI2C() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(50000);
  Wire.setTimeOut(150);
}

// =====================================================================
//  RUMBO: FILTRO CIRCULAR + FUSIÓN CON GPS
// =====================================================================
float normalizar360(float a) {
  a = fmodf(a, 360.0f);
  if (a < 0) a += 360.0f;
  return a;
}

float normalizar180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Llamar SOLO cuando mpu.update() devuelve true (tasa constante)
void actualizarFiltroRumbo() {
  float r = radians(SIGNO_YAW * mpu.getYaw());
  float cx = cosf(r), cy = sinf(r);
  if (!filtroInicializado) {
    filtroSx = cx;
    filtroSy = cy;
    filtroInicializado = true;
  } else {
    const float alpha = 0.05f;
    filtroSx += alpha * (cx - filtroSx);
    filtroSy += alpha * (cy - filtroSy);
  }
}

float rumboMagnetico() {
  return normalizar360(degrees(atan2f(filtroSy, filtroSx)));
}

// Corrige lentamente el offset del magnetómetro con el rumbo GPS al caminar
void fusionarConGPS() {
  if (!gps.course.isUpdated()) return;           // solo con muestra GPS nueva
  bool valido = gps.course.isValid() && gps.speed.isValid();
  float vel = gps.speed.mps();
  float curso = gps.course.deg();                // lectura limpia el flag "updated"
  float giroZ = fabsf(mpu.getGyroZ());           // °/s

  // Solo si camina, en línea recta (el curso GPS se retrasa al girar)
  if (valido && vel > 1.0f && giroZ < 20.0f) {
    float err = normalizar180(curso - (rumboMagnetico() + offsetMag));
    offsetMag = normalizar180(offsetMag + 0.2f * err);
  }
}

float rumboActual() {
  return normalizar360(rumboMagnetico() + offsetMag);
}

bool cargarCalibracion() {
  File f = LittleFS.open("/calib.json", "r");
  if (!f) {
    Serial.println("AVISO: no hay /calib.json, MPU sin calibrar");
    return false;
  }
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, f);
  f.close();

  if (e || !doc["accBias"].is<JsonArray>() || !doc["gyroBias"].is<JsonArray>() ||
      !doc["magBias"].is<JsonArray>() || !doc["magScale"].is<JsonArray>()) {
    Serial.println("AVISO: /calib.json invalido, MPU sin calibrar");
    return false;
  }

  mpu.setAccBias(doc["accBias"][0].as<float>(), doc["accBias"][1].as<float>(), doc["accBias"][2].as<float>());
  mpu.setGyroBias(doc["gyroBias"][0].as<float>(), doc["gyroBias"][1].as<float>(), doc["gyroBias"][2].as<float>());
  mpu.setMagBias(doc["magBias"][0].as<float>(), doc["magBias"][1].as<float>(), doc["magBias"][2].as<float>());
  mpu.setMagScale(doc["magScale"][0].as<float>(), doc["magScale"][1].as<float>(), doc["magScale"][2].as<float>());

  Serial.println("Calibracion MPU cargada desde /calib.json");
  return true;
}

bool iniciarMPU() {
  if (!mpu.setup(0x68)) return false;
  cargarCalibracion();
  mpu.setMagneticDeclination(-1.0); // aproximado para San Salvador, verifica en NOAA
  return true;
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  iniciarI2C();

  pinMode(BUZZER_PIN, OUTPUT);
  apagarBuzzer();

  if (!LittleFS.begin()) {
    Serial.println("Error crítico: LittleFS no montado");
    while (1);
  }

  if (!iniciarMPU()) {
    Serial.println("Error crítico: MPU-9250 no encontrado");
    while (1);
  }

  SerialBT.begin("Navegador_UES");
  SerialBT.setTimeout(20);

  // Pitido inicial (en setup el delay es inofensivo)
  encenderBuzzer(); delay(500); apagarBuzzer();

  estadoActual = ESPERANDO_GPS;
  tUltimoDatoVivo = millis();
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  // 1. GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2. MPU + watchdog basado en ruido crudo del acelerómetro
  if (mpu.update()) {
    float sumaAcc = mpu.getAccX() + mpu.getAccY() + mpu.getAccZ();
    if (sumaAcc != ultimaSumaAcc) {  // un sensor vivo siempre tiene ruido
      ultimaSumaAcc = sumaAcc;
      tUltimoDatoVivo = millis();
    }
    actualizarFiltroRumbo();
  }

  if (millis() - tUltimoDatoVivo > 2000) {
    Serial.println("¡CRÍTICO! MPU-9250 sin datos. Reiniciando bus I2C...");
    Wire.end();
    delay(10);
    destrabarI2C();
    iniciarI2C();
    if (iniciarMPU()) {
      Serial.println("MPU-9250 reiniciado con éxito.");
      filtroInicializado = false; // el filtro del sensor se reinició
    }
    tUltimoDatoVivo = millis();
  }

  // 3. Buzzer no bloqueante
  actualizarBuzzer();

  // 4. Comandos del teléfono (cualquier comando válido activa el modo app)
  if (leerComandosBT()) {
    ultimoMensajeApp = millis();
    if (estadoActual != MODO_APP) {
      estadoActual = MODO_APP;
      Serial.println("Modo APP activado");
    }
  }

  // 5. Estados
  switch (estadoActual) {

    case ESPERANDO_GPS:
      if (millis() - ultimoBip > 2000) {
        iniciarPatron(PAT_GPS, 1);
        ultimoBip = millis();
      }

      if (gps.location.isValid() && gps.location.isUpdated()) {
        detenerPatron();
        Serial.println("¡Fix GPS obtenido!");
        estadoActual = MODO_RUTA_DEFECTO; // el modo app se activa solo al recibir comandos
      }
      break;

    case MODO_APP:
      // 1. Enviar rumbo al teléfono (10 Hz)
      if (SerialBT.hasClient() && millis() - ultimoEnvioRumbo >= 100) {
        ultimoEnvioRumbo = millis();
        SerialBT.printf("Y,%.1f,%.1f\n", rumboMagnetico(), mpu.getGyroZ());
      }

      // 2. Pitidos de guía según el último estado recibido
      if (millis() - ultimoBip > 600 && !patronActivo) {
        if (estadoGuia == 'L')      iniciarPatron(PAT_IZQ, 3);
        else if (estadoGuia == 'R') iniciarPatron(PAT_DER, 1);
        ultimoBip = millis();
      }

      // 3. Seguridad: sin conexión o sin latido -> alerta y respaldo con GPS propio
      if (!SerialBT.hasClient() || millis() - ultimoMensajeApp > 3000) {
        Serial.println("APP sin respuesta: volviendo a modo GPS propio");
        iniciarPatron(PAT_ALERTA, 5);
        estadoGuia = 'A';
        estadoActual = gps.location.isValid() ? MODO_RUTA_DEFECTO : ESPERANDO_GPS;
      }
      break;

    case MODO_RUTA_DEFECTO:
      fusionarConGPS();

      if (!nodo_cargado) {
        if (indice_ruta < total_nodos_ida) {
          if (cargarNodoObjetivo(ruta_ida[indice_ruta])) {
            nodo_cargado = true;
            distanciaAnterior = TinyGPSPlus::distanceBetween(
              gps.location.lat(), gps.location.lng(), lat_objetivo, lon_objetivo);
            ultimoCalculoVelocidad = millis();
            tUltimoDatoVivo = millis();

            // Diagnóstico de datos
            Serial.printf("[DIAG] Nodo %d: %.6f, %.6f | desde tu posicion: %.1f m\n",
                          ruta_ida[indice_ruta], lat_objetivo, lon_objetivo, distanciaAnterior);
            if (hayNodoAnterior) {
              double dEntre = TinyGPSPlus::distanceBetween(
                latNodoAnterior, lonNodoAnterior, lat_objetivo, lon_objetivo);
              Serial.printf("[DIAG] Distancia entre nodo anterior y este: %.1f m\n", dEntre);
            }
            latNodoAnterior = lat_objetivo;
            lonNodoAnterior = lon_objetivo;
            hayNodoAnterior = true;
          }
        } else {
          estadoActual = RUTA_FINALIZADA;
          Serial.println("¡Ruta finalizada!");
        }
        break;
      }

      if (gps.location.isValid()) {
        double distancia = TinyGPSPlus::distanceBetween(
          gps.location.lat(), gps.location.lng(), lat_objetivo, lon_objetivo);
        double rumboObjetivo = TinyGPSPlus::courseTo(
          gps.location.lat(), gps.location.lng(), lat_objetivo, lon_objetivo);
        float rumbo = rumboActual();

        float errorRumbo = normalizar180((float)rumboObjetivo - rumbo);

        // Telemetría cada 2 s
        if (millis() - ultimoPrint > 2000) {
          ultimoPrint = millis();
          Serial.printf("Faltan %.1f m | Sat: %d | HDOP: %.1f | Pos: %.6f, %.6f\n",
                        distancia, (int)gps.satellites.value(), gps.hdop.hdop(),
                        gps.location.lat(), gps.location.lng());
          Serial.printf("   Rumbo obj: %.1f | Rumbo: %.1f | Err: %.1f | Off: %.1f\n",
                        rumboObjetivo, rumbo, errorRumbo, offsetMag);
          if (gps.satellites.value() < 5 || gps.hdop.hdop() > 2.5) {
            Serial.println("   AVISO: GPS impreciso, la distancia puede no ser confiable");
          }
        }

        // Feedback auditivo: decisión cada 600 ms, sin delays
        if (millis() - ultimoBip > 600 && !patronActivo) {
          if (errorRumbo < -20)      iniciarPatron(PAT_IZQ, 3);
          else if (errorRumbo > 20)  iniciarPatron(PAT_DER, 1);
          ultimoBip = millis();
        }

        // Nodo alcanzado
        if (distancia < UMBRAL_DISTANCIA) {
          Serial.println("¡Nodo alcanzado!");
          iniciarPatron(PAT_EXITO, 5); // no bloqueante
          indice_ruta++;
          nodo_cargado = false;
        }

        // Detección de paro
        if (millis() - ultimoCalculoVelocidad > 3000) {
          double velocidad = (distanciaAnterior - distancia) / 3.0;
          if (velocidad < 0.2 && distancia > UMBRAL_DISTANCIA) {
            Serial.println("Alerta: Baja velocidad detectada.");
          }
          distanciaAnterior = distancia;
          ultimoCalculoVelocidad = millis();
        }
      }
      break;

    case RUTA_FINALIZADA:
      // No bloquea: el Bluetooth y el buzzer siguen atendidos
      break;

    default:
      break;
  }
}
