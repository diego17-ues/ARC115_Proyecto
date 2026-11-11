#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <MPU9250.h>

#define I2C_SDA 21
#define I2C_SCL 22

MPU9250 mpu;
unsigned long tPrint = 0;

void cuentaAtras(int s) {
  for (int i = s; i > 0; i--) {
    Serial.printf("%d... ", i);
    delay(1000);
  }
  Serial.println("YA");
}

bool calibracionPlausible() {
  for (int i = 0; i < 3; i++) {
    float s = mpu.getMagScale(i);
    if (s < 0.5f || s > 2.0f) return false;
  }
  return true;
}

JsonDocument construirJson() {
  JsonDocument doc;
  JsonArray ab = doc["accBias"].to<JsonArray>();
  JsonArray gb = doc["gyroBias"].to<JsonArray>();
  JsonArray mb = doc["magBias"].to<JsonArray>();
  JsonArray ms = doc["magScale"].to<JsonArray>();
  for (int i = 0; i < 3; i++) {
    ab.add(mpu.getAccBias(i));
    gb.add(mpu.getGyroBias(i));
    mb.add(mpu.getMagBias(i));
    ms.add(mpu.getMagScale(i));
  }
  return doc;
}

bool guardarCalibracion() {
  JsonDocument doc = construirJson();
  File f = LittleFS.open("/calib.json", "w");
  if (!f) {
    Serial.println("ERROR: no se pudo abrir /calib.json para escribir");
    return false;
  }
  serializeJsonPretty(doc, f);
  f.close();

  Serial.println("\n--- Guardado en LittleFS (/calib.json). Contenido: ---");
  serializeJsonPretty(doc, Serial);
  Serial.println("\n--- Copia este texto como respaldo (ver instrucciones) ---");
  return true;
}

void mostrarArchivo() {
  File f = LittleFS.open("/calib.json", "r");
  if (!f) {
    Serial.println("No existe /calib.json");
    return;
  }
  while (f.available()) Serial.write(f.read());
  f.close();
  Serial.println();
}

void calibrar() {
  Serial.println("\n=== PASO 1: ACELEROMETRO Y GIROSCOPIO ===");
  Serial.println("Deja el dispositivo COMPLETAMENTE QUIETO y HORIZONTAL.");
  cuentaAtras(5);
  mpu.verbose(true);
  mpu.calibrateAccelGyro();

  Serial.println("\n=== PASO 2: MAGNETOMETRO ===");
  Serial.println("Cuando diga YA, mueve el dispositivo dibujando un '8' en el aire,");
  Serial.println("girandolo en TODAS las orientaciones, durante ~15 segundos.");
  cuentaAtras(5);
  mpu.calibrateMag();
  mpu.verbose(false);

  if (!calibracionPlausible()) {
    Serial.println("\nAVISO: las escalas del magnetometro son raras (fuera de 0.5-2.0).");
    Serial.println("NO se guardo nada. Repite con 'c' alejado de metales y moviendo mas.");
    return;
  }

  if (guardarCalibracion()) {
    Serial.println("\nListo. Ahora gira el dispositivo: mira el yaw de abajo.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(50000);
  Wire.setTimeOut(150);

  if (!LittleFS.begin()) {
    Serial.println("Error: LittleFS no montado");
    while (1);
  }
  if (!mpu.setup(0x68)) {
    Serial.println("Error: MPU-9250 no encontrado");
    while (1);
  }

  Serial.println("\nComandos: 'c' = calibrar y guardar | 'p' = mostrar /calib.json guardado");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') calibrar();
    if (c == 'p' || c == 'P') mostrarArchivo();
  }

  if (mpu.update() && millis() - tPrint > 300) {
    tPrint = millis();
    Serial.printf("Yaw: %.1f  Pitch: %.1f  Roll: %.1f\n",
                  mpu.getYaw(), mpu.getPitch(), mpu.getRoll());
  }
}