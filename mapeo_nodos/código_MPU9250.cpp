#include <Wire.h>

// Direcciones I2C por defecto
const uint8_t MPU9250_ADDR = 0x68; // Cambia a 0x69 si el pin AD0 está conectado a 3.3V
const uint8_t AK8963_ADDR  = 0x0C; // Dirección interna de la brújula

void setup() {
  Serial.begin(115200);
  Wire.begin(); // En el ESP32 los pines I2C por defecto son SDA=21, SCL=22

  // 1. Despertar el MPU9250 (Registro PWR_MGMT_1)
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

  // 2. Habilitar modo "Bypass" para exponer el magnetómetro AK8963 (Registro INT_PIN_CFG)
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
  delay(50);

  // 3. Configurar magnetómetro (Registro CNTL1, modo continuo a 100Hz, 16 bits)
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x0A);
  Wire.write(0x16);
  Wire.endTransmission();
  delay(50);

  Serial.println("Iniciando lectura de sensores en crudo...");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz, mx, my, mz, tempRaw;

  // --- LEER ACELERÓMETRO Y GIROSCOPIO (14 bytes en total) ---
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B); // Registro inicial (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() >= 14) {
    // El MPU9250 envía primero el byte más significativo (Big Endian)
    ax = (Wire.read() << 8 | Wire.read());
    ay = (Wire.read() << 8 | Wire.read());
    az = (Wire.read() << 8 | Wire.read());
    tempRaw = (Wire.read() << 8 | Wire.read()); // Temperatura (la leemos para no desfasarnos)
    gx = (Wire.read() << 8 | Wire.read());
    gy = (Wire.read() << 8 | Wire.read());
    gz = (Wire.read() << 8 | Wire.read());
  }

  // --- LEER MAGNETÓMETRO (Brújula) ---
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x03); // Registro inicial (HXL)
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, (uint8_t)7, (uint8_t)true); // Son 6 bytes de datos + 1 byte de estado

  if (Wire.available() >= 7) {
    // El AK8963 envía primero el byte MENOS significativo (Little Endian)
    mx = (Wire.read() | Wire.read() << 8);
    my = (Wire.read() | Wire.read() << 8);
    mz = (Wire.read() | Wire.read() << 8);
    uint8_t st2 = Wire.read(); // Es OBLIGATORIO leer el registro ST2 para que la brújula libere la siguiente lectura
  }

  // --- IMPRESIÓN EN FORMATO AMIGABLE ---
  Serial.print("Acel[X:"); Serial.print(ax); Serial.print(" Y:"); Serial.print(ay); Serial.print(" Z:"); Serial.print(az); Serial.print("]\t");
  Serial.print("Giro[X:"); Serial.print(gx); Serial.print(" Y:"); Serial.print(gy); Serial.print(" Z:"); Serial.print(gz); Serial.print("]\t");
  Serial.print("Mag[X:"); Serial.print(mx); Serial.print(" Y:"); Serial.print(my); Serial.print(" Z:"); Serial.print(mz); Serial.println("]");

  delay(200); // 5 lecturas por segundo para poder leerlas en pantalla
}