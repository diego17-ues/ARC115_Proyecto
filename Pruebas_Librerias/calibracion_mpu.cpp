/*
 * PROGRAMA 2: Calibración del Magnetómetro del MPU-9250
 * Hardware: ESP32 + MPU-9250
 * 
 * Conexiones I2C estándar del ESP32:
 * MPU SDA  -> ESP32 D21
 * MPU SCL  -> ESP32 D22
 * MPU VCC  -> ESP32 3.3V
 * MPU GND  -> ESP32 GND
 */

#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

void setup() {
    // Configurar monitor serie a 115200 baudios
    Serial.begin(115200);
    
    // Iniciar el bus I2C (SDA = 21, SCL = 22)
    Wire.begin();
    
    // Pausa de 3 segundos para darte tiempo de abrir el Monitor Serie
    delay(3000); 
    Serial.println("\n--- Iniciando MPU-9250 ---");

    // Iniciar comunicación con la dirección 0x68 (por defecto en la mayoría de MPU9250)
    if (!mpu.setup(0x68)) {  
        while (1) {
            Serial.println("Error: No se encontro el MPU-9250. Revisa cables SDA/SCL o la energia.");
            delay(3000);
        }
    }

    Serial.println("Sensor detectado correctamente.");
    Serial.println("\n==========================================================");
    Serial.println("ATENCION: Calibracion de la brujula en 3 segundos.");
    Serial.println("Preparate para mover tu prototipo formando un numero '8'");
    Serial.println("en el aire, rotandolo en todos sus ejes (3D).");
    Serial.println("No te detengas hasta ver el mensaje de finalizacion.");
    Serial.println("==========================================================");
    
    delay(3000); 
    Serial.println("\n*** CALIBRANDO... MUEVE EL SENSOR AHORA! ***");
    
    // Ejecutar rutina de calibración. Esto tomará aproximadamente 15 segundos.
    mpu.calibrateMag(); 

    Serial.println("\n==========================================================");
    Serial.println("CALIBRACION FINALIZADA. EXCELENTE.");
    Serial.println("Copia estos 6 valores y guardalos en un bloc de notas.");
    Serial.println("Los insertaremos manualmente en el Programa 3:");
    Serial.println("----------------------------------------------------------");
    
    Serial.print("Mag Bias X:  "); Serial.println(mpu.getMagBiasX(), 6);
    Serial.print("Mag Bias Y:  "); Serial.println(mpu.getMagBiasY(), 6);
    Serial.print("Mag Bias Z:  "); Serial.println(mpu.getMagBiasZ(), 6);
    
    Serial.print("Mag Scale X: "); Serial.println(mpu.getMagScaleX(), 6);
    Serial.print("Mag Scale Y: "); Serial.println(mpu.getMagScaleY(), 6);
    Serial.print("Mag Scale Z: "); Serial.println(mpu.getMagScaleZ(), 6);
    Serial.println("==========================================================");
}

void loop() {
    // Para esta prueba no necesitamos leer datos continuos.
    // El programa se detiene aquí hasta que lo reinicies.
    delay(1000);
}