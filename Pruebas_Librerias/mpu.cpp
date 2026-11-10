/*
 * PROGRAMA 3 DEFINITIVO: Brújula + Feedback de Buzzer}
    HACE PITAR CUANDO ESTÁ ENTRE 10 Y 20 GRADOS (NORTE)
 * Hardware: ESP32 + MPU-9250 (GY-91) + Buzzer Low Level
 */

#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

// ====================================================================
// CONFIGURACIÓN DE TU PROTOTIPO
// ====================================================================
const float OFFSET_NORTE = 86.0; 

// Pin del Buzzer (Cámbialo si estás usando otro pin)
const int BUZZER_PIN = 4; 
// ====================================================================

void setup() {
    Serial.begin(115200);
    
    // Configurar Buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    // Como es Low Level Trigger, lo ponemos en HIGH para que empiece APAGADO
    digitalWrite(BUZZER_PIN, HIGH); 
    
    Wire.begin(21, 22);
    Wire.setClock(400000);
    
    delay(2000);

    Serial.println("\n--- Iniciando Brujula (MPU-9250) ---");
    
    if (!mpu.setup(0x68)) {  
        Serial.println("Error: No se encontro el MPU-9250.");
        delay(3000);
        ESP.restart();
    }

    Serial.println("Calibrando Giroscopio y Acelerometro...");
    Serial.println("NO MUEVAS EL SENSOR DURANTE 3 SEGUNDOS");
    delay(1000);
    mpu.calibrateAccelGyro(); 
    Serial.println("Giroscopio calibrado. Listo.");

    // ====================================================================
    // PEGA AQUÍ LAS 2 LÍNEAS DE CALIBRACIÓN MAGNÉTICA (Del Programa 2)
    // ====================================================================
    
    mpu.setMagBias(0.0, 0.0, 0.0);
    mpu.setMagScale(1.0, 1.0, 1.0);
    
    // ====================================================================

    mpu.setMagneticDeclination(-1.5);

    Serial.println("MPU-9250 configurado.");
    Serial.println("Leyendo datos... (Gira el sensor para probar)");
    delay(1000);
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        
        // Imprimir datos y evaluar el buzzer cada 200ms
        if (millis() > prev_ms + 200) {
            
            float yaw_crudo = mpu.getYaw();
            float yaw_corregido = yaw_crudo - OFFSET_NORTE;
            
            if (yaw_corregido < 0) {
                yaw_corregido += 360.0;
            } else if (yaw_corregido >= 360.0) {
                yaw_corregido -= 360.0;
            }
            
            Serial.print("Direccion (Yaw): ");
            Serial.print(yaw_corregido, 1);
            Serial.println(" grados");

            // --- LÓGICA DEL BUZZER (Low Level Trigger) ---
            if (yaw_corregido >= 0.0 && yaw_corregido <= 20.0) {
                digitalWrite(BUZZER_PIN, LOW);  // ENCENDER
            } else {
                digitalWrite(BUZZER_PIN, HIGH); // APAGAR
            }
            
            prev_ms = millis();
        }
    }
}