# EcoGuía
[![Ver video en YouTube](https://img.youtube.com/vi/rLsQrRdL5t0/maxresdefault.jpg)](https://www.youtube.com/watch?v=rLsQrRdL5t0)

EcoGuía es un prototipo de bajo costo que orienta a personas con discapacidad visual por rutas predefinidas dentro del campus de la Universidad de El Salvador. El sistema está formado por un dispositivo portátil basado en ESP32 y una aplicación Android que actúa como fuente principal de navegación.

Proyecto desarrollado para la asignatura Arquitectura de Computadoras (ARC115), UES.

## Funcionalidades

- Guía por indicaciones sonoras con buzzer: L (izquierda), R (derecha) y F (recto).
- Posicionamiento con GPS NEO-6M, con respaldo en el GPS del teléfono si no hay conexión Bluetooth.
- Orientación mediante magnetómetro y giroscopio del MPU9250.
- Cálculo del tiempo estimado de recorrido con velocidad de caminata configurable.
- Indicación del siguiente obstáculo y su distancia.
- Seguimiento de la ubicación por parte de una persona responsable, mediante Supabase.
- Modo de simulación para pruebas sin hardware.

## Arquitectura

- **Dispositivo:** ESP32 DevKit (chip clásico), GPS NEO-6M por UART, MPU9250 por I2C, buzzer en GPIO25 y alimentación con batería LiPo, módulo TP4056 y convertidor boost a 5 V.
- **Aplicación:** Android con Kotlin, mapa con Mapbox y backend en Supabase.
- **Comunicación:** Bluetooth Classic (SPP).

## Protocolo Bluetooth

| Dirección | Mensaje | Descripción |
|---|---|---|
| ESP32 → app | `Y,rumbo,giro` | Rumbo magnético y velocidad angular en Z, cada 100 ms |
| ESP32 → app | `P,latitud,longitud,fix` | Posición del GPS del ESP32 (si el firmware lo envía) |
| App → ESP32 | `L`, `R`, `F` | Indicación de guía |
| App → ESP32 | `H` | Latido cada segundo; sin mensajes en 3 s activa el respaldo |

## Requisitos

- Android Studio [versión].
- Android [minSdk] o superior.
- Cuenta de Mapbox con token público (`pk.`) y token de descargas (`sk.`) con permiso DOWNLOADS:READ.
- Proyecto de Supabase con las tablas usuarios, rutas, nodos, aristas, seguimientos, posiciones_usuario y accesos_encargado.
- Firmware del ESP32 cargado con Arduino IDE o PlatformIO.

## Configuración

Crea o edita `local.properties` en la raíz del proyecto (no se sube a Git):

```properties
MAPBOX_PUBLIC_TOKEN=pk.tu_token
SUPABASE_URL=https://tu-proyecto.supabase.co
SUPABASE_ANON_KEY=tu_clave_anon
```

En `gradle.properties` de tu usuario, agrega el token de descargas:

```properties
MAPBOX_DOWNLOADS_TOKEN=sk.tu_token
```

Antes del primer commit, verifica que `.gitignore` excluya `local.properties`, `.gradle/`, `build/` y `.idea/`.

## Emparejamiento del dispositivo

1. Carga el firmware en el ESP32.
2. En Ajustes del teléfono, empareja el dispositivo con el nombre definido en la constante del código (`[NOMBRE_BT]`).
3. Abre la app y conéctate desde la pantalla principal.

## Modo de simulación

Para probar sin ESP32, activa la bandera `[NOMBRE_FLAG]` en `[archivo]`. La app generará posiciones y rumbos siguiendo una ruta de prueba.

## Pruebas

```bash
./gradlew test
```

Las pruebas unitarias cubren la fórmula de Haversine, los umbrales de dirección y el cálculo del tiempo estimado.

## Estado del proyecto

- [ ] Conexión Bluetooth Classic con el ESP32
- [ ] Guía con GPS del ESP32 y MPU9250
- [ ] Respaldo con GPS del teléfono
- [ ] Tiempo estimado de recorrido
- [ ] Siguiente obstáculo y distancia
- [ ] Rastreo por el encargado

## Limitaciones

- La precisión del GPS disminuye en interiores y bajo edificios altos.
- El sistema solo sigue rutas predefinidas.
- No detecta obstáculos en tiempo real; los obstáculos son puntos registrados en la ruta.

## Autores

[Nombres del equipo] — UES, [carrera].

## Licencia

Producto bajo licencia de la cátedra de Arquitectura de Computadoras 
