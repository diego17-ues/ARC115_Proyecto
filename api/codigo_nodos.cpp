#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>

// =====================================================
// CONFIGURACIÓN: CAMBIAR ESTOS DATOS
// =====================================================

const char* WIFI_SSID = "NOMBRE DE RED";
const char* WIFI_PASSWORD = "CONTRASEÑA DE RED";

// IP de la computadora que ejecuta FastAPI.
const char* API_URL = "http://192.10.20.3:8000"; //Ejemplo//
// Debe coincidir con MAPA_API_KEY del servidor.
const char* API_KEY = "contraseña_api"; // a su elección//

// GPS TX -> GPIO16 del ESP32.
// GPS RX -> GPIO17 del ESP32, opcional.
constexpr int GPS_RX_PIN = 16;
constexpr int GPS_TX_PIN = 17;

constexpr uint32_t GPS_BAUD = 9600;

// Criterios iniciales de calidad.
// No garantizan exactitud; ayudan a rechazar malos datos.
constexpr uint32_t MAX_EDAD_POSICION_MS = 2000;
constexpr uint32_t MAX_EDAD_CALIDAD_MS = 3000;

constexpr uint32_t MIN_SATELITES = 3;
constexpr double MAX_HDOP = 5.8;

// =====================================================
// OBJETOS Y ESTADO
// =====================================================

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

String lineaSerial;
bool descartarLinea = false;

bool wifiEstabaConectado = false;

// =====================================================
// GPS
// =====================================================

void leerGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

bool gpsAceptable() {
  return
    gps.location.isValid() &&
    gps.location.age() < MAX_EDAD_POSICION_MS &&

    gps.satellites.isValid() &&
    gps.satellites.age() < MAX_EDAD_CALIDAD_MS &&
    gps.satellites.value() >= MIN_SATELITES &&

    gps.hdop.isValid() &&
    gps.hdop.age() < MAX_EDAD_CALIDAD_MS &&
    gps.hdop.hdop() <= MAX_HDOP;
}

void mostrarGPS() {
  Serial.println();
  Serial.println("========== ESTADO GPS ==========");

  Serial.printf(
    "Caracteres recibidos: %lu\n",
    (unsigned long)gps.charsProcessed()
  );

  Serial.printf(
    "Sentencias con checksum correcto: %lu\n",
    (unsigned long)gps.passedChecksum()
  );

  if (gps.location.isValid()) {
    Serial.printf("Latitud:  %.8f\n", gps.location.lat());
    Serial.printf("Longitud: %.8f\n", gps.location.lng());

    Serial.printf(
      "Edad de posicion: %lu ms\n",
      (unsigned long)gps.location.age()
    );
  } else {
    Serial.println("Posicion: todavia no disponible.");
  }

  if (gps.satellites.isValid()) {
    Serial.printf(
      "Satelites: %lu\n",
      (unsigned long)gps.satellites.value()
    );
  } else {
    Serial.println("Satelites: sin informacion.");
  }

  if (gps.hdop.isValid()) {
    Serial.printf("HDOP: %.2f\n", gps.hdop.hdop());
  } else {
    Serial.println("HDOP: sin informacion.");
  }

  Serial.printf(
    "Listo para registrar: %s\n",
    gpsAceptable() ? "SI" : "NO"
  );

  if (gps.charsProcessed() == 0) {
    Serial.println(
      "No llegan datos: revisar alimentacion, GND y TX del GPS."
    );
  }

  Serial.println("================================");
}

void reiniciarLecturaTrasHTTP() {
  /*
   * HTTP es bloqueante en esta versión.
   *
   * Descartamos los datos acumulados durante la petición
   * y esperamos nuevas sentencias para no registrar una
   * posición antigua como si fuera reciente.
   */
  while (gpsSerial.available()) {
    gpsSerial.read();
  }

  gps = TinyGPSPlus();

  Serial.println(
    "Esperando nuevas lecturas GPS antes del siguiente nodo."
  );
}

// =====================================================
// ESTADO Y AYUDA
// =====================================================

void mostrarAyuda() {
  Serial.println();
  Serial.println("========== COMANDOS ==========");
  Serial.println("GPS");
  Serial.println("  Mostrar estado y coordenadas.");
  Serial.println();
  Serial.println("ESTADO");
  Serial.println("  Mostrar conexion Wi-Fi.");
  Serial.println();
  Serial.println("N nombre|descripcion|id_conexion");
  Serial.println("  Guardar un nodo en la API.");
  Serial.println("  Usar 0 para no conectarlo.");
  Serial.println();
  Serial.println("Ejemplo primer nodo:");
  Serial.println("N Entrada principal|Inicio del recorrido|0");
  Serial.println();
  Serial.println("Ejemplo conectado al nodo 1:");
  Serial.println("N Cruce del jardin|Interseccion peatonal|1");
  Serial.println();
  Serial.println("AYUDA");
  Serial.println("==============================");
}

void mostrarEstado() {
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi: conectado.");

    Serial.print("IP del ESP32: ");
    Serial.println(WiFi.localIP());

    Serial.printf("Senal Wi-Fi: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("Wi-Fi: desconectado.");
  }

  Serial.print("Servidor configurado: ");
  Serial.println(API_URL);

  Serial.printf(
    "Heap libre: %u bytes\n",
    (unsigned)ESP.getFreeHeap()
  );
}

// =====================================================
// ENVÍO A LA API
// =====================================================

void enviarNodo(
  const String& nombre,
  const String& descripcion,
  int conectarCon
) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERROR: no hay conexion Wi-Fi.");
    Serial.println("El nodo NO se guardo.");
    return;
  }

  // Procesar cualquier dato disponible antes de validar.
  leerGPS();

  if (!gpsAceptable()) {
    Serial.println("ERROR: GPS sin calidad suficiente.");
    Serial.println("El nodo NO se guardo.");
    mostrarGPS();
    return;
  }

  // Capturar la posición antes de iniciar la petición.
  const double latitud = gps.location.lat();
  const double longitud = gps.location.lng();

  JsonDocument documento;

  documento["latitud"] = latitud;
  documento["longitud"] = longitud;
  documento["nombre"] = nombre;

  // Se puede cambiar posteriormente desde la API.
  documento["tipo"] = "punto";
  documento["descripcion"] = descripcion;

  if (conectarCon > 0) {
    documento["conectar_con"] = conectarCon;
  }

  String cuerpo;
  serializeJson(documento, cuerpo);

  Serial.println();
  Serial.println("Enviando nodo...");
  Serial.printf("Lugar: %s\n", nombre.c_str());
  Serial.printf("Posicion: %.8f, %.8f\n", latitud, longitud);

  if (conectarCon > 0) {
    Serial.printf("Conectar con: %d\n", conectarCon);
  } else {
    Serial.println("Sin conexion con otro nodo.");
  }

  WiFiClient cliente;
  HTTPClient http;

  String url = String(API_URL) + "/nodos";

  if (!http.begin(cliente, url)) {
    Serial.println("ERROR: no se pudo iniciar HTTP.");
    return;
  }

  http.setConnectTimeout(3000);
  http.setTimeout(5000);

  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);

  int codigo = http.POST(cuerpo);

  if (codigo > 0) {
    String respuesta = http.getString();

    Serial.printf("HTTP: %d\n", codigo);

    if (codigo == 201) {
      Serial.println("NODO GUARDADO EN LA COMPUTADORA.");

      JsonDocument resultado;
      DeserializationError error =
        deserializeJson(resultado, respuesta);

      if (!error && resultado["nodo"]["id"].is<int>()) {
        int id = resultado["nodo"]["id"].as<int>();

        Serial.printf("ID asignado: %d\n", id);
        Serial.println(
          "Usa este ID si quieres conectar el siguiente nodo."
        );

        if (!resultado["conexion"].isNull()) {
          double distancia =
            resultado["conexion"]["distancia_m"].as<double>();

          Serial.printf(
            "Distancia de la conexion: %.2f metros\n",
            distancia
          );
        }
      } else {
        Serial.println(
          "No se pudo interpretar la respuesta. "
          "Consulta GET /nodos para obtener el ID."
        );
        Serial.println(respuesta);
      }

    } else {
      Serial.println("La API no confirmo la creacion.");
      Serial.println(respuesta);

      if (codigo == 401) {
        Serial.println("Revisa la clave API.");
      } else if (codigo == 404) {
        Serial.println(
          "Revisa si existe el nodo al que intentas conectarte "
          "y si la URL corresponde a la API correcta."
        );
      } else if (codigo == 422) {
        Serial.println("La API rechazo el formato de los datos.");
      }

      Serial.println(
        "Si hay duda sobre el resultado, consulta GET /nodos "
        "antes de repetir."
      );
    }

  } else {
    Serial.print("Error de comunicacion: ");
    Serial.println(HTTPClient::errorToString(codigo));

    Serial.println(
      "No se recibio confirmacion. El servidor pudo haber "
      "guardado el nodo: revisa GET /nodos antes de repetir."
    );
  }

  http.end();
  reiniciarLecturaTrasHTTP();
}

// =====================================================
// COMANDOS
// =====================================================

void procesarComando(String comando) {
  comando.trim();

  if (comando.isEmpty()) return;

  if (comando == "GPS") {
    mostrarGPS();
    return;
  }

  if (comando == "ESTADO") {
    mostrarEstado();
    return;
  }

  if (comando == "AYUDA") {
    mostrarAyuda();
    return;
  }

  if (!comando.startsWith("N ")) {
    Serial.println("Comando desconocido. Escribe AYUDA.");
    return;
  }

  String datos = comando.substring(2);

  int separador1 = datos.indexOf('|');

  if (separador1 < 0) {
    Serial.println("Formato: N nombre|descripcion|id_conexion");
    return;
  }

  int separador2 = datos.indexOf('|', separador1 + 1);

  if (separador2 < 0 ||
      datos.indexOf('|', separador2 + 1) >= 0) {
    Serial.println("Usa exactamente dos separadores |");
    Serial.println("Formato: N nombre|descripcion|id_conexion");
    return;
  }

  String nombre = datos.substring(0, separador1);
  String descripcion =
    datos.substring(separador1 + 1, separador2);
  String textoId = datos.substring(separador2 + 1);

  nombre.trim();
  descripcion.trim();
  textoId.trim();

  if (nombre.isEmpty() || nombre.length() > 80) {
    Serial.println("Nombre vacio o demasiado largo.");
    return;
  }

  if (descripcion.length() > 200) {
    Serial.println("Descripcion demasiado larga.");
    return;
  }

  if (textoId.isEmpty() || textoId.length() > 9) {
    Serial.println("ID invalido.");
    return;
  }

  for (size_t i = 0; i < textoId.length(); i++) {
    if (textoId[i] < '0' || textoId[i] > '9') {
      Serial.println("El ID debe ser un entero no negativo.");
      return;
    }
  }

  int conectarCon = textoId.toInt();

  enviarNodo(nombre, descripcion, conectarCon);
}

void leerComandos() {
  while (Serial.available()) {
    char caracter = Serial.read();

    if (caracter == '\n') {
      if (!descartarLinea) {
        procesarComando(lineaSerial);
      }

      lineaSerial = "";
      descartarLinea = false;

    } else if (caracter != '\r' && !descartarLinea) {
      if (lineaSerial.length() < 320) {
        lineaSerial += caracter;
      } else {
        lineaSerial = "";
        descartarLinea = true;

        Serial.println(
          "Comando demasiado largo. Se descarta esta linea."
        );
      }
    }
  }
}

// =====================================================
// INICIO Y BUCLE
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  lineaSerial.reserve(321);

  gpsSerial.setRxBufferSize(4096);
  gpsSerial.begin(
    GPS_BAUD,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println();
  Serial.println("REGISTRADOR DE NODOS GPS");
  Serial.println("Iniciando conexion Wi-Fi...");
  Serial.println("Coloca el GPS al aire libre.");
  Serial.println("Monitor serie: 115200 y Nueva linea.");

  mostrarAyuda();
}

void loop() {
  leerGPS();

  bool conectado = WiFi.status() == WL_CONNECTED;

  if (conectado != wifiEstabaConectado) {
    wifiEstabaConectado = conectado;

    if (conectado) {
      Serial.println();
      Serial.println("Wi-Fi conectado.");

      Serial.print("IP del ESP32: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("Se perdio la conexion Wi-Fi.");
    }
  }

  leerComandos();
  delay(1);
}