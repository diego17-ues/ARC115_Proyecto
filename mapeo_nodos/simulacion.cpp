// Simulación en PC, parte 2: (1) A* vs Dijkstra, (2) k rutas alternativas,
// (3) el problema del acelerómetro: doble integración vs ZUPT vs Kalman con GPS.
//
// Compilar:  g++ -std=c++17 -O2 -o sim2 sim_red_nodos_v2.cpp
// Ejecutar:  ./sim2 [semilla=1] [origen=0] [destino=13]

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <queue>
#include <random>
#include <algorithm>

// =====================================================================
//  Grafo CSR (igual que en la parte 1)
// =====================================================================
struct Nodo   { int32_t lat_e6, lon_e6; uint16_t primerVecino; uint8_t numVecinos; };
struct Arista { uint16_t destino; uint16_t peso_dm; uint8_t rumbo; };
static std::vector<Nodo> nodos;
static std::vector<Arista> aristas;

static const double D2R = M_PI / 180.0;
static double distM(int32_t la1, int32_t lo1, int32_t la2, int32_t lo2) {
  double latm = 0.5 * (la1 + la2) * 1e-6 * D2R;
  double dx = (lo2 - lo1) * 1e-6 * std::cos(latm) * 111320.0;
  double dy = (la2 - la1) * 1e-6 * 110540.0;
  return std::sqrt(dx * dx + dy * dy);
}
static double rumboDeg(int32_t la1, int32_t lo1, int32_t la2, int32_t lo2) {
  double latm = 0.5 * (la1 + la2) * 1e-6 * D2R;
  double dx = (lo2 - lo1) * 1e-6 * std::cos(latm), dy = (la2 - la1) * 1e-6;
  return std::fmod(std::atan2(dx, dy) / D2R + 360.0, 360.0);
}
static const double LAT0 = 13.7186, LON0 = -89.2012;
struct PtM { double este, norte; };
static const PtM RED[] = {
  {30,-30},{85,-20},{145,-35},{210,-20},{280,-30},
  {50,-85},{110,-80},{175,-90},{240,-80},{305,-90},
  {75,-145},{150,-142},{225,-145},{295,-145}
};
static const uint16_t ARI[][2] = {
  {0,1},{1,2},{2,3},{3,4},{0,5},{1,6},{2,6},{2,7},{3,8},{4,9},{5,6},
  {6,7},{7,8},{8,9},{5,10},{6,11},{7,11},{8,12},{9,13},{10,11},{11,12},{12,13}
};
static void construirGrafo() {
  const int n = sizeof(RED) / sizeof(RED[0]);
  std::vector<std::vector<uint16_t>> ady(n);
  for (auto &e : ARI) { ady[e[0]].push_back(e[1]); ady[e[1]].push_back(e[0]); }
  nodos.resize(n);
  for (int i = 0; i < n; i++) {
    nodos[i].lat_e6 = (int32_t)std::lround((LAT0 + RED[i].norte / 110540.0) * 1e6);
    nodos[i].lon_e6 = (int32_t)std::lround((LON0 + RED[i].este / (111320.0 * std::cos(LAT0 * D2R))) * 1e6);
  }
  for (int i = 0; i < n; i++) {
    nodos[i].primerVecino = (uint16_t)aristas.size();
    nodos[i].numVecinos = (uint8_t)ady[i].size();
    for (uint16_t j : ady[i]) {
      Arista a; a.destino = j;
      a.peso_dm = (uint16_t)std::lround(distM(nodos[i].lat_e6, nodos[i].lon_e6, nodos[j].lat_e6, nodos[j].lon_e6) * 10.0);
      a.rumbo = (uint8_t)((int)std::lround(rumboDeg(nodos[i].lat_e6, nodos[i].lon_e6, nodos[j].lat_e6, nodos[j].lon_e6) * 256.0 / 360.0) & 0xFF);
      aristas.push_back(a);
    }
  }
}
static int idxArista(uint16_t u, uint16_t v) {
  for (int k = 0; k < nodos[u].numVecinos; k++)
    if (aristas[nodos[u].primerVecino + k].destino == v) return nodos[u].primerVecino + k;
  return -1;
}

// =====================================================================
//  (1) Dijkstra / A* con montículo binario y multiplicadores de peso
// =====================================================================
static std::vector<double> mult;   // multiplicador por arista (1.0 = sin penalizar)

struct Res { std::vector<uint16_t> ruta; double metros = 0; int expandidos = 0; };

static Res buscar(uint16_t s, uint16_t t, bool astar) {
  const int n = (int)nodos.size();
  std::vector<double> g(n, 1e18);
  std::vector<int> prev(n, -1);
  std::vector<bool> cerrado(n, false);
  typedef std::pair<double, uint16_t> Par;
  std::priority_queue<Par, std::vector<Par>, std::greater<Par>> pq;
  auto h = [&](uint16_t u) -> double {
    if (!astar) return 0.0;
    return 0.99 * 10.0 * distM(nodos[u].lat_e6, nodos[u].lon_e6, nodos[t].lat_e6, nodos[t].lon_e6);
  };
  Res r;
  g[s] = 0; pq.push({h(s), s});
  while (!pq.empty()) {
    uint16_t u = pq.top().second; pq.pop();
    if (cerrado[u]) continue;
    cerrado[u] = true; r.expandidos++;
    if (u == t) break;
    for (int k = 0; k < nodos[u].numVecinos; k++) {
      int ia = nodos[u].primerVecino + k;
      uint16_t v = aristas[ia].destino;
      double ng = g[u] + aristas[ia].peso_dm * mult[ia];
      if (ng < g[v]) { g[v] = ng; prev[v] = u; pq.push({ng + h(v), v}); }
    }
  }
  for (int v = t; v != -1; v = prev[v]) r.ruta.push_back((uint16_t)v);
  std::reverse(r.ruta.begin(), r.ruta.end());
  for (size_t i = 0; i + 1 < r.ruta.size(); i++)
    r.metros += aristas[idxArista(r.ruta[i], r.ruta[i + 1])].peso_dm / 10.0;
  return r;
}
static void imprimirRuta(const Res &r) {
  for (size_t i = 0; i < r.ruta.size(); i++) std::printf("%s%u", i ? " -> " : "", r.ruta[i]);
}

// =====================================================================
//  (2) k rutas por penalización de aristas
// =====================================================================
static void kRutas(uint16_t s, uint16_t t, int k, double factor) {
  mult.assign(aristas.size(), 1.0);
  std::vector<std::vector<uint16_t>> vistas;
  for (int i = 0; i < k; i++) {
    Res r = buscar(s, t, true);
    bool repetida = false;
    for (auto &v : vistas) if (v == r.ruta) repetida = true;
    if (!repetida) {
      vistas.push_back(r.ruta);
      std::printf("  ruta %zu (%.0f m): ", vistas.size(), r.metros);
      imprimirRuta(r); std::printf("\n");
    }
    for (size_t j = 0; j + 1 < r.ruta.size(); j++) {
      mult[idxArista(r.ruta[j], r.ruta[j + 1])] *= factor;
      mult[idxArista(r.ruta[j + 1], r.ruta[j])] *= factor;
    }
  }
  mult.assign(aristas.size(), 1.0);
}

// =====================================================================
//  (3) Acelerómetro: por qué falla la doble integración y qué hacer
//      Modelo 1D a lo largo de la ruta (paradas, aceleración, crucero).
// =====================================================================
struct KF {                           // estado: [posición, velocidad, sesgo del acelerómetro]
  double x[3] = {0, 0, 0};
  double P[3][3] = {{25, 0, 0}, {0, 1, 0}, {0, 0, 0.01}};
  void predecir(double a, double dt, double qa, double qb) {
    double ac = a - x[2];
    x[0] += x[1] * dt + 0.5 * ac * dt * dt;
    x[1] += ac * dt;
    double F[3][3] = {{1, dt, -0.5 * dt * dt}, {0, 1, -dt}, {0, 0, 1}};
    double FP[3][3], Pn[3][3];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
      FP[i][j] = 0; for (int k = 0; k < 3; k++) FP[i][j] += F[i][k] * P[k][j];
    }
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
      Pn[i][j] = 0; for (int k = 0; k < 3; k++) Pn[i][j] += FP[i][k] * F[j][k];
    }
    double G[3] = {0.5 * dt * dt, dt, 0};
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) P[i][j] = Pn[i][j] + G[i] * G[j] * qa;
    P[2][2] += qb * dt;
  }
  void actualizar(int idx, double z, double R) {   // medida escalar directa de x[idx]
    double y = z - x[idx], S = P[idx][idx] + R, K[3], fila[3];
    for (int i = 0; i < 3; i++) { K[i] = P[i][idx] / S; fila[i] = P[idx][i]; }
    for (int i = 0; i < 3; i++) x[i] += K[i] * y;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) P[i][j] -= K[i] * fila[j];
  }
};

static void simAcelerometro(unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> N01(0.0, 1.0);

  const double DT = 0.01, TOTAL = 70.0;          // 100 Hz, 70 s
  const double G = 9.80665;
  const double SESGO = 0.03;                     // m/s^2, sesgo residual tras calibrar
  const double TILT = G * std::sin(0.5 * D2R);   // 0.5 grados de error de inclinacion -> fuga de gravedad
  const double RUIDO_QUIETO = 0.03, RUIDO_MOV = 0.30;  // vibración al moverse
  const double GPS_SIGMA_POS = 5.0, GPS_SIGMA_VEL = 0.3;
  const double APAGON_INI = 20.0, APAGON_FIN = 45.0;   // sin señal GPS

  auto aReal = [](double t) -> double {
    if (t >= 5 && t < 9)   return  1.5;
    if (t >= 25 && t < 29) return -1.5;
    if (t >= 39 && t < 43) return  1.5;
    if (t >= 60 && t < 64) return -1.5;
    return 0.0;
  };
  auto enMovimiento = [](double t) { return (t >= 5 && t < 29) || (t >= 39 && t < 64); };

  // detector de reposo: varianza de aceleración en ventana de 0.5 s
  const int W = 50;
  std::vector<double> buf(W, 0.0); int ib = 0, llenos = 0;

  double pR = 0, vR = 0;                         // verdad
  double pN = 0, vN = 0;                         // (a) doble integración pura
  double pZ = 0, vZ = 0, bZ = 0;                 // (b) integración + ZUPT + sesgo estimado en reposo
  KF kf;                                         // (c) Kalman GPS + acelerómetro + ZUPT
  double ultimoFixPos = 0;                       // (d) solo GPS (mantiene último fix)
  double maxErr[4] = {0, 0, 0, 0};

  std::printf("\nt(s)  real(m) | error GPS-solo | doble integr. | ZUPT+sesgo | Kalman GPS+IMU\n");
  std::printf("----------------------------------------------------------------------------\n");

  int pasos = (int)(TOTAL / DT);
  for (int i = 1; i <= pasos; i++) {
    double t = i * DT;
    double a = aReal(t);
    vR += a * DT; pR += vR * DT;

    double sigma = enMovimiento(t) ? RUIDO_MOV : RUIDO_QUIETO;
    double am = a + SESGO + TILT + sigma * N01(rng);  // lectura del acelerómetro

    // reposo?
    buf[ib] = am; ib = (ib + 1) % W; if (llenos < W) llenos++;
    bool quieto = false; double media = 0;
    if (llenos == W) {
      double s = 0, s2 = 0;
      for (double x : buf) { s += x; s2 += x * x; }
      media = s / W;
      double var = s2 / W - media * media;
      quieto = var < 0.01;                          // umbral entre 0.0009 (quieto) y 0.09 (movimiento)
    }

    // (a) doble integración pura
    vN += am * DT; pN += vN * DT;

    // (b) ZUPT: en reposo, v = 0 y la media de lo medido ES el sesgo
    if (quieto) { vZ = 0; bZ = media; }
    double aCorr = am - bZ;
    vZ += aCorr * DT; pZ += vZ * DT;
    if (quieto) vZ = 0;

    // (c) Kalman
    kf.predecir(am, DT, 0.09, 1e-6);
    if (quieto) kf.actualizar(1, 0.0, 0.01);
    bool hayGPS = !(t >= APAGON_INI && t < APAGON_FIN);
    if (i % 100 == 0 && hayGPS) {                   // 1 Hz
      double fixP = pR + GPS_SIGMA_POS * N01(rng);
      double fixV = vR + GPS_SIGMA_VEL * N01(rng);
      kf.actualizar(0, fixP, GPS_SIGMA_POS * GPS_SIGMA_POS);
      kf.actualizar(1, fixV, GPS_SIGMA_VEL * GPS_SIGMA_VEL);
      ultimoFixPos = fixP;
    }

    double e[4] = {std::fabs(ultimoFixPos - pR), std::fabs(pN - pR), std::fabs(pZ - pR), std::fabs(kf.x[0] - pR)};
    for (int k = 0; k < 4; k++) if (e[k] > maxErr[k]) maxErr[k] = e[k];

    if (i % 500 == 0)
      std::printf("%4.0f%s %7.1f  | %12.1f   | %12.1f  | %10.1f | %13.1f\n",
                  t, hayGPS ? " " : "*", pR, e[0], e[1], e[2], e[3]);
  }
  std::printf("----------------------------------------------------------------------------\n");
  std::printf("error maximo (m)    | %12.1f   | %12.1f  | %10.1f | %13.1f\n", maxErr[0], maxErr[1], maxErr[2], maxErr[3]);
  std::printf("(* = sin senal GPS entre %.0f s y %.0f s)\n", APAGON_INI, APAGON_FIN);
  std::printf("Fuga de gravedad por 0.5 grados de inclinacion: %.3f m/s^2, sesgo residual: %.3f m/s^2\n", TILT, SESGO);
}

// =====================================================================
int main(int argc, char **argv) {
  unsigned seed = (argc > 1) ? (unsigned)std::atoi(argv[1]) : 1;
  int s = (argc > 2) ? std::atoi(argv[2]) : 0;
  int t = (argc > 3) ? std::atoi(argv[3]) : 13;

  construirGrafo();
  if (s < 0 || t < 0 || s >= (int)nodos.size() || t >= (int)nodos.size()) { std::puts("nodos fuera de rango"); return 1; }
  mult.assign(aristas.size(), 1.0);

  std::printf("== (1) Dijkstra vs A* de %d a %d ==\n", s, t);
  Res d = buscar((uint16_t)s, (uint16_t)t, false);
  Res a = buscar((uint16_t)s, (uint16_t)t, true);
  std::printf("  Dijkstra: %.0f m, nodos expandidos: %d\n    ", d.metros, d.expandidos); imprimirRuta(d);
  std::printf("\n  A*:       %.0f m, nodos expandidos: %d\n    ", a.metros, a.expandidos); imprimirRuta(a);
  std::printf("\n");

  std::printf("\n== (2) k rutas alternativas (penalizando x2 las aristas usadas) ==\n");
  kRutas((uint16_t)s, (uint16_t)t, 5, 2.0);

  std::printf("\n== (3) Acelerometro: deriva vs correcciones (semilla %u) ==", seed);
  simAcelerometro(seed);
  return 0;
}