import math
import os
import secrets
import sqlite3

from contextlib import closing, asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Annotated

from fastapi import FastAPI, Depends, Header, HTTPException
from pydantic import BaseModel, ConfigDict, Field


DB_PATH = Path(__file__).resolve().parent / "mapa.db"
API_KEY = os.environ.get("MAPA_API_KEY")

if not API_KEY:
    raise RuntimeError("Debes definir la variable MAPA_API_KEY")


# ---------------- Seguridad ----------------

def verificar_clave(
    x_api_key: Annotated[str | None, Header()] = None
):
    if not secrets.compare_digest(x_api_key or "", API_KEY):
        raise HTTPException(401, "Clave API incorrecta")


# ---------------- Base de datos ----------------

def abrir_db():
    conexion = sqlite3.connect(DB_PATH, timeout=10)
    conexion.row_factory = sqlite3.Row
    conexion.execute("PRAGMA foreign_keys = ON")
    return conexion


@asynccontextmanager
async def lifespan(app: FastAPI):
    with closing(abrir_db()) as db:
        db.executescript("""
            CREATE TABLE IF NOT EXISTS nodos (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                latitud REAL NOT NULL,
                longitud REAL NOT NULL,
                nombre TEXT NOT NULL,
                tipo TEXT NOT NULL,
                descripcion TEXT NOT NULL,
                recibido_utc TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS aristas (
                origen INTEGER NOT NULL,
                destino INTEGER NOT NULL,
                distancia_m REAL NOT NULL,
                PRIMARY KEY (origen, destino),
                FOREIGN KEY (origen) REFERENCES nodos(id),
                FOREIGN KEY (destino) REFERENCES nodos(id),
                CHECK (origen < destino),
                CHECK (distancia_m >= 0)
            );
        """)
        db.commit()

    yield


app = FastAPI(
    title="API de mapa peatonal",
    version="1.0",
    lifespan=lifespan,
    dependencies=[Depends(verificar_clave)]
)


# ---------------- Modelos ----------------

class Lugar(BaseModel):
    nombre: str = Field(
        default="Sin asignar", min_length=1, max_length=80
    )
    tipo: str = Field(
        default="punto", min_length=1, max_length=32
    )
    descripcion: str = Field(default="", max_length=200)


class NodoNuevo(Lugar):
    model_config = ConfigDict(allow_inf_nan=False)

    latitud: float = Field(ge=-90, le=90)
    longitud: float = Field(ge=-180, le=180)

    # Opcional: conectar el nuevo nodo con uno existente.
    conectar_con: int | None = Field(default=None, ge=1)


class ConexionNueva(BaseModel):
    origen: int = Field(ge=1)
    destino: int = Field(ge=1)


# ---------------- Distancias ----------------

def distancia_metros(lat1, lon1, lat2, lon2):
    radio = 6371000.0

    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    dlat = lat2 - lat1
    dlon = math.radians(lon2 - lon1)

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1)
        * math.cos(lat2)
        * math.sin(dlon / 2) ** 2
    )

    a = max(0.0, min(1.0, a))

    return 2 * radio * math.atan2(
        math.sqrt(a),
        math.sqrt(1 - a)
    )


def obtener_nodo(db, nodo_id):
    fila = db.execute(
        "SELECT * FROM nodos WHERE id = ?",
        (nodo_id,)
    ).fetchone()

    if fila is None:
        raise HTTPException(404, f"No existe el nodo {nodo_id}")

    return fila


def conectar_nodos(db, origen, destino):
    if origen == destino:
        raise HTTPException(400, "No puedes conectar un nodo consigo mismo")

    a = obtener_nodo(db, origen)
    b = obtener_nodo(db, destino)

    distancia = distancia_metros(
        a["latitud"], a["longitud"],
        b["latitud"], b["longitud"]
    )

    # Arista no dirigida: almacenar una sola vez.
    origen, destino = sorted((origen, destino))

    db.execute("""
        INSERT INTO aristas (origen, destino, distancia_m)
        VALUES (?, ?, ?)
        ON CONFLICT(origen, destino)
        DO UPDATE SET distancia_m = excluded.distancia_m
    """, (origen, destino, distancia))

    return {
        "origen": origen,
        "destino": destino,
        "distancia_m": distancia
    }


# ---------------- Endpoints ----------------

@app.post("/nodos", status_code=201)
def crear_nodo(datos: NodoNuevo):
    recibido = datetime.now(timezone.utc).isoformat()

    with closing(abrir_db()) as db:
        # Si falla la conexión solicitada, también se deshace el alta.
        with db:
            cursor = db.execute("""
                INSERT INTO nodos (
                    latitud, longitud, nombre,
                    tipo, descripcion, recibido_utc
                )
                VALUES (?, ?, ?, ?, ?, ?)
            """, (
                datos.latitud,
                datos.longitud,
                datos.nombre,
                datos.tipo,
                datos.descripcion,
                recibido
            ))

            nodo_id = cursor.lastrowid
            conexion = None

            if datos.conectar_con is not None:
                conexion = conectar_nodos(
                    db,
                    datos.conectar_con,
                    nodo_id
                )

            nodo = dict(obtener_nodo(db, nodo_id))

    return {"nodo": nodo, "conexion": conexion}


@app.get("/nodos")
def listar_nodos():
    with closing(abrir_db()) as db:
        filas = db.execute(
            "SELECT * FROM nodos ORDER BY id"
        ).fetchall()

    return [dict(fila) for fila in filas]


@app.put("/nodos/{nodo_id}/lugar")
def asignar_lugar(nodo_id: int, datos: Lugar):
    with closing(abrir_db()) as db:
        with db:
            obtener_nodo(db, nodo_id)

            db.execute("""
                UPDATE nodos
                SET nombre = ?, tipo = ?, descripcion = ?
                WHERE id = ?
            """, (
                datos.nombre,
                datos.tipo,
                datos.descripcion,
                nodo_id
            ))

            resultado = dict(obtener_nodo(db, nodo_id))

    return resultado


@app.post("/aristas")
def crear_conexion(datos: ConexionNueva):
    with closing(abrir_db()) as db:
        with db:
            resultado = conectar_nodos(
                db,
                datos.origen,
                datos.destino
            )

    return resultado


@app.get("/mapa")
def exportar_mapa():
    with closing(abrir_db()) as db:
        # Mantener una vista consistente durante ambas consultas.
        db.execute("BEGIN")

        filas_nodos = db.execute(
            "SELECT * FROM nodos ORDER BY id"
        ).fetchall()

        filas_aristas = db.execute(
            "SELECT * FROM aristas ORDER BY origen, destino"
        ).fetchall()

        db.commit()

    nodos = {}

    for fila in filas_nodos:
        nodos[fila["id"]] = {
            "id": fila["id"],
            "gps": {
                "latitud": fila["latitud"],
                "longitud": fila["longitud"]
            },
            "lugar": {
                "nombre": fila["nombre"],
                "tipo": fila["tipo"],
                "descripcion": fila["descripcion"]
            },
            "recibido_utc": fila["recibido_utc"],
            "vecinos": []
        }

    for fila in filas_aristas:
        origen = fila["origen"]
        destino = fila["destino"]
        peso = fila["distancia_m"]

        nodos[origen]["vecinos"].append({
            "id": destino,
            "peso_m": peso
        })

        nodos[destino]["vecinos"].append({
            "id": origen,
            "peso_m": peso
        })

    return {
        "version": 1,
        "unidad_peso": "metros",
        "grafo_dirigido": False,
        "nodos": list(nodos.values())
    }