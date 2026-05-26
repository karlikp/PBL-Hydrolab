"""SQLite persistence for the ground station.

Schema (current):

  measurements  one row per TLM frame received while the drone is in
                MEASURING. `measurement_valid` distinguishes settled
                readings from in-flight settle samples — both are kept;
                the UI filters on the flag for headline plots.
  collections   one row per EVT,Cx,COLLECTED — per-tank physical-water
                capture, geo-tagged at the moment the tank reached FULL.
  events        one row per EVT frame (any kind) — operator audit log.

Legacy:

  telemetry     previous-semester schema, populated by an older parser
                against an older firmware format. Left in place so old
                data is preserved, but not written to or read by the
                current code path.
"""

import sqlite3
from typing import Optional

DB_NAME = "drone_data.db"


def get_db_connection():
    conn = sqlite3.connect(DB_NAME, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    conn = get_db_connection()
    cur = conn.cursor()

    # Legacy table — never written to by new code; kept so the file
    # can hold pre-existing data without us destroying it.
    cur.execute('''
        CREATE TABLE IF NOT EXISTS telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT NOT NULL,
            lat REAL,
            lon REAL,
            cond REAL,
            temp REAL,
            ph REAL,
            oxygen REAL,
            water_flag INTEGER,
            checksum INTEGER
        )
    ''')

    cur.execute('''
        CREATE TABLE IF NOT EXISTS measurements (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            received_at TEXT NOT NULL,
            drone_utc TEXT,
            lat REAL,
            lon REAL,
            cond REAL,
            temp REAL,
            ph REAL,
            oxygen REAL,
            water_flag INTEGER,
            measurement_valid INTEGER
        )
    ''')

    cur.execute('''
        CREATE TABLE IF NOT EXISTS collections (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            received_at TEXT NOT NULL,
            tank TEXT NOT NULL,
            collected_at_utc TEXT,
            lat REAL,
            lon REAL,
            fix_valid INTEGER
        )
    ''')

    cur.execute('''
        CREATE TABLE IF NOT EXISTS events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            received_at TEXT NOT NULL,
            source TEXT NOT NULL,
            kind TEXT NOT NULL,
            details TEXT
        )
    ''')

    conn.commit()
    conn.close()
    print(f"[Database] Initialized {DB_NAME}")


# ---------- measurements ----------

def insert_measurement(row: dict):
    conn = get_db_connection()
    conn.execute('''
        INSERT INTO measurements
            (received_at, drone_utc, lat, lon, cond, temp, ph, oxygen,
             water_flag, measurement_valid)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    ''', (
        row["received_at"], row.get("drone_utc"),
        row.get("lat"), row.get("lon"),
        row.get("cond"), row.get("temp"), row.get("ph"), row.get("oxygen"),
        1 if row.get("water_flag") else 0,
        1 if row.get("measurement_valid") else 0,
    ))
    conn.commit()
    conn.close()


def get_recent_measurements(limit: int = 5000,
                            valid_only: bool = False,
                            start_iso: Optional[str] = None,
                            end_iso: Optional[str] = None) -> list[dict]:
    conn = get_db_connection()
    sql = "SELECT * FROM measurements WHERE 1=1"
    args: list = []
    if valid_only:
        sql += " AND measurement_valid = 1"
    if start_iso:
        sql += " AND received_at >= ?"
        args.append(start_iso)
    if end_iso:
        sql += " AND received_at <= ?"
        args.append(end_iso)
    sql += " ORDER BY id DESC LIMIT ?"
    args.append(limit)

    rows = conn.execute(sql, args).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ---------- collections ----------

def insert_collection(row: dict):
    conn = get_db_connection()
    conn.execute('''
        INSERT INTO collections
            (received_at, tank, collected_at_utc, lat, lon, fix_valid)
        VALUES (?, ?, ?, ?, ?, ?)
    ''', (
        row["received_at"], row["tank"], row.get("collected_at_utc"),
        row.get("lat"), row.get("lon"),
        1 if row.get("fix_valid") else 0,
    ))
    conn.commit()
    conn.close()


def get_recent_collections(limit: int = 1000) -> list[dict]:
    conn = get_db_connection()
    rows = conn.execute(
        "SELECT * FROM collections ORDER BY id DESC LIMIT ?", (limit,)
    ).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ---------- events ----------

def insert_event(row: dict):
    conn = get_db_connection()
    conn.execute('''
        INSERT INTO events (received_at, source, kind, details)
        VALUES (?, ?, ?, ?)
    ''', (
        row["received_at"], row["source"], row["kind"], row.get("details"),
    ))
    conn.commit()
    conn.close()


def get_recent_events(limit: int = 500) -> list[dict]:
    conn = get_db_connection()
    rows = conn.execute(
        "SELECT * FROM events ORDER BY id DESC LIMIT ?", (limit,)
    ).fetchall()
    conn.close()
    return [dict(r) for r in rows]
