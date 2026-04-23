import sqlite3
import json
from datetime import datetime

DB_NAME = "drone_data.db"

def get_db_connection():
    """Creates a connection to the SQLite database."""
    conn = sqlite3.connect(DB_NAME, check_same_thread=False)
    # This allows us to access columns by name (row["lat"]) instead of index (row[1])
    conn.row_factory = sqlite3.Row 
    return conn

def init_db():
    """Creates the table if it doesn't exist."""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    cursor.execute('''
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
    
    conn.commit()
    conn.close()
    print(f"[Database] Initialized {DB_NAME}")

def insert_reading(data: dict):
    """Inserts a single reading into the database."""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    cursor.execute('''
        INSERT INTO telemetry (timestamp, lat, lon, cond, temp, ph, oxygen, water_flag, checksum)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
    ''', (
        data["time"],
        data["lat"],
        data["lon"],
        data["cond"],
        data["temp"],
        data["ph"],
        data["oxygen"],
        1 if data["water_flag"] else 0, # Convert bool to int for SQLite
        data["checksum"]
    ))
    
    conn.commit()
    conn.close()

def get_all_readings(limit=1000):
    """Fetches readings (newest first)."""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # Get last N readings
    cursor.execute('SELECT * FROM telemetry ORDER BY id DESC LIMIT ?', (limit,))
    rows = cursor.fetchall()
    conn.close()
    
    # Convert SQLite rows back to a clean list of dictionaries
    results = []
    for row in rows:
        results.append({
            "time": row["timestamp"],
            "lat": row["lat"],
            "lon": row["lon"],
            "cond": row["cond"],
            "temp": row["temp"],
            "ph": row["ph"],
            "oxygen": row["oxygen"],
            "water_flag": bool(row["water_flag"]),
            "checksum": row["checksum"]
        })
    
    return results