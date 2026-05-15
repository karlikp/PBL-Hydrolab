# H2O Drone – Water Quality Monitoring

FastAPI backend + web dashboard for a water-quality drone. A radio module streams
telemetry frames over a serial link; the backend validates, stores, and exposes
them; the browser UI plots them on a map and on live charts.

> Branch `chart-extended` adds: oxygen chart, per-parameter statistics cards,
> Flatpickr date/time filter, chart zoom/pan, CSV export and PNG chart export.

---

## Architecture

```
 ┌────────────┐   serial    ┌──────────────────┐   SQLite   ┌────────────┐
 │ Drone / TX │ ──────────▶ │ RadioService     │ ─────────▶ │ drone_data │
 │ (SiK radio)│   57600 8N1 │ (background      │            │   .db      │
 └────────────┘             │  thread)         │            └─────┬──────┘
                            └────────┬─────────┘                  │
                                     │                            │
                                     ▼                            ▼
                            ┌──────────────────┐    GET /api/data/all_readings
                            │ FastAPI (main.py)│ ◀───────────────────────────┐
                            └────────┬─────────┘                             │
                                     │ /static                               │
                                     ▼                                       │
                            ┌──────────────────┐                             │
                            │ static/index.html│ ─── fetch() every 5s ───────┘
                            │ Leaflet + Chart.js│
                            └──────────────────┘
```

### Data flow

1. Drone transmits an ASCII line over the radio:
   `Time,Lat,Lon,Cond,Temp,pH,O2,WaterFlag*ADLER32HEX`
2. `RadioService._listen_loop` reads lines, splits on `*`, recomputes **Adler-32**
   over the data portion and compares against the received hex checksum.
3. Valid frames are parsed (lat/lon are divided by `10_000_000`), inserted into
   the `telemetry` SQLite table, and timestamped with server-side `datetime.now()`.
4. The browser polls `/api/data/all_readings` every 5 s (paused while a date
   filter is active) and rerenders the map, charts, stats, and history list.

---

## Repository layout

```
main.py                     FastAPI app + lifespan (boots DB + radio thread)
requirements.txt
app/
  api/endpoints.py          /api/data/submit_reading, /api/data/all_readings
  database.py               sqlite3 wrapper: init_db, insert_reading, get_all_readings
  schemas/frame_reading.py  Pydantic model for a reading
  services/radio.py         Serial listener + Adler-32 verification
static/index.html           Dashboard (Leaflet, Chart.js, chartjs-plugin-zoom, Flatpickr)
```

### Frame / table schema

| Field         | Type    | Notes                                           |
|---------------|---------|-------------------------------------------------|
| `timestamp`   | TEXT    | ISO-8601, set server-side on receipt           |
| `lat`, `lon`  | REAL    | degrees; wire format is int ×10⁷              |
| `cond`        | REAL    | conductivity (µS)                               |
| `temp`        | REAL    | °C                                              |
| `ph`          | REAL    |                                                 |
| `oxygen`      | REAL    | mg/L                                            |
| `water_flag`  | INTEGER | 0/1, whether the probe is submerged            |
| `checksum`    | INTEGER | received Adler-32                               |

The `submit_reading` endpoint silently discards frames where any sensor value
is `0` (treated as "sensor not ready") — it returns the payload without
inserting it.

---

## Running locally

Requirements: Python 3.10+, a serial device at `/dev/ttyUSB1` (change
`RADIO_PORT` in `main.py` if yours differs — the current default is hard-coded).

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
uvicorn main:app --reload
```

Then open:

- Dashboard: http://127.0.0.1:8000/static/
- API docs:  http://127.0.0.1:8000/docs
- Health:    http://127.0.0.1:8000/health

The SQLite file `drone_data.db` is created in the working directory on first
run. If you don't have radio hardware, you can still populate the DB via
`POST /api/data/submit_reading`.

---

## API

| Method | Path                         | Purpose                                  |
|--------|------------------------------|------------------------------------------|
| GET    | `/health`                    | Liveness + whether the radio thread runs |
| POST   | `/api/data/submit_reading`   | Manual insert (testing); rejects zeroed frames |
| GET    | `/api/data/all_readings`     | Last 5000 readings, newest first          |

---

## Dashboard features (added on this branch)

- **Map** (Leaflet + OSM tiles) with one marker per filtered point; popup shows
  timestamp, pH, oxygen.
- **Four synchronized line charts**: pH, oxygen, temperature, conductivity.
  Hovering on any chart highlights the same index across all four and opens
  the corresponding map marker.
- **Linear trend line** overlaid on each chart (least-squares over the filtered
  window).
- **Stat cards** per parameter: min / avg / max over the current filter.
- **Flatpickr** date-time range filter (Polish locale, 24h). When no range is
  set, the charts show the last 25 points and auto-refresh every 5 s.
- **Zoom/pan** on charts via `chartjs-plugin-zoom` (mouse wheel + drag);
  "Domyślny widok wykresu" resets all charts at once.
- **CSV export** (semicolon-separated, UTF-8 BOM, comma decimal → opens
  cleanly in Polish Excel).
- **PNG export** stitches all four chart canvases vertically into one image.

---

## Notes for contributors

- The radio thread is a `daemon=True` `threading.Thread` started from the
  FastAPI lifespan; SQLite connections use `check_same_thread=False` so both
  the radio thread and request handlers can write.
- Checksum is **Adler-32** (matches the C++ sender); the older XOR
  implementation was replaced in commit `b3a6ad9`.
- Lat/Lon arrive as scaled integers and are divided by 1e7 in
  `RadioService._process_data` (commit `ea1356e`) — don't scale again
  downstream.
- CORS is wide open (`allow_origins=["*"]`) for local development; tighten
  before deploying.
