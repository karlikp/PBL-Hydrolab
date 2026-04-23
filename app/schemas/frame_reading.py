from pydantic import BaseModel, Field
from datetime import datetime

class FrameReading(BaseModel):
    """
    Definicja struktury danych pomiarowych (ramki) z drona.
    Format: Time, Lat, Lon, Sats, FixType, Cond, Temp, pH, WaterFlag
    """
    time: datetime
    lat: float
    lon: float 
    cond: float
    temp: float
    ph: float
    oxygen: float
    checksum: int 