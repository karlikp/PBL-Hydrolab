from fastapi import APIRouter
from app.schemas.frame_reading import FrameReading
from app.database import get_all_readings, insert_reading

router = APIRouter(
    prefix="/api/data",
    tags=["Dane Pomiarowe"]
)

@router.post("/submit_reading", response_model=FrameReading)
def submit_reading(reading: FrameReading):
    """Allows manual submission of data (e.g. for testing)"""
    if (reading.lat == 0 or 
        reading.lon == 0 or 
        reading.cond == 0 or 
        reading.temp == 0 or 
        reading.ph == 0 or 
        reading.oxygen == 0):
        return reading
    insert_reading(reading.model_dump())
    return reading
    
@router.get("/all_readings")
def read_all_readings():
    """Returns the history from the SQLite database."""
    data = get_all_readings(limit=5000) # Get last 5000 points
    return {"count": len(data), "readings": data}