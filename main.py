from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware 
from fastapi.staticfiles import StaticFiles
from app.api.endpoints import router as data_router
from app.services.radio import RadioService
from app.database import init_db

# --- CONFIGURATION ---
RADIO_PORT = "/dev/ttyUSB1"
radio_service = RadioService(port=RADIO_PORT)

# --- LIFESPAN MANAGER ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    # 1. Startup: Launch Radio Thread
    init_db()
    radio_service.start()
    yield
    # 2. Shutdown: Stop Radio Thread
    radio_service.stop()

# --- APP DEFINITION ---
app = FastAPI(
    title="Water Quality Drone API", 
    lifespan=lifespan
)

origins = ["*"] 

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,             
    allow_credentials=True,            
    allow_methods=["*"],               
    allow_headers=["*"],               
)

app.mount("/static", StaticFiles(directory="static", html=True), name="static")

app.include_router(data_router)

@app.get("/health", summary="Sprawdzenie statusu serwera")
def read_root():
    return {"status": "ok", "radio_running": radio_service.running}