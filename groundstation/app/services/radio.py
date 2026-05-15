import serial
import time
import threading
from datetime import datetime
from app.database import insert_reading

class RadioService:
    def __init__(self, port="/dev/ttyUSB1", baud=57600):
        self.port = port
        self.baud = baud
        self.running = False
        self.thread = None

    def start(self):
        """Starts the background listener thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
            print(f"[Radio] Service started on {self.port}")

    def stop(self):
        """Stops the background listener."""
        self.running = False
        if self.thread:
            self.thread.join()
            print("[Radio] Service stopped.")

    def _calculate_adler32(self, data_string):
        """
        Calculates Adler-32 checksum matching the C++ implementation.
        """
        data_bytes = data_string.encode('utf-8') 
        
        a = 1
        b = 0
        MOD_ADLER = 65521
        
        for byte in data_bytes:
            a = (a + byte) % MOD_ADLER
            b = (b + a) % MOD_ADLER
            
        return (b << 16) | a

    def _listen_loop(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2) # Allow connection to settle
        except serial.SerialException as e:
            print(f"[Radio Error] Could not open port {self.port}: {e}")
            return

        print("[Radio] Listening for packets...")

        while self.running:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not line or "*" not in line:
                        continue

                    # 1. Parse Data*Checksum
                    # We expect the last part to be an 8-char HEX string
                    try:
                        data_part, received_cs_hex = line.rsplit("*", 1)
                        # Parse hex to integer
                        received_cs = int(received_cs_hex, 16)
                    except ValueError:
                        print(f"[Radio] Bad Checksum Format: {line}")
                        continue

                    # 2. Verify Checksum (Adler-32)
                    calculated_cs = self._calculate_adler32(data_part)

                    if calculated_cs == received_cs:
                        self._process_data(data_part, received_cs)
                    else:
                        print(f"[Radio] Checksum Mismatch! Calc: {calculated_cs:X} vs Recv: {received_cs:X}")
                        
            except Exception as e:
                print(f"[Radio Loop Error] {e}")
                time.sleep(1)

        ser.close()

    def _process_data(self, data_str, checksum):
        # CSV: Time,Lat,Lon,Cond,Temp,pH,O2,WaterFlag
        parts = data_str.split(',')
        
        if len(parts) >= 8:
            reading = {
                "time": datetime.now().isoformat(),
                "lat": float(parts[1]) / 10000000.0,
                "lon": float(parts[2]) / 10000000.0,
                "cond": float(parts[3]),
                "temp": float(parts[4]),
                "ph": float(parts[5]),
                "oxygen": float(parts[6]),
                "water_flag": bool(int(parts[7])),
                "checksum": checksum
            }
            
            # SAVE TO DB (Persistent Storage)
            insert_reading(reading) 
            
            print(f"[Radio] Saved to DB: pH {reading['ph']} | O2 {reading['oxygen']}")