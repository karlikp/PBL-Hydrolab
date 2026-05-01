import serial
import time
import csv
import threading
from datetime import datetime

# --- KONFIGURACJA ---
PORT = 'COM7' 
BAUD_RATE = 57600
TIMEOUT_SERIAL = 0.5 
PLIK_ELMETRON = 'baza_elmetron.csv'
PLIK_ZBIORNIKI = 'baza_zdarzenia.csv'
# --------------------

class StacjaBazowa:
    def __init__(self, port, baud_rate, timeout):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.dziala = False
        
        self.watek_nasluchu = None
        self.plik_lock = threading.Lock()

        self.stan_zbiornikow = {
            'C1': 'PUSTY',
            'C2': 'PUSTY',
            'C3': 'PUSTY'
        }
        
        self.gps_lat = "Brak_Danych"
        self.gps_lon = "Brak_Danych"

    def polacz(self):
        """Otwiera port szeregowy i uruchamia nasłuch w tle."""
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            print(f"[SYSTEM] Pomyślnie połączono z modułem RFD na porcie {self.port}")
            self.dziala = True
            
            self.watek_nasluchu = threading.Thread(target=self._nasluchuj, daemon=True)
            self.watek_nasluchu.start()
            return True
            
        except serial.SerialException as e:
            print(f"[BŁĄD KRYTYCZNY] Nie można otworzyć portu {self.port}.")
            print(f"Szczegóły: {e}")
            return False

    def rozlacz(self):
        """Bezpiecznie wyłącza wątki i port COM."""
        self.dziala = False
        if self.watek_nasluchu and self.watek_nasluchu.is_alive():
            self.watek_nasluchu.join(timeout=1.0)
            
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[SYSTEM] Port szeregowy został bezpiecznie zamknięty.")

    def _nasluchuj(self):
        """Pętla działająca w tle, dedykowana wyłącznie do odbierania telemetrii."""
        while self.dziala:
            try:
                if self.ser.in_waiting > 0:
                    odpowiedz = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if odpowiedz:
                        self._przetworz_odpowiedz(odpowiedz)
            except Exception as e:
                print(f"[BŁĄD NASŁUCHU] {e}")
            time.sleep(0.01) 

    def _przetworz_odpowiedz(self, odpowiedz):
        """Zarządza odebraną ramką i kieruje dane do pliku CSV."""
        
        if odpowiedz.startswith("GPS:"):
            try:
                dane_gps = odpowiedz.split(":")[1].split(",")
                self.gps_lat = dane_gps[0].strip()
                self.gps_lon = dane_gps[1].strip()
                # Opcjonalnie możesz odkomentować poniższą linię, jeśli chcesz widzieć ciągłe aktualizacje GPS na żywo
                # print(f"[GPS] Pozycja zaktualizowana: {self.gps_lat}, {self.gps_lon}")
            except Exception:
                pass # Ignorujemy błędnie uciętą ramkę GPS i czekamy na kolejną
            return # Kończymy przetwarzanie tej konkretnej linijki (nie zapisujemy samego GPS jako pomiaru)

        # Wyświetlamy w terminalu tylko istotne dane z czujników
        print(f"\n[DRON_RX] Otrzymano: {odpowiedz}")
        
        # Obsługa nowej ramki zbiorczej z Elmetronu (wiele zmiennych oddzielonych '|')
        if "|" in odpowiedz:
                    czas = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    pomiary = odpowiedz.split("|")
                    
                    with self.plik_lock:
                        try:
                            with open(PLIK_ELMETRON, mode='a', newline='', encoding='utf-8') as file:
                                writer = csv.writer(file)
                                for pomiar in pomiary:
                                    if ":" in pomiar:
                                        sensor, wartosc = pomiar.split(":", 1)
                                        writer.writerow([czas, sensor.strip(), wartosc.strip(), self.gps_lat, self.gps_lon])
                                        print(f"[ZAPIS ELMETRON] {czas} -> {sensor.strip()} = {wartosc.strip()}")
                        except Exception as e:
                            print(f"[BŁĄD ZAPISU] Nie udało się otworzyć bazy: {e}")

        # ZBIORNIKI C (Maszyna stanów)
        elif odpowiedz.startswith("STATUS_"):
            czesc_statusu = odpowiedz.split(":")
            if len(czesc_statusu) == 2:
                akcja = czesc_statusu[0] 
                krok = czesc_statusu[1]  
                zbiornik = akcja.split("_")[1] 

                if krok == "0":
                    print(f"\n[MASZYNA_STANÓW] {zbiornik} -> Winda opuszcza moduł poboru.")
                elif krok == "1":
                    print(f"\n[MASZYNA_STANÓW] {zbiornik} -> Moduł zanurzony, trwa napełnianie pojemnika.")
                elif krok == "2":
                    print(f"\n[MASZYNA_STANÓW] {zbiornik} -> Pojemnik pełny, winda wciąga ładunek.")

        elif odpowiedz.startswith("SUCCESS_"):
                    zbiornik = odpowiedz.split("_")[1]
                    self.stan_zbiornikow[zbiornik] = 'PELNY'
                    print(f"\n[SUKCES] Procedura poboru do {zbiornik} zakończona powodzeniem. Zbiornik zablokowany.")
                    
                    czas = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    with self.plik_lock:
                        try:
                            with open(PLIK_ZBIORNIKI, mode='a', newline='') as file:
                                csv.writer(file).writerow([czas, f"Pobor_{zbiornik}", "SUKCES", self.gps_lat, self.gps_lon])
                        except Exception:
                            pass

        elif odpowiedz.startswith("ERROR_"):
            czesc_bledu = odpowiedz.split(":")
            zbiornik = czesc_bledu[0].split("_")[1]
            powod = czesc_bledu[1] if len(czesc_bledu) > 1 else "Nieznany błąd"
            self.stan_zbiornikow[zbiornik] = 'AWARIA'
            print(f"\n[AWARIA KRYTYCZNA] Problem z podzespołem {zbiornik}. Powód: {powod}")
            czas = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with self.plik_lock:
                try:
                     with open(PLIK_ZBIORNIKI, mode='a', newline='') as file:
                            csv.writer(file).writerow([czas, f"Pobor_{zbiornik}", "ERROR", powod, self.gps_lat, self.gps_lon])
                except Exception:
                    pass

        # Obsługa pojedynczego pomiaru (w sensie że jeden element robi pomiar, np cały elmetron)
        elif ":" in odpowiedz:
            czas = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            try:
                sensor, wartosc = odpowiedz.split(":", 1)
                sensor = sensor.strip()
                wartosc = wartosc.strip()
                
                with self.plik_lock:
                    with open(PLIK_ELMETRON, mode='a', newline='', encoding='utf-8') as file:
                        # --- ZMIANA: Zapis 5 elementów do bazy (dochodzi GPS) ---
                        csv.writer(file).writerow([czas, sensor, wartosc, self.gps_lat, self.gps_lon])
                        print(f"[ZAPIS CSV] {czas} -> {sensor} = {wartosc} (GPS: {self.gps_lat}, {self.gps_lon})")
            except Exception as e:
                print(f"[BŁĄD ZAPISU] Problem z pojedynczą zmienną: {e}")

        elif "ACK" in odpowiedz:
            print("[STATUS] Dron pomyślnie potwierdził komendę (ACK).")
        elif "ERR" in odpowiedz:
            print(f"[BŁĄD NA DRONIE] System zgłasza awarię: {odpowiedz}")
        else:
            print(f"[INFO] Nierozpoznany log z drona: {odpowiedz}")

    def wyslij_komende(self, komenda):
        """Wysyła ramkę tekstową do drona przez moduł RFD."""
        if self.ser and self.ser.is_open:
            if not komenda.endswith('\n'):
                komenda += '\n'
            self.ser.write(komenda.encode('utf-8'))
            print(f"[STACJA_TX] Wysłano rozkaz: {komenda.strip()}")
        else:
            print("[BŁĄD] Brak aktywnego połączenia radiowego.")

    def pobierz_wode(self, numer_zbiornika):
        """Uruchamia sekwencję poboru z weryfikacją stanu i blokadą globalną maszyny."""
        zbiornik = numer_zbiornika.upper()
        
        if zbiornik not in self.stan_zbiornikow:
            print(f"[BŁĄD OPERATORA] Nieznany identyfikator zbiornika: {zbiornik}.")
            return

        pracujace_zbiorniki = [nazwa for nazwa, stan in self.stan_zbiornikow.items() if stan == 'W_TRAKCIE']
        
        if pracujace_zbiorniki:
            aktywny_zbiornik = pracujace_zbiorniki[0]
            if aktywny_zbiornik == zbiornik:
                print(f"[ZABEZPIECZENIE] Zbiornik {zbiornik} jest już w trakcie pracy.")
            else:
                print(f"[ZABEZPIECZENIE KRYTYCZNE] Maszyna poboru zajęta przez zbiornik {aktywny_zbiornik}.")
            return 

        aktualny_stan = self.stan_zbiornikow[zbiornik]

        if aktualny_stan == 'PELNY':
            print(f"[ZABEZPIECZENIE] Odmowa akcji. Zbiornik {zbiornik} jest już PEŁNY!")
        elif aktualny_stan == 'AWARIA':
            print(f"[UWAGA] Zbiornik {zbiornik} zgłosił AWARIĘ. Próba zignorowana.")
        elif aktualny_stan == 'PUSTY':
            print(f"[SYSTEM] Rozpoczynam procedurę poboru dla {zbiornik}...")
            self.stan_zbiornikow[zbiornik] = 'W_TRAKCIE'
            self.wyslij_komende(f"START_{zbiornik}")

    def zatrzymanie_awaryjne(self):
        """Wysyła bezwzględny rozkaz zatrzymania mechanizmów na dronie i resetuje stany stacji."""
        print("\n[!!! ZATRZYMANIE AWARYJNE INICJOWANE !!!]")
        self.wyslij_komende("E_STOP")
        
        zatrzymano_cos = False
        for zbiornik, stan in self.stan_zbiornikow.items():
            if stan == 'W_TRAKCIE':
                self.stan_zbiornikow[zbiornik] = 'AWARIA'
                zatrzymano_cos = True
                print(f" -> Przerwano operację dla zbiornika {zbiornik}. Zmieniono status na AWARIA.")
        
        if not zatrzymano_cos:
            print(" -> Maszyna nie wykonywała aktualnie żadnej akcji, sygnał STOP wysłany prewencyjnie.")
            
        print("[!!! SYSTEM ZATRZYMANY. ZALECA SIĘ INSPEKCJĘ DRONA !!!]\n")


# =====================================================================
# GŁÓWNY INTERFEJS OPERATORA
# =====================================================================
if __name__ == "__main__":
    stacja = StacjaBazowa(PORT, BAUD_RATE, TIMEOUT_SERIAL)
    
# Nagłówki dla Elmetronu
    try:
        with open(PLIK_ELMETRON, 'x', newline='') as f:
            csv.writer(f).writerow(["Czas_pomiaru", "Typ_sensora", "Odczyt", "Szerokosc_Geo", "Dlugosc_Geo"])
    except FileExistsError:
        pass

    # Nagłówki dla Zbiorników
    try:
        with open(PLIK_ZBIORNIKI, 'x', newline='') as f:
            csv.writer(f).writerow(["Czas_zdarzenia", "Zdarzenie", "Szczegoly", "Szerokosc_Geo", "Dlugosc_Geo"])
    except FileExistsError:
        pass

    if stacja.polacz():
        print("\n==================================================")
        print(" INTERFEJS STEROWANIA STACJĄ BAZOWĄ ZAINICJOWANY")
        print(" [E]  - Pobierz pomiary z Elmetronu (T, pH, Cond, O2)")
        print(" [G]  - Sprawdź aktualną pozycję GPS drona")
        print(" [C1] - Rozpocznij pobór wody dla zbiornika 1")
        print(" [C2] - Rozpocznij pobór wody dla zbiornika 2")
        print(" [C3] - Rozpocznij pobór wody dla zbiornika 3")
        print(" [S]  - Sprawdź status wszystkich zbiorników")
        print(" [X]  - !!! ZATRZYMANIE AWARYJNE (E-STOP) !!!")
        print(" [EXIT] - Zamknij program")
        print("==================================================")
        
        try:
            while True:
                komenda = input("\n[Terminal] >> ").strip().upper()
                
                if not komenda:
                    continue
                
                if komenda == 'E' or komenda == 'GET_ELMETRON_ALL':
                    print("[SYSTEM] Żądam danych z Elmetronu...")
                    stacja.wyslij_komende("GET_ELMETRON_ALL")
                    
                elif komenda == 'C1':
                    stacja.pobierz_wode('C1')
                    
                elif komenda == 'C2':
                    stacja.pobierz_wode('C2')
                    
                elif komenda == 'C3':
                    stacja.pobierz_wode('C3')
                    
                elif komenda == 'S' or komenda == 'STATUS':
                    print("\n--- RAPORT ZBIORNIKÓW ---")
                    for zb, stan in stacja.stan_zbiornikow.items():
                        print(f" Zbiornik {zb}: {stan}")
                    print("-------------------------")
                    
                elif komenda == 'X' or komenda == 'STOP':
                    stacja.zatrzymanie_awaryjne()
                    
                elif komenda == 'EXIT':
                    print("Rozpoczynam procedurę wyłączania...")
                    break

                elif komenda == 'G' or komenda == 'GPS':
                    print("\n--- LOKALIZATOR DRONA ---")
                    if stacja.gps_lat == "Brak_Danych":
                        print(" [UWAGA] Brak sygnału GPS. Dron nie połączył się z satelitami.")
                    else:
                        print(f" Szerokość: {stacja.gps_lat}")
                        print(f" Długość:   {stacja.gps_lon}")
                        print(f" Link Google Maps: https://www.google.com/maps/place/{stacja.gps_lat},{stacja.gps_lon}")
                    print("-------------------------")
                    
                else:
                    print(f"[BŁĄD] Nieznana komenda: {komenda}. Wybierz E, C1, C2, C3, S, X lub EXIT.")
                
                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n[ZATRZYMANIE AWARYJNE] Użyto Ctrl+C. Wymuszam awaryjny STOP.")
            stacja.zatrzymanie_awaryjne()
            
        finally:
            stacja.rozlacz()
            print("Program zakończył działanie.")