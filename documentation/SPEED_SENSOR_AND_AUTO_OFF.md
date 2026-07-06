# Czujnik prędkości, filtr prędkości i auto-wyłączanie HMI

Wersja: 2026-06-24

---

## 1. Błąd czujnika prędkości — Error 21

### Uzasadnienie
Jeśli czujnik prędkości odłączy się podczas jazdy, sterownik cicho zerował Speedx100 po 5 sekundach — HMI nie dostawał żadnej informacji. Rower jedzie dalej na PAS (silnik nie odcięty), ale użytkownik nie wiedział o problemie.

### Implementacja
- `config.h`: `#define ERR_SPEED_SENSOR 21` + `#define SPEED_FAULT_TICKS 20000`
- `main.c`: zmienna `speed_fault`, wykrycie: `Speed_counter > SPEED_FAULT_TICKS && Pedal_Cadence > 0`
- Kasowanie: w `Speed_processing()` przy każdym świeżym impulsie
- Łańcuch błędów (`main.c ~linia 581`): najniższy priorytet po overtemp i torque

### Zachowanie
| Warunek | Działanie |
|---|---|
| Pedałowanie + brak impulsu 5 sek | Error 21 → HMI wyświetla "21" |
| Rower stoi (brak kadencji) | Brak błędu (nie fałszywy alarm) |
| Impuls wraca | Błąd kasuje się natychmiast |
| Napęd PAS | **Nie jest odcinany** — rower jedzie normalnie |
| Walk Assist | **Blokowany** — bez prędkości regulator WA nie może limitować |

### Standardowy numer błędu Bafang
Error 21 = speed sensor signal failure (standard Bafang, weryfikować przez CAN log).

---

## 2. Filtr prędkości — upper bound extrapolation

### Problem z poprzednią implementacją
- `Speed_counter > 20000` → twarda granica: prędkość stoi na ostatniej wartości przez 5 sek, potem skacze do 0
- Walk Assist czeka do 5 sek na aktywację po zatrzymaniu roweru
- Regulator WA PID widzi "zamrożoną" prędkość (niedokładne sterowanie)

### Rozwiązanie
Między impulsami ciągłe obliczanie fizycznej górnej granicy prędkości:

```c
upper_bound = wheel_circumference * 4 * 360 / (pulses_per_revolution * Speed_counter)
```

Koło nie może jechać SZYBCIEJ niż wynikałoby z czasu od ostatniego impulsu. Jeśli upper_bound < aktualnej prędkości → obniż obie wartości (`Speedx100` i `Speedx100_cumulated`).

**Ważne:** Clamp dotyczy też `Speedx100_cumulated` (akumulatora EMA). Gdyby clampować tylko `Speedx100`, to przy ponownym ruszeniu EMA startowałaby od starej wysokiej wartości → fałszywy skok prędkości.

### Zachowanie
| Scenariusz | Przed | Po |
|---|---|---|
| Hamowanie stopniowe | prędkość stoi, potem skok do 0 | płynny fizyczny spadek |
| Nagłe zatrzymanie z 20 km/h | stoi 5 sek | spada do 0 w ~1.5 sek |
| WA engage po zatrzymaniu | czeka 5 sek | aktywuje po ~1 sek |
| Ruszenie 0→20 km/h | OK | OK (upper_bound nie ingeruje w rosnącą prędkość) |
| Brak fałszywych skoków | — | EMA cumulated też clampowany |

### Timeout 20000 ticks
Zostaje jako sieć bezpieczeństwa. Przy normalnym działaniu upper_bound dosięgnie 0 przed upływem 5 sek.

### Liczba magnesów (pulses_per_revolution)
Parametr `Para1[20]` (zakres 1–8, domyślnie 1). Ustawić `2` przez HMI po zamontowaniu 2 magnesów. Cały kod prędkości skaluje się automatycznie — zero zmian w firmware.

---

## 3. Auto-wyłączanie HMI — hipoteza i test

### Kontekst
HMI (wyświetlacz) ma ustawienie "Auto Off" (1–8 minut). Ustawione na 8 minut, ale rower się nie wyłącza.

### Badania
Przeszukano dokumentację Bafang (BESST Para1, OpenSourceEBike, OpenBafangTool, Kaspars CAN blog, Endless Sphere). **Wniosek: nie istnieje żaden parametr CAN wysyłany z HMI do sterownika z czasem auto-shutdown.** Oryginalny Bafang wyłącza się przez HMI odcinający zasilanie po linii zasilającej.

### Hipoteza (pod testem)
Firmware wysyła co 480ms ramkę `0x320F` z bajtem 0 = `0x01` ("sterownik aktywny"). HMI prawdopodobnie interpretuje to jako "system jeździ" i resetuje swój licznik auto-off. Gdy bajt 0 = `0x00` → HMI może zacząć odliczać.

```
CAN ID: 0x02F8320F (source=2 controller, target=31 broadcast)
Bajt 0: 0x01 = aktywny  ← wysyłamy zawsze, stąd brak auto-off
         0x00 = idle     ← hipoteza: pozwala HMI odliczyć
```

### Implementacja testu
- `main.c`: licznik `idle_ticks` (uint16_t, tyki slow loop @40ms)
- Reset gdy: `Speedx100 > 0` LUB `Pedal_Cadence > 0`
- Próg: 750 ticks × 40ms = **30 sekund** bezczynności
- Po 30 sek: `sendCAN_status_broadcast(&MS, is_idle=1)` → bajt 0 = `0x00`
- Przy kalibracji (linia ~1618): zawsze `is_idle=0`

### Oczekiwany wynik testu
Jeśli hipoteza poprawna: HMI powinien wyłączyć się po ~8 minutach bezczynności od momentu gdy rower stoi (bez jazdy i pedałowania przez 30 sek + 8 minut timera HMI).

### Ryzyko
Nieznane — nie wiadomo czy `0x00` w `0x320F` wpływa na coś innego (np. Walk Assist icon, komunikacja). Odwrócić jeśli pojawią się nowe problemy:

```c
// cofnięcie — powrót do zawsze 0x01:
else if(i==1) transmit_message.tx_data[0] = 0x01;
```

### Status
**UNDER TEST** — 2026-06-24. Nie potwierdzone.

---

## 4. Przegląd znanych ramek CAN sterownika → HMI

| CAN EID | Częstotliwość | Opis | Status |
|---|---|---|---|
| `0x02F83201` | ~280ms | Prędkość, prąd, napięcie, temperatura | Aktywny |
| `0x02F83200` | ~1500ms | SOC, dystans, kadencja, torque, zasięg | Aktywny |
| `0x02F83205` | ~320ms | Dane dodatkowe | Aktywny |
| `0x02F83202` | ~120ms | Heartbeat (1 bajt 0x00) | Aktywny |
| `0x02F8320F` | ~480ms | Status sterownika (bajt 0: 0x01=active) | **TEST** |
| `0x02FF1200` | ~480ms | Ostrzeżenia / hamulec | Aktywny |
| `0x02F83000` | ~480ms | Status (bajt 3=0x0B) | Aktywny |
| `0x01F83100` | ~10ms | Torque sensor emulacja (kadencja+moment) | Aktywny |

---

## 5. Obserwacje dotyczące innych zmian (nie wdrożone)

Ze znalezionych notatek użytkownika — następujące zmiany były testowane na innej wersji kodu i działały:

### Throttle improvement (do wdrożenia)
Lokalizacja: po `mapped_throttle = map(...)`, przed `mapped_torque`.
```c
if(MP.legalflag && !MS.offroadflag) {
    if((uint16_cadence_filtered >> 3) > 15) {
        mapped_throttle = 0; // blokada przy pedałowaniu
    } else {
        if(phase_current_max_scaled > 0)
            mapped_throttle = map(mapped_throttle, 0, phase_current_max_scaled, 0, 250);
        else
            mapped_throttle = 0;
        if(MS.Speedx100 >= 600) mapped_throttle = 0;
        else if(MS.Speedx100 >= 400) mapped_throttle = map(MS.Speedx100, 400, 600, mapped_throttle, 0);
    }
}
```
Efekt: manetka ograniczona do ~180W, fade-out 4–6 km/h, blokada przy pedałowaniu w trybie legal.

### Safety fuse (do wdrożenia)
W bloku `legalflag`, po obliczeniu `i_q_setpoint_temp`:
```c
static uint16_t speed_sensor_fault_timer = 0;
if(MS.Speedx100 > 0)          speed_sensor_fault_timer = 0;
else if(MS.i_q_setpoint_temp > 0) { if(speed_sensor_fault_timer < 20000) speed_sensor_fault_timer++; }
if(speed_sensor_fault_timer >= 20000) MS.i_q_setpoint_temp = 0;
```
Efekt: odcięcie silnika w trybie legal po 5 sek bez sygnału prędkości gdy motor generuje moc.
Różni się od Error 21 — ten wyłącza SILNIK (tylko w legal), Error 21 tylko informuje HMI.

### Temperatura zamiast kadencji w display
`CAN_Display.c` ramka `0x3200`, bajt `tx_data[3]`: zmienić `MS->cadence` na `MS->int_Temperature`.
Torque_on_crank w bajtach [4,5] — **już zaimplementowane** w aktualnym kodzie.

### Szybkie wyłączanie przyciskiem
`main.c`: `shutoffcounter > 62` → zmienić na `> 15` (600ms zamiast 2.5 sek).
Uwaga: `> 10` powoduje crash (zbyt krótki czas).
