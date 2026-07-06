# Changelog

## [Unreleased] — branch test/soc-temp

### Nowe funkcje

**Wspomaganie pedalowania — poprawa jakości jazdy**
- Torque EMA przeniesiony na pełną rozdzielczość kwadraturową (co 3,75° / 96×/obrót zamiast co 15°). Algorytmy widzą teraz profil siły przez cały obrót, nie tylko co 15°. Eliminuje pulsowanie on/off przy małej sile i zbyt szybkie zanikanie mocy przy zmniejszaniu nacisku.
- Cadence-gate: `torque_counter` resetuje się przy każdym kroku kwadraturowym do przodu, jeśli `torque_filtered > 0`. Silnik nie wchodzi w decay podczas martwego punktu korby dopóki korba się kręci i był jakikolwiek moment.
- `TQFILTER` domyślnie zmieniony z 4 na 6 — zachowuje tę samą stałą czasową filtra (~667 ms @ 60 RPM) przy nowej częstości aktualizacji.
- Slew limiters na prądzie `i_q` (IQ_SLEW_UP=5, IQ_SLEW_DOWN=10 mA/tyk): łagodne zaangażowanie/wyłączenie silnika, ochrona napędu przed szarpnięciem.
- Rampa `i_q` podobna do TSDZ2: nowy `IQ_RAMP_TIME_MODE=1` używa ułamkowego akumulatora, więc czasy są przewidywalne przy pętli 4 kHz: narastanie ok. 2,3 s / 0,3 s, opadanie ok. 1,0 s / 0,14 s zależnie od prędkości i kadencji. Hamulec, kręcenie wstecz i odcięcie termiczne nadal działają natychmiast.
- Start kadencji podobny do TSDZ2: `START_CADENCE_SEED_ENABLE=1` publikuje tymczasowo 10 rpm po 2 poprawnych krokach do przodu i realnym nacisku. Nie uruchamia silnika samodzielnie; `START_MIN_STEPS=4` i `TQ_GATE_MIN=25` nadal pilnują, żeby nie było wzbudzenia od ruchu przód-tył bez nacisku.

**Walk Assist — zamkniętopętlowy regulator PI prędkości**
- Zastąpiono prosty mapowanie prędkości → prąd regulatorem PI utrzymującym `walk_assist_speed`.
- Rozróżnienie kick (start z miejsca) vs resume (rozruch w ruchu): kick tylko gdy `Speedx100 < WA_KICK_SPEED`.
- Anti-windup integratora, kickstart slew 180 ms, zabezpieczenie przed przekroczeniem 6 km/h.
- `walk_assist_current` domyślnie 50% (poprzednio 30%).
- Poprawka: limit prędkości nie obcinał mocy Walk Assist (`!MS.pushassist_flag`).

**Czujnik momentu — detekcja usterek i auto-kalibracja (Error 25)**
- Detekcja sygnału poza zakresem (300–4300 mV, debounce 100 ms) → Error 25.
- Cykliczna re-kalibracja zera podczas wybiegu (korba ≥1,5 s w spoczynku): korekta offsetu max 20 mV/wybieg, pasmo ±100 mV, wymóg 3 zgodnych pomiarów dla dużych dryfów.
- Sanity-check startowy: jeśli surowy odczyt spoczynkowy poza oknem 300–1500 mV → offset zignorowany, Error 25 do pierwszego poprawnego wybiegu.

**CAN — pełna emulacja ramek oryginalnego firmware M820**
- `sendCAN_3100()`: emulacja węzła czujnika momentu (source=1), frame 10 ms z cadence i `torque_on_crank` — wymagany przez HMI do wyświetlania kadencji i momentu.
- `sendCAN_3202()`: ramka keepalive Walk Assist co ~120 ms — bez niej HMI wyłącza tryb WA po kilku sekundach.
- `sendCAN_status_broadcast()`: heartbeat 0x1200 (hamulec), 0x320F (status), 0x3000 — wymagany przez HMI do migania ikony WA.
- Refaktor timingu CAN: osobne liczniki per ramka zamiast round-robin `pollnumber`. Timings: heartbeat 480 ms, prędkość 280 ms, kadencja 1480 ms, misc 320 ms.
- `sendAcknoledge()`: odpowiedź do source ramki żądania zamiast hardcoded BESST (target=5).

**SOC / zasięg**
- Nauka zużycia Wh/km osobno dla każdego poziomu wspomagania (`wh_km_level[10]`). Zmiana poziomu nie zeruje nauki.
- Poprawka: po doładowaniu "top-up" (np. z 96% do 100%) SOC nie wracał do 100% — warunek `soc_ocv - soc_real > RECHARGE_MARGIN_PCT (5%)` nie był spełniony, licznik Coulombów startował od zapisanego ~96%, a korekcja OCV (gain=0,02/s) dobijała do ~99% dopiero po kilku minutach. Dodano osobny przypadek: jeśli napięcie ogniwa przekracza górną granicę tabeli OCV (4,07 V/ogniwo = niezbite 100%) i zapisany SOC ≥ 80%, licznik Coulombów jest natychmiast resetowany do 100% / pełnej pojemności.

**Inne**
- Wersja buildu: `EBICS_BUILD_VERSION` wstrzykiwana przez `build_firmware.ps1` do `inc/build_version.h` (gitignored) i wysyłana w polu info HMI.
- Hall autodetect: watchdog feed + keepalive CAN podczas kalibracji (procedura blokuje main loop >5 s).
- `PH_CURRENT_MAX` powiązany z `BATTERYCURRENT_MAX / CAL_I`.
- Usunięto: `statehistory[]`, `Poll_commands[]`, `pollnumber`, `soc_have_real_consumption`.

### Strojenie (Canable)

| Parametr | Pole Canable | Domyślna wartość | Uwaga |
|---|---|---|---|
| TQfilter | Ride Mode (per poziom) | 6 | Obniżyć do 4–5 dla szybszej reakcji |
| PAS timeout | Current Loading Time | 1 (=400 tyk) | Zwiększyć do 5 (=2000 tyk) przy pulsowaniu |
| Walk Assist prąd | Walk Assist Current | 50% | |
| Walk Assist prędkość | Walk Assist Speed | 6,0 km/h | |

---

## Poprzednie commity (gałąź test/soc-temp)

- `aad8365` feat(pas): quadrature decoder (PC12+PD2) for cadence & direction
- `20dfb2f` revert(assist): restore origin/M820 pedal-assist + torque logic for clean re-analysis
- `447cb26` fix(range): send remaining range in 0.01 km units for 0x3200
- `bf8d5f5` feat(assist): real-time torque filtering + disable latched min-assist
- `544aab6` feat(walk-assist): ramp power up over ~700ms on engage
- `4ff88a9` fix(temp): set TEMP_OFFSET_C=11 to match original firmware
