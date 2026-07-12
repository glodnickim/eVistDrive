# Changelog

## [Unreleased] — branch test/soc-temp

### Nowe funkcje

**Walk Assist — strojenie po teście 0.0135: start ×2, utrzymanie ÷2**
- Objaw z jazdy: pierwsza chwila za słaba, potem po ~1 s rower „gna" aż do odcięcia anty-przelotowego.
- **Start = bezwzględny % prądu fazowego** (`WA_START_PCT`=100): sufit przy 0 km/h to teraz pełny prąd fazowy (było min(200%·wa_max, 60% fazowego) = 60%; żądane ×2=120% obcięte fizycznie do 100%). Odwiązany od Walk Current z Canable/HMI — ta sama filozofia co niezależny od poziomów boost pedałowy. Wygasa liniowo do sufitu utrzymania przy 3 km/h (`WA_START_FULL_SPEED` bez zmian).
- **Utrzymanie = połowa ustawionego Walk Current** (`WA_HOLD_PCT`=50): sufit PI, sufit integratora i wygaszanie przy celu liczone z `wa_hold = fazowy·walk_assist_current·50%/100` — przy zapisanych 30% realnie 15% fazowego. Skalowanie w firmware, więc działa niezależnie od tego, co HMI/Canable ma zapisane.
- Usunięte `WA_START_BOOST_PCT` i `WA_BOOST_CEIL_PCT` (zastąpione przez `WA_START_PCT`); kick slew 180 ms i szybka rampa WA bez zmian.

**Walk Assist — strojenie po jeździe testowej 0.0132 (4 poprawki)**
- **Zwłoka załączania usunięta:** w trybie WA zewnętrzna rampa `i_q` przełącza się na szybkie tempo (0,3 s w górę / 0,14 s w dół zamiast 2,3 s / 1,0 s na postoju). WA ma własny kick 180 ms, więc podwójna rampa tylko opóźniała start; szybka rampa w dół dodatkowo przyspiesza cięcie anty-przelotowe z ~0,9 s do ~0,14 s.
- **Start boost — ruszenie 2× mocniejsze:** sufit prądu przy ruszaniu podniesiony do `WA_START_BOOST_PCT`=200% `wa_max` przy 0 km/h, wygasa liniowo do 100% przy `WA_START_FULL_SPEED`=3 km/h; twardy limit `WA_BOOST_CEIL_PCT`=60% prądu fazowego. Poniżej 3 km/h wymuszona podłoga = pełny boost (gwarantowane dopchnięcie), narastająca przez kick 180 ms. Integrator PI dalej clampowany do `wa_max` — boost nie nawija się w całkę.
- **Anty-przelot 6 km/h:** `WA_FADE_BAND` 150→250 (wygaszanie mocy od 2,5 km/h przed celem), `WA_NEAR_HOLD_PCT` 25→15 (przy celu 15% `wa_max`). Razem ze słabszym utrzymaniem i szybką rampą w dół eliminuje przelatywanie zadanej prędkości.
- **`walk_assist_current` domyślnie 30%** (było 50%): boost 2× = dokładnie 60% fazowego; słabsze utrzymanie dodatkowo ogranicza przelot. Jeśli HMI ma zapisane 50, ustawić 30 w Canable (Para1[36]).

**Pedał-assist — niższy próg startu**
- `TQ_GATE_MIN` 25→18 mV: delikatnie lżejszy nacisk uruchamia wspomaganie (próg wspólny dla wszystkich poziomów). Odczuwalna różnica poziomu E wynika ze skalowania mocy (profil E w Canable), nie z progu — jeśli E dalej za słabe przy starcie, podnieść pierwszy segment profilu E w Canable.

**CAN — przywrócony odczyt Para0 dla Canable (zakładka Full Assist)**
- Naprawa pustego ekranu Info w HMI podmieniła odpowiedź READ 0x6010 na fabryczny 4-bajtowy mini-blok `01 00 02 06` dla WSZYSTKICH pytających — Canable przestał dostawać Para0 i zakładka Full Assist była nieaktywna (Assist Light działała, bo czyta Para1/0x6011).
- Teraz odpowiedź zależy od nadawcy żądania: **source=5 (BESST/Canable) → pełny multiframe Para0**, source=3 (HMI) i inne → mini-blok jak fabryka. Handshake ekranu Info w HMI bez zmian; zapisy Para0 z Canable działały cały czas.

**Pedał-assist — boost startowy w stylu TSDZ2 (`STARTUP_BOOST`); zastępuje `STARTUP_FLOOR`**
- Port `apply_startup_boost()` z OSF TSDZ2: **mnożnik nacisku** malejący geometrycznie z kadencją — `factor(kadencja)% = STARTUP_BOOST_FACTOR × (1 − CADENCE_STEP/256)^kadencja`. Przy kadencji 0 nacisk wzmocniony o `STARTUP_BOOST_FACTOR`=200%, boost sam wygasa w miarę rozpędzania korby (`STARTUP_BOOST_CADENCE_STEP`=50 steruje tempem zaniku). Kick proporcjonalny do siły nacisku: mocno depczesz → mocny, ale kontrolowany start.
- Trzy tryby aktywacji jak w TSDZ2 (`STARTUP_BOOST_MODE`): 0=CADENCE (zawsze, gaśnie z kadencją), 1=SPEED (tylko od postoju, wyłączany >45 rpm), 2=AUTO (wyłączany przy małym nacisku w ruchu, próg `STARTUP_BOOST_AUTO_TQ`).
- Wzmocniony nacisk zamykany do pełnego `MP.phase_current_max` (kick niezależny od poziomu). Działa na `mapped_torque`, więc przechodzi przez te same bramki co reszta (latch: nacisk + 4 kroki do przodu) — bez wzbudzania na zjeździe.
- **`STARTUP_FLOOR` usunięty w całości** (kod + parametry): jeden mechanizm boostu zamiast dwóch nakładających się — feedback z jazdy jednoznacznie wskazuje, co stroić.
- **Fix interakcji z cadence seed:** seed publikuje sztuczne 10 rpm zanim istnieje pomiar kadencji, przez co boost liczył współczynnik przy kadencji 10 zamiast 0 i kasował sam siebie w chwili ruszenia (przy step=50 z 200% zostawało ~23%). Nowa flaga `cadence_seeded` (ustawiana przy seedzie, kasowana pierwszym realnym pomiarem lub stopem) — boost traktuje seedowaną kadencję jako 0. Dodatkowo `STARTUP_BOOST_CADENCE_STEP` 50→25 (typowe TSDZ): ~36% boostu przy 10 rpm, wygasa ~40 rpm.
- **Rampa i_q na postoju skrócona pod boost:** `IQ_RAMP_UP_SLOW_TICKS` 9200→2400 (2,3 s → 0,6 s) — wolna rampa rozsmarowywała kopniak boostu na 2 sekundy; 0,6 s pozwala go poczuć, nadal chroniąc napęd. Jeśli start za ostry: wrócić do 9200.

**Wspomaganie pedalowania — poprawa jakości jazdy**
- Torque EMA przeniesiony na pełną rozdzielczość kwadraturową (co 3,75° / 96×/obrót zamiast co 15°). Algorytmy widzą teraz profil siły przez cały obrót, nie tylko co 15°. Eliminuje pulsowanie on/off przy małej sile i zbyt szybkie zanikanie mocy przy zmniejszaniu nacisku.
- Cadence-gate: `torque_counter` resetuje się przy każdym kroku kwadraturowym do przodu, jeśli `torque_filtered > 0`. Silnik nie wchodzi w decay podczas martwego punktu korby dopóki korba się kręci i był jakikolwiek moment.
- `TQFILTER` domyślnie zmieniony z 4 na 6 — zachowuje tę samą stałą czasową filtra (~667 ms @ 60 RPM) przy nowej częstości aktualizacji.
- Slew limiters na prądzie `i_q` (IQ_SLEW_UP=5, IQ_SLEW_DOWN=10 mA/tyk): łagodne zaangażowanie/wyłączenie silnika, ochrona napędu przed szarpnięciem.
- Rampa `i_q` podobna do TSDZ2: nowy `IQ_RAMP_TIME_MODE=1` używa ułamkowego akumulatora, więc czasy są przewidywalne przy pętli 4 kHz: narastanie ok. 2,3 s / 0,3 s, opadanie ok. 1,0 s / 0,14 s zależnie od prędkości i kadencji. Hamulec, kręcenie wstecz i odcięcie termiczne nadal działają natychmiast.
- Miękkie odcięcie stopnia mocy (`SOFT_CUTOFF_ENABLE=1`): usuwa „klik" na samym końcu wspomagania. Zamiast skokowego `timer_primary_output_config(DISABLE)` po zatrzymaniu wirnika, napięcia faz zjeżdżają liniowo do wektora neutralnego (`_T/2`) przez `SOFT_CUTOFF_TICKS`=40 (≈10 ms), dopiero potem mostek jest odcinany. Taktowanie w `reg_ADC_processing` (4 kHz). Ścieżki awaryjne (hamulec/wstecz/przegrzanie) nadal tną natychmiast; start bez zmian.
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
| Walk Assist prąd | Walk Assist Current | 30% | Boost startowy = 2× tej wartości (max 60% fazowego) |
| Walk Assist prędkość | Walk Assist Speed | 6,0 km/h | |

---

## Poprzednie commity (gałąź test/soc-temp)

- `aad8365` feat(pas): quadrature decoder (PC12+PD2) for cadence & direction
- `20dfb2f` revert(assist): restore origin/M820 pedal-assist + torque logic for clean re-analysis
- `447cb26` fix(range): send remaining range in 0.01 km units for 0x3200
- `bf8d5f5` feat(assist): real-time torque filtering + disable latched min-assist
- `544aab6` feat(walk-assist): ramp power up over ~700ms on engage
- `4ff88a9` fix(temp): set TEMP_OFFSET_C=11 to match original firmware
