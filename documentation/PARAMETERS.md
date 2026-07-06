# EBiCS Parameter Registry
## Dokument wspólny firmware ↔ Canable developer

Wersja: 2026-06-24  
Branch: test/soc-temp

---

## Format tabeli

| Pole | Znaczenie |
|------|-----------|
| **Para1[x]** | Slot w tablicy konfiguracji (bajt 0-63, przekazywany przez CAN) |
| **Skalowanie** | Jak przeliczać wartość bajtu na jednostkę fizyczną |
| **Zakres** | Dopuszczalne wartości bajtu |
| **Domyślna** | Wartość po reset / InitEEPROM |
| **Status** | `AKTYWNY` = działa, `MARTWY` = parsowany ale nieużywany w kodzie |

---

## Para1 — Electric Parameters / Driving Parameters (64 bajtów)

### Już istniejące (Para1, aktywne)

| Para1[x] | Nazwa w MP / config | Skalowanie | Zakres bajtu | Domyślna | Efekt |
|----------|---------------------|-----------|-------------|---------|-------|
| [0] | `system_voltage` | 1 V/bit | 24–72 | 52 | Napięcie nominalne systemu; używane w obliczeniach zasięgu |
| [1] | `battery_current_max` | 1 A/bit | 5–30 | 12 | Limit prądu baterii [A]; góra mocy całkowitej |
| [2] | `max_voltage` | 1 V/bit | 42–84 | 59 | Napięcie maksymalne (ładowanie pełne); ochrona BMS |
| [3,4] | `voltage_min` (2B LE) | CAL_BAT_V=17 → V×17 | — | 1320 (33V) | Napięcie odcięcia; poniżej → ramp-down mocy |
| [7,8] | `battery_capacity_mah` (2B LE) | 1 mAh/bit | 1000–50000 | 14000 | Pojemność baterii; używane do liczenia zasięgu i SOC |
| [9] | `phase_current_max` | A×CAL_I/1000 → bajt | 5–25 | 126 (=12A bat) | Limit prądu fazowego; bezwzględny sufit mocy silnika |
| [10] | `limp_soc_limit` | 1 %SOC/bit, 0xFF=wyłączone | 0–100, 0xFF | 0xFF | SOC próg limp mode (redukcja mocy przy niskim SoC) |
| [11] | `limp_soc_limit_stage2` | 1 %SOC/bit, 0xFF=wyłączone | 0–100, 0xFF | 0xFF | SOC próg limp mode stage 2 (min moc) |
| [12] | `Cadence_exponent` | bezwymiarowy | 0–20 | 10 | Wykładnik kadencji w formule mocy: power ∝ cadence^(1/(1+exp)). 0 = liniowy, 10 = pierwiastkowy, 20 = słabo zależny od kadencji |
| [14] | `legalflag` | 0/1 | 0–1 | 0 | 1 = limit prędkości aktywny (EU 25 km/h); 0 = offroad |
| [18] | `reverse` | 0=-1, 1=+1 | 0–1 | 0 | Kierunek silnika; odwrócić jeśli jedzie wspak |
| [19] | `gear_ratio` | bezwymiarowy | 1–200 | 80 | Przełożenie silnik/koło; używane do obliczania prędkości |
| [20] | `pulses_per_revolution` | impulsy/obrót koła | 1–8 | 1 | Liczba impulsów czujnika prędkości koła na obrót |
| [21] | `decay_base` | bezwymiarowy | 1–255 | 16 | Kształt krzywej wygaszania mocy po zatrzymaniu nacisku. Większa = szybszy decay w oknie 1s. **255 = instant cutoff** |
| [24,25] | `MagicNumber` (2B LE) | — | — | 202 | Identyfikator konfiguracji; NIE zmieniać |
| [34] | `throttle_offset` | (V×33)>>12+1 | — | — | Napięcie ADC manetki przy spoczynku [mV→ADC] |
| [35] | `throttle_max` | (V×33)>>12+1 | — | — | Napięcie ADC manetki przy pełnym gazie |
| [36] | `walk_assist_current` | 1 % / bit (% z phase_current_max) | 10–100 | 50 | Limit prądu w trybie Walk Assist jako % max |
| [37] | `Override_Duration` | ×40 tyki @4kHz | 0–255 | 100 (=4000 tyk=1s) | Czas trwania Extended Boost po ostatnim peak momentu |
| [38] | `PAS_timeout` | ×400 tyki @4kHz (=×0,1s) | 1–10 | 1 (=400 tyk=0,1s) | Czas "hold" przed wejściem w decay gdy siła <760mV. **Zwiększyć do 5 (0,5s) aby zlikwidować pulsowanie w martwym punkcie korby** |
| [39] | `ramp_end` | **MARTWY** — parsowany, nieużywany | — | 1200 | Kiedyś próg kadencji dla ramp-down. Aktualnie nieaktywny w kodzie |
| [41] | `assist_settings[1][0]` | 1 % / bit | 0–100 | 100 | Limit prądu dla poziomu 1 (% z phase_current_max) |
| [43] | `assist_settings[2][0]` | 1 % / bit | 0–100 | 100 | Limit prądu dla poziomu 2 |
| [45] | `assist_settings[3][0]` | 1 % / bit | 0–100 | 100 | Limit prądu dla poziomu 3 |
| [47] | `assist_settings[4][0]` | 1 % / bit | 0–100 | 100 | Limit prądu dla poziomu 4 |
| [48] | `assist_settings[5][0]` | 1 % / bit | 0–100 | 100 | Limit prądu dla poziomu 5 |
| [50] | `assist_settings[1][1]` | 1 % / bit | 0–100 | 100 | Limit prędkości dla poziomu 1 (% z speedLimitx100) |
| [52] | `assist_settings[2][1]` | 1 % / bit | 0–100 | 100 | Limit prędkości dla poziomu 2 |
| [54] | `assist_settings[3][1]` | 1 % / bit | 0–100 | 100 | Limit prędkości dla poziomu 3 |
| [56] | `assist_settings[4][1]` | 1 % / bit | 0–100 | 100 | Limit prędkości dla poziomu 4 |
| [57] | `assist_settings[5][1]` | 1 % / bit | 0–100 | 100 | Limit prędkości dla poziomu 5 |
| [60] | `walk_assist_speed` LSB | 0,01 km/h / bit (2B LE) | — | 600 (=6,0 km/h) | Prędkość docelowa Walk Assist |
| [61] | `walk_assist_speed` MSB | — | — | 0 | Walk Assist speed MSB |

### Para0 — per-poziom (Acceleration / Ride Mode / Torque Threshold)

| Para0[x] | Nazwa | Efekt |
|----------|-------|-------|
| [2,4,6,8] | `assist_settings[1..4][2]` | TQfilter per poziom 1–4 ("Ride Mode" / "Acceleration" w Canable). Po zmianie torque EMA na 3,75°: **zalecane 6** (=667ms τ), 4 = szybszy (167ms τ) |
| [9] | `assist_settings[5][2]` | TQfilter poziom 5 |
| [12..27] | `TQO_threshold[1..5]` (2B LE każdy) | Próg momentu dla Torque Override (mV). Poniżej progu = brak wspomagania niezależnie od siły |

---

## Nowe parametry — PROPONOWANE (wolne sloty)

Parametry aktualnie hardcoded w `config.h`, do dodania w kolejnym kroku.

### Aktualne stale kompilacyjne - rampa TSDZ2-like i start

Te wartosci sa juz uzywane przez firmware, ale **nie sa jeszcze polami Para/CAN**. Zmienia sie je w
`inc/config.h`, potem trzeba przebudowac firmware.

| Define w `config.h` | Wartosc | Znaczenie | Uwagi dla drugiej osoby |
|----------|--------:|-----------|--------------------------|
| `IQ_RAMP_TIME_MODE` | `1` | Wlacza nowa rampe czasowa `i_q` z wewnetrznym licznikiem ulamkowym. | Domyslnie zostawic `1`. `0` wraca do starej rampy krokowej `IQ_SLEW_*`. |
| `IQ_RAMP_Q_SHIFT` | `8` | Liczba bitow ulamekowych dla `iq_setpoint_q`. | Dzieki temu rampa moze miec kroki mniejsze niz 1 jednostka pradu na tyk 4 kHz. |
| `IQ_RAMP_UP_SLOW_TICKS` | `9200` | Wolne narastanie pradu: ok. 2,3 s od 0 do limitu. | Uzywane przy starcie / niskiej predkosci / niskiej kadencji. |
| `IQ_RAMP_UP_FAST_TICKS` | `1200` | Szybkie narastanie pradu: ok. 0,3 s. | Uzywane, gdy predkosc albo kadencja pokazuja, ze rower juz jedzie. |
| `IQ_RAMP_DOWN_SLOW_TICKS` | `4000` | Wolny zanik pradu: ok. 1,0 s. | Miekko wygasza moc przy wolnej jezdzie; nie dotyczy hamulca, wstecz i overtemp. |
| `IQ_RAMP_DOWN_FAST_TICKS` | `560` | Szybki zanik pradu: ok. 0,14 s. | Daje szybkie puszczenie mocy podczas normalnej jazdy. |
| `START_CADENCE_SEED_ENABLE` | `1` | Wlacza tymczasowy maly odczyt kadencji na swiezym starcie. | Nie uruchamia silnika sam; tylko pomaga pierwszemu obliczeniu mocy po przejsciu bramek startu. |
| `START_CADENCE_SEED_STEPS` | `2` | Liczba kolejnych krokow do przodu przed seedem kadencji. | Dalej obowiazuje osobno `START_MIN_STEPS=4` dla faktycznego uzbrojenia wspomagania. |
| `START_CADENCE_SEED_RPM` | `10` | Tymczasowa kadencja publikowana na starcie. | Mala wartosc: ma usunac martwy pierwszy pomiar, a nie dac boost bez kontroli. |

Jesli te ustawienia beda przenoszone do CAN, lepiej wystawic czasy w milisekundach
(`iq_ramp_up_slow_ms`, `iq_ramp_up_fast_ms`, `iq_ramp_down_slow_ms`, `iq_ramp_down_fast_ms`),
a w firmware przeliczac je na ticki 4 kHz. Stare `iq_slew_up/down` sa teraz tylko kompatybilnym
fallbackiem dla `IQ_RAMP_TIME_MODE=0`.

| Para1[x] | Nowa nazwa | Skalowanie | Zakres | Domyślna | Efekt | Status |
|----------|-----------|-----------|--------|---------|-------|--------|
| [22] | `iq_slew_up` | 1 jednostka/bit (@4kHz) | 1–50 | 5 | **Rampa zaangażowania silnika** (0→moc). 5 = ~7,5ms do pełnej mocy, 1 = ~37ms, 20 = ~1,9ms. Zmniejszyć dla łagodniejszego startu |
| [23] | `iq_slew_down` | 1 jednostka/bit (@4kHz) | 1–50 | 10 | **Rampa zwalniania silnika** (moc→0). 10 = ~3,7ms, 1 = ~37ms. Zmniejszyć dla płynniejszego zwalniania |
| [26] | `pas_stop_ticks_x8` | ×8 = tyki @4kHz | 1–255 | 250 (=2000 tyk=500ms) | **Czas detekcji zatrzymania pedałowania**. Krótszy = szybsza odpowiedź na stop, dłuższy = stabilniej przy niskiej kadencji |
| [27] | `tq_recal_band_mv` | 1 mV/bit | 10–200 | 100 | **Pasmo auto-kalibracji zera czujnika momentu** (±mV). Jeśli odczyt spoczynkowy w paśmie 740±band → re-zero natychmiast. Większa = częstsza korekta |
| [28] | `wa_kp` | numerator PI (WA_KP_NUM) | 1–20 | 3 | **Wzmocnienie P regulatora Walk Assist**. Większe = szybsza reakcja, ryzyko oscylacji prędkości |
| [29] | `wa_ki_shift` | WA_KI_SHIFT (shift prawa) | 6–16 | 11 | **Wzmocnienie I regulatora Walk Assist**. Większy shift = słabszy całkujący, wolniejsza eliminacja uchybu stałego |
| [30] | `r_batt_mohm` | 1 mΩ/bit | 10–500 | 80 | **Rezystancja wewnętrzna baterii** [mΩ]. Używana do kompensacji IR w obliczeniach OCV/SOC |

---

## Mapa wolnych slotów Para1 (do dyspozycji)

```
Zajęte: 0,1,2,3,4,7,8,9,10,11,12,14,18,19,20,21,24,25,
         34,35,36,37,38,39,41,43,45,47,48,50,52,54,56,57,60,61
Wolne:  5,6,13,15,16,17,22,23,26,27,28,29,30,31,32,33,
         40,42,44,46,49,51,53,55,58,59,62,63
```

---

## Implementacja w firmware (kolejność kroków)

Dla każdego nowego parametru z tabeli "Proponowane":

1. **`inc/main.h`** — dodać pole do `MotorParams_t`:
   ```c
   uint16_t iq_slew_up;   // mA-units/tick @4kHz
   uint16_t iq_slew_down;
   ```

2. **`src/parser.c`** — `parse_DPparams()`:
   ```c
   MP->iq_slew_up  = Para1[22] ? Para1[22] : 5;   // 0 = niezainicjowany -> fallback
   MP->iq_slew_down = Para1[23] ? Para1[23] : 10;
   ```
   I `parse_MOparams()`:
   ```c
   Para1[22] = MP->iq_slew_up;
   Para1[23] = MP->iq_slew_down;
   ```
   I `InitEEPROM()`:
   ```c
   MP->iq_slew_up  = IQ_SLEW_UP;   // z config.h
   MP->iq_slew_down = IQ_SLEW_DOWN;
   ```

3. **`src/main.c`** — zastąpić stałe polami MP:
   ```c
   // było: (d > IQ_SLEW_UP) ? IQ_SLEW_UP : d
   MS.i_q_setpoint += (d > MP.iq_slew_up) ? MP.iq_slew_up : d;
   ```

4. **Canable** — dodać pole w odpowiedniej zakładce z opisem i zakresem.

---

## Algorytm SOC — zachowanie przy starcie

### Inicjalizacja (`soc_init`)

Kolejność decyzji przy każdym włączeniu:

1. Wczytaj stan z flasha (`soc_state_load`) → `soc_real`, `remaining_mah`, `capacity_est_mah`
2. Oblicz `soc_ocv` z napięcia pakietu (IR-kompensowanego) przez tabelę OCV LG M58T
3. **Przypadek A — wykrycie doładowania**: jeśli `soc_ocv − soc_real > RECHARGE_MARGIN_PCT (5%)` → `soc_real = soc_ocv`, przelicz `remaining_mah`
4. **Przypadek B — niezbite pełne naładowanie** *(dodano 2026-06-24)*: jeśli `soc_ocv ≥ 100%` (napięcie ogniwa powyżej górnego wpisu tabeli = 4,07 V) **i** `soc_real ≥ 80%` → `soc_real = 100%`, `remaining_mah = capacity_est_mah`. Obsługuje doładowanie "top-up" gdzie delta < 5%, np. start z 96% po pełnym ładowaniu
5. Bez flasha (pierwszy rozruch): `soc_real = soc_ocv`, `remaining_mah` z pojemności

### Tabela OCV (`calculate_SOC`)

Zmierzona na LG M58T przy 3 A (rozładowanie). Górna granica tabeli: **4,070 V/ogniwo = 100%**. Napięcia ≥ 4,07 V → klamrowane do 100%. Rzeczywiste napięcie ogniwa po pełnym naładowaniu i balansowaniu wynosi ~4,16–4,17 V, co poprawnie zwraca 100% z klamry.

| Napięcie ogniwa [V] | SOC [%] |
|---------------------|---------|
| 2,799 | 0 |
| 2,968 | 5 |
| 3,086 | 10 |
| 3,247 | 20 |
| 3,450 | 30 |
| 3,569 | 40 |
| 3,681 | 50 |
| 3,774 | 60 |
| 3,853 | 70 |
| 3,946 | 80 |
| 3,989 | 90 |
| 4,070 | 100 |

### Korekcja OCV w spoczynku

Co sekundę w `soc_update()`: jeśli `|I_bat| < I_REST_MA (500 mA)` przez ≥ `REST_TIME_S (30 s)` → `soc_real += OCV_CORR_GAIN (0,02) × (soc_ocv − soc_real)`. Powolne ciągnięcie licznika Coulombów w stronę OCV (ogranicza dryft, nie skacze).

---

## Uwagi dla developera Canable

- Para1 to tablica 64×uint8_t przesyłana w ramce CAN 0x3205 (zapis) i 0x3200 (odczyt)
- Para0 to osobna tablica 64×uint8_t (ramka 0x3201)
- Para2 to tablica dla profili wspomagania (assist profile)
- Wszystkie nowe parametry mieszczą się w Para1 — nie potrzeba nowej ramki CAN
- Wartość 0 w wolnym slocie = "nie ustawiono" → firmware używa domyślnej z `config.h`
- Zakres uint8_t ogranicza do 0–255; dla wartości >255 użyć 2 kolejnych slotów (LE: LSB, MSB)
