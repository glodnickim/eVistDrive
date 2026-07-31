# Karta zmiany FW-030 — jeden tryb ride core (usunięcie wyboru Legacy) + prąd fazowy 700

- **Data:** 2026-07-26
- **Status:** WDROŻONE W DRZEWIE, build `0.0201`. Czeka na test sprzętowy.
- **Zakres:** firmware (wybór silnika, manetka, prąd fazowy) + Canable (UI trybu).
- **Powiązane:** [[project-ride-core-runaway-fixed]] (FW-028, `0.0199`), FW-029 (prąd
  fazowy — zrealizowany tutaj jako stałe 700).

---

## 1. Decyzja

ride core zostaje JEDYNYM trybem. Legacy wycofany jako wybór. Walk Assist i kalibracja
Halla ZACHOWANE (używały monolitu Legacy niezależnie od wyboru silnika). Manetka
przeniesiona do ride core. Prąd fazowy ustawiony na 700 (wartość dewelopera).

## 2. Firmware — co zrobiono (Krok A, bezpieczny)

- **Wybór silnika usunięty** (`ride_control.c`): ride core jest jedyną ścieżką pedałową.
  `ride_control_set_engine`, `ride_control_update_request`, `active_engine` usunięte.
  `ride_control_get_engine()` = zaślepka zwracająca ride core (telemetria FW-028/0x6029
  jej używa). Gałęzie `walk_active` i `position_calibration_active` bez zmian
  (dalej wołają `legacy_assist_calculate()`).
- **Persystencja/przełączanie usunięte:** restart-restore silnika (main.c),
  zastosowanie przy postoju, `ride_engine_request`, komenda `0x6027`. Pola
  `MP.ride_engine`/`ride_engine_magic` ZOSTAWIONE w strukturze (rezerwa —
  stabilność układu EEPROM/rekordu FW-023). `0x6028` zostaje (niesie próg SOC
  FW-018), pola silnika ustawione na stałe (ride core / brak pending).
- **Manetka przeniesiona do ride core** (`ride_control.c` + `main.c`): `throttle_iq`
  (map z ADC, jak Legacy) wchodzi do wejścia ride-control i działa jako PODŁOGA
  prądu na wyjściu ride core, cięta przez `safety_cut` (hamulec/cofanie/usterka —
  ostrzej niż stare Legacy). `throttle_offset` jest naturalną bramką (ADC poniżej
  offsetu → 0), więc niepodłączona manetka nic nie daje.
- **Prąd fazowy = 700** (`config.h` `PH_CURRENT_MAX 700`, odwiązane od baterii):
  sanity-cap podniesiony (`parser.c`, 80000/CAL_I), wartość wymuszona po parse
  (`main.c`: `MP.phase_current_max = PH_CURRENT_MAX`) — nadpisuje ewentualny
  zapis Para1[9], żeby wartość programowa zawsze wygrywała.
  **Bezpieczeństwo:** prąd BATERII dalej trzymany na `BATTERYCURRENT_MAX` (15A)
  przez limiter (`main.c` runPIcontrol). 700 to prąd FAZOWY (moment przy ruszaniu).
  Uwaga: więcej ciepła w silniku; Walk Assist też mocniejszy (używa phase_current_max).
- Build: `0.0201_M820_BL820.bin`, SHA-256
  `61EA963E66062BD4B84B39C736C6AA83F06DAC0C0A955613A988506B52F4B787`. Bez błędów.

## 3. Canable — co zrobiono

- Usunięty przełącznik silnika (select „Ride engine" z Profiles; przyciski Set
  Legacy/Set ride core), zakładka „eVistDrive Legacy" (przycisk + treść).
- `canbus.js` `setEngine` (0x6027) usunięty; `server.js` handler `SET_ENGINE`
  usunięty. `readSystem`/`0x6028` ZOSTAJE (próg SOC FW-018).
- Serwer zrestartowany z nowym kodem. UI wymaga odświeżenia przeglądarki (Ctrl+F5).

## 4. Co NIE zostało zmienione (celowo)

- „Banks" i „eVistDrive Profiles" — konfiguracja ride core, ZOSTAJE.
- Monolit Legacy (`legacy_assist_calculate_monolith`) — ZOSTAJE, bo obsługuje Walk
  Assist i kalibrację Halla. Jego gałąź wspomagania pedałowego jest teraz
  nieosiągalna (martwa), ale kod zostaje do bezpiecznego wydzielenia w Kroku B.
- SOC/zasięg, hamulec, temp, błędy, auto-off, limity, HMI — wspólna baza, bez zmian.

## 5. Odroczone (Krok B — z testem na stojaku)

Wydzielić Walk Assist (`main.c:2805-2851`) i kalibrację fazy-2 z monolitu do
osobnych funkcji, usunąć martwą gałąź pedałową Legacy (2853-2947) + `legacy_assist.c/.h`.
NIE robić przed testem WA + kalibracji na sprzęcie.

## 6. Test po wgraniu 0.0201

1. ride core: wspomaganie pedałowe działa (jedyny tryb), mocniejsze ruszanie (prąd 700).
2. Walk Assist: obraca kołem (stojak).
3. Kalibracja Halla `0x6200`: przebiega poprawnie.
4. Manetka (jeśli podłączona u kogoś): daje prąd bez pedałowania, cięta hamulcem/cofaniem.
5. Temperatura silnika przy mocnym ruszaniu — obserwować (wyższy prąd fazowy).
6. Canable: brak zakładki/przełącznika trybu; reszta UI + SOC działa.
