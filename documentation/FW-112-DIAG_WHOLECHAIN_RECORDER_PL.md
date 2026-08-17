# FW-112-DIAG — Rejestrator zdarzeń całego łańcucha pedal→Iq (część obserwowalności karty FW-112)

**Werdykt: GOTOWE DO PRZEGLĄDU. NIE DO JAZDY.** Obserwowalność NIE jest potwierdzona na rowerze;
wszystkie dowody to testy hostowe na prawdziwym module `fw112_diag.c` + buildy ARM.

**FW-112-DIAG.1 (ta karta): pojemność rekordera 8 → 24.** Pierwszy zrzut z jazdy wypełnił wszystkie
8 rekordów dokładnie w momencie `RECOVERY_ENTER`, chowając ogon odzyskiwania (WAIT/TRACK/COLLAPSE,
ponowny GRANTED, finalne iq_setpoint). 24 × 32 B = 768 B ma pokryć pełną krótką epizod. Polityka
**reject-on-full zachowana** (nie nadpisywanie). **Zero zmian produkcyjnych.** **SCHEMA BEZ ZMIAN**
(poprzednio v4; układ rekordu i ramki 0x1022A–E identyczne — schema koduje layout, nie pojemność).

**PRODUCTION BEHAVIOR CHANGE = NONE.** **FW-113 / WALK ASSIST = UNTOUCHED.**
**FLASH / COMMIT / PUSH = NO** — nic nie zostało wgranane ani zacommitowane.

---

## 1. Cel

Z jednego logu drogowego (zrzut diagnostyczny po jeździe) ma być jednoznacznie widać, DLACZEGO
wspomaganie nie startuje / nie wraca podczas jazdy. Karta pyta: w każdym momencie, gdy rowerzysta
kręci do przodu, KTÓRY etap łańcucha trzyma Iq na 0 — bramka startowa (kroki/nacisk), bezpieczeństwo
kierunku, safety cut, okno rearm, automat odzyskiwania RUN, liczba poziomu, czy sam wynik mode.

**Twardy zakres:** wyłącznie `#if CAN_DIAGNOSTICS_ENABLE`. Zero zmian zachowania FW-112. Zero
nowego RAM w buildzie produkcyjnym (DIAG=0).

## 2. Architektura — osobny event recorder, nie nowa ramka agregatu

Wzorzec: `rearm_delay_diag.c` (FW-111). Nowy moduł diag-only `src/fw112_diag.c` + `inc/fw112_diag.h`.
Aggregate block (0x10200…0x10228) i `DIAG_AGGREGATE_SNAPSHOT_MAX=14` **BEZ ZMIAN** — rozszerzenie
0x10208 NIE jest robione. WHO-ZEROED jest realizowane jako zdarzenie recordera (ZEROED), nie jako
nowa ramka agregatu, bo agregat jest zamrażany w momencie cichego kandydata i zrzucany tylko na
postoju — krawędzie blokady/rearm/kolapsu trwają kilka ticków, głęboko w środku sesji. Jedyny
sposób, by to przetrwało do zrzutu, to nagranie AT TRANZYCJI do kolejki per-sesji.

**Strukturalna ochrona karty** (punkt „no TARGET_RECOVERED as proof"): moduł nigdy nie wnioskuje
sukcesu z żadnego pochodnego znacznika „recovered" — nagrywa wyłącznie to, co było prawdą w ticku
tranzycji, a iq_setpoint/iq_request z tego samego ticku mówią same za siebie.

## 3. Kolejka — ring per-sesji, reject-on-full

- `FW112_DIAG_RECORDS = 24` rekordów × 32 B = 768 B w buforze (FW-112-DIAG.1: było 8 = 256 B;
  pierwszy zrzut z jazdy wypełnił wszystkie 8 w `RECOVERY_ENTER`).
- Kluczowanie session id jak każdy inny źródło zrzutu (`count_session`/`peek_session`/`release_session`).
- **Pełna kolejka = odrzucenie nowego** (`rejected_total++`), NIGDY nadpisanie starszego — zrzut nie
  może stracić historii sesji (ta sama polityka co FW-111). Odrzucenie jest widoczne w logu jako
  `DIAG_ERR_CAPTURES_FULL` + nowy bit trailera `DIAG_TRAILER_F_FW112_REJECTED (0x04)` (karta punkt 10).
- `event_id` to monotoniczny, per-rekord klucz (zawija na 65536) — czytelnik wykrywa każdy event i
  może go spiąć z rekordem FW-111 po parze `(session_id, event_id)`.

## 4. Ramki CAN — 0x1022A (nagłówek) + 0x1022B..0x1022E (4×8 B snapshot)

Rekord = dokładnie 32 B (`_Static_assert` w `fw112_diag.c`), więc 4 ramki danych serializują go bez
padding. Wszystkie ID wolne (0x1021E trailer, 0x1021F..0x10227 rearm, 0x10228 WA).

| EFID | zawartość |
|------|-----------|
| `0x0001022A` | nagłówek: `[0]=FW112_DIAG_SCHEMA_VERSION(1)` `[1]=session_id` `[2-3]=event_id` BE `[4]=event_type` `[5]=reason_bits` `[6-7]=0` |
| `0x0001022B` | snapshot 1: `[0]=session_state` `[1]=dir_state` `[2]=recovery_state` `[3]=fwd_run` `[4]=crank_forward_steps` `[5]=required_steps` `[6]=start_steps` `[7]=cadence_rpm` |
| `0x0001022C` | snapshot 2: `[0-1]=assist_hold_ticks` BE `[2-3]=load_centikg` BE `[4-5]=load_threshold_centikg` BE `[6]=flags` `[7]=0` |
| `0x0001022D` | snapshot 3: `[0-1]=iq_request` `[2-3]=iq_pre_ramp` `[4-5]=iq_setpoint` `[6-7]=iq_actual` (wszystkie i16 BE) |
| `0x0001022E` | snapshot 4: `[0-3]=elapsed_ticks` u32 BE (odstęp od poprzedniego eventu; pierwszy = 0) `[4-7]=0` |

### Eventy (`fw112_diag_event_t`), każdy detekowany jako krawędź

| # | event | krawędź |
|---|-------|---------|
| 1 | `BLOCKED` | wejście w ciągły odcinek „kręci do przodu, brak permission" (`fwd_run>0 && !latched`) — jeden raz na odcinek |
| 2 | `PERMISSION_GRANTED` | sesja weszła w ACTIVE (cold start albo fast rearm; flaga mówi która) |
| 3 | `PERMISSION_REVOKED` | sesja opuściła ACTIVE (inhibit kierunku / zdarzenie terminalne) |
| 4 | `RECOVERY_ENTER` | automat odzyskiwania RUN opuścił IDLE |
| 5 | `RECOVERY_EXIT` | automat wrócił do IDLE (ukończone lub anulowane) |
| 6 | `RECOVERY_COLLAPSE` | TRACK_FAST → WAIT_FRESH_LOAD (utrata nacisku w trakcie odzyskiwania) |
| 7 | `ZEROED` | wejście w „permission jest, iq_setpoint=0" (`fwd_run>0 && latched && iq_setpoint==0`) — pytanie WHO-ZEROED, z powodem od warstwy decydującej |
| 8/9 | `HOLD_ARMED` / `HOLD_EXPIRED` | uzbrojenie (0→>0) / wygaśnięcie (>0→0) grace holda przy zaciągniętym latchu |

### Flagi i powód

- `FW112_FLAG_*`: LATCHED 0x01, FAST_REARM 0x02 (GRANTED był fast rearm), COLD_ARM 0x04 (GRANTED był
  cold start), PWM_ON 0x08, SENSOR_VALID 0x10, CAL_USER 0x20. FAST/COLD liczy moduł z poprzedniego
  stanu sesji (2 → fast, 0 → cold) — nie jest wstrzykiwane z zewnątrz.
- `FW112_REASON_*` (bitfield, wiele bitów naraz): NONE 0x00, LEVEL_ZERO 0x01, DIRECTION 0x02,
  START_STEPS 0x04, LOAD_BELOW 0x08, REARM_GRANT 0x10, RECOVERY_WAIT 0x20, MODE_ZERO 0x40, SAFETY 0x80.
  Liczy go **`ride_control.c`** — warstwa, która decyduje o permission i demand — w ticku na końcu
  gałęzi assist (nowy `static uint8_t ride_diag_reason` + read-only getter
  `ride_control_get_diag_reason()` pod `#if CAN_DIAGNOSTICS_ENABLE`). Rekord BLOCKED/ZEROED zatem
  nazywa dokładny etap trzymający Iq na 0, a nie re-derivuje go w main.c z wtórnych kopii.

## 5. Podpięcie w main.c

- `fw112_diag_init()` w `diag_diagnostics_init()` (po `rearm_delay_init()`).
- W `reg_ADC_processing()`, PO `ride_control_update()` i PO bloku rearm FW-111 (linia ~2775), blok
  pod `#if CAN_DIAGNOSTICS_ENABLE`: `fw112_diag_set_session_id(diag_session_current_id())`, budowa
  `fw112_diag_input_t` z getterów tylko-do-odczytu (`ride_control_get_session_state/debug_flags/
  assist_hold_ticks/gate_snapshot/arm_snapshot/diag_reason`, `pas_direction_*`, `torque_input_*`,
  `rider_input_get()`, `tuning_config_start_steps()`, `assist_modes_get_last_output()`, `MS.i_q*`,
  `ui_8_PWM_ON_Flag`, `diag_clamp16`), potem `fw112_diag_tick(&in, control_now)`. Umieszczenie PO
  decyzji gwarantuje, że każdy odczytany stan jest z tego samego ticku.
- **5. źródło zrzutu**: `DIAG_SRC_FW112 = 4`, `DIAG_SRC_COUNT = 5`, `DIAG_REC_FW112`, `kind_of_source`
  w `diag_session.c`, mostek `diag_fw112_count/frame/release/accepted/refused` + 5. wpis `diag_ops`
  (serializacja nagłówek+4 fragmenty, ta sama dyscyplina co mostek rearm).
- **`DIAG_SCHEMA_VERSION` 3 → 4** (komentarz przy definicji opisuje, że v3 log czyta stare ramki
  identycznie i po prostu nie zawiera 0x1022x).

## 6. Budżet RAM (zmierzony, nie szacowany)

Metoda taka jak w `inc/diag_budget.h`: izolowany single-file compile na arm-none-eabi-gcc 13.2.1
(`CAN_DIAGNOSTICS_ENABLE=1`), `arm-none-eabi-size` odczytuje data+bss:

| moduł | zmierzony bss | linia budżetu | uwagi |
|-------|---------------|---------------|-------|
| `fw112_diag.c` (NOWY) | **800 B** | 864 B | struct R po FW-112-DIAG.1: 24×32 B rekordy (768 B) + ~32 B bookkeeping/edge; +64 B headroom (było 288/352) |
| `diag_session.c` | **1204 B** | 1268 B | +60 B dokładnie: 5. źródło = po jednym elemencie w 4 tablicach liczników + 2 tablicach refused |
| suma linii | | **13192 B** | razem z headroomem (12680 + 512 = 13192) — wciąż pod sufitem 13 KB (13312) |

**Sufit 12 KB → 13 KB** (karta FW-112-DIAG). Pełny obraz DIAG=1: bss **24444 B** (po FW-112-DIAG.1;
było 23932 B, **+512 B dokładnie = 16 × 32 B** na same rekordy), text 127012 / data 268 — daleko pod
48 KB SRAM GD32F303RCT6. DIAG=0: bss **11692 B — BAJT-W-BAJT bez zmian** względem baselinu
(produkcja nie zyskuje ani bajta stanu; rekorder kosztuje ZERO w DIAG=0).

## 7. Testy hostowe — `tests/host/fw112_diag_host.c` (prawdziwy `fw112_diag.c`)

Moduł jest odsprzężony (całe wejście = `fw112_diag_input_t`), więc harness jedzie nim bezpośrednio —
bez stubów. S1–S14:

| # | dowód |
|---|-------|
| S1 | jedna krawędź BLOCKED na cały ciągły odcinek (brak spamu per-tick); pierwszy event id 0, elapsed 0 |
| S2 | BLOCKED niesie `reason_bits` warstwy decydującej (dwa bity naraz) |
| S3 | GRANTED cold start — flaga COLD_ARM, snapshot ACTIVE + iq_setpoint |
| S4 | rearm: REVOKED potem GRANTED z FAST_REARM, po jednym na krawędź |
| S5 | krawędzie odzyskiwania ENTER/EXIT/COLLAPSE (TRACK→WAIT), brak spamu w stanie steady |
| S6 | HOLD_ARMED (0→>0) i HOLD_EXPIRED (>0→0) przy latched; ignorowane przy !latched |
| S7 | ZEROED z powodem WHO-ZEROED; brak re-fire w ciągłym odcinku; !latched z iq 0 to BLOCKED, nie ZEROED |
| S8 | `sizeof(fw112_diag_record_t)==32`; id monotoniczne 0,1,2; elapsed = 0/21/8 |
| S9 | kolejka per-sesji: count/peek/release 1 nie rusza sesji 2; ring się przesuwa poprawnie |
| S10 | pełna kolejka = odrzucenie (rejected = made−FW112_DIAG_RECORDS, enqueued = FW112_DIAG_RECORDS), nigdy nadpisanie |
| S11 (FW-112-DIAG.1) | pojemność = dokładnie **24**: 24 eventów przyjętych, record_count == 24, brak korupcji, nic poniżej pojemności odrzucone (dosłowne 24, nie makro — patrz M1 niżej) |
| S12 (FW-112-DIAG.1) | 25. event ODRZUCONY: rejected == 1, record_count zostaje 24, enqueued bez zmian — reject-on-full, nigdy nadpisanie |
| S13 (FW-112-DIAG.1) | rekordy 0..23 przetrwały w całości, w kolejności, żaden nie nadpisany przez odrzucony 25. |
| S14 (FW-112-DIAG.1) | realistyczna sekwencja >8 eventów (BLOCKED, GRANTED cold, HOLD_ARMED, ZEROED, REVOKED, BLOCKED, GRANTED fast, RECOVERY_ENTER, COLLAPSE, EXIT, ZEROED — 11 eventów) zapisana end-to-end z dokładnymi typami i flagami cold/fast |

**Wynik: ALL PASS** (gcc 13.2.1 w64devkit, `-Wall -Wextra -Werror`, `CAN_DIAGNOSTICS_ENABLE=1`).

### Mutacje M1–M5 (każda udowodniona: po wprowadzeniu mutacji odpowiedni test FAIL)

| mutacja | skutek | łapie |
|---------|--------|-------|
| M1 | reason zawsze 0 (usunięty passthrough) | **FAIL S2** (i S7) |
| M2 | usunięta detekcja krawędzi sesji | **FAIL S3/S4** (23 assercje) |
| M3 | spam per-tick BLOCKED (usunięty flag odcinka) | **FAIL S1** |
| M4 | usunięta detekcja krawędzi odzyskiwania | **FAIL S5** |
| M5 | `event_id` nie rośnie | **FAIL S8** |

Każdą mutację zweryfikowano przez: patch źródła → build → run → potwierdzenie FAIL → **pełne
przywrócenie źródła** → ponowny run ALL PASS.

### Mutacje FW-112-DIAG.1 M1–M3 (pojemność; każda udowodniona FAIL i przywrócona)

| mutacja | skutek | łapie |
|---------|--------|-------|
| M1 | `FW112_DIAG_RECORDS` wstecz do 8 | **FAIL S11** (record_count == 8 ≠ 24, enqueued ≠ 24, rejected ≠ 0), **FAIL S12**, **FAIL S13**, **FAIL S14** (11 > 8 — ogon ucięty) |
| M2 | warunek odrzutu `>= RECORDS-1` (off-by-one: tylko 23) | **FAIL S11** (record_count == 23), **FAIL S12/S13** |
| M3 | guard odrzutu usunięty — 25. zapisany poza pojemnością (nadpisuje najstarszy, q_depth=25) | **FAIL S12** (rejected == 0, count > 24), **FAIL S13** (najstarszy nadpisany) |

Każdą zweryfikowano tym samym cyklem patch → build → run → FAIL → restore → ALL PASS.

## 8. Buildy ARM (build_firmware.ps1, BL820)

| build | bss | flash (bin) | werdykt |
|-------|-----|-------------|---------|
| DIAG=1 `0.0358_M820_BL820.bin` | 23 932 B | 127 180 B | PASS (karta FW-112-DIAG) |
| DIAG=0 `0.0360_M820_BL820.bin` | 11 692 B | — | PASS — bss bez zmian względem baselinu |
| DIAG=1 `0.0362_M820_BL820.bin` (FW-112-DIAG.1) | 24 444 B | 127 280 B | PASS — bss +512 B dokładnie (16 × 32 B), text 127012 / data 268 |
| DIAG=0 `0.0363_M820_BL820.bin` (FW-112-DIAG.1) | 11 692 B | 98 256 B | PASS — bss bez zmian względem baselinu (zero kosztu produkcyjnego) |

Ostrzeżenia: wyłącznie wcześniej istniejące (`CAN_Display.c` pointer-sign, `main.c` unused
`fw_ver`) — żadnego z nowego kodu.

## 9. Obszary NIETKNIĘTE (zakres chroniony)

- **`src/pas_direction.c`, `src/ride_session.c`, `src/torque_input.c`, `src/assist_modes.c`,
  `src/assist_start.c`, `src/assist_extended_boost.c`, `src/assist_dynamics.c`, `src/assist_limits.c`**
  — bez zmian (ani jednej linii).
- **`src/ride_control.c`** — jedyna zmiana to nowy `ride_diag_reason` + getter pod
  `#if CAN_DIAGNOSTICS_ENABLE`; żadna decyzja nie czyta tego stanu.
- **`FW-113 / Walk Assist** (`walk_assist_motor.c`, `walk_speed_controller.c`, ramka 0x10228,
  `walk_timeout_s`) — **UNTOUCHED**, zero zmian.
- Agregat 0x102xx (0x10200–0x10228), `DIAG_AGGREGATE_SNAPSHOT_MAX=14`, progi jazdy, limiter,
  kalibracja, FW-111 ramki/schema — bez zmian.
- Build DIAG=0: zero nowego stanu (bss 11692 B = baseline), także po FW-112-DIAG.1.
- FW-112-DIAG.1 objął wyłącznie: `FW112_DIAG_RECORDS` 8→24 w `inc/fw112_diag.h`, linia budżetu
  `DIAG_BUDGET_FW112_DIAG_BYTES` 352→864 w `inc/diag_budget.h`, testy S11–S14 + mutacje M1–M3,
  ten dokument. **Ani jednej zmiany w logice rekordera, main.c, diag_session, ride_control.**

## 10. Uwagi i ryzyka

- `run-host-tests.ps1` nadal BLOCKED przez Windows Defender (`Trojan:Win32/Wacatac.B!ml` kwarantanna
  exe z `$env:TEMP`) — harness FW-112-DIAG zbudowany i uruchomiony ręcznie do katalogu repo (PASS);
  wpis w `run-host-tests.ps1` dodany; żadnego obejścia nie zastosowano.
- Po FW-112-DIAG.1 kolejka trzyma 24 rekordy/sesję (768 B, +512 B bss w DIAG=1, zero w DIAG=0) —
  wcześniejsze nasycenie wszystkich 8 dokładnie w `RECOVERY_ENTER` (pierwszy zrzut z jazdy) było
  powodem tej karty. Odporniejsze, ale wciąż skończone: nadmiar w nadzwyczaj zdarzeniowej jeździe
  jest odrzucany i widoczny w logu (trailer bit 0x04), więc brak danych nie jest cichy.
- `DIAG_SCHEMA_VERSION` **BEZ ZMIAN** po FW-112-DIAG.1 (nadal 4): układ rekordu 32 B i ramki
  0x1022A–E identyczne — schema koduje layout, nie pojemność. Parser v3 czyta wszystkie stare ramki
  identycznie, 0x1022x po prostu nie występują w starym logu — kompatybilność wsteczna jednoznaczna.

## 11. Pliki

- NOWE: `src/fw112_diag.c`, `inc/fw112_diag.h`, `tests/host/fw112_diag_host.c`.
- ZMIENIONE (karta FW-112-DIAG): `src/main.c` (include, blok tick, mostek `diag_fw112_*`, 5. wpis
  `diag_ops`, `fw112_diag_init`, `DIAG_SCHEMA_VERSION 3→4`), `inc/diag_session.h` + `src/diag_session.c`
  (5. źródło, `DIAG_REC_FW112`, trailer bit 0x04), `src/ride_control.c` + `inc/ride_control.h`
  (`ride_control_get_diag_reason`), `inc/diag_budget.h` (linia FW-112-DIAG, sesja 1204→1268,
  sufit 12→13 KB), `tests/host/run-host-tests.ps1` (rejestracja suity).
- ZMIENIONE (FW-112-DIAG.1): `inc/fw112_diag.h` (`FW112_DIAG_RECORDS` 8→24), `inc/diag_budget.h`
  (linia 352→864), `tests/host/fw112_diag_host.c` (S11–S14 + mutacje M1–M3), ten dokument.
  **`src/fw112_diag.c` — bez zmian.**

## 12. Uruchomienie

- Harness: `powershell -File tests/host/run-host-tests.ps1` (suita FW-112-DIAG) albo ręczny build gcc
  wg sekcji 7 z wyjściem poza `$env:TEMP`.
- Build ARM: `powershell -File build_firmware.ps1 -BootloaderMode 820` (+ `-CanDiagnostics` dla DIAG=1).

**Nie flashować. Nie commitować. Nie pushować.** Karta do przeglądu właściciela.