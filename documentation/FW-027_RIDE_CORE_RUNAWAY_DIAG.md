# Karta zmiany FW-027 (diagnostyka) — log ucieczki wspomagania ride core

- **Data:** 2026-07-25
- **Status:** ROZWIĄZANE przez [[FW-028_PI_WINDUP_FIX]] (`0.0199`). Diagnostyka
  FW-027 mierzyła KOMENDĘ prądu (`MS.i_q_setpoint`) — a ta spadała poprawnie.
  Przyczyna leżała ZA komendą: windup regulatora PI (rzeczywisty `MS.i_q`).
  FW-028 dodało telemetrię rzeczywistego prądu i naprawiło przyczynę. Wniosek:
  mierzyć wyjście siłownika, nie komendę.
- **Zakres:** WYŁĄCZNIE diagnostyka. Zero zmian w zachowaniu silnika/wspomagania.
- **Powiązane:** plan `~/.claude/plans/jak-to-jest-w-hidden-whistle.md`;
  [[FW-024_PAS_DIRECTION_LATCH]] (kierunek, zamknięte).

---

## 1. Problem

W ride core silnik napędza rower stałą mocą po zaprzestaniu pedałowania (bez nacisku,
bez kręcenia), gaśnie dopiero po cofnięciu korby. Objaw DYNAMICZNY (tylko pod
obciążeniem) — w spoczynku wszystko czyste (kadencja=0, moment=0, without_rotation
OFF, iq=0). Log na żywo przez agenta zawodzi (USB rwie się w jeździe, brak
internetu). Właściciel loguje sam, potem oddaje CSV.

Uwaga właściciela: pamięć „było przed zmianami" NIEPEWNA — nie traktować jako fakt.

## 2. Co zmieniono (firmware)

Rozgłaszana istniejąca ramka debug `0x00010203` (`print_debug_on_CAN`, wołana co
40 ms, ~25 Hz; `PRINTDEBUG_CAN` było już włączone). Wypełniona danymi ucieczki:

| Bajty | Pole | Znaczenie |
|---|---|---|
| 0–1 | `MS.i_q_setpoint` | prąd silnika — czy napędza? |
| 2 | `MS.cadence` | kadencja rpm — utknięta >0 przy stojącej korbie? |
| 3 | flagi | b0 forward_pedaling, b1 backward≥4, b2 pwm, b3 cadence_seeded, b4 without_rotation |
| 4–5 | `pas_fwd_accum` | **narastający** licznik kroków w przód (nie zerowany) |
| 6–7 | delta momentu | `torque_on_crank − 750` — czy moment podtrzymuje? |

`pas_fwd_accum` narastający → analiza liczy różnicę kolejnych wierszy; przyrost >0
przy stojącej korbie = fałszywe impulsy (EMI). Odporne na gubienie ramek.

Pliki: `src/main.c` (globalna `pas_fwd_accum`, inkrement w kroku w przód dekodera,
treść `print_debug_on_CAN`). Bez zmian w `inc/config.h` (flaga już była).

## 3. Logger (po stronie PC)

Nowy `bafang_canable_pro/debug-logger-cli.js` — samodzielny (bezpośredni Canable,
bez websocketa/agenta/internetu). Nasłuchuje `raw_frame_received`, dla KAŻDEJ ramki
`0x00010203` zapisuje wiersz CSV z timestampem do `logs/`. Bez odpytywania, bez
scalania, bez warunku `speed||cadence` (świadomie — istniejący `logger.js` scala i
gubi ramki). Kolumny: `t_ms;iq_setpoint;cadence;ped;back;pwm;seed;norot;pas_fwd_accum;torque_delta`.

## 4. Build

- **`0.0197_M820_BL820.bin`, 82344 B, SHA-256
  `B96B47C40E1029F44483B0ACFE41509DF308D73873CEA46A1D3579B0AD25F0FB` — DO WGRANIA.**
- `0.0196` (81936 B) BŁĘDNE: `PRINTDEBUG_CAN` było gated pod `#if SEND_DEV_TELEMETRY`
  (=0), więc ramka `0x00010203` NIE była kompilowana — podsłuch na sprzęcie
  potwierdził brak ramki (agent przeoczył warunek). Naprawa w `inc/config.h`:
  bezwarunkowe `#define PRINTDEBUG_CAN` (tylko ramka debug; flood 0x81F83100 dalej
  OFF, HMI nietknięte). Wzrost rozmiaru +408 B potwierdza wkompilowanie.
- Weryfikacja: podsłuch wszystkich ramek na 0.0196 pokazał telemetrię
  (`0x82F8xxxx`, `0x8310xxxx`) ale BRAK `0x00010203` → dowód, że frame nie był
  wysyłany. Po 0.0197 agent sam potwierdzi obecność ramki (sniff), bez jazdy.

## 5. Procedura dla właściciela

1. Wgrać `0.0196`.
2. Na laptopie: `node debug-logger-cli.js` (pisze CSV do `logs/`).
3. Laptop do plecaka, jazda w ride core, wywołać ucieczkę (przestać pedałować, poczekać
   aż silnik ciągnie kilka sekund; raz cofnąć korbą, by uciąć). Powtórzyć kilka razy.
4. Ctrl+C, przysłać plik CSV z `logs/`.

## 6. Analiza (agent, po otrzymaniu CSV)

- iq_setpoint>0 podczas zalegania → silnik napędzany (potwierdza objaw).
- kadencja>0 + przyrost `pas_fwd_accum`>0 przy stojącej korbie → **EMI (dekoder PAS)**.
- kadencja>0 + przyrost=0 → kadencja utknęła z innego powodu.
- delta momentu>0 → tor momentu podtrzymuje.
- flagi → stan bramki. To wskaże warstwę do naprawy.

## 7. Poza zakresem

Poprawka przyczyny (dopiero po analizie logu). Do realnej jazdy: Legacy (bezpieczne);
ride core tylko do wywołania objawu pod log.
