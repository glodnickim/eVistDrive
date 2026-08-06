# FW-086 — Pierwszy pomiar kadencji po postoju niszczy start wspomagania

- **Data:** 2026-08-05
- **Status:** WDROŻONE 2026-08-05 — testy hostowe zielone. **NIEPRZETESTOWANE NA ROWERZE.**
  Build firmware jeszcze nie wykonany. Punkty z §5 świadomie NIE wdrożone.
- **Cel:** usunąć zapad wspomagania w okolicy 15° obrotu korby przy ruszaniu.
- **Zakres:** `src/main.c` (dekoder PAS), test hostowy. Bez zmian w transporcie i UI.
- **Powiązane:** `FW-025_PAS_STOP_CUT.md`, `FW-016_RIDE_CORE_START_FIX.md`,
  `FW-085_RUN_FILTER_CRANK_ANGLE.md`.

## 1. Usterka

`pas_cycle_ticks` (`src/main.c:227`) mierzy takty między impulsami kadencji. Rośnie
**w każdym takcie 4 kHz** (`src/main.c:1666`) z nasyceniem na 64000, a zerowany jest
**wyłącznie** w gałęzi impulsu kadencji (`src/main.c:1704`).

Gałąź wykrycia postoju (`src/main.c:1728`) zeruje `MS.cadence`, `cadence_seeded`,
`uint16_cadence_filtered`, `pas_fwd_steps`, `fwd_run` — ale **nie `pas_cycle_ticks`**.

Po postoju licznik jest więc nasycony. Przy ruszaniu, w czwartym kroku kwadratury
(`PAS_STEPS_PER_PULSE = 4`):

```c
if(pas_cycle_ticks>70){                  /* 64000 > 70 -> prawda */
    MS.cadence = 10000/pas_cycle_ticks;  /* 10000/64000 = 0 (dzielenie calkowite) */
    cadence_seeded = 0;                  /* kasuje flage chroniaca start */
}
```

Strażnik `>70` odrzuca odczyty absurdalnie **szybkie**. Przed absurdalnie **wolnymi**
nie chroni nic.

### 1.1. Przebieg startu (1 krok kwadratury = 3,75°)

| Krok | Co się dzieje | Wspomaganie |
|---:|---|---|
| 2 (7,5°) | ziarno: `MS.cadence=1`, `cadence_seeded=1` | 0 — latch potrzebuje 4 kroków |
| **4 (15°)** | latch się uzbraja, ale **równocześnie** impuls liczy 0 i kasuje flagę | **zapad do podłogi** |
| 5–7 | brak nowego impulsu, `MS.cadence` zostaje 0 | podłoga |
| **8 (30°)** | drugi impuls mierzy realny odstęp → prawdziwa kadencja | pełne wspomaganie |

`MS.cadence` idzie wprost do `rider_input` (`src/main.c:1774`), a `prepare_assist_input`
(`src/assist_modes.c:590-595`) przy zerowej kadencji zwraca `false` → `iq_target = 0`.

**Wspomaganie realnie rusza w 8. kroku (30°) zamiast w 4. (15°)** — dwa razy więcej ruchu
korby, niż wynika z nastawy „Crank movement to start".

Dotyczy **każdego** startu z zatrzymania: detekcja stopu zapada po 200–500 ms, więc nawet
najkrótsza przerwa daje `pas_cycle_ticks ≥ 800`, czyli kadencję ≤ 12 obr/min.

### 1.2. Drugi, ukryty skutek

Skasowanie `cadence_seeded` przedwcześnie **włącza sufit prądu wyprowadzony z mocy**
(`src/assist_modes.c:695`), który jest celowo pomijany podczas startu. Jedna usterka
psuje więc dwie rzeczy naraz.

## 2. Czy start powinien liczyć się z momentu, a nie z mocy?

Pytanie właściciela. Odpowiedź: **tak — i tak już jest**, wbrew pozorom, jakie stwarza
sztuczna kadencja.

- Żądanie prądu liczy `calculate_load_iq_request()` (`src/assist_modes.c:507-525`) —
  wyłącznie z **obciążenia w kg** i `support_ratio`. **Kadencja nie występuje w tym wzorze.**
- Sufit prądu z mocy jest jawnie pomijany, dopóki trwa faza startu
  (`src/assist_modes.c:695`), z komentarzem wprost: *„At launch duty is near zero, so the
  torque-derived Iq request remains authoritative."*

Czyli zasada „na starcie z momentu, nie z mocy" jest już zamierzona i zaimplementowana.
Tak samo rozwiązuje to TSDZ2 OSF: ma **osobny tryb Startup assist** oraz osobne typy
wspomagania (Power / Torque / Cadence) zamiast wstrzykiwania fałszywej kadencji do wzoru
na moc.

**Ale zastrzeżenie właściciela jest słuszne co do formy.** `START_CADENCE_SEED_RPM = 1`
nie jest żadną kadencją — to **flaga „trwa start"** przebrana za wielkość fizyczną.
Stąd kruchość: wystarczy, że jeden zepsuty pomiar skasuje `cadence_seeded`, i wali się
cała ochrona startu. Docelowo powinien to być jawny stan startu, a nie magiczna wartość
w polu kadencji — patrz §5.

### 2.1. Pozostały realny wpływ mocy na start (Progressive / Curve)

Podczas fazy startu `power_cadence` jest ustawiane na 0 (`src/assist_modes.c:753`), więc
moc ludzka wychodzi 0, a `support_ratio` liczy się właśnie z niej:

| Tryb | `support_ratio` w fazie startu | Skutek |
|---|---|---|
| Power Linear | stały (`calculate_power_linear_support_pct`) | start prawidłowy |
| **Power Progressive / Curve** | z mocy = 0 → **`support_min_pct`** | **start słaby** |
| eMTB / Torque | z momentu², kadencja tylko w mianowniku | start prawidłowy |

To jest **osobna usterka** od §1. Nie naprawiać jej w tej samej zmianie — najpierw
zmierzyć, ile daje sama naprawa licznika.

## 3. Naprawa (zakres tej karty)

Przy **pierwszym kroku do przodu po postoju** rozpocząć nowy pomiar odstępu:

```c
if(fwd_run==0){ pas_cycle_ticks=0; pas_fwd_steps=0; }
```

Umieścić w gałęzi kroku do przodu dekodera PAS, **przed** inkrementacją `fwd_run`
(`src/main.c:1671-1675`), bo `fwd_run == 0` jest tu jedynym wiarygodnym znacznikiem
„pierwszy krok po zatrzymaniu lub po cofnięciu".

Po zmianie impuls kadencji wypada w 5. kroku i niesie **prawdziwy** odstęp czterech
kroków, więc `cadence_seeded` jest kasowane dopiero wtedy, gdy jest czym je zastąpić.
Wspomaganie jest ciągłe od 4. kroku, a sufit prądu z mocy włącza się we właściwym momencie.

Ta sama klasa poprawki co FW-025 i mieści się w jednym warunku. Celowo nie ruszamy nic
innego.

## 4. Testy

Test hostowy `tests/fw086_cadence_first_pulse.js` (port logiki dekodera, styl domowy):

1. Po symulowanym postoju pierwszy impuls daje kadencję zgodną z zadanym odstępem kroków,
   a **nie 0**.
2. `cadence_seeded` nie może zostać skasowane wartością 0.
3. Odstęp odpowiadający 60 obr/min daje 60 obr/min (±1) już przy pierwszym impulsie.
4. Bez naprawy (model sprzed zmiany) pierwszy impuls daje 0 — test dokumentuje regresję,
   przed którą chroni.
5. Krok wstecz zeruje licznik tak samo jak postój (`fwd_run` wraca do 0).
6. Kolejne impulsy podczas ciągłej jazdy nie są zaburzone.
7. Strażnik `>70` nadal odrzuca odczyty absurdalnie szybkie.

## 4a. Wdrożenie (2026-08-05)

Naprawa w gałęzi kroku do przodu (`src/main.c`):

```c
uint8_t cadence_interval_restart = (fwd_run==0);
if(cadence_interval_restart){ pas_cycle_ticks=0; pas_fwd_steps=0; }
...
if(!cadence_interval_restart && ++pas_fwd_steps>=PAS_STEPS_PER_PULSE){
```

**Pierwsza wersja naprawy była błędna i test to wychwycił.** Zerowała tylko licznik
i licznik kroków, przez co impuls wypadał w 4. kroku, obejmując **3 odstępy zamiast 4** —
każdy pierwszy odczyt kadencji wychodził **4/3 za wysoki** (60 obr/min czytane jako 79).

Poprawka: krok restartu jest **początkiem** przedziału, a nie jego pierwszym zliczeniem.
Skrót logiczny `!cadence_interval_restart &&` powstrzymuje inkrementację na tym jednym
kroku, więc impuls wypada krok później i obejmuje pełne `PAS_STEPS_PER_PULSE` odstępów.
Test 7 pilnuje tego strukturalnie, żeby ktoś nie usunął skrótu jako „zbędnego".

Testy: `tests/fw086_cadence_first_pulse.js` — modeluje zachowanie sprzed i po naprawie,
więc dokumentuje regresję, przed którą chroni. Zestaw firmware (8) i Canable (7) zielony.

## 5. Do rozważenia osobno (NIE w tej karcie, NIE wdrożone)

1. **Jawny stan startu zamiast `START_CADENCE_SEED_RPM`.** Zastąpić magiczną kadencję 1
   polem `start_phase` w `rider_input`. Usuwa całą klasę kruchości opisaną w §2 i sprawia,
   że „kadencja" znaczy zawsze jedno.
2. **`support_ratio` w Progressive/Curve podczas startu** (§2.1) — wejściem krzywej
   powinno być obciążenie, a nie moc, dopóki kadencja nie jest mierzalna.
3. **Zweryfikować liczbę przejść kwadratury na obrót.** `src/config.h:350` zawiera jawne
   ostrzeżenie autora, że 96/obrót nie zostało zmierzone („Reverser says 48 pulses/rev
   … VERIFY by measuring before changing to 2"). Dotyczy to również skali stopni w FW-085:
   przy 48 przejściach okno byłoby dwa razy szersze, niż mówi etykieta.

## 6. Kryteria odbioru

- pierwszy impuls kadencji po postoju niesie wartość zgodną z rzeczywistym tempem korby;
- `cadence_seeded` nigdy nie jest kasowane zerową kadencją;
- ruszanie z miejsca bez zapadu wspomagania w okolicy 15° obrotu korby;
- brak zmian w zachowaniu podczas ciągłej jazdy;
- pełny zestaw testów firmware i Canable bez regresji.
