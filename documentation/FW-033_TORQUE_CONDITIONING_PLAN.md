# FW-033 - plan poprawy pulsowania ride core / Ride Core

- **Data:** 2026-07-26
- **Status:** WARIANT A WDROZONY, build `0.0204`. Reszta planu (eMTB blend, peak guard, wariant kadencyjny) odrocona.
- **Build:** `0.0204_M820_BL820.bin`, SHA-256
  `255525F995C00C4830D864EF4E20C5F01BE25DA0B6FEAB922E402F0A9F627A59`. Bez bledow.

## Wdrozenie - wariant A (torque_run)

Potwierdzone z wlascicielem: przyczyna = scenariusz B (nie saturacja; Canable pokazuje
poprawne kg). Ride Core liczyl moc z JEDNEGO szybkiego sygnalu momentu (35 ms), wiec
FOC wiernie odtwarzal kazde pchniecie nogi. Uwaga dewelopera: rozwiazanie referencyjne mialo wolna warstwe
prad-baterii/PWM przed regulacja; EBiCS ma szybki FOC PI, wiec brakuje filtra WEJSCIA.

Wiekszosc potoku dewelopera juz istnieje w EBiCS: raw i_q + clamp (assist_modes),
slew rise/fall na i_q (assist_dynamics), limity fazy/baterii (assist_limits + runPIcontrol).
Brakowal filtr wejscia -> dodany jako `torque_run`.

Zrobione:
- **torque_run** - drugi, wolniejszy filtr sygnalu momentu (kaskada fast 35 ms -> run).
  Power/eMTB/Torque i startup_boost licza z `torque_run` (wygladzanie PRZED kwadratem eMTB).
  Start-gate/latch i safety dalej na szybkim `torque_assist_filtered`. Seed run=fast przy
  starcie (krisp launch). `torque_input.c` `update_run_filter`, `torque_input_seed_run`.
- **Suwak Canable** `assist_torque_run_filter_ms` (0-1000, dom. 300, 0=wyl.=stare zachowanie),
  globalny w tuning (Dynamics). Blob tuning v2->v3 (22->24 B); v2 czytany z backfillem =
  **bez resetu EEPROM** (rozmiar struktury bez zmian).
- **Log raw/run/effective/actual** - diagnostyka 0x6029 v2->v3 (37 B): dodane
  `run pressure`, `measured i_q` (MS.i_q), `battery_limiting` (BC_limit_flag). Pelny lancuch:
  raw torque -> run torque -> iq_request -> iq_setpoint -> measured i_q -> batt-limit.
  Test: iq_setpoint pulsuje z momentem -> filtr wejscia; iq_setpoint stabilny a measured i_q
  pulsuje -> problem nizej w FOC.

Odrocone: eMTB curve blend, peak guard, filtr zalezny od kadencji (wymagaja >24 B blobu -> reset).

---

## (Oryginalny plan dewelopera ponizej)

- **Data:** 2026-07-26
- **Status:** PLAN DO WDROZENIA
- **Powod:** build 0.0203 poprawil gasniecie wspomagania przez latch/floor, ale
  nadal zostalo wyrywanie "lewa noga / prawa noga" przy wiekszym nacisku
  (od ok. 100 W na displayu).
- **Zakres:** firmware Ride Core, diagnostyka CAN, Canable Dynamics. Bez zmian w
  FOC, PWM, hamulcu, reverse cut i Walk Assist.

---

## 1. Diagnoza problemu

FW-031/FW-032 naprawily dolki:

- `assist_latched` trzyma stan jazdy po legalnym starcie,
- `assist_min_iq_pct` robi mala podloge pradu,
- `assist_hold_ms` przezywa martwy punkt korby,
- `power_fall_filter_ms` i `release_ms` zmiekczaja opadanie.

To nie usuwa jednak pikow. Przy mocniejszym pedalaowaniu kazda noga nadal
generuje osobny pik nacisku. Obecny Ride Core moze zamieniac ten chwilowy pik
bezposrednio na pik `iq_request`.

Najbardziej podejrzane sa dwie rzeczy:

1. **Zla skala / saturacja torque** - normalny nacisk moze dobijac do gory
   zakresu (`torque_assist_filtered`, `emtb_target_x160`, `iq_request`), wiec
   algorytm widzi prawie "max" przy kazdym mocniejszym depnieciu.
2. **Brak warstwy kondycjonowania torque przed eMTB** - przeniesiono wzor eMTB,
   ale nie pelne zachowanie RUN, ktore w ride core/Legacy wygladzalo wysilek pedalowania.

Nie wolno najpierw slepo filtrowac koncowego `Iq`. Jesli wejscie torque jest
zle wyskalowane albo saturuje, filtr tylko zamaskuje uszkodzony sygnal.

---

## 2. Co zostalo przeniesione, a czego brakuje

### Przeniesione / istnieje

- eMTB formula w `assist_modes.c`:

```text
target_x160 ~= torque_delta^2 / denominator
denominator = 510 - 2 * emtb_parameter [- cadence] + 10
```

- start guard: forward PAS + minimalny nacisk,
- startup boost,
- latch/floor z FW-031/FW-032,
- globalne rampy `assist_dynamics`,
- `power_rise_filter_ms`, `power_fall_filter_ms`, `release_ms`,
- limity temperatury, napiecia, predkosci, pradu baterii.

### Nieprzeniesione / niedokonczone

1. **Ride Core nie dziedziczy Legacy `TQfilter`.**
   FW-016 mowi wprost: Ride Core uzywa wlasnego filtra torque `35 ms`, a Legacy
   nadal uzywa `MS.torque_filtered` i per-level `TQfilter`. W starym torze
   `TQfilter=6` oznaczal rzad setek ms. Ride Core jest duzo szybszy.

2. **Brak osobnego sygnalu `torque_run` dla jazdy.**
   Ten sam szybki torque jest uzywany do startu i do liczenia mocy/eMTB. Start
   ma byc szybki, ale RUN powinien uzywac estymacji wysilku, nie chwilowego piku.

3. **Brak sprawdzenia saturacji normalnego nacisku.**
   Nie ma jasnej flagi "torque/target dobil do limitu przy zwyklej jezdzie".
   Bez tego nie wiadomo, czy problemem jest dynamika, czy zla skala.

4. **Wzor eMTB jest nieliniowy (`torque^2`).**
   Jesli torque ma sinusoidalne piki nog, kwadrat wzmacnia piki. Samo przeniesienie
   wzoru bez wolniejszego toru wejscia daje efekt "wzmacniacza nog".

5. **Brak crank/PAS-step sustain.**
   Obecny sustain jest czasowy. ride core/Legacy mialo bardziej naturalne powiazanie
   z ruchem korby przez aktualizacje torque/cadence na impulsach PAS i dluzsza
   pamiec filtra. Nie trzeba kompensowac lewej/prawej nogi absolutnym katem, ale
   RUN powinien widziec sredni wysilek przez fragment obrotu korby.

---

## 3. Najpierw diagnostyka, potem algorytm

Developer ma najpierw zrobic log, ktory rozdzieli dwa scenariusze:

- **A: saturacja / zla skala** - naprawic kalibracje/parametry, nie filtrowac.
- **B: brak saturacji, ale duza fala lewa/prawa** - dodac estymator torque RUN.

### Wymagane sygnaly w logu

Dodac do istniejacej diagnostyki Ride Core albo nowej ramki:

```text
torque_raw_mv
torque_zero_mv
torque_delta_mv
torque_assist_fast_mv        // obecny szybki filtr 35 ms
torque_run_mv                // nowy filtr/estymator RUN, po wdrozeniu
torque_span_native
torque_norm_x160
emtb_denominator
emtb_target_x160_raw
emtb_target_x160_effective
iq_request_raw
iq_request_effective
iq_setpoint
iq_actual
cadence
cadence_filtered
startup_boost_active
startup_boost_extra_pct
assist_latched
assist_hold_ticks
limit_flags
```

`limit_flags` minimum:

```text
TORQUE_SAT       // torque_delta blisko span, np. >90%
TARGET_SAT       // emtb_target_x160 blisko 160
IQ_PROFILE_CAP   // max_iq_pct ogranicza
IQ_GLOBAL_CAP    // ride_core_iq_limit ogranicza
POWER_CAP        // P/U power ceiling ogranicza
SPEED_CAP        // legal/speed limit ogranicza
BOOST_ACTIVE
FLOOR_ACTIVE
```

### Kryterium saturacji

Przy normalnej jezdzie testowej 100-200 W na displayu nie powinno byc stale:

```text
torque_delta_mv > 85-90% torque_span_native
emtb_target_x160 > 140
iq_request_raw == limit przez kazdy pik nogi
```

Jesli to wystepuje, najpierw poprawic skale:

- torque calibration / torque span,
- `emtb_parameter`,
- `max_iq_pct`,
- ewentualnie `support` / poziomy banku.

---

## 4. Poprawka algorytmu: dwa tory torque

Dodac dwa rozne sygnaly torque:

```text
torque_fast_mv  // start, safety, diagnostyka natychmiastowego nacisku
torque_run_mv   // normalne liczenie mocy/eMTB po legalnym starcie
```

### Zasady

1. Start z OFF uzywa `torque_fast_mv`.
   Nie wolno startowac z pamieci wolnego filtra.

2. Po `assist_latched=true` tryby Power/eMTB/Torque uzywaja `torque_run_mv`.
   To ma byc estymacja wysilku pedalowania, nie chwilowy pik jednej nogi.

3. `torque_run_mv` resetowac albo sprowadzac do zera przy:

```text
safety_cut
brake
backward PAS
pedaling stopped
torque fault
assist level 0
```

4. `torque_run_mv` nie moze podtrzymywac startu z OFF.
   Po ponownym starcie filtr ma byc zainicjalizowany z aktualnego szybkiego torque
   albo narastac kontrolowanie w ENGAGE/RUN.

### Minimalny wariant implementacji

W `torque_input` albo `ride_control` dodac filtr pierwszego rzedu:

```c
torque_run_q += (torque_fast_q - torque_run_q) / filter_ticks;
```

Parametr:

```text
assist_torque_run_filter_ms
range: 0-1000 ms
default test: 300 ms
tuning: 200, 300, 450, 600 ms
```

Nie dawac jednego filtra 800-1000 ms jako default, bo rower zrobi sie gumowy.

### Lepszy wariant: filtr zalezy od kadencji

Przy niskiej kadencji pik nogi trwa dlugo, wiec filtr powinien byc dluzszy.
Przy wyzszej kadencji za dlugi filtr opozni rower.

Przyklad:

```text
cadence < 40 rpm  -> 400-500 ms
40-70 rpm         -> 250-350 ms
>70 rpm           -> 150-250 ms
```

Parametry:

```text
assist_torque_filter_low_ms
assist_torque_filter_high_ms
assist_torque_filter_cadence_mid_rpm
```

Na start mozna wdrozyc tylko jedno `assist_torque_run_filter_ms`.

---

## 5. Poprawka eMTB: mniej agresywne wzmacnianie piku

Obecne eMTB robi kwadrat torque. To moze byc prawidlowy wzor ride core, ale tylko gdy
wejscie torque jest juz dobrze przygotowane. W EBICS na razie torque jest szybki,
wiec kwadrat robi piki.

Po dodaniu `torque_run_mv` developer ma dodac opcjonalny blend krzywej:

```text
linear = delta_x160
quad   = delta_x160^2 / denominator
target = mix(linear, quad, emtb_curve_pct)
```

Parametr:

```text
emtb_curve_pct
range: 0-100
0   = bardziej liniowo / spokojniej
100 = obecne zachowanie
default test: 50-70
```

Alternatywa: zostawic formule eMTB bez zmian, ale mocno obnizyc `emtb_parameter`.
To jednak jest strojenie poziomow, nie naprawa architektury.

---

## 6. Peak guard dopiero po poprawieniu wejscia

Jesli po poprawnej skali i `torque_run_mv` nadal sa krotkie piki, mozna dodac
ogranicznik piku. To nie jest pierwszy krok.

Logika:

```text
iq_avg = wolna srednia iq_request_effective
max_allowed = iq_avg + headroom_pct
iq_effective = min(iq_raw, max_allowed)
```

Parametr:

```text
assist_peak_headroom_pct
range: 0-150 %
default test: 50 %
```

Warunki:

- nie dziala przy starcie przez pierwsze `engage_ms`, albo ma osobny limit,
- nie blokuje safety cut,
- nie maskuje `TORQUE_SAT` i `TARGET_SAT` w diagnostyce,
- w logu widac jednoczesnie raw i effective.

---

## 7. CANable / tuning_config

Dodac jako globalne Ride Feel / Dynamics, nie per bank:

```text
assist_torque_run_filter_ms       0-1000 ms, default 300
emtb_curve_pct                    0-100 %,  default 60 albo 100 jesli tylko opcja testowa
assist_peak_headroom_pct          0-150 %,  default 0/off na pierwszy build
```

Jesli developer robi wariant kadencyjny:

```text
assist_torque_filter_low_ms
assist_torque_filter_high_ms
assist_torque_filter_cadence_mid_rpm
```

Wersjonowanie:

- podniesc `TUNING_VERSION`,
- rozszerzyc blob 0x6023/0x6024,
- Canable ma backfill defaultow dla starszych kontrolerow,
- zapis flash dalej przez 0x6022 i tylko na postoju.

---

## 8. Test A/B przed kodem

Zanim developer zmieni algorytm, zrobic testy Canable:

1. Ten sam poziom, `startup_boost=OFF`.
2. Ten sam poziom, tryb `Power Linear` zamiast `eMTB`.
3. eMTB z nizszym `emtb_parameter`, np. L3 140 -> 90/100.
4. `max_iq_pct` 100 -> 70.

Interpretacja:

- Pulsowanie znika w `Power Linear`: winna jest nieliniowosc eMTB / skala targetu.
- Pulsowanie zostaje tez w `Power Linear`: winny szybki tor torque lub saturacja.
- Zmniejszenie `max_iq_pct` pomaga tylko czesciowo: pik jest nadal w algorytmie,
  tylko przyciety.

---

## 9. Test po wdrozeniu

Testy drogowe:

1. 80-120 W na displayu, niska kadencja, rowna nawierzchnia.
2. 120-200 W, niska/srednia kadencja, lekki podjazd.
3. Ten sam odcinek: eMTB old vs eMTB z `torque_run_mv`.
4. Power Linear jako kontrola.
5. Start z miejsca: nie moze byc gumowy ani opozniony.
6. Stop pedaling: nie moze wracac przeciaganie mocy.
7. Cofniecie korby i hamulec: twardy cut natychmiast.

Kryteria akceptacji:

```text
brak wyraznego "kopniecia" na kazda noge przy 100-200 W
iq_request_effective ma mniejsza amplitude niz iq_request_raw
torque_run_mv nie dobija do span przy normalnym nacisku
emtb_target_x160 nie siedzi stale przy 140-160
start i reakcja na zmiane nacisku nadal sa akceptowalne
safety cut bez opoznienia
```

---

## 10. Czego nie robic

- Nie filtrowac slepo finalnego `MS.i_q_setpoint` jako glownej naprawy.
- Nie zwiekszac `release_ms`, zeby ukryc piki; to wraca do przeciagania mocy.
- Nie podnosic `assist_min_iq_pct`, zeby przykryc piki; floor naprawia dolki,
  nie gorne szarpniecia.
- Nie startowac z wolnego filtra torque.
- Nie usuwac twardego `safety_cut`.
- Nie zakladac kompensacji lewej/prawej nogi bez dowodu. Najpierw potrzebny jest
  poprawny `torque_run_mv` i log saturacji.

---

## 11. Najkrotsza instrukcja dla developera

1. Dodaj log saturacji torque/target/iq.
2. Sprawdz, czy przy 100-200 W torque lub eMTB target dobija do limitu.
3. Jesli dobija: napraw skale/parametry, nie filtr.
4. Jesli nie dobija: dodaj `torque_run_mv` i uzyj go w eMTB/Power/Torque w RUN.
5. Dopiero potem dodaj opcjonalny blend eMTB albo peak guard.
6. Porownuj `raw` i `effective` w logu, inaczej strojenie bedzie zgadywaniem.
