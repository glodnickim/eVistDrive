# Plan: ścieżka mocy silnika — płynna, przewidywalna jazda (styl Bosch)

## Cel (czego chce użytkownik)
1. **Płynne ruszanie** — silnik narasta miękko, proporcjonalnie do nacisku na pedał (jak Bosch), bez szarpnięcia i bez zwłoki.
2. **Brak przeciągania po zaprzestaniu pedałowania** — gdy przestajesz pedałować, moc szybko i gładko schodzi do zera, silnik nie „ciągnie" dalej.
3. **Załączanie przy lekkim pedałowaniu** — delikatny nacisk + ruch korby wystarcza, by wspomaganie ruszyło.

Uwaga od użytkownika: **nie trzymać się ściśle obecnego kodu, jeśli jest ograniczeniem.**
Nowe zmienne dopisać, z myślą o przyszłej **konfiguracji przez CAN** (bloki Para) przez użytkownika.

---

## Jak działa ścieżka mocy DZIŚ (stan faktyczny, zweryfikowany w kodzie)

Łańcuch: `reg_ADC_processing()` @4 kHz (`main.c:1386`) → `update_setpoint()` (`main.c:2545`)
→ slew do `MS.i_q_setpoint` (`main.c:1534-1550`) → FOC (`runPIcontrol`).

### ⚠️ Błąd #1 — maszyna stanów smooth-start jest MARTWYM KODEM
`update_setpoint()` ma **bezwarunkowy `return` w linii 2552** (ścieżka „RESCUE"):
```c
if(MS.brake_active_flag) return assist_apply_common_limits(0);
mapped_throttle = map(...);
uint16_t legacy_current = assist_legacy_running_current(1);
if(mapped_throttle>legacy_current) legacy_current=mapped_throttle;
return assist_apply_common_limits(legacy_current);   // <-- KONIEC, reszta funkcji nieosiągalna
```
Cała maszyna `ASSIST_OFF/ENGAGE/RUNNING/RELEASE` (linie 2554-2661), `assist_start_current()`
(rampa startowa), `assist_shape_running_current()` (sustain + łagodny zjazd) — **nigdy się nie
wykonują.** Wszystkie stałe `ASSIST_ENGAGE_TICKS`, `ASSIST_SUSTAIN_TICKS`,
`ASSIST_CURRENT_FALL_TICKS`, `ASSIST_START_*` (config.h 186-201) są bez efektu.

### Aktywna ścieżka = `assist_legacy_running_current()` (`main.c:2446`)
```c
mapped_torque = map(torque_on_crank, TQO_threshold, 3300, 0, phase_current_max_scaled);
out = TS_coeff * cadence^helper * torque_filtered * 0.0005 * interpolate_assistfactor();
out = map_rezi(out, torque_counter, PAS_timeout, decay_base);   // zanik po czasie bez momentu
if(mapped_torque>out){ Overrun_strength=max(...); out=mapped_torque; }   // boost
// OVERRUN / OVERRIDE:
if(Overrun_counter < (Override_Duration*ext_boost_duration)/100 && ...){
    out = (Overrun_strength*ext_boost_strength)/100;   // trzyma moc PO puszczeniu pedału
    Overrun_flag=1;
    torque_counter = PAS_timeout+1;                    // ubija zanik map_rezi
}
```
- `helper = 1/(1+Cadence_exponent)` (`main.c:476`) → człon `cadence^helper`.
- Slew do i_q: `IQ_SLEW_UP=3` (~13 ms 0→max), `IQ_SLEW_DOWN=5` (~8 ms max→0) (config.h 138-139).
- `Override_Duration`: default 8000 (`main.c:421`) / 4000 (`parser.c:152`), z CAN = `Para1[37]*40` (`parser.c:29`).
- `decay_base`: `Para1[21]` (`parser.c:28`), map_rezi w `main.c:2385`.

### ⚠️ Błąd #2 — PRZECIĄGANIE (główna przyczyna problemu #2 użytkownika)
Blok Override (linie 2462-2466) po zaniku momentu **utrzymuje** `Overrun_strength*ext_boost_strength/100`
przez `Override_Duration*ext_boost_duration/100` ticków i **wymusza `torque_counter=PAS_timeout+1`**,
co blokuje naturalny zanik `map_rezi`. Przy `Override_Duration` rzędu 4000-8000 to setki ms – s
mocy po tym jak przestałeś pedałować. To jest „przeciąganie / silnik się nie wyłącza".

### ⚠️ Błąd #3 — ruszanie nie jest płynne ani proporcjonalne
Przy starcie `cadence=0` (brak jeszcze pomiaru kadencji do `pas_cycle_ticks>70` i
`PAS_STEPS_PER_PULSE=4` kroków), więc główny człon `cadence^helper ≈ 0` → `out≈0`.
Prąd startowy pojawia się **tylko** przez `mapped_torque`/Overrun (skokowo), a nie przez
kontrolowaną rampę momentu. Efekt: albo zwłoka, albo szarpnięcie. Zaprojektowana rampa
startowa (`assist_start_current`, `ASSIST_ENGAGE_TICKS=200 ms`) jest wyłączona (Błąd #1).

### ⚠️ Błąd #4 — zwłoka odcięcia
`torque_counter>4000` (**1 s**) zanim reset kadencji/integrali (`main.c:1552`),
`PAS_STOP_TICKS=2000` (**500 ms**) zanim `cadence=0` (`config.h:160`). W połączeniu z Override
silnik długo „wisi" po zaprzestaniu pedałowania. Slew-down sam w sobie jest OK (~8 ms), problem
jest w progach czasowych i Override.

### ⚠️ Błąd #5 — GŁÓWNY: zmiany mocy W TRAKCIE JAZDY są SKOKOWE (pomijają rampę)
> To jest zgłoszony problem użytkownika: „schodzenie mocy oraz załączanie i wyłączanie silnika
> podczas jazdy" jest szarpane.

Blok slew (`main.c:1534-1550`) rampuje **wyłącznie** przy przejściu przez zero:
```c
static uint8_t slewing_up = 0;
if(MS.i_q_setpoint==0 && iq_target>0) slewing_up=1;   // tylko start od zera
if(iq_target==0 || MS.i_q_setpoint>=iq_target) slewing_up=0;
...
else if(iq_target > MS.i_q_setpoint){
    if(slewing_up){ rampa w górę o IQ_SLEW_UP }
    else{ MS.i_q_setpoint=iq_target; }        // <-- WZROST w jeździe = SKOK, bez rampy
}else if(iq_target < MS.i_q_setpoint){
    if(iq_target==0){ rampa w dół o IQ_SLEW_DOWN }
    else{ MS.i_q_setpoint=iq_target; }        // <-- SPADEK w jeździe = SKOK, bez rampy
}
```
Skutki:
- **Schodzenie mocy** gdy zwalniasz nacisk (cel maleje, ale ≠0) → `i_q` **spada skokowo** do
  nowego celu. Zamiast płynnego zejścia dostajesz szarpnięcie.
- **Załączanie/wyłączanie w ruchu** (np. przez martwy punkt korby, chwilowy spadek momentu,
  krótkie odpuszczenie) → cel skacze między wartościami, `i_q` podąża **skokami**.
- Rampa jest binarna: albo pełny start 0→x (raz), albo pełne 0 (odcięcie). Wszystko „w środku"
  jest natychmiastowe.

**To jest rdzeń problemu** — dokładnie ta różnica względem TSDZ2, który rampuje ZAWSZE (Lekcja #1/#2).
Naprawa: rampować `i_q` w stronę celu **przy każdej zmianie** (góra i dół), krokiem adaptacyjnym
od prędkości/kadencji. Wtedy schodzenie mocy i przejścia w jeździe stają się płynne.

---

## Docelowa architektura ścieżki mocy (styl Bosch)

Zasada Bosch: **moment na korbie × współczynnik wspomagania** = moc, z miękką rampą przy
załączeniu i szybkim, gładkim odcięciem gdy korba przestaje się obracać. Kadencja tylko
lekko modeluje (mostek przez martwe punkty korby), nie warunkuje startu.

Model docelowy (jeden, spójny, deterministyczny):
```
requested = f(torque_filtered) * assist_factor(level)         // proporcjonalne do nacisku
if (świeży ruch korby do przodu) i (torque > mały deadband):
      state -> ENGAGE: rampa 0->requested przez START_RAMP_MS (proporcjonalna do nacisku)
      state RUN: podążaj za requested (w górę natychmiast, w dół z FALL_RAMP)
if (brak ruchu korby przez STOP_MS  lub  brak momentu przez COAST_MS):
      state -> RELEASE: rampa requested->0 przez STOP_RAMP_MS, potem OFF
BRIDGE_MS: krótkie podtrzymanie przez martwe punkty korby (NIE długi overrun)
```
Kluczowe różnice vs dziś: **brak Override/Overrun** trzymającego moc po puszczeniu pedału;
**rampa startowa** zamiast skoku/slew; **krótkie i konfigurowalne** progi stop/bridge.

---

## Plan krok po kroku (dla Opusa)

### KROK 1 — Uwolnić i uporządkować `update_setpoint()`
- Usuń bezwarunkowy `return` z linii 2552 (ścieżka RESCUE) — LUB przepisz `update_setpoint()`
  od zera wg modelu docelowego. Rekomendacja: **przepisać**, bo martwa maszyna stanów ma
  bug „i_q utyka na 0" (komentarz 2546-2547) w warunku wejścia `ASSIST_OFF->ENGAGE`
  (wymaga jednocześnie `fresh_pas && start_torque_latched && assist_enabled` — gdy kadencja
  nie policzy się przy starcie, utyka). Zachowaj throttle-override i `assist_apply_common_limits`.

### KROK 2 — Steady-state: moc proporcjonalna do momentu (Bosch-like)
- Zdefiniuj `requested = map(torque_filtered, TQ_DEADBAND, TQ_FULL_SCALE, 0, phase_current_max_scaled)`
  × `assist_factor` (z `interpolate_assistfactor()` / poziomu). Kadencja jako **mnożnik
  ograniczony** (np. 0.7-1.0), nie jako `cadence^helper` dający 0 przy starcie.
- Zachowaj `assist_apply_common_limits()` (napięcie/temperatura/limit prędkości/legal) — jest OK.

### KROK 3 — Płynny start (problem #1)
- Wprowadź jawną **rampę startową**: przy przejściu z 0→moc narastaj do `requested` przez
  `START_RAMP_MS` (nowa zmienna, np. 300-500 ms), tempo skalowane naciskiem (większy nacisk =
  nieco szybszy start, ale zawsze miękki). Zastąp `IQ_SLEW_UP=3` wartością z `START_RAMP_MS`.
- Warunek startu: `torque_filtered > START_DEADBAND` **i** `assist_forward_steps >= START_MIN_STEPS`
  (lekki, konfigurowalny) → spełnia „załączanie przy lekkim pedałowaniu" (problem #3).

### KROK 4 — Szybkie, gładkie odcięcie (problem #2 — usuń przeciąganie)
- **Usuń blok Override/Overrun** (`main.c:2462-2470`) z normalnej jazdy (lub przenieś za flagę,
  domyślnie OFF). To on trzyma moc i resetuje `torque_counter`.
- Detekcja stopu: brak kroku kwadratury do przodu przez `STOP_MS` (nowa, np. 150-250 ms, dziś
  `PAS_STOP_TICKS=500 ms`) **lub** moment < deadband przez `COAST_MS`. Wtedy `RELEASE`.
- **Mostek martwych punktów**: `BRIDGE_MS` (np. 100-150 ms) krótkiego podtrzymania, żeby moc nie
  migotała między naciskami — ale to setki ms, nie sekundy.
- Rampa zejścia `STOP_RAMP_MS` (np. 120-200 ms) do zera; potem OFF. Skróć progi `torque_counter>4000`
  (`main.c:1552`) do wartości spójnej ze `STOP_MS`.

### KROK 5 — Zachowania bezpieczeństwa (zostawić)
- Hamulec → natychmiast 0 (jest, `2548`), `Backwards_counter>=4` → 0, `overtemp_stage>=2` → 0,
  `s_is_idle` → 0 (`main.c:1551`), throttle-override (max z pedałem). Nie ruszać.

---

## Nowe zmienne (config.h teraz, CAN/Para w przyszłości)

Dodaj w `inc/config.h` z sensownymi domyślnymi, jako **parametry strojenia** (nie hardcode w logice):
| Nazwa | Domyślnie | Znaczenie |
|---|---|---|
| `START_RAMP_MS` | 400 | czas miękkiej rampy 0→moc przy załączeniu |
| `STOP_RAMP_MS` | 160 | czas rampy moc→0 przy odcięciu |
| `START_DEADBAND_MV` | 30 | próg momentu do załączenia (lekki nacisk) |
| `RUN_DEADBAND_MV` | 12 | histereza dolna w trakcie jazdy |
| `START_MIN_STEPS` | 2 | min. kroki kwadratury do przodu przed startem |
| `STOP_MS` | 200 | brak ruchu korby → uznaj stop |
| `BRIDGE_MS` | 120 | podtrzymanie przez martwe punkty korby |
| `TQ_FULL_SCALE_MV` | **3300** | poziom momentu [mV] = pełne wspomaganie (górna granica mapy). Domyślnie 3300 = obecne zachowanie; konfigurowalne przez CAN |
| `ASSIST_CADENCE_MIN_PCT` | 70 | dolny mnożnik kadencji (start nie zależy od kadencji) |

**Przyszła konfiguracja przez CAN (do zaplanowania osobno):** zarezerwuj pola w blokach Para
(`parser.c` `parseParameters`/`buildParameters`) — analogicznie do `Override_Duration=Para1[37]*40`,
`decay_base=Para1[21]`. Wybrać wolne indeksy Para1/Para2, dodać mapowanie do `MotorParams_t`
i ekspozycję w HMI/serwisie. **Nie wdrażać teraz** — najpierw ustrojić stałe na rowerze, potem
przenieść wybrane (START_RAMP_MS, STOP_MS, TQ_FULL_SCALE_MV, assist_factor/poziom) na CAN.

Rozszerz `MotorParams_t` (`inc/main.h`) o odpowiedniki runtime jeśli mają być zmienne w locie.

---

## Wnioski z TSDZ2 OSF (emmebrusa/TSDZ2-Smart-EBike-1) — co przenieść do EBICS

Referencja lokalna: `.external/TSDZ2-Smart-EBike-860C/src/ebike_app.c` (ten sam rdzeń wspomagania).
To firmware jest chwalone za płynność — poniżej mechanizmy, których EBICS NIE ma, i jak je odwzorować.

### Lekcja #1 (kluczowa) — ROZDZIEL „ile prądu" od „jak szybko tam dojść"
TSDZ2 liczy **cel prądu** (`ui8_adc_battery_current_target`) osobno od **tempa dochodzenia**
(`ramp_up/down_inverse_step`). Sterownik płynnie przesuwa duty w stronę celu z zadanym krokiem.
EBICS ma tylko sztywny `IQ_SLEW_UP=3/IQ_SLEW_DOWN=5` stosowany wyłącznie przy przejściach 0.
→ **EBICS:** zamień sztywny slew i_q na regulowany krok rampy (osobno góra/dół), stosowany
zawsze (nie tylko przy 0). To fundament reszty.

### Lekcja #2 — ADAPTACYJNA rampa od prędkości i kadencji (`set_motor_ramp`, ebike_app.c:507)
Krok rampy jest mapowany: przy niskiej prędkości/kadencji rampa WOLNA (miękki, przewidywalny
start), przy rosnącej prędkości/kadencji rampa SZYBKA (żwawa reakcja w jeździe). To jest sedno
„czucia Bosch". Wzór: `map_ui8(wheel_speed, 4-20 kph, default→min)` oraz `map_ui8(cadence, 20-70 rpm, default→min)`, bierze się mniejszy (szybszy) z obu w górę.
→ **EBICS:** wprowadź `iq_slew_up/down` liczone z `MS.Speedx100` i `uint16_cadence_filtered`
(mapa jak wyżej). Zastępuje stały `IQ_SLEW_*`. Daje jednocześnie płynny start (#1) i żwawą jazdę.

### Lekcja #3 — SMOOTH START: tłumienie pierwszego momentu (`apply_smooth_start`, :579)
Gdy `cadence==0 && motor stoi`: pierwszy „delta momentu" jest skalowany 0→100% przez licznik
(`SMOOTH_START_RAMP`). Pierwsze naciśnięcie nie wchodzi skokiem → zero szarpnięcia przy ruszaniu.
→ **EBICS:** przy starcie z postoju przemnóż `torque_filtered` przez narastającą obwiednię
(0→1 w `START_RAMP_MS`) zanim policzysz prąd. Prostsze i skuteczniejsze niż sam slew.

### Lekcja #4 — moc = human power (`apply_power_assist`, :618)
`assist = cadence_RPM × torque_delta × multiplier / stała`, potem prąd = moc/napięcie.
Proporcjonalne, przewidywalne, bez `cadence^helper` dającego 0 przy starcie.
→ **EBICS:** utrzymaj model proporcjonalny do momentu (patrz KROK 2), a start oprzyj o moment
(smooth start #3), nie o kadencję.

### Lekcja #5 — BRAK przeciągania jest naturalny, nie wymuszany
TSDZ2 nie „trzyma" mocy po puszczeniu pedału. Gdy `cadence→0`, cel = 0, a `ramp_down_inverse_step`
(mały przy prędkości = szybko, większy przy niskiej = miękko) gładko sprowadza duty do 0.
„Assist without pedal rotation" to świadoma OPCJA (domyślnie krótki próg), nie ciągły overrun.
→ **EBICS:** usuń blok Override/Overrun (KROK 4). Niech spadek kadencji sam zeruje cel, a
adaptacyjna rampa-dół (#2) da gładkie, krótkie odcięcie. To rozwiązuje problem #2 wprost.

### Lekcja #6 — czyste wyłączenie silnika (ebike_app.c:479)
Gdy `erps==0 && current_target==0 && duty==0` → `motor_disable_pwm()`. Ponowne włączenie tylko
przy wolnych/stojących obrotach. EBICS ma odpowiednik przez `s_is_idle`, ale warto dopiąć
warunek „stoi + zero celu" do twardego zera bez mielenia.

### Lekcja #7 — opcjonalne startup boost / startup assist (:546 / :599)
- **startup boost:** przy niskiej kadencji dodaj % do momentu (tablica wg kadencji, zanika z RPM)
  — mocniejsze ruszanie pod górę bez wpływu na jazdę.
- **startup assist:** utrzymanie prądu na postoju (hill-hold), skalowane do human power.
Oba **konfigurowalne/opcjonalne** w TSDZ2 z wyświetlacza → wzorzec dla „konfiguracja przez CAN".
→ **EBICS:** dodać jako opcje za flagą (domyślnie off), później wystawić przez Para/CAN.

### Mapowanie stałych TSDZ2 → nowe zmienne EBICS
| TSDZ2 | EBICS (nowa) | Rola |
|---|---|---|
| `PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT/MIN` | `IQ_SLEW_UP_SLOW/FAST` | rampa góra wolna(start)/szybka(jazda) |
| `PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT/MIN` | `IQ_SLEW_DOWN_SLOW/FAST` | rampa dół (odcięcie) |
| `SMOOTH_START_RAMP_DEFAULT` | `START_RAMP_MS` | obwiednia tłumienia startu |
| mapa `set_motor_ramp` (4-20 kph, 20-70 rpm) | progi mapy slew | adaptacja od prędkości/kadencji |
| `ui8_power_assist_multiplier_x50` | `assist_factor`/poziom | siła wspomagania |
| startup boost / assist | flagi opcji + przyszłe Para | ruszanie pod górę / hill-hold |

**Wniosek zbiorczy:** największy zysk płynności EBICS uzyska z Lekcji #1+#2+#3+#5 —
adaptacyjna rampa i_q (od prędkości/kadencji) + tłumienie startu + usunięcie Override.
To jest dokładnie różnica między EBICS a TSDZ2 w odczuciu jazdy.

---

## Mapowanie momentu (mV): zero auto vs zakres ustawialny (różnica vs TSDZ2)

### Jak jest dziś
- **Zero (offset):** EBICS auto-kalibruje CIĄGLE — „re-zero on coast" (`main.c:1449-1479`) dosuwa
  `torque_offset_correction`, by spoczynek trzymał stałe `ASSIST_TORQUE_ZERO_MV=740 mV`.
  Rate-limit, granice `TQ_REST_RAW_MIN/MAX`, **Error 25** przy nieprawdopodobnym zerze.
  Zostawiamy — to dobre, bardziej automatyczne niż TSDZ2 (który zeruje tylko przy starcie).
- **Górna granica (max/span):** aktywna ścieżka `assist_legacy_running_current()` (`main.c:2448`):
  `mapped_torque = map(torque_on_crank, MP.TQO_threshold[poziom], 3300, 0, phase_current_max_scaled)`.
  Górna granica **3300 mV jest zahardkodowana** i praktycznie nieosiągalna → nacisk używa
  ułamka zakresu prądu, reszta mocy z członu `cadence×torque_filtered`. Brak liniowości „nacisk→moc".

### TSDZ2 (dla kontrastu)
Zero mierzone przy starcie, a **zakres (nachylenie/max) ustawia użytkownik** (kalibracja
obciążnikami: `pedal_torque_per_10_bit_ADC_step`). Stąd „nacisk działa w zdefiniowanym zakresie".

### Zmiana w EBICS
- Wprowadź `TQ_FULL_SCALE_MV` jako **zmienną** (nie hardcode), użyj jej w miejsce stałej `3300`
  w mapowaniu momentu (`main.c:2448` i analogiczne).
- **Domyślnie `TQ_FULL_SCALE_MV = 3300`** → zero zmian w zachowaniu względem dziś (bezpieczne).
- **Użytkownik zmienia ją przez CAN** (blok Para) — obniżenie (np. do ~1240-1600 mV) daje
  TSDZ2-podobną liniowość: lżejszy nacisk szybciej sięga pełnego wspomagania.
- Zero (`ASSIST_TORQUE_ZERO_MV`) zostaje auto — użytkownik reguluje tylko górę (span).

### Wdrożenie CAN (wzorzec)
Dodaj `MP.tq_full_scale_mv` do `MotorParams_t` (`inc/main.h`), init 3300
(`parser.c` defaults + `main.c` init), zmapuj na wolny indeks Para1/Para2 w
`parseParameters()`/`buildParameters()` (wzorem `Override_Duration=Para1[37]*40`,
`decay_base=Para1[21]`). 3300 mV nie mieści się w jednym bajcie → zapisz jako
`Para[x]=tq_full_scale_mv/16` (0..4080 mV) albo w dwóch bajtach LE. Wybór indeksu i skali
do ustalenia przy wdrożeniu CAN (osobny krok, po strojeniu na rowerze).

---

## Weryfikacja (end-to-end, na rowerze)
1. Zbuduj (`build_firmware.ps1`), wgraj. Podłącz UART debug (`printf` w `main.c:563` daje
   `Battery_Current, i_q_setpoint, i_q, p_human, Speedx100`) i/lub `0x80010203` na CAN.
2. **Start:** lekki nacisk + ćwierć obrotu korby → wspomaganie ma ruszyć miękko w ~`START_RAMP_MS`,
   `i_q_setpoint` narasta gładko (nie skok). Sprawdź brak szarpnięcia.
3. **Stop:** przestań pedałować w ruchu → `i_q_setpoint` schodzi do 0 w ~`STOP_MS+STOP_RAMP_MS`
   (kilkaset ms, nie sekundy). Potwierdź brak przeciągania.
4. **Martwe punkty:** wolne pedałowanie pod górę → moc stabilna dzięki `BRIDGE_MS`, bez migotania.
5. **Regresje:** hamulec ucina natychmiast; throttle nadal działa; limit prędkości/temperatury
   bez zmian; brak Error 25.
6. Log porównawczy `i_q_setpoint` vs czas przed/po zmianie — krzywa startu miękka, zejście krótkie.

## Ryzyka
- Usunięcie Override zmieni „feel" — konieczne strojenie `TQ_FULL_SCALE_MV`/assist_factor.
- `torque_filtered` to EMA (`TQfilter` z poziomu) — zbyt duża filtracja opóźni start; sprawdzić
  `MS.TQfilter` per poziom (`main.c:533`).
- Kadencja przy starcie = 0 (fizyka pomiaru) — dlatego start MUSI bazować na momencie, nie kadencji.
- Zmiana progów stop/bridge wpływa na jazdę pod górę z martwymi punktami — testować na wzniesieniu.
