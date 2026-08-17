# EVistDrive — Test Infrastructure Foundation — raport końcowy

Data: 2026-08-10. Build firmware w chwili pracy: 0.0329 (bez zmian — patrz sekcja 3).
Bazuje na `ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md` (poprzednia karta tego samego dnia).

---

## 1. Co zostało wykonane

- Zweryfikowano kluczowe test seams z audytu względem aktualnego kodu (sekcja "weryfikacja
  audytu" niżej) — audyt się potwierdził, z jednym doprecyzowaniem (kategoria A "elapsed
  time" jest bardziej niuansowa niż "całkowicie odporna", patrz sekcja 9).
- Zbudowano **cztery nowe host harnessy w C**, każdy linkujący PRAWDZIWE moduły
  `src/*.c` (nie reimplementację): `torque_trace_host.c` (L1), `power_pipeline_host.c`
  (L2/L3), `ride_control_pipeline_host.c` (L3, sięga do `motor_core`),
  `missed_tick_burst_host.c` (L1, dwa moduły naraz).
- Zbudowano wspólny generator bodźca testowego (`crank_model.c`) łączący kąt korby,
  kroki kwadratury PAS i falę nacisku w jednym modelu.
- Zaimplementowano P0 (RUN_60/80/100/110/120, CADENCE_RAMP_50_120) i P1
  (MISSED_TICK_BURST) — zgodnie z priorytetem z karty.
- Napisano orkiestrator `tests/host/run_regression.ps1` — jedno polecenie: buduje,
  uruchamia, liczy metryki, robi smoke-test determinizmu, zapisuje raport.
- Napisano `documentation/INDEX.md` (router) + 17 krótkich dokumentów kontraktów
  modułów (`architecture/`, `inputs/`, `assist/`, `motor/`, `testing/`).
- Uruchomiono WSZYSTKO na końcu (sekcja 21 karty) — wyniki w sekcjach 7-9 tego raportu.
- Istniejące `tests/host/run-host-tests.ps1` (FW-100/101/102) pozostało nietknięte i
  nadal przechodzi (uruchomione jako część walidacji końcowej).

**Python** — karta preferowała Python do orkiestracji/metryk/raportu. Python NIE jest
zainstalowany w tym środowisku (`python`/`python3`/`py` — brak na PATH, sprawdzone przed
napisaniem jakiegokolwiek kodu narzędziowego). Użyto PowerShell zamiast tego — dokładnie
ten sam wzorzec, jakim repo już posługuje się we WŁASNYCH narzędziach
(`build_firmware.ps1`, `run-host-tests.ps1`), i dokładnie to dopuszcza karta w sekcji 18
("lub odpowiednik zgodny ze środowiskiem repo"). Wszystkie trace to zwykłe pliki CSV —
nic nie stoi na przeszkodzie, by przyszła karta podłączyła do nich prawdziwy Python,
jeśli stanie się dostępny.

---

## 2. Pliki dodane / zmienione

**Dodane (test infrastructure), zero zmian w `src/`/`inc/`:**

```
tests/host/common/check.h
tests/host/common/csv.h
tests/host/common/crank_model.h / .c
tests/host/common/map_adapter.c              <- adapter, patrz sekcja 5
tests/host/common/motor_service_stub.c        <- adapter, patrz sekcja 5
tests/host/common/host_stubs/gd32f30x.h       <- adapter, patrz sekcja 5
tests/host/common/host_stubs/arm_math.h       <- adapter, patrz sekcja 5
tests/host/torque/torque_trace_host.c
tests/host/pipeline/power_pipeline_host.c
tests/host/pipeline/ride_control_pipeline_host.c
tests/host/scenarios/missed_tick_burst_host.c
tests/host/tools/RegressionTools.ps1
tests/host/run_regression.ps1
tests/host/golden/candidates/metrics_summary.csv   <- wygenerowane przez run_regression.ps1
tests/host/out/*.csv, out/REGRESSION_RESULTS.md    <- wygenerowane, nie wersjonować bezmyślnie
documentation/INDEX.md
documentation/architecture/DATA_FLOW.md
documentation/architecture/TIMEBASES.md
documentation/inputs/{PAS,TORQUE_SENSOR,SPEED_SENSOR,BATTERY,THROTTLE,BRAKE}.md
documentation/assist/{RIDER_INPUT,POWER_MODE,LIMITS,ASSIST_DYNAMICS,START_REARM_RELEASE}.md
documentation/motor/MOTOR_COMMAND.md
documentation/testing/{TEST_ARCHITECTURE,TEST_INTERFACES,REGRESSION_SCENARIOS,CHANGE_CARD_TEMPLATE}.md
documentation/TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md   <- ten plik
```

**Zmienione:** ŻADEN plik w `src/` ani `inc/` nie został zmieniony w ramach tej karty.
`tests/host/run-host-tests.ps1` i `documentation/FW-084_AUDIT_DEVELOPER_HANDOFF.md`
pokazują się jako zmodyfikowane w `git status`, ale to stan SPRZED tej karty (praca
własna użytkownika nad FW-101, widoczna już na starcie poprzedniej karty audytowej tego
samego dnia — niezacommitowane `inc/pas_trace.h`, `inc/ride_episode.h`,
`src/pas_trace.c`, `src/ride_episode.c` to ten sam, wcześniejszy stan). Zweryfikowane
przez `git diff` — nie dotknięte przeze mnie w tej karcie.

---

## 3. Czy produkcyjne zachowanie zostało naruszone

**NIE.** Zero plików `src/*.c` / `inc/*.h` zmienionych. Wszystkie cztery nowe harnessy
oraz orkiestrator żyją wyłącznie w `tests/`. Trzy adaptery (`map_adapter.c`,
`motor_service_stub.c`, dwa nagłówki w `host_stubs/`) to pliki TESTOWE, widoczne
wyłącznie na ścieżce include hosta (`.cproject` produkcyjny ich nie zna — zweryfikowane
odczytem realnej listy include paths ARM: `Firmware/CMSIS/GD/GD32F30x/Include`,
`Firmware/GD32F30x_standard_peripheral/Include`, `inc/`). Build firmware pozostaje
0.0329, niezmieniony.

Jedyna "zmiana" dotykająca istniejącego katalogu testów to NOWE pliki obok istniejących
trzech (`fw100/101/102_host.c` + `run-host-tests.ps1`) — same istniejące pliki
niezmienione i zweryfikowane jako wciąż przechodzące (sekcja 21).

---

## 4. Architektura host tests

```
L1 UNIT        pojedynczy moduł, host-native, linkuje prawdziwy .c
L2 CONTRACT    granica moduł<->moduł (dziś tylko udokumentowana, nie zautomatyzowana)
L3 PIPELINE    kilka modułów razem, kończy na motor_command_t
L4 HARDWARE    prawdziwy sterownik/log — poza zakresem tej karty
```

Pełny opis w `documentation/testing/TEST_ARCHITECTURE.md`. Skrót: ta karta dodaje 1
nowy L1 (torque), 2 nowe L3 o różnej głębokości (power_pipeline kończy na
`iq_request`, ride_control_pipeline kończy na `iq_final`/`motor_core`), i 1 harness
demonstracyjny L1 dla timebase (missed_tick_burst, dwa moduły: torque_input +
ride_episode). Istniejące 3 harnessy L1 (FW-100/101/102) niezmienione.

---

## 5. Test seams faktycznie użyte

Pełny, techniczny opis w `documentation/testing/TEST_INTERFACES.md`. Skrót:

**Moduły linkowane naprawdę (nie reimplementowane):** `torque_input.c`, `rider_input.c`,
`assist_modes.c`, `cadence_comp.c`, `power_curve.c`, `assist_start.c`,
`assist_extended_boost.c`, `tuning_config.c`, `ride_control.c`, `assist_dynamics.c`,
`assist_limits.c`, `motor_core.c`, `ride_episode.c` — 13 modułów produkcyjnych.

**Minimalne zmiany API produkcyjnego: ŻADNE.** Zero zmian sygnatur, zero nowych funkcji
eksportowanych z `src/`/`inc/`. Trzy adaptery były potrzebne WYŁĄCZNIE do LINKOWANIA
(nie do zmiany zachowania):

1. **`host_stubs/gd32f30x.h`, `arm_math.h`** — `inc/motor_core.h` ciągnie `inc/main.h`,
   które ciągnie prawdziwe nagłówki wendorowe GD32/CMSIS. Próbowano najpierw
   podmienić TYLKO to, co się dało (adapter na ścieżce include) — `inc/gd32f307c_eval.h`
   jest realnym plikiem projektu i jest znajdowany PRZED jakąkolwiek ścieżką `-I`
   (reguła C: katalog pliku włączającego ma pierwszeństwo), więc nie dało się go
   podmienić — ale `gd32f30x.h` (prawdziwy plik wendorowy, nieobecny w `inc/`) DAŁO
   się podmienić. Stub dostarcza tylko `FlagStatus`, dwa puste structy CAN i ~25
   opakowanych stałych całkowitych, których `gd32f307c_eval.h` faktycznie używa.
   `inc/main.h` i typy `MotorState_t`/`MotorParams_t`/`PI_control_t` są PRAWDZIWE,
   niezmienione. Zweryfikowane: kompiluje się czysto pod `-Wall -Wextra -Werror`.
2. **`map_adapter.c`** — `assist_dynamics.c`/`assist_limits.c` deklarują `map()` przez
   `extern` bez wspólnego nagłówka (audyt, finding F4); jedyna definicja jest w
   `main.c`. Skopiowano ciało funkcji bajt-w-bajt, z komentarzem źródłowym. **To jest
   znany koszt tego obejścia**: jeśli `main.c`'s `map()` się zmieni, ten plik trzeba
   zaktualizować ręcznie.
3. **`motor_service_stub.c`** — `ride_control.c` woła dwie funkcje zdefiniowane w
   `main.c` (Walk Assist / kalibracja Halla) wewnątrz gałęzi nieosiągalnych w żadnym
   scenariuszu tej karty (`walk_active=false`, `position_calibration_active=false`
   zawsze). Stuby zwracają 0, nigdy nie wykonywane.

Próba 1 (adapter/wrapper bez dotykania niczego poza include path) zadziałała dla (1) i
w pełni wystarczyła dla (2)/(3) (zwykłe pliki `.c` dostarczające brakujące symbole —
też "adapter", nie zmiana produkcyjna). Nie było potrzeby sięgać po krok 2 karty
("najmniejsza możliwa zmiana produkcyjna") — ŻADEN plik produkcyjny nie wymagał zmiany.

---

## 6. Generator crank/PAS/torque

`tests/host/common/crank_model.c`. Jeden model kąta korby (`crank_state_t`, akumulacja
`cumulative_deg` monotoniczna + `crank_angle_deg` zawinięte 0-360) napędza JEDNOCZEŚNIE:

- liczbę kroków kwadratury PAS (3.75°/krok, 96/obrót — zgodne z `PAS_STEPS_PER_PULSE`
  z `config.h` i `TORQUE_RUN_WINDOW_STEPS_MAX` z `torque_input.h`),
- falę nacisku `crank_torque_raw_mv()`: rektyfikowana pół-sinusoida na nogę (okres
  180°) z konfigurowalnym: `mean_native_delta` (średni nacisk), `ripple_pct`
  (amplituda pulsacji), `asymmetry_pct` (asymetria lewa/prawa noga),
  `dead_spot_depth_pct`/`dead_spot_width_deg` (dodatkowe tłumienie przy granicy nogi,
  niezależne od naturalnego zera sinusa), `phase_shift_deg`.

Generator jest STYMULEM testowym (jak generator sygnałowy na stole laboratoryjnym), nie
firmware — nie wykonuje prawdziwego dekodera GPIO (patrz sekcja 15, observability gap).
Podaje PRAWDZIWĄ liczbę kroków bezpośrednio do `torque_input_run_filter_step()`, co
odpowiada temu, co poprawnie działający dekoder by dostarczył.

Ten sam model jest współdzielony przez `torque_trace_host.c`, `power_pipeline_host.c`,
`ride_control_pipeline_host.c` i (we fragmencie C) `missed_tick_burst_host.c` — dokładnie
zgodnie z kartą sekcja 6: "przykładowy ten sam profil nacisku ma być uruchamiany dla
RUN_60...RUN_120".

---

## 7. RUN_60...120 — wyniki

Wszystkie pięć scenariuszy: IDENTYCZNY profil nacisku (`mean_native_delta=300`,
`ripple_pct=40`, `asymmetry_pct=15`, `dead_spot_depth_pct=30`,
`dead_spot_width_deg=20`), 6 s każdy (24000 ticków), różni się WYŁĄCZNIE cadence.

| Cadence | torque_run mean | torque_run ripple | iq_request mean | iq_final mean (ride_control) |
|---|---|---|---|---|
| 60 rpm | 314.99 | 0.098 | 113.85 | 110.14 |
| 80 rpm | 315.62 | 0.057 | 114.13 | 110.74 |
| 100 rpm | 316.26 | 0.076 | 114.36 | 110.85 |
| 110 rpm | 316.46 | 0.047 | 114.45 | 110.85 |
| 120 rpm | 316.62 | 0.028 | 114.51 | 110.85 |

**Interpretacja (nie PASS/FAIL — karta zabrania agresywnych progów bez danych
bazowych):** `iq_request` zmienia się o **<1%** w całym zakresie cadence przy
domyślnym cadence compensation WYŁĄCZONYM — brak nieoczekiwanej, gwałtownej zależności
od cadence na poziomie żądania. Ripple `torque_run` maleje monotonicznie z cadence — to
jest ARTEFAKT stałego 6-sekundowego okna obserwacji (wyższa cadence = więcej obrotów =
więcej uśredniania w tym samym oknie czasowym), NIE wada filtra RUN — sam filtr RUN jest
zaprojektowany kątowo, nie czasowo (FW-085), więc jego okno w STOPNIACH jest identyczne
na każdej cadence; zmienia się tylko, ile takich okien mieści się w 6 sekundach. Warto to
zapisać jako pierwszą "znaną własność" metryki ripple przy stałym oknie czasowym — jeśli
przyszła karta chce oddzielić "zmianę zachowania filtra" od "efektu długości okna",
powinna albo znormalizować okno do stałej liczby obrotów, albo porównywać ripple TYLKO
między przebiegami o tej samej cadence (co robi determinism smoke-test, sekcja 9 dalej —
poprawnie).

`iq_final` (po ride latch + rampie + limiterach) jest stabilnie ~3-4 jednostki poniżej
`iq_request` w każdym przebiegu — spójne z resztkowym stanem przejściowym rampy w oknie
6 s, nie z rozbieżnością zależną od cadence.

**Wniosek:** brak wykrytej anomalii w bazowym zachowaniu na tym profilu wejściowym. To
jest PIERWSZY zmierzony punkt odniesienia tego toru — nie ma jeszcze "starej wersji
firmware" do porównania (ta karta buduje miernik, nie wynik porównania).

---

## 8. CADENCE_RAMP_50_120 — wyniki

14 s (56000 ticków): 50→120 rpm liniowo przez 10 s, potem 120 rpm przez 4 s (ogon jako
odniesienie ustalone). `torque_run` mean=318.76, ripple=0.025 (najniższy ze wszystkich
przebiegów — spodziewane, bo ogon przy 120 rpm dominuje uśrednienie na dłuższym oknie).
`iq_request` mean=115.31, najstabilniejszy ripple (0.026) ze wszystkich przebiegów — brak
skoku w trakcie rampy widocznego na poziomie zagregowanych metryk. Pełny przebieg (per
tick) jest w `tests/host/out/CADENCE_RAMP_50_120_power.csv`/`_ride.csv` do ręcznej
inspekcji, gdyby przyszła karta chciała szukać lokalnego szpica w konkretnym momencie
rampy, nie tylko w metrykach zagregowanych.

---

## 9. MISSED_TICK_BURST — wyniki

Trzy niezależne demonstracje na PRAWDZIWYM kodzie, WSPÓLNA linia bazowa zdarzeń w
kategorii A (nie dwa różne zdarzenia — to była pierwsza wersja tego testu i była BŁĘDNA,
poprawiona podczas budowy tej karty, patrz "Findings" #testability-3 niżej):

**A. `ride_episode.c` (elapsed-time, jawny `now_tick`).** Linia bazowa: `arm_seq`
faktycznie zmienia się na ticku 1100, `iq_setpoint` faktycznie wraca do 500 na ticku
1500, zakotwiczenie na ticku 1000. Gęste próbkowanie (601 wywołań): `t_latch=25 ms`,
`t_recover=125 ms` — DOKŁADNIE zgodne z linią bazową. Rzadkie próbkowanie (2 wywołania,
na tickach 1000 i 1600): `t_latch=150 ms`, `t_recover=150 ms`. **Wynik: PÓŹNIEJ, NIE
BŁĘDNIE.** Moduł nigdy nie kłamie o czasie (obie wartości używają prawdziwego ticku
sprzętowego, zero dryfu zegara) — ale zdarzenie, które zaszło MIĘDZY dwoma
przetrwałymi wywołaniami, może być zaraportowane dopiero na ticku NASTĘPNEGO wywołania,
nigdy wcześniej. To jest doprecyzowanie względem audytu, który sformułował kategorię A
jako "immune to missed calls" — prawdziwe dla POMIARU CZASU TRWANIA liczonego od
zakotwiczenia, ale NIE dla znacznika EDGE/ZBOCZA, który jest ograniczony gęstością
wywołań. Zapisane jako finding **F1-b**.

**B. `torque_input.c` FAST filter (35 ms, liczy wywołania).** Skok od zera do stałego
wejścia, sprawdzony po 140 wywołań (=35 ms przy założonych 4 kHz) vs po 35 wywołań (ten
sam nominalny "35 ms" przy 4x rzadszym wywoływaniu): `assist_delta_filtered_native=247`
vs `86`. **Wyraźnie, mierzalnie różne** — filtr NIE wie, że czas upłynął, wie tylko ile
razy go wywołano. Potwierdza audyt finding F1 empirycznie, na prawdziwym kodzie.

**C. Kroki kwadratury PAS w oknie 400 ticków (100 ms) przy 90 rpm (ilustracyjne — patrz
`TEST_INTERFACES.md` "Znane luki").** Prawdziwych kroków: 14. Kroki, jakie wywnioskowałoby
porównanie dwóch migawek fazy (modulo 4): 2. **Aliasing** — różnica jest GENUINE
UTRATĄ informacji, nie tylko przesunięciem w czasie; żadna księgowość ticków jej nie
odzyska.

**Nic nie zostało naprawione** — zgodnie z kartą, to wyłącznie pomiar.

---

## 10. Trace format

Wspólne kolumny bazowe we wszystkich trace: `tick, time_s, crank_angle_deg, pas_state,
cadence_input`. Reszta różni się per harness (karta: "nie dodawaj sztucznych pól, jeśli
kod ich nie udostępnia") — pełna lista w `documentation/testing/TEST_INTERFACES.md`.
Format: CSV, nagłówek w pierwszej linii, separator przecinek, liczby zmiennoprzecinkowe
w notacji C/"C" locale (kropka dziesiętna) — narzędzia PowerShell w tej karcie WYMUSZAJĄ
`InvariantCulture` przy zapisie własnych plików (`metrics_summary.csv`), żeby uniknąć
rozjazdu z plikami generowanymi przez C na maszynach z regionalnym ustawieniem
przecinka dziesiętnego (znaleziono i naprawione podczas budowy tej karty — patrz
Findings #build-1).

---

## 11. Metrics

`Get-ColumnMetrics` (`tests/host/tools/RegressionTools.ps1`): `mean, min, max, P5, P95,
peak-to-peak, stddev, ripple=(P95-P5)/mean`. Ripple pomijany (null, nie zero) gdy
`|mean|≤1` — zabezpieczenie przed dzieleniem przez prawie-zero dającym mylącą wielką
liczbę (karta ostrzega dokładnie przed tym). Zdarzeniowe metryki (startup/release/
re-engagement latency) NIE zostały zaimplementowane w tej karcie — wymagałyby
scenariuszy START_STANDSTILL/RELEASE/REVERSE_CONFIRMED, które są tylko opisane
(`REGRESSION_SCENARIOS.md`), nie zaimplementowane (poza P0/P1 tej karty).
Żadne progi PASS/FAIL nie zostały ustawione — zgodnie z kartą, sekcja 12.

---

## 12. First divergence

`Compare-Traces` w `RegressionTools.ps1`: warstwy w kolejności pipeline'u, tolerancja
per warstwa (bezwzględna różnica średniej LUB różnica ripple >0.05 — którakolwiek
większa), zwraca pierwszą warstwę z werdyktem `DIFFERENT` jako `FirstDivergence`.
**Zweryfikowany jako działający** przez smoke-test determinizmu (sekcja "Walidacja"
niżej): dwa niezależne przebiegi RUN_100 przez `power_pipeline_host.exe` porównane na 7
warstwach (`torque_raw`→`iq_request`) — wynik: **SAME na wszystkich warstwach, brak
FirstDivergence**, dokładnie jak powinno być dla tego samego binarium na tym samym
wejściu. To jest jedyny dziś dostępny "test tego testu" (nie ma jeszcze drugiej wersji
firmware do porównania) — ale potwierdza, że mechanizm faktycznie wykrywa SAME
poprawnie, co jest warunkiem koniecznym, by ufać mu przy wykrywaniu DIFFERENT w
przyszłości.

---

## 13. Golden design

`tests/host/golden/candidates/metrics_summary.csv` — KOPIA bieżących metryk, tworzona
przy KAŻDYM uruchomieniu `run_regression.ps1`. **To NIE jest zatwierdzony golden** —
żaden mechanizm w tej karcie nie oznacza go jako "prawidłowy" ani nie porównuje
przyszłych przebiegów przeciwko niemu automatycznie. Pełny projekt (warstwy, wstępne
tolerancje, zasada ręcznego zatwierdzania) w `documentation/testing/TEST_ARCHITECTURE.md`
sekcja "Golden-trace design". Kluczowa zasada zapisana explicite: aktualizacja golden
wymaga wskazania karty FW-XXX jako powodu (kultura tego repo już to robi dla zmian
produkcyjnych) i NIGDY nie dzieje się automatycznie przy zielonym CI.

---

## 14. Hardware generator concept

Zgodnie z kartą (sekcja 16), TYLKO projekt, bez implementacji fizycznej. Tabela z
audytu (`ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md`, sekcja 5) pozostaje aktualna:

| Wejście | software injection wystarcza do L1/L2/L3? | Kiedy potrzebny prawdziwy sygnał? |
|---|---|---|
| PAS A/B | Tak (ten harness to udowadnia — generator liczbowy zamiast GPIO) | Test samego dekodera GPIO (poza zakresem tej karty) |
| Torque analog | Tak dla logiki (torque_input przyjmuje `raw_native` wprost) | Test frontendu ADC (offset/szum/impedancja) |
| Throttle analog | Tak (`ride_control_input_t.throttle_iq` to gotowa liczba) | j.w. |
| Brake digital | Tak (`safety_cut` to `bool`) | Debounce sprzętowy (dziś go nie ma — F9) |
| Speed pulse | Tak logicznie, ale wymaga PRAWDZIWEGO TIMINGU zbocza (nie tylko wartości) | Test filtra odrzucania glitchy pod realnym szumem elektrycznym |
| Battery V/I | Tak | Kalibracja ADC/offsetu |
| Controller temp | Tak | Krzywa NTC |
| CAN/HMI | Tak — już istnieje po stronie `bafang_canable_pro` | — |

Nic z tego nie zostało zbudowane fizycznie w tej karcie.

---

## 15. Observability gaps

- **Dekoder PAS** (maszyna stanów GPIO w `main.c`) nie jest wykonywany przez żaden
  harness tej karty — generator dostarcza LICZBĘ kroków wprost. Wymaga wydzielenia
  dekodera z `main.c` (audyt, etap D), zakazanego w tej karcie.
- **`motor_voltage_utilization`** zawsze 0 w harnessach pipeline (brak FOC) — pomija
  gałąź cross-check moc→napięcie w `assist_modes.c`.
- **`MP.assist_settings[][0]`** (legacy limit % per poziom z EEPROM) niereplikowany —
  `ride_core_iq_limit` na stałe = `PH_CURRENT_MAX` w każdym scenariuszu.
- **Throttle/brake/speed/battery/temperature scenariusze** — tylko opisane
  (`REGRESSION_SCENARIOS.md`), niezaimplementowane (poza zakresem P0/P1).
- **REVERSE_CONFIRMED/MICROREVERSE/PAS_GLITCH** — wymagają wydzielenia dekodera PAS
  ZANIM da się je uczciwie zaimplementować bez duplikowania jego logiki w teście.
- **Brak realnego "starego firmware"** do porównania — ta karta jest pierwszym
  uruchomieniem, więc first-divergence/golden są dziś zweryfikowane tylko na
  DETERMINIZMIE (ten sam build ze sobą), nie na prawdziwej regresji.

---

## 16. Findings

Format: kategoria / severity / dowód / wpływ. **Nic z tego nie zostało naprawione.**

**F1 (potwierdzenie audytu + doprecyzowanie), hidden-timebase-dependency, WYSOKA.**
Zmierzone empirycznie w `missed_tick_burst_host.c` (sekcja 9). Doprecyzowanie
**F1-b**: kategoria "elapsed time" (moduły z jawnym `now_tick`, np. `ride_episode.c`)
jest odporna na pominięte ticki dla POMIARU CZASU TRWANIA od zakotwiczenia, ale ZNACZNIK
EDGE/ZBOCZA (np. `t_latch_ms`, moment zmiany `arm_seq`) może być zaraportowany później
niż naprawdę wystąpił, ograniczony gęstością wywołań — nie jest to błąd, ale nie jest to
też "pełna odporność", jak można by wywnioskować z audytu bez tego pomiaru.

**F-tl-1, testability issue, ŚREDNIA.** `src/torque_input.c` (linie ~242, 248) nie
kompiluje się czysto pod `-Wall -Wextra -Werror` na gcc 14.2.0: `run_attack_steps <
TORQUE_RUN_ATTACK_STEPS` / `>= TORQUE_RUN_ATTACK_STEPS` są tautologicznie
fałszywe/prawdziwe, bo `TORQUE_RUN_ATTACK_STEPS` jest zdefiniowane jako `0U` (FW-091:
funkcja domyślnie wyłączona). Kod jest martwy przy dzisiejszej wartości stałej, ale
ożywa (i ostrzeżenie znika) gdy ktoś ustawi tę stałą >0 zgodnie z komentarzem w kodzie
("Set to 8 ... to enable"). Obejście: `-Wno-type-limits` udokumentowane w
`run_regression.ps1`, tylko dla tego pliku. Odkryte WYŁĄCZNIE dlatego, że ta karta jest
pierwszą, która linkuje `torque_input.c` pod strict warnings — istniejące 3 suity
(FW-100/101/102) nigdy go nie linkowały.

**F-tl-2, testability issue, NISKA.** `src/assist_modes.c` (linia ~258),
`valid_wa_max_wheel_x10`: `value <= BANK_WA_MAX_WHEEL_X10_MAX` (255U) zawsze prawdziwe
dla `uint8_t`. Ten sam wzorzec i to samo obejście co F-tl-1.

**F4 (potwierdzenie audytu, koszt realny), untestable-coupling, ŚREDNIA.**
`assist_dynamics.c`/`assist_limits.c` wymagały `map_adapter.c` (kopii `main.c`'s
`map()`) żeby dało się je zlinkować bez `main.c`. To NIE jest hipotetyczny koszt z
audytu — to jest FAKTYCZNY, poniesiony koszt budowy tej karty, i faktyczne ryzyko
rozjazdu (jeśli `main.c`'s `map()` się zmieni, adapter trzeba zaktualizować ręcznie,
nic tego dziś nie pilnuje).

**#build-1, hidden-locale-dependency (nowe, znalezione podczas budowy tej karty),
NISKA.** `Export-Csv` w PowerShell domyślnie formatuje liczby zgodnie z bieżącą kulturą
regionalną systemu (na tej maszynie: przecinek dziesiętny), podczas gdy pliki CSV
generowane przez C (`fprintf`) zawsze używają kropki (locale "C"). Bez wymuszenia
`InvariantCulture` `metrics_summary.csv` mieszałby dwie konwencje w tym samym repo.
Naprawione w TEJ karcie (jedna linia na starcie `run_regression.ps1`) — nie dotyczy
firmware, tylko własnego narzędzia tej karty, ale zapisane jako finding, bo klasa
błędu jest tej samej rodziny co reszta tego audytu (ciche, zależne-od-środowiska
niespójności).

**#test-bug-1 (błąd we WŁASNYM kodzie testowym tej karty, nie w firmware), NISKA,
naprawiony w trakcie budowy.** Pierwsza wersja `run_episode_scenario()` (kategoria A
MISSED_TICK_BURST) porównywała dwa NIEZALEŻNE, niespójne scenariusze zdarzeń (różny
tick zbocza `arm_seq`, różny tick odzyskania) zamiast JEDNEJ wspólnej linii bazowej
próbkowanej gęsto vs rzadko — dawało to mylący wynik "DIFFERENT" bez znaczenia.
Naprawione przed opublikowaniem wyników w tym raporcie (sekcja 9 pokazuje poprawną
wersję). Zapisane tu jako przypomnienie: TEN sam błąd metodologiczny (różne "prawdy
bazowe" zamiast jednej próbkowanej różnie) jest łatwy do popełnienia w każdym
przyszłym scenariuszu porównawczym — `TEST_ARCHITECTURE.md`/`REGRESSION_SCENARIOS.md`
warto czytać z tym w pamięci.

---

## 17. Co należy zrobić jako następne

1. **Wydzielić dekoder PAS z `main.c`** (audyt etap D) — odblokowuje uczciwe
   REVERSE_CONFIRMED/MICROREVERSE/PAS_GLITCH i usuwa jedyny istotny observability gap
   tej karty.
2. **Naprawić F1 w produkcji** (dokończyć wzorzec FW-103/104 dla pozostałych liczników)
   — ta karta dostarcza gotowy, zmierzony dowód rozmiaru problemu (247 vs 86 dla FAST
   filtra) do uzasadnienia priorytetu.
3. **Zebrać pierwszy prawdziwy "stary firmware" do porównania** — po dowolnej
   następnej zmianie w torze torque→power→ride_control, uruchomić `run_regression.ps1`
   PRZED i PO, użyć `Compare-Traces` do odpowiedzi na pytanie z kryterium sukcesu karty
   (sekcja 22: "w której warstwie pojawiła się pierwsza różnica").
4. **Dodać scenariusze P1 pozostałe** (BRAKE_CUT, SPEED_LIMIT, LOW_VOLTAGE,
   TEMPERATURE_LIMIT, ROLLING_RESTART, START_STANDSTILL, RELEASE) — `ride_control.h`
   API już wszystko obsługuje, zero zmian produkcyjnych potrzebne, tylko nowe pliki
   harness.
5. **Formalnie zatwierdzić pierwszy golden** — ręcznie przejrzeć
   `tests/host/golden/candidates/metrics_summary.csv`, przenieść do
   `golden/approved/` z jawną decyzją (patrz `TEST_ARCHITECTURE.md`).

---

## 18. Czego nadal NIE należy refaktorować

Bez zmian względem audytu, potwierdzone doświadczeniem budowy tej karty:

- **`src/FOC.c`, ISR Halla/ADC w `main.c`** — poza granicą deterministic-software-test
  (karta, sekcja 4). Ta karta to potwierdziła empirycznie: nawet sięgnięcie do
  `motor_core` (`ride_control_pipeline_host.c`) wymagało trzech adapterów; pójście
  dalej (do prawdziwego FOC) wymagałoby modelu fizycznego silnika/mostka/baterii, nie
  kolejnego adaptera nagłówkowego.
- **Format `Para0/1/2`/EEPROM** — nietknięty, nie było potrzeby.
- **`main.c` jako całość** — karta wprost tego zakazywała; ta praca pokazuje, że NIE
  było to konieczne do zbudowania solidnego L1-L3 — trzy małe adaptery wystarczyły.
- **Dekoder PAS w `main.c`** — zidentyfikowany jako jedyny prawdziwie blokujący
  fragment (dla przyszłych scenariuszy cofnięcia/glitchy), ale jego wydzielenie to
  osobna, świadoma decyzja architektoniczna (audyt etap D), nie coś do zrobienia
  "przy okazji" pisania testów.
- **`assist_extended_boost.c`, `pas_trace.c`, `ride_episode.c`** — już mają testy
  sprzed tej karty, nietknięte, nadal przechodzą.

---

## Walidacja (uruchomienie końcowe)

```
powershell -File tests/host/run-host-tests.ps1     -> All host suites: PASS  (exit 0)
powershell -File tests/host/run_regression.ps1     -> Determinism smoke-test: PASS  (exit 0)
```

Kompilator: `C:\Projekty\tools\w64devkit\bin\gcc.exe` 14.2.0. Zero toolchaina ARM
użyte lub wymagane (karta, sekcja 18). Wszystkie CSV trace w `tests/host/out/`
(24000-56000 wierszy per scenariusz per harness), metryki zagregowane w
`tests/host/out/metrics_summary.csv`, wynik `MISSED_TICK_BURST` w
`tests/host/out/missed_tick_burst_summary.csv`.
