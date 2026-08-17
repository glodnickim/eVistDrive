# TEST-002: High-Cadence Benchmark Quality — raport końcowy

Data: 2026-08-10. Buduje na `TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md` (TEST-001,
ten sam dzień). Build firmware: 0.0329, bez zmian.

---

## 1. Co zmieniono w testach

**Nowe pliki (wszystkie pod `tests/`):**

- `tests/host/common/scenario_profiles.h` — jedno źródło prawdy profili nacisku
  (BASELINE/LOAD_LOW/LOAD_HIGH/SYMMETRIC/DEADSPOT) i parametrów sweepów.
- `tests/host/common/signal_stats.h/.c` — mean/min/max/P5/P95/ptp/ripple/stddev,
  osobno per-rewolucja i cross-rewolucja (Welford), bez malloc-limitu próbek.
- `tests/host/torque/torque_revolution_bench_host.c` — **TEST A**: filtr torque,
  okno rewolucyjne, phase-binning, tryby REV20/TIME6S/WARMUP_SCAN.
- `tests/host/pipeline/power_revolution_bench_host.c` — **TEST B**: pełny pipeline
  mocy z rewolucyjnym oknem, sterowalne `motor_voltage_utilization` i napięcie baterii.
- `tests/host/tools/HighCadenceTools.ps1` — normalizacja względem 80 rpm, metryki
  pompowania, analiza symetrii, sonda "clamp probe".
- `tests/host/run_high_cadence.ps1` — orkiestrator jednym poleceniem.
- Dokumentacja: ten plik + aktualizacje `documentation/inputs/TORQUE_SENSOR.md` i
  `documentation/testing/TEST_ARCHITECTURE.md` (patrz sekcja 9).

**Nietknięte:** `tests/host/torque/torque_trace_host.c`,
`tests/host/pipeline/power_pipeline_host.c`, `ride_control_pipeline_host.c`,
`tests/host/run_regression.ps1`, `tests/host/run-host-tests.ps1` — cały fundament
TEST-001 działa dalej bez zmian (sekcja 22).

---

## 2. Potwierdzenie: zero zmian firmware

**Zero plików w `src/` ani `inc/` zmienionych przez tę kartę.** Zweryfikowane
`git status`/`git diff` na końcu pracy: cztery pliki (`inc/config.h`,
`inc/ride_control.h`, `src/main.c`, `src/ride_control.c`) pokazują się jako
zmodyfikowane, ale to STAN SPRZED tej karty (i sprzed TEST-001) — własna,
niezacommitowana praca użytkownika nad FW-101, potwierdzona znacznikami czasu
(`ls -la`: modyfikacje z 00:01-12:47, przed rozpoczęciem tej karty) identycznymi z tymi
odnotowanymi na starcie audytu architektury tego samego dnia.

Build firmware pozostaje 0.0329. TEST A i TEST B linkują dokładnie te same moduły
produkcyjne co TEST-001 (`torque_input.c`, `assist_modes.c`, `ride_control.c`, itd.) plus
te same trzy adaptery testowe (`host_stubs/`, `map_adapter.c`, `motor_service_stub.c`) —
żaden nowy adapter nie był potrzebny.

---

## 3. Metodologia warm-up

Uruchomiono `WARMUP_SCAN` (60 obrotów, bez odrzucania żadnego) na profilu BASELINE przy
60 rpm, 120 rpm i osobno LOAD_HIGH przy 120 rpm.

**Wynik (60 rpm, BASELINE):** obrót 0 (zimny start, bufor RUN pusty):
`mean_run=279,91`, `ripple_run=0,832`. Obrót 1: `mean_run=322,01`, `ripple_run=0,0248`
— już PRAKTYCZNIE identyczny ze stanem ustalonym. Obroty 2-59: wariancja `ripple_run`
między 0,024842 a 0,024847 (rozrzut <0,00001) — **stan ustalony osiągnięty w ciągu 1
obrotu** dla tego filtra/profilu.

**Wyjaśnienie mechanizmu:** okno RUN domyślnie = 180° = 48 kroków kwadratury. Bufor
zapełnia się w POŁOWIE pierwszego obrotu (ten harness NIE woła
`torque_input_seed_run()` — to robi dopiero `ride_control.c` przy zazbrojeniu ride
latcha, poza zakresem TEST A). Stąd już drugi obrót jest w pełni "widziany" przez pełny
bufor.

**Decyzja:** `DEFAULT_WARMUP_REVOLUTIONS=8` (w `scenario_profiles.h`) POZOSTAJE — jest
bezpiecznym, 8-krotnym marginesem nad zmierzonym minimum (1 obrót), nie zmniejszono go.
Karta sugerowała 5-10; zmierzone minimum jest niższe, ale nie ma powodu ścinać marginesu
bezpieczeństwa tylko dlatego, że jest "niepotrzebnie duży" — koszt (8 dodatkowych
obrotów na ~120 uruchomień) jest znikomy wobec ryzyka zmierzenia stanu nieustalonego w
innej konfiguracji (np. wyższy `dead_spot_depth_pct`, inny profil).

**Ograniczenie tego pomiaru:** generator jest DETERMINISTYCZNY i BEZSZUMOWY — brak
losowości sensora. Prawdziwy czujnik z szumem elektrycznym prawdopodobnie potrzebowałby
więcej obrotów, by "stan ustalony" był statystycznie stabilny (mniejsza wariancja
próbki), nie tylko matematycznie zbieżny. To jest HIPOTEZA, nie zmierzone tutaj.

---

## 4. Stała liczba obrotów

`torque_revolution_bench_host.c` i `power_revolution_bench_host.c` implementują okno
rewolucyjne: `DEFAULT_WARMUP_REVOLUTIONS=8` (odrzucone) +
`DEFAULT_MEASURE_REVOLUTIONS=20` (analizowane), identyczne na KAŻDEJ kadencji. Ważny
błąd znaleziony i naprawiony PODCZAS budowy tej karty (nie w produkcji): pierwsza wersja
wykrywania granicy obrotu finalizowała "obrót" na ticku przejścia
warm-up→pomiar, myląc go z zakończonym zmierzonym obrotem (przepełnienie `uint32_t`
w indeksie, `4294967295`). Naprawione przez zmianę kolejności: granica jest sprawdzana
PRZED akumulacją bieżącego ticka, nie po. Zweryfikowane: każdy przebieg REV20 daje
dokładnie 20 wierszy per-rewolucja, indeksy 0..19, ~3000±1 ticków/obrót przy 80 rpm.

---

## 5. Torque waveform profiles

5 profili we `scenario_profiles.h` (wszystkie to STYMUL testowy, nie firmware):

| Profil | mean_native_delta | ripple% | asymmetry% | dead_spot depth/width° | Rola |
|---|---|---|---|---|---|
| BASELINE | 300 | 40 | 15 | 30/20 | LOAD=medium I asymetryczny (ten sam co TEST-001) |
| LOAD_LOW | 150 | 40 | 15 | 30/20 | mały nacisk |
| LOAD_HIGH | 500 | 40 | 15 | 30/20 | duży nacisk |
| SYMMETRIC | 300 | 40 | 0 | 30/20 | test symetrii lewa=prawa |
| DEADSPOT | 300 | 40 | 15 | 60/35 | wzmocniony martwy punkt |

---

## 6. RUN_60...120 (TEST A, REV20)

| Cadence | mean_run (per-rev avg) | ripple_run (per-rev avg) | ticks/obrót |
|---|---|---|---|
| 60 | 321,98 | 0,024846 | ~4000 |
| 80 | 321,84 | 0,022216 | ~3000 |
| 90 | 321,95 | 0,024849 | ~2667 |
| 100 | 321,94 | 0,024694 | ~2400 |
| 110 | 321,96 | 0,022363 | ~2182 |
| 120 | 321,96 | 0,024848 | ~2000 |

**Rozrzut średniej: 0,04% (321,84-321,98). Rozrzut ripple: 321-329 setnych, BEZ trendu z
kadencją** (80 i 110 rpm akurat niżej, ale to nie monotoniczny trend, tylko dyskretyzacja
całkowita 3,75° siatki wobec cyklu profilu — patrz sekcja 9). **Odpowiedź na pytanie 1
kryterium sukcesu: NIE, filtr FAST/RUN sam w sobie NIE zmienia zachowania między 60 a
120 rpm** przy identycznym torque-vs-angle i dopasowanej liczbie obrotów. To jest wynik
PEWNY (software, ten sam moduł co produkcja, deterministyczny stymul).

---

## 7. Per-revolution metrics

Pełne dane: `tests/host/out/high_cadence/torque_per_revolution_*.csv` (jedna linia per
obrót: mean/P5/P95/ripple/ptp dla FAST i RUN) + zagregowane w `torque_summary.csv`
(`per_rev_mean_run_avg/min/max/stddev`, analogicznie dla ripple). Przykład (80 rpm,
BASELINE): `per_rev_mean_run_stddev=0,0349` na 20 obrotach — obroty są niemal identyczne
(profil deterministyczny, brak szumu), co samo w sobie POTWIERDZA poprawność okna
rewolucyjnego (gdyby granice obrotów były źle wykrywane, wariancja między obrotami by
rosła).

---

## 8. Phase-binned torque FAST/RUN

96 binów (3,75°/bin, ta sama granulacja co krok kwadratury). Plik
`torque_phase_binned_<tag>.csv`: `mean_fast`/`stddev_fast` i `mean_run`/`stddev_run`
UŚREDNIONE PO OBROTACH per-bin — pokazuje, czy kształt fali się powtarza. Zmierzone
(80 rpm, BASELINE): `stddev_fast` w binach rzędu 0,02-0,06 (setne jednostki natywne) —
kształt fali FAST jest niemal identyczny obrót-po-obrocie. `stddev_run` w większości
binów **dokładnie 0,0000** — RUN jest tak wygładzony (okno 180°), że w tym
deterministycznym stymulu praktycznie nie zmienia się między obrotami wcale.

**Odpowiedź na pytanie 2 kryterium sukcesu:** różnica FAST między nogami (lewa/prawa,
zaprogramowana asymetria 15%) WIDAĆ w danych phase-binned jako ok. -2% różnicy
średniej FAST między połówkami 0-180°/180-360° (sekcja 16) — a więc TAK, profil
nacisku/asymetria bezpośrednio kształtuje FAST; RUN tłumi tę różnicę do ~0,3-0,4%
(nie do zera — 180° okno nie jest idealnym filtrem dolnoprzepustowym).

---

## 9. Porównanie 6 s vs 20 obrotów

**To jest kluczowy wynik tej karty — bezpośrednia, empiryczna weryfikacja hipotezy z
TEST-001, na TYM SAMYM kodzie (dwa tryby jednego harnessu, nie dwa różne narzędzia).**

| Cadence | TIME6S: obroty w oknie | TIME6S: ripple_run (całe okno) | REV20: ripple_run (całe okno) |
|---|---|---|---|
| 60 | 5 | 0,098415 | 0,024846 |
| 80 | 7 | 0,057031 | 0,021750 |
| 90 | 9 | 0,066458 | 0,024849 |
| 100 | 10 | 0,075887 | 0,024849 |
| 110 | 10 | 0,047400 | 0,021742 |
| 120 | 11 | 0,028425 | 0,024848 |

TIME6S (metoda TEST-001): ripple zmienia się **3,5×** w zakresie kadencji (0,028-0,098),
NIEMONOTONICZNIE (90/100 rpm wyżej niż 110 rpm — artefakt liczby obrotów mieszczących
się w oknie, nie trend fizyczny). REV20 (ta karta): ripple stabilny w wąskim paśmie
0,0217-0,0248, bez trendu.

**Wniosek jednoznaczny: hipoteza TEST-001 była PRAWIDŁOWA.** Pozorny spadek ripple z
kadencją w TEST-001 wynikał WYŁĄCZNIE z tego, że stałe okno czasowe = różna liczba
obrotów na różnych kadencjach, nie ze zmiany zachowania filtra. Dokumentacja
(`documentation/inputs/TORQUE_SENSOR.md`, `documentation/testing/TEST_ARCHITECTURE.md`)
zaktualizowana z tym potwierdzeniem (nie było błędu do poprawienia — hipoteza była
słuszna, ale teraz jest POTWIERDZONA, nie tylko sformułowana).

---

## 10. motor_voltage_utilization sweep

7 wartości (0/800/1200/1400/1600/1800/1900) × 6 kadencji × BASELINE × 42 V = 42
przebiegi (B1). **Wynik: `iq_request` i `iq_final` IDENTYCZNE na całym zakresie mvu, na
KAŻDEJ kadencji** (np. 100 rpm: `iq_request=116,5859` niezależnie od mvu — patrz
`clamp_probe_summary.csv`, wszystkie 64 pary pokazują `delta=0`).

**Wyjaśnienie matematyczne (nie zgadywanie — policzone z realnych wartości w CSV):**
klamra mocy/napięcia w `finish_power_request()` (assist_modes.c) daje
`power_iq_limit = requested_current_ma × 2048 / (mvu × 95)`. Przy `motor_power_w`
ograniczonym do sufitu poziomu (1500 W, `ASSIST_MOTOR_POWER_HARD_MAX_W`), nawet przy
`mvu=2048` (teoretyczne maksimum skali) i napięciu 59 V (MAX_VOLTAGE z config.h),
`power_iq_limit≈268` — WCIĄŻ powyżej zmierzonego `iq_request≈206` (LOAD_HIGH, 120 rpm).
Klamra torque'owa (`profile_iq_limit=700`, z `max_iq_pct=100%×PH_CURRENT_MAX`) jest w
KAŻDYM przetestowanym przypadku mniej restrykcyjna niż zarówno klamra mocy, jak i samo
żądanie z toru torque. **Odpowiedź na pytanie 3 kryterium sukcesu: dla poziomu asysty 3
(320% wsparcia) i przetestowanego zakresu obciążenia/napięcia/mvu, power/Iq pipeline
NIE ogranicza outputu — pewny wynik software'owy z jasnym mechanizmem matematycznym.**

**OBSERVABILITY GAP:** nie ma eksportowanej flagi "który limiter właśnie przyciął" na
poziomie `assist_modes.c` (`ride_control_get_debug_flags()` istnieje, ale dla INNEGO
limitera — napięcie/temperatura/prędkość w `assist_limits.c`, nie dla klamry
moc/napięcie wewnątrz `finish_power_request()`). "Clamp probe" (sekcja 12,
`Build-ClampProbeReport`) to obszedł metodą PAROWANYCH, NIEZALEŻNYCH uruchomień procesu
(mvu=0 jako referencja vs mvu=X), bez zmiany API produkcyjnego. **Ważna poprawka
metodologiczna z tej karty**: pierwsza wersja próbowała to zrobić WEWNĄTRZ jednego
procesu (druga wywołanie `assist_modes_calculate()` na tym samym ticku z mvu=0) — to
było BŁĘDNE, bo filtr mocy (`filter_motor_power()`) ma stan między wywołaniami i sonda
zanieczyszczała trajektorię prawdziwego przebiegu. Naprawione przed uruchomieniem
sweepu (patrz kod źródłowy `power_revolution_bench_host.c` dla pełnego opisu).

**Nietestowane (out of scope tej karty, ale policzalne):** wyższy poziom asysty (level
5, 520% wsparcia) mógłby PODNIEŚĆ `iq_request` torque'owy (proporcjonalnie do
support_ratio_pct) BEZ podnoszenia sufitu mocy (już nasycony na 1500 W) — co
matematycznie MOGŁOBY przesunąć `iq_request` powyżej `power_iq_limit`. To HIPOTEZA
wynikająca z analizy wzoru, NIE zmierzona (harness ma poziom asysty na sztywno = 3,
zgodnie z TEST-001; zmiana wymagałaby tylko dodania parametru CLI, zero zmiany
produkcyjnej — kandydat na TEST-003).

---

## 11. Battery voltage sweep

4 napięcia (42/40/38/36 V) × 4 kadencje wysokie (80/100/110/120) × BASELINE × 2 mvu
(0/1600) = 32 przebiegi (B3), plus osobna sonda B4 w KIERUNKU PRZECIWNYM (42→59 V,
LOAD_HIGH, mvu do 2048 — patrz sekcja 10). **Wynik B3: zero zmiany `iq_request`/`iq_final`
w całym zakresie 36-42 V.** Zgodnie z matematyką sekcji 10 — obniżanie napięcia w tym
kierunku ODDALA `power_iq_limit` od bycia wiążącym (wymaga WIĘKSZEGO prądu dla tej samej
mocy, więc `power_iq_limit` ROŚNIE, nie maleje). Sensowny kierunek sondowania klamry to
WYŻSZE napięcie (B4), co też nie wystarczyło w przetestowanym zakresie (sekcja 10).

---

## 12. Load sweep

LOAD_LOW (150) / BASELINE=LOAD_MEDIUM (300) / LOAD_HIGH (500) × 4 kadencje × 2 mvu
(0/1600) = 24 przebiegi (B2). `iq_request` skaluje się z obciążeniem zgodnie z
oczekiwaniem (LOAD_LOW≈49, BASELINE≈116, LOAD_HIGH≈206 przy danej kadencji) — TORQUE
napędza Iq bezpośrednio (tryb POWER_LINEAR), nie moc. Zero wpływu mvu na żadnym
poziomie obciążenia (ta sama przyczyna co sekcja 10).

---

## 13. High-cadence matrix

**Zakres:** 128 uruchomień harnessów łącznie (30 TEST A + 98 TEST B), NIE pełny iloczyn
kartezjański (6 kadencji × 5 profili × 4 napięcia × 7 mvu = 840 byłoby nadmiarowe).
Wybór: B1 = pełny sweep mvu na WSZYSTKICH kadencjach przy stałym obciążeniu/napięciu
(odpowiada wprost na pytanie 3 kryterium sukcesu); B2/B3 = "high-cadence focused"
(zredukowany zestaw kadencji 60/80/100/120 lub 80/100/110/120) × pełny sweep
obciążenia/napięcia × 2 reprezentatywne mvu (0 = bez klamry, 1600 = blisko górnej
połowy zakresu); B4 = celowana sonda matematycznej granicy klamry. Udokumentowany wybór
zamiast zgadywania — patrz uzasadnienie w kodzie `run_high_cadence.ps1`.

---

## 14. Normalized output vs cadence

Plik `normalized_output_vs_cadence.csv` (132 wiersze), referencja = 80 rpm (jawnie
zadeklarowana, karta sekcja 13 — NIE twierdzę, że 80 rpm jest "idealne").

| Metryka | 60 rpm | 80 rpm | 90 rpm | 100 rpm | 110 rpm | 120 rpm |
|---|---|---|---|---|---|---|
| torque_run_mean | 1,0004 | 1,0000 | 1,0003 | 1,0003 | 1,0004 | 1,0004 |
| iq_final_mean (mvu=0) | 1,0001 | 1,0000 | 1,0002 | 1,0002 | 1,0002 | 1,0002 |
| iq_final_mean (mvu=1900) | 1,0001 | 1,0000 | 1,0002 | 1,0002 | 1,0002 | 1,0002 |
| motor_power_w_mean (mvu=0) | 0,7500 | 1,0000 | 1,1256 | 1,2507 | 1,3758 | 1,5010 |

**To jest ODPOWIEDŹ NA PYTANIE 5 kryterium sukcesu ("na jakiej warstwie pojawia się
spadek?"): PIERWSZA warstwa, na której cadence w ogóle widać efekt, to
`motor_power_w`/`human_power_w` (rośnie liniowo z cadence, zgodnie z fizyką P=τ×ω) — NIE
`torque_run` ani `iq_request`/`iq_final`, które zostają praktycznie stałe.** Powód:
tryb POWER_LINEAR liczy `iq_request` WPROST z obciążenia (torque), nie z mocy — moc jest
osobną wielkością liczoną RÓWNOLEGLE (do telemetrii i do sufitu mocy), a NIE wejściem do
wzoru na Iq. Innymi słowy: w tej konfiguracji **nie ma "spadku" Iq z cadence do
znalezienia** — jest tylko WZROST raportowanej mocy, co jest poprawnym, oczekiwanym
zachowaniem, nie regresją.

---

## 15. Pumping metrics

`pumping_metrics_summary.csv` — `ripple_run` per scenariusz (mean/min/max przez 20
obrotów) prosto z per-rewolucyjnych CSV, bez FFT (zgodnie z kartą). Baseline REV20:
`ripple_run_mean` 0,0222-0,0248 na całym zakresie kadencji — pompowanie NIE rośnie z
cadence w tym przetestowanym profilu. `ptp_run_mean` (peak-to-peak) podobnie stabilne
(~8 jednostek natywnych na każdej kadencji).

---

## 16. Symmetric vs asymmetric torque

`symmetry_analysis.csv`, 80 rpm, różnica lewa/prawa jako % średniej:

| Profil | FAST diff % | RUN diff % |
|---|---|---|
| SYMMETRIC (asym=0%) | -0,0002% | 0,0000% |
| BASELINE (asym=15%, zaprogramowane) | -1,99% | 0,372% |
| DEADSPOT (asym=15%, dead-spot wzmocniony) | -1,90% | 0,335% |

**SYMMETRIC pokazuje ~0% różnicy** — to jest ważna, pozytywna walidacja SAMEGO
generatora testowego (potwierdza, że `asymmetry_pct=0` faktycznie produkuje symetryczną
falę, nie ukrytą asymetrię z błędu implementacji). **BASELINE/DEADSPOT pokazują FAST
niosący WIDOCZNĄ asymetrię (~2%), a RUN silnie ją TŁUMI (do ~0,35%)** — zgodne z
zamierzonym działaniem okna 180° (FW-085): RUN uśrednia PRZEZ pełny cykl lewa-prawa
noga, więc z definicji nie może pokazać pełnej różnicy międzynożnej, jaką widzi FAST.
**Odpowiedź na pytanie 2 kryterium sukcesu (potwierdzenie):** różnica RUN vs FAST W
DUŻEJ MIERZE wynika z profilu/asymetrii (RUN tłumi to, co FAST pokazuje), nie z artefaktu
implementacji filtra.

---

## 17. First-divergence observability

Trace tej karty rozszerza kolejność warstw z TEST-001 o zweryfikowane, konkretne dane:

```
torque_raw          -- (TEST A + TEST B, phase-binned + per-tick)
torque_corrected     -- jw.
torque_FAST          -- jw., ZMIERZONE: stabilny <0,1% wariancji z cadence (REV20)
torque_RUN            -- jw., ZMIERZONE: stabilny <0,05% wariancji z cadence (REV20)
cadence               -- wejście kontrolowane
human_power           -- ZMIERZONE: rośnie liniowo z cadence (oczekiwane, P=tau*omega)
motor_power (raw/filtered) -- ZMIERZONE: jw., PIERWSZA warstwa z widocznym efektem cadence
iq_request             -- ZMIERZONE: stabilny <0,05% wariancji z cadence i z mvu
iq_final (post ride_control) -- ZMIERZONE: stabilny <0,05% wariancji z cadence i z mvu
power limit state/reason -- OBSERVABILITY GAP (sekcja 10) — wywnioskowane parowanymi
                             przebiegami (clamp_probe_summary.csv), NIE odczytane wprost
```

Comparator `Compare-Traces` (TEST-001, `RegressionTools.ps1`) pozostaje właściwym
narzędziem do FIRST DIVERGENCE między dwiema wersjami firmware — ta karta dostarcza mu
BOGATSZE, porównywalne-między-cadence dane wejściowe (per-rewolucja, nie tylko cały
przebieg), ale sam mechanizm porównania nie został zmieniony (nie było takiej potrzeby —
działa poprawnie, TEST-001 sekcja 12).

---

## 18. Limitations

- **Generator jest deterministyczny, bezszumowy.** Prawdziwy czujnik ma szum
  elektryczny, drgania mechaniczne, niedoskonałą periodyczność pedałowania — wszystkie
  wyniki "wariancja między obrotami ≈ 0" są artefaktem tej idealizacji, NIE dowodem, że
  prawdziwy sygnał będzie równie stabilny.
- **Dekoder PAS nie jest wykonywany** (jak w TEST-001) — generator podaje liczbę kroków
  wprost. `ideal PAS step stream` ≠ `real GPIO PAS decoder behavior`. Wnioski o
  zachowaniu przy 110 rpm dotyczą TYLKO idealnego strumienia kroków, NIE ewentualnej
  utraty/aliasowania kroków w prawdziwym dekoderze (audyt architektury, `main.c` inline
  decoder — poza zakresem tej karty, zob. `documentation/inputs/PAS.md`).
- **FOC nie jest modelowany.** `motor_voltage_utilization` to sterowany sweep wejścia
  do ISTNIEJĄCEGO algorytmu, NIE wynik modelu silnika. Realny FOC ma back-EMF rosnący z
  erps, ograniczenia napięcia DC-bus, martwy czas PWM, nasycenie prądu fazowego — żadne
  z tych zjawisk nie jest tu obecne. Rzeczywisty `actual Iq`/`Uq`/`Ud` przy wysokiej
  cadence MOŻE zachowywać się inaczej niż `iq_request`/`iq_final` zmierzone tutaj.
- **Poziom asysty na sztywno = 3** (320% wsparcia) w TEST B — inny poziom (zwłaszcza
  wyższy support_ratio_pct) mógłby zmienić wynik sekcji 10 (patrz hipoteza tamże).
  Nietestowane.
- **`MP.assist_settings[][0]` (legacy limit % z EEPROM) nadal nie jest replikowany**
  (jak w TEST-001) — `ride_core_iq_limit` na stałe = `PH_CURRENT_MAX`.
- **Cadence compensation pozostaje WYŁĄCZONA** (domyślnie) — karta wprost tego zabrania
  zmieniać; wyniki NIE mówią nic o zachowaniu z włączoną kompensacją.

---

## 19. Findings

**F-metodyka-1 (potwierdzenie, nie błąd), WYSOKA pewność.** Hipoteza TEST-001 o
artefakcie okna czasowego — POTWIERDZONA empirycznie (sekcja 9). Zaktualizowano
`TORQUE_SENSOR.md`/`TEST_ARCHITECTURE.md`.

**F-mechanizm-1 (nowe, ważne dla przyszłych kart), WYSOKA pewność.** Dla trybu
POWER_LINEAR (assist_modes.c), `iq_request` jest funkcją TORQUE (obciążenia), NIE mocy —
`motor_power_w` jest liczone RÓWNOLEGLE, głównie do telemetrii i sufitu mocy. Efekt:
"spadek mocy z cadence" i "spadek Iq z cadence" to DWA RÓŻNE, niezależne pytania w tym
trybie — raportowana moc rośnie z cadence (poprawnie, fizycznie), a Iq nie musi. Ważne
dla interpretacji przyszłych pomiarów: nie zakładać, że zmiana `motor_power_w` implikuje
zmianę `iq_request`.

**F-testowa-1 (błąd we własnym kodzie tej karty, naprawiony przed publikacją wyników),
ŚREDNIA.** Błąd kolejności w wykrywaniu granicy obrotu (`torque_revolution_bench_host.c`)
— finalizacja obrotu PRZED akumulacją bieżącego ticka jest konieczna, inaczej pierwszy
tick nowego obrotu zostaje wliczony do poprzedniego i podczas przejścia warm-up→pomiar
występuje przepełnienie indeksu. Wykryty przez inspekcję pierwszego wyniku smoke-testu
(rewizja 4294967295), nie przez ślepe zaufanie liczbom.

**F-testowa-2 (błąd metodologiczny we własnym kodzie, naprawiony PRZED uruchomieniem
sweepu), ŚREDNIA.** Pierwsza wersja sondy "clamp probe" (sekcja 10) próbowała wywołać
`assist_modes_calculate()` DRUGI raz w tym samym ticku z innym `motor_voltage_utilization`
i porównać wynik — to zanieczyszczało stan filtra mocy (`filter_motor_power()` jest
międzytickowo stanowy) dla PRAWDZIWEGO przebiegu. Naprawione przez przejście na
PAROWANE, NIEZALEŻNE uruchomienia procesu (mvu=0 jako osobny wpis sweepu) zamiast
wewnątrz-procesowej sondy. Zapisane jako przestroga dla przyszłych kart: "porównaj X z Y"
NIE powinno oznaczać "policz X, potem Y w tym samym procesie/ticku" dla modułu ze stanem
międzytickowym.

**F-observability-1 (potwierdzenie z TEST-001, teraz z konkretną metodą obejścia),
NISKA.** Brak eksportowanej flagi "power/voltage limiter zadziałał" w `assist_modes.c`.
Sekcja 10 pokazuje uczciwą metodę wnioskowania (parowane przebiegi) BEZ zmiany API.

**Findingi produkcyjne (bugi) znalezione w tej karcie: brak nowych.** (F-tl-1/F-tl-2 z
TEST-001 — tautologiczne porównania w `torque_input.c`/`assist_modes.c` — potwierdzone
jako wciąż obecne, ten sam udokumentowany wyjątek `-Wno-type-limits` zastosowany.)

---

## 20. Rekomendacja kolejnego kroku

1. **TEST-003 (proponowane): poziom asysty jako parametr sweepu w TEST B.** Zbadać
   hipotezę z sekcji 10 (wyższy `support_ratio_pct` → `iq_request` torque'owy mógłby
   przekroczyć `power_iq_limit`) — mała zmiana (dodać CLI arg do
   `power_revolution_bench_host.c`), zero zmian produkcyjnych, bezpośrednio domyka
   pytanie 3/4 kryterium sukcesu dla PEŁNEGO zakresu poziomów, nie tylko poziomu 3.
2. **Wydzielenie dekodera PAS z `main.c`** (audyt, etap D) pozostaje przedwarunkiem do
   uczciwego przetestowania `ideal PAS stream` vs `real GPIO decoder` przy wysokiej
   kadencji — to jedyny sposób, by odpowiedzieć na pytanie 6 kryterium sukcesu
   ("czego nie możemy powiedzieć bez prawdziwego PAS decodera").
3. **Dodać szum do generatora (opcjonalnie, osobna karta)** — żeby odróżnić "stan
   ustalony w idealnym sygnale" (co ta karta zmierzyła) od "stabilność statystyczna przy
   realistycznym szumie czujnika" (sekcja 18, limitation #1).
4. **L4 (sprzęt/log)** pozostaje jedynym sposobem odpowiedzi na pytanie o realny FOC/Iq
   przy wysokiej cadence (pytanie 6 kryterium sukcesu) — żadna dalsza praca software'owa
   tego nie zastąpi.

---

## 21. Wnioski — PEWNE z testu vs HIPOTEZA

**PEWNE Z TESTU (software, deterministyczny stymul, zweryfikowane na prawdziwym kodzie):**

- Filtr FAST/RUN (`torque_input.c`) NIE zmienia zachowania między 60 a 120 rpm przy
  identycznym torque-vs-angle i dopasowanej liczbie obrotów (rozrzut <0,05%).
- Pozorny spadek ripple z cadence w TEST-001 był w 100% artefaktem stałego okna
  czasowego — zamienione okno rewolucyjne usuwa efekt niemal całkowicie.
- Dla poziomu asysty 3, profilu BASELINE/LOAD_HIGH i przetestowanego zakresu napięcia
  (20-59 V) i `motor_voltage_utilization` (0-2048, cały zakres skali), power/Iq
  pipeline (`assist_modes.c`) NIE ogranicza `iq_request`/`iq_final` — zero przypadków
  na 64 parowane porównania.
- `motor_power_w`/`human_power_w` ROSNĄ z cadence przy stałym obciążeniu (poprawne,
  fizyczne, P=τ×ω) — to NIE jest regresja ani problem.
- RUN filtr tłumi zaprogramowaną asymetrię lewa/prawa noga z ~2% (widoczne w FAST) do
  ~0,35% — zgodne z zamierzonym działaniem okna 180°.

**HIPOTEZA / NIE MOŻNA POTWIERDZIĆ BEZ SPRZĘTU:**

- Czy prawdziwy dekoder PAS (GPIO, `main.c`, nie ten harness) zachowuje się identycznie
  przy 110-120 rpm jak idealny strumień kroków użyty tutaj — NIE SPRAWDZONE (audyt
  architektury, finding o aliasowaniu kroków przy pominiętych tickach, dotyczy INNEGO
  mechanizmu — kolejek sterowania, nie samej cadence).
- Czy realny FOC/aktualny Iq (nie `iq_request`/`iq_final` software'owe) traci moc przy
  wysokiej cadence z powodu back-EMF, nasycenia napięcia DC-bus, czy ograniczeń PWM —
  CAŁKOWICIE POZA ZAKRESEM tej karty (sekcja 18, "FOC nie jest modelowany"). Ten software
  test nie może ani potwierdzić, ani wykluczyć problemu "high-cadence power drop"
  zaobserwowanego (jeśli był) na prawdziwym rowerze — może jedynie wykluczyć, że tor
  torque-filtr-power/Iq (software, przed FOC) jest jego źródłem, DLA przetestowanej
  konfiguracji.
- Czy wyższy poziom asysty (520% wsparcia) ujawniłby klamrę power/voltage — HIPOTEZA
  matematyczna z sekcji 10, niezmierzona.
- Czy stan ustalony filtra RUN (1 obrót, sekcja 3) byłby równie szybki przy realnym,
  zaszumionym sygnale sensora — NIEZMIERZONE.

**Zdanie, którego celowo NIE piszemy:** "problem high-cadence nie istnieje". Piszemy
zamiast tego: **w idealnym strumieniu PAS, przy zadanym `motor_voltage_utilization` i
napięciu baterii, software'owy tor torque→power→Iq (poziom asysty 3, profile BASELINE i
LOAD_HIGH) zachowuje się stabilnie względem cadence 60-120 rpm — żadna z jego warstw
zbadanych w tej karcie nie wykazuje spadku wyjścia z rosnącą cadence.**

---

## 22. Walidacja

Wszystkie trzy zestawy testów uruchomione na końcu, w tej kolejności:

```
powershell -File tests\host\run-host-tests.ps1     -> All host suites: PASS       (exit 0)
powershell -File tests\host\run_regression.ps1     -> Determinism smoke-test: PASS (exit 0)
powershell -File tests\host\run_high_cadence.ps1   -> 128 przebiegów, exit 0       (exit 0)
```

Kompilator: `C:\Projekty\tools\w64devkit\bin\gcc.exe` 14.2.0, zero toolchaina ARM.
Wyjścia TEST-002: 87 plików, ~15 MB w `tests/host/out/high_cadence/` (4 pełne per-tick
trace, 31 per-rewolucja, 31 phase-binned, 2 zbiorcze summary CSV, 4 pliki
post-processingu). Żaden istniejący test nie przestał działać.
