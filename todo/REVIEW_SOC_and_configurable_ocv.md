# Przegląd i plan: SOC baterii, zasięg, konfigurowalna krzywa OCV

Krytyczny przegląd algorytmu SOC/zasięgu w EBICS + plan poprawek. Zasada: „bardziej
rozbudowane ≠ poprawne". Powiązane: [[archive/COMPARISON_SOC_range]] (porównanie z rozwiązaniem referencyjnym).

Pliki/funkcje: `soc_update()` @1 Hz (main.c:2166), `soc_init()` (2122), `calculate_SOC()`
(2000), coulomb w `reg_ADC_processing()` (1481), stałe w `config.h:108-120`.

---

## Werdykt
- **Rdzeń estymacji `soc_real`** (coulomb counting + korekta OCV w spoczynku + detekcja
  doładowania + flash) — **logicznie poprawny.**
- **Prezentacja i pochodne mają realne wady** — jedna poważna (wyświetlany SOC), reszta średnie.

---

## Błędy — wg wagi

### 🔴 POWAŻNY — wyświetlany SOC nie nadąża pod obciążeniem
`SOC_DISP_MAX_STEP = 1.0` = **1%/min** (config.h:114) → `max_step = 1/60 ≈ 0,017%/s`
(main.c:2196). `MS.SOC` (z `soc_display`) zmienia się max o 1%/min.
- **Martwa gałąź:** `if(MS.soc_real<10) step=diff;` (main.c:2197) miało dać szybką zbieżność
  przy rozładowaniu, ale linie 2198-2199 **i tak przycinają `step` do ±max_step** → nie działa.
- `SOC_DISP_GAIN=0.05` nieistotny — dla `diff>0,33` clamp i tak dominuje.

**Ale skala zależy od C-rate (uczciwie):** przy uśrednionym zużyciu wskaźnik zwykle NADĄŻA.

#### Analiza C-rate — bateria 15 Ah, 43 V (≈645 Wh, 1% = 6,45 Wh)
Ciągłe 15 A = 1C → realny spadek **1,67%/min** (>1%/min → lag 0,67%/min). Ale ciągłe 15 A to
podjazd/pełny gaz, nie średnia. Przez **domyślne Wh/km** (`{6,6,7,7,9,9,12,13,16,18}`),
`%/min = wh_km × v_kmh / 387`:

| Poziom | Wh/km | @15 km/h | @20 km/h | @30 km/h |
|---|---|---|---|---|
| 0–1 | 6 | 0,23 | 0,31 | 0,47 |
| 4–5 | 9 | 0,35 | 0,47 | 0,70 |
| 6 | 12 | 0,47 | 0,62 | 0,93 |
| 7 | 13 | 0,50 | 0,67 | **1,01** |
| 8 | 16 | 0,62 | 0,83 | **1,24** |
| 9 | 18 | 0,70 | 0,93 | **1,40** |

Wniosek: przy normalnej jeździe (15–20 km/h) **wszystkie poziomy ≤0,93%/min → wskaźnik nadąża**.
Rozjazd tylko w epizodach dużej mocy (podjazd, sprint, max assist 30+ km/h) i rośnie z C-rate
(15 Ah lag ~0,67%/min; 7 Ah @15 A aż ~2,6%/min).

**Fix:** limit zależny od prądu zamiast sztywnego 1%/min (przy dużym poborze pozwól szybciej
schodzić), oraz naprawa martwej gałęzi `<10%`. Wygładzanie w normalnej jeździe zostaje.

### 🟠 ŚREDNI — niespójna podstawa napięcia w zasięgu
- `remaining_wh = remaining_mah/1000 × system_voltage` — **nominalne** (main.c:2206).
- `used_wh += dmah/1000 × MS.Voltage/1000` — **rzeczywiste** (main.c:2173), i z tego uczone `wh_km`.
- `range = remaining_wh(nominalne) / wh_km(rzeczywiste)` (main.c:2217) → różne podstawy nie skracają się.

Bias: przy **niskim SOC** rzeczywiste napięcie < nominał, więc `remaining_wh` **przeszacowuje**
→ zasięg **zbyt optymistyczny akurat przy słabej baterii**. Przy pełnej — zbyt konserwatywny.
Przykład (3000 mAh, 39 V real vs 43 V nominał, 15 Wh/km): `range=129/15=8,6 km`, realnie
`117/15≈7,8 km` → ~10% za dużo.

**Fix (najlepszy):** liczyć zasięg w **ładunku**, bez napięcia:
`range = remaining_mah / (used_mah_na_km)`. Coulomb i tak daje ładunek → napięcie znika, brak biasu.
Alternatywnie: `remaining_wh` też liczyć z `MS.Voltage` (spójna podstawa).

### 🟠 ŚREDNI — „uczenie pojemności" jest tautologiczne
main.c:2220-2234. `soc_real = remaining_mah/cap_est×100`, a `remaining_mah` maleje o `dmah`, więc
`frac = (soc_start−soc_end)/100 = cycle_discharge_mah/cap_est`, i
`measured = cycle_discharge_mah/frac = cap_est` → **zwraca to, co już masz**. EMA nic nie zmienia.
Jedyny niezależny sygnał to korekta OCV (rzadka, gain 0.02), a start cyklu przy `soc_real>92%`
kotwiczy na **płaskim szczycie** krzywej (OCV niewiarygodne) → adaptacja słaba/biasowana.

**Fix:** uczyć z dwóch punktów SOC liczonych **z napięcia (OCV), nie z coulomba**:
`cap = Δcoulomby / (SOC_ocv_start − SOC_ocv_end)`, oba OCV w spoczynku na **stromej** części
(np. ~80%→~20%). Wtedy licznik i mianownik niezależne → realny pomiar.

### 🟠 ŚREDNI/konfig — krzywa OCV i liczba ogniw (patrz plan niżej)
`cells = system_voltage/3.6` (main.c:2129,2177) + krzywa **zahardkodowana LG M58T (NMC)**
(main.c:2002). Dla LFP lub „dziwnych" napięć SOC z napięcia jest przekłamany (seed, korekta OCV,
detekcja doładowania). Dla typowego NMC działa OK → to brak konfigurowalności, nie błąd logiki.

### 🟡 DROBNY — coulomb zakłada dokładnie 4000 Hz
`soc_mAs_acc += Battery_Current/4000`, okno „1 s” = 4000 ticków (main.c:1481-1482). Realna
częstotliwość ≠ 4 kHz → systematyczny błąd całkowania ładunku. Skalibrować stałą do realnego f.

---

## Priorytety napraw
1. **1%/min → limit zależny od prądu** + naprawa martwej gałęzi `<10%` (najbardziej widoczne).
2. **Podstawa zasięgu** → liczyć w ładunku (`remaining_mah / used_mah_na_km`).
3. **Konfigurowalna krzywa OCV** (plan niżej) — dokładność dla różnych baterii.
4. **Pojemność** → uczyć z OCV, nie tautologicznie (albo wyłączyć, jeśli ryzyko biasu > zysk).
5. Coulomb: kalibracja do realnej częstotliwości.

---

## PLAN: konfigurowalna krzywa OCV przez CAN

### Decyzje
- **`cells = system_voltage/3.6` zostaje** (świadoma decyzja użytkownika — napięcie szacuje ilość ogniw).
- Krzywa **per-OGNIWO** (spójne z obecnym kodem: `cell_voltage = pack/cells`, potem lookup).
- Kotwice SOC `{0,5,10,20,30,40,50,60,70,80,90,100}` **stałe** (zostają jako `soc_values[]`);
  user podaje **tylko 12 napięć**.

> Uwaga: przy per-ogniwie błąd `Vsys/3.6` nadal przesuwa `cell_voltage`. Jedyny sposób całkiem
> to usunąć = krzywa per-PAKIET (wtedy `cells` nieużywane do SOC). Świadomie zostajemy przy per-ogniwie.

### Zakres
1. **`ocv_curve_id` (1 bajt):** 0 = wbudowana NMC/LG (domyślna), 1 = LFP (preset), 2 = custom.
   Presety pokrywają ~95%; custom dla reszty.
2. **Custom = 12 napięć na ogniwo, 2 bajty (mV, LE) każde = 24 bajty** (multiframe).
   2 bajty, nie 1 — LFP ma płaski odcinek 3,2–3,3 V wymagający drobnej rozdzielczości.
3. **Walidacja przy odbiorze (WAŻNE):** ciąg **ściśle rosnący**; jeśli nie → odrzuć upload,
   zostaw poprzednią/domyślną (interpolacja dzieli przez `v[i+1]-v[i]` → zero/ujemne = śmieci).
4. **Fallback:** pola `0xFF`/puste → wbudowana krzywa (bezpieczna migracja starego configu).
5. **Przy okazji:** wystaw `r_batt_mohm` (już w `MotorParams_t`, main.c:2126) na CAN — kompensacja
   IR decyduje o jakości `u_comp` pod obciążeniem, więc dopełnia krzywą.

### Zmiany w kodzie
- `calculate_SOC()` (main.c:2000): zamień hardkodowaną `voltages[]` na tablicę z `MotorParams_t`
  (wybieraną wg `ocv_curve_id`; presety NMC/LFP jako `const`, custom z Para).
- `MotorParams_t` (`inc/main.h`): `uint8_t ocv_curve_id; uint16_t ocv_cell_mv[12];` (+ `r_batt_mohm` już jest).
- `parser.c` `parseParameters()`/`buildParameters()`: mapowanie na wolne indeksy Para (24 bajty
  custom → rozważyć osobny blok/multiframe, wzorem `Override_Duration=Para1[37]*40`).
  Walidacja monotoniczna po parsowaniu.
- `soc_init()`/`soc_update()`: bez zmian logiki — używają `calculate_SOC()`, która czyta nową krzywą.

---

## Weryfikacja
1. Log `MS.SOC`, `soc_real`, `soc_voltage`, `range`, `avg_wh_per_km` (UART main.c:563 / CAN 0x0203).
2. **1%/min:** podjazd/pełny ciąg → `soc_real` spada realnie, `soc_display` po fixie nadąża
   (limit prądowy); w normalnej jeździe brak drgań.
3. **Zasięg:** przy niskim SOC nie zawyża (po przejściu na ładunek).
4. **Krzywa OCV:** wgraj preset LFP na baterii LFP → SOC z napięcia zgodny z rzeczywistością;
   custom z błędną (nierosnącą) tablicą → odrzucony, brak śmieci.
5. **Pojemność:** po fixie `measured` różni się od `cap_est` (realny pomiar), mieści się w klamrze.
6. Wyłącz/włącz → SOC z flash; ładowanie przy off → skok do OCV.

## Ryzyka
- Limit prądowy SOC źle dobrany → albo drgania (za luźny), albo dalej lag (za ciasny). Stroić.
- Krzywa per-ogniwo nadal wrażliwa na błąd `Vsys/3.6` — user musi ustawić `system_voltage`
  tak, by dawało poprawną liczbę ogniw (np. 48 dla 13S).
- 24 bajty custom przez Para — sprawdzić dostępne indeksy/rozmiar bloku; może wymagać multiframe.
