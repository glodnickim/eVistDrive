# Porównanie: SOC baterii i zasięg (range) — EBICS vs TSDZ2

Cel: zrozumieć jak liczony jest stan naładowania (SOC) i zasięg w obu firmware'ach,
wyciągnąć wnioski dla EBICS. Wskaźniki, które widzi użytkownik na HMI: **kreski/% baterii**
oraz **zasięg [km]**.

---

## Różnica architektoniczna (najważniejsza)

| | Gdzie liczony SOC/range | Konsekwencja |
|---|---|---|
| **EBICS** | **na sterowniku** (self-contained) | HMI Bafang to „głupi" wyświetlacz — oczekuje gotowego SOC (kreski) i zasięgu od sterownika. EBICS MUSI to policzyć. |
| **TSDZ2** | **na wyświetlaczu** (860C/850C/OSF) | Sterownik wysyła tylko surowe `battery_voltage_x10` i `battery_current_x5` (ebike_app.c:97-98). SOC/Wh/zasięg liczy wyświetlacz. |

> W lokalnym repo `.external/TSDZ2-Smart-EBike-860C` jest firmware **sterownika** — nie ma
> tam kodu SOC/range (to logika wyświetlacza OSF). Poniżej metoda wyświetlacza OSF z wiedzy o projekcie.

---

## Jak liczy EBICS (na sterowniku) — zweryfikowane w kodzie

Rdzeń: `soc_update()` @1 Hz (`main.c:~2166`), `calculate_SOC()` (`main.c:2000`), coulomb w
`reg_ADC_processing()` (`main.c:1481`).

1. **Coulomb counting (główne):** `soc_mAs_acc += Battery_Current/4000` co tick 4 kHz →
   co 1 s `dmah = mAs/3600` → `remaining_mah -= dmah` (znak: rozładowanie −, **regen +**).
   `soc_real = remaining_mah / battery_capacity_estimated_mah × 100`.
2. **OCV lookup z kompensacją IR:** `calculate_SOC(u_comp, cells)` — tablica krzywej
   rozładowania **ogniw LG M58T** (12 punktów 2.799→4.070 V/ogniwo), `u_comp = V + I×R`,
   `cells = system_voltage/3.6`.
3. **Seed + korekty:**
   - boot: SOC z OCV; jeśli był ładowany przy wyłączonym (skok OCV vs zapis) → zaufaj OCV;
   - jazda: **powolna korekta OCV tylko w spoczynku** (`OCV_CORR_GAIN`, anti-drift, nigdy skok);
   - display: `soc_display` dolnoprzepustowy z limitem kroku/min (anti-jump), szybciej przy <10%.
4. **Zasięg:** `remaining_wh = remaining_mah/1000 × system_voltage`;
   `range = remaining_wh / wh_km_level[poziom]`. **Zużycie Wh/km uczone PER POZIOM** wspomagania
   (EMA `RANGE_EMA_ALPHA`), seed `default_wh_km_for_level()` (Eco 7 … Boost 18 Wh/km).
5. **Auto-pojemność:** pełny cykl rozładowania (SOC 92%→<12%) → estymacja `capacity_est_mah`.
6. **Persistencja:** flash wear-leveled, 64 sloty (strona 127), CRC32; zapis przy zmianie SOC
   ≥ `SOC_SAVE_DELTA` lub przy wyłączeniu (`soc_state_save`).
7. **Limp mode:** `compute_limp_factor(soc)` skaluje moc przy niskim SOC.

## Jak liczy TSDZ2 (wyświetlacz OSF) — metoda

1. **Wh-counting (domyślne w OSF):** użytkownik ustawia **całkowitą pojemność baterii [Wh]**.
   Wyświetlacz całkuje `consumed_Wh = ∫ V·I dt`. `SOC% = (total_Wh − consumed_Wh)/total_Wh`.
   **Auto-reset do 100%** gdy napięcie ≥ „napięcie pełnego naładowania" przy spoczynku.
2. **Wariant napięciowy:** SOC z prostej krzywej napięcia (opcja).
3. **Zasięg:** `remaining_Wh / (consumed_Wh / przejechany_dystans)` — jedna średnia Wh/km z tripu.
4. Wszystko **konfigurowalne z wyświetlacza** (pojemność Wh, napięcia reset/cutoff, jednostki).

---

## Podsumowanie różnic

| Aspekt | EBICS (sterownik) | TSDZ2 (wyświetlacz) |
|---|---|---|
| Metoda SOC | **hybryda coulomb + OCV** (samokorekta) | Wh-counting lub krzywa napięcia |
| Regen | uwzględniany (znak) | zależne od metody |
| Ładowanie przy off | wykrywane (skok OCV vs zapis) | auto-reset po napięciu |
| Krzywa ogniwa | **hardkod LG M58T** | konfig. napięcia / brak krzywej ogniwa |
| Zasięg | **Wh/km uczone PER POZIOM** | jedna średnia Wh/km z tripu |
| Pojemność | auto-uczona z cyklu | ustawiana ręcznie [Wh] |
| Persistencja | flash wear-level + CRC | stan wyświetlacza |
| Konfiguracja user | **słaba** (dużo hardkodu) | **mocna** (wszystko z menu) |

---

## Wnioski dla EBICS

1. **EBICS jest technicznie BOGATSZY** (coulomb+OCV, uczenie per poziom, auto-pojemność, flash) —
   to nie TSDZ2 ma tu przewagę metodą, tylko **konfigurowalnością**. Nie kopiować metody TSDZ2.
2. **Główny brak EBICS = konfigurowalność (lekcja z TSDZ2).** Wystawić przez CAN (bloki Para):
   - `battery_capacity_estimated_mah` (dziś auto/estymowana) — pozwolić ustawić ręcznie;
   - **wybór krzywej OCV / chemii ogniwa** — dziś zahardkodowana LG M58T; dla LiFePO4 lub
     innych Li-ion SOC seed/korekta będą błędne. Dodać wybór tablicy (Li-ion/LiFePO4) lub
     parametry krzywej; to najważniejsza poprawka dokładności.
   - domyślne `wh_km_level[]` / reset uczenia zasięgu;
   - napięcia: `voltage_min` (cutoff) i próg „pełne = 100%".
3. **Prosty reset do 100% po napięciu** (jak TSDZ2) — EBICS ma częściowo (detekcja doładowania
   przez skok OCV). Rozważyć jawny próg „V_full → SOC=100%" przy spoczynku dla pewności.
4. **Zasięg per poziom** (EBICS) zostawić — jest lepszy niż pojedyncza średnia TSDZ2;
   ewentualnie dodać widoczny „reset licznika zasięgu".
5. **Ryzyko #1 dokładności:** `cells = system_voltage/3.6` + krzywa LG. Jeśli bateria to nie
   ~4.1 V/ogniwo Li-ion NMC → SOC przekłamany. Priorytet: konfigurowalna chemia (pkt 2).

### Wdrożenie CAN (wzorzec — jak w innych planach)
Dodać pola do `MotorParams_t`, init w `parser.c`/`main.c`, mapować na wolne indeksy Para1/Para2
w `parseParameters()`/`buildParameters()` (wzorem `Override_Duration=Para1[37]*40`).
Kandydaci: `battery_capacity_mah` (2 bajty LE), `ocv_curve_id` (1 bajt: 0=LG NMC, 1=LiFePO4, …),
`v_full_reset` (1 bajt, skala). Wdrożyć po weryfikacji dokładności na realnej baterii.

## Weryfikacja
1. Log `MS.SOC`, `MS.soc_real`, `MS.soc_voltage`, `MS.range`, `MS.avg_wh_per_km` (UART/CAN).
2. Pełna bateria → SOC ~100% (seed OCV) i po jeździe zbieżny z coulomb.
3. Postój → korekta OCV nie robi skoków; near-empty (<10%) konwerguje szybko.
4. Zasięg maleje sensownie z Wh/km danego poziomu; zmiana poziomu → uczenie osobnego Wh/km.
5. Wyłącz/włącz → SOC odtworzony z flash; ładowanie przy off → SOC podskakuje do OCV.
6. Inna chemia baterii → potwierdzić błąd SOC (uzasadnia pkt 2 wniosków).
