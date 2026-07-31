# FW-056 / CB-056 — tryb wspomagania Power Curve (gamma)

- **Data:** 2026-07-29
- **Status:** ZAIMPLEMENTOWANE (etapy A i B), **niezbudowane i nieprzetestowane
  na sprzęcie**. Build firmware nie był uruchamiany — czeka na wyraźne polecenie.
  Stan szczegółowo w rozdziale 0.
- **Zakres:** firmware M820 (`assist_modes.c/.h`) + zakładka `eVistDrive Profiles`
  w Canable. Obie strony wdrażane **równocześnie**, etap po etapie.
- **Powiązane:** `FW-051_054_WA_BANK_CONFIG.md` (aktualny format banku v3),
  `FW-043_WA_RPM_UNLOCK_BANK_CUTOFF.md`, `FW-033_TORQUE_CONDITIONING_PLAN.md`.
- **Zastępuje:** wcześniejszy szkic „Plan wdrożenia trybu POWER_CURVE" —
  jego rozdziały 7, 8 i 15 opierały się na nieaktualnym formacie banku
  (v1 / 185 B / nagłówek 8 B) i nie domykały się arytmetycznie.

---

## 0. Stan wdrożenia

### Firmware (EBICS)

| Plik | Co zrobione |
|---|---|
| `tools/generate_power_curve_lut.js` | nowy — generator tablicy; wypuszcza **oba** artefakty: nagłówek C i moduł JS dla Canable |
| `inc/power_curve_lut.h` | nowy — generowany, 23 × (65 + 9) punktów, 3404 B |
| `inc/power_curve.h`, `src/power_curve.c` | nowe — `power_curve_eval_permille()` |
| `inc/assist_modes.h` | `ASSIST_MODE_POWER_CURVE = 6`, `curve_exponent_x10`, dwa pola diagnostyczne |
| `src/assist_modes.c` | `bank_mode_valid`, `normalize_power_support_bounds`, trzy funkcje wsparcia, `switch` dispatcher, `switch` w `calculate_power`, bajt 9 per tryb, `BANK_BLOB_VERSION_V4` |
| `inc/main.h`, `src/CAN_Display.c` | tylko nieaktualne komentarze |
| `tests/fw056_power_curve.js` | nowy — test matematyki |

`src/*.c` jest globowane przez `build_firmware.ps1`, więc `power_curve.c`
wejdzie do buildu bez zmian w skrypcie.

### Canable

| Plik | Co zrobione |
|---|---|
| `bafang-parser.js` | v4 przyjmowane, bajt 9 czytany per tryb |
| `canbus.js` | v4 odsyłane tylko przy v4 na wejściu, bajt 9 zapisywany per tryb |
| `ui/js/power-curve-lut.js` | nowy, **generowany przez firmware** — ta sama tablica i ten sam całkowitoliczbowy algorytm |
| `ui/js/tab-ebics.js` | tryb 6 w liście, dwie gammy, podgląd liczony dokładnie jak firmware, dwa wykresy |
| `ui/index.html` | dwa wykresy obok siebie: Support ratio i Requested motor power |
| `tests/fw056_bank_blob_roundtrip.js` | nowy — test round-tripu bloku |

### Weryfikacja wykonana

- `node tests/fw056_power_curve.js` → **PASS**. Najgorszy błąd dostarczonej mocy
  silnika **1,41 W** (0,17% mocy dostarczanej w tym punkcie) przy budżecie
  1 W lub 1%. Szczegóły i uzasadnienie miary w rozdziale 4.3.
- `node tests/fw056_bank_blob_roundtrip.js` → **PASS** (długość 189 B, wersja 4,
  bajt 9 per tryb, v3 dla starych sterowników, złe CRC / wersja / długość odrzucane).
- `node --check` na `bafang-parser.js`, `canbus.js`, `ui/js/tab-ebics.js` → OK.

### Czego NIE zrobiono

- **Build firmware nie był uruchamiany** — zgodnie z ustaleniem, że
  `build_firmware.ps1` odpala się tylko na wyraźne polecenie. Zajętość Flash
  po dodaniu 3404 B tablicy jest więc **niepotwierdzona**.
- Brak testów sprzętowych (rozdziały 8.3 i 8.4).
- `protocol/ebics_config_schema.yaml` nietknięty — porządki dokumentacyjne,
  świadomie odłożone (uzasadnienie w rozdziale 7).

### Odchyłki od planu

1. **Generator w Node, nie w Pythonie.** Na maszynie nie ma ani Pythona,
   ani hostowego gcc. Node jest i tak wymagany przez Canable, więc jeden
   runtime obsługuje obie strony.
2. **Test matematyki jest portem, nie kompilacją C.** Bez hostowego gcc nie da
   się wykonać `power_curve_eval_permille()` z `src/power_curve.c`. Test czyta
   **prawdziwą wygenerowaną tablicę** z `inc/power_curve_lut.h` i przepuszcza ją
   przez wierny port funkcji. Port jest celowo zduplikowany i opisany w nagłówku
   pliku testowego — przy zmianie kodu C trzeba go zmienić razem z nim.
3. **Wykres: dwa panele obok siebie zamiast przełącznika widoku** (na życzenie
   właściciela). Support ratio i Requested motor power są widoczne jednocześnie,
   bo krzywa łagodna na jednym wykresie potrafi być stroma na drugim.
5. **Podgląd krzywej jest bit w bit zgodny z firmware** (na życzenie
   właściciela). Generator wypuszcza tę samą tablicę także jako
   `ui/js/power-curve-lut.js`, a `tab-ebics.js` powtarza całkowitoliczbową
   matematykę sterownika: to samo obcinające dzielenie na promile, ta sama
   interpolacja, to samo okno wsparcia. Test w firmware porównuje oba artefakty
   i zgłasza błąd, jeśli moduł JS się rozjedzie z nagłówkiem C.
   **Uwaga: to dotyczy wyłącznie kształtu krzywej.** Startup Boost, filtry mocy,
   Smooth Start, Release, rampa Iq oraz limity prędkości i temperatury nie są
   modelowane nigdzie na wykresie — do porównania z rzeczywistością służy
   telemetria `0x6029`.
6. **Druga gamma** (na życzenie właściciela) — rozdział 4.4. Karta w pierwotnej
   wersji przewidywała jedną.
4. **Tryb jest zawsze widoczny na liście, a nie ukrywany** (zmiana na życzenie
   właściciela). Rozdział 6.3 zakładał filtrowanie listy po
   `bank_schema_version >= 4`. Zamiast tego: pozycja jest zawsze dostępna, przy
   starym sterowniku dostaje dopisek „needs newer firmware", edytor pokazuje
   ostrzeżenie, a **przycisk Apply odmawia zapisu z czytelnym powodem**.
   Zabezpieczenie zostaje w mocy — nadal nie da się wysłać bloku, który
   firmware odrzuciłoby w całości, cicho zostawiając stare profile. Zyskiem
   jest to, że krzywą i wykres można obejrzeć przed flashowaniem.

---

## 1. Po co to jest — prostym językiem

Dziś w trybie Power Progressive kształt pomocy ustawia suwak „Progression"
0–100%. To miesza dwie krzywe (prostą i kwadratową) i przy małych wartościach
prawie nic nie zmienia.

Power Curve robi to samo, ale jednym, zrozumiałym pokrętłem — **gamma**:

- **gamma 1,0** — pomoc rośnie równo z Twoją siłą (linia prosta),
- **gamma większa od 1** — na początku silnik pomaga delikatnie, a im mocniej
  naciskasz, tym szybciej dokłada. Spokojny start, mocna końcówka,
- **gamma mniejsza od 1** — krzywa wygina się w drugą stronę: silnik od razu
  mocno dokłada przy lekkim nacisku i dochodzi do maksymalnego wsparcia
  na długo przed mocą odniesienia. Agresywne — pierwszy test na stojaku.

Nic z obecnych trybów się nie zmienia. Power Curve jest dodatkiem, wybieranym
osobno dla każdego poziomu wspomagania.

---

## 2. Stan faktyczny firmware (zweryfikowany w kodzie)

Poprzedni szkic zawierał nieaktualne liczby. Rzeczywistość:

| Element | Szkic mówił | Jest w kodzie |
|---|---|---|
| Wersja bloku banku | 1 | **3** (`BANK_BLOB_VERSION_V3`, `assist_modes.c:130`) |
| Nagłówek | 8 B | **12 B** (`BANK_BLOB_HEADER_LEN`) |
| Rekord poziomu | 35 B | 35 B — zgodne, ale **w pełni zajęty, bajty 0..34** |
| Cały blok | 185 B | **189 B** (`ASSIST_BANK_BLOB_LEN`, `assist_modes.h:59`) |
| Parser przyjmuje | tylko v1 | v1 / v2 / v3 (`assist_modes.c:909-922`) |
| Bufory | „190 zmieści się w 192" | `bank_store[2][192]`, `BankBlob[192]` |
| Limit multiframe | nie uwzględniony | `command < 23` → **maks. 24 ramki = 192 B** |
| Canable | „nie zakłada 185 na sztywno" | zakłada **189 na sztywno**, `canbus.js:792` |

Wniosek: propozycja „rekord 36 B, blok 190 B" daje w rzeczywistości
`12 + 5×36 + 2 = 194 B`, czyli **przekracza oba bufory i limit ramek**.
Wymagałaby zmiany układu EEPROM i strażnika ramek — dużo ryzyka za jeden bajt.

---

## 3. Co się w szkicu zgadzało i zostaje bez zmian

- `POWER_CURVE = 6` jest wolne (enum kończy się na `ASSIST_MODE_TORQUE = 5`).
- Interpretacja osi X = moc człowieka **po** Startup Boost — zgodna z kodem
  (`assist_basis_power_mw` liczone z `assist_load_centikg`, `assist_modes.c:550`).
- Krzywa zmienia **współczynnik wsparcia**, nie zadaje bezpośrednio mocy.
  Dalej działa niezmieniony tor: limit mocy → filtr → P/U → `max_iq_pct` →
  rampa Iq → FOC (`calculate_power` + `finish_power_request`).
- Brak `float` w pętli sterowania.
- `bank_mode_valid()` (`assist_modes.c:149`) trzeba rozszerzyć, inaczej cały
  blok z trybem 6 zostanie odrzucony.
- Rozbicie `calculate_support_ratio_pct()` na jawny `switch` — obecne
  „jak nie Linear, to Progressive" (`assist_modes.c:291`) po dodaniu trzeciego
  trybu Power byłoby pułapką.

---

## 4. Trzy decyzje projektowe, które naprawiają plan

### 4.1 Gammy jadą w bajtach martwych dla tego trybu — zero zmian formatu

Tryb ma **dwie gammy**: dolną i górną (opis w 4.4). Obie mieszczą się bez
powiększania rekordu, bo zajmują bajty należące do pól, które w POWER_CURVE nie
mają żadnego znaczenia:

```
bajt 9 rekordu = (mode == POWER_CURVE) ? curve_exponent_x10       (dolna gamma)
                                       : progression_pct          (tylko PROGRESSIVE)
bajt 1 rekordu = (mode == POWER_CURVE) ? curve_exponent_high_x10  (górna gamma)
                                       : support_ratio_pct        (tylko LINEAR, u16)
bajt 2 rekordu = (mode == POWER_CURVE) ? 0 (zarezerwowany)
                                       : starszy bajt support_ratio_pct
```

Cena: poziom przełączony na Power Curve traci na wire swoją wartość
`progression_pct` i `support_ratio_pct`. Po powrocie na Linear/Progressive
dostaje wartości domyślne. To ta sama klasa kompromisu w obu przypadkach
i jedyna, jaką płacimy za brak zmiany formatu.

Skutki:
- rekord dalej 35 B, blok dalej **189 B**, wersja formatu bez zmian,
- `bank_store[2][192]`, `BankBlob[192]`, strażnik `command < 23` — bez zmian,
- **zero migracji** starych profili, zero ryzyka utraty ustawień,
- cały „commit 3" ze szkicu znika.

W strukturze C zostają **dwa osobne pola** (`progression_pct` oraz
`curve_exponent_x10`), żeby kod pozostał czytelny; łączy je dopiero
serializacja. Pole nieaktywne dla danego trybu dostaje swoją wartość domyślną.

### 4.2 Wersja bloku 4 jako znacznik możliwości (bez zmiany długości)

Canable musi wiedzieć, czy podłączone firmware w ogóle zna tryb 6 — inaczej
użytkownik wybierze Power Curve, firmware odrzuci cały blok i profile po cichu
zostaną stare.

Rozwiązanie bez kosztu: **`BANK_BLOB_VERSION = 4` przy identycznym układzie
bajtów i identycznej długości 189 B**. Wersja niesie tylko informację
„to firmware zna Power Curve".

- Firmware: serializuje v4, parsuje v1/v2/v3/v4 (v4 = ten sam kod co v3).
- Canable: po odczycie banku patrzy na `bank_schema_version`.
  - `>= 4` → pozycja „Power Curve" widoczna w liście trybów, zapis jako v4,
  - `< 4` → pozycja ukryta, zapis jako v3 (dokładnie dzisiejsze zachowanie).

Stare firmware nigdy nie zobaczy bloku v4, bo Canable go nie wyśle.
Nowe firmware czyta wszystko wstecz. To jest cały handshake — nie trzeba
czytać wersji software'u kontrolera ani dodawać nowej komendy CAN.

### 4.3 Zakres gamma 0,3–2,5 — pełny, wraz z krzywymi poniżej 1

**Korekta wcześniejszej wersji tej karty.** Pierwotnie odłożyłem gamma < 1,
powołując się na błąd tablicy przy zerze. Miara była zła. Krzywa `x^0,3`
faktycznie rośnie przy zerze pionowo i równomierna tablica myli się tam
o kilkadziesiąt promili — ale to jest błąd **wartości krzywej**, a do silnika
idzie `moc człowieka × wsparcie`. Obszar dużego błędu to dokładnie ten, w którym
moc człowieka jest bliska zeru, więc realna pomyłka mocy silnika wynosi tam
ułamek wata. Krzywa pozostaje monotoniczna i ciągła — nie ma żadnego skoku.

Kryterium poprawności jest więc postawione na **dostarczonej mocy silnika**,
nie na wartości krzywej. Test przechodzi, gdy błąd mocy jest poniżej 1 W
albo poniżej 1% mocy aktualnie dostarczanej, przy zachowawczej konfiguracji
odniesienia 300 W / okno wsparcia 500 punktów procentowych.

Żeby dodatkowo zbić błąd tam, gdzie był największy, tablica ma **podsiatkę**:
pierwszy odcinek siatki głównej (x od 0 do 1/64) jest podzielony jeszcze na
8 części. Kosztuje 414 B i dla gamma ≥ 1 jest bezczynna, bo krzywa jest tam
i tak płaska.

Zmierzone wartości (`node tests/fw056_power_curve.js`):

| Miara | Wynik | Budżet |
|---|---|---|
| błąd mocy silnika | **1,41 W** (gamma 2,4, rowerzysta 279 W) | 1 W **lub** 1% |
| błąd względny tam, gdzie przekroczono 1 W | **0,17%** | 1% |
| błąd wartości krzywej | 46,9 promila (gamma 0,3, x = 1 promil ≈ 0,3 W nacisku) | — |

Tablica: 23 wykładniki × (65 + 9) punktów × 2 B = **3404 B Flash**.
Zajętość całości do potwierdzenia przy pierwszym buildzie.

Gamma poniżej 1 zostaje w interfejsie **oznaczona jako agresywna**, z zaleceniem
pierwszego testu na stojaku, przy włączonym Smooth Start i zachowawczym
`max_iq_pct`. To ograniczenie procedury, nie możliwości.

### 4.4 Dwie gammy — osobno dolna i górna połowa okna wsparcia

Okno wsparcia dzieli się na pół i każda połowa dostaje własny wykładnik:

| Odcinek | Moc człowieka | Wsparcie | Kształtuje |
|---|---|---|---|
| dolny | 0 → połowa mocy odniesienia | `support_min` → środek okna | dolna gamma |
| górny | połowa mocy odniesienia → moc odniesienia | środek okna → `support_max` | górna gamma |

Obie połowy liczone są na pełnej rozdzielczości tablicy, więc żadna nie traci
dokładności. Środek okna wypada **dokładnie** przy połowie mocy odniesienia —
punkt styku obu odcinków jest ciągły, bez uskoku.

Obie gammy równe 1,0 dają dokładnie linię prostą. Obie równe tej samej wartości
powyżej 1 dają symetryczny kształt S, a nie zwykłe `x^gamma` — to celowa zmiana
znaczenia względem jednogammowej wersji tej karty. Tryb nie był jeszcze nigdy
wgrany na rower, więc nie ma czego migrować.

Zastosowanie praktyczne: dolna gamma steruje tym, jak spokojnie silnik rusza
przy lekkim pedałowaniu, a górna tym, czy szczyt wsparcia jest oddawany od razu
po przekroczeniu połowy zakresu, czy trzymany w rezerwie do pełnej mocy
odniesienia.

---

## 5. Zmiany w firmware

### 5.1 `inc/assist_modes.h`

```c
ASSIST_MODE_POWER_CURVE = 6      /* dopisane na końcu enum, nic nie przesuwa */
```

W `assist_level_config_t`, bezpośrednio po `progression_pct`:

```c
uint8_t curve_exponent_x10;      /* 10..25 = gamma 1,0..2,5 */
```

`ASSIST_BANK_BLOB_LEN` **bez zmian (189)**.

W `assist_mode_output_t` (diagnostyka, bez wysyłki po CAN):

```c
uint16_t curve_input_permille;
uint16_t curve_output_permille;
```

### 5.2 `inc/power_curve_lut.h` (nowy, generowany)

Tablica `uint16_t POWER_CURVE_LUT[16][33]` w promilach, z komentarzem
zawierającym parametry generatora. Generator: `tools/generate_power_curve_lut.py`.

Funkcja `power_curve_eval_permille(uint16_t input_permille, uint8_t exponent_x10)`:
- wiersz = `exponent_x10 - 10`,
- indeks = `input_permille * 32 / 1000`, reszta → interpolacja liniowa,
- kontrakt: `0 → 0`, `1000 → 1000`, monotonicznie, nigdy powyżej 1000,
  bez `float`.

### 5.3 `src/assist_modes.c`

1. Stałe: `POWER_CURVE_EXP_MIN_X10 3`, `MAX_X10 25`, `DEFAULT_X10 15`.
2. `bank_mode_valid()` (linia 149) — dopisać `mode == ASSIST_MODE_POWER_CURVE`.
   Przy okazji rozstrzygnąć `ASSIST_MODE_EMTB_CUSTOM` (dziś tryb istnieje
   w enum, ale nie da się go zapisać do profilu) — **poza zakresem tej karty,
   tylko odnotowane**.
3. Wydzielić `normalize_power_support_bounds()` z obecnego
   `calculate_support_ratio_pct()` (linie 297-316) — wspólne dla
   PROGRESSIVE i CURVE, bez duplikacji walidacji min/max/reference.
4. Nowa `calculate_power_curve_support_pct()` — jak w szkicu, ale
   z zakresem wykładnika 10..25.
5. `calculate_support_ratio_pct()` → jawny `switch` po `mode_type`,
   `default: return 0`.
6. `calculate_power()` (linie 539-540) — warunek „brak wsparcia" jako `switch`;
   dla CURVE kryterium to `support_max_pct == 0`.
7. Główny `switch` w `assist_modes_calculate()` (linia 1025) — dopisać
   `case ASSIST_MODE_POWER_CURVE:` do gałęzi `calculate_power()`.
8. Serializacja (linia 868) i parsowanie (linia 964) bajtu 9 według decyzji 4.1,
   z walidacją zależną od trybu:
   - CURVE → `clamp(10, 25)`, wartość spoza zakresu → 15,
   - pozostałe → dotychczasowy `clamp` do `PROGRESSION_MAX_PCT`.
9. `BANK_BLOB_VERSION` → 4, `BANK_BLOB_VERSION_V4` obsługiwane w parserze
   identycznie jak v3 (ten sam nagłówek 12 B, ta sama długość).

### 5.4 Czego **nie** ruszamy

`main.h` (`bank_store[2][192]`), `CAN_Display.c` (`BankBlob[192]`, strażniki
ramek), układ EEPROM, telemetria 0x6029. Aktualizujemy tylko nieaktualne
komentarze: `CAN_Display.c:58` mówi „187 B used" zamiast 189.

### 5.5 Diagnostyka — nic nowego po CAN

Blok diagnostyczny 0x6029 (`CAN_Display.c:750`) już wysyła: moc człowieka,
zastosowany support, moc silnika, `iq_request`, `i_q_setpoint`, prędkość,
kadencję i nacisk. Wejście krzywej to `moc człowieka / reference_power`,
wyjście odtwarza się z supportu i min/max. **Nie dodajemy pól do wire.**
Nowe pola w `assist_mode_output_t` służą tylko debugowi na biurku.

---

## 6. Zmiany w Canable

### 6.1 `bafang-parser.js` — `bankBlob()` (linia 354)

- Dopuścić `d[2] === 4`, `HEADER` dla v4 = 12 (jak v3).
- `bank_schema_version` już jest zwracany — wystarczy, żeby przeszedł 4.
- Pole `progression_pct` czytać jak dziś, dodatkowo wystawić
  `curve_exponent_x10 = d[r + 9]` (ten sam bajt, druga etykieta).

### 6.2 `canbus.js` — `serializeBankBlob()` (linia 791)

- `BLOB_LEN` **pozostaje 189**, `HEADER = 12`, `RECORD = 35`.
- `d[2] = bankObj.bank_schema_version >= 4 ? 4 : 3` — nigdy nie wysyłamy v4
  do firmware, które go nie zgłosiło.
- `d[r + 9] = (lv.mode_type === 6) ? lv.curve_exponent_x10 : lv.progression_pct`.

### 6.3 `ui/js/tab-ebics.js`

- `MODES` (linia 34) — dopisać `{ value: 6, label: 'Power Curve' }`,
  **filtrowane po `bank_schema_version >= 4`** przy budowie listy (linia 125).
- `modeFields()` (linia 171) — gałąź dla trybu 6: `support_min_pct`,
  `support_max_pct`, `reference_power_w` (identyczne jak tryb 2) plus:
  ```
  { key: 'curve_exponent_x10', label: 'Curve shape (gamma)',
    min: 1.0, max: 2.5, step: 0.1,
    fromNative: v => v / 10, toNative: v => Math.round(v * 10),
    help: '1.0 = support rises evenly with your effort. Higher = gentler at
           low effort, stronger as you approach Reference rider power.' }
  ```
- `buildProfilePlaceholderBank()` (linia 253) — dodać `curve_exponent_x10: 15`.
- Zapis poziomu (linia 727) — przy zmianie trybu z 2 na 6 i odwrotnie
  **ustawić sensowną wartość docelowego pola**, bo dzielą bajt: wchodząc
  w Power Curve bez zapisanej gammy wstawić 15. To jedyny koszt decyzji 4.1
  i jest w całości po stronie UI.
- Wykres (`requestedPowerForLevel`, linia 359, gałąź `progression` linia 369) —
  dodać gałąź trybu 6: `y = x^gamma`, ta sama matematyka co firmware.
  Wykres ma dwa widoki: **Support [%]** i **Motor power [W]**, z zaznaczoną
  pionową linią `reference_power_w` oraz limitem `max_motor_power_w`.
- Etykieta pod suwakiem: `1,0 = liniowa` / `> 1 = progresywna, łagodny start`.

### 6.4 Zakładki, których nie ruszamy

Stare karty Bafang (Controller / Battery) — bez zmian, zgodnie z zasadą
„nowe funkcje tylko do `tab-ebics-*`".

---

## 7. Kolejność wdrożenia — obie strony razem

Każdy etap kończy się **działającym rowerem i działającą aplikacją**.
Nie ma etapu, w którym firmware umie coś, czego Canable nie potrafi ustawić.

### Etap A — matematyka (bez wpływu na jazdę)

- FW: generator + `power_curve_lut.h` + `power_curve_eval_permille()` + testy
  hostowe. Funkcja nie jest jeszcze przez nikogo wywoływana.
- Canable: bez zmian.
- Weryfikacja: testy z punktu 8.1 przechodzą, build się kompiluje,
  rozmiar Flash sprawdzony.

### Etap B — tryb + format v4 + UI, w jednym rzucie

- FW: punkty 5.1, 5.3 w całości (enum, pole, funkcja, oba `switch`,
  serializacja bajtu 9, wersja 4).
- Canable: punkty 6.1, 6.2, 6.3 w całości.
- Weryfikacja: build FW OK, `node --check` na trzech zmienionych plikach OK,
  odczyt banku w Canable pokazuje `v4`, tryb Power Curve widoczny,
  zapis i ponowny odczyt zwraca tę samą gammę.
- **Dopiero po tym etapie flash na rower.**

### Etap C — walidacja jazdowa i korekta zakresów

- Testy z punktów 8.3 i 8.4, ewentualna korekta domyślnych profili testowych.
- Aktualizacja tej karty: status, numer buildu, wyniki.

Etap „schemat protokołu": `protocol/ebics_config_schema.yaml` ma
`version: 0`, `status: draft`, `complete_for_codegen: false` i wszystkie
`wire_id: null` — **to nie jest źródło formatu wire**, tylko dokument roboczy.
Aktualizujemy go osobno, przy okazji (dopisać `torque: 5`, `power_curve: 6`,
podnieść `max` w `mode_type` z 4 na 6, dodać `curve_exponent_x10`), i **nie
blokujemy tym wdrożenia**.

---

## 8. Testy

### 8.1 Matematyka krzywej (host)

Dla każdej gammy 1,0–2,5:
- `x = 0 → y = 0`, `x = 1000 → y = 1000`,
- monotoniczność na całym zakresie,
- nigdy powyżej 1000,
- maksymalny błąd względem referencyjnego `pow()` **poniżej 3 promile**
  (test ma wypisać zmierzone maksimum, nie tylko „OK"),
- `gamma = 1,0` → `y ≈ x` (błąd 0),
- `gamma = 2,0` → `y ≈ x²`,
- `gamma > 1` → `y < x` dla `0 < x < 1000`.

### 8.2 Współczynnik i serializacja (host)

- `support_min = support_max` → stałe wsparcie niezależnie od gammy,
- `x = 0 → support_min`, `x ≥ reference → support_max`,
- odwrócone min/max normalizowane bezpiecznie,
- wartości powyżej 1000% obcinane,
- odczyt v1/v2/v3 nadal działa i **nie ustawia trybu 6**,
- zapis v4 → odczyt v4 → identyczne dane, długość dokładnie 189 B,
- blok z trybem 6 i błędnym CRC odrzucony,
- poziom w trybie 2 z bajtem 9 = 20 dalej znaczy „Progression 20%",
  a nie gamma 2,0.

### 8.3 Testy na stojaku

1. **Zgodność z Linear** — `support_min = support_max = obecny support_ratio`.
   Power Curve musi zachować się identycznie jak Power Linear, niezależnie
   od gammy. To jest test całej ścieżki i bezpieczny punkt odniesienia.
2. **gamma 1,0** — różne min/max, wsparcie interpoluje liniowo.
3. **gamma 2,0** — początek zakresu wyraźnie łagodniejszy, support dochodzi
   do max przy mocy odniesienia.
4. **Granice** — brak kadencji, kadencja zasiana, bardzo mały nacisk,
   gwałtowny nacisk, moc powyżej `reference_power_w`, aktywny limit mocy,
   aktywny limit Iq.
5. **Zgodność wsteczna** — poziomy w trybach 1/2/3/5 zachowują się dokładnie
   jak przed zmianą (porównanie telemetrii 0x6029 przed/po).

### 8.4 Testy drogowe

Jedna zmiana naraz. Kolejność: gamma 1,5 → potem 1,3 i 1,7 → porównanie
z Power Linear → dopiero potem strojenie min/max → na końcu Startup Boost
i filtry. Nie zmieniać gammy razem ze Smooth Start i rampą Iq.

Kryteria zaliczenia:
- brak skoku żądanego Iq przy pojawieniu się wspomagania,
- support rośnie monotonicznie z mocą człowieka,
- brak oscylacji w okolicy `reference_power_w`,
- limity mocy i Iq działają,
- Linear / Progressive / eMTB / Torque bez zmiany zachowania,
- profile przeżywają restart,
- stary bank v3 nadal się wczytuje,
- wykres w Canable zgodny z telemetrią firmware.

---

## 9. Profil startowy do testów

Nie jest to kalibracja produkcyjna — punkt wyjścia do porównania z obecnymi
poziomami liniowymi.

| Poziom | Obecny Linear | Support min | Support max | Ref. power | Gamma |
|---|---|---|---|---|---|
| 1 | 100% | 85% | 130% | 200 W | 1,5 |
| 2 | 200% | 170% | 260% | 200 W | 1,5 |
| 3 | 320% | 270% | 420% | 200 W | 1,5 |
| 4 | 420% | 350% | 550% | 200 W | 1,5 |
| 5 | 520% | 440% | 680% | 200 W | 1,5 |

Na pierwsze jazdy: Startup Boost wyłączony lub maks. 30–50%, Smooth Start
włączony, `max_iq_pct` zachowawczy, „assist without rotation" wyłączony.

---

## 10. Wycofanie

- Przełączyć poziomy z powrotem na Power Linear — natychmiastowy powrót
  do obecnego zachowania.
- Nie trzeba nic czyścić: bajt 9 wraca do roli `progression_pct`.
- Dotychczasowe numery trybów niezmienione, format banku tej samej długości,
  więc **stary firmware wczytuje bank zapisany przez nowy** — pod warunkiem,
  że żaden poziom nie jest w trybie 6 (wtedy `bank_mode_valid()` odrzuci blok
  i zostaną ustawienia domyślne; to jest cena i trzeba ją znać).
- Pełny rollback = flash poprzedniego buildu, bez migracji danych.

---

## 11. Definicja ukończenia

- działa bez `float` w pętli sterowania,
- testy matematyczne i serializacji przechodzą, z wypisanym zmierzonym błędem,
- czyta profile v1/v2/v3/v4, zapisuje v4 o długości dokładnie 189 B,
- `bank_store`, `BankBlob` i strażniki ramek nietknięte,
- Canable pokazuje tryb tylko przy firmware v4 i poprawnie rysuje obie osie,
- diagnostyka 0x6029 pozwala rozdzielić kształt krzywej od filtra mocy
  i rampy Iq,
- Linear / Progressive / eMTB / Torque bez zmiany zachowania,
- przeszedł stojak i jazdę porównawczą.

---

## 12. Otwarte pytania do właściciela

1. **Gamma poniżej 1** — potwierdzasz odłożenie na osobną kartę? (powód: 4.3)
2. **Nazwa w UI** — „Power Curve" czy coś po polsku w interfejsie?
3. **Czy Power Curve ma docelowo zastąpić Power Progressive**, czy oba tryby
   zostają na stałe? Jeśli ma zastąpić, w kolejnej karcie można wycofać
   suwak Progression i uwolnić bajt 9 w całości.
