# FW-084 — Extended Boost dla Ride Core

- **Data:** 2026-08-04
- **Status:** **IMPLEMENTED — BENCH/ROAD VALIDATION PENDING** (2026-08-06). Domyślnie
  WYŁĄCZONE (`duration = 0`). Po audycie z 2026-08-06 naprawione oba P0 i wszystkie P1/P2;
  nie wgrane i nie przetestowane na rowerze — patrz sekcja 16.
- **Nazwa:** funkcja nazywa się **Extended Boost**. Nazwy `Overrun`/`Override` należą
  wyłącznie do nieaktywnego mechanizmu Legacy (`Legacy overrun (inactive in Ride Core)`).
- **Cel:** świadome podtrzymanie napędu po zatrzymaniu korb, przeznaczone do pokonywania
  stopni, kamieni i krótkich przerw w pedałowaniu na technicznym podjeździe.
- **Zakres:** firmware Ride Core, bank blob v8, transport CAN, Canable Profiles,
  diagnostyka oraz testy automatyczne i stanowiskowe.
- **Powiązane:** `FW-025_PAS_STOP_CUT.md`, `FW-031_RIDE_LATCH_FLOOR.md`,
  `FW-069_RAMPS_PER_LEVEL.md`, `FW-072_SINGLE_RELEASE_RAMP.md`,
  `FW-074_ASSIST_CARRY_OVER_PLAN.md`.

## 1. Decyzja funkcjonalna

Extended Boost nie jest zwykłym wydłużeniem zaniku wspomagania. Użytkownik świadomie uzbraja
go mocnym naciśnięciem pedału. Po rozpoznaniu zatrzymania korb firmware przez określony czas
podaje prąd wyliczony ze szczytowego nacisku ostatniego kwalifikującego impulsu. Następnie
przekazuje sterowanie do istniejącej rampy `Release duration`.

```text
obrót korby do przodu + mocny nacisk ponad próg
        |
        v
ARMED — zapamiętany szczyt nacisku
        |
        | pedaling_active: true -> false
        v
ACTIVE — Extended Boost przez skonfigurowany czas
        |
        | timeout
        v
RELEASE — istniejące release_ms
        |
        v
OFF
```

Funkcja:

- nie jest wyzwalana przyciskiem, impulsem koła ani impulsem Halla silnika;
- nie jest wyzwalana samym dużym `dTorque/dt`;
- korzysta z bezwzględnego, skalibrowanego nacisku na pedał w kg;
- nie przechwytuje prądu manetki ani Walk Assist;
- jest domyślnie wyłączona;
- działa tylko w zwykłej ścieżce Ride Core.

Nie włączać starego `EXTENDED_BOOST_ENABLE` i nie przenosić bezpośrednio
`Overrun_strength`, `Overrun_counter` ani `Overrun_flag` z monolitu Legacy. Stary blok ma
inne źródła stanu, rozpoczyna licznik w niewłaściwym momencie i omija część obecnej
architektury Ride Core.

## 2. Parametry użytkownika

Trzy nowe parametry są przechowywane osobno dla każdego banku i poziomu wspomagania.
Istniejący `release_ms` jest czwartym elementem wpływającym na końcowy przebieg, ale nie jest
częścią nowej struktury.

> **Zmiana z 2026-08-06 (wdrożenie):** próg dostał pełny zakres czujnika zamiast 22,5 kg, a
> domyślna wartość to 20,0 kg. Uzasadnienie w sekcji 16.2.

| Parametr | Jednostka i zakres | Domyślna wartość | Znaczenie |
|---|---:|---:|---|
| `extended_boost_trigger_load_kg` | 0,5 kg; 1,0–60,0 kg | 20,0 kg | Minimalny nacisk uzbrajający boost |
| `extended_boost_strength_pct` | 0–255% | 100% | Mnożnik prądu wyliczonego ze szczytu nacisku |
| `extended_boost_duration_ms` | 0–1000 ms | 0 ms | Czas ACTIVE; `0` całkowicie wyłącza funkcję |
| istniejący `release_ms` | 0–3000 ms | obecnie 650 ms | Czas zejścia do zera po ACTIVE |

Semantyka procentu:

- `100%` — prąd wynikający bezpośrednio z zapamiętanego nacisku;
- `150%` — ten prąd pomnożony przez 1,5;
- `255%` — maksymalny mnożnik 2,55;
- wynik zawsze jest ograniczony do `ride_core_iq_limit` aktywnego poziomu, a następnie przez
  wszystkie wspólne limity prądu, mocy, napięcia, temperatury, prędkości i trybu legal.

Wartości startowe do pierwszego testu terenowego:

```text
Trigger load:       20,0 kg
Boost strength:     100%
Boost duration:     200 ms
Release duration:   150 ms
```

Nie dodawać w pierwszej wersji regulacji debounce, histerezy, czasu ważności uzbrojenia,
minimalnej prędkości ani osobnego limitu prądu. Są to stałe bezpieczeństwa w firmware, a
nie parametry charakteru jazdy.

## 3. Jednostki i obliczenie prądu

Wejściem jest `rider->torque_load_centikg`, czyli skalibrowany nacisk z `torque_input`:

```text
100 centikg = 1,00 kg
TORQUE_PUBLIC_FULL_SCALE_CENTIKG = 6000 = 60,00 kg
```

Nie używać surowych mV, `torque_filtered`, `torque_run_filtered` ani starego
`MP.TQO_threshold`. Publiczny próg w kg musi mieć identyczne znaczenie po zmianie czujnika i
po kalibracji jego zakresu.

Prąd bazowy Extended Boost jest liniowym przeskalowaniem części nacisku znajdującej się ponad
progiem:

```c
peak = min(peak_load_centikg, TORQUE_PUBLIC_FULL_SCALE_CENTIKG);
span = TORQUE_PUBLIC_FULL_SCALE_CENTIKG - trigger_load_centikg;
above = peak > trigger_load_centikg ? peak - trigger_load_centikg : 0;

base_iq = (ride_core_iq_limit * above + span / 2) / span;
boost_iq = (base_iq * strength_pct + 50) / 100;
boost_iq = min(boost_iq, ride_core_iq_limit);
```

Wszystkie obliczenia wykonać całkowitoliczbowo, z co najmniej 32-bitowym wynikiem pośrednim.
`trigger_load_centikg` jest walidowany poniżej pełnej skali, więc `span` nie może być zerem.

Konsekwencje są zamierzone:

- samo dotknięcie progu uzbraja funkcję, ale daje prąd bliski zeru;
- silniejszy nacisk daje większy boost;
- `strength_pct > 100` może zwiększyć wynik względem mapy nacisku;
- boost nie może przekroczyć maksymalnego prądu wybranego poziomu.

## 4. Moduł i maszyna stanów

Dodać mały, testowalny moduł:

```text
inc/assist_extended_boost.h
src/assist_extended_boost.c
```

Minimalne stany:

```c
typedef enum {
    ASSIST_EXT_BOOST_IDLE = 0,
    ASSIST_EXT_BOOST_QUALIFY,
    ASSIST_EXT_BOOST_ARMED,
    ASSIST_EXT_BOOST_ACTIVE
} assist_extended_boost_state_t;
```

Zalecane stałe wewnętrzne:

```c
#define EXT_BOOST_CONTROL_TICKS_PER_MS 4U
#define EXT_BOOST_CONFIRM_MS           30U
#define EXT_BOOST_RELEASE_HYST_CENTIKG 50U    /* 0,50 kg */
#define EXT_BOOST_ARM_TIMEOUT_MS        1500U
#define EXT_BOOST_MIN_SPEED_X100        100U  /* 1,00 km/h */
#define EXT_BOOST_MIN_MOTOR_ERPS        10U
```

### 4.1. IDLE i kwalifikacja nacisku

Moduł pozostaje w `IDLE`, gdy czas wynosi 0, poziom ma indeks 0 albo występuje warunek
kasujący. Przy prawidłowym pedałowaniu do przodu i nacisku równym lub większym od progu
przechodzi do `QUALIFY`.

Nacisk musi pozostać ponad progiem przez `EXT_BOOST_CONFIRM_MS`. Krótszy pik jest odrzucany.
To chroni przed pojedynczym zakłóceniem ADC, uderzeniem łańcucha i impulsem od wyboju.

Podczas `QUALIFY` zapisywać najwyższy nacisk. Po 30 ms przejść do `ARMED` i zachować szczyt.

### 4.2. Ostatni kwalifikujący impuls nacisku

„Ostatni skok pedału” należy definiować jako ostatnie ciągłe okno nacisku ponad progiem, a nie
największy nacisk od początku jazdy:

1. przekroczenie progu rozpoczyna nowe okno i zeruje szczyt kandydata, zachowując poprzedni
   ważny szczyt do zakończenia kwalifikacji;
2. przez całe okno aktualizowany jest `peak_load_centikg`;
3. spadek poniżej `trigger - 0,5 kg` kończy okno, ale pozostawia moduł uzbrojony;
4. następne potwierdzone przekroczenie zastępuje poprzedni szczyt, nawet jeśli jest mniejsze;
5. uzbrojenie wygasa po 1500 ms bez kwalifikującego nacisku.

Dzięki temu dawny bardzo mocny obrót korby nie może zostać odtworzony wiele sekund później.
Histereza nie zmienia progu użytkownika — jedynie stabilizuje zakończenie impulsu.

Implementacyjnie przechowywać osobno `candidate_peak` i ostatni ważny `armed_peak`. Nowe
okno może zastąpić `armed_peak` dopiero po pełnych 30 ms kwalifikacji. Odrzucony krótki pik nie
może skasować wcześniej poprawnie uzbrojonego impulsu.

### 4.3. Wejście w ACTIVE

Boost uruchamia się wyłącznie na zboczu:

```c
previous_pedaling_active && !pedaling_active
```

i tylko gdy równocześnie:

- stan ma ważne, wcześniej potwierdzone uzbrojenie;
- `duration_ms > 0` i `strength_pct > 0`;
- wcześniej działało legalnie uruchomione wspomaganie z pedałów;
- aktywny poziom jest większy od 0;
- nie działa manetka jako źródło zapamiętywanego prądu;
- rower ma co najmniej 1,0 km/h, a silnik co najmniej 10 ERPS;
- nie ma `safety_cut`, Walk Assist, kalibracji Halla ani kalibracji torque;
- czujniki PAS i torque są ważne.

Timer ACTIVE zaczyna się w tym miejscu, nie w chwili pierwszego przekroczenia progu. Impulsy
PAS, Halla, prędkości ani kolejne drgania nie mogą go przedłużyć.

Aktualny `assist_latched` spada w `ride_control` w tym samym takcie co PAS STOP. Moduł musi
używać tej flagi wyłącznie podczas kwalifikacji/uzbrajania, gdy korba jeszcze się obraca.
Nie wolno skasować poprawnego `ARMED` tylko dlatego, że bieżąca wartość latcha stała się false
na zboczu wyzwalającym ACTIVE; wcześniejsze legalne uruchomienie potwierdza już stan modułu.

Jeżeli obliczony `boost_iq` wynosi 0, nie przechodzić do `ACTIVE`; oddać cel 0 do zwykłego
Release.

### 4.4. ACTIVE i zakończenie

Podczas ACTIVE moduł zwraca `boost_iq` jako cel wspomagania pedałami i ustawia dla
`ride_control` informację `hold_profile_active = true`. Dzięki temu `assist_dynamics` stosuje
normalne rampy Iq, ale nie rozpoczyna jeszcze `release_ms`.

Jeżeli wspólny limit chwilowo obniży wynik, timer nadal biegnie. Limit prędkości, napięcia lub
mocy nie może pauzować ani ponownie uruchamiać ACTIVE.

Po dokładnie `duration_ms * 4` taktach:

- stan przechodzi do `IDLE`;
- cel Extended Boost wynosi 0;
- `hold_profile_active = false`;
- istniejący `assist_dynamics` wykonuje jedno zejście do zera przez `release_ms`.

Nie implementować drugiej rampy wewnątrz modułu Extended Boost.

### 4.5. Kasowanie

Stan `QUALIFY`, `ARMED` i `ACTIVE` jest zerowany w tym samym takcie przy:

- `safety_cut`: hamulec, cofanie korb, błąd torque, kalibracja torque lub przegrzanie stopnia 2;
- nieważnym PAS albo torque;
- poziomie wspomagania 0;
- zmianie poziomu lub banku;
- wejściu w Walk Assist;
- wejściu w kalibrację położenia silnika;
- spadku prędkości poniżej 1,0 km/h albo ERPS poniżej 10 podczas ACTIVE;
- powrocie pedałowania podczas ACTIVE;
- inicjalizacji sterownika.

Kasowanie stanu nie oznacza obecnie twardego wyłączenia mostka. `ride_control` ustawia target
0, natomiast hamulec/cofanie/błąd korzystają z istniejącego `RIDE_SAFETY_RELEASE_MS = 200 ms`.
Zmiana tej wspólnej polityki safety na hard-cut jest osobnym zadaniem i nie może zostać ukryta
w implementacji FW-084.

Po powrocie pedałowania w ACTIVE wymagany jest nowy potwierdzony impuls nacisku. Nie wolno
pozostawić starego uzbrojenia ani wznowić niedokończonego timera.

## 5. Proponowany interfejs modułu

Interfejs powinien być niezależny od globalnych struktur `MS` i `MP`, żeby można go było
testować na komputerze:

```c
typedef struct {
    uint16_t trigger_load_centikg;
    uint8_t strength_pct;
    uint16_t duration_ms;
} assist_extended_boost_config_t;

typedef struct {
    bool pedaling_active;
    bool pedal_assist_latched;
    bool motion_valid;
    bool safety_cut;
    bool walk_active;
    bool position_calibration_active;
    bool torque_sensor_valid;
    bool pas_sensor_valid;
    uint8_t bank_index;
    uint8_t level_index;
    uint16_t pedal_load_centikg;
    int32_t ride_core_iq_limit;
} assist_extended_boost_input_t;

typedef struct {
    int32_t iq_target;
    bool profile_hold_active;
    bool armed;
    bool active;
} assist_extended_boost_output_t;

void assist_extended_boost_init(void);
void assist_extended_boost_reset(uint8_t reason);
void assist_extended_boost_update(
    const assist_extended_boost_input_t *input,
    const assist_extended_boost_config_t *config,
    assist_extended_boost_output_t *output);
```

Moduł może mieć osobny getter diagnostyczny. Nie wystawiać wskaźników do modyfikowalnego
stanu wewnętrznego.

## 6. Wpięcie w `ride_control.c`

W gałęzi zwykłej jazdy zachować następującą kolejność:

```text
assist_modes_calculate
    -> ride latch / minimalny Iq
    -> EXTENDED BOOST              <-- nowe miejsce
    -> throttle floor
    -> safety_cut
    -> assist_limits
    -> smooth start / preload
    -> coast_release
    -> assist_dynamics
    -> motor_core_set_command
```

Ważne wymagania:

1. Przed wywołaniem modułu zachować osobno `pedal_iq_target`. Nie wolno zapisywać wyniku po
   dodaniu manetki.
2. Przekazać do modułu stan istniejącego `assist_latched`; sam obrót korby i nacisk nie mogą
   ominąć legalnej bramki startu.
3. Jeżeli `output.active`, zastąpić tylko pedal-only `iq_target` wartością boostu.
4. Dopiero potem zastosować manetkę jako niezależny floor.
5. `safety_cut` i wszystkie limity pozostają za boostem i mają ostatnie słowo.
6. Podczas ACTIVE ustawić `profile_pedaling_active = true`; po timeout ustawić false, jeśli
   nie ma innego aktywnego źródła, aby rozpocząć `release_ms`.
7. Wczesne ścieżki Walk Assist i kalibracji położenia muszą jawnie resetować moduł przed
   `return`.
8. `ride_control_init()` musi inicjalizować moduł.

Nie dodawać Extended Boost do `assist_modes_calculate()`. Tryby Power/eMTB/Torque obliczają
normalne wspomaganie, natomiast Extended Boost jest późniejszym, wspólnym zachowaniem jazdy.

## 7. Konfiguracja profilu i bank blob v8

Do `assist_level_config_t` dodać:

```c
assist_extended_boost_config_t extended_boost;
```

### 7.1. Układ rekordu

Aktualny bank v7 ma 245 B:

```text
13 B header + 5 * 46 B record + 2 B CRC = 245 B
```

Transport przenosi długość w jednym bajcie, więc absolutny limit wynosi 255 B. Trzy nowe pola
zmieszczą się dokładnie dzięki wykorzystaniu dwóch bajtów zarezerwowanych przez FW-077:

| Offset rekordu | Pole | Typ przewodowy | Zakres |
|---:|---|---|---:|
| 36 | `extended_boost_trigger_load_kg` | u8, 0,5 kg/LSB | 2–120 = 1,0–60,0 kg |
| 37 | `extended_boost_strength_pct` | u8 | 0–255% |
| 46–47 | `extended_boost_duration_ms` | u16 LE | 0–1000 ms |

Nowy format:

```text
13 B header + 5 * 48 B record + 2 B CRC = 255 B
CRC: offset 253–254
```

To zużywa cały obecny limit transportu. Następne pole per-level będzie wymagało ponownego
wykorzystania istniejącego bajtu, zmiany sposobu pakowania albo nowej wersji transportu z
długością u16.

### 7.2. Zmiany firmware

1. Dodać `BANK_BLOB_VERSION_V8 = 8` i ustawić bieżącą wersję na v8.
2. Nazwać długości jawnie: `BANK_RECORD_LEN_V7 = 46`, `BANK_RECORD_LEN_V8 = 48`.
3. Ustawić `BANK_RECORD_LEN = BANK_RECORD_LEN_V8` i `ASSIST_BANK_BLOB_LEN = 255`.
4. Dodać `_Static_assert(ASSIST_BANK_BLOB_LEN == 255U, ...)` oraz asercję `<= UINT8_MAX`.
5. Serializer zapisuje offsety 36, 37 i 46–47.
6. Parser v8 wymaga dokładnie rekordu 48 B.
7. Parser v1–v7 ustawia bezpieczne wartości migracyjne:

   ```text
   trigger = 20,0 kg
   strength = 100%
   duration = 0 ms
   ```

8. Pola 36–37 odczytywać jako Extended Boost wyłącznie dla `version >= 8`; v6/v7 miały tam
   inne lub zarezerwowane znaczenie.
9. Zachować osobne sprawdzanie `record_len >= 46` dla pól FW-068/069/077. Nie uzależniać
   istniejących ramp i progu jazdy od nowej długości 48 B.
10. Zaktualizować komentarze `bank_store[2][256]`, `BankBlob[256]` i opisy liczby ramek.

`MotorParams_t.bank_store[2][256]` oraz `BankBlob[256]` mieszczą 255 B bez zmiany układu
`MotorParams_t`.

### 7.3. Ostatnia ramka CAN

255 B wymaga 32 ramek danych. Ostatnia ma indeks 31 i zawiera 7 bajtów. W obu gałęziach
odbioru `0x6021` w `CAN_Display.c` zmienić warunek:

```c
if (Ext_ID_Rx.command < 30)
```

na:

```c
if (Ext_ID_Rx.command < 31)
```

Test musi potwierdzić zapis bajtów 248–254, końcową długość 255 oraz poprawny CRC. Typ
`uint8_t length` nadal może przenieść 255; nie wykonywać na nim inkrementacji długości.

## 8. Schemat konfiguracji

W `protocol/evistdrive_config_schema.yaml` dodać trzy pola `since: 8`:

```yaml
- key: extended_boost_trigger_load_kg
  since: 8
  access: persistent_rw
  per_level: true
  value: {type: u8, scale: 0.1, unit: kilogram_force, min: 1.0, max: 22.5, default: 8.0}
  implementation: planned_bank_v8_record_36
  safety_relevant: true
  ui_group: extended_boost

- key: extended_boost_strength_pct
  since: 8
  access: persistent_rw
  per_level: true
  value: {type: u8, scale: 1, unit: percent_multiplier, min: 0, max: 255, default: 100}
  implementation: planned_bank_v8_record_37
  safety_relevant: true
  ui_group: extended_boost

- key: extended_boost_duration_ms
  since: 8
  access: persistent_rw
  per_level: true
  value: {type: u16, scale: 1, unit: millisecond, min: 0, max: 1000, default: 0}
  implementation: planned_bank_v8_record_46
  zero_semantics: disabled
  safety_relevant: true
  ui_group: extended_boost
```

## 9. Canable

Repozytorium: `C:\Projekty\bafang_canable_pro`.

### 9.1. Parser i zapis

W `bafang-parser.js`:

- zaakceptować bank schema v8;
- dla rekordu co najmniej 48 B i wersji co najmniej 8 odczytać nowe offsety;
- dla v1–v7 zwrócić domyślne `8.0 / 100 / 0`;
- nie interpretować starych bajtów 36–37 jako boost.

W `canbus.js`:

- użyć `RECORD = 48`, `BLOB_LEN = 255` tylko gdy sterownik zgłosi
  `bank_schema_version >= 8`;
- zapisywać próg jako u8 decikg, siłę jako u8 i czas jako u16 LE;
- do sterownika v7 i starszego wysyłać jego dotychczasowy format;
- nigdy nie obcinać 255 do 0 ani nie traktować wartości jako signed byte;
- obsłużyć końcową ramkę 7 B.

### 9.2. UI

W `eVistDrive Profiles`, dla wybranego banku i poziomu, dodać osobną kartę:

```text
Obstacle assist — Extended Boost
```

Pola:

```text
Trigger pedal load       [kg]
Boost strength           [%]
Boost duration           [ms] — 0 = Off
```

`Release duration` pozostaje w istniejącej karcie `Power smoothing and release`. Dymek
Extended Boost musi wyjaśniać, że czasy się sumują:

```text
czas od rozpoznania PAS STOP do zera = Boost duration + Release duration
czas od ostatniego impulsu PAS do zera ≈ 200–500 ms + Boost duration + Release duration
```

Proponowane opisy:

- **Trigger pedal load:** „A confirmed pedal load at or above this value arms Extended
  Boost. It uses calibrated pedal load, not the rate at which the signal rises.”
- **Boost strength:** „Multiplies the current calculated from the peak load of the latest
  qualifying pedal push. The result is still capped by this level's current limit and all
  controller safety limits.”
- **Boost duration:** „How long the motor may keep pushing after forward pedalling is
  recognized as stopped. 0 disables Extended Boost.”

Gdy odczytany kontroler obsługuje maksymalnie v7, ukryć kartę albo pokazać ją jako
nieobsługiwaną. Nie pozwalać na edycję wartości, której nie da się zapisać.

Rozszerzyć copy level, copy section, restore defaults, import/export presetów oraz offline
placeholdery. Parametry Extended Boost muszą być kopiowane razem jako jedna sekcja.

## 10. Diagnostyka

Bez diagnostyki trudno rozróżnić brak uzbrojenia, oczekiwanie na PAS STOP, limit prądu i
zakończenie timera. Rozszerzyć `0x6029` do wersji 5.

Proponowane pola za dotychczasowym bajtem 44:

| Bajty | Pole |
|---:|---|
| 45 | flagi: bit0 qualify, bit1 armed, bit2 active, bit3 arm expired |
| 46–47 | ostatni/aktywny `peak_load_centikg` |
| 48–49 | wyliczony `boost_iq` |
| 50–51 | pozostały czas ACTIVE w ms |
| 52 | ostatnia przyczyna anulowania |
| 53–54 | CRC16 |

Całkowita długość v5: 55 B. Parser Canable musi nadal przyjmować diagnostykę v4 o długości
47 B.

Zalecane przyczyny anulowania:

```text
0 none
1 disabled
2 safety_cut
3 reverse
4 sensor_invalid
5 walk
6 calibration
7 level_or_bank_change
8 motion_lost
9 pedaling_resumed
10 arm_timeout
11 completed
```

## 11. Testy automatyczne

### 11.1. Moduł stanu przy 4 kHz

Utworzyć `tests/fw084_extended_boost.js` albo równoważny test hostowy obejmujący:

1. `duration_ms = 0` daje identyczny przebieg jak firmware bez FW-084.
2. Nacisk poniżej progu nie uzbraja funkcji.
3. Duży, ale krótszy niż 30 ms pik nie uzbraja funkcji.
4. Nacisk ponad próg przez dokładnie wymagany czas przechodzi do ARMED.
5. Szybkość przyrostu nacisku sama w sobie nie ma znaczenia.
6. Zapisywany jest szczyt bieżącego kwalifikującego impulsu.
7. Następny kwalifikujący impuls zastępuje poprzedni także wtedy, gdy jest słabszy.
8. Uzbrojenie wygasa po 1500 ms i nie uruchamia starego boostu.
9. ACTIVE zaczyna się tylko na zboczu `true -> false` pedałowania.
10. Czasy 1, 200 i 1000 ms trwają dokładnie odpowiednio 4, 800 i 4000 taktów.
11. Wzór dla 100%, 150% i 255% jest zgodny z obliczeniem referencyjnym.
12. Wynik nigdy nie przekracza `ride_core_iq_limit`.
13. Obliczenia graniczne nie przepełniają typu całkowitego.
14. Hamulec/safety, cofanie, błąd czujnika, poziom 0, Walk i kalibracja kasują stan w tym
    samym takcie.
15. Zmiana banku lub poziomu kasuje stan.
16. Utrata ruchu podczas ACTIVE kasuje stan.
17. Powrót pedałowania kasuje ACTIVE i wymaga nowego impulsu nacisku.
18. Hall, prędkość i drgania nie odświeżają timera.
19. Manetka nie uzbraja funkcji i nie zmienia zapamiętanego szczytu.
20. Po ACTIVE uruchamia się dokładnie jeden istniejący Release.

### 11.2. Limity i integracja Ride Core

Sprawdzić, że w ACTIVE nadal działają:

- `max_iq_pct` poziomu;
- `max_motor_power_w`;
- limit prądu baterii;
- low-voltage taper;
- temperatura;
- limit prędkości i tryb legal;
- `coast_release` po zaniku targetu;
- hard overcurrent w FOC.

Każdy test powinien rozdzielić `boost_iq` przed limitami od finalnego `iq_setpoint`.

### 11.3. Bank blob i CAN

1. V7 pozostaje przyjmowany i migruje do `duration = 0`.
2. V7 round-trip w Canable pozostaje bajt w bajt bez zmian.
3. V8 ma dokładnie 255 B, rekord 48 B i CRC w bajtach 253–254.
4. Wszystkie trzy wartości przechodzą round-trip dla pięciu poziomów.
5. Nieprawidłowy próg, czas i wartości graniczne są naprawiane przez parser.
6. Odbiornik przyjmuje ostatnią ramkę o indeksie 31 i długości 7 B.
7. Brak ostatniej ramki kończy się jawnym ERROR_ACK.
8. Canable nigdy nie wysyła v8 do kontrolera zgłaszającego maksymalnie v7.
9. Copy/restore/import/export zachowują komplet trzech parametrów.

## 12. Test stanowiskowy

Najpierw koło uniesione, potem kontrolowane obciążenie, na końcu zamknięty odcinek terenowy.

1. Ustawić `8,0 kg / 100% / 200 ms / release 150 ms`.
2. Lekko pedałować i zatrzymać korbę — boost nie może się uzbroić.
3. Nacisnąć ponad próg, ustawić korby poziomo i zmierzyć czas od `pedaling_active=false` do
   końca ACTIVE oraz do `iq_setpoint=0`.
4. Powtórzyć z małym i dużym szczytem nacisku; prąd musi rosnąć monotonicznie ze szczytem.
5. Powtórzyć dla 100%, 150% i 255%; potwierdzić cap aktywnego poziomu.
6. Podczas QUALIFY, ARMED i ACTIVE nacisnąć hamulec.
7. Podczas ACTIVE cofnąć korbę, zmienić poziom na 0, zmienić bank i uruchomić Walk Assist.
8. Wznowić pedałowanie w połowie ACTIVE — timer ma zostać skasowany bez release i bez skoku.
9. Jechać równocześnie na manetce i pedałach — boost nie może kopiować prądu manetki.
10. Sprawdzić wszystkie tryby Power Linear, Progressive, eMTB, Torque i Power Curve.
11. Sprawdzić limit prędkości, limit mocy, niski stan baterii i podwyższoną temperaturę.
12. Zapisać bank do flash, wyłączyć sterownik, uruchomić ponownie i odczytać wartości.

Próby terenowe rozpocząć od 100% i 200 ms. Siłę zwiększać dopiero po potwierdzeniu hamulca,
cofania, limitów i braku ponownego szarpnięcia przy końcu boostu.

## 13. Kryteria odbioru

- nowe i migrowane profile mają `duration = 0`, więc funkcja jest domyślnie OFF;
- przy OFF przebieg Iq jest identyczny z wersją bez FW-084;
- boost wymaga potwierdzonego nacisku w kg oraz legalnie uruchomionej asysty;
- duży przyrost sygnału bez utrzymanego nacisku nie wyzwala funkcji;
- timer startuje na PAS STOP i nie jest odświeżany;
- ostatni impuls nacisku zastępuje starszy, nawet jeżeli ma mniejszy szczyt;
- manetka i Walk Assist nigdy nie są źródłem zapamiętanego prądu;
- aktywny poziom i wszystkie wspólne limity nadal ograniczają wynik;
- wszystkie ścieżki kasowania działają w tym samym takcie sterowania;
- po boost działa dokładnie jedna rampa `release_ms`;
- bank v8 ma dokładnie 255 B i poprawnie przenosi ostatnią ramkę;
- zapis RAM, zapis flash, restart oraz ponowny odczyt zachowują parametry;
- dostępny jest build diagnostyczny przed wydaniem normalnej wersji.

## 14. Kolejność wdrożenia dla developera

1. Dodać strukturę konfiguracji i bezpieczne wartości domyślne, ale jeszcze nie wpływać na Iq.
2. Wprowadzić bank v8, serializer/parser, 255 B oraz obsługę ramki 31.
3. Zaktualizować schema, parser i serializer Canable; potwierdzić round-trip v7/v8.
4. Dodać czysty moduł `assist_extended_boost` i jego testy hostowe.
5. Wpiąć moduł pomiędzy latch i throttle w `ride_control`.
6. Dodać resety do Walk Assist, kalibracji, poziomu/banku oraz `ride_control_init()`.
7. Rozszerzyć diagnostykę `0x6029` i parser Canable.
8. Dodać kartę UI, dymki, copy/restore/import/export oraz ukrywanie dla firmware v7.
9. Uruchomić pełny zestaw testów firmware i Canable.
10. Zbudować wersję diagnostyczną i wykonać test z kołem uniesionym.
11. Wykonać test obciążeniowy i dopiero potem zamknięty test terenowy.
12. Po akceptacji uzupełnić `CHANGELOG.md`, instrukcję użytkownika i numery buildów.

## 15. Relacja do Assist Carry-Over

FW-074 opisuje łagodne podtrzymanie części bieżącego prądu po każdym zatrzymaniu pedałów.
FW-084 opisuje świadomie uzbrajany, zależny od nacisku Extended Boost, który może zwiększyć
prąd przez `strength_pct`.

Nie wdrażać obu funkcji jednocześnie w pierwszej wersji. Mają różne wyzwalanie i różną
semantykę, a ich nałożenie utrudniłoby rozstrzygnięcie, który moduł jest właścicielem celu Iq.
Dla pokonywania kamieni i stopni pierwszeństwo ma FW-084. Ewentualny Carry-Over należy później
dodać jako osobny tryb z jednoznaczną regułą wzajemnego wykluczania.

Przed użyciem poza zamkniętym terenem należy osobno sprawdzić wymagania właściwe dla roweru i
rynku. Plan techniczny nie rozstrzyga dopuszczalności podtrzymania napędu bez ruchu korb.

## 16. Stan wdrożenia (2026-08-06)

### Zrobione

| Element | Miejsce |
|---|---|
| Moduł i maszyna stanów | `inc/assist_extended_boost.h`, `src/assist_extended_boost.c` |
| Konfiguracja per poziom + domyślne OFF | `inc/assist_modes.h`, `src/assist_modes.c` |
| Bank v8 (rekord 48 B, blob 255 B, CRC 253–254) | `src/assist_modes.c` |
| Ostatnia ramka CAN o indeksie 31 | `src/CAN_Display.c` (obie gałęzie 0x6021) |
| Wpięcie między latch a manetkę + resety | `src/ride_control.c` |
| Diagnostyka `0x6029` v5 (55 B) | `src/CAN_Display.c` |
| Schema (3 pola `since: 8`) | `protocol/evistdrive_config_schema.yaml` |
| Testy hostowe firmware | `tests/fw084_extended_boost.js` — 20 przypadków + integracja + blob/CAN + diagnostyka |
| Canable: parser i zapis v8 | `bafang-parser.js`, `canbus.js` |
| Canable: karta UI + dymki + blokada na v7 | `ui/js/evistdrive/profiles.js` |
| Canable: diagnostyka v5 | `bafang-parser.js`, `ui/js/evistdrive/system.js`, `ui/index.html` |
| Canable: test round-trip | `tests/fw084_bank_v8_roundtrip.js` (w `npm test`) |

Wszystkie testy hostowe firmware (15 plików) i Canable (8 plików) przechodzą.

### 16.1. Poprawki po audycie z 2026-08-06

Audyt (`FW-084_AUDIT_DEVELOPER_HANDOFF.md`) znalazł dwa błędy krytyczne. Oba naprawione:

| # | Problem | Poprawka |
|---|---|---|
| P0 | Boost omijał `max_iq_pct` i `max_motor_power_w` poziomu, bo zastępował target PO ich zastosowaniu w `finish_power_request()` | Nowy wspólny `assist_modes_profile_iq_ceiling()` (procent + moc przeliczona tą samą metodą), stosowany w `ride_control` do celu boostu, przed manetką. Procent ma jedną implementację używaną przez obie ścieżki |
| P0 | Klasyfikacja `NON_PEDAL` wychodziła przypadkiem z opadniętego `assist_latched` | Polityka zapisana wprost: `if (boost_active) source = NON_PEDAL`, z komentarzem, że zmiana na `PEDAL_CONFIRMED` jest decyzją produktowo-prawną. Opisane w dymku Canable |
| P1 | Zapis banku nie kasował uzbrojenia zrobionego pod starą konfiguracją | `assist_modes_apply_bank_blob()` woła reset z nowym kodem `CONFIG_CHANGED = 12` |
| P1 | Test firmware był kopią algorytmu w JS | Doszedł `tests/host/fw084_extended_boost_host.c` linkujący **prawdziwy** `assist_extended_boost.c` + `tests/host/run-host-tests.ps1`. Nagłówek testu JS mówi teraz wprost, że jest modelem |
| P1 | Preset v8 wczytany na kontroler v7 pokazywał wartości, które zapis cicho pomijał | `minBankSchema: 8` na deskryptorach pól; importer je pomija i **mówi o tym** w logu. Bramka sekcji jest wyliczana z pól, więc edytor i importer nie mogą się rozjechać |
| P2 | `remaining_ms` zaniżało czas | Zaokrąglanie w górę + test graniczny |
| P2 | Nazewnictwo i statusy | `Legacy overrun (inactive in Ride Core)` w `config.h`/`main.c`, aktualizacja FW-091, ta sekcja, wpis w `CHANGELOG.md` |

### 16.2. Decyzje właściciela z 2026-08-06

1. **Zakres progu: 1,0–60,0 kg, krok 0,5 kg** (zamiast 1,0–22,5 kg co 0,1 kg). Powód: pełna
   skala czujnika ma 60 kg, a próg jest tylko progiem UZBROJENIA. Jeden bajt na wire przy
   0,1 kg kończy się na 25,5 kg, więc rozdzielczość musiała ustąpić — miejsca w rekordzie nie
   ma, blob jest na suficie 255 B. Krok 0,5 kg (a nie 0,25) po to, żeby każda zapisywalna
   wartość była dokładna przy JEDNYM miejscu po przecinku, jak pozostałe pola kg.
2. **Domyślny próg: 20,0 kg.** Wysoki próg nic nie kosztuje przy normalnym pedałowaniu, a
   chroni przed uzbrojeniem w trakcie zwykłej jazdy.
3. Próg ustawiony na 60,0 kg jest dozwolony i po prostu nigdy nie uzbraja — szczyt jest
   obcinany do pełnej skali, więc warunek „szczyt > próg" nie może zajść. Wzór wychodzi zerem
   przed dzieleniem przez zerowy span.

### 16.3. Odstępstwa od planu

1. **Sekcja 5 — `throttle_active` nie istnieje w wejściu modułu.** Plan wymagał, żeby manetka
   nie była źródłem zapamiętanego prądu. Jest to spełnione *strukturalnie*: moduł liczy prąd
   z nacisku w kg, a `ride_control` woła go na celu wyłącznie pedałowym, przed dodaniem
   manetki. Blokowanie uzbrojenia flagą manetki byłoby dodatkowo błędne — test stanowiskowy 9
   zakłada jazdę jednocześnie na manetce i pedałach.
2. **Sekcja 4.5 — poziom 0 raportuje przyczynę 7 (`level_or_bank_change`), nie osobny kod.**
   Kod 1 (`disabled`) musi jednoznacznie znaczyć „wyłączone w konfiguracji".
3. **Wykres podglądu karty UI** ma inną formę niż pozostałe: dwa panele obok siebie (nacisk →
   prąd, oraz czas po zatrzymaniu korb), zgodnie z zasadą z `engine-preview-ui.js`, że dwie
   różne jednostki nie dzielą jednej ramki.

### 16.4. Znane ograniczenia weryfikacji

- **Testy hostowe w C nie zostały uruchomione** na tej maszynie: nie ma kompilatora
  hostowego (tylko `arm-none-eabi-gcc`). Runner kompiluje wtedy harness krzyżowo i kończy się
  kodem 2 = SKIPPED, nigdy „PASS". Harness **kompiluje się i linkuje** z `-Wall -Wextra
  -Werror`, co dowodzi poprawności C i tego, że moduł nie wymaga żadnych stubów, ale nie
  dowodzi zachowania. Żeby je uruchomić, wystarczy MinGW-w64, LLVM albo MSVC.
- Pokrycie `assist_modes_profile_iq_ceiling()` jest na razie modelem w teście JS plus
  asercjami strukturalnymi — ten plik ciągnie zbyt wiele zależności, żeby wejść do harnessu
  bez stubów.

### 16.5. Do zrobienia (kolejne kroki z sekcji 14)

- [ ] punkt 10: build diagnostyczny i test z kołem uniesionym
- [ ] punkt 11: test obciążeniowy, potem zamknięty test terenowy (sekcja 12)
- [ ] punkt 12: numer buildu w CHANGELOG po akceptacji, instrukcja użytkownika
- [ ] testy odbiorowe z §4 handoffu: limity prądu/mocy, klasyfikacja przy 3/6/7/10 km/h
      (legal i offroad), zapis banku w trakcie ACTIVE

Firmware **nie został zbudowany** — zgodnie z ustaleniem `build_firmware.ps1` uruchamiany jest
wyłącznie na wyraźne polecenie.
