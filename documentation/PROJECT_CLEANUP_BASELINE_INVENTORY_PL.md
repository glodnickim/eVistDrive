# Punkt bazowy i inwentaryzacja przed porządkami

> **STATUS:** MIGAWKA FAZY 0.
>
> Ten dokument jest dowodem pomocniczym dla
> [nadrzędnego planu porządków](PROJECT_CLEANUP_MASTER_PLAN_PL.md). Nie zastępuje
> planu ani changelogu. Aktualizujemy go, gdy zostanie wybrany nowy punkt bazowy
> albo zmieni się klasyfikacja istniejących plików.

Data migawki: 2026-07-31.

## 1. Zakres porządków

Publicznym targetem bieżącej gałęzi porządków jest:

```text
Sterownik: M820 / GD32F303RCT6
Bootloader: BL820
Początek aplikacji: 0x08005000
Gałąź robocza: cleanup/publication
```

Kod i historia innych bootloaderów/kontrolerów pozostają w repo do późniejszej
decyzji. Nie są usuwane w Fazie 0, ale nie mogą wymuszać ryzykownej zgodności
kosztem poprawności wydania M820/BL820.

## 2. Rozróżnienie punktów bazowych

Dokumentacja używała określenia „sprawdzony build” dla różnych poziomów
weryfikacji. Od teraz rozdzielamy:

1. **Ride-confirmed** — przejechany na rowerze i potwierdzający podstawową jazdę.
2. **Hardware-exercised** — uruchomiony na sprzęcie, ale test wykazał znany
   problem; nie jest bezpiecznym release.
3. **Build candidate** — kompiluje się i przechodzi testy hostowe, ale wymaga
   potwierdzenia na sprzęcie.
4. **Diagnostic build** — służy do obserwacji; nie jest kandydatem normalnego
   wydania.

### 2.1 Zidentyfikowane artefakty

| Build | Klasa | Rozmiar | SHA-256 lokalnego artefaktu | Wniosek |
|---|---|---:|---|---|
| `0.0199_M820_BL820.bin` | wcześniejszy znany-dobry bazowy ride core | 83 000 B | `C4D3B8D82A0A3BB8E8AA07908827C52C022C30405A2D90834E7B4FC5999B58C5` | Punkt odniesienia działania silnika, nie aktualnego formatu funkcji |
| `0.0206_M820_BL820.bin` | ride-confirmed | 84 012 B | `1AA11E6200E7A38CCA547BDE91E98D3C3C69058063705C854FE4E8F6D768989B` | Najpóźniejszy jednoznacznie opisany stan przejechany na rowerze; miał nadal otwarte strojenia |
| `0.0252_M820_BL820.bin` | hardware-exercised | 91 680 B | `ABC6B342AAA7A10C4294BF3EC1F931046F48F83141AFF5BECCDDE0DAF274A998` | Test ujawnił fałszywy `STALL` i problem wybiegu; nie oznaczać jako znany-dobry |
| `0.0256_M820_BL820.bin` | starszy build candidate | 92 120 B | `10A82FFDDF62A42B241989C5FAD50647B5B62B9C124CFE99F781DF4C5A7DB7C9` | Build i test hostowy przechodzą; zastąpiony do testu przez normalny build `0.0258` |
| `0.0257_M820_BL820.bin` | lokalny diagnostic build | 92 120 B | `338E4F2FEF095705815151FEF3DDC8ABCDB3084D191D6FA4BABEEFBB9324FCE9` | Nie używać jako normalnego release |
| `0.0258_M820_BL820.bin` | hardware-exercised, odrzucony jako known-good | 88 572 B | `13CD342CE86B5086A6A963DB51268A227189694B7EBCEE717B7E106CDF189FD4` | Zwykłe wspomaganie działa jak we wcześniejszych buildach, ale WA nie stabilizuje RPM: obroty stale narastają, a operator musi przerwać tryb; ok. 15 km/h było dopiero odcięciem bezpieczeństwa |
| `0.0259_M820_BL820.bin` | hardware-exercised, odrzucony przez szarpanie WA | 88 588 B | `27CAF8B7ACE449B5942816806CDDF78F31EE206A9D111D3625C8C7E41996FC7D` | Ograniczył wcześniejsze rozpędzanie, ale przełączanie podłogi `5/0 Iq` powodowało szarpanie przy niskiej prędkości |
| `0.0259-diag_M820_BL820.bin` | build diagnostyczny, nie do normalnej jazdy | 93 128 B | `E7E2C5872FD66F6DAC074BFACA29D350C2D4D9828687C63773CE756E02A915AF` | Ten sam algorytm FW-062, diagnostyka CAN włączona; użyć tylko jeśli normalny test stojakowy nadal wykaże problem |
| `0.0260_M820_BL820.bin` | hardware-exercised, odrzucony przez agresywne cykle WA | 88 588 B | `56BD59EDF2CC5264090CC4CD96BEC4255A2D2EFBB3FD48A28A97A2B0E7287304` | Usunął dawny skok podłogi, ale zbyt mocno doganiał cel, przestrzeliwał i po kilku cyklach przestawał działać do puszczenia przycisku; objaw odpowiadał `STALL`, bez potwierdzenia ramką |
| `0.0260-diag_M820_BL820.bin` | historyczny build diagnostyczny, nie do normalnej jazdy | 93 128 B | `F5809890A60DA3BB824423F4808709930AA25AA1D481314A6027F7F6A17E17AB` | Odpowiada odrzuconemu `0.0260`; zastąpiony do ewentualnego logowania przez `0.0261-diag` |
| `0.0261_M820_BL820.bin` | zastąpiony przed testem sprzętowym | 88 588 B | `1DE53EA14AC227F88D74DA34E387323A455CC00E989D4E6239EEA33F42C850EE` | Złagodził wartości docelowe, ale dodatnie rampy uznano za nadal zbyt szybkie przed wgraniem; nie klasyfikować jako hardware-rejected |
| `0.0261-diag_M820_BL820.bin` | historyczny build diagnostyczny, nie do normalnej jazdy | 93 128 B | `C1006CFFED686F9BC392634DD11A4823B47F730E6EA732D7C21384160686D39D` | Odpowiada niewgranemu `0.0261`; zastąpiony przez `0.0262-diag` |
| `0.0262_M820_BL820.bin` | zastąpiony przed testem sprzętowym | 88 588 B | `2A4ABA23D2DF0F546C33B0ABBBAA6CF8160495CDF0A16D70185C0433A03D5477` | FW-065 z długimi rampami; zastąpiony przed wgraniem po doprecyzowaniu wymagań RUN `Iq_min..Iq_max` i odcięcia dopiero przy 80–90 rpm; nie klasyfikować jako hardware-rejected |
| `0.0262-diag_M820_BL820.bin` | historyczny build diagnostyczny, nie do normalnej jazdy | 93 128 B | `E403AB0E7722C742E093018D5A3ECB0FFD41299CCFAC12F0422B8FCB5B270E4B` | Odpowiada niewgranemu `0.0262`; zastąpiony przez `0.0263-diag` |
| `0.0263_M820_BL820.bin` | zastąpiony przed testem sprzętowym | 88 772 B | `AB4CC37872E48F8C97429574A4471B4C876A680703962F504BAE9B7E23D99447` | FW-066 z START 80 Iq, RUN 5..36 Iq i stałym COAST 85/70 rpm; zastąpiony przed wgraniem po decyzji o progach zależnych od celu |
| `0.0263-diag_M820_BL820.bin` | historyczny build diagnostyczny, nie do normalnej jazdy | 93 312 B | `BDCEC56A28C7D0A9872D8589A9144E81B9C0A538BDEC3F4D8F56C153BC637437` | Odpowiada niewgranemu `0.0263`; zastąpiony przez `0.0264-diag` |
| `0.0264_M820_BL820.bin` | bieżący kandydat FW-067, test stojakowy oczekuje | 88 844 B | `438CC4E68712586112C575DFC98352A3D1DF5FEB7C25550AE5BC198269B85CC7` | Zachowuje START/RUN FW-066, ale COAST wylicza jako `target+20/+5 rpm`; testy 5/5 PASS, CAN diag OFF |
| `0.0264-diag_M820_BL820.bin` | build diagnostyczny, nie do normalnej jazdy | 93 384 B | `584A0BE4667C3AAEFE8901AF92F6352D62CE7F3A1E782D339DB416F60E6366DB` | Diagnostyka CAN ON; użyć wyłącznie do logu, jeżeli normalny `0.0264` nie przejdzie stojaka |

Hash potwierdza zawartość lokalnego pliku, ale nie zastępuje powiązania z czystym
commitem. Aktualny worktree jest zmodyfikowany, dlatego `0.0256–0.0264` nie mają
jeszcze odtwarzalnego źródła w Git. Kontrola symboli ELF `0.0258` nie znalazła
`print_debug_on_CAN`, `diag_peak`, `debug_can` ani `can_diag`.

### 2.2 Kandydat do zamrożenia

Przed pierwszą zmianą zachowania:

- [x] wgrać normalny build `0.0258`;
- [x] wgrać normalny build `0.0259`; test ujawnił szarpanie WA;
- [x] wgrać normalny build `0.0260`; test ujawnił agresywne cykle i wyłączenie
      odpowiadające zatrzaskowi WA, bez potwierdzenia ramką;
- [x] wycofać `0.0261` przed wgraniem po decyzji o dłuższych rampach;
- [x] wycofać `0.0262` przed wgraniem po doprecyzowaniu docelowego działania;
- [x] wycofać `0.0263` przed wgraniem po decyzji o dynamicznym progu COAST;
- [ ] wgrać normalny build `0.0264`;
- [ ] wykonać procedurę z `FW-060_WA_CONSTANT_RPM_CONTROLLER.md`;
- [x] potwierdzić zwykłą jazdę ride core i pozostałe funkcje — użytkownik zgłosił
      zachowanie takie jak we wcześniejszych firmware;
- [ ] potwierdzić Walk Assist na stojaku i na ziemi;
- [ ] odczytać bank 1, bank 2, tuning, torque calibration i Hall;
- [ ] zapisać log CAN oraz eksport ustawień;
- [ ] dopiero wtedy oznaczyć build jako znany dobry dla migracji.

Do czasu tego testu:

- `0.0206` jest referencją potwierdzonej jazdy;
- `0.0258` jest historyczną referencją formatu konfiguracji i
  reprodukowalnego błędu rozpędzania WA, ale nie jest buildem znanym jako dobry;
- `0.0259` jest odrzuconym kandydatem FW-062: test stojakowy ujawnił szarpanie
  przy znacznie niższej prędkości;
- `0.0260` jest odrzuconym kandydatem FW-063: test ujawnił agresywne
  doganianie, przestrzał i wyłączenie zgodne z zatrzaskiem po kilku cyklach;
- `0.0261` został zastąpiony przed testem i nie ma wyniku sprzętowego;
- `0.0262` został zastąpiony przed testem i nie ma wyniku sprzętowego;
- `0.0263` został zastąpiony przed testem i nie ma wyniku sprzętowego;
- `0.0264` jest bieżącym kandydatem FW-067 i oczekuje testu stojakowego;
- żaden z nich samodzielnie nie spełnia pełnej roli finalnego punktu bazowego.

### 2.3 Manifest normalnego builda `0.0258`

Build wykonano na gałęzi `cleanup/publication`, przy HEAD `992c910` i
zmodyfikowanym worktree opisanym w sekcji 3. Użyto komendy:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command `
  "& '.\build_firmware.ps1' -ArtifactName '0.0258' -BootloaderMode ''"
```

Pusty `BootloaderMode` jest celowy: wykorzystuje już ustawiony target M820/BL820
i nie uruchamia funkcji skryptu, która przepisuje `config.h` oraz linker script.
Brak `-CanDiagnostics` ustawia `CAN_DIAGNOSTICS_ENABLE=0`.

Wynik:

```text
text:       88 280 B
data:          256 B
bss:         6 352 B
Flash BIN:  88 536 B
RAM:         6 608 B (.data + .bss)
BIN BL820:  88 572 B
```

Wszystkie obecne testy hostowe przeszły:

```text
tests/fw016_ride_core_model.ps1       PASS
tests/fw056_power_curve.js            PASS
tests/fw057_cadence_comp.js           PASS
tests/fw058_coast_rezero.js           PASS
tests/fw060_walk_speed_controller.js  PASS
```

W `fw056_power_curve.js` porównanie z modułem Canable zostało pominięte, ponieważ
sąsiedni moduł nie był dostępny. To znane ograniczenie testu, a nie jego awaria.

Build nadal zgłasza istniejące ostrzeżenia:

- niezgodność znaku wskaźników `char *`/`uint8_t *` w `CAN_Display.c`;
- nieużywane `fw_ver` w `main.c`;

Nie są to nowe ostrzeżenia dodane przez Fazę 0. Muszą zostać usunięte lub jawnie
rozstrzygnięte w Fazie 8 przed publicznym wydaniem.

### 2.4 Odtworzenie `0.0258` nowym procesem

Śledzony skrypt `scripts/build-firmware.ps1` zbudował wariant
`M820_BL820/debug/normal` z wersją `0.0258`. Wynik BL820 ma ponownie 88 572 B i
identyczny SHA-256:

```text
13CD342CE86B5086A6A963DB51268A227189694B7EBCEE717B7E106CDF189FD4
```

Oznacza to brak zmian bajtów firmware po przeniesieniu procesu builda i
zabezpieczeniu mapy. Nowy linker raportuje:

```text
Flash image end: 0x0801A9D8
Flash limit:     0x0803E800
RAM z heap/stack: 6 600 B
RWE: brak
```

Dokładna instrukcja oraz znaczenie profili znajdują się w
[`BUILD_FIRMWARE.md`](BUILD_FIRMWARE.md).

## 3. Stan Git podczas inwentaryzacji

```text
Gałąź: cleanup/publication
HEAD: 992c910
Zmodyfikowane śledzone pliki: 22
Nieśledzone pozycje przed utworzeniem planów: 49
Nieśledzone pozycje po utworzeniu trzech planów i `.gitattributes`: 53
```

### 3.1 Zmodyfikowane pliki śledzone

Dokumentacja i changelog:

```text
CHANGELOG.md
documentation/EBICS_ZMIANY_I_KONFIGURACJA_PL.md
documentation/FW-022_HALL_CALIBRATION_PERSISTENCE.md
documentation/README.md
documentation/WALK_ASSIST_DZIALANIE.md
```

Nagłówki projektu:

```text
inc/CAN_Display.h
inc/assist_dynamics.h
inc/assist_modes.h
inc/config.h
inc/main.h
inc/ride_control.h
inc/rider_input.h
inc/torque_input.h
inc/tuning_config.h
```

Implementacja:

```text
src/CAN_Display.c
src/assist_dynamics.c
src/assist_modes.c
src/main.c
src/parser.c
src/ride_control.c
src/torque_input.c
src/tuning_config.c
```

Klasyfikacja: **ZACHOWAĆ, W TOKU, WYMAGA CHECKPOINTU.** Zmiany obejmują wiele
kart FW-023…FW-061 i są wzajemnie splecione. Nie wolno ich resetować ani
próbować automatycznie dzielić według nazw plików.

`inc/main.h` ma dodatkowo dużą różnicę wynikającą z zakończeń linii. Po
zignorowaniu EOL pozostają tylko trzy rzeczywiste zmiany. Dodano
`.gitattributes`, ale celowo nie wykonano jeszcze masowej normalizacji drzewa,
aby nie mieszać zmian EOL ze zmianami funkcjonalnymi.

### 3.2 Nowe źródła i nagłówki

```text
inc/cadence_comp.h
inc/level_gesture.h
inc/power_curve.h
inc/power_curve_lut.h
inc/walk_assist_motor.h
inc/walk_speed_controller.h
src/cadence_comp.c
src/level_gesture.c
src/power_curve.c
src/walk_assist_motor.c
src/walk_speed_controller.c
```

Klasyfikacja: **KOD PRODUKTU — ZACHOWAĆ I OBJĄĆ TESTAMI.**

Główne grupy:

- Power Curve i generator LUT: FW-056;
- kompensacja kadencji: FW-057;
- gesty poziomu/banku: FW-050;
- regulator i ochrona Walk Assist: FW-060.

Brak tych plików w commicie spowoduje, że zmodyfikowany kod śledzony nie będzie
kompletny po klonowaniu.

### 3.3 Nowe testy i narzędzia

```text
tests/fw056_power_curve.js
tests/fw057_cadence_comp.js
tests/fw058_coast_rezero.js
tests/fw060_walk_speed_controller.js
tools/generate_power_curve_lut.js
```

Klasyfikacja: **ZACHOWAĆ.** Są częścią dowodu działania nowych modułów.

Do poprawy później:

- wspólny runner;
- wykonywanie kodu C zamiast części kopii modelu;
- jawna integracja z Canable;
- usunięcie zakodowanej ścieżki do sąsiedniego repo z generatora.

### 3.4 Nowa dokumentacja FW-023…FW-061

32 karty dokumentują storage, bezpieczeństwo PAS/torque, przejście do ride core,
Walk Assist, Power Curve, kompensację kadencji i diagnostykę.

Klasyfikacja: **ZACHOWAĆ JAKO DOWÓD/HISTORIĘ, ALE NIE WSZYSTKIE JAKO AKTUALNE
ŹRÓDŁO PRAWDY.**

Przed publikacją każda karta otrzyma jeden status:

- `AKTUALNY`;
- `PLAN`;
- `WDROŻONE — HISTORIA DECYZJI`;
- `ARCHIWALNY → <następca>`.

Nie należy usuwać wyników testów sprzętowych. Należy natomiast usunąć
sprzeczności typu „czeka na test”, gdy późniejszy dokument potwierdza wykonanie.

### 3.5 Dokumenty planu porządków

```text
documentation/PROJECT_CLEANUP_MASTER_PLAN_PL.md
documentation/PLAN_FLASH_CONFIG_STORAGE_EXPANSION_PL.md
documentation/PROJECT_CLEANUP_BASELINE_INVENTORY_PL.md
```

Klasyfikacja: **AKTYWNE DOKUMENTY DEVELOPERSKIE — DODAĆ DO GIT W ODDZIELNYM
COMMICIE DOKUMENTACYJNYM.**

Przy commitowaniu `documentation/README.md` trzeba uważać, ponieważ zawiera
również wcześniejsze zmiany użytkownika niezwiązane wyłącznie z planami.

### 3.6 Artefakt lokalny

```text
WA_sterowanie_0.0229.zip
```

Klasyfikacja: **NIE DODAWAĆ DO REPO ŹRÓDŁOWEGO.**

Pozostawić lokalnie do czasu potwierdzenia, czy zawiera unikalne dane. Jeżeli ma
wartość historyczną, opisać zawartość i przenieść do prywatnego archiwum albo
wydania, nie do głównego drzewa źródeł.

## 4. Proponowana kolejność zabezpieczenia istniejących zmian

1. Wgrać normalny `0.0264` i wykonać wyłącznie test stojakowy WA na najniższym
   biegu; nie wykonywać testu WA na ziemi.
2. Zapisać eksport ustawień i log.
3. `.gitattributes` jest dodany; nie normalizować jeszcze całego drzewa.
4. Utworzyć checkpoint bieżącego kodu na `cleanup/publication`, wykluczając ZIP
   i niejasne binaria.
5. W osobnym commicie dodać aktywne plany developerskie.
6. Dopiero od checkpointu wykonywać małe commity bezpieczeństwa/refaktoru.
7. Przed publicznym merge uporządkować lub świadomie zachować checkpoint; nie
   ukrywać pochodzenia wcześniejszego kodu.

Nie tworzyć teraz sztucznego „czystego” commita przez resetowanie plików. Celem
checkpointu jest zabezpieczenie pracy, nie udawanie finalnej historii wydania.

## 5. Brakujące dowody Fazy 0

- [ ] eksport obu banków;
- [ ] eksport globalnego tuningu;
- [ ] eksport torque calibration;
- [ ] odczyt Halla i `param_record_state`;
- [ ] log testu normalnej jazdy;
- [ ] log testu Walk Assist;
- [x] decyzja o `0.0258`: nie może być punktem known-good; pozostaje
      reprodukowalną bazą do diagnozy i naprawy WA;
- [ ] powiązanie finalnej binarki bazowej z commitem czystego drzewa.
