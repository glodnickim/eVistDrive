# FW-074 — Assist Carry Over: kontrolowane dociągnięcie po zakończeniu pedałowania

- **Data:** 2026-07-31
- **Status:** PLAN DO WDROŻENIA — funkcji jeszcze nie ma w Ride Core ani w Canable.
- **Zakres:** firmware Ride Core, bank blob, CANable Profiles, testy automatyczne i stanowiskowe.
- **Powiązane:** `FW-072_SINGLE_RELEASE_RAMP.md`, `FW-025_PAS_STOP_CUT.md`,
  `FW-048_COAST_RELEASE.md`, `FW-069_RAMPS_PER_LEVEL.md`.

## 1. Cel i decyzja

Dodać opcjonalne, krótkie podtrzymanie części wspomagania po tym, jak firmware rozpozna
zatrzymanie korb. Funkcja ma przypominać **Assist Carry Over** spotykane w napędach Shimano:
użytkownik wybiera prosty poziom `Off / Short / Normal / Long`, osobno dla każdego poziomu
wspomagania.

Docelowy przebieg:

```text
PEDALING / ARMED
        |
        | brak impulsu PAS przez około 200 ms
        v
CARRY OVER — krótko utrzymuje ograniczony prąd
        |
        | upłynął czas wybranego presetu
        v
RELEASE — istniejąca pojedyncza rampa release_ms
        |
        v
OFF
```

Najważniejsze decyzje:

1. `PAS_STOP_TICKS` pozostaje detektorem końca pedałowania, a nie czasem overrunu — ta
   zasada się nie zmienia. **Aktualizacja 2026-08-03:** sama wartość progu jest teraz
   adaptacyjna (patrz [[FW-025_PAS_STOP_CUT]], sekcja "Aktualizacja 2026-08-03) —
   `PAS_STOP_TICKS` (200 ms) to niezmieniona DOLNA granica, `PAS_STOP_TICKS_MAX` (500 ms)
   to nowa górna granica używana tylko przy realnie wolnej/nierównej kadencji. Poniższe
   tabele czasów ("≈ 200 ms + carry-over + release") zakładają górny zakres kadencji
   (szybkie/normalne pedałowanie); przy bardzo wolnym pedałowaniu (np. techniczny podjazd)
   ten pierwszy człon może realnie wynosić do 500 ms zamiast 200 ms — do uwzględnienia przy
   wdrażaniu carry-over i jego opisach w UI.
2. `Power fall filter` pozostaje filtrem spadków mocy **w czasie pedałowania**. Nie wolno
   używać go jako overrunu, ponieważ przy utracie ważnego wejścia wspomagania jego stan jest
   zerowany przez `stop_power_filter()`.
3. Nie wskrzeszać `EXTENDED_BOOST_ENABLE` i `Overrun_counter` z monolitu Legacy. Zwykła jazda
   korzysta teraz z Ride Core; stary kod ma inne źródła stanu, nie ma nowych ograniczeń i
   mieszałby dwa silniki sterowania.
4. Nowa funkcja podtrzymuje wyłącznie wcześniej legalnie uruchomione **wspomaganie z pedałów**.
   Nie przechwytuje manetki ani Walk Assist i nigdy nie może zwiększyć aktualnego polecenia Iq.
5. Domyślna wartość oraz wartość po migracji starszego banku to `Off`.

To nie jest kopia **Bosch Extended Boost**. Bosch opisuje tę funkcję jako zależny od nacisku
impuls do pokonywania przeszkody. FW-074 jest prostszym, ograniczonym czasowo przeniesieniem
końcowego wspomagania. Ewentualny impuls zależny od siły nacisku byłby osobnym zadaniem i nie
powinien być dokładany do tej samej maszyny stanów.

## 2. Jak działa firmware obecnie

Obecny łańcuch po puszczeniu korb wygląda tak:

```text
ostatnie przejście kwadratury PAS
        |
        | PAS_STOP_TICKS = 800 taktów przy 4 kHz ≈ 200 ms
        v
rider->pedaling_active = false
        |
        v
iq_target = 0
        |
        v
pojedyncza liniowa rampa release_ms -> 0
```

Punkty odniesienia w kodzie:

- `inc/config.h`: `PAS_STOP_TICKS = 800`;
- `src/main.c`: wyzerowanie kadencji i wyliczenie `forward_pedaling`;
- `src/ride_control.c`: obliczenie wspomagania, latch, manetka, safety i limity;
- `src/assist_dynamics.c`: jedna rampa od bieżącego Iq do zera przez `release_ms`;
- `src/assist_modes.c`: `stop_power_filter()` zeruje filtr mocy po zaniku ważnego wejścia.

`release_ms` już może dać odczucie miękkiego dociągnięcia, ale jego prąd maleje od razu. FW-074
dodaje przed nim krótki, jawny etap podtrzymania. Nie zmienia znaczenia `release_ms` ustalonego
w FW-072: jest to nadal pełny czas **jednego** liniowego zejścia do zera, bez ukrytej końcówki.

## 3. Parametr użytkownika i presety

Dodać do `assist_level_config_t` jedno pole:

```c
typedef enum {
    ASSIST_CARRY_OVER_OFF = 0,
    ASSIST_CARRY_OVER_SHORT = 1,
    ASSIST_CARRY_OVER_NORMAL = 2,
    ASSIST_CARRY_OVER_LONG = 3
} assist_carry_over_mode_t;

uint8_t carry_over_mode;
```

W pierwszej wersji nie wystawiać surowego czasu i limitu procentowego. Cztery wartości mają
mapować się w firmware na stałe, ostrożne presety:

| Wartość | Czas podtrzymania od wykrycia PAS STOP | Limit Iq względem limitu Ride Core |
|---|---:|---:|
| `Off` | 0 ms | 0% |
| `Short` | 80 ms = 320 taktów | 15% |
| `Normal` | 150 ms = 600 taktów | 20% |
| `Long` | 250 ms = 1000 taktów | 25% |

To są **wartości startowe EBICS do testów**, a nie odtworzone wartości Shimano lub Bosch.
Producenci nie publikują pełnych tabel czasów i krzywych sterowania.

Limit procentowy jest sufitem, a nie poleceniem osiągnięcia tego prądu. Przykład: jeżeli przy
końcu pedałowania płynęło tylko 8% dostępnego Iq, preset `Normal` ma podtrzymać najwyżej te 8%,
nie podbić prąd do 20%.

## 4. Maszyna stanów

Nowy moduł: `inc/assist_carry_over.h` + `src/assist_carry_over.c`.

Moduł ma być mały i testowalny bez sprzętu. Własne stany wystarczą dwa:

- `ARMED` — podczas jazdy było legalnie uruchomione wspomaganie z pedałów;
- `HOLD` — trwa odmierzane podtrzymanie.

Stan `RELEASE` należy do istniejącego `assist_dynamics`, więc nowy moduł nie może implementować
drugiej rampy.

### 4.1. Uzbrojenie

Uzbrój funkcję tylko wtedy, gdy jednocześnie:

- wybrano `Short`, `Normal` albo `Long`;
- poziom wspomagania jest większy od 0;
- korby obracają się do przodu (`rider->pedaling_active`);
- wspomaganie zostało legalnie uruchomione przez istniejący latch;
- pedal-only `iq_target` był większy od zera;
- nie trwa hamowanie, błąd, kalibracja, Walk Assist ani cofanie korbą.

Moduł może pozostać uzbrojony do końca bieżącego ciągu obrotów korb, ale w każdym takcie ma
aktualizować końcowy `pedal_iq_target`, również gdy spadnie do zera. Dzięki temu nigdy nie
odtworzy starego szczytu z wcześniejszego naciśnięcia.

### 4.2. Wyzwolenie

Wejdź w `HOLD` wyłącznie na zboczu:

```c
previous_pedaling_active && !pedaling_active
```

i tylko gdy moduł był uzbrojony oraz napęd nadal się obraca. W `ride_control.c` wyliczyć jeden
warunek przekazywany do modułu:

```c
bool carry_motion_valid =
    input->speed_x100 >= RIDE_START_REDUCTION_MIN_SPEED_X100 && /* 1.0 km/h */
    rider->motor_erps >= RIDE_COAST_RELEASE_ERPS;               /* 10 erps */
```

Timer jest jednorazowy. Impulsy Halla, prędkość koła ani drgania czujnika nie mogą go
odświeżać.

Prąd przejęty na początku:

```c
carry_iq = min(last_pedal_iq_target,
               max(current_commanded_iq, 0),
               ride_core_iq_limit * preset_cap_pct / 100);
```

`current_commanded_iq` oznacza bieżący setpoint przekazany w `input->current_iq`, nie surowy
pomiar fazy. Użycie minimum gwarantuje, że przejście do `HOLD` nie zrobi skoku prądu w górę.
Jeżeli wynik jest równy 0, nie uruchamiaj podtrzymania — od razu przejdź do zwykłego release.

### 4.3. Działanie HOLD

Podczas `HOLD` moduł zwraca `carry_iq` jako pedal-only target oraz ustawia
`profile_pedaling_active = true`. Dzięki temu `assist_dynamics` nie uruchamia jeszcze
`release_ms`, ale nadal obowiązują zwykłe rampy i wszystkie ograniczenia umieszczone dalej w
torze.

Po upływie dokładnie 320/600/1000 taktów moduł zwraca target 0 i ustawia
`profile_pedaling_active = false`. Od tego taktu istniejący kod FW-072 wykonuje liniowe zejście
przez `release_ms`.

### 4.4. Anulowanie

Stan `ARMED/HOLD` wyzerować bezwarunkowo przy:

- `safety_cut` — hamulec, cofanie, błąd torque, kalibracja torque albo przegrzanie stopnia 2;
- poziomie wspomagania 0;
- wejściu w Walk Assist lub kalibrację położenia silnika;
- zmianie banku/poziomu w trakcie podtrzymania;
- zmianie `carry_motion_valid` na false (prędkość poniżej 1,0 km/h lub ERPS poniżej 10);
- inicjalizacji sterownika.

Anulowanie overrunu ma nastąpić w tym samym takcie sterowania. Nie oznacza to zmiany obecnej
polityki hamulca: po skasowaniu `HOLD` istniejący `safety_cut` nadal wybiera własne wygaszanie
`RIDE_SAFETY_RELEASE_MS = 200 ms`. Zmiana safety fade na hard-cut nie należy do FW-074.

Jeżeli pedałowanie wróci podczas `HOLD`, anuluj timer i natychmiast przekaż z powrotem normalny
pedal-only target. Nie uruchamiaj wtedy release.

## 5. Miejsce wpięcia w `ride_control.c`

Wpiąć moduł w gałęzi Ride Core:

```text
assist_modes_calculate
        -> ride latch / minimum Iq
        -> ASSIST CARRY OVER       <-- nowe miejsce
        -> throttle floor
        -> safety_cut
        -> voltage/speed/temperature/legal limits
        -> smooth start / preload
        -> coast_release
        -> assist_dynamics
        -> motor_core_set_command
```

To położenie jest wiążące:

- za latchem, żeby podtrzymywać tylko wspomaganie, które naprawdę zostało uruchomione;
- przed manetką, żeby nie zapisać i nie odtworzyć prądu manetki;
- przed limitami, żeby limity napięcia, temperatury, prędkości, prądu i trybu legal nadal miały
  ostatnie słowo;
- przed `assist_dynamics`, żeby wykorzystać istniejący `release_ms`, a nie tworzyć drugi fade.

Ścieżki, które wychodzą wcześniej (`position_calibration_active`, Walk Assist), muszą jawnie
wywołać reset modułu. Sam brak wywołania `apply()` nie może zostawić timera na później.

## 6. Bank blob v8 i kompatybilność

Parametr jest per bank i per poziom, więc powinien wejść do bank blob, a nie do globalnego
tuning blob.

Obecny format v6:

```text
13 B header + 5 * 46 B record + 2 B CRC = 245 B
```

Nowy format v8:

```text
13 B header + 5 * 47 B record + 2 B CRC = 250 B
```

Nowy bajt rekordu:

| Offset rekordu | Pole | Typ | Zakres |
|---:|---|---|---:|
| 46 | `carry_over_mode` | `u8` | 0..3 |

Zmiany w firmware:

1. Dodać `BANK_BLOB_VERSION_V8 = 8`, ustawić wersję bieżącą na v8 (v7 zajmuje FW-077 Start condition w kg).
2. Nazwać stare długości jawnie: `BANK_RECORD_LEN_V5 = 35`,
   `BANK_RECORD_LEN_V7 = 46`, `BANK_RECORD_LEN_V8 = 47`.
3. Ustawić `ASSIST_BANK_BLOB_LEN = 250`.
4. Serializer zapisuje `record[46]`.
5. Parser przy `record_len >= 47` przyjmuje tylko 0..3; wartość spoza zakresu zamienia na `Off`.
6. Dla v1-v6 ustawiać `carry_over_mode = Off`, zachowując wszystkie dotychczasowe pola.
7. **Nie zmieniać** sprawdzania pól FW-068/069/FW-077 na `record_len >= BANK_RECORD_LEN_V8`.
   Musi pozostać osobny warunek `record_len >= BANK_RECORD_LEN_V6`, inaczej odczyt banku v6
   straci warunki startu i rampy Iq.
8. Dodać statyczne asercje długości i limitu `<= 255`.

`MotorParams_t.bank_store[2][256]` już mieści blob 250 B. Nie zmieniać rozmiaru tablicy ani
układu `MotorParams_t`; dzięki temu samo wdrożenie v8 nie powinno unieważnić całego rekordu
ustawień. Poprawić przy okazji nieaktualne komentarze o rozmiarach w `inc/main.h`,
`inc/assist_modes.h`, `src/assist_modes.c` i `src/CAN_Display.c`.

### 6.1. Transport wieloramkowy

250 B zajmuje 32 ramki CAN o indeksach 0..31. W dwóch gałęziach odbioru `0x6021` w
`src/CAN_Display.c` zmienić ograniczenie:

```c
if (Ext_ID_Rx.command < 30)   // stare, do ramki 30
```

na:

```c
if (Ext_ID_Rx.command < 31)   // nowe, pozwala dopisać ramkę 31
```

Bufor `BankBlob[256]` wystarcza. Limit protokołu 255 B pozostawia po v8 jeszcze 5 B miejsca w
formacie przewodowym.

## 7. Zmiany w Canable

Repozytorium: `C:\Projekty\bafang_canable_pro`.

### 7.1. Parser i serializer

W `bafang-parser.js`:

- dopuścić wersję banku 8;
- nadal brać stride z `d[5]`;
- odczytać `carry_over_mode: RECORD >= 47 && d[r + 46] <= 3 ? d[r + 46] : 0`.

W `canbus.js`:

- negocjować v8 tylko, gdy odczytany kontroler zgłosił `bank_schema_version >= 8`;
- dla v8 ustawić `RECORD = 47`, `BLOB_LEN = 250` i zapisać offset 46;
- do firmware v6 i starszego nadal wysyłać ich obsługiwany format, bez pola carry-over.

Nie wolno bezwarunkowo wysyłać v8 do starszego firmware — odrzuci blob jako nieobsługiwaną
wersję.

### 7.2. Interfejs

Pole umieścić w:

```text
eVistDrive Profiles
  -> wybrany Bank
  -> wybrany Assist level
  -> Power smoothing and release
```

Nie umieszczać go w globalnej sekcji Dynamics. Każdy poziom może potrzebować innego zachowania.

Etykieta:

```text
Assist carry-over
```

Opcje:

```text
Off (default) / Short / Normal / Long
```

Proponowana treść dymka:

> After forward pedalling is recognized as stopped, briefly holds a capped part of the
> pedal-assist current, then hands off to Release duration. It never increases the current,
> and it does not copy throttle or Walk Assist. Off/Short/Normal/Long use 0/80/150/250 ms with
> 0/15/20/25% current caps. Estimated time from the last PAS transition is about 200 ms +
> carry-over + Release duration. Default: Off. These are experimental EBICS presets.

Uzupełnić też placeholdery profili wartością `carry_over_mode: 0`, obsługę edycji/restore oraz
test zawartości dymków.

## 8. Jak użytkownik ma tego używać

Po wdrożeniu i wgraniu zgodnej pary firmware + Canable:

1. Otwórz `eVistDrive Profiles` i wybierz bank oraz poziom wspomagania.
2. Kliknij `Read banks`.
3. W sekcji `Power smoothing and release` wybierz `Assist carry-over`.
4. Ustaw także `Release duration`; oba czasy sumują się, ale mają inny kształt.
5. Kliknij `Write selected bank (RAM)` i najpierw sprawdź zachowanie bez zapisu trwałego.
6. Jeżeli wynik jest bezpieczny, użyj `Save to Flash` na postoju.

Interpretacja czasu:

```text
od chwili wykrycia PAS STOP = carry-over hold + release_ms
od ostatniego przejścia PAS ≈ 200 ms + carry-over hold + release_ms
```

Przykładowe zestawy startowe:

| Charakter | Carry-over | `release_ms` | Około od ostatniego impulsu PAS do zera* |
|---|---:|---:|---:|
| Szybki / szosowy | Off | 250 ms | 450 ms |
| Naturalny trail | Short 80 ms | 200 ms | 480 ms |
| Zbalansowany | Normal 150 ms | 250 ms | 600 ms |
| Techniczny, tylko po testach | Long 250 ms | 300 ms | 750 ms |

\* `coast_release` może zakończyć prąd wcześniej, jeżeli silnik prawie się zatrzyma.

Ważna różnica kształtu: `Off + 250 ms` od razu zaczyna maleć; `Short + 200 ms` najpierw krótko
podtrzymuje ograniczony prąd, a dopiero potem go wygasza. Podobny łączny czas nie oznacza takiego
samego odczucia.

Nie zaczynać od obecnego domyślnego `release_ms = 650 ms` razem z `Normal` lub `Long`. Dałoby to
około 1,0–1,1 s od ostatniego przejścia PAS do zera. Najpierw skrócić release zgodnie z tabelą.

Przed użyciem poza terenem testowym trzeba osobno sprawdzić wymagania prawne dla danego roweru i
rynku. Ten plan techniczny nie stwierdza, że podtrzymanie po ustaniu pedałowania jest dopuszczone
w ruchu drogowym.

## 9. Testy automatyczne

### 9.1. Firmware — moduł stanu przy 4 kHz

1. `Off` daje taki sam przebieg wyjścia jak firmware bez FW-074.
2. HOLD uruchamia się tylko na zboczu `true -> false` po wcześniejszym legalnym uruchomieniu.
3. Czasy trwają dokładnie 320, 600 i 1000 taktów od zbocza detektora PAS.
4. Wynik jest ograniczony przez ostatni pedal-only target, bieżący setpoint oraz procent limitu.
5. Przejście do HOLD nigdy nie zwiększa polecenia Iq.
6. Hamulec, cofanie, błąd torque, przegrzanie, poziom 0, Walk i kalibracja kasują HOLD w tym
   samym takcie.
7. Manetka nie uzbraja funkcji i nie jest przechwytywana przy końcu pedałowania.
8. Ponowne pedałowanie kasuje HOLD i oddaje sterowanie normalnemu assist bez wejścia w release.
9. Hall/speed nie przedłużają jednorazowego timera.
10. Po HOLD dokładnie jedna istniejąca rampa używa `release_ms`.
11. Limity prądu, mocy, napięcia, temperatury, prędkości i legal nadal mogą obniżyć target.
12. `coast_release` może zakończyć końcówkę wcześniej i nie powoduje ponownego startu.

### 9.2. Protokół

1. Parser firmware nadal przyjmuje poprawne v1-v6.
2. V6 zachowuje pola z offsetów 35..45 i tylko dodaje `carry_over_mode = Off`.
3. V8 ma 250 B, rekord 47 B, poprawny CRC i round-trip wszystkich pięciu poziomów.
4. Odbiornik przyjmuje ostatnią ramkę o indeksie 31.
5. Canable nigdy nie wysyła v8 do kontrolera, który zgłosił maksymalnie v7.
6. Restore/copy level/copy section zachowują `carry_over_mode`.

W Canable rozszerzyć istniejące testy round-trip v7 i dodać osobny test v8. V7 powinno pozostać
niezmienione bajt w bajt.

## 10. Test stanowiskowy i kryteria odbioru

Testować najpierw z kołem uniesionym, potem pod kontrolowanym obciążeniem, a dopiero na końcu na
zamkniętym odcinku:

1. Zmierzyć czasy z logu `iq_setpoint`, osobno od ostatniego PAS i od zbocza
   `pedaling_active=false`.
2. Sprawdzić `Short/Normal/Long` przy małym i dużym końcowym Iq.
3. Wcisnąć hamulec w każdej fazie; HOLD ma zniknąć natychmiast, bez czekania na swój timeout.
4. Sprawdzić cofnięcie korby, zmianę na poziom 0, zmianę banku, zanik ruchu i powrót pedałowania.
5. Jechać jednocześnie na manetce i pedałach; po puszczeniu korb funkcja nie może skopiować
   wyższego prądu manetki.
6. Sprawdzić zmianę biegu i bardzo niskie ERPS — brak kliknięcia, ponownego szarpnięcia i
   niespodziewanego restartu po osiągnięciu zera.
7. Potwierdzić, że każdy limit nadal redukuje prąd w HOLD.

Kryteria wydania:

- domyślnie `Off` w nowych i migrowanych bankach;
- brak regresji przebiegu przy `Off`;
- brak wzrostu Iq na wejściu do HOLD;
- pełne testy safety i kompatybilności v7/v8;
- wersja testowa/diagnostyczna przed normalnym buildem;
- aktualizacja `CHANGELOG.md`, dokumentacji użytkownika i dymków Canable.

## 11. Kolejność prac dla developera

1. Dodać enum, pole per-level, domyślne `Off` i bank blob v8 wraz z migracją.
2. Naprawić limit ramki 31 i komentarze o rzeczywistych długościach.
3. Dodać czysty moduł `assist_carry_over` i jego testy jednostkowe.
4. Wpiąć moduł za latchem, przed manetką; dodać jawne resety na wszystkich ścieżkach wyjścia.
5. Rozszerzyć telemetrię diagnostyczną co najmniej o flagę `carry_over_active` i pozostałe takty
   HOLD. Bez tej informacji trudno odróżnić błąd timera od zwykłego `release_ms`.
6. Zaktualizować parser, serializer, model, UI i dymki Canable.
7. Uruchomić testy v1-v8, testy Ride Core oraz build normalny i diagnostyczny.
8. Wykonać procedurę stanowiskową; wartości presetów zmieniać dopiero na podstawie logów.

## 12. Referencje producentów

- Shimano: [E-TUBE Project Professional](https://bike.shimano.com/en-UK/products/apps/e-tube-project-professional.html)
  i [katalog systemów E-BIKE 2026](https://si.shimano.com/en/pdfs/sm/2026_E-BIKE_CAT_GB/SM-2026_E-BIKE_CAT-000-ENG-GB.pdf)
  opisują konfigurowalne Assist Carry Over i trzy poziomy, ale nie publikują kompletnego algorytmu.
- Bosch: [Extended Boost w smart system](https://help.bosch-ebike.com/#/help-center/ebw-flowapp/asset-ast-00375)
  opisuje zależne od nacisku wsparcie przy przeszkodzie; jest to inny przypadek użycia niż
  konserwatywne podtrzymanie z FW-074.
