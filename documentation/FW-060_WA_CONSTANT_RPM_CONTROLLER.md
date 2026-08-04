# FW-060 / FW-079 / FW-080 / FW-081 / FW-082 — regulator prędkości Walk Assist

- **Aktualizacja:** 2026-08-04
- **Aktualny kandydat:** FW-082, normalny firmware `0.0282`
- **Status:** testy hostowe 7/7 i oficjalny build zakończone powodzeniem; test
  sprzętowy szybkości reakcji i podtrzymania Halla oczekuje
- **Zakres:** M820/BL820; bez zmiany formatu banku, Canable, zwykłego
  wspomagania i autokalibracji Halla

## 1. Potwierdzona przyczyna problemu 30/50 rpm

Nastawa była przekazywana poprawnie:

```text
Canable/bank -> assist_modes_get_wa_target_rpm()
             -> walk_motor_input.target_chainring_rpm
             -> rpm_to_erps()
```

Dla M820 obowiązuje:

```text
motor ERPS = chainring rpm × 4 / 3
30 rpm -> 40 ERPS
50 rpm -> 67 ERPS
```

Pomiar Halla również miał właściwe jednostki: TIMER2 pracuje z częstotliwością
500 kHz, a estymator liczy ERPS z sześciu przejść Halla na obrót elektryczny i
średniej maksymalnie 12 okresów.

Błąd znajdował się na wyjściu regulatora. FW-074 wymuszał:

```c
#define WA_MOTOR_RUN_MIN_IQ 5
desired_iq = max(pi_iq, startup_iq, iq_floor);
```

Podłoga `5 Iq` była aktywna po zakończeniu START przy każdej prędkości, także
daleko ponad celem. Na lekkim napędzie stały dodatni moment dalej dodawał energię.
30 i 50 rpm zbiegały więc do podobnej prędkości wyznaczonej przez opory.

Regresja testowa FW-074 utrwalała błąd: wymagała `5 Iq` przy pomiarze aż
`160 ERPS` oraz akceptowała około `92 ERPS` dla lekkiego napędu z celem `67 ERPS`.
PASS tego testu nie oznaczał kontroli prędkości.

## 2. Decyzja FW-079

1. Usunąć `WA_MOTOR_RUN_MIN_IQ` i pole `iq_floor` z interfejsu regulatora.
2. Pozostawić jednorazowy START 80 Iq, ponieważ jest potrzebny do ruszenia
   obciążonego roweru pod górę.
3. RUN ma regulować w pełnym zakresie `0..36 Iq`.
4. Powyżej celu PI może zejść do zera przez istniejącą rampę; nie dodawać
   twardego governora ani progów `target+20/+5 rpm`.
5. Nie uzbrajać START ponownie po wybiegu ani utracie Halla.
6. Po zaniku Halla zachować ograniczony i powolny `REACQUIRE`, aby podjąć wirnik
   bez skoku prądu.
7. Zmniejszyć deadband z `±7` do `±2 ERPS`, żeby nastawa odpowiadała prędkości
   zębatki z dokładnością około `±1,5 rpm`, a nie `±5,25 rpm`.

## 3. Decyzja FW-081

1. Normalny RUN nie schodzi do zera, lecz zachowuje `1 Iq`, aby podtrzymać
   obrót wirnika i impulsy Halla.
2. Podłoga nie działa w żadnym stanie bezpieczeństwa. Hamulec, puszczenie WA,
   fault, limit koła, `LIMIT` i `STALL` nadal mają prawdziwe `0 Iq`.
3. START nie ma już trajektorii 18→80 Iq. Jedyny cel i twardy sufit to `30 Iq`.
4. START kończy się po pierwszym wiarygodnym filtrowanym ruchu `2 ERPS`, a nie
   po osiągnięciu 30% bankowego celu.
5. Obie próby odzyskania są ograniczone do `24 Iq` i nie uzbrajają START.
6. Watchdog rozpoznaje istotny prąd od `24 Iq`, żeby nadal chronić zablokowany
   silnik po obniżeniu sufitu START.

## 4. Decyzja FW-082

1. Po pozytywnym kierunku testu `0.0280` podnieść START z `30` do `40 Iq`.
2. Kończyć START przy `8 ERPS`, czyli około `6 rpm` zębatki. Dla przełożenia
   36/48 i koła 29 cali jest to około `0,63 km/h`.
3. Podnieść normalny zakres RUN z `1..36` do `2..40 Iq`.
4. Przyspieszyć wyłącznie odpowiedź na wzrost obciążenia: dodatnią rampę RUN
   z `15,625` do `31,25 Iq/s` i dodatni krok całki PI z `1` do `2` w Q8.
5. Pozostawić spadek `31,25 Iq/s`, odzyskiwanie Halla do `24 Iq`, progi
   watchdoga `24/24 Iq` oraz wszystkie odcięcia bezpieczeństwa bez zmian.

## 5. Parametry regulatora

### START — jeden raz na przytrzymanie WA

| Parametr | Wartość |
|---|---:|
| pojedynczy cel i twardy sufit | `40 Iq` |
| rampa wyjścia | `93,75 Iq/s` |
| czas dojścia od 0 do 40 Iq | około 0,42 s, jeżeli próg ruchu nie pojawi się wcześniej |
| koniec START | pierwszy wiarygodny filtrowany pomiar `8 ERPS` |
| maksymalne zasianie całki | `8 Iq` |

START nie wraca po przestrzale, wybiegu, REACQUIRE ani ograniczeniu prądu.
Ponownie uzbraja go dopiero pełne zakończenie zadania WA.

### RUN

| Parametr | Wartość |
|---|---:|
| zakres wyjścia | `2..40 Iq` |
| rampa narastania | `31,25 Iq/s` |
| rampa opadania | `31,25 Iq/s` |
| deadband PI | `±2 ERPS`, około `±1,5 rpm` zębatki |
| ograniczenie błędu PI | `±12 ERPS` |
| Kp | `1 Iq/ERPS` |
| Ki | dodatni `error × 2`, ujemny `error × 8` w Q8 na krok PI |
| częstotliwość PI | 200 Hz |

Sufit `40 Iq` pozostaje miękki. Podłoga `2 Iq` obowiązuje tylko w normalnym RUN
z ważnym Hallem. Hamulec, fault, puszczenie WA i limity bezpieczeństwa mogą nadal
odciąć natychmiast do prawdziwego zera.

W teście maksymalnego dodatniego błędu regulator osiąga około `30 Iq` po 1 s
i pełne `40 Iq` po `1,51 s`. Jest to wynik całego toru PI i rampy wyjściowej,
nie tylko teoretyczny czas przejścia rampy między dwoma już zadanymi wartościami.

### REACQUIRE

FW-082 zachowuje dwa czasy zaniku Halla, ale oba ogranicza do 24 Iq:

| Przypadek | Sufit | Rampa | Timeout | Czas do sufitu / zapas |
|---|---:|---:|---:|---:|
| nieoczekiwany zanik przy małym prądzie | `24 Iq` | `31,25 Iq/s` | `1,5 s` | około `0,75 / 0,75 s` |
| zatrzymanie mimo podłogi RUN 2 Iq | `24 Iq` | `31,25 Iq/s` | `4 s` | około `0,70 / 3,30 s` |

W obu przypadkach:

- START pozostaje zakończony;
- całka jest zerowana;
- model daje około `6 Iq` po 200 ms, bez skoku prądu;
- powrót Halla natychmiast oddaje sterowanie PI;
- brak ruchu prowadzi do `LIMIT`, a następnie zatrzaśniętego `STALL`.

W `0.0277` zwykły timeout `1,5 s` był krótszy niż około `1,54 s` potrzebne
starej rampie do dojścia od 0 do 24 Iq. Firmware mógł więc sam wejść w
`LIMIT/STALL`, zanim próba odzyskania osiągnęła swój limit. FW-080 usuwa tę
sprzeczność czasową. Zamiar wybiegu zapisuje już przy żądaniu PI równym zero,
zanim wolniejsza rampa wyjściowa faktycznie zejdzie do zera.

Domyślna wartość `20 rpm` jest ustawiona w trzech miejscach, które uczestniczą
w inicjalizacji: banku, fasadzie regulatora i starszym polu EEPROM
`walk_assist_speed`. Dzięki temu migracja lub fabryczny reset nie przywraca 50 rpm.

## 6. Maszyna stanów i bezpieczeństwo

| Stan | Znaczenie |
|---|---|
| `OFF` | WA nie steruje silnikiem |
| `REGULATE` | START, RUN albo REACQUIRE |
| `LIMIT` | ograniczenie do `15 Iq` po wykryciu problemu ruchu |
| `STALL` | `Iq=0`, blokada do pełnego puszczenia WA |

Nadrzędne warunki natychmiastowego zakończenia WA:

- puszczenie przycisku bez aktywnego bankowego latch;
- hamulec;
- fault sterownika;
- osiągnięcie bankowego `Walk assist cut-off` mierzonego na kole.

Watchdog nadal wykrywa brak ruchu lub częściowe zakleszczenie. Próg istotnego
prądu wynosi `24 Iq`, więc pozostaje osiągalny przy START ograniczonym do
40 Iq. Hamulec, fault, limit koła i zatrzask prawdziwego zakleszczenia pozostają aktywne.

## 7. Test regresji

`tests/fw060_walk_speed_controller.js` sprawdza:

- jednorazowy START do 40 Iq bez skoku i twardy sufit 40 Iq;
- zakończenie START przy pierwszym wiarygodnym ruchu `8 ERPS`;
- miękkie przejęcie START -> RUN;
- RUN `2..40 Iq`, podwojoną dodatnią reakcję PI i obie rampy;
- około `30 Iq` po 1 s i pełne `40 Iq` po około `1,51 s` przy maksymalnym błędzie;
- zejście wyłącznie do `2 Iq` przy dużym przekroczeniu oraz nadal prawdziwe
  zero po wyłączeniu podłogi przez bezpieczeństwo;
- brak ponownego START w REACQUIRE;
- około 6 Iq po 200 ms odzyskiwania, sufit 24 Iq przed timeoutem;
- osobne odzyskanie po zaniku przy podłodze 2 Iq do 24 Iq, z dużym
  zapasem przed timeoutem i bez ponownego START;
- pięć kolejnych cykli utraty/powrotu Halla bez ponownego START i bez zejścia
  normalnego RUN poniżej 2 Iq;
- odporność na szum wewnątrz deadbandu;
- stałe i nagle rosnące obciążenie;
- oddzielne punkty równowagi lekkiego napędu dla 30 i 50 rpm.

Wynik modelu lekkiego napędu:

| Nastawa | Cel | Stan ustalony modelu | Zębatka |
|---:|---:|---:|---:|
| 30 rpm | 40 ERPS | 38,2 ERPS | około 28,7 rpm |
| 50 rpm | 67 ERPS | 66,9 ERPS | około 50,2 rpm |

Wszystkie testy hostowe przeszły 7/7:

```text
fw016_ride_core_model.ps1       PASS
fw056_power_curve.js            PASS
fw057_cadence_comp.js           PASS
fw058_coast_rezero.js           PASS
fw060_walk_speed_controller.js  PASS
fw077_start_condition_kg.js     PASS
fw078_hall_autocalibration.js   PASS
```

## 8. Artefakt

```text
.build/0.0282_M820_BL820.bin
89 644 B
SHA-256 E36E0BCF1937205BAAE36007208C0ED09E775BCB317F8EDE99941F65AF1B87E9
Arm GNU Toolchain 13.2.1
CAN diagnostics OFF
```

Build do wgrania wykonano używanym wcześniej skryptem i zapisano bezpośrednio
w dotychczasowym katalogu `.build`:

```powershell
.\build_firmware.ps1
```

Numer `0.0281` został zużyty przez przerwany przebieg skryptu i nie jest
kompletnym kandydatem. Do wgrania służy wyłącznie obraz M820/BL820 `0.0282`.

## 9. Procedura testu sprzętowego

1. Wgrać `0.0282_M820_BL820.bin`, nie plik surowy `0.0282.bin`.
2. Unieść koło, wybrać najniższy bieg i zapewnić natychmiastowy dostęp do hamulca.
3. Ustawić 20 rpm i przytrzymać WA minimum 15 s. Po pierwszym przestrzale silnik
   nie powinien zatrzymać wirnika; jeżeli Hall mimo 2 Iq zaniknie, napęd musi sam
   wrócić bez puszczenia przycisku i bez drugiego impulsu START.
4. Przy nadal trzymanym WA delikatnie zwiększyć opór napędu. Iq powinno reagować
   płynnie: w teście modelowym około 30 Iq po 1 s i maksymalnie 40 Iq po około 1,51 s.
5. Po pełnym puszczeniu sprawdzić osobno 30 rpm i 50 rpm.
6. 50 rpm musi być wyraźnie szybsze od 30 rpm; żadna próba nie może stale
   przyspieszać aż do limitu koła.
7. Przerwać test przy cyklicznym mocnym zatrzymaniu/rozruchu, przekroczeniu
   startowego limitu 40 Iq, gwałtownym
   wzroście prędkości lub braku reakcji hamulca.
8. Dopiero po zaliczeniu stojaka wykonać ostrożny test na ziemi.

## 10. Historia decyzji

| Firmware | Wynik / decyzja |
|---|---|
| `0.0258` | `5 Iq` ponad celem stale rozpędzało lekkie koło |
| `0.0259` | twarde przełączanie `5/0 Iq` ograniczyło obroty, ale szarpało |
| `0.0260` | agresywne doganianie powodowało coast/reacquire i prawdopodobny STALL |
| `0.0263` | jednorazowy START, RUN 5–36 Iq i stały governor 85/70 rpm |
| `0.0264` | governor zależny od celu: `target+20/+5 rpm` |
| `0.0268` | sprzęt potwierdził cykl zatrzymania i ponownego rozruchu governora |
| `0.0269` / FW-074 | usunięto governor, ale stałe 5 Iq świadomie porzuciło kontrolę prędkości lekkiego napędu |
| `0.0275` | sprzęt potwierdził, że 30/50 rpm nie kontroluje zębatki i stale ją rozpędza |
| `0.0276` / FW-079 | RUN 0–36 Iq, deadband ±2 ERPS, oddzielne testy 30/50 rpm; oczekuje testu sprzętowego |
| `0.0277` | przy 20 rpm po około sekundzie następował fałszywy STALL do puszczenia WA |
| `0.0279` / FW-080 | poprawione rozpoznanie zamierzonego wybiegu, timeouty osiągalne przez rampę i spójny default 20 rpm; oczekuje testu sprzętowego |
| `0.0280` / FW-081 | START 30 Iq, RUN 1–36 Iq, odzyskanie ≤24 Iq i watchdog dopasowany do nowego sufitu; oczekuje testu sprzętowego |
| `0.0282` / FW-082 | START 40 Iq do 8 ERPS, RUN 2–40 Iq, dwukrotnie szybszy wzrost i dodatnia całka PI; oczekuje testu sprzętowego |
