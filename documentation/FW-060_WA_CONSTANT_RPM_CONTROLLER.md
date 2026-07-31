# FW-060 / FW-067 — regulator Walk Assist

- **Aktualizacja:** 2026-07-31
- **Aktualny kandydat:** FW-067, firmware `0.0264`
- **Status:** testy hostowe 5/5 i oba buildy zakończone powodzeniem; oczekuje
  kontrolowanego testu na stojaku
- **Zakres:** M820; bez zmiany formatu banku, zwykłego wspomagania i algorytmu
  przełączania Hall/FOC

## 1. Artefakty testowe

| Wariant | Plik | Rozmiar | SHA-256 | Diagnostyka CAN |
|---|---|---:|---|---|
| normalny | `.build/M820_BL820/debug/normal/0.0264_M820_BL820.bin` | 88 844 B | `438CC4E68712586112C575DFC98352A3D1DF5FEB7C25550AE5BC198269B85CC7` | OFF |
| diagnostyczny | `.build/M820_BL820/debug/diagnostic/0.0264-diag_M820_BL820.bin` | 93 384 B | `584A0BE4667C3AAEFE8901AF92F6352D62CE7F3A1E782D339DB416F60E6366DB` | ON |

Oba obrazy nie mają segmentu RWE. Do pierwszego testu należy wgrać wyłącznie
wariant normalny. Wariant diagnostyczny służy do zebrania logu, jeżeli zachowanie
normalnego obrazu będzie niejednoznaczne.

`0.0262` i `0.0263` nie zostały odrzucone testem sprzętowym. Oba zastąpiono
przed wgraniem po kolejnych doprecyzowaniach oczekiwanego działania regulatora.

## 2. Wymaganie użytkownika

Walk Assist ma działać w czterech funkcjonalnych etapach:

```text
OFF -> jednorazowy START -> łagodny RUN -> twardy COAST
```

1. Pierwsze naciśnięcie ma ruszyć energicznie, ponieważ WA jest często używany
   do ruszenia pod górę.
2. Po ruszeniu prąd ma być ograniczony do niewielkiego zakresu `Iq_min..Iq_max`.
   Regulator PI ma pracować wolno i płynnie.
3. Bankowy `Target chainring RPM` jest miękkim celem, a nie punktem twardego
   odcięcia prądu.
4. Przy zwykłym przestrzale prąd nie może przełączać się cyklicznie `5/0 Iq`.
5. Próg prawdziwego `Iq=0` ma nadążać za ustawionym celem i wynosić
   `target + 20 rpm`.
6. RUN ma wracać przy `target + 5 rpm`, bez ponownego impulsu START.
7. Pełne puszczenie przycisku kończy sesję. Hamulec, fault, limit prędkości koła
   i istniejący watchdog zablokowania pozostają nadrzędne.

FW-067 realizuje ten opis dla całego poprawnego zakresu celu 20–60 rpm.

## 3. Parametry FW-067

### 3.1 START

| Parametr | Wartość |
|---|---:|
| początkowy cel | `18 Iq` przez pierwsze 80 ms |
| maksymalny breakaway | `80 Iq` |
| rampa wyjścia | `93,75 Iq/s` |
| czas dojścia od 0 do 80 Iq | około `0,85 s` |
| koniec START | `30%` bankowego celu |
| maksymalne zasianie całki przy przejęciu | `8 Iq` |

START jest jednorazowy w obrębie jednego przytrzymania przycisku. Nie uzbraja
się ponownie po przestrzale, wybiegu, chwilowym braku Halla ani ograniczeniu
prądu. Ponowny START jest możliwy dopiero po pełnym zakończeniu zadania WA.

### 3.2 RUN

| Parametr | Wartość |
|---|---:|
| miękkie minimum | `5 Iq` |
| miękkie maksimum | `36 Iq` |
| rampa narastania | `15,625 Iq/s` |
| rampa opadania | `31,25 Iq/s` |
| deadband PI | `+/-7 ERPS`, około `+/-5,25 rpm` zębatki |
| ograniczenie błędu PI | `+/-12 ERPS` |
| Kp | `1 Iq/ERPS` |
| Ki | dodatni `error * 1`, ujemny `error * 8` w Q8 na krok PI |
| częstotliwość PI | `200 Hz` |

Sufit `36 Iq` jest miękki. Po zakończeniu START prąd odziedziczony powyżej
`36 Iq` schodzi przez rampę opadania, zamiast zostać ucięty jednym krokiem.

Sama rampa potrzebuje co najmniej około 1,98 s na przejście `5 -> 36 Iq`.
W modelu regulatora przy maksymalnym dodatnim błędzie pełne przejście trwa około
2,59 s, ponieważ powoli narasta również żądanie PI.

Stara krzywa anti-stall `0..48 Iq` została całkowicie usunięta. To RUN, istniejący
watchdog i ograniczony `REACQUIRE` odpowiadają teraz za zachowanie przy spadku
obrotów.

### 3.3 Twardy regulator maksymalnych obrotów

| Parametr | Wartość |
|---|---:|
| wejście w COAST | `target + 20 rpm` zębatki |
| powrót do RUN | `target + 5 rpm` zębatki |
| zakres progu COAST | `40..80 rpm` dla celu `20..60 rpm` |
| zakres progu wznowienia | `25..65 rpm` dla celu `20..60 rpm` |
| Iq podczas COAST | `0 Iq` |
| brak Halla podczas COAST | oczekiwanie 1 s, potem łagodny `REACQUIRE` |
| maksimum `REACQUIRE` | `24 Iq` |

Stała histereza 15 rpm zapobiega szybkiemu przełączaniu na jednej granicy.
Wejście w COAST zeruje stare żądanie PI, ale nie resetuje zatrzasku START.

Na bardzo lekko obciążonym kole dodatnie minimum `5 Iq` może rozpędzić napęd
ponad miękki cel. Jest to świadomy skutek wymagania, aby zwykłe przekroczenie
celu nie zerowało prądu. Twardą granicą jest zawsze bankowy cel powiększony
o 20 rpm.

Przykłady:

| Target chainring RPM | COAST, Iq=0 | Powrót RUN |
|---:|---:|---:|
| 20 | 40 rpm | 25 rpm |
| 40 | 60 rpm | 45 rpm |
| 50 | 70 rpm | 55 rpm |
| 60 | 80 rpm | 65 rpm |

## 4. Przepływ sterowania

```text
przycisk WA + brak hamulca/faultu/limitu koła
        |
        v
jednorazowy START do 80 Iq
        |
        v
RUN: PI ograniczone do 5..36 Iq
        |
        +-- rpm < target+20 -------------------+
        |                                      |
        +-- rpm >= target+20 -> COAST, Iq=0    |
                              |                 |
                              +-- rpm <= target+5
                              |
                              +-- brak Halla 1 s -> REACQUIRE <=24 Iq
```

Niezależnie od tej ścieżki:

- puszczenie WA bez bankowego latch zeruje zadanie;
- hamulec, fault lub bankowy limit prędkości koła zatrzymuje WA;
- rzeczywisty brak ruchu pod momentem przechodzi do `LIMIT`, a następnie
  zatrzaśniętego `STALL`;
- `STALL` wymaga puszczenia przycisku przed następną próbą.

## 5. Architektura kodu

| Plik | Odpowiedzialność |
|---|---|
| `src/walk_speed_controller.c` | START, PI, anti-windup, miękki sufit RUN i rampy Iq |
| `src/walk_assist_motor.c` | estymacja Halla, przeliczenie rpm/ERPS, regulator `target+20/+5 rpm`, `REACQUIRE`, `LIMIT/STALL` |
| `src/assist_dynamics.c` | jeden właściciel rampy podczas WA i synchronizacja wyjścia |
| `src/main.c` | aktywacja, pomiary, odroczenie zmiany banku i opcjonalna diagnostyka CAN |

Regulator otrzymuje finalny `MS.i_q_setpoint` z poprzedniego ticku. Jeśli limit
napięcia, temperatury albo inne zabezpieczenie obniży prąd, wewnętrzny stan
wyjścia jest sprowadzany do wartości rzeczywistej. Ogranicza to nawijanie całki
i skok po ustąpieniu ograniczenia.

## 6. Konfiguracja i zgodność

Aktywne ustawienia bankowe pozostają bez zmiany:

- `Target chainring RPM`, zakres 20–60 rpm;
- `Walk assist cut-off`, czyli niezależny limit prędkości koła;
- podtrzymanie po puszczeniu przycisku i jego timeout.

Przeliczenie dla M820:

```text
motor ERPS = chainring rpm * 4 / 3
```

Pole `Walk current` pozostaje w bank blob v5 wyłącznie dla zgodności ze starszym
Canable i firmware. FW-067 nie używa go do sterowania. `Iq_min=5` i
`Iq_max=36` są na tym etapie stałymi testowymi firmware. Progi COAST są
wyliczane z aktywnego `Target chainring RPM`.

Problem Canable, w którym po zapisie ponowny `Read` pokazuje stare wartości,
pozostaje oddzielnym zadaniem. FW-067 nie zmienia Canable ani formatu banku.

Wartość celu spoza zakresu 20–60 rpm jest odrzucana i zastępowana domyślnym
celem 50 rpm, co daje progi 70/55 rpm. Przy maksymalnym poprawnym celu 60 rpm
progi wynoszą 80/65 rpm, więc obliczenia pozostają w bezpiecznym zakresie.

## 7. Zabezpieczenia

Zewnętrznie raportowane stany pozostają zgodne:

| Stan | Znaczenie |
|---|---|
| `OFF` | WA nie steruje silnikiem |
| `REGULATE` | START, RUN, COAST albo kontrolowany `REACQUIRE` |
| `LIMIT` | ograniczenie do `15 Iq` przy potwierdzonym problemie ruchu |
| `STALL` | po 400 ms bez poprawy w `LIMIT`: `Iq=0`, blokada do puszczenia |

Watchdog zachowuje 1,5 s okresu startowego. Następnie obserwuje brak ruchu,
zbyt mały ruch przy istotnym prądzie i utratę Halla podczas podawania momentu.
FW-067 nie osłabia tych zabezpieczeń.

Szczególny przypadek COAST:

- zanik Halla jest oczekiwany, ponieważ `Iq=0`;
- firmware przez 1 s nie uznaje go za zablokowanie;
- po tym czasie uruchamia istniejące łagodne odzyskanie do `24 Iq`;
- brak skutecznego odzyskania nadal prowadzi do `LIMIT/STALL`.

## 8. Diagnostyka CAN

Ramki są dostępne tylko w buildzie diagnostycznym.

`0x00010205`:

| Bajty | Wartość |
|---|---|
| 0 | stan |
| 1 | flagi |
| 2–3 | target ERPS |
| 4–5 | measured ERPS |
| 6–7 | zadany Iq |

Flagi: `0x01` Hall valid, `0x02` jam, `0x04` blocked, `0x08` saturation/slew,
`0x10` start active, `0x20` above target, `0x40` LIMIT,
`0x80` gentle Hall reacquire.

`0x00010206`:

| Bajty | Wartość |
|---|---|
| 0–1 | błąd ERPS, signed |
| 2–3 | człon całkujący w Iq |
| 4–5 | chwilowa podłoga START w Iq |
| 6–7 | wiek ostatniego Halla w ms |

Do analizy ograniczenia napięciowego należy równolegle rejestrować
`0x00010204`: actual Iq, `u_abs`, `u_q`.

## 9. Testy automatyczne

`node tests/fw060_walk_speed_controller.js` sprawdza między innymi:

- jednorazowy START 93,75 Iq/s do 80 Iq;
- przejęcie bez skoku i bez ponownego uzbrojenia START;
- miękkie granice RUN `5..36 Iq`;
- rampy RUN 15,625/31,25 Iq/s;
- brak dawnej krzywej anti-stall;
- brak zerowania Iq przy zwykłym przekroczeniu miękkiego celu;
- dynamiczny COAST przy `target+20 rpm` i powrót przy `target+5 rpm`;
- poprawne progi dla celu 20, 40, 50 i 60 rpm;
- brak natychmiastowego błędu Halla podczas celowego wybiegu;
- łagodny `REACQUIRE <=24 Iq` po sekundzie bez Halla;
- brak pompowania w deadbandzie;
- zachowanie przy stałym i nagle rosnącym obciążeniu;
- model lekkiego napędu dochodzący do twardego regulatora;
- natychmiastowe oddanie sterowania po zakończeniu WA.

Pełny zestaw zakończył się wynikiem 5/5 PASS:

```text
tests/fw016_ride_core_model.ps1       PASS
tests/fw056_power_curve.js            PASS
tests/fw057_cadence_comp.js           PASS
tests/fw058_coast_rezero.js           PASS
tests/fw060_walk_speed_controller.js  PASS
```

Opcjonalne porównanie z sąsiednim modułem Canable w `fw056_power_curve.js`
zostało pominięte, ponieważ moduł nie był dostępny. Rdzeń testu przeszedł.

## 10. Procedura pierwszego testu 0.0264

1. Wgrać normalny `0.0264_M820_BL820.bin`.
2. Testować wyłącznie z kołem w powietrzu, na najniższym biegu i z możliwością
   natychmiastowego użycia hamulca.
3. Nie zakładać, że wpisana w Canable wartość została zapisana. Jeżeli `Read`
   pokazuje 50 rpm, traktować 50 rpm jako rzeczywisty cel testu.
4. Nacisnąć WA jeden raz i trzymać. Start powinien być zdecydowany, ale płynny;
   dojście do 80 Iq trwa około 0,85 s.
5. Po ruszeniu nie powinno być szybkiego doganiania, okresowego zaniku momentu
   ani szarpania. RUN ma pozostać pomiędzy 5 i 36 Iq.
6. Na lekkim kole dopuszczalne jest przekroczenie miękkiego celu. Dla wartości
   odczytanej jako 50 rpm COAST ma rozpocząć się około 70 rpm, a RUN wrócić
   około 55 rpm. Dla innego celu użyć odpowiednio `target+20` i `target+5`.
7. Sprawdzić kilka cykli COAST/RUN przy ciągłym trzymaniu przycisku. Nie może
   wrócić impuls START 80 Iq ani zatrzasnąć się `STALL`.
8. Umiarkowanie zwiększyć obciążenie ręką. Prąd ma narastać wolno; nie blokować
   gwałtownie napędu tylko w celu wymuszenia zabezpieczenia.
9. Puścić WA. Bez aktywnego bankowego latch zadany prąd ma natychmiast spaść do
   zera.
10. Przerwać test przy gwałtownym przyroście prędkości, mocnym szarpaniu,
    niekontrolowanym wzroście Iq lub braku reakcji hamulca.

Test na ziemi pozostaje zablokowany do zaliczenia tej procedury. Jeżeli wynik
będzie nieprawidłowy, nie zmieniać jednocześnie kilku parametrów. Najpierw
zebrać log z buildem diagnostycznym i podać: zachowanie START, przybliżone rpm,
`iq_target`, stan oraz moment wystąpienia problemu.

## 11. Historia decyzji

| Firmware | Wynik / decyzja |
|---|---|
| `0.0258` | stałe dodatnie `5 Iq` ponad celem rozpędzało uniesione koło aż do około 15 km/h |
| `0.0259` | usunęło trwałe rozpędzanie, ale przełączanie `5/0 Iq` powodowało szarpanie |
| `0.0260` | usunęło skok podłogi, lecz agresywne doganianie powodowało cykle coast/reacquire i prawdopodobny `STALL` |
| `0.0261` | złagodziło regulator; zastąpione przed testem po decyzji o dłuższych rampach |
| `0.0262` | wydłużyło rampy; zastąpione przed testem po doprecyzowaniu wymagania `Iq_min..Iq_max` oraz odcięcia dopiero przy 80–90 rpm |
| `0.0263` | FW-066: START 80 Iq, RUN 5–36 Iq, rampy 15,625/31,25 Iq/s, stały COAST 85/70 rpm; zastąpione przed testem |
| `0.0264` | FW-067: zachowuje START/RUN, ale wylicza COAST jako `target+20/+5 rpm` |

## 12. Kolejność strojenia po teście

1. Start zbyt słaby lub zbyt mocny: `WA_SPEED_BREAKAWAY_IQ`, następnie rampa
   `WA_SPEED_START_RISE_STEP_Q`.
2. RUN zbyt mocno dogania: najpierw `WA_MOTOR_RUN_MAX_IQ`, potem rampa
   `WA_SPEED_RUN_RISE_STEP_Q`.
3. Pompowanie wokół miękkiego celu: deadband i Ki; Kp zmieniać na końcu.
4. Za częste cykle COAST: ocenić dynamiczne progi `target+20/+5 rpm`
   i mechaniczne obciążenie; nie usuwać histerezy.
5. Brak momentu przy wysokim `u_abs`: zbadać ograniczenie napięciowe/field
   weakening zamiast podnosić PI.

Jedna zmiana parametru na jeden build i ten sam zestaw logów.
