# Walk Assist — aktualne działanie

Aktualizacja: 2026-08-04, FW-082, firmware `0.0282`.

Status: wszystkie 7 testów hostowych i oficjalny build M820/BL820 przeszły.
Szybsza reakcja na obciążenie i podtrzymanie Halla przez minimum 2 Iq oczekują
na test z uniesionym kołem.

## Co steruje prędkością

Walk Assist reguluje obroty zębatki/łańcucha na podstawie Halla silnika. Nie
reguluje bezpośrednio prędkości koła. Bieg mechaniczny nadal decyduje, jak szybko
porusza się rower.

Bankowy parametr `Target chainring RPM` jest rzeczywistym celem regulatora:

```text
motor ERPS = chainring rpm × 4 / 3
```

Przykłady:

| Ustawienie | Cel silnika |
|---:|---:|
| 30 rpm | 40 ERPS |
| 50 rpm | 67 ERPS |

Wpis spoza zakresu 20–60 rpm jest zastępowany wartością domyślną 20 rpm.

## Przebieg po naciśnięciu przycisku

1. Jednorazowy START ma cel i twardy sufit `40 Iq`. Prąd narasta płynnie
   `93,75 Iq/s`, więc pełną wartość osiąga po około 0,42 s, jeżeli silnik wcześniej nie ruszy.
2. Pierwszy wiarygodny filtrowany ruch `8 ERPS` kończy START i przełącza do RUN
   `2..40 Iq`. Jest to około `6 rpm` zębatki; przy przełożeniu 36/48 i kole
   29 cali odpowiada około `0,63 km/h`.
3. W RUN prąd narasta maksymalnie `31,25 Iq/s` i maleje `31,25 Iq/s`.
   Dodatnia całka PI również działa dwukrotnie szybciej niż w FW-081.
4. Poniżej celu PI zwiększa prąd tylko tak mocno, jak wymaga obciążenie.
5. Powyżej celu PI odwija całkę i płynnie schodzi do podłogi `2 Iq`, która ma
   podtrzymać wirnik oraz impulsy Halla bez dawnego rozpędzania przez 5 Iq.
6. Nie ma twardego progu `target+20 rpm`, przełączania `5/0 Iq` ani osobnego
   automatu COAST.
7. Jeżeli Hall zaniknie mimo 2 Iq, ograniczone odzyskanie może podjąć wirnik
   maksymalnie do 24 Iq. Nie uruchamia ponownie START.
8. Pełne puszczenie przycisku kończy sesję i uzbraja START na następne użycie.

```text
OFF -> jednorazowy START <=40 Iq, do 8 ERPS -> RUN: PI 2..40 Iq
                              |
                              +-- za wolno: płynnie więcej Iq
                              +-- za szybko: płynnie do 2 Iq
                              +-- Hall zniknął: odzyskanie do 24 Iq, bez START
```

Deadband regulatora wynosi `±2 ERPS`, czyli około `±1,5 rpm` zębatki. Na
rzeczywistym napędzie chwilowy błąd może być większy podczas START, zmiany
obciążenia lub zmiany biegu; nie powinien jednak stale narastać.

## Dlaczego zmieniono FW-074

FW-074 utrzymywał minimum `5 Iq` nawet daleko ponad celem. Na lekkim napędzie
30 i 50 rpm zbiegały przez to do niemal tej samej prędkości wynikającej z oporów.
Test również błędnie uznawał ten stan za poprawny.

FW-079 usunął problematyczną podłogę 5 Iq. Aktualny model lekkiego napędu uzyskuje oddzielne
punkty równowagi:

| Nastawa | Wynik modelu | W przeliczeniu na zębatkę |
|---:|---:|---:|
| 30 rpm | 38,2 ERPS | około 28,7 rpm |
| 50 rpm | 66,9 ERPS | około 50,2 rpm |

Model uwzględnia deadband i zaokrąglenie pomiaru. Ostateczną dokładność trzeba
potwierdzić na silniku.

FW-082 utrzymuje minimalne `2 Iq`, czyli mniej niż połowę problematycznych 5 Iq.
Celem nie jest napędzanie zębatki, lecz niedopuszczenie do zatrzymania wirnika
i utraty pomiaru Halla. Test nadal wymaga oddzielnych wyników dla 30 i 50 rpm.

Przy maksymalnym dodatnim błędzie model całego regulatora osiąga około `30 Iq`
po 1 s i pełne `40 Iq` po około `1,51 s`. To czas odpowiedzi PI razem z rampą,
a nie gwarantowany czas mechanicznego rozpędzenia roweru — ten zależy od obciążenia.

## Ustawienia banku

Aktywne ustawienia WA:

- `Target chainring RPM`: 20–60 rpm;
- `Walk assist cut-off`: niezależny limit prędkości koła;
- podtrzymanie po puszczeniu przycisku i timeout podtrzymania.

Pole `Walk current` pozostaje w banku dla zgodności ze starszym Canable, ale
aktualny regulator WA go nie używa. Zakres RUN `2..40 Iq` jest stałą firmware.

## Bezpieczeństwo

Hamulec, fault sterownika i bankowy limit prędkości koła są nadrzędne.

| Stan | Znaczenie |
|---|---|
| `OFF` | Walk Assist nie steruje silnikiem |
| `REGULATE` | jednorazowy START, RUN albo łagodny REACQUIRE |
| `LIMIT` | wykryty problem ruchu; prąd ograniczony do `15 Iq` |
| `STALL` | brak poprawy przez 400 ms; `Iq=0`, blokada do puszczenia WA |

Nieoczekiwany zanik Halla przy małym prądzie ma sufit `24 Iq` i timeout `1,5 s`.
Zanik Halla mimo podłogi 2 Iq może być odzyskiwany do `24 Iq` przez maksymalnie
4 s. Obie ścieżki narastają
`31,25 Iq/s` (około `6 Iq` po 200 ms), czyli trzy razy wolniej niż pierwszy
START. Brak skutecznego odzyskania nadal prowadzi do `LIMIT/STALL`.

## Firmware do testu

```text
.build/0.0282_M820_BL820.bin
89 644 B
SHA-256 E36E0BCF1937205BAAE36007208C0ED09E775BCB317F8EDE99941F65AF1B87E9
CAN diagnostics OFF
```

Plik powstał tym samym skryptem i w tym samym katalogu, co wcześniejsze
firmware wgrywane do sterownika:

```powershell
.\build_firmware.ps1
```

Numer `0.0281` został zużyty przez przerwany przebieg kompilacji i nie jest
gotowym obrazem. Format konfiguracji i protokół CAN nie zmieniły się, dlatego
do tej poprawki nie trzeba budować ani wgrywać nowego Canable.

## Pierwszy test sprzętowy

1. Wgrać wyłącznie powyższy plik `0.0282_M820_BL820.bin`.
2. Unieść koło, wybrać najniższy bieg i zapewnić natychmiastowy dostęp do hamulca.
3. Ustawić 20 rpm i przytrzymać WA co najmniej 15 s. Minimum 2 Iq ma utrzymać
   Hall; jeżeli mimo to zaniknie, napęd ma wrócić bez puszczenia przycisku.
4. Przy nadal trzymanym WA delikatnie zwiększyć opór. Iq ma zwiększać się płynnie,
   bez skoku i nie przekraczać 40 Iq.
5. Puścić przycisk, sprawdzić osobno 30 rpm i 50 rpm.
6. Przy 50 rpm zębatka musi obracać się wyraźnie szybciej niż przy 30 rpm. Obie
   próby mają dojść do stanu ustalonego, a nie stale się rozpędzać.
7. Niedopuszczalne są powtarzające się mocne cykle zatrzymania/rozruchu,
   przekroczenie startowego limitu 40 Iq,
   dalsze rozpędzanie mimo przekroczenia celu lub brak reakcji hamulca.

Historia wcześniejszych prób i szczegóły regulatora są w
`documentation/FW-060_WA_CONSTANT_RPM_CONTROLLER.md`.
