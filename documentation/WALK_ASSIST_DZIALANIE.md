# Walk Assist — aktualne działanie

Aktualizacja: 2026-07-31, FW-067, build testowy `0.0264`.

Status: testy hostowe 5/5 i build normalny/diagnostyczny przeszły. Firmware
oczekuje testu na stojaku. Nie testować WA na ziemi przed zaliczeniem testu
z uniesionym kołem.

## Najważniejsze zachowanie

Walk Assist steruje obrotami zębatki/łańcucha, a nie bezpośrednio prędkością
koła. Bieg mechaniczny decyduje, jak szybko porusza się rower.

Po naciśnięciu przycisku:

1. Jednorazowy START rusza energicznie do maksymalnie `80 Iq`. Prąd narasta
   płynnie z szybkością `93,75 Iq/s`, więc pełną wartość osiąga po około 0,85 s.
2. Po rozpędzeniu sterowanie przechodzi do spokojnego RUN w zakresie
   `5..36 Iq`.
3. W RUN prąd narasta maksymalnie `15,625 Iq/s`, a maleje `31,25 Iq/s`.
4. Bankowy `Target chainring RPM` jest miękkim celem. Regulator nie zeruje
   momentu natychmiast po jego przekroczeniu.
5. Twardy wybieg `Iq=0` włącza się przy `Target chainring RPM + 20 rpm`.
6. RUN może wrócić przy `Target chainring RPM + 5 rpm`. Stała histereza 15 rpm
   zapobiega szybkiemu przełączaniu na jednej granicy.
7. Powrót z wybiegu nie uruchamia ponownie START 80 Iq.
8. Pełne puszczenie przycisku kończy sesję i pozwala uzbroić następny START.

Schemat:

```text
OFF -> START do 80 Iq -> RUN 5..36 Iq -> COAST przy target+20
                           ^                    |
                           +-- przy target+5 ---+
```

Na bardzo lekko obciążonym, uniesionym kole stałe minimum `5 Iq` może rozpędzić
napęd powyżej miękkiego celu. Jest to zamierzony skutek wymagania, aby prąd nie
zanikał w normalnym zakresie. Obroty ogranicza regulator zależny od celu.

| Ustawiony cel | Iq=0 / COAST | Powrót RUN |
|---:|---:|---:|
| 20 rpm | 40 rpm | 25 rpm |
| 40 rpm | 60 rpm | 45 rpm |
| 50 rpm | 70 rpm | 55 rpm |
| 60 rpm | 80 rpm | 65 rpm |

Cel bliski maksymalnym 60 rpm nie psuje algorytmu: najwyższe aktywne progi
wynoszą wtedy 80/65 rpm. Wpis spoza poprawnego zakresu 20–60 rpm jest zastępowany
celem domyślnym 50 rpm.

## Ustawienia banku

Aktywne ustawienia:

- `Target chainring RPM`, zakres 20–60 rpm;
- `Walk assist cut-off`, niezależny limit prędkości koła;
- podtrzymanie po puszczeniu przycisku i timeout podtrzymania.

Przeliczenie dla M820:

```text
motor ERPS = chainring rpm * 4 / 3
```

Pole `Walk current` pozostaje w banku dla zgodności ze starszym Canable, ale
FW-067 nie używa go do regulacji. Granice RUN `5/36 Iq` są obecnie stałymi
testowymi w firmware, natomiast progi COAST wynikają z aktywnego celu banku.

Canable ma oddzielny, nierozwiązany problem: po zapisie ponowny `Read` może
pokazać starą wartość. Firmware `0.0264` nie zmienia Canable. Do testu należy
przyjąć wartość rzeczywiście zwróconą przez `Read`.

## Bezpieczeństwo

Hamulec, fault sterownika oraz bankowy limit prędkości koła są nadrzędne wobec
regulatora WA.

Stany raportowane przez sterownik:

| Stan | Znaczenie |
|---|---|
| `OFF` | Walk Assist nie steruje silnikiem |
| `REGULATE` | START, RUN, COAST albo kontrolowane odzyskanie Halla |
| `LIMIT` | wykryty problem ruchu; prąd ograniczony do `15 Iq` |
| `STALL` | brak poprawy przez 400 ms; `Iq=0`, blokada do puszczenia WA |

Jeżeli Hall zniknie podczas celowego COAST:

1. firmware utrzymuje `Iq=0` i czeka 1 s;
2. następnie może rozpocząć łagodne odzyskanie do maksymalnie `24 Iq`;
3. powrót Halla kontynuuje RUN bez ponownego START;
4. brak skutecznego odzyskania nadal prowadzi do `LIMIT/STALL`.

Istniejący watchdog zablokowania pozostaje aktywny. Przy nagłym dużym oporze
sterownik nie powinien gwałtownie podnosić momentu ponad zakres RUN. Bezpieczne
ograniczenie lub zatrzymanie ma pierwszeństwo.

## Firmware do testu

Normalny:

```text
.build/M820_BL820/debug/normal/0.0264_M820_BL820.bin
88 844 B
SHA-256 438CC4E68712586112C575DFC98352A3D1DF5FEB7C25550AE5BC198269B85CC7
CAN diagnostics OFF
```

Diagnostyczny, tylko do logowania:

```text
.build/M820_BL820/debug/diagnostic/0.0264-diag_M820_BL820.bin
93 384 B
SHA-256 584A0BE4667C3AAEFE8901AF92F6352D62CE7F3A1E782D339DB416F60E6366DB
CAN diagnostics ON
```

Do pierwszego testu wgrać wariant normalny, ustawić najniższy bieg i pozostawić
koło w powietrzu. Start ma być zdecydowany, ale bez uderzenia. Po ruszeniu nie
może wystąpić szybkie doganianie, okresowy zanik momentu ani szarpanie.

Na lekkim kole można pozwolić na wejście dynamicznego regulatora, stale trzymając
przycisk. Dla celu 50 rpm oczekiwany jest COAST około 70 rpm i powrót około
55 rpm, bez impulsu 80 Iq. Test przerwać przy gwałtownym wzroście prędkości,
szarpaniu, niekontrolowanym wzroście prądu lub braku reakcji hamulca.

Pełna procedura, historia diagnozy i kolejność strojenia:
`documentation/FW-060_WA_CONSTANT_RPM_CONTROLLER.md`.

## Historia ostatnich prób

| Firmware | Wynik |
|---|---|
| `0.0258` | stale rozpędzał uniesione koło, w jednym teście do około 15 km/h |
| `0.0259` | ograniczył rozpędzanie, ale szarpał przez przełączanie `5/0 Iq` |
| `0.0260` | zbyt mocno doganiał, cyklicznie przestrzeliwał i prawdopodobnie wchodził w `STALL` |
| `0.0261` | zastąpiony przed testem po decyzji o wolniejszych rampach |
| `0.0262` | zastąpiony przed testem po doprecyzowaniu koncepcji `Iq_min..Iq_max` i odcięcia przy 80–90 rpm |
| `0.0263` | START 80 Iq, RUN 5–36 Iq i stały COAST 85/70 rpm; zastąpiony przed testem |
| `0.0264` | aktualny kandydat: ten sam START/RUN, dynamiczny COAST `target+20/+5 rpm` |
