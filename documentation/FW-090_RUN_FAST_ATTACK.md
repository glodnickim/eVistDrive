# FW-090 — Estymator RUN: szybki atak, wolne opadanie

- **Data:** 2026-08-06
- **Status:** WDROŻONE, ale **DOMYŚLNIE WYŁĄCZONE** (`TORQUE_RUN_ATTACK_STEPS = 0`).
  Zdegradowane przez `FW-091` do mechanizmu eksperymentalnego.
  **Ta karta NIE jest naprawą problemu ponownego uruchomienia** — patrz §0.
- **Cel:** ponowne złapanie wspomagania w jeździe ma być powtarzalne, bez konieczności
  przejechania pół obrotu korby z naciskiem.
- **Zakres:** `inc/torque_input.h`, `src/torque_input.c`, test hostowy. Bez zmian w
  transporcie, blobie i UI.
- **Powiązane:** `FW-085_RUN_FILTER_CRANK_ANGLE.md` (stąd przyczyna), `FW-033`.

## 0. KOREKTA — ta karta myliła przyczynę z objawem

Napisana jako naprawa objawu „w jeździe po zaniku mocy ponowne złapanie bywa leniwe".
Późniejszy audyt wykazał, że **główną przyczyną był limiter**: `assist_limits.c` klasyfikował
żądanie pedałowania na podstawie **filtrowanej kadencji** i przy `(cadence_x8>>3) <= 15`
ograniczał je do 5–7 km/h, czyli do **twardego zera** przy prędkości jazdy. Filtr jest
zerowany przy każdym zatrzymaniu pedałowania i odbudowuje się wykładniczo:

| Kadencja | Obrót korby, zanim limiter przestanie zerować |
|---:|---|
| 60 obr/min | 45° |
| 30 obr/min | 90° |
| 20 obr/min | 180° |
| ≤ 15 obr/min | **NIGDY** |

Wolne narastanie średniej RUN opisane niżej jest **realne, ale wtórne** — daje stopniowe
osłabienie, a nie twarde zero, i nie tłumaczy przypadku „nigdy".

Naprawa: `FW-091_LIMIT_SOURCE_CLASSIFICATION.md`. Mechanizm z tej karty zostaje w kodzie
**wyłączony** (`TORQUE_RUN_ATTACK_STEPS = 0`), żeby najpierw zobaczyć zachowanie bez ukrytej
blokady kadencji. Włączenie to zmiana jednej stałej na 8, a zachowanie jest pokryte testami.

## 1. Zgłoszenie z jazdy

Po 0.0289 właściciel: w trakcie jazdy, gdy moc zaniknie, ponowne złapanie jest
**niepowtarzalne** — raz wystarczy muśnięcie pedału, a czasem trzeba przejechać
**pół obrotu z naciskiem**. Druga obserwacja: **na wyższym poziomie wspomagania łatwiej
złapać**, jakby próg się obniżał.

## 2. Przyczyna — skutek uboczny FW-085

Do wzoru wspomagania trafia estymator RUN (`src/assist_modes.c:560`), a po FW-085 jest to
**średnia krocząca po 180° obrotu korby** (48 próbek, po jednej na krok kwadratury). Pełne
odzwierciedlenie nowego, wyższego nacisku wymaga więc **48 kroków = pół obrotu**:

| Przejechane | Widziany nacisk | Wspomaganie w eMTB (kwadrat) |
|---|---|---|
| 45° | 25% | ~6% |
| 90° | 50% | ~25% |
| 180° | 100% | 100% |

**Niepowtarzalność** bierze się z zawartości bufora: po krótkim zaniku mocy są w nim jeszcze
spore wartości (muśnięcie wystarcza), po dłuższym toczeniu się — same wartości bliskie zeru
(trzeba wypełnić od nowa).

**Uzbrojony latch tego nie ratuje:** `torque_input_seed_run()` wypełnia bufor natychmiast,
ale jest wołane **wyłącznie przy UZBRAJANIU latcha** (`src/ride_control.c:233`). Latch trzyma
się przez 1400 ms lekkiego pedałowania, więc przy krótkim zaniku się nie rozbraja i nie ma
ponownego zasiania.

### 2.1. Obserwacja o poziomach potwierdza diagnozę

`demand_permille = load × support_ratio / 3000`. Przy buforze wypełnionym w 25% o przełamaniu
oporów decyduje iloczyn ze wsparciem — na poziomie 300% zapotrzebowanie jest **6×** większe
niż na 50%, więc silnik rusza wcześniej. **Gdyby winny był próg w kilogramach, poziom nie
robiłby różnicy** — próg kg jest twardym „tak/nie" i nie skaluje się wsparciem.

### 2.2. Skala regresji

Stary filtr (EMA 300 ms) w przeliczeniu na kąt korby: 54° przy 30 obr/min, 108° przy 60,
162° przy 90. Nowy: zawsze 180°. Na wolnych obrotach reakcja na **wzrost** jest ok. 3×
późniejsza — to cena uniezależnienia od kadencji.

## 3. Zmiana

Uśrednianie ma zabijać **dołki** między nogami; nie ma powodu, by opóźniało **wzrost**.
Dodany zostaje **atak przy utrzymanym wzroście**: jeśli przez `TORQUE_RUN_ATTACK_STEPS`
kolejnych kroków próbka przekracza bieżącą średnią, bufor jest zasiewany tak jak przy
uzbrojeniu latcha.

Asymetria ma precedens w tym kodzie — filtry mocy mają narastanie 150 ms i opadanie 375 ms.

### 3.1. Progi dobrane tak, by pedałowanie NIGDY ich nie wyzwoliło

Szczyt impulsu nogi (wyprostowana sinusoida) to ok. **1,57× jego własnej średniej**. Dlatego:

| Warunek | Wartość | Po co |
|---|---:|---|
| stosunek do średniej | **2,0×** | leży ponad szczytem tętnienia (1,57×) |
| kolejne kroki | **8** (30°) | jeden szczyt nogi trwa krócej — wymusza trwały wzrost |
| minimalna różnica bezwzględna | **10** (`TORQUE_ASSIST_DEADBAND_NATIVE`) | przy średniej bliskiej zeru sam stosunek spełnia szum (średnia 3, próbka 7) |

**Oba warunki muszą zachodzić naraz.** Pojedynczy cichy krok zeruje licznik — liczy się
wyłącznie wzrost utrzymany.

Kierunek doboru jest celowo ostrożny: pompowanie z FW-085 zostało potwierdzone jako
naprawione w terenie i **nie wolno oddać tego zysku za szybszy chwyt**. Jeśli po jeździe
złapanie nadal będzie leniwe, luzujemy próg — nie odwrotnie.

## 3.2. Rozważona alternatywa: maksimum zamiast średniej (wariant TSDZ2)

Firmware OSF dla TSDZ2 ma **dwa** podejścia w różnych wersjach. Pierwsze to średnia po
pełnym obrocie — to na nim wzorowane jest FW-085 i ma ono dokładnie tę wadę, którą opisuje
ta karta. Drugie liczy wspomaganie jako:

```text
max(wartość bieżąca, maksimum z bieżącego obrotu, maksimum z poprzedniego obrotu)
```

To rozwiązuje problem **konstrukcją, bez żadnych progów**: wzrost widać natychmiast (człon
„wartość bieżąca"), a dołek między nogami nie istnieje, bo przykrywa go maksimum z obrotu.
Pod względem inżynierskim jest to elegantsze niż atak progowy z §3.

**Odrzucone z jednego, konkretnego powodu: przeskalowałoby całe wspomaganie.** Szczyt
impulsu nogi to ok. 1,57× jego średniej, więc przy tym samym wysiłku:

| Tryb | Zmiana magnitudy po przejściu na maksimum |
|---|---|
| Power Linear / Progressive / Curve | ~1,57× |
| **eMTB** (moment do kwadratu) | **~2,5×** |

Wymagałoby to przestrojenia **wszystkich poziomów na wszystkich bankach** — co zresztą
sygnalizuje sama dokumentacja TSDZ2 („requiring parameter adjustments when switching between
averaging and max torque values"). Właściciel potwierdził po jeździe, że eMTB czuje się teraz
dobrze; oddanie wystrojonej skali za elegancję konstrukcji byłoby złym interesem.

FW-090 daje ten sam efekt użytkowy przy **zachowanej skali mocy**, kosztem progów, które
trzeba dobrać — stąd ich konserwatywny dobór i testy 1/2 pilnujące tętnienia.

Gdyby jednak progi okazały się w praktyce kłopotliwe, wariant z maksimum jest znanym,
sprawdzonym w innym projekcie rozwiązaniem zapasowym — pod warunkiem zaplanowania
przestrojenia poziomów.

### 3.2.1. Jak to jest wybierane w TSDZ (z lektury źródła)

Sprawdzone w `mstrens/OSF` (plik `ebike_app.c`). Wybór jest **kompilacyjny**, nie jest ani
opcją użytkownika, ani przełącznikiem w czasie jazdy:

```c
#define USE_SPIDER_LOGIC_FOR_TORQUE     /* > 0 -> srednia, == 0 -> maksimum */
```

- **średnia**: bufor 20 próbek na obrót (po jednej na przejście czujnika kadencji),
  wynik `ui16_TSum / 20`;
- **maksimum**: `max(próbka bieżąca, ui16_adc_torque_actual_rotation,
  ui16_adc_torque_previous_rotation)`.

W wariancie z maksimum jest dodatkowo **warunek zależny od kadencji**:

```c
#define PEDAL_CADENCE_MIN_FOR_USING_ROTATION 30
if (ui8_pedal_cadence_RPM > PEDAL_CADENCE_MIN_FOR_USING_ROTATION) { /* dopiero tu max z obrotow */ }
```

Poniżej 30 obr/min używana jest **wyłącznie wartość chwilowa**, bez pamięci obrotu — czyli
przy ruszaniu i bardzo wolnym mieleniu reakcja jest z definicji natychmiastowa.

**Dwa wnioski istotne dla tej karty:**

1. Wariant ze średnią **nie ma żadnego mechanizmu szybkiego ataku** — czyli ma dokładnie tę
   wadę, którą właściciel wykrył na rowerze. Nasze znalezisko jest wpisane w to podejście,
   a nie specyficzne dla naszej implementacji.
2. Wariant z maksimum jest odporny **strukturalnie**, bo człon „próbka bieżąca" jest zawsze
   w maksimum — wzrost widać natychmiast przy każdej kadencji, bez progów.

Zastrzeżenie: `mstrens/OSF` celuje w **TSDZ8** (obecny `config_tsdz8.h`). Rodzina i logika
momentu są wspólne, ale gałęzie stricte TSDZ2 mogą różnić się szczegółami.

## 4. Testy

`tests/fw090_run_attack.js`:

1. **Najważniejszy: zwykłe pedałowanie po napełnieniu bufora NIGDY nie wyzwala ataku**, a
   tłumienie tętnienia z FW-085 pozostaje nienaruszone.
2. To samo w całym zakresie wysiłku (amplitudy 200–3000) — margines jest stosunkiem, więc
   musi trzymać tak samo przy lekkim kręceniu i przy mocnym docisku.
3. Po okresie toczenia się nacisk jest odzwierciedlony w ~30°, a nie po 180°.
4. Pojedynczy pik krótszy niż wymagane 8 kroków **nie** zasiewa; licznik zeruje się na
   pierwszym cichym kroku.
5. Opadanie nadal wolne — spadek do zera nigdy nie zasiewa.
6. Szum wokół średniej bliskiej zeru nie zasiewa (podłoga bezwzględna), ale prawdziwy wzrost
   ponad podłogę już tak.

**Znane, zamierzone zachowanie (test 2b):** napełnianie **pustego** bufora wyzwala jedno
wczesne zasianie ok. 8. kroku. Jest nieszkodliwe — przyspiesza tylko zbieżność z pustego
bufora, a na rowerze latch i tak zasiewa bufor, zanim wspomaganie ruszy. Udokumentowane,
żeby nie pomylić go z regresją, przed którą chronią testy 1 i 2.

## 5. Kryteria odbioru

- ponowne złapanie wspomagania w jeździe jest powtarzalne, bez „pół obrotu";
- **pompowanie na podjeździe NIE wraca** — to jest warunek nadrzędny;
- opadanie/dołki między nogami nadal wygładzone;
- różnica między poziomami wspomagania przestaje decydować o tym, czy silnik w ogóle ruszy;
- pełny zestaw testów firmware i Canable bez regresji.
