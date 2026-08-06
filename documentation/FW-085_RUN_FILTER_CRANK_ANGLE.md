# FW-085 — Okno filtra RUN w stopniach korby zamiast w milisekundach

- **Data:** 2026-08-04, wdrożone 2026-08-05
- **Status:** WDROŻONE — kod napisany, testy hostowe zielone. **NIEPRZETESTOWANE NA ROWERZE.**
  Wybrano wariant z buforem kołowym (§4.2). Build firmware jeszcze nie wykonany.
- **Cel:** usunąć zależność wygładzania nacisku od kadencji, żeby jedna nastawa była
  poprawna zarówno przy 50, jak i przy 110 obr/min.
- **Zakres:** `torque_input`, wywołanie w `main.c`, tuning blob v7, UI Canable (Dynamics),
  testy hostowe.
- **Powiązane:** `FW-033_TORQUE_CONDITIONING_PLAN.md` (wprowadził filtr RUN),
  `FW-025_PAS_STOP_CUT.md` (ten sam wzorzec adaptacji do tempa korby).

## 1. Problem

Właściciel zgłosił na buildzie 0.0287 pompowanie mocy na stromych podjazdach przy
kadencji 50–60 obr/min: „moc zdąży opaść, zanim druga noga ją podtrzyma". To nie jest
regresja 0.0287 — zachowanie istnieje od FW-033.

W firmware działają **dwa** filtry nacisku, w dwóch różnych domenach:

| Filtr | Gdzie | Domena | Odporny na kadencję |
|---|---|---|---|
| `MS.torque_filtered` | `src/main.c:1678-1680` | kąt korby — EMA co krok kwadratury (96/obrót, co 3,75°) | TAK |
| `run_filter_q` (FW-033) | `src/torque_input.c:140-158` | czas — `run_filter_ms × TORQUE_INPUT_TICKS_PER_MS` | **NIE** |

Komentarz przy tym drugim (`src/torque_input.c:136-137`) deklaruje zamiar:

> „it averages pedal load over **a fraction of a crank turn**"

Implementacja go nie realizuje, bo okno jest zapisane w milisekundach. Ułamek obrotu,
który filtr faktycznie obejmuje, zmienia się z kadencją:

| Kadencja | Czas obrotu | 300 ms obejmuje | Skutek |
|---:|---:|---:|---|
| 110 obr/min | 545 ms | 55% obrotu | gładko |
| 90 obr/min | 667 ms | 45% obrotu | gładko |
| 60 obr/min | 1000 ms | 30% obrotu | odczuwalne |
| **50 obr/min** | 1200 ms | **25% obrotu** | **pompowanie** |

Podniesienie wartości w ms nie rozwiązuje problemu, tylko przesuwa go na drugi koniec
zakresu: 700 ms naprawia podjazd, ale przy 90 obr/min obejmuje 105% obrotu, a przy
110 obr/min 128% — czyli robi się ospałe. **Żadna pojedyncza wartość w milisekundach nie
jest poprawna dla całego zakresu kadencji.** To jest istota problemu.

Referencyjnie: firmware OSF dla TSDZ2 rozwiązuje to trzymając bufor momentu obejmujący
pełny obrót korby i licząc z niego średnią kroczącą — czyli okno w stopniach, nie w ms.

## 2. Rozwiązanie

Okno filtra RUN wyrażone w **stopniach obrotu korby**. Stała czasowa wyliczana na bieżąco
z mierzonego tempa korby, dokładnie tym samym wzorcem, który już działa w FW-025.

Efekt docelowy — okno i tłumienie przestają zależeć od kadencji:

| Kadencja | Dziś (300 ms) | Po zmianie (okno 180°) |
|---:|---|---|
| 50 obr/min | okno 25% · tętnienie 30% | okno 50% · tętnienie 16% |
| 60 obr/min | okno 30% · tętnienie 26% | okno 50% · tętnienie 16% |
| 90 obr/min | okno 45% · tętnienie 17% | okno 50% · tętnienie 16% |
| 110 obr/min | okno 55% · tętnienie 14% | okno 50% · tętnienie 16% |

Przy szybkim kręceniu reakcja będzie **szybsza** niż po dostrojeniu obecnego parametru
do podjazdów, bo filtr sam się skraca wraz ze wzrostem kadencji.

## 3. Parametr użytkownika

**Decyzja właściciela: jedno pole, zmienione na stopnie.** Nie dodajemy drugiego pokrętła;
ograniczniki bezpieczeństwa są stałymi w firmware i nie są wystawiane do UI — tak samo jak
`PAS_STOP_TICKS`/`PAS_STOP_TICKS_MAX` w FW-025.

| Parametr | Było | Będzie |
|---|---|---|
| nazwa wewnętrzna | `torque_run_filter_ms` | `torque_run_window_deg` |
| jednostka | ms | stopnie obrotu korby |
| zakres | 0–1000 | 0–360, krok 15 |
| domyślna | 300 | **180** (pół obrotu = jedna noga) |
| `0` | filtr wyłączony (sygnał surowy) | **bez zmian: filtr wyłączony** |

Krok 15° odpowiada dokładnie 4 krokom kwadratury, więc każda nastawa z UI mapuje się na
całkowitą liczbę kroków bez zaokrągleń.

## 4. Obliczenie — filtr taktowany krokiem korby, nie zegarem

**Nie liczymy stałej czasowej.** Zamiast wyliczać, ile milisekund odpowiada zadanemu
kątowi, przenosimy aktualizację filtra z pętli sterowania (4 kHz) do **obsługi kroku
kwadratury PAS**. Okno jest wtedy wyrażone w krokach korby z definicji, a nie przez
przeliczenie.

Kwadratura daje 96 kroków na obrót (`src/main.c:1677`), czyli 3,75° na krok:

```c
window_steps = window_deg / 3.75     /* 180 deg -> 48 krokow, 360 deg -> 96 */
```

i filtr wykonuje **jeden krok aktualizacji na jedno przejście kwadratury do przodu**.
Między krokami wartość jest przytrzymywana — dokładnie tak, jak robi to sąsiedni
`MS.torque_filtered` (`src/main.c:1678-1680`).

### 4.1. Dlaczego to usuwa ogranicznik, przepełnienie i ścieżkę awaryjną

Pierwotny szkic tej karty wyliczał `filter_ticks = window_steps × pas_last_period_ticks`
i musiał ograniczać wynik, pilnować 32-bitowego pośrednika oraz mieć wartość awaryjną na
czas, gdy tempo korby nie jest jeszcze znane. **Wszystkie trzy problemy znikają**, gdy
filtr jest taktowany krokiem korby:

| Sytuacja | Wariant z przeliczaniem na ms | Taktowanie krokiem korby |
|---|---|---|
| bardzo wolne kręcenie (20 obr/min) | stała czasowa rośnie do 1500 ms → potrzebny ogranicznik | okno **nadal** równe zadanemu kątowi; nie ma czego ograniczać |
| korba zatrzymana | stała czasowa rośnie bez granic | brak nowych kroków → filtr po prostu stoi |
| start, brak pomiaru tempa | trzeba wartości awaryjnej w ms | uśrednia to, co już przyszło |
| arytmetyka | `96 × 64000` przekracza `uint16_t` | brak mnożenia |

To jest ta sama konstrukcja, którą stosuje firmware OSF dla TSDZ2 (bufor obejmujący pełny
obrót, próbka na impuls PAS, średnia z bufora; przy niepełnym obrocie średnia z tego, co
jest) — i którą ten kod **już stosuje** dla `MS.torque_filtered`.

### 4.2. Do rozstrzygnięcia: EMA czy prawdziwa średnia krocząca

| Wariant | RAM | Skuteczność | Uwagi |
|---|---:|---|---|
| **EMA co krok** (τ = `window_steps`) | ~0 B | tłumi tętnienie do ~16% | zgodny z sąsiednim `MS.torque_filtered`, najmniejsza zmiana |
| **Bufor kołowy + średnia** (jak TSDZ2) | 96 × 2 B = **192 B** | przy oknie równym pełnemu obrotowi **kasuje tętnienie niemal do zera** | rozwiązanie sprawdzone w TSDZ2; średnia po pełnym okresie znosi zaburzenie, a nie tylko je tłumi |

**Rekomendacja: bufor kołowy.** GD32F303RCT6 ma 48 KB RAM, więc 192 B jest bez znaczenia,
a zysk jakościowy jest realny — średnia po pełnym obrocie znosi tętnienie nogi dokładnie,
podczas gdy EMA je tylko osłabia. Jeżeli jednak zajętość RAM okaże się problemem,
EMA jest w pełni akceptowalnym wariantem zapasowym i nie zmienia reszty tej karty.

Nie zmieniać zachowania `torque_input_seed_run()` (`src/ride_control.c:233`) — zasianie
filtru wartością szybką przy starcie zostaje bez zmian (przy wariancie z buforem: wypełnić
bufor wartością zasiewu).

## 5. Interfejs modułu

`torque_input.c` musi pozostać testowalny na komputerze, więc **nie wolno mu sięgać po
globalne `pas_*`**. Przy taktowaniu krokiem korby interfejs jest prostszy niż w pierwotnym
szkicu — nie trzeba w ogóle przekazywać tempa korby:

```c
/* zastepuje torque_input_set_run_filter_ms() */
void torque_input_set_run_window_deg(uint16_t window_deg);

/* NOWE: jeden krok filtra RUN. Wolane raz na przejscie kwadratury do przodu. */
void torque_input_run_filter_step(void);
```

Wywołania:

- `torque_input_set_run_window_deg(tuning_config_assist_torque_run_window_deg())`
  w `src/main.c:1752` (tam gdzie dziś stoi `torque_input_set_run_filter_ms`);
- `torque_input_run_filter_step()` w gałęzi kroku do przodu dekodera PAS
  (`src/main.c:1671-1681`), obok istniejącej aktualizacji `MS.torque_filtered`.

Test hostowy steruje filtrem, wołając `torque_input_run_filter_step()` zadaną liczbę razy —
bez symulowania zegara, co dodatkowo upraszcza testy.

Usunąć `TORQUE_RUN_FILTER_MS_DEFAULT` i `TORQUE_RUN_FILTER_MS_MAX`
(`inc/torque_input.h:37-38`) na rzecz stałych okna w stopniach.

## 6. Tuning blob v7

**Transportu nie trzeba ruszać.** Blob ma dziś 32 B z trzema wolnymi u16 na offsetach
24–29 (`src/tuning_config.c:125-126`), a parametr i tak zajmuje istniejący slot.

1. Zachować **offset 20** — to ta sama nastawa, tylko w innej jednostce.
2. Podbić `TUNING_VERSION` do 7.
3. Zmienić ogranicznik zapisu/odczytu z `TUNING_TORQUE_RUN_FILTER_MS_MAX` (1000)
   na `TUNING_TORQUE_RUN_WINDOW_DEG_MAX` (360).

### 6.1. Migracja — punkt krytyczny

Stara wartość w ms **nie ma sensownego odpowiednika w stopniach**, bo jej znaczenie
zależało od kadencji, przy której właściciel ją stroił. Nie wolno jej przeliczać ani
wpisywać wprost — zapisane 700 zostałoby zinterpretowane jako 700°, a po ograniczeniu do
360° dałoby ciche, niezamierzone podwojenie filtracji.

Reguła migracji z bloba v6 i starszych:

```text
stara wartosc == 0  ->  0    (filtr byl wylaczony, zostaje wylaczony)
stara wartosc  > 0  ->  180  (nowa wartosc domyslna)
```

To musi być migracja jednorazowa, testowana przeciw wersji **< 7**, a nie przeciw
`TUNING_VERSION` — inaczej odpali przy każdym kolejnym podbiciu wersji i skasuje
strojenie właściciela. Ta pułapka jest już opisana w `src/tuning_config.c:179-184`.

## 7. Canable

`ui/js/evistdrive/dynamics.js` — pole `assist_torque_run_filter_ms` (linia 29):

- klucz → `assist_torque_run_window_deg`, `unit: '°'`, `min: 0, max: 360, step: 15`;
- etykieta: `RUN torque smoothing (anti-pulse)` zostaje — objaw, który leczy, się nie zmienił;
- domyślna w `dynamics.js:89` → `180`;
- dymek do przepisania. Musi powiedzieć wprost, że okno jest **ułamkiem obrotu korby, nie
  czasem**, i że dlatego jedna nastawa działa tak samo na wolnym podjeździe i przy szybkim
  kręceniu. Sugerowane presety (Aggressive / Normal / Smooth): 90 / 180 / 270°.
- parser/serializer bloba musi odrzucić v7 wysyłany do sterownika zgłaszającego v6.

## 8. Testy hostowe

`tests/fw085_run_window.js` albo równoważny:

Testy sterują filtrem, wołając `torque_input_run_filter_step()` — bez symulowania zegara.

1. `window_deg = 0` daje sygnał surowy — przebieg identyczny jak dziś przy 0.
2. `window_deg = 180` odpowiada dokładnie 48 krokom, `360` — 96 krokom (mapowanie bez
   zaokrągleń dla całego zakresu co 15°).
3. **Tłumienie tętnienia jest identyczne niezależnie od tego, jak gęsto w czasie
   przychodzą kroki** — to jest główne kryterium tej karty. Podać ten sam przebieg
   tętnienia raz przy odstępach odpowiadających 50 obr/min, raz 110 obr/min, i porównać
   wynik. Musi wyjść ten sam.
4. Zatrzymanie kroków (korba stoi) zamraża wartość filtra i nie powoduje jej dryfu
   ani zerowania.
5. Niepełne okno po starcie: przy mniejszej liczbie kroków niż `window_steps` wynik jest
   średnią z dostępnych próbek, a nie wartością zaniżoną przez puste miejsca w buforze.
6. Skrajne wartości nacisku przez pełne okno nie przepełniają akumulatora sumy.
7. Migracja: v6 z wartością 0 → 0; v6 z 300 → 180; v6 z 700 → 180.
8. Migracja nie odpala ponownie przy blobie v7.
9. Round-trip v7 zachowuje wartość dla całego zakresu 0–360 co 15°.
10. `torque_input_seed_run()` działa jak dotąd (przy wariancie z buforem — wypełnia bufor).

## 9. Test stanowiskowy i terenowy

1. Ten sam podjazd co przy zgłoszeniu, kadencja 50–60 obr/min: pompowanie ma zniknąć.
2. **Ten sam zestaw nastaw na płaskim przy 85–95 obr/min: reakcja nie może być ospała.**
   To jest cały sens zmiany — jeżeli trzeba przestrajać między terenami, cel nie został
   osiągnięty.
3. Porównać reakcję na świadome mocne dociśnięcie przy obu kadencjach.
4. Sprawdzić ruszanie z miejsca (ścieżka awaryjna, brak pomiaru tempa korby) — start ma
   być nieodróżnialny od obecnego.
5. Zapis RAM → zapis flash → restart → odczyt.
6. Sprawdzić wszystkie tryby: Power Linear, Progressive, eMTB, Torque, Power Curve.

## 10. Kryteria odbioru

- tłumienie tętnienia nogi jest **stałe w całym zakresie kadencji** (główne kryterium);
- przy `0` przebieg jest identyczny z obecnym `0`;
- start z miejsca nie zmienia się względem dzisiejszego zachowania;
- zatrzymana korba zamraża filtr i nie wymaga żadnego ogranicznika czasowego;
- w kodzie filtra RUN **nie ma przeliczania kąta na milisekundy** — gdyby się pojawiło,
  wracają wszystkie problemy opisane w §4.1;
- migracja z v6 nie zamienia milisekund na stopnie i nie odpala powtórnie;
- `torque_input` pozostaje testowalny na komputerze, bez sięgania po globalne `pas_*`;
- Canable nie wysyła v7 do sterownika obsługującego v6.

## 11. Kolejność wdrożenia

1. Stałe okna i nowy interfejs `torque_input_set_run_window()`, jeszcze bez zmian w UI.
2. Testy hostowe modułu, w tym pomiar tłumienia przy dwóch kadencjach.
3. Wpięcie w `main.c:1752`, przekazanie `pas_last_period_ticks`.
4. Tuning blob v7 z migracją i testami round-trip.
5. Canable: pole, dymek, presety, ochrona wersji.
6. Build diagnostyczny, test z kołem uniesionym.
7. Test terenowy: podjazd 50 obr/min oraz płaskie 90 obr/min — oba muszą wypaść dobrze.
8. `CHANGELOG.md`, numer builda.

## 11a. Co faktycznie zostało zrobione (2026-08-05)

Wariant: **bufor kołowy**, zgodnie z rekomendacją §4.2 (192 B, kasuje tętnienie).

| Plik | Zmiana |
|---|---|
| `inc/torque_input.h` | stałe okna w stopniach zamiast `TORQUE_RUN_FILTER_MS_*`; `torque_input_set_run_window_deg()`, `torque_input_run_filter_step()` |
| `src/torque_input.c` | średnia krocząca po buforze kołowym z sumą przyrostową; usunięty filtr czasowy |
| `src/main.c` | `torque_input_run_filter_step()` w gałęzi kroku do przodu dekodera PAS (obok EMA `MS.torque_filtered`); nowy setter w miejscu starego |
| `inc/tuning_config.h`, `src/tuning_config.c` | blob **v7**, migracja jednostki, akceptacja v6 |
| Canable: `bafang-parser.js`, `canbus.js`, `ui/js/evistdrive/dynamics.js` | odczyt/zapis v7, negocjacja w dół, pole w stopniach |
| `tests/fw085_run_window.js`, Canable `tests/fw085_tuning_window_roundtrip.js` | testy z §8 |

**Dwa błędy wykryte przez testy przy wdrożeniu — warte zapamiętania:**

1. Podbicie `TUNING_VERSION` z 6 na 7 sprawiłoby, że warunek `version >= TUNING_VERSION`
   przy `start_steps` przestałby obejmować blob v6 — każdy zapisany v6 **po cichu straciłby
   ustawione kroki startu**. Poprawione na `>= TUNING_VERSION_V6`. To dokładnie ta klasa
   pułapki, przed którą ostrzega komentarz FW-053 w tym samym pliku.
2. Parser Canable odrzucał wersję 7 (`d[2] > 6`), więc każdy odczyt nowego bloba kończyłby
   się błędem magic/version. Wykryte przez nowy test round-tripu — istniejący test
   pokrywał wyłącznie v6 i v5.

**Nie zweryfikowano:** zachowania na rowerze. Kryteria z §9 czekają na jazdę.

## 12. Czego nie robić

- Nie zmieniać przy okazji `power_rise_filter_ms` ani `power_fall_filter_ms` na domenę
  kąta. Jedna zmiana, mierzalny efekt; jeżeli filtr RUN wystarczy, tamte zostają.
- **Nie wracać do przeliczania kąta na stałą czasową w milisekundach.** To był pierwotny
  szkic tej karty; sprowadza z powrotem ogranicznik, ochronę przed przepełnieniem i
  ścieżkę awaryjną, których taktowanie krokiem korby w ogóle nie potrzebuje (§4.1).
- Nie przeliczać starej wartości w ms na stopnie.
- Nie ruszać drugiego filtra (`MS.torque_filtered`) — on już jest w domenie kąta i działa.
