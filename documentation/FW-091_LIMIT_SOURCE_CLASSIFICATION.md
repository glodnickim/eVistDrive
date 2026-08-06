# FW-091 — Limiter klasyfikuje ŹRÓDŁO żądania, a nie filtrowaną kadencję

- **Data:** 2026-08-06
- **Status:** DO WDROŻENIA
- **Cel:** siła i kąt potrzebne do ponownego uruchomienia wspomagania w jeździe mają zależeć
  wyłącznie od ustawionego progu w kg i liczby kroków PAS — nigdy od stanu filtra kadencji.
- **Zakres:** `inc/assist_limits.h`, `src/assist_limits.c`, `src/ride_control.c`, testy.
  Bez zmian w transporcie, blobie i UI.
- **Powiązane:** `FW-090_RUN_FAST_ATTACK.md` (degradowane do eksperymentu),
  `FW-089`, `FW-030` (manetka).

## 1. Usterka

`src/assist_limits.c:21-31` odpowiada na pytanie „czy rowerzysta pedałuje" **filtrowaną
kadencją**:

```c
if ((input->cadence_filtered_x8 >> 3) > 15) {
    limited = map(speed, limit, limit + 200, limited, 0);   /* normalny limit */
} else {
    limited = map(speed, 500, 700, limited, 0);             /* 5..7 km/h -> ZERO */
}
```

To błąd znaczeniowy: filtrowana kadencja jest **pomiarem prędkości obrotu**, a nie
potwierdzeniem, że korba się kręci. Przy prędkości ≥ 7 km/h druga gałąź daje **twarde zero**.

`uint16_cadence_filtered` jest zerowane przy wykryciu postoju (`src/main.c:1750`) i odbudowuje
się wykładniczo (`f = f - (f>>3) + cadence`, zbieżne do 8× kadencji), a próg wymaga `f >= 128`:

| Kadencja | Obrót korby, zanim limiter przestanie zerować |
|---:|---|
| 60 obr/min | 45° |
| 45 obr/min | 60° |
| 30 obr/min | 90° |
| 20 obr/min | **180°** |
| 16 obr/min | **390°** (ponad pełny obrót) |
| **≤ 15 obr/min** | **NIGDY** — filtr zbiega do 120, próg to 128 |

**To jest główna przyczyna** zgłoszonego objawu „raz muśnięcie wystarcza, a czasem trzeba
pół obrotu albo więcej" — łącznie z jego niepowtarzalnością, bo czas zależy od kadencji
sprzed zaniku. Przy stałym wolnym pedałowaniu (≤ 15 obr/min) wspomaganie powyżej 7 km/h
**nie wraca w ogóle**.

### 1.1. Korekta wcześniejszej diagnozy

FW-090 (wolne narastanie średniej RUN) opisywało zjawisko **wtórne**. Jest realne, ale daje
stopniowe osłabienie, a nie twarde zero, i nie tłumaczy przypadku „nigdy". Ta karta zastępuje
je jako właściwa naprawa — patrz §5.

## 2. Zmiana: klasyfikacja źródła

```c
typedef enum {
    ASSIST_LIMIT_SOURCE_PEDAL_CONFIRMED = 0, /* legalnie potwierdzone pedałowanie */
    ASSIST_LIMIT_SOURCE_NON_PEDAL = 1        /* manetka, start bez obrotu korby, FW-084 Extended Boost */
} assist_limit_source_t;
```

W trybie legal:

- `PEDAL_CONFIRMED` → normalny limit prędkości (`speed_limit ... +2 km/h`);
- `NON_PEDAL` → limit 5…7 km/h, jak dotąd.

Potwierdzeniem pedałowania jest **sam zatrzask** — a ten nie uzbroi się bez prawidłowego
kierunku korby i wymaganej liczby kroków PAS (`crank_moving_enough`, `ride_control.c:213`),
ponad progiem w kg, poziomem i zabezpieczeniami. To są warunki, które rowerzysta widzi
i ustawia. Filtrowana kadencja przestaje o czymkolwiek decydować.

### 2.1. Poprawiony błąd: klasyfikacja nie może zależeć od `assist_without_rotation_active`

Pierwsza wersja tej karty wymagała dodatkowo `&& !mode_output.assist_without_rotation_active`.
To było **błędne i szkodliwe**.

Ta flaga (`assist_modes.c:563`) jest podnoszona wyłącznie na podstawie „kadencja = 0 **oraz**
nacisk ponad próg **postoju**" — **nigdy nie sprawdza, czy korba się kręci**. W jeździe, zanim
pojawi się pierwszy impuls kadencji, mocniejsze naciśnięcie podnosiło ją mimo że rowerzysta
demonstracyjnie pedałował. Skutek — zależność od siły nacisku, i to odwrotna:

| Nacisk | Flaga | Źródło | Wspomaganie przy 10 km/h |
|---|---|---|---|
| 0,3–0,7 kg (nad progiem jazdy, pod progiem postoju) | 0 | PEDAL | **14** |
| > 0,7 kg (mocniej!) | 1 | NON_PEDAL | **0** |

**Mocniejsze pedałowanie dawało mniej wspomagania.** Poprawka: klasyfikacja zależy wyłącznie
od zatrzasku. Zatrzask już potwierdza ruch korby, więc flaga nie ma prawa unieważniać
fizycznie potwierdzonego pedałowania. Pokryte testem 8, który zawiesza się na starej wersji.

## 3. Rozdzielenie prądu pedałowania od manetki

Dziś manetka jest wmieszana w `iq_target` **przed** limiterem (`src/ride_control.c:263-264`),
więc jeden limiter obsługuje oba źródła. Samo dodanie flagi „pedałuje" byłoby wtedy
**niebezpieczne**: aktywny zatrzask przepuściłby również prąd manetki na normalny limit
prędkości.

Dlatego oba prądy liczone są osobno i osobno limitowane:

```text
pedal_iq    = wynik trybu -> zatrzask + podłoga min_iq
pedal_iq    = assist_limits_apply(pedal_iq, PEDAL_CONFIRMED)

throttle_iq = assist_limits_apply(throttle_iq, NON_PEDAL)

iq_target   = max(pedal_iq, throttle_iq)
```

Dzięki temu:

- prawidłowe pedałowanie dostaje normalny limit prędkości;
- **manetka nadal nie przekroczy 7 km/h bez pedałowania** — nawet przy uzbrojonym zatrzasku;
- Walk Assist pozostaje pod własnym ogranicznikiem (`walk_active` omija cały blok legal);
- przyszły Extended Boost (FW-084) dostanie świadomie wybraną klasyfikację.

Ograniczniki napięcia i temperatury działają na **oba** źródła identycznie jak dotąd.

## 4. Podłoga minimalnego prądu

Podłoga z `src/ride_control.c:248` zostaje jako właściwy mechanizm, z zaokrągleniem w górę,
żeby małe procenty nie ginęły w dzieleniu całkowitym:

```c
min_iq = (ride_core_iq_limit * min_iq_pct + 99) / 100;
```

**Kontrakt:** po potwierdzeniu ruchu PAS i przekroczeniu progu kg żądany prąd pedałowania
osiąga co najmniej skonfigurowane `min_iq` — chyba że działa prawdziwy ogranicznik
bezpieczeństwa.

**Czego NIE robimy:** nie wymuszamy `Iq >= 1` po ogranicznikach. To obeszłoby hamulec,
przegrzanie, zbyt niskie napięcie, limit prędkości i błąd czujnika. Poza tym Iq=1 to ok.
95 mA i i tak nie ruszy napędu. Przy `Minimum Iq = 0%` gwarancja jest świadomie wyłączona.

## 5. Status FW-090

FW-090 **nie jest podstawą tej naprawy** i zostaje zdegradowane do mechanizmu
eksperymentalnego, **domyślnie wyłączonego** (`TORQUE_RUN_ATTACK_STEPS = 0`). Najpierw trzeba
sprawdzić zachowanie bez ukrytej blokady kadencji — możliwe, że szybki atak nie jest wcale
potrzebny. Kod i testy zostają, żeby dało się go włączyć jedną stałą.

## 6. Testy

`tests/fw091_limit_source.js` — pełna tabela z audytu:

| Przypadek | Oczekiwane |
|---|---|
| 10 km/h, filtrowana kadencja 0, 3 kroki, TQ ≥ 0,3 kg | `iq_after_limits >= min_iq` |
| To samo przy 20 obr/min | reakcja po 3 krokach, nie po ~180° |
| TQ poniżej progu | Iq = 0 |
| Za mało kroków PAS | Iq = 0 |
| Krok wstecz | zatrzask wyłączony |
| Poziom 0 | Iq = 0 |
| Hamulec / błąd / przegrzanie | zejście do zera |
| Prędkość ponad normalnym limitem | Iq = 0 |
| Manetka bez pedałowania przy ≥ 7 km/h | Iq = 0 |
| Walk Assist | bez zmiany zachowania |
| `Minimum Iq = 0%` | brak wymuszonej podłogi |

Dodatkowo strukturalnie: `cadence_filtered_x8` nie może już wybierać gałęzi limitu prędkości,
a klasyfikacja nie może zależeć od `assist_without_rotation_active`.

**Test jest integracyjny.** `latched` i `withoutRotation` są w modelu **wyprowadzane**
z surowego stanu (nacisk w kg, liczba kroków do przodu, kierunek, kadencja, prędkość,
poziom, safety) — dokładnie tak jak w `assist_modes.c` i `ride_control.c`. Poprzednia wersja
przyjmowała je jako gotowe wejścia i przez to potrafiła przejść, potwierdzając zachowanie,
którego firmware nie ma. Pokrycie kończy się na `iq_after_limits`; smooth start, preload
i rampa są poza zakresem tej karty.

## 7. Kryterium odbioru na rowerze

Przy jadącym rowerze siła potrzebna do ponownego uruchomienia zależy **wyłącznie** od
ustawionego progu jazdy w kg, a kąt obrotu jest **stały** i wynika z liczby kroków PAS
(domyślnie ok. 11°) — niezależnie od tego, czy filtr kadencji został wyzerowany.

`Release duration` nie jest częścią tej naprawy: odpowiada za wygaszanie już aktywnej mocy,
a nie za zgodę na ponowne uruchomienie.

## 8a. Znalezione przy okazji: „Assist without crank rotation" NIE działa z postoju

Audyt wykazał, a kod to potwierdza: przy rzeczywistym braku ruchu korby
`crank_moving_enough` jest fałszywe (`ride_control.c:213`), więc `ride_control.c:220-224`
**wymusza rozbrojenie zatrzasku**, a `:234-236` zeruje `iq_target`. Tryb w `assist_modes`
wystawia niezerowe żądanie, ale `ride_control` je kasuje.

**Czyli funkcja jest w ride core nieczynna z martwego punktu**, mimo że jest w UI.

Ta karta **niczego tu nie zmienia** i niczego nie obiecuje. Wcześniejsza wersja testu
twierdziła, że rower rusza z postoju — robiła to, ręcznie konstruując stan
`latched: true, withoutRotation: true`, którego firmware nie potrafi wytworzyć. Taki test
jest gorszy niż jego brak i został zastąpiony testem 8b, który dokumentuje **rzeczywiste**
zachowanie (żądanie jest, zatrzasku nie ma, prąd zero).

Do rozstrzygnięcia w osobnej karcie: albo start bez obrotu korby dostaje własną autoryzację
opartą na nacisku (z limitem 5–7 km/h, czyli klasyfikacją NON_PEDAL), albo opcja znika z UI.
Nie mieszać tego do FW-091 — to zmiana bramki startu i wymaga własnej próby terenowej.

## 7b. Aktualizacja 2026-08-06 — FW-084 Extended Boost korzysta z tej klasyfikacji

Komentarz „przyszły overrun" w `assist_limits.h` dotyczył funkcji, która wtedy nie istniała.
Istnieje od FW-084 i nazywa się **Extended Boost** (nazwa `overrun` należy wyłącznie do
nieaktywnego mechanizmu Legacy — patrz `EXTENDED_BOOST_ENABLE` w `config.h`).

Extended Boost jest klasyfikowany jako `NON_PEDAL`, i to **decyzją zapisaną wprost** w
`ride_control.c`, a nie skutkiem tego, że na zboczu PAS STOP `assist_latched` już opadł. W
trybie legal oznacza to zanik 5–7 km/h i zero powyżej 7 km/h; karta Canable mówi o tym
jeźdźcowi wprost. Zmiana tej polityki na `PEDAL_CONFIRMED` (wcześniejsze legalne pedałowanie
autoryzuje czas ACTIVE) jest decyzją produktowo-prawną, nie refaktorem — szczegóły w sekcji 16
karty FW-084.

## 8. Poza zakresem tej karty

Rozbudowa diagnostyki `0x6029` (rozdzielenie `iq_mode` / `iq_after_latch` / `iq_after_limits`
/ `iq_setpoint`, `pedal_confirmed`, przyczyna limitowania) jest potrzebna, ale wymaga podbicia
wersji bloba i zmian w parserze Canable. Wrzucona tutaj utrudniłaby przypisanie efektów
próby terenowej. Do zrobienia osobno, jeśli jazda nie rozstrzygnie.
