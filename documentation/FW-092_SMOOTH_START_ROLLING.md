# FW-092 — Smooth Start nie może uznawać toczącego się roweru za postój

- **Data:** 2026-08-06
- **Status:** WDROŻONE — testy hostowe zielone. **NIEPRZETESTOWANE NA ROWERZE.**
- **Cel:** obietnica FW-091 („reakcja zależy wyłącznie od kroków PAS") ma być prawdziwa
  także przy **włączonym** Smooth Start.
- **Zakres:** `inc/assist_start.h`, `src/assist_start.c`, `src/ride_control.c`, test.
  Bez zmian w transporcie, blobie i UI.
- **Powiązane:** `FW-091_LIMIT_SOURCE_CLASSIFICATION.md`, `FW-034_035_LEVEL0_BUMPLESS.md`.

## 1. Problem

`src/assist_start.c` uznawał stan za zatrzymany wyłącznie na podstawie:

```c
bool stopped = input->measured_cadence_rpm == 0 && input->motor_erps == 0;
```

W średniobiegowym napędzie **to jest dokładnie obraz zwykłego toczenia się**: wolnobieg
pozwala silnikowi stać przy pełnej prędkości roweru, a przestanie pedałowania zeruje
kadencję. Prędkość koła nie była w ogóle brana pod uwagę.

Skutek: każde ponowne złapanie wspomagania w jeździe uzbrajało kopertę **startu z postoju**.
Przy `duration_ms = 300`:

| Docelowy Iq | Kiedy pojawia się pierwszy niezerowy prąd |
|---:|---|
| 14 (podłoga zatrzasku, domyślne 2%) | ~87 taktów = **~22 ms** |
| 1 | dopiero po pełnych **300 ms** |

Dłuższa konfiguracja wydłuża to proporcjonalnie.

To nie jest dawne oczekiwanie 45–180° na filtr kadencji (FW-091), ale **podważa kryterium
odbioru FW-091**: „kąt potrzebny do ponownego uruchomienia jest stały i wynika z liczby
kroków PAS" było prawdziwe tylko przy **wyłączonym** Smooth Start — a fabrycznie jest on
wyłączony, więc problem dotyczy konfiguracji użytkownika, nie ustawień domyślnych.

## 2. Zmiana

Warunek postoju wymaga teraz, żeby rower **też** stał:

```c
bool stopped = input->measured_cadence_rpm == 0 && input->motor_erps == 0 &&
               !input->bike_rolling;
```

Do modułu trafia **gotowa flaga `bike_rolling`**, a nie prędkość. `assist_start` nie trzyma
własnego progu: pierwsza wersja definiowała `SMOOTH_START_ROLLING_SPEED_X100` obok
`RIDE_START_REDUCTION_MIN_SPEED_X100` i karta twierdziła, że to „jedna definicja" — **było to
nieprawdą**. Dwie stałe o tej samej wartości to kopia, która wygląda na wspólną dokładnie do
chwili, gdy ktoś zmieni jedną. Teraz właścicielem pojęcia „rower się toczy" jest wyłącznie
`ride_control`.

### 2.1. Kasowanie niewykorzystanego uzbrojenia

Sama poprawka warunku nie wystarczyła. `armed` było kasowane **tylko** po dopełnieniu koperty
(`assist_start.c:218`), więc uzbrojenie ze stania przeżywało przejście w toczenie:

```text
1. postój                      -> armed = true
2. rower toczy się bez pedałowania (zjazd, odepchnięcie) -> stopped = false, ale armed nadal true
3. rowerzysta zaczyna pedałować przy 15 km/h             -> stara koperta postoju wciąż tłumi Iq
```

Dlatego niewykorzystana koperta jest anulowana przy toczeniu bez żądania:

```c
if (input->bike_rolling && input->iq_target <= 0) {
    smooth_start_state.armed = false;
    smooth_start_state.elapsed_ticks = 0;
}
```

Warunek `iq_target <= 0` jest istotny: koperta **już trwająca pod realnym żądaniem** jest
prawidłowa i musi się dokończyć, gdy rower nabiera prędkości po starcie z miejsca (test 5c).

Smooth Start dalej robi to, do czego służy — **łagodzi start z postoju**. Przestaje jedynie
traktować jazdy jako startu.

## 3. Testy

`tests/fw092_smooth_start_rolling.js`:

1. Stan sprzed zmiany: toczenie uzbrajało kopertę, a pierwszy prąd przychodził z dużym
   opóźnieniem.
2. Po zmianie toczenie **nie** uzbraja koperty — prąd płynie od pierwszego taktu.
3. Prawdziwy postój nadal dostaje łagodny start.
4. Arytmetyka z audytu jako zabezpieczenie przed regresją: Iq=14 → ~22 ms, Iq=1 → prawie
   pełne 300 ms.
5. Brak toczenia nadal liczy się jako start (ruszanie z prawie-postoju).
5b. **Sekwencja, której brakowało: postój → toczenie bez pedałowania → pedałowanie.**
   Bez kasowania uzbrojenia pierwszy nacisk przy 15 km/h dostawał kopertę startu z postoju.
   Test modeluje **obie połówki naprawy osobno** (`new SmoothStart(fix, disarm)`), więc
   pokazuje, że każda jest konieczna: sam poprawiony warunek `stopped` tego przypadku nie
   ratuje. Test 2 nie mógł tego wykryć — startuje od świeżego obiektu, który nigdy się
   nie uzbroił.
5c. Koperta **już trwająca** pod realnym żądaniem dokańcza się mimo nabrania prędkości.
   To jest uzasadnienie warunku `iq_target <= 0` przy kasowaniu: gdyby go nie było,
   normalny start z miejsca gubiłby swoje łagodzenie w chwili ruszenia roweru.
6. Wyłączony Smooth Start (ustawienie fabryczne) przepuszcza cel bez zmian.
7. Strukturalnie: `bool bike_rolling` w wejściu, `!input->bike_rolling` w warunku postoju,
   **brak** `SMOOTH_START_ROLLING_SPEED_X100` w `assist_start.c`, `ride_control` wyprowadza
   flagę z `RIDE_START_REDUCTION_MIN_SPEED_X100`, a kasowanie uzbrojenia jest obwarowane
   brakiem żądania.

## 4. Kryteria odbioru

- przy **włączonym** Smooth Start ponowne złapanie w jeździe następuje bez dodatkowego
  opóźnienia koperty;
- start z prawdziwego postoju jest nadal łagodzony;
- zachowanie przy wyłączonym Smooth Start bez zmian.

## 5. Pozostaje otwarte (osobna karta)

„Assist without crank rotation" nadal nie działa z rzeczywistego postoju, bo zatrzask
wymaga kroków korby — patrz `FW-091` §8a. To nie jest regresja tych zmian, tylko odrębny
problem funkcjonalny do rozstrzygnięcia: albo własna autoryzacja z limitem 5–7 km/h,
albo usunięcie opcji z UI.
