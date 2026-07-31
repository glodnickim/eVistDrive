# FW-036 - plan diagnozy i naprawy falszywych impulsow speed

- **Data:** 2026-07-26
- **Status:** WDROZONE (minimalny wariant "rise-rate"), build `0.0206`. Czeka na test.
- **Build:** `0.0206_M820_BL820.bin`, SHA-256
  `1AA11E6200E7A38CCA547BDE91E98D3C3C69058063705C854FE4E8F6D768989B`. Bez bledow.

## Wdrozenie - wariant "filtr wzrastania" (wg wlasciciela)

Zamiast progu bezwzglednego 70 km/h jako glowny mechanizm: walidacja oparta na FIZYCZNEJ
NIEMOZLIWOSCI szybkiego WZROSTU. Kolo moze zwolnic w ulamku sekundy, ale nie moze przyspieszyc
o kilkanascie km/h w ulamku sekundy. `Speed_processing` (main.c):
- `ticks==0` -> odrzut (dwa zbocza w jednym ticku, guard /0).
- `instant = wheel_circ*4*360/(ppr*ticks)` (km/h x100).
- `allowed = last_valid + SPEED_MAX_ACCEL_X100_PER_S(2500=25 km/h/s) * ticks/4000`.
- odrzut gdy `instant > SPEED_MAX_INSTANT_X100 (7000=70 km/h)` LUB `instant > allowed`.
- odrzut NIE aktualizuje `Speedx100_cumulated`, `MS.Speedx100`, dystansu ani `Speed_counter`
  (seria glitchy nie utrzyma falszywej predkosci; prawdziwy impuls liczy poprawny czas).
- spadek predkosci przyjmowany bez ograniczen.
- `last_valid_speed_x100` zerowany przy hard-zero po SPEED_STOP_TICKS (pierwszy impuls po
  postoju nie jest odrzucany) i sledzi zanik przy decay.
- `speed_glitch_count` (globalny) liczy odrzuty - na razie wewnetrznie (nie w CAN).

**Izolacja (wymog wlasciciela):** filtr dotyka WYLACZNIE czujnika kola (EXTI2/Speed_processing).
Limit predkosci banku (bank1=0, bank2=20) i inne wartosci config po CAN NIE sa filtrowane.

Odrozone: max-jump osobny, okno po safety_cut, rozdzial speed_display/speed_control, log glitchy
po CAN. Dodac tylko jesli minimalny wariant nie wystarczy.

- **Objaw:** przy mocnym narastaniu wspomagania i szybkim cofaniu korby w chwili
  narastania mocy licznik potrafi pokazac zawyzona predkosc przez ok. 2-3 s,
  a wspomaganie w tym czasie nie chce ruszyc.
- **Zakres:** wejscie czujnika predkosci PB2/EXTI2, `Speed_processing()`,
  diagnostyka CAN, limiter speed w Ride Core/Legacy. Bez zmian w FOC i bez
  zmiany logiki hamulec/reverse cut.

---

## 1. Wnioski z obserwacji uzytkownika

Warunki wywolania:

1. Jazda z mala moca, ale duzym wspolczynnikiem assist.
2. Mocne nacisniecie pedalu powoduje szybki przyrost momentu/pradu.
3. W czasie narastania mocy uzytkownik zaczyna krecic korba do tylu.
4. Dziala cut od cofania/safety.
5. W tym momencie wskazanie predkosci zawyza sie o kilkanascie km/h albo wyglada
   jak wartosc spoza realnej skali.
6. Wspomaganie nie rusza, dopoki predkosc nie opadnie do normalnej wartosci.

Najbardziej prawdopodobna przyczyna:

```text
duzy dI/dt albo twardy cut -> zaklocenie na linii speed PB2
-> falszywe zbocze EXTI2
-> firmware liczy chwilowa predkosc z bardzo malego Speed_counter
-> MS.Speedx100 robi skok
-> speed limiter ucina iq_request
-> po ok. 2-3 s predkosc opada / zeruje sie i assist wraca
```

To moze byc jeden falszywy impuls albo krotka seria impulsow. HMI moze nie
pokazac pelnej wartosci, bo ma wlasne wygladzanie/rampowanie ekranu. Sterownik
moze wewnetrznie zobaczyc np. 80-100 km/h, a na HMI uzytkownik zobaczy tylko
lagodny skok o kilkanascie km/h.

---

## 2. Dlaczego jeden impuls wystarcza

Aktualny tor:

```c
EXTI2_IRQHandler:
    Speed_flag = 1;

Speed_processing:
    Speedx100_cumulated -= Speedx100_cumulated / MP.pulses_per_revolution;
    Speedx100_cumulated += wheel_circumference * 4 * 360 /
                            (MP.pulses_per_revolution * Speed_counter);
    MS.Speedx100 = Speedx100_cumulated / MP.pulses_per_revolution;
    Speed_counter = 0;
```

Problem:

- przerwanie EXTI2 akceptuje kazde zbocze falling jako impuls kola,
- `Speed_processing()` nie sprawdza minimalnego czasu od poprzedniego impulsu,
- nie ma testu fizycznie mozliwej zmiany predkosci,
- przy `pulses_per_revolution=1` jeden falszywy impuls moze ustawic cala predkosc,
- po falszywym impulsie `MS.Speedx100` jest uzyte od razu w limiterze wspomagania.

Dlaczego efekt trwa ok. 2-3 s:

- `SPEED_STOP_TICKS=10600` przy 4 kHz to ok. 2.65 s,
- po impulsie predkosc nie musi zniknac natychmiast,
- kod dopiero po `Speed_counter > 400` zaczyna ograniczac wskazanie przez
  "cisze od impulsu",
- HMI moze dodatkowo rampowac wyswietlana wartosc.

---

## 3. Dlaczego wspomaganie nie dziala

`assist_limits_apply()` uzywa `input->speed_x100` do legalnego limitu predkosci.

Jesli firmware zobaczy zawyzona predkosc:

```text
MS.Speedx100 za wysokie
-> assist_limits_apply() mapuje iq_request do 0
-> silnik nie dostaje pradu wspomagania
-> kiedy Speedx100 spadnie, limiter przestaje ciac
-> wspomaganie wraca
```

Dodatkowo przy niskiej/niepewnej kadencji dziala branch 5-7 km/h:

```c
if cadence_filtered > 15:
    taper przy speed_limit
else:
    taper 5-7 km/h
```

Po backpedal/cut kadencja moze byc chwilowo niska albo gaszona, wiec falszywy
speed powyzej 7 km/h moze wyzerowac assist nawet ponizej glownego limitu 25 km/h.

---

## 4. Wymagana diagnostyka

Developer ma dodac log, zanim zmieni filtr. Minimum:

```text
speed_irq_count
speed_counter_at_irq
speed_counter_at_processing
instant_speed_x100_raw
speed_x100_before
speed_x100_after
speed_pulse_accepted
speed_reject_reason
speed_glitch_count
Speed_counter
MS.Speedx100
cadence
cadence_filtered
backward flag
safety_cut
pwm_cutoff_active
iq_request
iq_setpoint
iq_actual
u_abs / u_q
```

`speed_reject_reason` jako bitfield:

```text
MIN_INTERVAL
MAX_INSTANT_SPEED
MAX_ACCEL
NOISE_WINDOW_AFTER_CUT
PIN_STATE_BAD
```

Log powinien pozwolic odpowiedziec:

- czy falszywy impuls pojawia sie dokladnie przy backpedal/safety cut,
- ile tickow mial `Speed_counter` przy impulsie,
- jaka byla wyliczona predkosc chwilowa,
- czy limiter speed wyzerowal `iq_request`.

---

## 5. Naprawa firmware - warstwy ochrony

### 5.1 Minimalny interwal impulsu

Odrzucac impulsy speed z niemozliwie malym `Speed_counter`.

Parametry compile-time:

```text
SPEED_PULSE_MIN_TICKS
SPEED_MAX_INSTANT_X100
```

Start test:

```text
SPEED_MAX_INSTANT_X100 = 7000   // 70 km/h, duzo powyzej realnego e-bike
SPEED_PULSE_MIN_TICKS wyliczony z wheel circumference i max speed
```

Nie aktualizowac:

- `Speedx100_cumulated`,
- `MS.Speedx100`,
- `distance_since_startup`,
- `Speed_counter`.

Odrzucony impuls nie moze resetowac licznika czasu od ostatniego poprawnego
impulsu, bo wtedy seria glitchy utrzyma falszywa predkosc.

### 5.2 Ograniczenie fizycznego przyspieszenia

Nawet jesli instant speed nie przekracza absolutnego maksimum, sprawdzic skok
wzgledem poprzedniej zaakceptowanej predkosci.

Przyklad:

```text
prev_speed = 14 km/h
instant_speed = 35 km/h po jednym impulsie
-> odrzucic albo ograniczyc do dozwolonego maximum
```

Parametry:

```text
SPEED_MAX_ACCEL_X100_PER_S
SPEED_MAX_JUMP_X100
```

Na poczatek mozna uzyc prostszego progu:

```text
if instant_speed_x100 > MS.Speedx100 + SPEED_MAX_JUMP_X100
    reject
```

ale docelowo lepiej liczyc z czasu od ostatniego poprawnego impulsu.

### 5.3 Noise window po safety cut / backpedal

Poniewaz objaw wystepuje przy twardym cut po cofaniu, dodac krotkie okno
ostrozniejszej walidacji po:

```text
safety_cut rising edge
backward PAS detected
pwm_cutoff_active start
duzy spadek iq_setpoint / iq_actual
```

Przez np. 100-300 ms:

- nie ignorowac wszystkich impulsow,
- ale wymagac bardziej konserwatywnej walidacji,
- odrzucic pojedynczy impuls, ktory oznacza niemozliwy skok predkosci.

### 5.4 Oddzielic speed display od speed control

Rozwazyc dwa sygnaly:

```text
speed_raw_x100       // ostatni poprawny instant/filtered pomiar
speed_control_x100   // walidowany do limiterow
speed_display_x100   // wygladzony do HMI
```

Limiter wspomagania ma uzywac tylko `speed_control_x100`.
HMI moze dostawac `speed_display_x100`, ale nigdy odwrotnie.

Minimalnie: `MS.Speedx100` musi byc walidowane przed uzyciem w limiterze.

---

## 6. Pseudokod minimalnej poprawki

```c
static uint16_t speed_glitch_count;
static uint16_t last_valid_speed_x100;

static bool speed_pulse_valid(uint16_t ticks, uint16_t *instant_x100, uint8_t *reason)
{
    if (ticks < SPEED_PULSE_MIN_TICKS) {
        *reason = SPEED_REJECT_MIN_INTERVAL;
        return false;
    }

    uint32_t v = wheel_circumference * 4 * 360 /
        ((uint32_t)MP.pulses_per_revolution * ticks);

    if (v > SPEED_MAX_INSTANT_X100) {
        *reason = SPEED_REJECT_MAX_INSTANT;
        return false;
    }

    if (last_valid_speed_x100 > 0 &&
        v > (uint32_t)last_valid_speed_x100 + SPEED_MAX_JUMP_X100) {
        *reason = SPEED_REJECT_MAX_JUMP;
        return false;
    }

    *instant_x100 = (uint16_t)v;
    return true;
}

void Speed_processing(void)
{
    uint16_t ticks = Speed_counter;
    uint16_t instant = 0;
    uint8_t reject = 0;

    if (!speed_pulse_valid(ticks, &instant, &reject)) {
        speed_glitch_count++;
        Speed_flag = 0;
        // Do NOT reset Speed_counter and do NOT update distance.
        return;
    }

    Speedx100_cumulated -= Speedx100_cumulated / MP.pulses_per_revolution;
    Speedx100_cumulated += instant;
    MS.Speedx100 = Speedx100_cumulated / MP.pulses_per_revolution;
    last_valid_speed_x100 = MS.Speedx100;
    Speed_counter = 0;
    Speed_flag = 0;
    MS.distance_since_startup += MP.wheel_cirumference /
        (MP.pulses_per_revolution * 1000);
}
```

Uwaga: sprawdzic wzor z obecnym `pulses_per_revolution`, bo aktualny kod dzieli
przez `pulses_per_revolution` zarowno w akumulatorze, jak i przy `MS.Speedx100`.
Nie zmieniac skali przy okazji tej naprawy bez osobnego testu.

---

## 7. Testy

### Test reprodukcji

1. Jazda lub stojak z kolem krecacym sie wolno.
2. Assist z duzym wspolczynnikiem.
3. Mocny nacisk -> rosnacy `iq_request/iq_actual`.
4. Cofniecie korby podczas narastania.
5. Zapis logu.

Oczekiwane przed poprawka:

```text
speed_counter_at_processing bardzo maly
instant_speed_x100 bardzo wysoki
MS.Speedx100 skacze
iq_request spada przez speed limiter
HMI pokazuje lagodniejszy/pozniejszy skok
```

Oczekiwane po poprawce:

```text
glitch_count rosnie
speed_pulse_accepted = 0
MS.Speedx100 bez skoku
iq_request nie jest ciete przez falszywa predkosc
reverse/safety cut nadal dziala
```

### Test normalnej jazdy

- Wolna jazda 3-7 km/h: nie gubic prawdziwych impulsow.
- 10-25 km/h: speed stabilny, bez opoznienia.
- Szybkie przyspieszenie normalna jazda: brak falszywych odrzutow.
- Hamowanie do zera: obecny smooth decay predkosci zostaje.
- Dystans nie nalicza sie od odrzuconych glitchy.

---

## 8. Czego nie robic

- Nie maskowac problemu tylko wiekszym wygladzaniem HMI.
- Nie filtrowac dopiero `MS.Speedx100` po tym, jak falszywy impuls zaktualizowal
  akumulator i dystans.
- Nie resetowac `Speed_counter` po odrzuconym impulsie.
- Nie wylaczac limitera predkosci jako obejscia.
- Nie robic globalnego ignorowania speed po kazdym cut, bo przy prawdziwej jezdzie
  speed nadal ma byc dostepny. Potrzebna jest walidacja, nie slepota.

---

## 9. Najkrotsza lista dla developera

1. Zaloguj `Speed_counter` i `instant_speed_x100` dla kazdego impulsu EXTI2.
2. Potwierdz, czy skok speed wystepuje przy `backward/safety_cut/pwm_cutoff`.
3. Dodaj walidacje impulsu: minimalny interwal + max instant speed + max jump.
4. Odrzucony impuls nie aktualizuje `Speedx100_cumulated`, `MS.Speedx100`,
   dystansu ani `Speed_counter`.
5. Dodaj licznik `speed_glitch_count` i flage powodu odrzutu do diagnostyki.
6. Sprawdz, czy po poprawce limiter speed nie odcina assist od falszywego piku,
   a prawdziwy limit predkosci nadal dziala.
