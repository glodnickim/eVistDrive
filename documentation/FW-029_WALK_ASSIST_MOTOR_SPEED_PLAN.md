> **ARCHIWALNY:** historyczny plan i zapis wczesniejszych prob. Aktualne
> zachowanie oraz parametry opisuje `FW-060_WA_CONSTANT_RPM_CONTROLLER.md`.

# Karta zmiany FW-029 (plan) - Walk Assist po predkosci silnika / zebatki

- **Data:** 2026-07-25 (plan) / 2026-07-27 (wdrozenie)
- **Status:** WDROZONE, build `0.0209`. Czeka na test NA STOJAKU (procedura 9.1-9.4).
- **Build:** `0.0209_M820_BL820.bin`, SHA-256
  `E875F57742FDA6E3EB8BFE4B11BCCF04DD38BCF0992E218EC270F6AD46538266`. Bez bledow.
  (Uwaga: `0.0208` to identyczny build — skrypt inkrementuje wersje przy kazdym uruchomieniu.)

## Powod pilnego wdrozenia (2026-07-27)

Wlasciciel: "jak zalaczylem tryb WA to malo nie uciekl rower". Przyczyna znaleziona w kodzie:
stary WA mial "start floor" (`main.c`: `if(Speedx100 < WA_START_FULL_SPEED && out < wa_cap) out = wa_cap;`),
ktory ponizej 3 km/h wymuszal PELNY sufit pradu = `WA_START_PCT(100%) * MP.phase_current_max`.
Gdy prad fazowy podniesiono 157 -> 700 (FW-030, prosba dewelopera), szarpniecie startowe WA
wzroslo ~4.5x, mimo ze nikt WA nie stroil. W karcie FW-030 byla o tym notatka ostrzegawcza,
ale nie zostalo to naprawione. Dlatego nowy modul uzywa ABSOLUTNYCH limitow pradu.

## Co wdrozono

- Nowy modul `inc/walk_assist_motor.h` + `src/walk_assist_motor.c`.
- **Regulacja po ERPS silnika** (Hall), nie po `MS.Speedx100`. Predkosc kola zostaje wylacznie
  jako bramka bezpieczenstwa (`>= 700` = 7 km/h wylacza WA).
- **ABSOLUTNE limity pradu** (`WA_MOTOR_IQ_ABS_MAX = 157`), odwiazane od `MP.phase_current_max`.
  Podniesienie prad fazowego nie wzmocni juz WA.
- Maszyna stanow: OFF -> START -> CLOSED_LOOP -> (LIMIT) -> (OVERSPEED) -> (STALL).
- **Watchdog jam/stall na KAZDYM ticku**, przed logika stanow (wymog planu 4.1.1).
- START: rampa 20->39 Iq przez 400 ms, integrator ZAMROZONY na zero; brak "start floor".
- CLOSED_LOOP @200 Hz: FF=6, male P (1/8 Iq na erps), wolny integrator (KI_SHIFT 12),
  deadband 2 erps, anti-windup, rate limit (rise 1 / fall 4), approach cap przed targetem.
- OVERSPEED: `iq=0` (coast, bez hamowania silnikiem), powrot z histereza 2 erps.
- LIMIT/STALL: prad nie rosnie; STALL zatrzaskuje sie do puszczenia przycisku
  (`walk_motor_release()` wolane na tej samej regule co `ui8_WA_blocked`).
- **Naprawiony `ui16_erps_counter`**: byl inkrementowany, ale NIGDY nie resetowany i nigdzie
  nie czytany. Dodany reset w `TIMER2_IRQHandler` po zdarzeniu Halla — bez tego timeout
  predkosci silnika bylby falszywy (plan 3.1).
- **Ramka diagnostyczna `0x00010205`**: state, flags, target_erps, measured_erps, iq_cmd.
- Stary PI po predkosci kola oraz `wa_integral`/`wa_ramp_ticks` usuniete.

Wartosci startowe celowo ZACHOWAWCZE (plan 12, wariant "bardzo zachowawczy"), bo objawem
byl uciekajacy rower: START_IQ_MAX 39 (~25% starej skali), FF 6 (~4%).

**Target:** `WA_MOTOR_TARGET_ERPS_DEFAULT = 25` — stala w module (Etap 1 planu). Wystawienie
do Canable jako osobne pole "Walk motor speed" = Etap 2, jeszcze NIE zrobione.

## NIE zrobione z planu (swiadomie)

- Etap 2: target w Canable/UI (na razie stala firmware).
- Druga ramka diagnostyczna `0x00010206` (error/integral/cap/hall_age) — dodac jesli
  strojenie tego wymaga.
- Kolumny WA w CSV loggerze.

---

- **Intencja wlasciciela:** Walk Assist ma utrzymywac mozliwie stala predkosc
  silnika / zebatki, a nie stala predkosc roweru. Predkosc roweru wlasciciel
  ustawi przelozeniem. Przelozenie ma zmieniac predkosc roweru, ale firmware ma
  pilnowac tej samej predkosci napedu.
- **Najwazniejszy wymog jazdy:** regulator NIE MOZE agresywnie "doganiac"
  predkosci. Jesli silnik zwolni pod obciazeniem, prad ma narastac lagodnie i
  kontrolowanie. Lepszy jest chwilowy niedobor predkosci niz szarpniecie,
  przestrzelenie targetu i ponowne odciecie.
- **Najwazniejszy wymog startu:** poprzednio problemem bylo to, ze sama sciezka
  FOC/current command nie potrafila pewnie ruszyc z postoju. Start WA musi miec
  osobna, kontrolowana faze ruszania oraz twarde wykrywanie zablokowanego kola /
  silnika. Prad nie moze rosnac bez konca, jesli nie ma ruchu.
- **Najwazniejszy wymog ochrony w czasie pracy:** blokada kola/silnika moze
  wystapic takze juz po ruszeniu, w `CLOSED_LOOP`. Ten sam limit ma wtedy
  zadzialac runtime: ograniczyc prad, zatrzymac narastanie integratora i po
  krotkim czasie bez ruchu odciac WA, zeby nie grzac uzwojen/silnika.
- **Watchdog zablokowania dziala przez caly czas aktywnego WA:** sprawdzanie
  jam/stall ma byc wykonywane w kazdym ticku, gdy `MS.pushassist_flag` jest
  aktywny, niezaleznie od stanu `START`, `CLOSED_LOOP`, `LIMIT` czy `OVERSPEED`.
  Logika stanow moze decydowac co zrobic, ale detekcja zagrozenia nie moze byc
  lokalna tylko dla jednego stanu.
- **Zakres:** tylko logika Walk Assist. Nie zmieniac ride core/Legacy pedal assist,
  odciecia cofania, toru PI pradu ani kalibracji Halla.
- **Powiazane:** [[WALK_ASSIST_DZIALANIE]], [[FW-028_PI_WINDUP_FIX]].

---

## 1. Problem w obecnym WA

Obecny Walk Assist w `legacy_assist_calculate_monolith()` w `src/main.c`
reguluje po `MS.Speedx100`, czyli po predkosci roweru z czujnika kola.
W aktualnej galezi `MS.pushassist_flag`:

- target = `MP.walk_assist_speed` w jednostce `0.01 km/h`,
- blad = `MP.walk_assist_speed - MS.Speedx100`,
- wyjscie = `i_q_setpoint_temp`,
- logika zawiera start floor, fade przed celem i overspeed cut.

To jest zla wielkosc regulowana dla intencji wlasciciela. Jesli zmieni sie
przelozenie, predkosc roweru zmienia relacje do predkosci silnika, a regulator
dalej probuje pilnowac kola. Dodatkowo poprzednie podejscia byly zbyt agresywne:
gdy uklad zwalnial, regulator dodawal za duzo momentu, rower natychmiast
przestrzeliwal predkosc, po czym WA odcinal albo pompowal.

Wniosek: nie stroic dalej obecnego PI po kole. Zrobic osobny regulator WA po
predkosci silnika.

## 2. Zasada docelowa

Walk Assist ma byc regulatorem "creep":

1. Ruszyc silnik kontrolowanym, ograniczonym pradem.
2. Po wykryciu stabilnych przejsc Halla przejsc na zamknieta petle predkosci
   silnika.
3. Utrzymywac target z mala agresywnoscia.
4. Pod obciazeniem podnosic moment wolno, przez ograniczony integrator.
5. Przy zblizeniu do targetu ograniczac dostepny moment zanim target zostanie
   osiagniety.
6. Przy przekroczeniu targetu nie hamowac aktywnie, tylko ustawic `i_q=0`,
   zamrozic/wyzerowac czesc integratora i poczekac az predkosc spadnie.

To ma dzialac tak samo na postoju, na luzno obracajacym sie kole i pod obciazeniem.
Na pusto nie moze wyrywac; pod obciazeniem nie moze gasnac po jednym spadku
predkosci.

## 3. Wielkosci i sygnaly

### 3.1 Predkosc regulowana

Uzyc predkosci silnika z Halla:

- `ui16_erps` - juz liczony w przerwaniu TIMER2 CH0 z przejsc Halla,
- `ui16_erps_counter` - istnieje i jest inkrementowany w glownej petli ADC, ale
  w aktualnym kodzie nie widac resetu przy zdarzeniu Halla. Deweloper musi
  dodac reset w przerwaniu TIMER2 CH0 po poprawnym capture:

```c
ui16_erps_counter = 0;
```

Bez tego timeout predkosci silnika bedzie falszywy.

Nie uzywac `MS.Speedx100` do regulacji. `MS.Speedx100` zostaje wylacznie jako
warunek bezpieczenstwa i zgodnosci z WA:

- WA aktywny tylko ponizej ok. 7 km/h (`Speedx100 < 700`),
- opcjonalnie twarde odciecie, jesli rower fizycznie przekroczy limit mimo
  regulacji po silniku.

### 3.2 Target

Etap 1: target jako stala firmware w `inc/config.h`, zeby nie mieszac protokolu
i UI podczas pierwszych testow:

```c
#define WA_MOTOR_TARGET_ERPS_DEFAULT  25
```

Wartosc dobrac testowo. Nie zgadywac docelowej wartosci UI przed jazda.

Etap 2: wystawic target w Canable/UI jako osobne pole "Walk motor speed" albo
"Walk chainring speed". Nie przeciazac po cichu `walk_assist_speed`, bo nazwa i
jednostka tego pola oznaczaja obecnie predkosc roweru.

Jesli potrzebna jest predkosc zebatki zamiast ERPS:

```text
chainring_rpm = motor_erps * 60 / pole_pairs / reduction
```

Do pierwszej implementacji lepiej zostac przy `erps`, bo to jest natywny, gotowy
sygnal Halla.

### 3.3 Pomiar z timeoutem

`ui16_erps` moze trzymac ostatnia wartosc po zatrzymaniu. Dodac lokalny pomiar
WA z timeoutem:

```c
if (ui16_erps_counter > WA_MOTOR_ERPS_TIMEOUT_TICKS) {
    wa_motor_erps_raw = 0;
} else {
    wa_motor_erps_raw = ui16_erps;
}
```

Propozycja:

```c
#define WA_MOTOR_ERPS_TIMEOUT_TICKS  800  // 200 ms @4 kHz
```

Nastepnie lekki filtr tylko dla WA:

```c
wa_motor_erps_filt += (wa_motor_erps_raw - wa_motor_erps_filt) >> WA_MOTOR_ERPS_FILTER_SHIFT;
```

Startowo:

```c
#define WA_MOTOR_ERPS_FILTER_SHIFT  2  // dosc szybki filtr, bez duzego opoznienia
```

## 4. Maszyna stanow

Dodac jawny stan WA. Nie opierac zachowania na samym `pushassist_flag`.

```c
typedef enum {
    WA_STATE_OFF = 0,
    WA_STATE_START,
    WA_STATE_CLOSED_LOOP,
    WA_STATE_LIMIT,
    WA_STATE_OVERSPEED,
    WA_STATE_STALL
} walk_motor_state_t;
```

Zmienne statyczne / globalne:

```c
static walk_motor_state_t wa_motor_state;
static uint16_t wa_motor_ticks;
static int32_t wa_motor_integral;
static int32_t wa_motor_iq_cmd;
static uint16_t wa_motor_erps_filt;
static uint8_t wa_motor_hall_seen;
static uint8_t wa_motor_retry_count;
static uint8_t wa_motor_blocked;
static uint16_t wa_motor_jam_ticks;
```

Reset przy:

- `!MS.pushassist_flag`,
- hamulec,
- blad,
- przekroczenie limitu roweru,
- timeout WA,
- wyjscie ze stanu `STALL`.

### 4.1 OFF

Warunek: WA nieaktywny. Wszystko wyzerowane:

```c
wa_motor_state = WA_STATE_OFF;
wa_motor_integral = 0;
wa_motor_iq_cmd = 0;
wa_motor_ticks = 0;
wa_motor_jam_ticks = 0;
MS.i_q_setpoint_temp = 0;
```

### 4.1.1 Globalny watchdog zablokowania WA

Ten watchdog ma dzialac przez caly czas aktywnego WA, przed szczegolowa logika
stanow. Nie implementowac go tylko w `START` ani tylko w `CLOSED_LOOP`.

W kazdym ticku, gdy `MS.pushassist_flag` jest aktywny:

```c
bool no_or_too_little_motion =
    ui16_erps_counter > WA_MOTOR_JAM_NO_HALL_TICKS ||
    wa_motor_erps_filt < WA_MOTOR_JAM_MIN_ERPS;

bool command_or_current_is_significant =
    wa_motor_iq_cmd >= (MP.phase_current_max * WA_MOTOR_JAM_CMD_PCT / 100) ||
    abs(MS.i_q) >= (MP.phase_current_max * WA_MOTOR_JAM_ACTUAL_PCT / 100);

if (no_or_too_little_motion && command_or_current_is_significant) {
    if (wa_motor_jam_ticks < 65000) wa_motor_jam_ticks++;
} else {
    wa_motor_jam_ticks = 0;
}
```

Jesli `wa_motor_jam_ticks` przekroczy prog, stan ma przejsc do ochrony:

- w `START`: `STALL` albo jedna ograniczona proba retry,
- w `CLOSED_LOOP`: `LIMIT`, potem `STALL` jesli ruch nie wroci,
- w `LIMIT`: kontynuowac odliczanie do `STALL`,
- w `OVERSPEED`: zwykle `iq_cmd=0`, ale jesli mimo to `MS.i_q` jest istotny i
  ruchu nie ma, przejsc do `STALL`.

To jest watchdog bezpieczenstwa, nie czesc regulatora predkosci. Nie moze byc
wylaczany przez deadband, approach band, overspeed logic ani anti-windup.

### 4.2 START

Cel: ruszyc silnik bez szarpniecia i bez od razu pelnego "doganiania".

Wejscie: zbocze `MS.pushassist_flag` z OFF.

Zachowanie:

- `wa_motor_iq_cmd` rosnie powoli od `WA_MOTOR_START_IQ_MIN`,
- nie przekracza `WA_MOTOR_START_IQ_MAX`,
- jesli pojawi sie stabilna predkosc Halla, przejsc do `CLOSED_LOOP`,
- jesli po czasie nie ma ruchu, przejsc do `STALL`; nie dokladac pradu dalej.

Proponowane stale startowe:

```c
#define WA_MOTOR_START_IQ_MIN        30   // do potwierdzenia w jednostkach EBICS
#define WA_MOTOR_START_IQ_MAX_PCT    35   // % MP.phase_current_max
#define WA_MOTOR_START_RAMP_TICKS    1600 // 400 ms @4 kHz
#define WA_MOTOR_START_DETECT_ERPS   3
#define WA_MOTOR_START_DETECT_TICKS  2    // min. 2 ramki/zdarzenia lub kilka probek
#define WA_MOTOR_STALL_TICKS         2800 // 700 ms @4 kHz
```

Wazne: start ma miec sufit pradu, ale nie moze byc stale wymuszona podloga do
wysokiej predkosci. W poprzednim WA taki "floor" byl jedna z przyczyn wyrywania.

Jeszcze wazniejsze: START nie moze polegac na integratorze regulatora predkosci.
W stanie START integrator ma byc wyzerowany i zamrozony. To jest faza
"sprobuj ruszyc kontrolowanym momentem", a nie zamknieta petla predkosci.

Przy wejsciu w START developer powinien upewnic sie, ze wykonywana jest ta sama
inicjalizacja, ktora normalnie dzieje sie przy wlaczeniu PWM z postoju
(`get_standstill_position()` w glownej petli przy `MS.i_q_setpoint != 0`). Jesli
silnik nie rusza mimo pradu, nie obchodzic tego przez podnoszenie cap w nieskonczonosc.
Najpierw log WA ma pokazac:

- czy PWM faktycznie sie wlaczyl,
- czy `MS.i_q` / `iq_actual` rosnie,
- czy pojawily sie zdarzenia Halla,
- czy `ui16_erps_counter` resetuje sie po zdarzeniu Halla.

### 4.2.1 Ochrona startu: zablokowane kolo / brak ruchu

To jest obowiazkowy element. Jesli prad jest podawany, a silnik/zebatka nie
zaczyna sie obracac, WA ma zadzialac jak limiter zabezpieczajacy.

Warunek podejrzenia zablokowania:

```c
bool no_motor_motion =
    wa_motor_erps_filt < WA_MOTOR_JAM_MIN_ERPS &&
    ui16_erps_counter > WA_MOTOR_JAM_NO_HALL_TICKS;

bool high_command =
    wa_motor_iq_cmd >= (MP.phase_current_max * WA_MOTOR_JAM_CMD_PCT / 100);

bool high_actual =
    abs(MS.i_q) >= (MP.phase_current_max * WA_MOTOR_JAM_ACTUAL_PCT / 100);
```

Jesli `no_motor_motion && (high_command || high_actual)` trwa dluzej niz
`WA_MOTOR_JAM_TICKS`, przejsc do `STALL`:

```c
wa_motor_state = WA_STATE_STALL;
wa_motor_iq_cmd = 0;
wa_motor_integral = 0;
wa_motor_blocked = 1;
```

Proponowane stale:

```c
#define WA_MOTOR_JAM_MIN_ERPS          2
#define WA_MOTOR_JAM_NO_HALL_TICKS     400   // 100 ms bez Halla przy probie ruszenia
#define WA_MOTOR_JAM_TICKS             1200  // 300 ms trwalego warunku jam
#define WA_MOTOR_JAM_CMD_PCT           25    // komenda juz istotna, a ruchu brak
#define WA_MOTOR_JAM_ACTUAL_PCT        20    // rzeczywisty prad istotny, a ruchu brak
#define WA_MOTOR_MAX_START_RETRIES     1
```

Zachowanie po `STALL`:

- natychmiast `i_q=0`,
- integrator wyzerowany,
- PWM/FOC przejdzie zwykla sciezka odciecia,
- nie probowac w kolko automatycznie,
- odblokowac dopiero po puszczeniu przycisku / zaniku `walk_can_request`, tak jak
  obecny `ui8_WA_blocked`.

Opcjonalnie dopuscic jedna krotka druga probe (`WA_MOTOR_MAX_START_RETRIES=1`),
ale tylko po cooldown i z mniejszym cap. Jesli druga proba nie ruszy, blokada do
puszczenia przycisku. Nie robic nieskonczonych retry.

### 4.3 CLOSED_LOOP

Docelowa petla po predkosci silnika. Liczyc regulator wolniej niz 4 kHz, np.
co 20 tickow = 200 Hz:

```c
#define WA_MOTOR_CONTROL_DIV 20
```

Na pozostalych tickach tylko podawac ostatnie `wa_motor_iq_cmd` przez normalna
rampe pradu.

Wejscie regulatora:

```c
err = target_erps - wa_motor_erps_filt;
```

Przed normalnym PI sprawdzic ochrone zablokowania w czasie pracy. Jesli silnik
nagle traci predkosc, a `iq_cmd` albo `MS.i_q` sa juz istotne, nie wolno pozwolic
integratorowi "dobijac" pradu. To jest ten sam przypadek co zablokowane kolo w
trakcie prowadzenia roweru.

Wyjscie regulatora:

```c
iq_request = feed_forward + p_term + i_term;
```

Ale z bardzo waznymi ograniczeniami przeciw agresywnemu doganianiu.

## 5. Regulator bez agresywnego doganiania

### 5.1 Feed-forward zamiast samego PI

Dodac maly bazowy prad utrzymania:

```c
feed_forward = MP.phase_current_max * WA_MOTOR_FF_PCT / 100;
```

Startowo:

```c
#define WA_MOTOR_FF_PCT  6
```

PI ma korygowac, a nie sam od zera "wystrzeliwac" momentem.

### 5.2 Male P

P ma byc spokojne. Przy duzym bledzie nie moze od razu otwierac duzego pradu.

```c
p_term = (err * WA_MOTOR_KP_NUM) >> WA_MOTOR_KP_SHIFT;
```

Startowo:

```c
#define WA_MOTOR_KP_NUM    1
#define WA_MOTOR_KP_SHIFT  3   // 1/8 jednostki iq na 1 erps bledu
```

Jesli w testach dalej szarpie: zmniejszyc P, nie zwiekszac I.

### 5.2.1 Runtime jam/torque limit w CLOSED_LOOP

Ochrona zablokowania musi dzialac nie tylko przy starcie. W `CLOSED_LOOP` dodac
stan posredni `WA_STATE_LIMIT` albo rownowazna flage `wa_motor_limit_active`.

Warunek wejscia w limit runtime:

```c
bool motor_too_slow =
    wa_motor_erps_filt < WA_MOTOR_RUN_JAM_ERPS ||
    (target_erps > WA_MOTOR_RUN_JAM_TARGET_MIN_ERPS &&
     wa_motor_erps_filt * 100 < target_erps * WA_MOTOR_RUN_JAM_TARGET_PCT);

bool current_is_heating =
    wa_motor_iq_cmd >= (MP.phase_current_max * WA_MOTOR_RUN_JAM_CMD_PCT / 100) ||
    abs(MS.i_q) >= (MP.phase_current_max * WA_MOTOR_RUN_JAM_ACTUAL_PCT / 100);
```

Jesli `motor_too_slow && current_is_heating` trwa dluzej niz
`WA_MOTOR_RUN_JAM_DEBOUNCE_TICKS`, wejsc w limit:

```c
wa_motor_state = WA_STATE_LIMIT;
wa_motor_integral = 0; // albo szybki bleed; na start zerowac
wa_motor_iq_cmd = min(wa_motor_iq_cmd, wa_safe_limit_iq);
```

W `WA_STATE_LIMIT`:

- nie integrowac,
- nie zwiekszac `iq_cmd`,
- trzymac tylko bezpieczny maly prad przez krotki czas albo od razu zejsc do 0,
- jesli predkosc nie wroci w `WA_MOTOR_RUN_JAM_LIMIT_TICKS`, przejsc do `STALL`,
- jesli predkosc wroci, przejsc do `CLOSED_LOOP` z wyzerowanym integratorem i
  rate-limited wzrostem pradu.

Proponowane stale:

```c
#define WA_MOTOR_RUN_JAM_ERPS              2
#define WA_MOTOR_RUN_JAM_TARGET_MIN_ERPS   8
#define WA_MOTOR_RUN_JAM_TARGET_PCT        25  // <25% targetu przy istotnym pradzie
#define WA_MOTOR_RUN_JAM_CMD_PCT           30
#define WA_MOTOR_RUN_JAM_ACTUAL_PCT        25
#define WA_MOTOR_RUN_JAM_DEBOUNCE_TICKS    800   // 200 ms @4 kHz
#define WA_MOTOR_RUN_JAM_LIMIT_TICKS       1600  // 400 ms w limicie, potem STALL
#define WA_MOTOR_SAFE_LIMIT_PCT            8     // maly prad diagnostyczny/hold
```

Jesli celem jest maksymalna ochrona silnika, `WA_STATE_LIMIT` moze od razu
ustawiac `iq_cmd=0` i po prostu przechodzic do `STALL`. Wersja z malym limitem
jest tylko po to, by odroznic chwilowe przyblokowanie od stalej blokady. Nie
wolno w niej narastac pradem.

Ten limit ma byc traktowany jak zabezpieczenie termiczno-mechaniczne, nie jak
normalny regulator. Jego zadaniem jest ochrona silnika, nie utrzymanie targetu.

### 5.3 Wolny integrator jako trim obciazenia

Integrator sluzy tylko do stalego obciazenia, np. prowadzenie pod gore albo
docisk lancucha. Nie moze nadrabiac chwilowego spadku agresywnie.

```c
i_term = wa_motor_integral >> WA_MOTOR_KI_SHIFT;
```

Startowo:

```c
#define WA_MOTOR_KI_SHIFT  12
```

Integracja tylko gdy:

- WA jest w `CLOSED_LOOP`,
- nie ma overspeed,
- wyjscie nie jest w saturacji w kierunku bledu,
- `abs(err)` jest wieksze niz deadband,
- nie trwa start ani stall.

Przyklad anti-windup:

```c
bool high_sat = iq_unsat >= iq_cap && err > 0;
bool low_sat  = iq_unsat <= 0 && err < 0;
if (!high_sat && !low_sat && abs(err) > WA_MOTOR_DEADBAND_ERPS) {
    wa_motor_integral += err;
}
```

Limit integratora:

```c
integral_max = iq_cap * WA_MOTOR_INTEGRAL_MAX_PCT / 100 << WA_MOTOR_KI_SHIFT;
```

Startowo:

```c
#define WA_MOTOR_INTEGRAL_MAX_PCT  50
```

### 5.4 Deadband przy targetcie

W poblizu targetu nie pompowac pradem:

```c
#define WA_MOTOR_DEADBAND_ERPS  2
```

Jesli `abs(err) <= deadband`:

- nie integrowac,
- mozna powoli upuszczac integrator, np. `integral -= integral >> 6`,
- nie zwiekszac `iq_cmd`.

### 5.5 Limit narastania momentu

To jest kluczowy punkt przeciw problemowi "zwolnil -> dostal za duzo momentu ->
przestrzelil".

Po wyliczeniu `iq_target` zastosowac rate limiter:

```c
if (iq_target > wa_motor_iq_cmd) {
    wa_motor_iq_cmd += min(iq_target - wa_motor_iq_cmd, WA_MOTOR_IQ_RISE_STEP);
} else {
    wa_motor_iq_cmd -= min(wa_motor_iq_cmd - iq_target, WA_MOTOR_IQ_FALL_STEP);
}
```

Startowo:

```c
#define WA_MOTOR_IQ_RISE_STEP  1   // na krok regulatora 200 Hz: wolne doganianie
#define WA_MOTOR_IQ_FALL_STEP  4   // spadek szybszy niz wzrost
```

Uwaga: to jest niezalezne od wspolnej rampy `assist_dynamics`. Wewnatrz WA tez
trzeba ograniczyc narastanie, bo zewnetrzna rampa nie wie, ze to regulator
predkosci i nie zapobiega windup / overshoot logiki.

### 5.6 Wczesne ograniczanie przed targetem

Im blizej targetu, tym mniejszy sufit pradu:

```c
if (erps >= target - WA_MOTOR_APPROACH_BAND_ERPS) {
    iq_cap = map(erps,
                 target - WA_MOTOR_APPROACH_BAND_ERPS,
                 target,
                 WA_MOTOR_IQ_CAP,
                 WA_MOTOR_NEAR_CAP);
}
```

Startowo:

```c
#define WA_MOTOR_APPROACH_BAND_ERPS  8
#define WA_MOTOR_NEAR_CAP_PCT        20  // % normalnego cap przy targetcie
```

To jest odpowiednik "hamowania gazu przed celem". Ma byc zastosowane do
predkosci silnika, nie kola.

### 5.7 Overspeed

Jesli silnik przekroczy target:

```c
if (erps >= target + WA_MOTOR_OVERSPEED_ERPS) {
    iq_target = 0;
    wa_motor_iq_cmd = 0;
    wa_motor_integral = 0; // albo szybki bleed, ale na start zerowac
    wa_motor_state = WA_STATE_OVERSPEED;
}
```

Startowo:

```c
#define WA_MOTOR_OVERSPEED_ERPS  3
```

W `OVERSPEED`:

- nie dawac pradu,
- wyjsc z powrotem do `CLOSED_LOOP` dopiero gdy `erps < target - hysteresis`,
- histereza np. 2 ERPS.

Nie hamowac aktywnie silnikiem. WA ma ciagnac, nie stabilizowac przez regen.

## 6. Limity pradu

Oddzielic limity startu i pracy:

```c
wa_start_cap = MP.phase_current_max * WA_MOTOR_START_IQ_MAX_PCT / 100;
wa_hold_cap  = MP.phase_current_max * MP.walk_assist_current * WA_MOTOR_HOLD_PCT / 10000;
```

Startowo:

```c
#define WA_MOTOR_START_IQ_MAX_PCT  35
#define WA_MOTOR_HOLD_PCT          50
#define WA_MOTOR_MIN_HOLD_IQ       20
```

`MP.walk_assist_current` moze zostac jako "ile sily ma miec WA", ale nie jako
predkosc. Deweloper nie powinien uzywac `MP.walk_assist_speed` jako targetu
silnika bez jawnego mapowania i opisu w UI.

## 7. Miejsce implementacji

Minimalny wariant, bez refaktoru architektury:

1. Warunki aktywacji WA zostaja tam, gdzie sa teraz (`MS.pushassist_flag` w
   `reg_ADC_processing()` / okolice logiki PA4 i `walk_can_request`).
2. W `legacy_assist_calculate_monolith()` w galezi:

   ```c
   else if(MS.pushassist_flag) { ... }
   ```

   zastapic obecny regulator po `MS.Speedx100` wywolaniem nowej funkcji:

   ```c
   MS.i_q_setpoint_temp = walk_motor_assist_update();
   ```

3. Funkcje pomocnicze i stan moga na poczatek zostac w `main.c`, ale lepszy
   docelowy porzadek to nowe pliki:

   - `inc/walk_assist_motor.h`
   - `src/walk_assist_motor.c`

4. Jesli tworzyc nowy modul, przekazac mu tylko potrzebne wejscia:

   ```c
   typedef struct {
       bool active;
       bool brake;
       bool fault;
       uint16_t wheel_speed_x100;
       uint16_t motor_erps;
       uint16_t motor_erps_age_ticks;
       int32_t phase_current_max;
       uint8_t walk_current_pct;
   } walk_motor_input_t;
   ```

   Wyjscie:

   ```c
   typedef struct {
       int32_t iq_target;
       uint8_t state;
       uint16_t target_erps;
       uint16_t measured_erps;
       int16_t error_erps;
       int32_t integral;
       int32_t iq_cap;
   } walk_motor_output_t;
   ```

## 8. Diagnostyka obowiazkowa

Przed testami dodac ramke debug WA. Bez tego developer bedzie zgadywal.

Proponowany CAN ID: `0x00010205` (obok `0x10203` i `0x10204`).

Pola:

| Bajty | Pole | Znaczenie |
|---|---|---|
| 0 | `wa_state` | OFF/START/CLOSED_LOOP/LIMIT/OVERSPEED/STALL |
| 1 | flags | active, hall_valid, saturated, overspeed, stall, jam, blocked, retry, runtime_limit |
| 2-3 | `target_erps` | zadana predkosc silnika |
| 4-5 | `measured_erps` | filtrowana predkosc silnika |
| 6-7 | `iq_cmd` | zadany prad WA po limiterach |

Druga ramka, jesli trzeba wiecej danych (`0x00010206`):

| Bajty | Pole |
|---|---|
| 0-1 | `error_erps` signed |
| 2-3 | `integral >> WA_MOTOR_KI_SHIFT` |
| 4-5 | `iq_cap` |
| 6-7 | `hall_age_ms` |

Jesli diagnozujemy problem startu, dopisac jeszcze w loggerze lub w drugiej ramce:

```text
iq_actual, u_abs, pwm2, jam_ticks, retry_count
```

Bez `iq_actual` nie da sie odroznic "komenda byla, ale prad nie poplynal" od
"prad poplynal, ale silnik byl zablokowany albo nie zlapal pozycji".

CSV logger ma dopisywac te kolumny. Minimum do strojenia:

```text
t_ms, wa_state, target_erps, measured_erps, error_erps,
iq_cmd, iq_actual, u_abs, pwm2, hall_age_ms, wheel_speed_x100,
jam, runtime_limit, blocked, retry_count
```

## 9. Procedura testowa

Testy robic w tej kolejnosci. Nie przeskakiwac od razu do jazdy z obciazeniem.

### 9.1 Kolo w powietrzu

Cel: brak wyrywania.

Oczekiwane:

- START rusza bez szarpniecia,
- `measured_erps` dochodzi do targetu wolno i stabilnie,
- `iq_cmd` nie wskakuje od razu w cap,
- brak oscylacji START/CLOSED_LOOP/OVERSPEED.

Jesli wyrywa:

1. zmniejszyc `WA_MOTOR_START_IQ_MAX_PCT`,
2. zmniejszyc `WA_MOTOR_IQ_RISE_STEP`,
3. zmniejszyc `WA_MOTOR_KP_NUM`,
4. dopiero potem ruszac target.

### 9.2 Lekki opor reka / hamulcem

Cel: silnik nie gasnie, ale nie kontruje agresywnie.

Oczekiwane:

- spadek `measured_erps` powoduje powolny wzrost `iq_cmd`,
- brak skoku do maksymalnego cap,
- po odpuszczeniu oporu predkosc nie przestrzeliwuje mocno targetu,
- integrator nie zostaje wysoki po odpuszczeniu obciazenia.

Jesli przestrzeliwuje po odpuszczeniu:

1. zmniejszyc `WA_MOTOR_INTEGRAL_MAX_PCT`,
2. zwiekszyc `WA_MOTOR_APPROACH_BAND_ERPS`,
3. zmniejszyc `WA_MOTOR_IQ_RISE_STEP`,
4. dodac bleed integratora przy `err <= 0`.

### 9.3 Rower na ziemi, niski bieg

Cel: prowadzenie plynne, bez wyrywania.

Oczekiwane:

- predkosc roweru zalezy od przelozenia,
- predkosc silnika trzyma sie targetu,
- chwilowe przyblokowanie nie daje kopniecia.

### 9.4 Rower na ziemi, wyzszy bieg / wieksze obciazenie

Cel: potwierdzic, ze zmiana przelozenia nie zmienia logiki regulatora.

Oczekiwane:

- `target_erps` ten sam,
- rower jedzie inna predkoscia wynikajaca z przelozenia,
- regulator nie probuje wracac do `MP.walk_assist_speed` kola.

## 10. Kryteria akceptacji

Zmiane uznac za dobra dopiero gdy wszystkie punkty sa spelnione:

- WA utrzymuje predkosc silnika / zebatki, nie predkosc kola.
- Zmiana przelozenia zmienia predkosc roweru bez destabilizacji WA.
- Przy chwilowym spadku predkosci `iq_cmd` narasta lagodnie, nie skokowo.
- Po odpuszczeniu obciazenia nie ma mocnego przestrzelenia targetu.
- Na pusto nie wyrywa.
- Pod umiarkowanym obciazeniem nie staje po chwili.
- Puszczenie przycisku, hamulec, blad i przekroczenie limitu predkosci roweru
  odcinaja WA natychmiast.
- Zablokowane kolo/silnik jest wykryte: jesli `iq_cmd` lub `iq_actual` jest
  istotne, a Halla/predkosci nie ma przez zadany czas, WA przechodzi do `STALL`,
  zeruje prad i blokuje kolejne proby do puszczenia przycisku.
- Zablokowanie kola/silnika w trakcie `CLOSED_LOOP` jest wykryte jako runtime
  limit: przy istotnym pradzie i predkosci silnika ponizej progu regulator nie
  zwieksza pradu, zeruje/zamraza integrator, przechodzi do `LIMIT`, a potem do
  `STALL`, jesli predkosc nie wroci.
- Prad nie rosnie bez konca w START ani w CLOSED_LOOP. Kazdy wzrost jest
  ograniczony capem, rate limiterem i anti-windup.
- Log diagnostyczny pokazuje stany bez zgadywania: target, predkosc, blad,
  integrator, cap, komenda, rzeczywisty prad, jam/stall i retry.

## 11. Rzeczy, ktorych NIE robic

- Nie regulowac WA dalej po `MS.Speedx100` jako glownej zmiennej.
- Nie dawac stalego wysokiego `i_q` jako "prostej" naprawy.
- Nie obchodzic problemu startu przez coraz wiekszy prad. Jesli nie ma Halla /
  predkosci przy istotnym pradzie, ma zadzialac `STALL`, nie wiekszy cap.
- Nie robic wysokiego P ani szybkiego I, zeby "ladnie doganial" na stojaku.
  To jest dokladnie mechanizm, ktory powoduje wyrywanie i przestrzelenie.
- Nie robic nieskonczonych automatycznych ponowien startu przy zablokowanym kole.
  Maksymalnie jedna ograniczona proba, potem blokada do puszczenia przycisku.
- Nie implementowac detekcji zablokowania tylko w START albo tylko w CLOSED_LOOP.
  Watchdog jam/stall ma dzialac przez caly czas aktywnego WA, przed logika
  konkretnego stanu.
- Nie resetowac targetu na podstawie przelozenia. Przelozenie jest mechaniczna
  decyzja wlasciciela.
- Nie przeciazac `MP.walk_assist_speed` nowa jednostka bez opisania tego w UI
  i dokumentacji.
- Nie testowac bez ramki diagnostycznej WA.

## 12. Sugerowane pierwsze wartosci do builda testowego

To sa wartosci startowe, nie prawda objawiona:

```c
#define WA_MOTOR_TARGET_ERPS_DEFAULT      25
#define WA_MOTOR_ERPS_TIMEOUT_TICKS       800
#define WA_MOTOR_ERPS_FILTER_SHIFT        2
#define WA_MOTOR_CONTROL_DIV              20

#define WA_MOTOR_START_IQ_MIN             30
#define WA_MOTOR_START_IQ_MAX_PCT         35
#define WA_MOTOR_START_RAMP_TICKS         1600
#define WA_MOTOR_START_DETECT_ERPS        3
#define WA_MOTOR_STALL_TICKS              2800
#define WA_MOTOR_JAM_MIN_ERPS             2
#define WA_MOTOR_JAM_NO_HALL_TICKS        400
#define WA_MOTOR_JAM_TICKS                1200
#define WA_MOTOR_JAM_CMD_PCT              25
#define WA_MOTOR_JAM_ACTUAL_PCT           20
#define WA_MOTOR_MAX_START_RETRIES        1

#define WA_MOTOR_FF_PCT                   6
#define WA_MOTOR_KP_NUM                   1
#define WA_MOTOR_KP_SHIFT                 3
#define WA_MOTOR_KI_SHIFT                 12
#define WA_MOTOR_DEADBAND_ERPS            2
#define WA_MOTOR_INTEGRAL_MAX_PCT         50
#define WA_MOTOR_IQ_RISE_STEP             1
#define WA_MOTOR_IQ_FALL_STEP             4

#define WA_MOTOR_HOLD_PCT                 50
#define WA_MOTOR_MIN_HOLD_IQ              20
#define WA_MOTOR_APPROACH_BAND_ERPS       8
#define WA_MOTOR_NEAR_CAP_PCT             20
#define WA_MOTOR_OVERSPEED_ERPS           3
#define WA_MOTOR_RUN_JAM_ERPS             2
#define WA_MOTOR_RUN_JAM_TARGET_MIN_ERPS  8
#define WA_MOTOR_RUN_JAM_TARGET_PCT       25
#define WA_MOTOR_RUN_JAM_CMD_PCT          30
#define WA_MOTOR_RUN_JAM_ACTUAL_PCT       25
#define WA_MOTOR_RUN_JAM_DEBOUNCE_TICKS   800
#define WA_MOTOR_RUN_JAM_LIMIT_TICKS      1600
#define WA_MOTOR_SAFE_LIMIT_PCT           8
```

Jesli pierwszy build ma byc bardzo zachowawczy, zmniejszyc:

- `WA_MOTOR_START_IQ_MAX_PCT` 35 -> 25,
- `WA_MOTOR_IQ_RISE_STEP` 1 zostawic,
- `WA_MOTOR_KP_NUM` 1 zostawic,
- `WA_MOTOR_FF_PCT` 6 -> 4.

## 13. Minimalny pseudokod

```c
if (!walk_active || brake || fault || wheel_speed_x100 >= 700) {
    wa_reset();
    return 0;
}

if (wa_motor_blocked) {
    return 0; // release button / CAN request required to clear this latch
}

erps = hall_age_ticks > WA_MOTOR_ERPS_TIMEOUT_TICKS ? 0 : ui16_erps;
wa_motor_erps_filt += (erps - wa_motor_erps_filt) >> WA_MOTOR_ERPS_FILTER_SHIFT;

update_global_jam_watchdog(wa_motor_iq_cmd, MS.i_q,
                           wa_motor_erps_filt, hall_age_ticks);

switch (wa_motor_state) {
case WA_STATE_OFF:
    wa_enter_start();
    break;

case WA_STATE_START:
    wa_motor_iq_cmd = ramp_up_to_start_cap();
    if (wa_motor_jam_ticks > WA_MOTOR_JAM_TICKS) {
        wa_motor_iq_cmd = 0;
        wa_motor_integral = 0;
        if (wa_motor_retry_count < WA_MOTOR_MAX_START_RETRIES) {
            wa_motor_retry_count++;
            enter_limited_retry_start();
        } else {
            wa_motor_blocked = 1;
            wa_motor_state = WA_STATE_STALL;
        }
        break;
    }
    if (wa_motor_erps_filt >= WA_MOTOR_START_DETECT_ERPS) {
        wa_motor_integral = 0;
        wa_motor_state = WA_STATE_CLOSED_LOOP;
    } else if (++wa_motor_ticks > WA_MOTOR_STALL_TICKS) {
        wa_motor_state = WA_STATE_STALL;
        wa_motor_blocked = 1;
    }
    break;

case WA_STATE_CLOSED_LOOP:
    if (control_div_elapsed()) {
        err = target_erps - wa_motor_erps_filt;
        iq_cap = approach_limited_cap(target_erps, wa_motor_erps_filt);

        if (wa_motor_jam_ticks > WA_MOTOR_RUN_JAM_DEBOUNCE_TICKS ||
            runtime_jam_detected(wa_motor_iq_cmd, MS.i_q,
                                 wa_motor_erps_filt, target_erps)) {
            wa_motor_integral = 0;
            wa_motor_iq_cmd = min(wa_motor_iq_cmd,
                                  MP.phase_current_max * WA_MOTOR_SAFE_LIMIT_PCT / 100);
            wa_motor_ticks = 0;
            wa_motor_state = WA_STATE_LIMIT;
            break;
        }

        if (wa_motor_erps_filt >= target_erps + WA_MOTOR_OVERSPEED_ERPS) {
            wa_motor_iq_cmd = 0;
            wa_motor_integral = 0;
            wa_motor_state = WA_STATE_OVERSPEED;
            break;
        }

        p = (err * WA_MOTOR_KP_NUM) >> WA_MOTOR_KP_SHIFT;
        i = wa_motor_integral >> WA_MOTOR_KI_SHIFT;
        iq_unsat = feed_forward + p + i;

        if (can_integrate(iq_unsat, iq_cap, err)) {
            wa_motor_integral += err;
            clamp_integral();
        } else if (err <= 0) {
            bleed_integral();
        }

        iq_target = clamp(feed_forward + p + (wa_motor_integral >> WA_MOTOR_KI_SHIFT),
                          0, iq_cap);
        wa_motor_iq_cmd = rate_limit(wa_motor_iq_cmd, iq_target,
                                     WA_MOTOR_IQ_RISE_STEP,
                                     WA_MOTOR_IQ_FALL_STEP);
    }
    break;

case WA_STATE_LIMIT:
    // Runtime protection: wheel/motor slowed or blocked while current was significant.
    // Do not integrate and do not increase torque in this state.
    wa_motor_iq_cmd = min(wa_motor_iq_cmd,
                          MP.phase_current_max * WA_MOTOR_SAFE_LIMIT_PCT / 100);
    if (wa_motor_erps_filt >= WA_MOTOR_RUN_JAM_ERPS + 2) {
        wa_motor_integral = 0;
        wa_motor_state = WA_STATE_CLOSED_LOOP;
    } else if (++wa_motor_ticks > WA_MOTOR_RUN_JAM_LIMIT_TICKS) {
        wa_motor_iq_cmd = 0;
        wa_motor_integral = 0;
        wa_motor_blocked = 1;
        wa_motor_state = WA_STATE_STALL;
    }
    break;

case WA_STATE_OVERSPEED:
    wa_motor_iq_cmd = 0;
    if (wa_motor_erps_filt + 2 < target_erps) {
        wa_motor_state = WA_STATE_CLOSED_LOOP;
    }
    break;

case WA_STATE_STALL:
    wa_motor_iq_cmd = 0;
    wa_motor_integral = 0;
    // Do not keep pushing forever. Require release/retry or add one limited retry.
    break;
}

return wa_motor_iq_cmd;
```

## 14. Najwazniejszy komentarz dla developera

Nie optymalizowac pod szybkie dojscie do predkosci. Optymalizowac pod brak
szarpniecia i brak przestrzelenia. W Walk Assist uzytkownik idzie obok roweru;
komfort i przewidywalnosc sa wazniejsze niz idealne trzymanie targetu w pierwszej
sekundzie po zmianie obciazenia.

## 15. Doprecyzowanie: NIE robic przelaczenia HALL -> FOC

To jest wazne, bo poprzednie podejscie poleglo na probie wykonania trudnego
punktu "rusz na Hallach, potem przelacz na FOC". Tego NIE wdrazac w FW-029.

W tym firmware FOC jest juz normalna warstwa wykonawcza. Walk Assist nie ma
zmieniac trybu komutacji, nie ma wlaczac osobnego open-loop/sensorless startu i
nie ma robic zadnego handoffu silnika. Halla uzywamy tylko jako zrodla:

- pozycji wirnika, ktora FOC juz wykorzystuje,
- pomiaru predkosci `ui16_erps` do zewnetrznej petli Walk Assist.

Stany `START` i `CLOSED_LOOP` dotycza WYLACZNIE zewnetrznego regulatora WA:

- `START` = zadawaj lagodnie rosnacy `i_q`, az pojawi sie wiarygodny pomiar
  predkosci silnika z Halla,
- `CLOSED_LOOP` = dalej zadawaj `i_q`, ale liczony z regulatora predkosci
  silnika.

W obu stanach prad ma dalej realizowac ta sama istniejaca sciezka FOC/current PI.
Innymi slowy: developer ma zrobic "przelaczenie algorytmu zadawania momentu",
nie "przelaczenie sposobu sterowania silnikiem".
