# FW-093 — Iq = 0 ma znaczyć prawdziwy wybieg (Hi-Z), a nie tłumienie silnika

- **Data:** 2026-08-07
- **Status:** WDROŻONE w kodzie, build **0.0299** (poprawka po przeglądzie 0.0298).
  Testy hostowe zielone. **NIEPRZETESTOWANE NA ROWERZE.** Karta wymaga akceptacji przed jazdą.
- **Historia:** 0.0298 miał wdrożoną architekturę, ale kasowanie części całkującej PI
  (§2b) czyniło ją w praktyce nieskuteczną przy prędkości. Naprawione w 0.0299.
- **Cel:** po puszczeniu Walk Assist albo po zakończeniu wspomagania Torque silnik ma
  natychmiast przejść w rzeczywisty swobodny wybieg — bez wyczuwalnego oporu
  elektrycznego i bez twardego dociągnięcia przekładni na końcu.
- **Zakres:** `inc/config.h`, `inc/FOC.h`, `src/FOC.c`, `src/main.c`, komentarze w
  `src/ride_control.c` i `src/assist_dynamics.c`, nowy test
  `tests/fw093_power_stage_coast.js`. **Bez zmian w transporcie CAN konfiguracji, blobie
  banków i w UI Canable** — nie ma tu żadnego nowego ustawienia użytkownika.
- **Powiązane:** `FW-034_035_LEVEL0_BUMPLESS.md` (bumpless enable),
  `FW-040_041_FADE_PRELOAD.md`, `FW-072_SINGLE_RELEASE_RAMP.md`, FW-048 (coast_release).

---

## 1. Prostym językiem

Do tej pory „zero wspomagania" znaczyło tylko tyle, że sterownik **przestaje prosić o
moment**. Mostek MOSFET-ów dalej pracował, a regulator prądu dalej pilnował, żeby prąd
wynosił zero. Pilnowanie prądu na kręcącym się silniku to jest praca — i to ona jest
czuć jako opór, a na samym końcu jako „klik" przekładni.

Mostek wyłączał się dopiero po **3 sekundach bez ruchu wirnika**. Dopóki silnik się
kręcił, ten licznik był bez przerwy zerowany przez czujniki Halla, więc warunek nigdy nie
był spełniony. Innymi słowy: **im dłużej silnik się kręcił, tym dłużej był hamowany.**

Po zmianie: kiedy znika żądanie momentu i rzeczywisty prąd faktycznie opadnie (kilka
milisekund), mostek jest **fizycznie odłączany**. Silnik kręci się dalej własną
bezwładnością, bez żadnego elektrycznego oporu.

---

## 2. Rzeczywista przyczyna (potwierdzona w kodzie, nie założona)

Diagnoza z zamówienia była **w większości trafna**. Co potwierdziłem punkt po punkcie:

| Podejrzenie | Weryfikacja |
|---|---|
| `MS.i_q_setpoint` schodzi do 0, ale FOC dalej działa | **POTWIERDZONE.** `main.c` ADC0_1_IRQHandler: `if(ui_8_PWM_ON_Flag){ FOC_calculation(...) }` — jedynym warunkiem jest flaga PWM, nie zadanie momentu. |
| `_T/2, _T/2, _T/2` to nie jest Hi-Z | **POTWIERDZONE.** To zerowy wektor napięcia — na kręcącym się wirniku zwiera uzwojenia przez mostek, czyli **hamuje**. |
| Wyłączenie mostka jest związane z `uint16_half_rotation_counter` | **POTWIERDZONE.** `main.c` (dawna linia 864/891): `uint16_half_rotation_counter > POWER_STAGE_STOP_TICKS` (12000 taktów @4 kHz = **3 s**). |
| Obsługa Halla resetuje ten licznik podczas obrotu | **POTWIERDZONE.** `TIMER2_IRQHandler`, `case 13:` i `case 23:` — `uint16_half_rotation_counter = 0` co pół obrotu elektrycznego. |
| Regulator przy Iq = 0 tłumi wirnik | **POTWIERDZONE mechanizmem.** `runPIcontrol()` ustawia `PI_iq.setpoint = 0` i `PI_id.setpoint = 0` i reguluje **zmierzony** prąd do zera. Regulator ma ograniczoną szybkość (`max_step = 15` na cykl 16 kHz), więc przy zmieniającej się siły elektromotorycznej napięcie nigdy nie nadąża idealnie — różnica generuje prąd, czyli moment hamujący. |

### Dokładnie: plik, funkcja, warunek, zmienna

- **plik:** `src/main.c`
- **funkcja:** `main()`, blok sterowania stopniem mocy
- **warunek:** `if(uint16_half_rotation_counter > POWER_STAGE_STOP_TICKS && ui_8_PWM_ON_Flag && !pwm_cutoff_active)`
- **zmienna:** `uint16_half_rotation_counter`, zerowana w `TIMER2_IRQHandler` przy każdym
  zdarzeniu Halla

To jest **jedyny** powód, dla którego mostek zostawał aktywny przy `Iq = 0`.

### FW-048 / coast_release — podejrzenie potwierdzone w całości

`src/ride_control.c:459` ustawia `coast_release`, a `src/assist_dynamics.c:112` reaguje
**wyłącznie** tak:

```c
if (input->coast_release && iq_target == 0) {
    iq_reference_q = 0;
    profile_release_step_q = 0;
    return 0;                 /* Iq target = 0 — i tyle */
}
```

Czyli dokładnie jak podejrzewałeś: **coast_release ustawiał tylko zadanie momentu na 0 i
nigdy nie dotykał stopnia mocy.** „Coast" był nazwą bez pokrycia. Po tej karcie ta sama
ścieżka prowadzi do prawdziwego Hi-Z, bo zerowe zadanie momentu **z definicji** je
uruchamia — dla wszystkich źródeł momentu naraz.

## 2b. Druga przyczyna, znaleziona w przeglądzie 0.0298 — kasowanie całki PI

Pierwsza wersja tej karty tłumaczyła opór „ograniczoną szybkością regulatora". **To było
zbyt łagodne.** Przegląd wskazał konflikt, który okazał się drugą, samodzielną przyczyną.

W `reg_ADC_processing()` (i w dwóch innych miejscach) stał reset z FW-028:

```c
if(MS.i_q_setpoint==0){ PI_iq.integral_part=0; PI_id.integral_part=0; }
```

Rachunek, który to przesądza:

- FOC liczy w **16 kHz**, ten reset wykonywał się w **4 kHz** → całka przeżywała
  **4 cykle**.
- `I_FACTOR_I_Q = 0,01` wobec `P_FACTOR_I_Q = 1,5` → człon całkujący mógł urosnąć
  najwyżej do `4 × 0,01 / 1,5` = **2,6 %** członu proporcjonalnego.
- Czyli przy zerowym zadaniu **regulator prądu był praktycznie czysto
  proporcjonalny**.

A żeby utrzymać `i_q = 0` na kręcącym się wirniku, trzeba wytworzyć `u_q = SEM` — czyli
**duże wyjście przy zerowym uchybie**. Regulator P z definicji tego nie potrafi. Ustawia
się w kompromisie znacznie poniżej SEM, a różnica to płynący, **utrzymujący się prąd
hamujący**. To jest ten opór, który czuć — nie „powolne dochodzenie do zera", tylko
regulator strukturalnie niezdolny do wyzerowania prądu.

**Konsekwencja dla samego FW-093 była poważniejsza:** okno `|i_q| ≤ 20` nigdy nie
zostałoby osiągnięte przy prędkości, więc **każde zwolnienie wychodziłoby przez
bezpiecznik 50 ms**, a `coast_u_q_latched` byłby zapisywany przy płynącym prądzie — czyli
seed SEM przy powrocie DRIVE też byłby błędny. Nowa architektura stała na regulatorze,
który nie mógł spełnić jej warunku wejścia.

**Poprawka:** usunięte wszystkie trzy resety zależne wyłącznie od zerowego zadania
(`reg_ADC_processing` ×2 i `legacy_assist_calculate_monolith`, ta ostatnia dotyczy Walk
Assist). Całka pracuje normalnie, dopóki mostek steruje — bo to **ona** pozwala wyzerować
prąd. Gwarancja z FW-028 („zerowe żądanie nie może dalej robić momentu") jest teraz
realizowana mocniej: mostek jest **zwalniany** w kilka ms, a oba regulatory są zerowane
w `power_stage_enter_coast()` — **po** zapisaniu `coast_u_q_latched`. Wyłączony mostek nie
zrobi momentu niezależnie od tego, co trzyma całka. Nadprąd, samowyłączenie i kalibracja
zachowują własne natychmiastowe resety.

Okno ryzyka jest ograniczone: zadanie zerowe + mostek aktywny trwa najwyżej 50 ms.

## 2c. Warunek DRIVE obejmuje obie osie

Było `if(MS.i_q_setpoint)`, jest `if(MS.i_q_setpoint || MS.i_d_setpoint)`. `motor_core`
przenosi **oba** żądania, a prąd osi D to tak samo prąd w uzwojeniach. Dziś o `i_d` prosi
tylko kalibracja pozycji, która i tak sama trzyma mostek, więc **zachowanie się nie
zmienia** — ale przyszłe osłabianie pola nie mogłoby prosić o prąd ze zwolnionego mostka.

### Czego NIE potwierdziłem

- `SOFT_CUTOFF_TICKS` (40 taktów = 10 ms interpolacji do `_T/2`) **nie był przyczyną
  oporu podczas jazdy** — uruchamiał się wyłącznie po tych 3 sekundach bezruchu, czyli
  już na postoju. Ale gdyby zostawić go na nowej, szybkiej ścieżce, byłby 10 ms
  **impulsem hamującym** przy każdym puszczeniu pedałów. Dlatego nowa ścieżka go omija.

---

## 3. Co się zmieniło

### 3.1 Trzy pojęcia rozdzielone (wymóg z zamówienia)

W kodzie są teraz trzy **osobne** rzeczy, opisane w komentarzu blokowym w `main.c`:

| Pojęcie | Nośnik | Znaczenie |
|---|---|---|
| ZERO TORQUE | `MS.i_q_setpoint == 0` | ścieżka sterowania nie prosi o moment |
| COAST / Hi-Z | `pwm_stage`, MOE TIMER0 | półmostki fizycznie zwolnione |
| ROTOR STOPPED | `uint16_half_rotation_counter` | silnik się nie kręci (~3 s bez Halla) |

`uint16_half_rotation_counter` **zachowuje swoją dotychczasową rolę** (wykrywanie
zatrzymanego wirnika i awaryjne odcięcie mostka wpartego w stojący silnik), ale **nie
decyduje już** o przejściu z Iq = 0 do Hi-Z.

### 3.2 Wspólna maszyna stanów

```
                    Iq target != 0
   COAST  ─────────────────────────────►  DRIVE
     ▲                                      │
     │                                      │ Iq target == 0
     │  |i_q| i |i_d| < próg przez 6 ms     ▼
     └──────────────────────────  ZERO_CURRENT_WAIT
             (albo 50 ms bezpiecznika)
```

Jedno miejsce (`power_stage_enter_drive()` / `power_stage_enter_coast()` w `main.c`)
obsługuje **wszystkie** źródła momentu: Torque, Walk Assist, Power / Power Curve,
manetkę i Extended Boost — bo wszystkie kończą w tym samym `MS.i_q_setpoint`.
**Żaden moduł nie wyłącza MOSFET-ów sam.** Walk Assist ma szybszy release (natychmiastowe
`immediate_cut`), ale wychodzi tą samą wspólną ścieżką stopnia mocy.

### 3.3 Nowe żądanie momentu ma priorytet

Gałąź `if(MS.i_q_setpoint)` **zeruje oba liczniki oczekiwania w tym samym takcie**.
Nie ma stanu, w którym `Iq target != 0` i mostek jest wyłączany. Ponadto sekwencja
oczekiwania trwa 6 ms — jeśli żądanie wróci wcześniej, do wybiegu w ogóle nie dochodzi
(naturalna, darmowa histereza przeciw drganiu przy szybkim on/off).

### 3.4 Bezpieczny powrót COAST → DRIVE przy kręcącym się silniku (to była nowa praca)

To jest część, której **wcześniej w firmware nie było**, bo mostek nigdy nie wyłączał się
w ruchu. Bez tego karta wprowadziłaby nowy błąd zamiast usunąć stary:

- Włączenie mostka z `u = 0` na kręcącym się silniku to przyłożenie **zerowego wektora
  napięcia przeciw pełnej sile elektromotorycznej**. Regulator potrzebuje ~8 ms
  (`max_step 15` na cykl 16 kHz przez cały zakres `_U_MAX`), żeby się z tego wygrzebać —
  a to są milisekundy twardego hamowania odzyskowego.
- Dlatego przy wejściu w wybieg zapamiętywane jest `MS.u_q`. Przy `i_q = i_d ≈ 0`
  równanie stojana daje `u_q = SEM`, więc zapamiętana wartość **jest** siłą
  elektromotoryczną przy `coast_erps_latched`. Przy ponownym załączeniu jest skalowana do
  bieżących obrotów (SEM ∝ prędkość) i wstawiana jako punkt startowy regulatora.
- Zabezpieczenia: tylko przy żywym kącie (świeże zdarzenie Halla), tylko przy tym samym
  kierunku obrotu (odwrócony wirnik = odwrócona SEM = seed hamowałby), i nigdy więcej niż
  zmierzona wartość. Cokolwiek niepewnego → stary start od zera, który jest dokładnie
  poprawny na postoju (SEM = 0).
- **MOE włącza teraz ISR FOC**, zaraz po zapisaniu pierwszego wyliczonego zestawu
  `switchtime[]`. Gdyby włączać w pętli głównej, mostek przez maksymalnie jeden okres PWM
  (62,5 µs) podawałby neutralny wektor `_T/2` — czyli zwarcie uzwojeń.
- `get_standstill_position()` (blokujące 25 ms) wykonuje się **tylko przy zatrzymanym
  wirniku**. Gdy silnik się kręci, kąt jest już prowadzony przez ISR Halla i to
  odczytanie tylko dokładałoby martwy czas.
- Istniejący bumpless enable z FW-035 **został zachowany** — zerowanie obu regulatorów,
  `u_q`/`u_d`/`u_abs` i wymuszenie 50/50 są nadal na miejscu i są ścieżką domyślną.

### 3.5 Rampa i stara ścieżka nietknięte

- **Rampa zejścia Iq (FW-040/FW-072) bez zmian.** Moment nadal jest gaszony kontrolowanym
  zejściem prądu — Hi-Z przychodzi dopiero po nim.
- Ścieżka „wirnik stoi 3 s" wraz z `SOFT_CUTOFF_TICKS` **zostaje** jako zabezpieczenie dla
  mostka wpartego w stojący silnik. Tam `_T/2` jest bezpieczne właśnie dlatego, że nic się
  nie kręci. W normalnej jeździe ta ścieżka jest już nieosiągalna — wybieg jest szybszy.

---

## 4. Nowe progi

| Stała | Wartość | Jednostka | Uzasadnienie |
|---|---:|---|---|
| `POWER_STAGE_COAST_CURRENT` | 20 | zliczenia natywne (1 = `CAL_I` = 95 mA fazowo) → **1,9 A** | Wiążącym szumem nie jest rozdzielczość ADC, tylko **zaszyte na sztywno offsety zera** kanałów wtrąconych (2012/2020/2028 w `adc_config`), które mogą chybiać o ~10 zliczeń. 20 to bezpiecznie powyżej tego, a 2,9 % pułapu 700 (66,5 A) to poniżej progu wyczuwalności. Poniżej ~12 warunek mógłby nigdy się nie spełnić i zawsze wchodziłby bezpiecznik czasowy. |
| `POWER_STAGE_COAST_STABLE_TICKS` | 24 | takty 4 kHz = **6 ms** | Zamówione „5–10 ms". Dość długo, żeby pojedyncza zaszumiona próbka nie zwolniła mostka; za krótko, żeby rowerzysta to zauważył. |
| `POWER_STAGE_COAST_MAX_WAIT_TICKS` | 200 | takty 4 kHz = **50 ms** | Twardy sufit. Zamknięte zejście prądu po wyzerowaniu zadania trwa najwyżej ~8 ms, więc to ~6× zapas. Odpala się tylko przy niesprawnym pomiarze prądu — i wtedy **i tak** zwalnia mostek, bo nieczytelny prąd nie może zostawić aktywnego tłumienia. |
| `ROTOR_MOVING_HALL_AGE_TICKS` | 1000 | takty 4 kHz = **250 ms** | Wiek ostatniego zbocza Halla. Poniżej ~0,7 erps kąt uznajemy za nieznany i odtwarzamy z surowego stanu Halla. Dotyczy **wyłącznie** ścieżki załączania — nigdy nie bramkuje wybiegu. |

Wszystkie w `inc/config.h`, jako stałe kompilacji. **Świadomie nie wystawiam ich do
Canable** — to nie są parametry jazdy, tylko własności elektryczne stopnia mocy.

### Próg 20 (1,9 A) jest wartością WSTĘPNĄ — do ustawienia z pomiaru

Nie twierdzę, że 1,9 A fazowego jest mechanicznie nieodczuwalne — przed przekładnią
redukcyjną może być. Ale **nie obniżam go teraz arbitralnie** (np. na 5), bo zaszyte
offsety zera ADC wskazują na szum/błąd rzędu ~10 zliczeń i za niski próg tylko przerzuciłby
każde zwolnienie na bezpiecznik 50 ms.

Kolejność jest taka:

1. Wgraj z diagnostyką, zbierz log ramki **0x00010207**.
2. Odczytaj rzeczywiste `Iq`/`Id` **w chwili wejścia w COAST** (bajty [4..6]) z kilkunastu
   zwolnień, przy różnych prędkościach.
3. Ustaw próg na **zmierzony szczyt szumu + zapas**, a nie na założenie.

Po poprawce z §2b regulator faktycznie potrafi zejść do zera, więc zmierzone wartości
powinny być wyraźnie niższe niż 20 — dopiero one powiedzą, ile naprawdę można zejść.

### Bezpiecznik 50 ms to sytuacja awaryjna, nie tryb pracy

Bit **b4** w ramce 0x00010207 odróżnia wejście w COAST przez bezpiecznik od wejścia przez
zmierzone zero. **Podczas normalnego Torque i WA ten bit ma być zawsze 0.** Jeżeli
zobaczysz choć jedno wejście przez timeout — nie stroić progu „na oko", tylko sprawdzić
regulator prądu i offsety ADC, bo to znaczy, że prąd nie schodzi do zera.

---

## 5. Analiza bezpieczeństwa

| Zagadnienie | Ustalenie |
|---|---|
| Komplementarny PWM / czas martwy | `timer0_config()`: `TIMER_CCX_ENABLE` + `TIMER_CCXN_ENABLE`, `deadtime = 32` @120 MHz = **267 ns**. Bez zmian. |
| `timer_primary_output_config(TIMER0, DISABLE)` | Kasuje bit `POEN` (= MOE) w `TIMER_CCHP`. Przy `runoffstate = TIMER_ROS_STATE_DISABLE` i `ideloffstate = TIMER_IOS_STATE_DISABLE` wyjścia są **zwalniane**, nie wymuszane w stan spoczynkowy — czyli oba tranzystory OFF. |
| Czy to naprawdę Hi-Z na tym sprzęcie | **Tak, i nie jest to nowa ścieżka.** Dokładnie tego wywołania firmware używa już jako „silnik wolny": po zatrzymaniu wirnika, przy samowyłączeniu (`power_off_controller`), przy nadprądzie (`FOC.c`) i w kalibracji pozycji. Gdyby zwierało fazy, kalibracja pozycji nie mogłaby działać. |
| SEM podczas obrotu w Hi-Z | Silnik jest **napędem średniobiegowym z wolnobiegiem po stronie wyjścia** — nie może być rozpędzony ponad to, co sam wytworzył. SEM nie przekracza więc napięcia, które mostek podawał, czyli napięcia baterii. |
| Diody podłożowe / DC-link | Przy wejściu w wybieg prąd jest już **poniżej 1,9 A** (albo, w skrajnym przypadku bezpiecznika, resztkowy), więc nie ma dużego prądu do przełączenia w diody. Zanika ze stałą L/R w ~1 ms. Nie ma warunku przepięcia na DC-link. |
| Nadprąd | `FOC.c` bez zmian: `i_d > PH_CURRENT_MAX<<2` → natychmiastowy DISABLE + `while(1)`. Ścieżka awaryjna **nietknięta**. |
| Hamulec / cofanie / przegrzanie / błąd czujnika | Bez zmian: `safety_cut` w `ride_control` zjeżdża rampą 200 ms (FW-037), a Iq = 0 na jej końcu prowadzi teraz dodatkowo do Hi-Z. Zachowanie robi się **bardziej**, nie mniej bezpieczne. |
| Podnapięcie / przegrzanie | `assist_limits` bez zmian — działają na zadanie momentu. |
| Samowyłączenie i kalibracja pozycji | Zachowują własne, natychmiastowe i bezwarunkowe odcięcia. Dodano tylko czyszczenie `pwm_enable_request`, żeby nic zawieszonego nie mogło ponownie uzbroić MOE. |

**Rozdział zachowany:** ta karta dotyczy wyłącznie **normalnego zwolnienia momentu**.
`FAULT / BRAKE / EMERGENCY SHUTDOWN` nie zostały ruszone.

---

## 6. Diagnostyka

Nowa ramka CAN **0x00010207**, wysyłana **tylko przy zmianie stanu** (nie w pętli 4 kHz),
pod istniejącym przełącznikiem `CAN_DIAGNOSTICS_ENABLE`:

```
[0]    stan: 0 = COAST (Hi-Z), 1 = DRIVE, 2 = ZERO_CURRENT_WAIT
[1]    flagi: b0 PWM/FOC on, b1 wirnik się kręci, b2 Walk Assist,
              b3 hamulec, b4 wejście w COAST przez bezpiecznik czasowy
              (a nie przez zmierzone zero)
[2..3] Iq target  (MS.i_q_setpoint, zliczenia natywne)
[4..5] rzeczywiste Iq (ze znakiem)
[6]    rzeczywiste Id (ze znakiem, ograniczone do ±127)
[7]    ERPS silnika, ograniczone do 255
```

Widać po niej cały cykl `DRIVE → ZERO_CURRENT_WAIT → COAST` i `COAST → DRIVE`.
`debug-logger-cli.js` w repo Canable loguje na razie tylko 0x10203/0x10204 — **jeżeli
chcesz zapis tej ramki do CSV, trzeba go dopisać** (nie robiłem tego bez polecenia,
to inne repo).

---

## 7. Plan testów na rowerze

Wszystkie na **0.0299**. Punkt bazowy do powrotu: **0.0297**.

### Test 0 — log przed dotykaniem progów (zrobić PIERWSZY)
Wgraj wariant z diagnostyką, zbierz ramkę **0x00010207** przy kilkunastu zwolnieniach.
**Oczekiwane:** sekwencja `DRIVE → ZERO_CURRENT_WAIT → COAST`, **b4 = 0 za każdym razem**,
oraz małe `Iq`/`Id` w chwili wejścia w COAST. Dopiero te liczby są podstawą do strojenia
`POWER_STAGE_COAST_CURRENT`.

### Test 1 — Torque
1. Jedź na wspomaganiu Torque, dowolny poziom.
2. Przestań pedałować.
3. **Oczekiwane:** Iq schodzi łagodnie do zera swoją rampą (bez zmian), a zaraz po niej
   silnik przechodzi w swobodny wybieg. Nie ma fazy „jedzie, ale coś przytrzymuje".

### Test 2 — Walk Assist (najlepiej widoczny)
1. Przytrzymaj WA, puść przycisk.
2. Natychmiast spróbuj obrócić silnik **do tyłu** ręką (za korbę / koło).
3. **Oczekiwane:** brak wyraźnego oporu elektromagnetycznego. To jest główne kryterium
   odbioru tej karty.

### Test 3 — końcowe zatrzymanie
1. Pozwól napędowi zejść z kilku ERPS do zera.
2. **Oczekiwane:** klik przekładni zniknął lub wyraźnie zmalał.

### Test 4 — ponowne przyspieszenie w trakcie wybiegu
1. Jedź ~20 km/h, przestań pedałować (mostek → Hi-Z), silnik jeszcze się kręci.
2. Zacznij znów pedałować po ~1 s.
3. **Oczekiwane:** wspomaganie wraca płynnie. **Szukaj szarpnięcia lub sekundowego
   „przyhamowania" w momencie powrotu** — to by znaczyło, że skalowanie SEM jest złe.

### Test 5 — szybkie on/off
1. Kilka razy szybko: pedałuj → puść → pedałuj → puść.
2. **Oczekiwane:** brak szarpnięć, brak błędu 05/07 (kąt), brak nadprądu, brak momentu
   wstecznego.

### Test 6 — regresja startu z postoju
1. Kilka startów spod świateł, w tym pod górkę.
2. **Oczekiwane:** bez zmian względem 0.0297. Ta karta nie miała ruszyć startu, ale
   dokłada do niego ~25 ms (odtworzenie kąta z postoju) — sprawdź, czy tego nie czuć.

Jeżeli coś jest nie tak: zbierz log z ramką **0x00010207** — po niej widać, czy mostek
zwolnił się na pomiarze, czy na bezpieczniku (flaga b4).

---

## 8. Na przyszłość — NIE w tej karcie

**Trzymanie silnika w trybie Walk Assist.** Do rozważenia osobna funkcja: przy WA (a może
i po jego puszczeniu na podjeździe) silnik zamiast całkowicie zwalniać mostek **stawiałby
opór i przytrzymywał rower, żeby nie staczał się do tyłu**. To jest dokładne
przeciwieństwo tego, co robi FW-093, więc musi być świadomym, osobnym trybem z własnym
zadaniem prądu (nie „przypadkiem niewyłączonym mostkiem") i własną kartą — trzeba
przemyśleć nagrzewanie silnika przy zerowych obrotach, warunek wejścia i wyjścia,
oraz co się dzieje, gdy rowerzysta puści rower. **Odłożone, do zrobienia później.**

---

## 9. Kryteria odbioru

- [ ] Test 0 — w logu `DRIVE → ZERO_CURRENT_WAIT → COAST`, **b4 = 0 zawsze**.
- [ ] Test 2 (WA, obrót do tyłu) — brak wyczuwalnego oporu elektrycznego.
- [ ] Test 3 — klik przekładni zniknął lub wyraźnie zmalał.
- [ ] Test 4 — powrót wspomagania w trakcie wybiegu bez szarpnięcia i bez hamowania.
- [ ] Test 5 — brak nadprądu, błędów kąta i momentu wstecznego.
- [ ] Test 6 — start z postoju bez pogorszenia względem 0.0297.
