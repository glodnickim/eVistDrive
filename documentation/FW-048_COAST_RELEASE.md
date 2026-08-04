# FW-048 — koniec kliku przy zatrzymaniu: wybieg zamiast dowożenia prądu

- **Data:** 2026-07-27
- **Status:** WDROZONE W DRZEWIE — **NIE ZBUDOWANE** (czeka na polecenie).
- **Zakres:** `inc/assist_dynamics.h`, `src/assist_dynamics.c`, `src/ride_control.c`.
  Bez zmian w FOC, komutacji, wylaczaniu mostka, hamulcu, cofaniu, Walk Assist.
- **Powiazane:** FW-040, FW-047 (obie okazaly sie nieaktywne — patrz nizej).

---

## Historia diagnozy (wazne, zeby nie wracac do slepych zaulkow)

1. FW-040 (rampa o stalym czasie) i FW-047 (wolny ogon na koncu) **NIE DZIALALY WCALE**,
   bo zapisany bank mial `release_ms = 0`, co omija cala sciezke release
   (`profile_release_active` wymaga `profile_release_ms > 0`). Po ustawieniu **650** rampa
   jest wyczuwalna — **ale klik zostal**. Wniosek: klik nie pochodzi z ksztaltu rampy.
2. Wlasciciel doprecyzowal: szczekniecie jest **dokladnie w chwili zatrzymania**, bez odstepu.
   To wyklucza wylaczanie mostka, ktore nastepuje **sekunde pozniej**.

## Znaleziony mechanizm

`main.c` (~2126-2140) liczy kat wirnika **dwoma roznymi wzorami**:

```c
if (ui16_timertics > (SIXSTEPTHRESHOLD*6)>>2) ui8_6step_flag = 1;   // ~5,5 erps

if(!ui8_6step_flag)
    kat = hall + MP.angle_correction + interpolacja;   // FOC
else
    kat = hall - MP.reverse * deg_30;                  // six-step
```

Te wzory **nie daja tej samej wartosci**, wiec przy przelaczeniu kat zmienia sie **schodkowo**.
Plynacy w tej chwili prad skacze razem z nim -> skok/odwrocenie momentu -> szczekniecie.
Dodatkowo czlon interpolacji **nie jest ograniczony do sektora**: przy zwalnianiu
`ui16_tim2_recent` rosnie, a `uint32_tics_filtered` jest przycinany w dol (~2121-2124), wiec
kat "ucieka" jeszcze przed przelaczeniem.

`SIXSTEPTHRESHOLD = 10000` -> prog ~5,5 erps ~ 4 obr/min zebatki = **chwila zatrzymania**.
Tlumaczy komplet objawow: schodek (nie zbocze) -> rampa nie pomaga; "jakby hamowalo, a na
koniec cos sie zmienilo" -> moment najpierw sie psuje, potem skacze.

## Poprawka (pomysl wlasciciela)

Skoro szkoda powstaje **tylko gdy przez ta strefe plynie prad** — nie podawac go tam.
**Puscic silnik na wybieg** zanim wjedzie w obszar niewiarygodnego kata.

- `assist_dynamics_input_t`: nowe pole `coast_release`.
- `ride_control.c`: ustawiane, gdy `iq_target == 0` (czyli TRWA wygaszanie lub brak zadania)
  **i** `rider->motor_erps < RIDE_COAST_RELEASE_ERPS` (10, z zapasem nad progiem 5,5).
- `assist_dynamics.c`: przy tym warunku `iq_reference_q` idzie do zera od razu, stan rampy
  czyszczony, zwracane 0.

**Kluczowe zabezpieczenie:** warunek `iq_target == 0` sprawia, ze to NIGDY nie zadziala przy
starcie. Ruszanie z postoju odbywa sie w tym samym zakresie predkosci i wymaga pradu —
tam `iq_target > 0`, wiec wybieg sie nie wlacza. To najwazniejsza regresja do sprawdzenia.

Rampa NIE jest usunieta — dalej ksztaltuje zjazd z wysokich wartosci (FW-040/047). Zmienia sie
tylko sam koniec: zamiast dowozic prad do zera przez strefe przelaczenia, konczymy wczesniej.
Prad jest tam juz niski, wiec samo puszczenie nie powinno byc wyczuwalne.

## Odrocone (NIE laczyc w jednym buildzie)

Ograniczenie czlonu interpolacji do jednego sektora, zeby kat nie "uciekal" przy zwalnianiu.
Sensowne samo w sobie, ale dotyka rdzenia komutacji — osobno, zeby dalo sie ocenic, co pomoglo.

## Test

1. **NAJPIERW test rozstrzygajacy:** jazda na **poziomie 0** (zero wspomagania) i normalne
   zatrzymanie.
   - klik IDENTYCZNY -> jest MECHANICZNY (zapadki wolnobiegu / luz przekladni). Firmware go
     nie usunie; dalsze strojenie nie ma sensu.
   - klik NIE wystepuje -> elektryczny, poprawka ma sens.
2. **Stojak:** ruszanie z postoju **bez zmian** — najwazniejsza regresja (prad w strefie
   six-step musi byc dostepny przy starcie).
3. Jazda: zatrzymanie ze wspomaganiem — brak szczekniecia w chwili zatrzymania.
4. Hamulec i cofniecie korby — tna jak dotad, bez opoznienia.
5. Brak przeciagania po zatrzymaniu (regresja FW-028).
6. `release_ms` w zapisanym banku musi zostac **650** — przy 0 cala sciezka release jest
   omijana (to bylo przyczyna, dla ktorej FW-040 i FW-047 nie dzialaly).

---

## Do sprawdzenia: pulsowanie pod dużym obciążeniem i przy małym ERPS

- **Data analizy:** 2026-08-03.
- **Status:** HIPOTEZA DO TESTU I LOGOWANIA — bez zmian w kodzie FOC.
- **Objaw:** podczas technicznego podjazdu, przy bardzo małej kadencji i dużym
  obciążeniu silnik krótko podaje i odpuszcza moment. Rowerzysta nie wiąże tego
  z przekładaniem nogi. Należy rozpatrywać przypadek, w którym końcowe
  `MS.i_q_setpoint` pozostaje stałe.

### Najważniejsze rozróżnienie diagnostyczne

Stałe `MS.i_q_setpoint` nie dowodzi stałego momentu. Nawet stałe logowane
`MS.i_q` nie jest jeszcze rozstrzygające: transformacja Parka wylicza je przy
użyciu tego samego estymowanego kąta `q31_rotorposition_absolute`, którym FOC
steruje wektorem napięcia. Jeśli kąt jest błędny, regulator może widzieć
poprawne `Iq` we własnym, błędnie obróconym układzie współrzędnych, podczas gdy
rzeczywista składowa momentotwórcza względem wirnika maleje albo pulsuje.

Do analizy trzeba rozdzielać:

1. zadanie `MS.i_q_setpoint`;
2. wyliczone `MS.i_q` i `MS.i_d` w estymowanym układzie;
3. rzeczywisty moment, zależny także od błędu kąta wirnika.

### Hipoteza 1 — ekstrapolowany kąt ucieka przed zwalniającym wirnikiem

Pomiędzy zboczami Halla `main.c` wylicza postęp kąta z
`ui16_tim2_recent / (uint32_tics_filtered >> 3)`. Mianownik jest filtrowanym
czasem poprzednich sektorów. Nie ma ograniczenia postępu do jednego sektora
60 stopni.

Przy nagłym zwolnieniu pod dużym obciążeniem bieżący sektor trwa dłużej niż
wynika z historii. Estymator nadal przesuwa kąt według wcześniejszej prędkości
i może wyjść przed faktyczny wirnik. Możliwy przebieg objawu:

1. wirnik zwalnia wewnątrz sektora Halla;
2. estymowany kąt wyprzedza wirnik;
3. rzeczywista składowa momentotwórcza maleje mimo stałego zadanego/logowanego
   `Iq`;
4. kolejne prawidłowe zbocze Halla ponownie kotwiczy kąt;
5. moment wraca jako wyczuwalny krótki impuls.

To jest główny kandydat, jeśli pulsowanie jest zsynchronizowane ze zdarzeniami
Halla, ale `Iq_setpoint` i `BC_limit_flag` pozostają stabilne.

### Hipoteza 2 — przejście interpolacja / stały kąt six-step

Timer Halla pracuje z częstotliwością 500 kHz, a dla M820 obowiązuje:

```text
ERPS = kadencja_rpm * 80 / 60 = kadencja_rpm * 4/3
```

Aktualne progi mają histerezę:

- wejście w stały kąt six-step: `ui16_timertics > 15000`, czyli poniżej około
  5,56 ERPS / 4,17 rpm korby;
- powrót do interpolacji: `ui16_timertics < 10000` oraz świeże zbocze Halla,
  czyli powyżej około 8,33 ERPS / 6,25 rpm korby;
- pomiędzy progami pozostaje poprzedni tryb.

| Kadencja korby | ERPS | Oczekiwany stan |
|---:|---:|---|
| 40 rpm | 53,3 | interpolacja |
| 10 rpm | 13,3 | interpolacja |
| 6 rpm | 8,0 | histereza — zależy od poprzedniego stanu |
| 4 rpm | 5,3 | stały kąt six-step |

W six-step kąt pozostaje stały przez cały sektor elektryczny. Daje to naturalne
tętnienie momentu. Dodatkowo formuła `hall + angle_correction + interpolacja`
nie pokrywa się z `hall - reverse * 30 stopni`; przy zmianie trybu powstaje
skok kąta. Przy domyślnym `reverse = -1` i `angle_correction = 6 stopni`
różnica na początku sektora wynosi około 24 stopni.

Samo stabilne six-step powinno powodować głównie szorstkość/tętnienie. Wyraźne
zanikanie i ponowne pojawianie się mocy sugeruje dodatkowo ucieczkę kąta,
przeskakiwanie między trybami albo błędne/brakujące zbocze Halla.

Ten mechanizm dotyczy przede wszystkim kilku rpm korby. Jeżeli objaw występuje
przy 10–40 rpm, nie wolno przypisywać go samemu progowi six-step; wtedy nadal
trzeba sprawdzić dynamiczny błąd interpolacji, tor prądowy oraz wejściowy tor
PAS/torque.

### Hipoteza 3 — oscylowanie limitera prądu baterii

Po przekroczeniu `MP.battery_current_max` flaga `BC_limit_flag` przełącza ten
sam regulator `PI_iq` z regulacji `MS.i_q` na regulację prądu baterii. Końcowe
`MS.i_q_setpoint` może przez cały czas pozostać stałe.

Możliwy cykl:

```text
Iq rośnie -> prąd baterii przekracza limit -> regulator zmniejsza sterowanie
-> warunek z histerezą zwalnia limiter -> regulator Iq ponownie zwiększa sterowanie
```

Jeśli zaniki momentu pokrywają się ze zmianami bitu `BC_limit_flag`, najpierw
naprawić albo dostroić przejście między obiema regulowanymi wielkościami.
Nie zmieniać równocześnie estymatora kąta.

### Hipotezy drugorzędne

1. **Nasycenie napięcia.** `u_abs` jest ograniczane do `_U_MAX = 1920`.
   Przy stałym zadaniu rzeczywiste `Iq` może przestać nadążać. Przy małym ERPS
   samo nasycenie jest mniej prawdopodobne niż błąd kąta, ale duży prąd fazowy,
   rezystancja uzwojeń i zapadanie napięcia baterii mogą doprowadzić do limitu.
2. **Próbkowanie/rekonstrukcja prądów fazowych.** Sprawdzić zachowanie
   `dyn_adc_state()` przy dużym wypełnieniu PWM. `DYNAMIC_ADC_THRESHOLD` jest
   obecnie równy okresowi `_T`, a warunek przełożenia triggera używa `>`, więc
   należy potwierdzić, czy dynamiczne przesuwanie punktu próbkowania w ogóle
   może się uruchomić. Na tym etapie jest to podejrzenie audytowe, nie
   potwierdzony błąd.
3. **Nieprawidłowa sekwencja Halla.** ISR aktualizuje czas sektora, filtr ERPS
   i poprzedni stan przed potwierdzeniem jednego z 12 prawidłowych przejść.
   Zakłócenie może pozostawić stary kąt przez kolejne zdarzenia. Filtr wejściowy
   timera jest włączony, ale firmware nie liczy obecnie błędnych przejść.
4. **Rzeczywisty stall.** Po około 3 sekundach bez oczekiwanego przejścia Halla
   działa `POWER_STAGE_STOP_TICKS`. To pasuje tylko do wolnego cyklu liczonego
   w sekundach, a nie do szybkiego tętnienia.
5. **Przepełnienie timera przy niemal pełnym zatrzymaniu.** TIMER2 ma okres
   `0xFFFF` przy 500 kHz, czyli około 131 ms na sektor. Poniżej około 1,27 ERPS
   / 0,95 rpm korby czas między zboczami może zawinąć się bez osobnego liczenia
   overflow. Sprawdzić tylko wtedy, gdy wirnik faktycznie niemal stoi.

Twarde zabezpieczenie nadprądowe `MS.i_d > PH_CURRENT_MAX << 2` zatrzymuje kod
do resetu, więc nie pasuje do samoczynnego cyklicznego zaniku i wznowienia mocy.
Przy stałym końcowym `Iq_setpoint` jednorazowy Startup Boost również nie jest
pierwszym podejrzanym.

## Wymagana diagnostyka przyszłego testu

Istniejące ramki pozwalają na pierwszy podział:

- `0x00010203`: końcowe `Iq_setpoint`, kadencja i stan toru żądania;
- `0x00010204`: rzeczywiste `MS.i_q`, `u_abs`, `u_q`, `BC_limit_flag`, PWM,
  cutoff i licznik braku obrotu.

Są wystarczające do wykrycia spadku zadania, limitera baterii albo wyłączania
PWM, ale nie do potwierdzenia problemu kąta. Okres około 40 ms może również
aliasować tętnienie sektorowe.

Do testu FOC dodać snapshot zawierający co najmniej:

- znacznik czasu lub numer próbki;
- `MS.i_q_setpoint`, `MS.i_q`, `MS.i_d`;
- `MS.u_q`, `MS.u_d`, `MS.u_abs`;
- `BC_limit_flag`, `ui_8_PWM_ON_Flag`, `pwm_cutoff_active`;
- `ui8_6step_flag`;
- `ui16_timertics`, `ui16_tim2_recent`, `ui16_erps`;
- surowy `ui8_hall_state`, `ui8_hall_case` i osobny licznik przejść
  niepasujących do 12 prawidłowych przypadków;
- `q31_rotorposition_hall` oraz `q31_rotorposition_absolute` albo ich różnicę.

Nie wysyłać blokującej ramki CAN bezpośrednio z ISR Halla. Bezpieczniejszy jest
krótki bufor kołowy snapshotów zapisywany w ISR i opróżniany w pętli głównej,
ewentualnie zamrażany po wykryciu zdarzenia. Podczas testu kilku rpm pojedynczy
snapshot na każde zbocze Halla powinien pokazać mechanizm bez dużego obciążenia
magistrali.

### Interpretacja przyszłego logu

| Obserwacja | Najbardziej prawdopodobny tor |
|---|---|
| spada `Iq_setpoint` | PAS, torque, limit prędkości albo nadrzędny Ride Core — nie FOC |
| `Iq_setpoint` stałe, `MS.i_q` spada, przełącza się `BC_limit_flag` | limiter prądu baterii |
| `Iq_setpoint` i `MS.i_q` wyglądają stabilnie, moment pulsuje razem z Hallem lub zmianą `6step_flag` | błąd kąta / six-step |
| `u_abs = 1920`, `MS.i_q` nie dochodzi do celu | nasycenie napięcia; następnie sprawdzić próbki prądu |
| błędny `hall_case` poprzedza zanik | sygnał/sekwencja Halla |
| przełącza się PWM/cutoff, odstęp liczony w sekundach | detektor zatrzymanego stopnia mocy |

### Kolejność prac po uzyskaniu logu

1. Najpierw potwierdzić, czy stałe jest końcowe zadanie, rzeczywiste `MS.i_q`,
   oba sygnały, czy tylko odczuwany moment.
2. Rozstrzygnąć korelację z `BC_limit_flag`, nasyceniem oraz PWM.
3. Następnie sprawdzić błąd kąta na każdym zboczu i zmianę `ui8_6step_flag`.
4. Dopiero po rozstrzygnięciu zmieniać jeden mechanizm naraz. Pierwszym
   kandydatem do osobnego eksperymentu jest ograniczenie interpolacji do sektora
   i ciągłe, bezskokowe przejście kąta między interpolacją a trybem niskiej
   prędkości.
5. Osobno ocenić zakres 10–40 rpm. Poprawa six-step poniżej około 6 rpm nie
   rozwiąże objawu występującego wyżej w zakresie kadencji.
