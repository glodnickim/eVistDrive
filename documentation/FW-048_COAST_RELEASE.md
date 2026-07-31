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
