# FW-034 (poziom 0 = OFF) + FW-035 (bumpless start) + FW-037 (rampa odciecia zamiast twardego cutu)

- **Data:** 2026-07-26
- **Status:** WDROZONE, build `0.0205`. Bazuje na 0.0204 (znany-dobry, [[project-0203-latch-canable]]). Czeka na test.
- **Build:** `0.0205_M820_BL820.bin`, SHA-256
  `D63D5E5205D11EBCA9640B832BE287EA834A614A45CF916E0B2A9B94277F1DBF`. Bez bledow.

## FW-037 - rampa wylaczania zamiast twardego cutu (klik przy hamulcu/cofaniu)

Wlasciciel: rampa PWM-off ma byc uzywana dla WSZYSTKICH odciec; twardo tylko blad silnika.
Weryfikacja: przetezenie (jedyny prawdziwy blad silnika) juz twardo wylacza mostek w FOC.c
(overcurrent). Reszta (hamulec, cofanie, przegrzanie, blad czujnika momentu) robila twardy cut
w main.c -> klik przekladni.

Zmiany (4 spojne):
- `ride_control.c`: na `safety_cut` wymuszone `iq_target=0`, `profile_pedaling_active=false`,
  `profile_release_ms=RIDE_SAFETY_RELEASE_MS (200)`, rozbrojenie latcha -> assist gasnie RAMPA.
- `assist_dynamics.c`: usuniete dwa "snap Iq do 0" na `safety_cut` (obie galezie) -> rampa dziala.
- `main.c`: reset integralu tylko przy `i_q_setpoint==0` (nie na safety_cut) -> gladziej.
- `main.c`: usuniety twardy blok "safety_cut -> neutralny PWM + DISABLE mostka". Zostaje soft
  cutoff po zatrzymaniu wirnika. Overcurrent (FOC.c) dalej twardo.

Sekwencja STOP (wszystkie odciecia oprocz przetezenia): Iq->0 (rampa) -> PWM neutralny (soft
cutoff 10 ms) -> DISABLE mostka. Przetezenie: natychmiast DISABLE (FOC.c).

---

- **Zakres:** ride_control (latch) + main.c (rozruch mostka FOC). Bez zmian w rampie Iq,
  limitach, hamulcu, reverse cut, Walk Assist.

---

## FW-034 - poziom wspomagania 0 wylacza wspomaganie calkowicie

**Blad:** na poziomie 0 rower dalej wspomagal. Przyczyna: latch FW-031 zbrajal sie i
nakladal podloge pradu (`assist_min_iq_pct`) NIEZALEZNIE od poziomu. `assist_modes`
zwracal iq=0 dla poziomu 0 (idle level), ale latch dokladal podloge.

**Poprawka** (`ride_control.c`): `assist_off = (assist_level_index == 0)`. Przy poziomie 0
latch NIE zbraja sie i NIE naklada podlogi (dodane do warunku rozbrojenia i do warunku
zbrojenia). iq_target zostaje 0. Manetka (jesli uzywana) dalej dziala niezaleznie od poziomu.

## FW-035 - bumpless start mostka (na podstawie analizy dewelopera)

**Weryfikacja notatki wzgledem realnego kodu 0.0204:**
- Twierdzenie "stop pedalowania = twardy safety_cut -> klik" jest NIEAKTUALNE. W kodzie
  `safety_cut` = tylko hamulec/cofanie/przegrzanie/blad momentu/kalibracja
  (`main.c:1753`); stop pedalowania idzie miekka rampa (iq_target=0 + release_ms). Bez zmian.
- Twierdzenie "PI.out nie jest zerowane przy starcie" jest PRAWDA. `PI_control` slewuje
  `PI.out` po `max_step` (`FOC.c:175-177`), wiec `.out` to STAN. FW-028 zerowal tylko
  `integral_part`, nie `.out`. Przy ponownym ENABLE mostek startowal od starego napiecia
  -> mozliwy klik/kop przekladni.

**Poprawka** (`main.c`, w miejscu wlaczenia mostka przy `i_q_setpoint>0` i `!ui_8_PWM_ON_Flag`):
przed `timer_primary_output_config(TIMER0,ENABLE)` ustawiane jest czyste, neutralne wejscie:
```c
PI_iq.integral_part=0; PI_iq.out=0;
PI_id.integral_part=0; PI_id.out=0;
MS.u_q=0; MS.u_d=0; MS.u_abs=0;
switchtime[0..2]=_T>>1;                 // 50/50 PWM = brak napiecia miedzyfazowego
timer_channel_output_pulse_value_config(...neutral...);
```
Potem ENABLE, a FOC narasta przez istniejaca rampe Iq (300-600 ms). To jest prawdziwy
bumpless start bez kopiowania rampy PWM (rampa Iq juz istnieje w assist_dynamics).

**Nie zrobione (celowo, wg rekomendacji "najpierw to"):** osobna krotka rampa amplitudy
PWM 5-20 ms i wymuszony 1-2 ms neutralnego PWM po ENABLE. Zerowanie .out + neutralny PWM
przed ENABLE juz daje neutralny pierwszy cykl (out=0). Dodac tylko jesli klik pozostanie.

## Test

1. **Poziom 0:** pedalowanie z naciskiem NIE daje wspomagania (calkowicie off). Poziomy 1-5 OK.
2. **Start:** ruszanie gladkie, bez "kopa"/kliku na poczatku; reakcja dalej ostra (rampa Iq + torque_run).
3. Walk Assist, kalibracja Halla, hamulec/cofanie (twardy cut) - bez regresji.
4. eMTB/Power - pulsowanie dalej rozwiazane (torque_run 300).
