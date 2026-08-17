# BATTERY — napięcie, prąd, SOC/coulomb counting

**PURPOSE** Filtracja napięcia/prądu baterii i estymacja SOC (coulomb counting + korekta
OCV), używane przez `assist_limits` (napięcie) i limiter prądu baterii w `runPIcontrol()`.

**INPUTS** `adc_value[0]` (prąd, offset kalibrowany na starcie), `adc_value[3]` (napięcie).

**OUTPUTS** `MS.Battery_Current`, `MS.Voltage`, `MS.soc_real`/`soc_display`,
`MS.remaining_mah`, `MS.avg_wh_per_km`.

**STATE** ~15 zmiennych w `main.c` (`soc_mAs_acc`, `battery_current_cumulated`,
`voltage_raw_cumulated`, itd.) plus flash-persystowany stan (`soc_slot_t`).

**TIMEBASE** Filtr V/I: IIR `>>6` co tick. Coulomb counting: `soc_mAs_acc +=
Battery_Current/4000.0f` co tick — zakłada DOKŁADNIE 4000 wywołań/s (kategoria B,
`../architecture/TIMEBASES.md`); jeśli wywołanie jest CAŁKOWICIE pominięte (nie tylko
spóźnione), próbka prądu z tego okresu nigdy nie trafia do całki — to jest utrata próbki,
nie tylko przesunięcie w czasie. SOC update logiczny 1 Hz (`soc_one_second_flag`).

**INVARIANTS** `soc_full_pack_10mv` (jeśli skonfigurowane) kotwiczy 100% przy starcie po
`SOC_FULL_BOOT_SETTLE_S` sekund stabilnego napięcia. Zapis flash tylko przy
`|ΔSOC|≥SOC_SAVE_DELTA` i nie częściej niż `SAVE_MIN_INTERVAL_S`.

**TEST SEAMS** Brak — logika nie jest wydzielona z `main.c`. Matematyka (coulomb, OCV
correction, limp factor) jest czysto arytmetyczna i nadaje się do wydzielenia (audyt,
etap F), ale ta karta tego nie robi.

**RELATED SOURCE FILES** `src/main.c` (`soc_*`, `calculate_SOC`, `compute_limp_factor`).

**KNOWN ISSUES** Audyt finding F11 (utrata próbki coulomb pod pominiętymi tickami,
niezweryfikowana empirycznie w tej karcie — poza zakresem, wymagałaby wydzielenia modułu).
