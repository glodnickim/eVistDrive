# TIMEBASES — trzy różne rzeczy, które wyglądają jak "4 kHz"

**PURPOSE**
Zapobiec najczęstszemu błędowi w tym torze: pomyleniu "raz na tick" z "co 0.25 ms
realnego czasu". Rozróżnienie wymagane przez kartę TEST INFRASTRUCTURE FOUNDATION —
zweryfikowane empirycznie w `tests/host/scenarios/missed_tick_burst_host.c`.

**Trzy kategorie**

**A. ELAPSED TIME (timeout, debounce, episode duration)** — poprawny wzorzec: moduł
przyjmuje jawny `now_tick` (wolnobieżny licznik sprzętowy, `control_time_ticks` w
`main.c`, inkrementowany w `TIMER1_IRQHandler` NIEZALEŻNIE od tego, czy pętla główna
nadąża) i liczy czas przez ODEJMOWANIE od zapisanego zakotwiczenia. Przykład:
`src/ride_episode.c`. Zmierzone: nawet przy 2 wywołaniach zamiast 601, moduł zgłasza
PRAWDZIWY tick zdarzenia — ale zdarzenie MIĘDZY dwoma przetrwałymi wywołaniami może być
zaraportowane dopiero na tick NASTĘPNEGO wywołania, nigdy wcześniej niż naprawdę
wystąpiło. To NIE jest błąd zegara — to jest granica rozdzielczości obserwacji.

**B. CONTROL/DYNAMICS UPDATE (EMA, ramp, regulator)** — wzorzec ryzykowny: moduł liczy
WŁASNE wywołania jako proxy czasu (`static uint16_t ticks; ticks++;` przy każdym wejściu
do funkcji). Poprawny TYLKO jeśli wywołujący gwarantuje dokładnie 1 wywołanie na 1 realny
tick sprzętowy — czego `main.c` NIE gwarantuje pod obciążeniem (patrz `missed_control_ticks`
w `main.c`, dodane właśnie dlatego, że to się zdarza). Przykład zmierzony:
`torque_input.c`'s FAST filter (35 ms) osiągnął wartość 247 po 140 wywołaniach (= 35 ms
przy założonych 4 kHz), ale tylko 86 po 35 wywołaniach — WYRAŹNIE mniej ustabilizowany,
mimo że "nominalnie" upłynęło tyle samo czasu.

**C. LOST PHYSICAL SAMPLE (fizyczna próbka nie do odtworzenia)** — informacja, której
ŻADNA księgowość czasu nie odzyska, bo nikt nie odpytał czujnika w danym oknie. Dekoder
kwadratury PAS w `main.c` (`reg_ADC_processing`, nie jest wydzieloną funkcją) próbkuje
STAN dwóch linii GPIO co tick — jeśli tick jest pominięty, a w tym czasie fizycznie
wystąpiło więcej niż jedno przejście kwadratury, dekoder przy następnym wywołaniu widzi
tylko RÓŻNICĘ stanu modulo 4, nie prawdziwą liczbę kroków. Zmierzone (ilustracyjnie, na
tym samym modelu korby co reszta tej karty, NIE na prawdziwym dekoderze — patrz
`../testing/TEST_INTERFACES.md` "KNOWN ISSUES"): 14 prawdziwych kroków w oknie 400 ticków
przy 90 rpm, a porównanie dwóch migawek fazy dałoby tylko 2 — aliasing, nie błąd
timingu.

**Które liczniki w kodzie są dziś kategorii A (poprawne) vs B (ryzykowne)**

| Poprawne (A) | Ryzykowne (B), niezweryfikowane pod kątem pominiętych ticków |
|---|---|
| `control_time_ticks` (main.c, ISR) | `slow_loop_counter`, `PAS_counter`, `torque_counter` (main.c) |
| `speed_last_tick`/`speed_edge_tick` (main.c, FW-103) | `pas_idle_ticks`, `pas_cycle_ticks` (main.c) |
| `ride_episode.c` (`now_tick` jawny, FW-104) | `tq_fault_ticks`, `soc_tick_counter` (main.c) |
| — | `torque_input.c` FAST filter (`TORQUE_INPUT_TICKS_PER_MS`) |
| — | `assist_extended_boost.c` (`confirm_ticks`, `arm_idle_ticks`, `active_ticks_left`) |
| — | `walk_speed_controller.c` (`session_ticks`), `level_gesture.c` (`match_window`) |

**TIMEBASE** Nominalna baza 4 kHz = `CONTROL_TIMEBASE_HZ` (`inc/config.h`). Filtr RUN w
`torque_input.c` jest WYJĄTKIEM świadomie: liczony w KROKACH KORBY (3.75°/krok), nie w
czasie — patrz `../inputs/TORQUE_SENSOR.md`.

**INVARIANTS** Nowy kod licznikowy powinien domyślnie wybierać wzorzec A. Kategoria B jest
akceptowalna tylko tam, gdzie konsekwencja spowolnienia pod obciążeniem jest jawnie
akceptowana (i najlepiej — zmierzona).

**TEST SEAMS** `tests/host/scenarios/missed_tick_burst_host.c` demonstruje wszystkie trzy
kategorie na prawdziwym kodzie C. Nie naprawia niczego — wyłącznie mierzy.

**RELATED SOURCE FILES** `src/main.c` (TIMER1_IRQHandler, reg_ADC_processing),
`src/ride_episode.c`, `src/torque_input.c`, `src/assist_extended_boost.c`.

**KNOWN ISSUES** `ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md` finding F1 (pełny opis,
sekcja 6 i 21). Ten dokument jest jego żywym, testowalnym potwierdzeniem.
