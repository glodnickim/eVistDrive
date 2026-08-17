# SPEED_SENSOR — koło, `MS.Speedx100`

**PURPOSE** Jeden impuls/obrót koła (GPIOB2/EXTI_2) → prędkość (0.01 km/h) i dystans, z
odrzucaniem fizycznie niemożliwych impulsów (glitch).

**INPUTS** Zbocze na EXTI_2. ISR (`main.c:1716-1723`) zapisuje `speed_edge_tick =
control_time_ticks` (sprzętowy znacznik, patrz `../architecture/TIMEBASES.md`) i ustawia
`Speed_flag`. Pętla główna woła `Speed_processing()` (`main.c:1768-1808`).

**OUTPUTS** `MS.Speedx100`, `MS.distance_since_startup` (dokładna suma mm, nie akumulacja
per-impuls — FW-103 naprawiło ~9% błąd zaokrąglenia).

**STATE** `speed_last_tick`, `last_valid_speed_x100`, `Speedx100_cumulated` (main.c).

**TIMEBASE** WZORCOWO poprawne: okres liczony jako różnica dwóch znaczników
`control_time_ticks` (kategoria A z TIMEBASES.md), odporne na zawinięcie 32-bit i na
pominięte ticki pętli głównej.

**INVARIANTS** Odrzuca impuls jeśli implikowana prędkość > `SPEED_MAX_INSTANT_X100`
(70 km/h) LUB przyrost > `SPEED_MAX_ACCEL_X100_PER_S` (25 km/h/s) względem ostatniej
dobrej wartości — odrzucony impuls NIE przesuwa `speed_last_tick` (FW-036). Wyświetlana
prędkość zanika płynnie po `SPEED_STOP_TICKS` (~2.65 s) ciszy, nie skokiem.

**TEST SEAMS** Logika nie jest dziś wydzielona do osobnego modułu (żyje w `main.c`) —
NIE ma jeszcze testu hosta. Kandydat na przyszłą kartę (audyt, etap E). Ta karta go
nie dotyka.

**RELATED SOURCE FILES** `src/main.c` (`EXTI2_IRQHandler`, `Speed_processing`).

**KNOWN ISSUES** Brak testu — OBSERVABILITY GAP tej karty (nie naprawiony, poza zakresem:
wymagałby wydzielenia funkcji z `main.c`, co karta zabrania).
