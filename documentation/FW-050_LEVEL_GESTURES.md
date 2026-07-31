# FW-050 — jeden wspolny mechanizm gestow + naprawa bledow w starym

- **Data:** 2026-07-28
- **Status:** WDROZONE, build `0.0236`. Czeka na test.
- **Build:** `0.0236_M820_BL820.bin`, SHA-256
  `BC312C6DB5DBEE2C4D5AFB9C72C6345F3D5DC102FCEEA7FB448605D37D84F5EF`. Bez bledow.
- **Uwaga:** build zawiera takze **FW-048** (klik przy zatrzymaniu) i **FW-049** (limity
  przeliczane na biezaco) oraz rownoległe zmiany dewelopera w Walk Assist.
- **Zakres:** nowy modul `level_gesture`, `main.c`, `CAN_Display.c`.

---

## Powod

W firmware byly **dwa niezalezne** gesty "pobujaj poziomami": zmiana banku (FW-005) i offroad.
Dla jezdzacego dzialaja identycznie, ale zaimplementowane byly zupelnie inaczej. Wlasciciel:
ujednolicic, bo takich gestow moze przybyc. Przeglad **przed** ujednoliceniem wykazal, ze
wariant offroad zawiera realne bledy.

## Bledy w starym kodzie (naprawione)

1. **Przepelnienie (powazny).** `offroadcode` (uint16_t, max 65 535) liczony jako
   `offroadcode += pow(10, offroadtics) * poziom`. Po udanym przelaczeniu `offroadtics` bylo
   ustawiane na **8/9**, bo sluzylo TEZ jako wartosc na wyswietlaczu — wiec kolejna zmiana
   poziomu liczyla `10^9 x poziom` (~miliard) i zapisywala to do zmiennej 16-bitowej.
   W C to **zachowanie niezdefiniowane**; jesli wynik trafil w 202, offroad przelaczal sie sam.
   Warunek osiagalny normalna obsluga: poprawienie poziomu w ciagu sekundy po gescie.
2. **Przyczyna zrodlowa:** `offroadtics` mial **dwie sprzeczne role** — pozycja cyfry w kodzie
   ORAZ wartosc potwierdzenia na HMI. Rozdzielenie tych dwoch rzeczy usuwa cala klase bledu.
3. **Gest bankow zaklocal licznik offroad** — jego zmiany (~0,6 s) mieszcza sie w oknie offroad
   (0,25-1 s), wiec dopisywaly cyfry.
4. **Potwierdzenie zmiany banku bylo zaslaniane** — `offroadtics` mial priorytet nad
   `bank_splash_kmh`, wiec po przelaczeniu banku widac bylo najpierw ~4 km/h zamiast 10/20.
5. **Predkosciomierz klamal przy zwyklej jezdzie** — dwie zmiany poziomu w ciagu sekundy
   ustawialy `offroadtics=1`, czyli 1 km/h zamiast prawdziwej predkosci przez ~1 s.
6. **`pow()`** — wywolanie biblioteki zmiennoprzecinkowej przy kazdej zmianie poziomu.
7. **Pierwsza zmiana poziomu nie liczyla sie jako cyfra** (licznik przeterminowany), wiec na
   3 cyfry trzeba bylo 4 zmian. Nigdzie nie opisane.

Sprawdzone i **czyste**: poziom z CAN zawsze 0-9 (jawny `switch`), wiec indeksowanie
`level_to_array_element[10]` jest bezpieczne. `bank_save_pending` bylo poprawnie kasowane.

## Rozwiazanie

**Silnik w osobnym module, polityka w `main.c`.** Regula ujednolicona do prostszej, ktora gest
bankow juz stosowal: **dokladna sekwencja poziomow, cala w JEDNYM oknie czasu**. Bez arytmetyki,
bez minimalnego odstepu, bez wspoldzielonego stanu.

- `inc/level_gesture.h` + `src/level_gesture.c` — typ
  `{sequence[], length, window_ticks, splash_kmh, action}` i detektor. Stan dopasowania jest
  **per gest**, a splash ma **wlasna zmienna i wlasny licznik czasu** (to jest naprawa bledu 2,
  a przez to takze 1, 4 i 5).
- `main.c` — tablica gestow i akcje:

| Gest | Sekwencja | Okno | Potwierdzenie |
|---|---|---|---|
| offroad | 2 → 0 → 2 (Eco → 0 → Eco) | ~2,5 s | 9 = wlaczony, 8 = wylaczony |
| rezerwowy 24 | 2 → 4 → 2 → 4 (Eco/Tour) | ~2,5 s | 24; bez akcji |
| rezerwowy 46 | 4 → 6 → 4 → 6 (Tour/Sport) | ~2,5 s | 46; bez akcji |
| rezerwowy 68 | 6 → 8 → 6 → 8 (Sport/Sport+) | ~2,5 s | 68; bez akcji |
| zmiana banku | 8 → 9 → 8 → 9 (Sport+/Boost) | ~2,5 s | 10 = Power, 20 = eMTB |

  Dodanie kolejnego gestu = **jeden wiersz tablicy**.
- `CAN_Display.c` — lancuch priorytetow zastapiony jednym `level_gesture_splash_kmh()`;
  0 = pokaz prawdziwa predkosc.
- Usuniety martwy kod: `offroadcode`, `offroadcounter` i ich obsluga.

### Czego NIE ruszono (swiadomie)

- **`MP.MagicNumber` zostaje w strukturze**, mimo ze nie jest juz uzywany. Pole lezy w
  `MotorParams_t`, czyli w **zapisywanym rekordzie EEPROM** — usuniecie zmienicby uklad rekordu
  i wywolalo jednorazowy reset wszystkich ustawien (FW-023). Zostaje jako rezerwa.
- `MS.offroadtics` / `MS.bank_splash_kmh` zostaja w `MotorState_t` (stan ulotny), tylko nikt
  ich juz nie uzywa.

## Test

1. **Gest bankow:** 8→9→8→9 w ~2,5 s → bank sie przelacza, wyswietlacz pokazuje **od razu
   10 lub 20** (wczesniej najpierw ~4). Po postoju zapis do pamieci.
2. **Gest offroad:** Eco → 0 → Eco w ~2,5 s → limit predkosci przestaje obowiazywac,
   potwierdzenie 9. Powtorzenie przywraca limit (8). Wylaczenie roweru przywraca limit.
3. **Regresja na blad 1:** zaraz po udanym gescie offroad kilka razy zmienic poziom w ciagu
   sekundy — **nic nie moze przelaczyc sie samo**.
4. **Regresja na blad 5:** dwie szybkie zmiany poziomu przy normalnej jezdzie —
   predkosciomierz pokazuje **prawdziwa predkosc**, nie 1 km/h.
5. Gesty nie koliduja: gest bankow nie wlacza offroad i odwrotnie.
6. Przy wylaczonym offroad limit predkosci dziala jak w FW-049.
