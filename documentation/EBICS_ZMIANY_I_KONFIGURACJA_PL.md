# EBICS — co zrobiliśmy, dlaczego i jak to działa

Dokument dla **każdego** (użytkownik, deweloper, ktoś nowy). Opisuje całą pracę na gałęzi
`feat/incremental-from-be40f75`: kontekst, co zbadano, co zmieniono, co ustawiasz sam, co zostało.
Żywy — uzupełniany przy każdej zmianie. Na końcu = podstawa do release.

---

## 0. W jednym akapicie (TL;DR)
Firmware EBICS steruje silnikiem roweru elektrycznego (sterownik Bafang M820, wyświetlacz „HMI").
Wersja robocza **0.0114 zepsuła jazdę** (silnik nie ruszał). **Cofnęliśmy** kod do ostatniej
działającej bazy **be40f75 (= 0.0115)** — jeździ. Potem **małymi, testowanymi krokami** poprawiamy:
(1) wyłączyliśmy „przeciąganie" mocy po zaprzestaniu pedałowania, (2) wyłączyliśmy zbędną
telemetrię deweloperską zalewającą magistralę CAN. Trwa praca nad **płynnością jazdy**.
Temat „info firmware/prędkość max na ekranie HMI" **zaparkowany** — okazało się, że blokada jest
wewnątrz wyświetlacza, niewidoczna z magistrali.

---

## 1. Słowniczek (żeby każdy rozumiał)
- **Sterownik** — komputer w silniku (nasz firmware EBICS na nim działa).
- **HMI / wyświetlacz** — panel na kierownicy (pokazuje prędkość, baterię, menu). Gada ze sterownikiem po **CAN**.
- **CAN** — magistrala (kabel), po której lecą „ramki" (paczki danych) między HMI a sterownikiem.
- **Fake taxi** — oryginalny, fabryczny firmware Bafang. Używamy jego logów jako **wzorca** „jak ma być".
- **Walk Assist (WA)** — tryb prowadzenia roweru (trzymasz przycisk, rower jedzie ~6 km/h).
- **Telemetria** — dane, które sterownik wysyła na HMI (prędkość, bateria, moment…).
- **Override / Extended Boost / „przeciąganie"** — funkcja trzymająca moc silnika chwilę PO tym,
  jak przestaniesz naciskać pedał. Powodowała, że moc nie schodziła gładko.
- **i_q** — zadany prąd silnika (im większy, tym większa moc). „Rampa i_q" = jak szybko ten prąd rośnie/maleje.

---

## 2. Kontekst — co się działo (historia)
1. **0.0114 zepsute** — po wgraniu silnik i Walk Assist w ogóle nie ruszały; na HMI „tylko CR X30P".
   Przyczyna: duży, niezacommitowany refaktor ścieżki silnika (m.in. martwa pętla sterowania).
2. **Cofnięcie do be40f75 (0.0115)** — potwierdzona działająca baza (silnik + WA ruszają).
   Zepsute 0.0114 zachowane na gałęzi `wip-0.0114-broken` (na wypadek potrzeby).
3. **Praca krokami na nowej gałęzi** `feat/incremental-from-be40f75` — każda zmiana osobno,
   build → test na rowerze → dopiero następna. Zmiany „bezpieczne" (nie dotykają silnika)
   scalane w paczki; zmiany silnika — pojedynczo.

---

## 3. Flagi konfiguracyjne — co ustawiasz sam (`inc/config.h`)
Zmieniasz wartość → przebudowa (`build_firmware.ps1`) → wgranie. Domyślne wartości są bezpieczne.

| Flaga | Domyślnie | Co robi (po ludzku) | Kiedy zmienić |
|---|---|---|---|
| `EXTENDED_BOOST_ENABLE` | **0 (off)** | „Przeciąganie": trzymanie mocy PO puszczeniu pedału. Off = moc schodzi płynnie za pedałem (jak Bosch). | `1` jeśli chcesz starego „dociągania". |
| `SEND_DEV_TELEMETRY` | **0 (off)** | Wysyłanie 2 deweloperskich ramek (`0x81F83100` moment/kadencja co 10 ms, `0x80010203` debug FOC). Fabryka ich nie wysyła. | `1` tylko gdy deweloper stroi silnik i chce te dane. |
| `IQ_RAMP_ADAPTIVE` | **1 (on)** | Tempo narastania/opadania prądu **zależne od prędkości i kadencji** (miękko na wolno, żwawo przy prędkości). Gdy `0` — stałe `IQ_SLEW_UP/DOWN`. | `0` by wrócić do stałego tempa. Progi: `IQ_SLEW_UP/DOWN_SLOW/FAST`. |
| `SMOOTH_START_ENABLE` | **0 (off)** | Miękkie ruszanie: tłumi wspomaganie 0→100% przez `START_RAMP_TICKS` po postoju. | `1` jeśli ruszanie nadal zbyt „kopie" (rampa adaptacyjna już to łagodzi). |
| `TQ_FULL_SCALE_MV` | `3300` | Górna granica mapy „nacisk → moc". 3300 = jak dziś (nacisk słabo się przekłada). **Niżej (~1800–2200) = bardziej naciskowe/przewidywalne (Bosch)**. | Obniż by mocniej czuć nacisk pedału. |
| `TQ_GATE_MIN` | `15` | Próg momentu, poniżej którego brak wspomagania kadencyjnego. Blokuje „wzbudzanie przód-tył" bez nacisku i porządkuje załączanie. | Wyżej = trzeba mocniej nacisnąć by ruszyć (spokojniej); za wysoko = lekkie pedałowanie nie wspomaga. |
| `ASSIST_TORQUE_MODE` | **0 (off)** | Charakter wspomagania: `0`=kadencyjny (jak dziś), **`1`=naciskowy Bosch** (moc ∝ nacisk, kadencja tylko jako „pedałujesz"). Naprawia B/C/D u źródła. | `1` do wypróbowania — **wtedy obniż `TQ_FULL_SCALE_MV`** (~1800–2200), inaczej wspomaganie za słabe. |
| `IQ_SLEW_UP` / `IQ_SLEW_DOWN` | `5` / `10` | Stałe tempo (używane gdy `IQ_RAMP_ADAPTIVE=0`). | — |

---

## 4. Co zmieniliśmy (changelog — od najnowszego)

### 0.0124 — Strojenie wg feedbacku z jazdy (0.0123) + tryb naciskowy
Feedback: narastanie za wolne, moc odcina zamiast opadać, wzbudzanie „przód-tył" bez nacisku, nieregularne załączanie.
- **A** narastanie za wolne → `IQ_SLEW_UP_SLOW 3→6`, `FAST 7→12`.
- **B** odcięcie zamiast opadania → `IQ_SLEW_DOWN_SLOW 6→4`, `FAST 12→8` (łagodniej).
- **D** wzbudzanie bez nacisku → **`TQ_GATE_MIN=15`** (człon kadencyjny tylko przy realnym momencie).
- **C** nieregularne załączanie → bramka momentu daje spójny próg.
- **KROK 2 opcja:** `ASSIST_TORQUE_MODE` (flaga, off) — tryb czysto naciskowy Bosch. **Commit `7e74c10`.**
- **Test:** jak niżej — sprawdź narastanie (szybsze), opadanie (łagodne, nie cięte), i czy „przód-tył" już NIE wzbudza silnika.

### 0.0123 — Paczka jakości jazdy (3 gałki, każda flagą)
- **#1 Adaptacyjna rampa i_q** (`IQ_RAMP_ADAPTIVE=1`, aktywna): tempo zmiany prądu zależne od
  prędkości i kadencji → miękkie ruszanie, płynne przejścia w jeździe, gładkie schodzenie.
- **#2 Smooth-start** (`SMOOTH_START_ENABLE=0`, uśpiona): miękkie tłumienie startu — włącz jeśli trzeba.
- **#4 `TQ_FULL_SCALE_MV=3300`** (gałka, domyślnie bez zmian): obniż → bardziej naciskowe czucie.
- Domyślnie zmienia odczucie tylko **#1**; #2/#4 to uśpione gałki do strojenia. **Commit `76609bf`.**
- **Test:** patrz sekcja 4a poniżej.

#### 4a. Jak testować 0.0123
1. **Przejścia w jeździe** (główny cel #1): zwalniaj/dodawaj nacisk → moc płynie gładko, bez skoków.
2. **Ruszanie:** miękkie, bez kopa? (jak za miękko/ospale — zmniejsz progi lub włącz #2).
3. **Nacisk (opcjonalnie #4):** obniż `TQ_FULL_SCALE_MV` do ~2000, przebuduj → mocniej czujesz nacisk.
4. Regresja: WA działa, hamulec ucina natychmiast, brak dziwnego zachowania przy prędkości.

### 0.0122 — Koniec „przeciągania" mocy
- **Problem:** po zaprzestaniu pedałowania silnik „dociągał", moc nie schodziła gładko.
- **Zmiana:** flaga `EXTENDED_BOOST_ENABLE` (domyślnie 0) wyłącza blok Override/Extended Boost
  (`main.c` ~2384), który trzymał moc i blokował naturalny zanik.
- **Efekt:** moc podąża za pedałem — płynne opadanie. **Commit `d749f4d`.**
- **Test:** w jeździe przestań pedałować → moc schodzi gładko, bez dociągania i skoku.

### 0.0120 — Wyłączenie dev-telemetrii (czysta magistrala)
- **Problem:** EBICS od pierwszej ms po starcie zalewał CAN ramką `0x81F83100` (co 10 ms),
  podczas gdy fabryka w oknie startu jest cicha.
- **Zmiana:** flaga `SEND_DEV_TELEMETRY` (domyślnie 0) wycisza `0x81F83100` + `0x80010203`.
- **Efekt:** magistrala czysta jak fabryczna; zbędne dane wyłączone. **Commit `13b8098`.**

### 0.0116–0.0121 — Próba naprawy „info na HMI" (patrz sekcja 6)
- Seria poprawek protokołu CAN pod ekran info/ustawień HMI. Zgodne z fabryką, zostają w kodzie
  (czyszczą protokół), ale **problemu nie rozwiązały** — patrz sekcja 6. Commity `36efc35`, `de82001`, `27a83f4`.

---

## 5. Co zbadaliśmy (wiedza zebrana — folder `todo/`)
Podczas pracy powstały szczegółowe analizy (dla dewelopera, do dalszych prac):
- `todo/PLAN_CAN_fake_taxi.md` — jak wygląda cykliczna komunikacja fabrycznego sterownika (wzorzec).
- `todo/PLAN_POWER_PATH_smooth_ride.md` — analiza ścieżki mocy + wnioski z open-source TSDZ2 (płynna jazda).
- `todo/CODE_SKETCH_iq_ramp.md` — szkic adaptacyjnej rampy i_q (następny krok jakości jazdy).
- `todo/PLAN_walk_assist_speed_hold.md` — Walk Assist: siła na start + trzymanie prędkości.
- `todo/COMPARISON_SOC_range.md`, `todo/REVIEW_SOC_and_configurable_ocv.md` — bateria/SOC/zasięg: porównanie i krytyka algorytmu.
- `todo/PLAN_autooff_and_comms_watchdog.md` — auto-wyłączanie po bezczynności + reakcja na zanik CAN.

Pamięć projektu (dla Claude, między sesjami): `~/.claude/projects/.../memory/`.

---

## 6. HMI „info firmware / prędkość max" — dlaczego zaparkowane
- **Objaw:** w menu HMI puste pola: wersja firmware, prędkość max.
- **Co zrobiliśmy:** doprowadziliśmy komunikację CAN EBICS do stanu **identycznego z fabryką**
  (ten sam zestaw ramek, ta sama sekwencja). HMI **odbiera i potwierdza (ACK) wszystkie dane**.
- **Wynik:** mimo to HMI **nie wyświetla** info. Skoro na magistrali wszystko jest poprawne i
  potwierdzone, **blokada jest wewnątrz wyświetlacza** (jego wewnętrzny stan / sprawdzenie typu
  sterownika), czego **z logów CAN nie da się zobaczyć**.
- **Decyzja:** temat **zaparkowany**. Wróci, gdy pojawi się trop spoza logów (np. wariant/wersja
  firmware HMI, albo inny sterownik który NA TYM HMI pokazuje info).
- **Co zyskaliśmy mimo to:** czysty, fabryczny dialekt CAN + wyłączona dev-telemetria.

---

## 7. Stan tematów (skrót)
| Temat | Status |
|---|---|
| Silnik + Walk Assist rusza | ✅ działa (baza be40f75) |
| Przeciąganie mocy po puszczeniu pedału | ✅ naprawione (0.0122) |
| Dev-telemetria zalewająca CAN | ✅ wyłączona (0.0120) |
| Płynne schodzenie mocy / przejścia w jeździe | 🔧 w toku (adaptacyjna rampa i_q) |
| Walk Assist — trzymanie prędkości | ✅ pętla PI aktywna (do ew. dostrojenia) |
| HMI: firmware / prędkość max w menu | ⏸️ zaparkowane (blokada w HMI, nie w CAN) |

---

## 8. Build, wgrywanie, powrót do wersji
**Build:** `build_firmware.ps1 -ArtifactName "<numer>"` → wgrywasz `.build\<numer>_M820_BL820.bin`.
Numer to etykieta (na HMI jako `EBICS <numer>`); kod pochodzi z bieżącej gałęzi.

**Bezpieczeństwo / powrót:**
- Działająca baza: `git checkout be40f75` (build 0.0115).
- Zepsute 0.0114 (do analizy): gałąź `wip-0.0114-broken`.
- Każda zmiana = osobny commit → cofasz pojedynczo: `git revert <hash>`.

**Gałęzie:**
- `feat/incremental-from-be40f75` — bieżąca praca (tu są wszystkie poprawki).
- `experiment/tsdz-experiment` — gałąź z be40f75 (baza).
- `wip-0.0114-broken` — zachowane zepsute 0.0114.
