# FW-055 — usunięcie zakładki „Banks" z Canable

- **Data:** 2026-07-29
- **Status:** WDROŻONE. Tylko UI (Canable), brak zmian w firmware.
- **Zakres:** `ui/index.html`, `ui/js/init.js`, `ui/js/websocket.js`. Plik `ui/js/tab-banks.js`
  **usunięty**.

---

## Decyzja właściciela

Zakładka „Banks" ma zostać całkowicie usunięta z Canable.

## Weryfikacja przed usunięciem (żeby nic nie stracić)

Sprawdzono w kodzie, że **każda funkcja zakładki Banks ma już niezależny odpowiednik** w
zakładkach eVistDrive:

| Funkcja w Banks | Odpowiednik | Uwaga |
|---|---|---|
| Tabela poziomów (support/eMTB/torque/moc/prąd/boost/smooth/release) | **eVistDrive Profiles** | Profiles ma WIĘCEJ pól (m.in. `support_min/max_pct`, `reference_power_w`, `progression_pct`, `emtb_based_on_power`, `emtb_reference_voltage_mv`, `assist_without_rotation`, `without_rotation_threshold_mv`, `startup_boost_end_rpm`, `power_rise/fall_filter_ms`), edytowane pojedynczo (bank+poziom), nie tabelą wszystkich 5 naraz |
| Wybór trybu banku (Power/eMTB/…) | eVistDrive Profiles (`ebicsProfileModeSelect`) | Różnica: Banks ustawiał tryb dla **całego banku** jednym kliknięciem (`bank.levels.forEach`); Profiles ustawia **pojedynczy poziom** — trzeba 5× dla całego banku. Utrata wygody, nie funkcji |
| Read/Write RAM/Save Flash dla banków | eVistDrive Profiles: `ebicsProfilesReadButton`/`ApplyButton`/`SaveButton` | Identyczne komendy (`READ_BANK`/`WRITE_BANK`/`SAVE_BANKS`) |
| Próg odcięcia Walk Assist (`wa_cutoff_kmh`) | eVistDrive Walk | Był w obu miejscach (zdublowany); teraz tylko w Walk |
| Globalny Tuning (rampy, cadence step) | eVistDrive Dynamics: `ebicsDynamicsReadButton`/`ApplyButton`/`SaveButton` | Identyczne komendy (`READ_TUNING`/`WRITE_TUNING`/`SAVE_BANKS`) |
| Etykieta aktywnego banku | eVistDrive Live (`ebicsLiveBank`) | Już tam była, niezależnie |

**Wniosek: zero utraconych funkcji**, poza wygodą zbiorczego podglądu 5 poziomów w jednej
tabeli i przyciskiem „ustaw cały bank jednym kliknięciem".

## Zależności — sprawdzone przed usunięciem pliku

`websocket.js` importował z `tab-banks.js` funkcje `updateBanksUI()`/`updateTuningUI()` i
wołał je na zdarzeniach `controller_bank`/`controller_tuning`. Sprawdzono, że **niezależnie**
od tego w tym samym handlerze wołane jest `updateEbicsUI(parsedEvent.type)`
(`tab-ebics.js`), które na te same zdarzenia odświeża `renderProfileEditor()` /
`renderDynamics()` — czyli usunięcie wywołań `updateBanksUI()`/`updateTuningUI()` nie
zostawia niczego nieodświeżanego.

`selectedLevel()` w `tab-ebics.js` (używane przez Profiles) już wcześniej obsługiwało brak
danych (`state.lastBanks` puste) pokazując „No bank data." — nie polegało na tym, że
`tab-banks.js` wcześniej zasiewał `state.lastBanks` domyślnymi wartościami. Bezpieczne do
usunięcia bez zastępczego seedowania.

## Zmiany

- `ui/index.html`: usunięty przycisk zakładki (`data-tab="banks"`) i cała treść
  `#tab-banks` (tabela banków + sekcja Global Ride-Feel Tuning).
- `ui/js/init.js`: usunięty `import './tab-banks.js'`.
- `ui/js/websocket.js`: usunięty import `updateBanksUI`/`updateTuningUI` oraz ich dwa
  wywołania (na `controller_bank` i `controller_tuning`).
- `ui/js/tab-banks.js`: **plik usunięty**.
- Serwer Canable zrestartowany.

## Test

1. Ctrl+F5 — brak zakładki „Banks" na pasku, brak błędów w konsoli przeglądarki.
2. eVistDrive Profiles: Read banks → edycja poziomu → Write (RAM) → Save (Flash) — działa
   jak dotąd.
3. eVistDrive Dynamics: Read/Write/Save tuningu — działa jak dotąd.
4. eVistDrive Walk: próg odcięcia, siła, cel RPM, latch — działa jak dotąd (FW-051..054).
5. eVistDrive Live: „Active bank" nadal pokazuje poprawny bank po odczycie.
6. Gest zmiany banku na rowerze — bez zmian (logika w firmware, nieruszona).
