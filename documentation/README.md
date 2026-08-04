# Przewodnik po dokumentacji — gdzie czego szukać

Aktualizacja: 2026-08-04. Zasada: **jeden temat = jedno źródło prawdy.**
Dokumenty oznaczone ARCHIWALNY mają na górze baner wskazujący następcę —
zostają w repo tylko jako historia decyzji.

## 0. Nadrzędny plan uporządkowania projektu

| Dokument | Rola | Status |
|---|---|---|
| [`PROJECT_CLEANUP_MASTER_PLAN_PL.md`](PROJECT_CLEANUP_MASTER_PLAN_PL.md) | **NADRZĘDNY PLAN AUDYTU I PORZĄDKÓW:** bezpieczeństwo runtime, build, ISR, CAN, Flash/storage, architektura, testy, dokumentacja, licencje, Git i przygotowanie publicznego wydania; zawiera checkboxy i dziennik wznowienia | REALIZACJA — FAZA 1; test bazowy Fazy 0 oczekuje |
| [`PROJECT_CLEANUP_BASELINE_INVENTORY_PL.md`](PROJECT_CLEANUP_BASELINE_INVENTORY_PL.md) | Migawka Fazy 0: target M820/BL820, rozróżnienie buildów potwierdzonych/kandydatów, klasyfikacja zmodyfikowanych i nieśledzonych plików | AKTUALNY — FAZA 0 |
| [`BUILD_FIRMWARE.md`](BUILD_FIRMWARE.md) | **Źródło prawdy dla developera:** toolchain 13.2.1, normalny/diagnostyczny build, manifesty, mapa Flash i blokada nieaudytowanego release | AKTUALNY — FAZA 1 |

## 1. Nowy silnik jazdy (Ride Core — przepisanie toru wspomagania)

| Dokument | Rola | Status |
|---|---|---|
| `RIDE_CORE_MASTER_CHECKLIST_PL.md` | **NADRZĘDNA lista całego zadania**: decyzje wiążące, co zrobione, co w toku, kolejność dalszych prac, testy końcowe | AKTUALNY |
| `RIDE_CORE_STATUS_CANABLE.md` | Szczegóły techniczne statusu + pełny plan zmian w CANable (tabele parametrów, układ ekranów) | AKTUALNY |
| `RIDE_CORE_REFACTOR.md` | Zapis architektury etapu 1 (kto może pisać prąd silnika, opis modułów) | AKTUALNY (zapis architektury) |
| [`FW-048_COAST_RELEASE.md`](FW-048_COAST_RELEASE.md) | Źródło analizy kąta Halla/six-step: klik przy zatrzymaniu oraz hipotezy i wymagane logi dla pulsowania pod obciążeniem przy małym ERPS | ANALIZA DO TESTU SPRZĘTOWEGO |
| [`FW-022_HALL_CALIBRATION_PERSISTENCE.md`](FW-022_HALL_CALIBRATION_PERSISTENCE.md) | Kolejne wpisy FW-022/FW-078: trwałość kalibracji oraz poprawne przejście obu faz `0x6200` | FW-078 POTWIERDZONE SPRZĘTOWO 0.0275 |
| [`FW-068_START_CONDITION_CONFIGURABLE.md`](FW-068_START_CONDITION_CONFIGURABLE.md) | Kolejne wpisy FW-068/FW-077: historia warunku startu oraz aktualny jeden próg w kg, bank v7 i migracja | FW-077 WDROŻONE; TEST SPRZĘTOWY OCZEKUJE |
| `FW-033_TORQUE_CONDITIONING_PLAN.md` | Plan brakujacej warstwy ride core: diagnostyka saturacji torque, `torque_run_mv`, mniej pikujace eMTB i testy A/B | PLAN DO WDROZENIA |
| [`FW-074_ASSIST_CARRY_OVER_PLAN.md`](FW-074_ASSIST_CARRY_OVER_PLAN.md) | Plan bezpiecznego, opcjonalnego dociągnięcia po końcu pedałowania: presety, maszyna stanów, bank blob v7, Canable i testy | PLAN DO WDROŻENIA |

Szukasz „co jest zrobione / co dalej?" → **checklist**. „Jak to technicznie
działa / co dodać w CANable?" → **status**. „Dlaczego kod jest tak podzielony?"
→ **refactor**.

## 2. Protokół CAN i konfiguracja

| Dokument | Rola | Status |
|---|---|---|
| `../protocol/evistdrive_config_schema.yaml` | **Źródło prawdy przyszłych parametrów** (typy, zakresy, jednostki; ID jeszcze nieprzydzielone) | AKTUALNY (draft) |
| [`PLAN_FLASH_CONFIG_STORAGE_EXPANSION_PL.md`](PLAN_FLASH_CONFIG_STORAGE_EXPANSION_PL.md) | **Podplan planu nadrzędnego:** poprawna granica aplikacji, Config A/B, migracja ustawień, większe bloby banku/tuningu i bezpieczny CAN | REALIZACJA — MAPA FLASH WDROŻONA |
| `../protocol/HMI_COMMAND_AUDIT.md` | Które komendy CAN są zajęte, które bloki to konfiguracja, a które telemetria | AKTUALNY |
| `CAN_PROTOCOL_REFERENCE.md` | Analiza logu FABRYCZNEGO firmware Bafang (jak działa oryginał) | AKTUALNY (referencja) |
| [`EBICS_ZMIANY_I_KONFIGURACJA_PL.md` — przełącznik diagnostyki CAN](EBICS_ZMIANY_I_KONFIGURACJA_PL.md#can-diagnostics-build-switch) | **Źródło prawdy dla developera:** normalny/diagnostyczny build, wyłączane ID, wyjątki i reguły dodawania nowych ramek | AKTUALNY |
| `PARAMETERS.md` | Historyczny rejestr bajtów `Para0/1/2` | ARCHIWALNY → status w `RIDE_CORE_STATUS_CANABLE.md` |
| `FW-018_CB-007_SOC_FULL_VOLTAGE.md` | Karta konfigurowalnego napięcia pełnego pakietu i kotwicy SOC 100% | WDROŻONE W DRZEWIE, test sprzętowy progu oczekuje |
| `FW-022_HALL_CALIBRATION_PERSISTENCE.md` | Karta diagnostyki Halla: dowody z 0.0186/0.0187, format `0x6017`, ryzyka EEPROM i kolejność testu końcowego | AKTUALNY (finalna poprawka czeka na odczyt po kalibracji) |

## 3. Dla użytkownika (jak działa i co ustawiać)

| Dokument | Rola | Status |
|---|---|---|
| `EBICS_ZMIANY_I_KONFIGURACJA_PL.md` | **Główny opis po polsku**: co zmieniono względem bazy, jak działa, co ustawiasz w CANable | AKTUALNY |
| `MANUAL_KRZYWA_NACISKU.md` | Instrukcja jednej funkcji: krzywa nacisku (expo per poziom) | AKTUALNY |
| `WALK_ASSIST_DZIALANIE.md` | Jak działa prowadzenie roweru (Walk Assist) — pełny łańcuch | AKTUALNY |
| `SPEED_SENSOR_AND_AUTO_OFF.md` | Czujnik prędkości, filtr prędkości, auto-wyłączanie wyświetlacza | AKTUALNY |
| `FW-036_SPEED_SENSOR_GLITCH_PLAN.md` | Plan diagnozy i odrzucania falszywych impulsow speed, ktore zawyzaja `Speedx100` i chwilowo odcinaja assist | PLAN DO WDROZENIA |

## 4. Walk Assist — działanie i historia

| Dokument | Rola | Status |
|---|---|---|
| `WALK_ASSIST_DZIALANIE.md` | Obecne działanie (źródło prawdy) | AKTUALNY |
| `FW-060_WA_CONSTANT_RPM_CONTROLLER.md` | Architektura, zabezpieczenia, diagnoza kolejnych testów oraz procedura testu regulatora RPM | FW-082 / 0.0282, TEST STOJAKOWY OCZEKUJE |
| `FW-029_WALK_ASSIST_MOTOR_SPEED_PLAN.md` | Historyczny plan wcześniejszych regulatorów i prób strojenia | ARCHIWALNY → FW-060 |
| `FW-051_WA_SETTINGS_PER_BANK_PLAN.md` | Prąd, RPM i próg odcięcia WA zapisane osobno dla Banku 1/2 w bank blob v2; migracja v1 bez resetu EEPROM | WDROŻONE 0.0238 |
| `FW-054_WA_TIMED_RUN.md` | Opcjonalne bankowe podtrzymanie WA po puszczeniu przycisku, timeout 1-120 s i zatrzymanie dowolnym przyciskiem | WDROŻONE 0.0242, TEST SPRZĘTOWY |
| `../archive/PLAN_walk_assist_speed_hold.md` | Historia strojenia 0.0132–0.0134 + zaczątek przyszłego Walk wg obrotów silnika | CZĘŚCIOWO AKTUALNY (przyszły Walk ERPS) |
| `WALK_ASSIST_BUTTON.md` | Notatka historyczna gałęzi `M820-walk-button` (EN) | ARCHIWALNY |

## 5. Archiwum wczesnego eksperymentu (poprzednie podejście, przed Ride Core)

Wszystkie zastąpione przez dokumenty Ride Core (sekcja 1) i schemat (sekcja 2):

| Dokument | Czym było | Następca |
|---|---|---|
| `../archive/ASSIST_ANALYSIS_AND_ROADMAP.md` | Analiza i mapa drogowa pierwszego eksperymentu | `RIDE_CORE_MASTER_CHECKLIST_PL.md` |
| `../archive/EXPERIMENT_CAN_CANDIDATES.md` | Kandydaci parametrów do CAN | `../protocol/evistdrive_config_schema.yaml` |
| `../archive/PLAN_POWER_PATH_smooth_ride.md` | Plan płynnej ścieżki mocy (pisany na zepsutej 0.0114) | zrealizowane: strojenia 0.0133/0.0134 + Ride Core (Smooth Start/Release/filtry) |
| `../todo/CODE_SKETCH_iq_ramp.md` | Szkic adaptacyjnej rampy prądu | zrealizowane w `assist_dynamics` (build 0.0148) |

## 6. Pozostałe

| Dokument | Rola |
|---|---|
| `../CHANGELOG.md` | Changelog gałęzi `test/soc-temp` (SOC/temperatura) |
| `../archive/COMPARISON_SOC_range.md`, `../todo/REVIEW_SOC_and_configurable_ocv.md` | Analizy SOC/zasięgu (gałąź soc-temp) |
| `../todo/PLAN_autooff_and_comms_watchdog.md` | Plan auto-off i watchdoga łączności |
| `../todo/PLAN_CAN_fake_taxi.md` | Plan rozdzielenia rodzin ID CAN (logi fabryczne vs EBICS) |
| `local-notes/` | Notatki lokalne poza gitem (wersje robocze PL) |
| Pliki `.JPG/.png` | Zrzuty ekranów CANable i wykresy zachowań do instrukcji |
| Skrypty `CRC16*.py`, `Logaufbereitung.py`, `Python CAN listener.py` | Narzędzia: sumy kontrolne binarek, obróbka logów CAN |

## Zasady utrzymania

1. Nowy opis dopisuj do istniejącego źródła prawdy zamiast tworzyć kolejny plik.
2. Gdy dokument traci aktualność: baner `> **ARCHIWALNY** → następca` na górze
   i wpis w sekcji 5 — nie kasujemy historii.
3. Po każdej większej zmianie aktualizuj datę i wpis w tym przewodniku.
