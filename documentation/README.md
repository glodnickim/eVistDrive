# Przewodnik po dokumentacji — gdzie czego szukać

Aktualizacja: 2026-07-16. Zasada: **jeden temat = jedno źródło prawdy.**
Dokumenty oznaczone ARCHIWALNY mają na górze baner wskazujący następcę —
zostają w repo tylko jako historia decyzji.

## 1. Nowy silnik jazdy (Ride Core — przepisywanie wspomagania wg TSDZ2)

| Dokument | Rola | Status |
|---|---|---|
| `RIDE_CORE_MASTER_CHECKLIST_PL.md` | **NADRZĘDNA lista całego zadania**: decyzje wiążące, co zrobione, co w toku, kolejność dalszych prac, testy końcowe | AKTUALNY |
| `RIDE_CORE_STATUS_CANABLE.md` | Szczegóły techniczne statusu + pełny plan zmian w CANable (tabele parametrów, układ ekranów) | AKTUALNY |
| `RIDE_CORE_REFACTOR.md` | Zapis architektury etapu 1 (kto może pisać prąd silnika, opis modułów) | AKTUALNY (zapis architektury) |

Szukasz „co jest zrobione / co dalej?" → **checklist**. „Jak to technicznie
działa / co dodać w CANable?" → **status**. „Dlaczego kod jest tak podzielony?"
→ **refactor**.

## 2. Protokół CAN i konfiguracja

| Dokument | Rola | Status |
|---|---|---|
| `../protocol/ebics_config_schema.yaml` | **Źródło prawdy przyszłych parametrów** (typy, zakresy, jednostki; ID jeszcze nieprzydzielone) | AKTUALNY (draft) |
| `../protocol/HMI_COMMAND_AUDIT.md` | Które komendy CAN są zajęte, które bloki to konfiguracja, a które telemetria | AKTUALNY |
| `CAN_PROTOCOL_REFERENCE.md` | Analiza logu FABRYCZNEGO firmware Bafang (jak działa oryginał) | AKTUALNY (referencja) |
| `PARAMETERS.md` | Historyczny rejestr bajtów `Para0/1/2` | ARCHIWALNY → status w `RIDE_CORE_STATUS_CANABLE.md` |

## 3. Dla użytkownika (jak działa i co ustawiać)

| Dokument | Rola | Status |
|---|---|---|
| `EBICS_ZMIANY_I_KONFIGURACJA_PL.md` | **Główny opis po polsku**: co zmieniono względem bazy, jak działa, co ustawiasz w CANable | AKTUALNY |
| `MANUAL_KRZYWA_NACISKU.md` | Instrukcja jednej funkcji: krzywa nacisku (expo per poziom) | AKTUALNY |
| `WALK_ASSIST_DZIALANIE.md` | Jak działa prowadzenie roweru (Walk Assist) — pełny łańcuch | AKTUALNY |
| `SPEED_SENSOR_AND_AUTO_OFF.md` | Czujnik prędkości, filtr prędkości, auto-wyłączanie wyświetlacza | AKTUALNY |

## 4. Walk Assist — historia i przyszłość

| Dokument | Rola | Status |
|---|---|---|
| `WALK_ASSIST_DZIALANIE.md` | Obecne działanie (źródło prawdy) | AKTUALNY |
| `../todo/PLAN_walk_assist_speed_hold.md` | Historia strojenia 0.0132–0.0134 + zaczątek przyszłego Walk wg obrotów silnika | CZĘŚCIOWO AKTUALNY (przyszły Walk ERPS) |
| `WALK_ASSIST_BUTTON.md` | Notatka historyczna gałęzi `M820-walk-button` (EN) | ARCHIWALNY |

## 5. Archiwum eksperymentu TSDZ2 (poprzednie podejście, przed Ride Core)

Wszystkie zastąpione przez dokumenty Ride Core (sekcja 1) i schemat (sekcja 2):

| Dokument | Czym było | Następca |
|---|---|---|
| `TSDZ2_ASSIST_ANALYSIS_AND_ROADMAP.md` | Analiza i mapa drogowa pierwszego eksperymentu | `RIDE_CORE_MASTER_CHECKLIST_PL.md` |
| `TSDZ2_EXPERIMENT_CAN_CANDIDATES.md` | Kandydaci parametrów do CAN | `../protocol/ebics_config_schema.yaml` |
| `../todo/PLAN_POWER_PATH_smooth_ride.md` | Plan płynnej ścieżki mocy (pisany na zepsutej 0.0114) | zrealizowane: strojenia 0.0133/0.0134 + Ride Core (Smooth Start/Release/filtry) |
| `../todo/CODE_SKETCH_iq_ramp.md` | Szkic adaptacyjnej rampy prądu | zrealizowane w `assist_dynamics` (build 0.0148) |

## 6. Pozostałe

| Dokument | Rola |
|---|---|
| `../CHANGELOG.md` | Changelog gałęzi `test/soc-temp` (SOC/temperatura) |
| `../todo/COMPARISON_SOC_range.md`, `../todo/REVIEW_SOC_and_configurable_ocv.md` | Analizy SOC/zasięgu (gałąź soc-temp) |
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
