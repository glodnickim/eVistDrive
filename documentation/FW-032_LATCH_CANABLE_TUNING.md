# Karta zmiany FW-032 — latch w Canable (Dynamics) + nowe domyślne tuningu

- **Data:** 2026-07-26
- **Status:** WDROŻONE, build `0.0203`. Czeka na test sprzętowy.
- **Build:** `0.0203_M820_BL820.bin`, SHA-256
  `72206B0587F28A8189B960E2F45FE422CF58A2AED6150CA15953C730C2C2D9B3`. Bez błędów.
- **Zakres:** firmware (tuning_config, ride_control, CAN, EEPROM) + Canable (blob v2/v3/v4/v5, UI Dynamics).
- **Powiązane:** [[project-ride-core-runaway-fixed]], FW-031 (latch na stałych — teraz sterowany z Canable).

---

## 1. Co zrobiono

Parametry zatrzasku jazdy z FW-031 (dotąd stałe w kodzie) są **konfigurowalne w Canable**
(zakładka Dynamics) i **zapisywane na stałe** (blob tuning, 0x6022). Do tego zmiana
domyślnych wartości tuningu wg ustaleń właściciela.

## 2. Firmware

- **tuning_config** (globalny „ride feel"): dodane 3 pola z domyślnymi = wartości testowe FW-031:
  `run_deadband_mv=5`, `hold_ms=1400`, `min_iq_pct=2`. Gettery:
  `tuning_config_run_deadband_mv()`, `tuning_config_assist_hold_ticks()`,
  `tuning_config_min_iq_pct()`. Clamp: 0-100 mV / 0-3000 ms / 0-25 %.
- **Blob tuning:** v2 ma 22 B (latch), v3/v4/v5 ma 24 B (torque-run filter). FW-052
  obniża domyślną podłogę `min_iq_pct` z `4` do `2`; FW-053 wydłuża `hold_ms` z `700` do `1400`.
  Stare v2/v3/v4 z domyślnymi `4%/700 ms` są migrowane do `2%/1400 ms`.
- **ride_control.c:** latch używa getterów zamiast stałych `#define` (logika bez zmian:
  start=„Minimum pedal load", run deadband, hold, podłoga; safety_cut/stop tną natychmiast).
- **Bufory:** `TuningBlob[16]→[24]` (CAN_Display.c), `tuning_store[16]→[24]` (main.h).
  Multiframe 0x6024: 22 B = 3 ramki; END ma indeks 1, więc granica `< 2` dalej wiąże poprawnie
  (zweryfikowane z ramkowaniem narzędzia w bafang-serializer.js).
- **Nowe domyślne tuningu** (DEFAULT_POWER_LEVEL, wszystkie poziomy/oba banki):
  `power_rise_filter_ms=150`, `power_fall_filter_ms=375`, `release_ms=325`,
  `startup_boost strength=100` (było 200), `end_rpm=27` (było 45).

## 3. Canable

- `serializeTuningBlob` → v5, 24 B, pola `assist_run_deadband_mv/assist_hold_ms/assist_min_iq_pct/assist_torque_run_filter_ms`.
- Parser `tuningBlob` → czyta v1 (16 B), v2 (22 B), v3/v4/v5 (24 B), z backfillem defaultów.
- UI Dynamics: nowa karta „Ride latch" z 3 suwakami (Run deadband / Sustain / Current floor).
  Read/Apply/Save przez istniejące przyciski (0x6023 / 0x6024 / 0x6022).

## 4. ⚠️ Jednorazowy reset EEPROM (zaakceptowany przez właściciela — opcja A)

Powiększenie `tuning_store` zmienia układ rekordu MotorParams → FW-023 odrzuca stary zapis
([main.c] `param_record_valid`, sprawdza `length == FMC_OFFSET_FOOTER`). Skutek na TYM flashu:
**zapisane w Canable ustawienia wracają do fabrycznych** (banki, filtry, tuning, kalibracja
momentu kg, próg SOC 100%) — raz zapisać na nowo. **Kalibracja Halla PRZEŻYWA** (wkompilowana).

## 5. Test po build

1. Canable → Dynamics: Read pokazuje 3 suwaki latcha z wartościami (5/1400/2).
2. Zmiana suwaka → Apply → jazda: efekt latcha się zmienia; Save → przetrwa wyłączenie.
3. Jazda: start dopiero od wyraźnego nacisku; lekkie kręcenie nie gasi; stop/cofnięcie/hamulec tną.
4. Walk Assist, kalibracja Halla — działają.
5. Po pierwszym boot: ustawienia fabryczne (jednorazowy reset) — zapisać swoje na nowo.
