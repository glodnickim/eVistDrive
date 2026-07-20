# Audyt toru czujnika nacisku (KROK 0 spec. właściciela, 2026-07-19)

Pomiary referencyjne właściciela (korba 165 mm): 0 kg→740 mV, 2 kg→~794 mV,
6 kg→~886 mV, 11 kg→~1038 mV, 84 kg→~2320 mV (pomiar 5 kg odrzucony).
Charakterystyka domyślna: **zero 740 mV, ~27 mV/kg, span 60 kg = 1620 mV,
pełna skala ~2360 mV**. Obala wcześniejsze 750/40/3150.

Domeny: **A** surowe ADC/mV · **B** skorygowane bezwzględne · **C** delta od
zera · **D** nacisk 0,01 kg · **E** próg asysty · **F** diagnostyka/fault.

## Firmware

| Miejsce | Domena | Ocena |
|---|---|---|
| main.c:1457 `(adc*3300)>>12` | A | OK — elektryczne, zostaje |
| main.c:1458 `torque_input_correct()` | A→B | OK |
| torque_input.c:5 `REST_TARGET_NATIVE 740` | B (cel autozera) | OK — kanoniczne zero |
| torque_input.h:19 `TORQUE_INPUT_ZERO_NATIVE 750` | B użyte jako zero skali **D** | **BŁĄD: mieszanie domen** — zero kg musi być 740; 750 to deadband asysty (E), nie początek skali |
| torque_input.h:20 `FULL_SCALE_DEFAULT 3150` (40 mV/kg) | B dla D | **BŁĄD GŁÓWNY** — zmierzone ~2360; kg zaniżone ~1,48× |
| torque_input.h komentarz „40 mV/kg…3150" | dok. | do poprawy |
| assist_modes.c:9,271-274 `ASSIST_TORQUE_ZERO_MV 750` | B→C lokalnie | **BŁĄD architektury** — drugi właściciel zera (i 750≠740); ride-core ma brać deltę ze snapshotu torque_input |
| assist_modes.c:10 / assist_start.c:10 `DELTA_MAX 2550` | C clamp | pochodna starej skali (3300−750); po zmianie związać ze span (niegroźne, tylko clamp) |
| main.c:1420-21,1476 `torque_cumulated+=(x−750)` | B→C **E-legacy** | ZOSTAJE (bit-perfect Legacy); jawny adapter starej skali |
| main.c:1480,2571,2593 `(750+TQ_GATE_MIN/AUTO_TQ)` | E-legacy | ZOSTAJE |
| main.c:2560 `map(x, TQO_threshold, TQ_FULL_SCALE_MV=2000,…)` | E-legacy (mapowanie asysty) | ZOSTAJE — **nie mylić z pełną skalą czujnika** (wymóg spec §4); to „jak szybko rośnie moc", nie odczyt kg |
| main.c:1425,1484,1494 `p_human=cad×filtered×0.00342` | C→moc, kalib. wg datasheet „750–3200→0–80 Nm" | E-legacy, zostaje; nowa moc człowieka w Canable z kg+korba 165 |
| main.c:2614 cadence-assist ×torque_filtered | C legacy | zostaje |
| config.h:285-287 `TQ_FAULT_LOW/HIGH 300/4300` | F elektryczne (B) | OK — oddzielone od progów nacisku |
| config.h:296 `TQ_STUCK_HIGH_MV 3000` | F w domenie B ze starej skali | **BŁĄD** — przy realnych 27 mV/kg 3000 mV ≈ 84 kg; 84 kg dało 2320 mV → próg praktycznie NIEOSIĄGALNY (stuck-high martwy). Przenieść na `load_centikg ≥ 5600` |
| config.h:290 `TQ_RECAL_BAND_MV 100` | F/autozero | **BŁĄD KRYTYCZNY** — ±100 mV ≈ ±3,7 kg: statyczny nacisk <3,7 kg = natychmiastowe re-zero |
| config.h:292-295 reacquire + okno REST_RAW 300–1500 | F/autozero | **BŁĄD KRYTYCZNY** — statyczne 2–11 kg (794–1038 mV) mieści się w oknie; po 3 zgodnych coastach ciężar zostaje nauczony jako zero |
| main.h:143 `MP.torque_full_scale_native` (bez magic/wersji) | persist B | **BŁĄD wg ETAP 7** — pole może zawierać śmieci ze starszego FW; zastąpić strukturą span+magic+ver+CRC |
| CAN_Display.c:332-333, 421-422 (0x…3100, 0x3200) | B surowe mV na CAN | ZOSTAJE — standardowe ramki Bafang nietykalne |

## Canable

| Miejsce | Domena | Ocena |
|---|---|---|
| ui/js/shared.js:11-14 `750 / 40 / 3150` | model D | **BŁĄD GŁÓWNY** — usunąć jako źródło prawdy; firmware wysyła centikg |
| shared.js:16 `LEGACY_TORQUE_LINEAR_MAX_KG 31.2` | pochodna | do przeliczenia przez model |
| shared.js:19-30 `torqueMvToKg/KgToMv` | B↔D | zastąpić modelem: firmware-centikg / fallback „Estimated" 740+1620 / Bafang surowe |
| tab-ebics.js:168-169 `/40`, `×40`; :344 `/40` | duplikaty skali | **BŁĄD** — rozproszone kopie; jedno źródło w modelu |
| tab-ebics.js:170 „0-300 mV = 7,5 kg" | tekst | błędna skala (300 mV ≈ 11,1 kg @27) |
| logger.js:135 `calculateHumanPower(...,17)` | korba sztywno 17 cm | **BŁĄD** — ma być konfigurowalne, default 165 mm |
| logger.js:176,185 `V_ZERO 750`, `/40` | B→D | **BŁĄD** — jak wyżej; docelowo kg z firmware |
| tab-banks.js `without_rotation_threshold_mv 18` | C native (wire) | OK na drucie; prezentacja w kg przez model |

## Wnioski (kolejność napraw = KROKI 1–5 planu)

1. `torque_input` = jedyny właściciel: zero 740, span domyślny 1620, snapshot
   (delta, centikg, źródło kalibracji); deadband asysty oddzielony (relatywny).
2. Autozero: band ±30 mV, reacquire tylko |Δ od aktualnego zera| ≤ 60 mV
   (dryft, nie ciężar), warunek braku momentu silnika, stop podczas kalibracji.
3. Kalibracja stanowa (10–30 kg) + persist span z magic/ver/CRC; stare pole
   `torque_full_scale_native` zdeprecjonowane.
4. Stuck-high → `centikg ≥ 5600`.
5. CAN 0x6025 (telemetria+capabilities) / 0x6026 (operacje kalibracji);
   ramki Bafang i 0x6101 (reset EEPROM!) nietykalne.
6. Canable: model trójtrybowy (firmware / Estimated / Bafang), karta
   kalibracji, logger kg + korba 165 mm.

Legacy monolith (progi absolutne 750+X, TQO/TQ_FULL_SCALE_MV, p_human 0.00342)
celowo BEZ ZMIAN — kryterium odbioru nr 10: charakterystyka istniejącej asysty
nie może się przypadkowo zmienić.
