# Raport końcowy: naprawa toru czujnika nacisku (FW-013 / CB-005)

Data: 2026-07-19. Firmware build: **0.0170_M820_BL820**. Nic nie zacommitowane
(oczekuje na osobne polecenie właściciela).

## 1. Znalezione błędy (audyt KROK 0, pełny w TORQUE_PATH_AUDIT.md)

| # | Błąd | Miejsce | Skutek |
|---|---|---|---|
| B1 | Skala 40 mV/kg / zero 750 / pełna 3150 | torque_input.h; shared.js:11-14; tab-ebics.js:168/344; logger.js:176/185 | kg zaniżone ~1,48× (pomiar 84 kg → 38,7) |
| B2 | Autozero band ±100 mV + reacquire w oknie 300–1500 | config.h:290; torque_input.c coast_evaluate | statyczny ciężar 2–11 kg mógł zostać nauczony jako zero |
| B3 | Stuck-high 3000 mV (skala 40) | config.h:296 | przy 27 mV/kg próg ≈ 84 kg, praktycznie nieosiągalny → zabezpieczenie FW-003 martwe |
| B4 | Podwójny właściciel zera (750 w assist_modes vs 740 autozero) | assist_modes.c:9 | mieszanie domen, niespójny deadband |
| B5 | Persist bez magic/wersji | main.h torque_full_scale_native | śmieci ze starszego FW mogły udawać kalibrację |
| B6 | Korba loggera 17 cm zaszyta | logger.js:135 | błędna moc człowieka dla korby 165 mm |
| B7 | Brak ACK po zapisie banku 0x6021 | CAN_Display.c LONG_END | Canable timeout/FAILED mimo poprawnego zapisu (zgłoszone sprzętowo) |

## 2. Nowa architektura

Firmware `torque_input` jest jedynym właścicielem toru: ADC(mV) → autozero →
korekta → delta od zera → **centikg** → snapshot. Deadband asysty to osobna
stała względna (10) — kg zaczyna się od prawdziwego zera 740. Progi nacisku i
stuck-high pracują w domenie kg, niezależnie od skali. Kalibracja obciążeniem
to opcjonalna maszyna stanów; zapis span z magic/wersją/CRC; brak/uszkodzenie
→ default 1620, jazda zawsze możliwa. Canable czyta gotowe kg z firmware
(0x6025); bez telemetrii używa oznaczonego fallbacku „Estimated" (740/1620);
zwykły Bafang bez modelu EBICS.

## 3. Zmienione pliki

**Firmware:** inc/torque_input.h, src/torque_input.c (przepis v2 + kalibracja +
telemetria), inc/config.h (band 30, reacquire 40, stuck centikg), inc/main.h
(pola torque_cal_*), src/parser.c (init), src/main.c (init/update/tick/persist/
gate), src/assist_modes.c + src/assist_start.c (snapshot zamiast lokalnego 750),
src/CAN_Display.c (0x6025/0x6026 + ACK banków). Docs: TORQUE_PATH_AUDIT.md.

**Canable:** ui/js/shared.js (model), ui/js/tab-ebics.js (skala + karta
kalibracji + updateTorqueCalUI), ui/js/websocket.js (controller_torque),
bafang-parser.js (torqueTelemetry), canbus.js (dispatch + readTorque/torqueCalOp),
server.js (READ_TORQUE/TORQUE_CAL), logger.js (740/1620 + korba 165), ui/index.html
(karta kalibracji).

## 4. Wzory przed/po

- Odczyt kg: PRZED `(mV−750)/40` → PO `(mV−740)/1620×60` (firmware: centikg
  gotowe, Canable tylko `/100`).
- Stuck-high: PRZED `corrected>3000 mV` → PO `load_centikg≥5600`.
- Autozero reacquire: PRZED każdy rest w oknie 300–1500 → PO tylko |rest−zero|≤40 mV.
- Kalibracja: `span60 = (load_avg − zero_avg) × 6000 / reference_centikg`.
- Moc człowieka (logger): korba 0,165 m, zero 740, 27 mV/kg.

## 5. Migracja ustawień

Pola `torque_cal_*` dopisane na KOŃCU MotorParams_t (offsety istniejących pól
banków/tuning stabilne). Sterownik z istniejącym EEPROM: banki OK, torque_cal
magic = śmieć → fallback default. `torque_full_scale_native` (stare) — martwe,
ignorowane. Zapis flash tylko na pełnym postoju (wspólny mechanizm).

## 6. Testy wykonane (lustrzane, node)

- Konwersja: 740→0,00; 794→2,00; 1038→11,00; 2360→60,00; >skala→60; ujemna→0;
  round-trip 601 wartości bez odchyleń.
- Autozero: dryft ±20 mV kompensowany; statyczne 2/11 kg NIE uczą zera; powrót
  po zdjęciu czysty.
- Deadband asysty: bit-perfect ze starym progiem 750.
- Stuck-high osiągalny (84 kg = 58,5 kg ≥ 56).
- Telemetria 0x6025: round-trip firmware↔Canable OK; uszkodzony CRC odrzucony.
- Buildy firmware zielone (0.0166→0.0170); Canable node --check wszystkich modułów OK.

## 7. Wymaga testu na rowerze/adapterze (właściciel)

1. Wgrać 0.0170; zakładka EBICS Torque → „Read live" → kg z firmware, pill
   „Scale: firmware".
2. Zapis banku (0x6021) → OK zamiast FAILED (naprawiony ACK).
3. Kalibracja: Start (bez obciążenia) → zawieś np. 11 kg → Capture → preview
   span → Save; restart → „User calibrated" utrzymane; Restore default → wraca 1620.
4. Statyczny ciężar na korbie w postoju NIE przesuwa zera (odczyt wraca do 0
   po zdjęciu).
5. Mocne, długie deptanie pod górę → brak fałszywego Error 25.
6. Kompatybilność: nowy Canable + stary firmware → pill „Estimated"; zwykły
   Bafang → surowe mV.

## 8. Ryzyka kompatybilności

- Czujnik lekko nieliniowy u góry (27 mV/kg z 2–11 kg, ~19 z 84 kg) → model
  liniowy zaniża bardzo duże naciski; kalibracja per-rower dostraja środek
  zakresu roboczego. Dla asysty (0–20 kg) bez znaczenia.
- 0x6025/0x6026 zweryfikowane jako wolne w audycie/logu/źródłach; pełne logi
  BESST wciąż niedostępne (znane ograniczenie).
- eMTB/Torque skalowane teraz realnym span zamiast 2550 — zmiana w nieaktywnej
  ścieżce TSDZ, i tak czeka strojenie jazdą.
