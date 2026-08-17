# CHANGE_CARD_TEMPLATE — co każda przyszła karta powinna zadeklarować

Cel: przyszły agent/developer (i przyszły Ty) czyta ten szablon wypełniony na górze
karty i wie od razu, co uruchomić i czego NIE musi czytać, zamiast zgadywać.

```
Affected modules:
  - <lista plików src/*.c faktycznie zmienianych>

Affected contracts:
  - <lista granic modułów dotkniętych, np. "torque_input -> rider_input",
     "assist_modes -> ride_control" — patrz documentation/INDEX.md po nazwy dokumentów>

Required docs:
  - <które documentation/*.md trzeba przeczytać PRZED zmianą — użyj INDEX.md>

Required tests:
  - <które scenariusze z REGRESSION_SCENARIOS.md muszą przejść PRZED i PO zmianie>
  - <czy trzeba dopisać nowy scenariusz — jeśli tak, jaki>

Expected behavior change: YES / NO
  - jeśli YES: opisz DOKŁADNIE co się zmienia, zmierzone jak (które metryki/warstwy)
  - jeśli NO: zmiana musi przejść determinism/first-divergence check bez różnicy

Not affected:
  - <moduły/dokumenty, które NIE trzeba czytać ani testować — powiedz to wprost,
     to jest tak samo ważne jak lista "affected">
```

## Przykład (z karty samej TEST INFRASTRUCTURE FOUNDATION, retrospektywnie)

```
Affected modules:
  - tests/host/common/* (nowe)
  - tests/host/torque/*, pipeline/*, scenarios/* (nowe)
  - documentation/INDEX.md, architecture/*, inputs/*, assist/*, motor/*, testing/* (nowe)
  - ZERO plików w src/ lub inc/ zmienionych

Affected contracts:
  - torque_input -> rider_input -> assist_modes -> ride_control -> motor_core (tylko
    CZYTANE przez nowe harnessy, nie zmienione)

Required docs:
  - documentation/ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md (weryfikacja test seams)

Required tests:
  - tests/host/run-host-tests.ps1 (istniejące FW-100/101/102) - potwierdzone PASS,
    niezmienione
  - tests/host/run_regression.ps1 (nowe) - RUN_60..120, CADENCE_RAMP_50_120,
    MISSED_TICK_BURST

Expected behavior change: NO
  - firmware produkcyjny bit-identyczny; zmierzone przez determinism smoke-test
    (dwa przebiegi RUN_100 identyczne przez Compare-Traces)

Not affected:
  - SOC, FOC, Hall, main.c (poza CZYTANIEM), CAN protokół, EEPROM/format Para0/1/2
```

**RELATED SOURCE FILES** N/A — to jest szablon procesu, nie kod.

**KNOWN ISSUES** Brak — świeży dokument. Jeśli szablon okaże się niewystarczający po
kilku użyciach (np. brakuje pola "Rollback plan"), rozszerz go wtedy, nie teraz
("kalibruj bazując na realnym użyciu, nie zgaduj z góry").
