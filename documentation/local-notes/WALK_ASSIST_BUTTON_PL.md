# Walk Assist Button - notatki lokalne

Ten plik jest lokalna polska notatka robocza i jest ignorowany przez Git.

W repo do commitow/release notes prowadzimy rownolegle dokumentacje po angielsku:

- `documentation/WALK_ASSIST_BUTTON.md`

## Cel

Dodac dodatkowe zabezpieczenie dla Walk Assist:

- tryb Walk Assist musi byc aktywny po CAN (`MS.pushassist_flag`)
- fizyczny przycisk DOWN musi byc wykryty na analogowym wejsciu `adc_value[5]`
- wykrywanie przycisku ma miec prosty filtr czasowy, zeby pojedyncze smieci ADC nie uruchamialy WA

Zakres tej funkcji jest celowo waski. Nie dodajemy rampy pradu, zmian SOC, range ani innych poprawek z backupu.

## Jak ma sie zmienic zachowanie

Przed zmiana:

```text
CAN pushassist_flag = SET -> Walk Assist moze dzialac
```

Po zmianie:

```text
CAN pushassist_flag = SET
+ analogowy DOWN aktywny po filtracji
-> Walk Assist moze dzialac
```

## Notatki z testow

Format:

```text
Data:
Wersja/commit:
Warunki testu:
Co zrobiono:
Wynik:
Uwagi:
```
