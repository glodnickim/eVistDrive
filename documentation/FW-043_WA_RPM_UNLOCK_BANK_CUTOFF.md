# FW-043 — Walk Assist: odblokowanie celu RPM + prog odciecia per bank

- **Data:** 2026-07-27
- **Status:** WDROZONE, build `0.0212`. Czeka na TEST NA STOJAKU.
- **Build:** `0.0212_M820_BL820.bin`, SHA-256
  `CAC0DADD72092EA350B9A8971FCEC6F093DC58263BFAEA4F56E82D192777E565`. Bez bledow.
- **Baza:** `0.0211` (FW-042). Znany-dobry bazowy = `0.0204`.
- **Powiazane:** [[FW-042_WA_TARGET_RPM_FIXES]], [[FW-029_WALK_ASSIST_MOTOR_SPEED_PLAN]].
- **⚠️ CZĘŚCIOWO NIEAKTUALNE:** [[FW-051_054_WA_BANK_CONFIG]] przeniósł siłę WA i cel RPM
  z Para1 (`MP.walk_assist_current`/`walk_assist_speed`, opisane niżej) **do bloku banku**,
  osobno dla każdego banku, z jednorazową migracją starych wartości. Próg odcięcia opisany
  tu (bajt nagłówka banku) nadal aktualny, ale zyskał histerezę zamiast twardego cięcia.

---

## A. Celu RPM nie dalo sie ustawic powyzej 6 (BLAD w FW-042)

FW-042 przemianowal pole na "RPM zebatki", ale w sciezce zapisu zostaly trzy bariery z czasow,
gdy pole znaczylo km/h (max 6 km/h). Wpisane 45 cicho wracalo do 6 — relabel byl praktycznie
martwy. Wlasciciel widzial to jako "drugie pole pokazuje 6".

| Gdzie | Bariera |
|---|---|
| `bafang-serializer.js` | `Math.min(6, …)` + `* 100` |
| `bafang-parser.js` | `/ 100` |
| `parser.c` `repair_motor_params` | `walk_assist_speed > 700` -> reset do 600 (wolane po KAZDYM zapisie Para1) |

**Poprawka: zdjete skalowanie x100 — RPM jest liczba calkowita.**
Wartosci 5..120 mieszcza sie pod firmware'owym limitem 700, wiec **walidacji w `parser.c` NIE
trzeba bylo ruszac**. Stara wartosc 600 wypada poza pasmo 5..120 modulu WA -> uzywa domyslnych
45 RPM (automatyczna migracja, bez dzialania uzytkownika).

Zmienione: `bafang-serializer.js` (raw, clamp 0..120, domyslne 45), `bafang-parser.js` (bez /100),
`ebics-compat.js` (domyslny szkic 6 -> 45), `main.c` (`.target_chainring_rpm` bez `/100`),
`tab-controller.js` (format calkowity zamiast `toFixed(1)`).

## B. Prog odciecia WA — ustawialny z Canable, OSOBNO DLA KAZDEGO BANKU

Wczesniej prog byl zaszyta gola liczba `700` (7,00 km/h) w DWOCH niezaleznych miejscach:
`main.c` (`walk_speed_ok` — glowna bramka, gasi `pushassist_flag`) i `walk_assist_motor.c`
(`WA_MOTOR_MAX_WHEEL_X100`). Duplikat grozil rozjechaniem sie wartosci.

**Kluczowe: nie trzeba bylo niczego powiekszac.** Naglowek bloku banku mial wolny bajt
(`buffer[7]`, zawsze 0 — po obu stronach). Teraz niesie prog tego banku w jednostkach
**0,1 km/h** (70 = 7,0 km/h; zakres 1,0-25,5 km/h).

Skutek: **dlugosc bloku bez zmian (185 B), pozycja CRC bez zmian, `MP.bank_store` bez zmian ->
zero zmian w ukladzie EEPROM, zero resetu ustawien, bez podbicia wersji bloku.** Stare zapisane
banki maja tam 0 = "nie ustawiono" -> wartosc domyslna 7,0 km/h.

- `assist_modes.c`: `bank_wa_max_wheel_x10[ASSIST_BANK_COUNT]` (dom. 70), zapis/odczyt na
  `buffer[7]`, walidacja (`< 10` -> domyslne), reset w `assist_modes_init`.
- Nowy getter `assist_modes_get_wa_max_wheel_x100()` — zwraca prog AKTYWNEGO banku w 0,01 km/h.
- `walk_assist_motor.c/.h`: nowe pole wejscia `max_wheel_speed_x100`; stala `WA_MOTOR_MAX_WHEEL_X100`
  zostaje wylacznie jako wartosc awaryjna, gdy wejscie = 0.
- `main.c`: **oba** miejsca (bramka `walk_speed_ok` i wejscie modulu) czytaja ten sam getter —
  koniec z dwiema niezaleznymi liczbami.
- Canable: `canbus.js serializeBankBlob` zapisuje `d[7]`; `bafang-parser.js bankBlob` zwraca
  `wa_cutoff_kmh`; `tab-banks.js` + `index.html` — pole "Walk assist cut-off" (km/h, 1,0-25,5,
  krok 0,1) przy kazdym banku.

**UWAGA BEZPIECZENSTWA:** to jest bezpiecznik, nie ustawienie komfortu. WA trzyma predkosc
SILNIKA, wiec na wysokim biegu bez tego progu rower pojechalby szybciej, niz da sie isc obok.
Podnoszenie progu znacznie powyzej tempa marszu znosi ta ochrone.

## Swiadome decyzje (nie zglaszac jako blad)

- Pole "Walk motor current" (%) zostaje — to druga os: RPM mowi *jak szybko*, prad *jaka sila*.
  Regulator predkosci bez limitu momentu dokladalby pradu w nieskonczonosc pod obciazeniem.
- **Prog pozostaje TWARDYM odcieciem** (decyzja wlasciciela), bez miekkiego ograniczania.
  Znana konsekwencja: na zbyt wysokim biegu daje szarpane wlacz/wylacz (cut -> reset -> START).
  Akceptowalne przy prowadzeniu na niskim biegu.

## Test

1. Ctrl+F5. **Cel RPM:** eVistDrive Walk -> wpisac **45**, Apply, potem Sync — pole musi dalej
   pokazywac **45**, nie 6. To glowny test regresji.
2. `0x00010205`: `target_erps` odpowiada 45 RPM przez przelicznik 4/3 (-> 60 erps).
3. Przed pierwszym zapisem cel = domyslne 45 (stare 600 odrzucone jako spoza pasma).
4. **Prog per bank:** Banks -> bank 1 = 7,0, bank 2 = inna wartosc, Save (0x6022), restart
   sterownika, Read — obie wartosci przetrwaly i sie roznia. Przelaczenie banku zmienia prog
   bez ponownego zapisu.
5. Stare zapisane banki (bajt 7 = 0) daja 7,0 km/h — bez resetu ustawien.
6. Reszta WA wg FW-029 §9 na stojaku: kolo w powietrzu -> opor -> **przytrzymane kolo =
   LIMIT/STALL** -> dopiero potem ziemia. Plus pomiar przelicznika ERPS<->RPM (kadencja z
   `0x00010206` vs `measured_erps`) — `WA_ERPS_PER_RPM_NUM/DEN` jest wciaz TYMCZASOWE (4/3).
