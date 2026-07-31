# FW-044 — WA: przelicznik POTWIERDZONY + prog w zakladce Walk + wykresy Dynamics

- **Data:** 2026-07-27
- **Status:** WDROZONE, build `0.0213`. Czeka na TEST NA STOJAKU.
- **Build:** `0.0213_M820_BL820.bin`, SHA-256
  `2F311FD8F4C7AA3B489F9471F64EBF10C11CA8C452D19FC39741EF437B2ACA3C`. Bez bledow.
- **Baza:** `0.0212` (FW-043). Znany-dobry bazowy = `0.0204`.

---

## A. Przelicznik ERPS<->RPM POTWIERDZONY (koniec z "do pomiaru")

Wlasciciel dostarczyl analize rozebranego M820, ktora **uzasadnia wartosc wpisana wczesniej
jako tymczasowa**:

- przekladnia 3-stopniowa: `30/14 * 36/18 * 48/18` = **80/7** = 11,4286 : 1 (korba -> wirnik),
- **7 par biegunow** (MT6816 w trybie UVW, 6 zmian stanow na obrot elektryczny),
- 7 * 11,4286 = **dokladnie 80** = obecne `GEAR_RATIO`.

Czyli `GEAR_RATIO = 80` znaczy *obrotow ELEKTRYCZNYCH na obrot KORBY*. Poniewaz zebatka jest
sprzezona z korba:

```
erps = RPM_zebatki * 80/60 = RPM_zebatki * 4/3
```

To **dokladnie** `WA_ERPS_PER_RPM_NUM/DEN = 4/3`, ktore juz bylo w kodzie. 45 RPM -> 60 erps
(~4,8 km/h na niskim biegu = tempo marszu). Zmieniony tylko komentarz: z ostrzezenia
"PLACEHOLDER — MUST BE MEASURED" na wyprowadzenie. **Etap pomiaru na stojaku odpada.**

**⚠️ NIE zmieniac `GEAR_RATIO` na 11** — liczy obroty ELEKTRYCZNE; 11 zawyzyloby predkosc
i kadencje ~7x. Zapisane tez w pamieci projektu ([[m820-gear-ratio]]).

Weryfikacja spojnosci: `internal_tics_to_speedx100()` (jedyne liczenie predkosci z `GEAR_RATIO`)
jest zdefiniowane, ale **nigdy nie wywolywane** — martwy spadek po silnikach piastowych.
Prawdziwa predkosc idzie z czujnika kola, wiec nie ma sprzecznosci.

## B. Prog odciecia WA edytowalny w zakladce eVistDrive Walk

Prog jest per bank i fizycznie mieszka w blobie banku, ale edycja byla tylko w zakladce Banks.
Teraz jest rowniez w zakladce Walk — wszystko o WA w jednym miejscu.

**⚠️ Zabezpieczenie (obowiazkowe):** prog zapisuje sie sciezka BANKU (`WRITE_BANK` -> RAM,
`SAVE_BANKS` -> flash na postoju), a nie przez Para1 jak reszta pol tej zakladki.
`state.lastBanks` zawiera WARTOSCI DOMYSLNE dopoki banki nie zostana odczytane — wyslanie
`WRITE_BANK` wczesniej **nadpisaloby realne strojenie poziomow domyslnymi**. Dlatego:

- pole i przycisk sa **zablokowane** dopoki `state.banksSynced` nie jest ustawione
  (komunikat: "press Sync first — it also reads the profile banks"),
- **Sync** w zakladce Walk dodatkowo czyta banki (`READ_BANK:0`/`:1`), wiec blokada zwykle sama
  sie zdejmuje,
- `applyWalk()` wysyla `WRITE_BANK` **tylko dla AKTYWNEGO banku** — drugi nietkniety,
- osobny przycisk "Save cut-off to flash" (`SAVE_BANKS`), z podpowiedzia tlumaczaca roznice:
  prad i RPM zapisuja sie od razu (Para1), prog wymaga Save na postoju (bank).

Zakladka Banks bez zmian — wspolne zrodlo (`state.lastBanks`), wiec wartosci nie moga sie
rozjechac. Odswiezanie pola po odczycie banku: `controller_bank` dodany do listy zdarzen
w `updateEbicsCompatibilityUI`.

## C. Wykresy w zakladce Dynamics nachodzily na karty ponizej

Wykres "Startup boost decay" zaslanial pola karty "Ride latch" (Run deadband / Sustain /
Current floor). Przyczyna: trzy wykresy Dynamics **nie deklarowaly wysokosci**, w odroznieniu
od `profilePlotLayout` (`height = 380`). Bez tego Plotly rysuje na swojej domyslnej wysokosci
(450 px), a kontener `.ebics-chart` rezerwowal tylko `min-height: 300px` -> nadmiar wychodzil
poza karte. To INNY objaw niz poprawiona w FW-042 legenda (tamta wychodzila do GORY).

- `tab-ebics.js`: stala `DYNAMICS_CHART_HEIGHT = 380` ustawiana w `rampLayout()` i w wykresie boost.
- `style.css`: `.ebics-chart` `min-height` 300 -> 380 px, zeby rezerwowane miejsce zgadzalo sie
  z zadeklarowana wysokoscia (brak nachodzenia i brak luki).

## Test

1. Ctrl+F5. **Dynamics:** wykres boost nie zaslania pol "Ride latch"; sprawdzic tez
   Acceleration/Deceleration i przy zmianie szerokosci okna.
2. **Walk bez Sync:** pole progu NIEAKTYWNE z komunikatem (test zabezpieczenia banków).
3. **Sync w Walk** -> pole aktywne, pokazuje wartosc aktywnego banku, zgodna z zakladka Banks.
4. Zmiana progu w Walk -> **Apply** -> zakladka Banks pokazuje te sama wartosc.
5. **Save cut-off** -> restart sterownika -> Read: wartosc przetrwala, **drugi bank niezmieniony**.
6. Regresja: "Walk motor current" i "Walk chainring speed" (45) dalej zapisuja sie przez Para1.
7. WA wg FW-029 §9 na stojaku: kolo w powietrzu -> opor -> **przytrzymane kolo = LIMIT/STALL**
   -> dopiero potem ziemia. Pomiar przelicznika juz NIEPOTRZEBNY (pkt A).
