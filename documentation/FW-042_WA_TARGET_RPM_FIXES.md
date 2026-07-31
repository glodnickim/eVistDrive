# FW-042 — Walk Assist: cel jako RPM zebatki + naprawa 3 bledow z recenzji

- **Data:** 2026-07-27
- **Status:** WDROZONE, build `0.0211`. Czeka na TEST NA STOJAKU.
- **Build:** `0.0211_M820_BL820.bin`, SHA-256
  `FFC9AD8F17ACED890FE41AEAF0B09796B66520005E3351F37984C10F9EE60541`. Bez bledow.
- **Baza:** `0.0209` (FW-029 + FW-040/041). Znany-dobry bazowy = `0.0204`.
- **Powiazane:** [[FW-029_WALK_ASSIST_MOTOR_SPEED_PLAN]].

---

## 1. Cel predkosci WA ustawiany z Canable, jako RPM ZEBATKI

Wczesniej cel byl stala kompilacyjna (`WA_MOTOR_TARGET_ERPS_DEFAULT = 25` ~ 2.5 km/h),
wziata z planu jako punkt startowy, nie zmierzona.

- Przejete istniejace pole `MP.walk_assist_speed` (Para1[60..61]) — bylo NIEUZYWANE przez nowy
  modul WA, jest juz zapisywane i edytowalne w Canable. **Zero zmian w ukladzie EEPROM,
  zero resetu ustawien, zero zmian w formacie przewodowym** (dalej x100).
- **Nowe znaczenie: RPM zebatki.** 4500 na drucie = 45 RPM (firmware dzieli przez 100).
  Etykieta km/h byla mylaca: WA trzyma predkosc NAPEDU, a predkosc roweru ustala bieg.
- Sanity band 5..120 RPM — zabezpieczenie przed uszkodzonym EEPROM, nie limit strojenia.
  Stara wartosc `600` (znaczyla 6,00 km/h) czyta sie teraz jako 6 RPM: bardzo wolno, ale
  bezpiecznie; ustawic na nowo w Canable.

### !!! Przelicznik ERPS<->RPM jest TYMCZASOWY — DO POMIARU !!!

`WA_ERPS_PER_RPM_NUM/DEN = 4/3`. Firmware **nie zna** liczby par biegunow ani wewnetrznej
przekladni (`GEAR_RATIO=80` to zdarzenia Halla na obrot KOLA i splata w sobie takze
przelozenie lancucha), wiec nie da sie tego wyliczyc z kodu. 4/3 zaklada, ze GEAR_RATIO
odpowiada przelozeniu ~1:1, co daje 45 RPM ~ 60 erps (tempo starego WA).

**Jak zmierzyc:** na napedzie srodkowym zebatka jest zwiazana z korba, wiec przy pedalowaniu
ze wspomaganiem `MS.cadence` = RPM zebatki. Zestawic ja z `measured_erps`:
`NUM/DEN = erps / cadence`. Obie wartosci sa w ramkach `0x00010205` / `0x00010206`.

## 2. NAPRAWA: runtime jam nigdy nie dzialal (obowiazkowe)

Oba czlony `||` wymagaly `wa_jam_ticks > WA_MOTOR_RUN_JAM_DEBOUNCE`, a ten licznik rosnie
WYLACZNIE przy braku Halla / `erps < 2`. Przypadek "silnik kreci sie, ale duzo wolniej niz
target, przy istotnym pradzie" byl **martwy** — `LIMIT` nigdy by nie zadzialal. Dodatkowo
warunek siedzial w bloku 200 Hz, a stala 800 opisana jest jako "200 ms @4 kHz" (bylyby 4 s).

Poprawka: osobny licznik `wa_run_jam_ticks`, inkrementowany na **4 kHz** obok globalnego
watchdoga (stala zachowuje 200 ms). `CLOSED_LOOP` wchodzi w `LIMIT` gdy
`wa_jam_ticks > WA_MOTOR_JAM_TICKS` LUB `wa_run_jam_ticks > WA_MOTOR_RUN_JAM_DEBOUNCE`.
Licznik zerowany w `walk_motor_reset()` i przy powrocie z `LIMIT`.

## 3. NAPRAWA: `volatile` na sygnalach z przerwania (obowiazkowe)

`ui16_erps` / `ui16_erps_counter` sa zapisywane w `TIMER2_IRQHandler` (Hall) i czytane w petli
glownej — bez `volatile` kompilator moglby je cache'owac. To sciezka bezpieczenstwa WA
(timeout "silnik stoi"). Zmiana lokalna w `main.c` (brak `extern` gdzie indziej).

## 4. Diagnostyka uzupelniona

- `WA_FLAG_SATURATED` teraz FAKTYCZNIE ustawiana (byla zdefiniowana, nigdy nie ustawiana):
  gdy zadanie przyciete do `iq_cap`. `WA_FLAG_JAM` obejmuje oba liczniki jam.
- **Nowa ramka `0x00010206`** (pierwsza, 8 B, byla pelna):
  `error_erps` (signed) | `iq_cap` | `MS.cadence` | `hall_age_ms`.
  Kadencja jest tu celowo — sluzy do pomiaru przelicznika ERPS<->RPM (pkt 1).

## Canable

- `ebics-compat.js`: pole `walk_assist_speed` -> label "Walk chainring speed", jednostka `RPM`,
  zakres 5..120, krok 1 (bylo km/h 0.5..6).
- `tab-ebics.js` / `index.html`: metryka i podsumowanie -> "Walk chainring speed ... RPM".
- Stara zakladka Controller: ostrzezenie przy polu, ze firmware czyta je jako RPM zebatki,
  nie km/h. Pole zostaje (nie dodajemy tam funkcji), ale nie wolno go myloc z predkoscia.
- Serwer zrestartowany. **UI wymaga Ctrl+F5.**

## Test (STOJAK NAJPIERW, wg FW-029 sekcja 9)

1. Canable -> eVistDrive Walk: pole pokazuje RPM; ustawic np. 45 i zapisac.
2. **Kolo w powietrzu:** rusza bez wyrywania, predkosc dochodzi wolno, brak oscylacji stanow.
3. **Lekki opor reka/hamulcem:** prad rosnie powoli, po odpuszczeniu brak przestrzelenia.
4. **PRZYTRZYMANE KOLO: musi wejsc w LIMIT -> STALL** (to sprawdza naprawe z pkt 2 — wczesniej
   by nie zadzialalo), prad zeruje sie, zatrzask trzyma do puszczenia przycisku.
5. **Pomiar przelicznika:** pedalowac ze wspomaganiem w ustalonym tempie, odczytac `cadence`
   (0x00010206) i `measured_erps` (0x00010205) -> podac wynik, wpisze dokladny NUM/DEN.
6. Dopiero potem rower na ziemi, niski bieg.
7. Regresje: ride core assist, poziom 0 off, hamulec/cofanie, zejscie 650 ms, preload startu.
