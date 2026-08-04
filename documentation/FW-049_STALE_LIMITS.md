# FW-049 — limity przeliczane na biezaco (limit predkosci / pradu / limp nie dzialaly do zmiany poziomu)

> **Aktualizacja FW-075:** ten dokument zachowuje historyczny stan FW-049.
> Aktualne źródła mają `LEGALFLAG=1`, więc po inicjalizacji ustawień fabrycznych
> limit legalny 25 km/h jest domyślnie włączony. Istniejąca poprawna wartość
> zapisana w EEPROM nadal ma pierwszeństwo przed wartością kompilacyjną.

- **Data:** 2026-07-28
- **Status:** WDROZONE W DRZEWIE — **NIE ZBUDOWANE** (czeka na polecenie).
- **Zakres:** `src/main.c`, jedno miejsce w petli glownej. Bez zmian w logice limitow,
  FOC, Walk Assist, hamulcu.

---

## Zgloszenie

Wlasciciel: "tryb legal nie dziala, ograniczenie predkosci na pewno nie dziala".
W Canable (eVistDrive Limits) widoczne 25 km/h, nic nie bylo zapisywane po wgraniu.

## Ustalenia — DWIE osobne rzeczy

### 1. Glowna przyczyna: flaga legal jest domyslnie WYLACZONA (nie jest to blad)

`config.h`: `#define LEGALFLAG 0`. Przy wylaczonej fladze caly blok limitu w
`assist_limits.c` jest pomijany — **zaden limit predkosci nie obowiazuje**. Wartosc
`SPEEDLIMIT 2500` (25,00 km/h) jest poprawna, ale nic jej nie egzekwuje.

Dodatkowo: jednorazowy reset EEPROM przy `0.0203` (zmiana ukladu rekordu, FW-032) ustawil
`legalflag` z powrotem na 0, wiec nawet wczesniejsze wlaczenie zostalo skasowane.

Wlaczenie: Canable -> eVistDrive Limits -> **"Legal speed-limit flag" = Enabled** -> Apply.
(Uwaga: w STAREJ zakladce Controller ten sam bajt `Para1[14]` nazywa sie "Coaster brake" —
mylaca nazwa odziedziczona po Bafangu. W zakladce eVistDrive opis jest poprawny.)

`legalflag` nie robi nic poza tym — sprawdzone wszystkie uzycia. To wylacznie wlacznik
ogranicznika predkosci; nie wplywa na moc, prad ani wspomaganie.

### 2. Prawdziwy BLAD: limity przeliczane tylko przy zmianie poziomu

`main.c` (petla glowna) liczyl te trzy wartosci **wewnatrz** `if(MS.assist_level != assist_level_old)`:

```c
speedlimitx100_scaled  = MP.speedLimitx100 * assist_settings[lvl][1]/100;
phase_current_max_scaled = MP.phase_current_max * assist_settings[lvl][0]/100;
ride_core_iq_limit_scaled = MP.phase_current_max * limp_factor;
```

Wszystkie trzy zaleza od wielkosci, ktore zmieniaja sie **niezaleznie od poziomu**:
- `MP.speedLimitx100` — zapis z HMI/Canable komenda 0x3203,
- `MP.assist_settings` — zapis Para1,
- `limp_factor` — stan naladowania (SoC).

Skutki:
1. **Nowo zapisany limit predkosci nie dzialal**, dopoki jeździec nie zmienil poziomu.
2. **Po restarcie `speedlimitx100_scaled` = 0** (wartosc poczatkowa) az do pierwszej zmiany
   poziomu. Przy WLACZONEJ fladze legal oznacza to `map(speed, 0, 200, ...)` -> wspomaganie
   ucinane juz od ~2 km/h.
3. To samo dotyczylo **limitu pradu ride-core** i **trybu oszczedzania przy niskim SoC** —
   limp_factor nie mial efektu do zmiany poziomu.

Punkt 2 jest istotny: sam wlaczenie flagi bez tej poprawki daloby objaw "wspomaganie znika
zaraz po ruszeniu", trudny do powiazania z przyczyna.

## Poprawka

Trzy przeliczenia wyjete z bloku zmiany poziomu i wykonywane w kazdym obiegu petli glownej.
W bloku `if(assist_level != assist_level_old)` zostaje to, co faktycznie zalezy tylko od
poziomu: reset okna uczenia zasiegu, TQfilter, ext_boost, krzywa expo, gest offroad i
`assist_level_old`. Koszt: kilka mnozen na obieg petli — pomijalny.

## Test

1. Canable -> eVistDrive Limits -> "Legal speed-limit flag" = **Enabled**, "Speed limit" = 25
   -> Apply.
2. Jazda: wspomaganie ma **plynnie gasnac miedzy 25 a 27 km/h** (przy kadencji > 15 obr/min).
3. **Bez** zmiany poziomu po zapisie — limit ma dzialac od razu (wczesniej wymagal zmiany poziomu).
4. **Restart sterownika** i jazda bez dotykania poziomu: wspomaganie normalne do 25 km/h
   (wczesniej: ucinane od ~2 km/h przy wlaczonej fladze).
5. Zmiana limitu w Canable (np. 25 -> 20) -> Apply -> dziala od razu.
6. Regresja: przy WYLACZONEJ fladze brak jakiegokolwiek ograniczenia predkosci (jak dotad).
7. Regresja: zmiana poziomu dalej resetuje okno uczenia zasiegu; gest offroad dalej dziala.

## Uwaga na przyszlosc

Tryb **offroad** (gest: szybkie przeskakiwanie poziomow wg sekwencji) wylacza limit calkowicie.
Kasuje sie po wylaczeniu roweru. Jesli limit "przestanie dzialac" mimo wlaczonej flagi —
sprawdzic najpierw offroad.
