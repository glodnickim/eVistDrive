# FW-040 (fixed-time fade) + FW-041 (gear preload przy starcie)

- **Data:** 2026-07-27
- **Status:** WDROZONE, build `0.0207`. Czeka na test.
- **Build:** `0.0207_M820_BL820.bin`, SHA-256
  `6CC7E08D52D53A08140ED6454C56FD72DD361061C4A14E5ED62F2D8A4FAD6D02`. Bez bledow.
- **Baza:** `0.0206` (FW-034/035/036/037). Znany-dobry bazowy = `0.0204`.
- **Zakres:** `assist_dynamics.c` (zejscie), `ride_control.c` (start), default `release_ms`.
  Bez zmian w FOC, hamulcu, reverse cut, Walk Assist, filtrze predkosci.

---

## FW-040 — zejscie wspomagania: fixed-time fade

**Blad (zdiagnozowany na 0.0206):** krok rampy opadajacej byl liczony od PELNEJ skali
pradu (`step = iq_scale_q / ticks`). Krok byl wiec staly niezaleznie od tego, ile pradu
faktycznie plynelo, wiec zejscie z poziomu czesciowego trwalo UlAMEK ustawionego czasu:

| poziom w chwili puszczenia pedalow | realny czas zejscia przy release_ms=325 |
|---|---|
| 100% skali | 325 ms |
| 30% skali | ~98 ms |
| 10% skali | ~33 ms |

Normalna jazda to kilkanascie-kilkadziesiat % skali, wiec wlasciciel czul, ze "rampy nie ma"
i ze wspomaganie ucina sie ostro (klik przekladni przy rozprzeganiu).

**Poprawka:** release lapie (latch) wartosc `iq_reference_q` w chwili ROZPOCZECIA wygaszania
i dzieli JA przez `release_ms`. Czas zejscia jest teraz staly i rowny konfiguracji, z kazdego
poziomu. Krok nigdy < 1 (zejscie zawsze sie konczy), stan resetowany po dojsciu do zera
(kazde kolejne wygaszanie latchuje na nowo).

Dotyczy obu sciezek: zaprzestanie pedalowania (`release_ms` z profilu) oraz safety cut
(FW-037, `RIDE_SAFETY_RELEASE_MS=200`).

**Default zmieniony na zyczenie wlasciciela:** `release_ms` 325 -> **650 ms** (x2) dla
zaprzestania pedalowania, we wszystkich poziomach obu bankow. Teraz to realne 650 ms.
Safety cut (hamulec/cofanie) zostaje 200 ms — celowo szybszy.

## FW-041 — gear preload: cichsze zalaczenie ze spoczynku

**Problem:** przy ruszaniu z postoju naped ma luz + tarcie statyczne. Prad narasta przy
stojacym wirniku, po czym wirnik przelamuje opor NARAZ i zebatki uderzaja ("mocne wyrwanie
przy zalaczeniu"). Rampa Iq steruje PRADEM, nie predkoscia, wiec sama nie wygladzi tego
przelamania — dlatego objaw wystepuje nawet gdy uklad jest zapiety.

**Poprawka (wariant prosty, bez ruszania blokujacego `delay_1ms(25)`):** dopoki wirnik stoi
przy swiezym starcie, `iq_target` jest ograniczany do malego pulapu `PRELOAD_IQ_CAP=10`
(~1 A fazowego przy CAL_I=95). Istniejaca rampa dochodzi do tego pulapu lagodnie, cicho
wybierajac luz. Gdy wirnik faktycznie ruszy (`motor_erps > 3`) — albo minie timeout
300 ms — cap znika i normalna rampa jedzie dalej OD BIEZACEJ WARTOSCI (bumpless, bez resetu).

**Zabezpieczenia:** timeout 300 ms (sztywny naped nigdy nie zawiesi sie w preloadzie);
`safety_cut` i brak zadania natychmiast rozbrajaja preload; preload zbraja sie tylko przy
realnie swiezym starcie (`current_iq == 0`), nie przy kazdej zmianie nacisku.

## Test

1. **Zejscie:** po zaprzestaniu pedalowania wspomaganie gasnie WYRAZNIE lagodniej i dluzej
   (~650 ms), niezaleznie czy jechales lekko czy mocno. Brak kliku przy rozprzeganiu.
2. **Start:** ruszanie ze spoczynku cichsze, bez "wyrwania"; silnik ZAWSZE rusza (nie wisi).
3. Hamulec/cofanie dalej tna szybko (200 ms) — bez zmian.
4. Poziom 0 off, torque_run, filtr predkosci — bez regresji.
5. Jesli start nadal klika: zwiekszyc `PRELOAD_IQ_CAP` (10->12) lub wydluzyc rampe;
   jesli start ospaly: zmniejszyc cap lub skrocic timeout.
