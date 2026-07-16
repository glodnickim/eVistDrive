# EBICS — co zrobiliśmy, dlaczego i jak to działa

Dokument dla **każdego** (użytkownik, deweloper, ktoś nowy). Opisuje całą pracę na gałęzi
`feat/incremental-from-be40f75`: kontekst, co zbadano, co zmieniono, co ustawiasz sam, co zostało.
Żywy — uzupełniany przy każdej zmianie. Na końcu = podstawa do release.

---

## 0. W jednym akapicie (TL;DR)
Firmware EBICS steruje silnikiem roweru elektrycznego (sterownik Bafang M820, wyświetlacz „HMI").
Wersja robocza **0.0114 zepsuła jazdę** (silnik nie ruszał). **Cofnęliśmy** kod do ostatniej
działającej bazy **be40f75 (= 0.0115)** — jeździ. Potem **małymi, testowanymi krokami** poprawiamy:
(1) wyłączyliśmy „przeciąganie" mocy po zaprzestaniu pedałowania, (2) wyłączyliśmy zbędną
telemetrię deweloperską zalewającą magistralę CAN. Trwa praca nad **płynnością jazdy**.
Temat „info firmware/prędkość max na ekranie HMI" **zaparkowany** — okazało się, że blokada jest
wewnątrz wyświetlacza, niewidoczna z magistrali.

---

## 1. Słowniczek (żeby każdy rozumiał)
- **Sterownik** — komputer w silniku (nasz firmware EBICS na nim działa).
- **HMI / wyświetlacz** — panel na kierownicy (pokazuje prędkość, baterię, menu). Gada ze sterownikiem po **CAN**.
- **CAN** — magistrala (kabel), po której lecą „ramki" (paczki danych) między HMI a sterownikiem.
- **Fake taxi** — oryginalny, fabryczny firmware Bafang. Używamy jego logów jako **wzorca** „jak ma być".
- **Walk Assist (WA)** — tryb prowadzenia roweru (trzymasz przycisk, rower jedzie ~6 km/h).
- **Telemetria** — dane, które sterownik wysyła na HMI (prędkość, bateria, moment…).
- **Override / Extended Boost / „przeciąganie"** — funkcja trzymająca moc silnika chwilę PO tym,
  jak przestaniesz naciskać pedał. Powodowała, że moc nie schodziła gładko.
- **i_q** — zadany prąd silnika (im większy, tym większa moc). „Rampa i_q" = jak szybko ten prąd rośnie/maleje.

---

## 2. Kontekst — co się działo (historia)
1. **0.0114 zepsute** — po wgraniu silnik i Walk Assist w ogóle nie ruszały; na HMI „tylko CR X30P".
   Przyczyna: duży, niezacommitowany refaktor ścieżki silnika (m.in. martwa pętla sterowania).
2. **Cofnięcie do be40f75 (0.0115)** — potwierdzona działająca baza (silnik + WA ruszają).
   Zepsute 0.0114 zachowane na gałęzi `wip-0.0114-broken` (na wypadek potrzeby).
3. **Praca krokami na nowej gałęzi** `feat/incremental-from-be40f75` — każda zmiana osobno,
   build → test na rowerze → dopiero następna. Zmiany „bezpieczne" (nie dotykają silnika)
   scalane w paczki; zmiany silnika — pojedynczo.

---

## 3. Flagi konfiguracyjne — co ustawiasz sam (`inc/config.h`)
Zmieniasz wartość → przebudowa (`build_firmware.ps1`) → wgranie. Domyślne wartości są bezpieczne.

| Flaga | Domyślnie | Co robi (po ludzku) | Kiedy zmienić |
|---|---|---|---|
| `EXTENDED_BOOST_ENABLE` | **0 (off)** | „Przeciąganie": trzymanie mocy PO puszczeniu pedału. Off = moc schodzi płynnie za pedałem (jak Bosch). | `1` jeśli chcesz starego „dociągania". |
| `SEND_DEV_TELEMETRY` | **0 (off)** | Wysyłanie 2 deweloperskich ramek (`0x81F83100` moment/kadencja co 10 ms, `0x80010203` debug FOC). Fabryka ich nie wysyła. | `1` tylko gdy deweloper stroi silnik i chce te dane. |
| `IQ_RAMP_ADAPTIVE` | **1 (on)** | Tempo narastania/opadania prądu **zależne od prędkości i kadencji** (miękko na wolno, żwawo przy prędkości). Gdy `0` — stały czas rampy. | `0` tylko do testu stałego tempa; domyślnie zostawić `1`. |
| `IQ_RAMP_TIME_MODE` | **1 (on)** | Nowa rampa czasowa podobna do TSDZ2. Zamiast dużych całkowitych kroków używa wewnętrznego ułamkowego licznika, więc może robić długie, powtarzalne czasy przy pętli 4 kHz. | `0` tylko awaryjnie, żeby wrócić do starej rampy krokowej `IQ_SLEW_*`. |
| `IQ_RAMP_UP_*_TICKS` | `9200` / `1200` | Czas narastania prądu: wolno przy starcie ok. **2,3 s**, szybko w jeździe ok. **0,3 s**. | Mniej = szybciej i bardziej agresywnie; więcej = miękko, ale może być ospale. |
| `IQ_RAMP_DOWN_*_TICKS` | `4000` / `560` | Czas zaniku prądu: wolno przy małej prędkości ok. **1,0 s**, szybko w jeździe ok. **0,14 s**. Hamulec, wstecz i przegrzanie nadal tną natychmiast. | Mniej = szybsze puszczenie; więcej = dłuższe, bardziej miękkie wygaszanie. |
| `SMOOTH_START_ENABLE` | **0 (off)** | Miękkie ruszanie: tłumi wspomaganie 0→100% przez `START_RAMP_TICKS` po postoju. | `1` jeśli ruszanie nadal zbyt „kopie" (rampa adaptacyjna już to łagodzi). |
| `TQ_FULL_SCALE_MV` | **`2000`** | Górna granica mapy „nacisk → moc" (`mapped_torque`). To **podłoga naciskowa BEZ kadencji** — daje moc od pierwszej ms nacisku (fix startu: zanim policzy się kadencja). W jeździe człon kadencyjny zwykle wygrywa, więc charakter zostaje. `3300`=stare (nacisk ledwo się przekładał). | `~1800` = mocniej naciskowo; `3300` = z powrotem „kadencyjne". |
| `TQ_GATE_MIN` | `25` | Próg momentu, poniżej którego brak wspomagania kadencyjnego. Blokuje „wzbudzanie przód-tył" bez nacisku i porządkuje załączanie. | Wyżej = trzeba mocniej nacisnąć by ruszyć (spokojniej); za wysoko = lekkie pedałowanie nie wspomaga. |
| `START_CADENCE_SEED_*` | `on`, `2`, **`18 rpm`** | Mały tymczasowy odczyt kadencji po pierwszych poprawnych krokach do przodu. Nie włącza silnika samodzielnie; tylko ratuje człon kadencyjny na starcie (bez tego `kadencja=0 → moc≈0`). Podniesiony 10→18 = mocniejszy start. | Wyżej = mocniejszy start; wyłączyć tylko gdy za szybko buduje moc na pierwszym ruchu korby. |
| `ASSIST_TORQUE_MODE` | **0 (off)** | Charakter wspomagania: `0`=kadencyjny (jak dziś), **`1`=naciskowy Bosch** (moc ∝ nacisk, kadencja tylko jako „pedałujesz"). Naprawia B/C/D u źródła. | `1` do wypróbowania — **wtedy obniż `TQ_FULL_SCALE_MV`** (~1800–2200), inaczej wspomaganie za słabe. |
| `IQ_SLEW_UP` / `IQ_SLEW_DOWN` | `5` / `10` | Stara rampa krokowa, używana tylko gdy `IQ_RAMP_TIME_MODE=0`. | — |
| `SOFT_CUTOFF_ENABLE` | **1 (on)** | **Miękkie odcięcie stopnia mocy** — usuwa „klik" na samym końcu wspomagania. Zanik momentu (rampa `i_q`) był już płynny, ale finalne wyłączenie mostka (`DISABLE` ~1 s po zatrzymaniu wirnika) było skokowe: nagły zapis napięć faz + odcięcie mostka = słyszalny trzask. Teraz przez chwilę napięcia faz zjeżdżają płynnie do wektora neutralnego, dopiero potem mostek jest odcinany. Hamulec/wstecz/przegrzanie nadal tną natychmiast. | `0` = powrót do starego, natychmiastowego odcięcia (klik wraca). |
| `SOFT_CUTOFF_TICKS` | `40` (≈10 ms) | Długość okna miękkiego zjazdu do wektora neutralnego (w tykach pętli 4 kHz). | Więcej = dłuższe, jeszcze łagodniejsze zwolnienie; za dużo i słychać „mruknięcie" — wtedy zmniejsz. |
| `AUTO_OFF_MINUTES` | **`10`** | Samo-wyłączenie po bezczynności: rower stoi, nikt nie pedałuje, nie hamuje i nie dotyka przycisków przez N minut → sterownik gasi zasilanie (jak fabryka). Jeśli HMI wyśle własny czas ramką `0x6303`, ta wartość jest nadpisywana w locie. `0` = wyłączone. | Krócej = oszczędniej; `0` gdy nie chcesz auto-wyłączania. |
| `COMM_CUT_TICKS` | `75` (3 s) | **Watchdog CAN**: brak jakiejkolwiek ramki z HMI przez 3 s (urwany kabel, uszkodzone HMI) → wspomaganie natychmiast na 0. Silnik nie może „ciągnąć" bez kontroli z manetki. Wspomaganie NIE wraca samo — poziom musi znów przyjść z HMI. | Krócej = szybsza reakcja, ale wrażliwsze na chwilowe zakłócenia. |
| `COMM_OFF_TICKS` | `250` (10 s) | Ciąg dalszy watchdoga: 10 s bez HMI **i rower stoi** → sterownik gasi zasilanie. W trakcie jazdy NIGDY się nie wyłącza (najpierw tnie wspomaganie, dojeżdżasz siłą mięśni, gaśnie dopiero na postoju). | — |
| `WA_FADE_BAND` | `150` (1,5 km/h) | **Walk Assist — dochodzenie bez przelotu** (styl TSDZ2): sufit mocy maleje liniowo w ostatnich 1,5 km/h przed zadaną prędkością — siła słabnie ZANIM dojdziesz do celu, więc rozpęd nie przenosi ponad 6 km/h. | Szerzej = łagodniejsze, dłuższe dochodzenie; węziej = bardziej dynamiczne. |
| `WA_NEAR_HOLD_PCT` | `25` | Ile % maksymalnej siły WA zostaje PRZY zadanej prędkości. Nie 0 — inaczej rower stawałby tuż pod celem i „pompował". | Wyżej, jeśli WA nie domaga pod górkę przy celu. |
| `WA_OVERSPEED_MARGIN` | `50` (0,5 km/h) | Twarde zabezpieczenie: cel+0,5 km/h → prąd natychmiast 0 + wyzerowanie integratora (z górki silnik nie pcha). | — |
| `WA_DEADBAND` | `20` (0,2 km/h) | Martwa strefa integratora wokół celu — bez ciągłego dokręcania/odkręcania prądu przy 6 km/h (mniej „pompowania"). | — |
| `SPEED_STOP_TICKS` | `10600` (2,65 s) | Po ilu bez impulsu koła prędkość = 0. Było 5 s (wskazanie wisiało po zatrzymaniu). Minimalna mierzalna prędkość ≈ **3 km/h** (wolniej = pokazuje 0). | Mniej = szybsze zero, ale wyższa minimalna prędkość; więcej = odwrotnie. |
| `SPEED_DECAY_MARGIN_PCT` | `25` | Między impulsami wskazanie nie może być wyższe niż prędkość wynikająca z ciszy od ostatniego impulsu (+25% zapasu). Przy hamowaniu licznik **płynnie opada** zamiast wisieć; przy stałej jeździe nie odzywa się nigdy (impuls musiałby się spóźnić >25%). | Większy zapas = później zaczyna opadać. |
| `WHEEL_CIRCUMFERENCE` | `2218` mm | Domyślny obwód koła: 27,5″ + opona 2,4″ (średnica 706 mm × π). HMI może nadpisać ramką 0x3203. | Zmierz realny obwód (kreda/taśma) dla największej dokładności. |
| `ASSIST_TORQUE_MODE` | **0** | Charakter wspomagania: `0`=kadencyjny (jak dziś), `1`=naciskowy prosta, **`2`=naciskowy z krzywą expo per poziom** (nowość). | `2` żeby stroić charakter każdego poziomu osobno — patrz **MANUAL_KRZYWA_NACISKU.md**. |
| `ASSIST_CURVE_EXPO_L1…L5` | `0,0,0,0,0` | Wygięcie krzywej nacisk→moc **osobno dla Eco/Tour/Sport/S+/Boost** (−100…+100). `+`=progresywnie (moc gdy dociskasz), `−`=degresywnie (moc od dotknięcia), `0`=prosta. Działa tylko w trybie 2. | Symulacja na żywo: https://claude.ai/code/artifact/2fd06015-0b0a-40d6-bf53-2dfb3e6df175 |

---

## 4. Co zmieniliśmy (changelog — od najnowszego)

### 0.0161 — Dwa banki profili przełączane z kierownicy (FW-005)
Co to jest: obok 5 poziomów masz teraz dwa „charaktery" roweru — **bank 1 =
Power** (wsparcie proporcjonalne do Twojej mocy) i **bank 2 = eMTB**
(adaptacyjny: im mocniej depczesz, tym nieproporcjonalnie więcej pomocy).
Przełączanie bez komputera: na najwyższym poziomie (BOOST) szybko zejdź i
wróć DWA razy (− + − +, w ~2,5 s). Potwierdzenie: pole prędkości na
wyświetlaczu przez ~3 s pokaże **10 km/h = Power** albo **20 km/h = eMTB**.
Wybór jest zapamiętywany (zapis dopiero na postoju — celowo, żeby zapis do
pamięci nigdy nie zaciął jadącego silnika). Zwykłe zmiany poziomów nie
przełączają banku (potrzebne jest pełne podwójne wahnięcie w krótkim czasie).
UWAGA: banki działają w nowym silniku jazdy (developerskim); na domyślnym
Legacy gest niczego nie zmienia w charakterze jazdy — przygotowanie pod
przełączenie silnika po testach. Commit `8e1d134`.
WYMAGANY TEST NA ROWERZE/KOLE: gest, splash 10/20, restart, brak zacięć.

### 0.0160 — Tryb eMTB TSDZ w nowym silniku jazdy (FW-004; domyślnie nieaktywny)
Co to jest: adaptacyjny tryb w stylu „eMTB" — im mocniej naciskasz, tym
NIEPROPORCJONALNIE więcej pomocy (delikatne kręcenie = mało, zdecydowane
deptanie = szybko dochodzi do pełnej mocy). Wierny port wzoru z TSDZ2
(emmebrusa, `apply_emtb_assist`): progresja `nacisk²/mianownik`, mianownik
maleje z parametrem poziomu (60/100/140/160/180) i — w wariancie „power" —
z kadencją; moc odniesiona do 36 V, więc charakter nie zmienia się ze
zużyciem baterii. Przykład (SPORT): lekki nacisk ≈ 11 W, średni ≈ 230 W,
pełny ≈ 900 W (przed limitami poziomu).
WAŻNE: tryb jest w firmware, ale ŻADEN profil go jeszcze nie wybiera —
zachowanie roweru bez zmian do czasu protokołu konfiguracji / banków (FW-005).
Commit `df7bc8c`.

### 0.0159 — Error 25: silnik nie może wspomagać na uszkodzonym czujniku (FW-003)
Co czujesz: gdy czujnik nacisku się psuje, na wyświetlaczu pojawia się Error 25,
a wspomaganie **łagodnie gaśnie do zera** (pedałowanie i manetka; Walk był blokowany
już wcześniej). Wraca samo dopiero ~5 s po tym, jak sygnał jest stabilnie poprawny —
bez migotania błędu i szarpania mocą. Nowość w wykrywaniu: sygnał „zawieszony wysoko"
(awaria elektroniki czujnika udająca ogromny nacisk) jest teraz łapany także **w trakcie
pedałowania** — prawdziwy nacisk zawsze faluje między nogami, więc sygnał trzymający się
nieprzerwanie powyżej ~56 kg przez ~20 s oznacza usterkę. Wcześniej taka awaria dawała
pełną moc, dopóki nie przerwałeś pedałowania na 2 s.
Stałe: `TQ_STUCK_HIGH_MV` (3000 mV ≈ 56 kg), `TQ_STUCK_TICKS` (~20 s),
`TQ_FAULT_HOLD_TICKS` (~5 s trzymania błędu). Commity `ffe28b9` + `f69b8e5`.
**WYMAGANY TEST NA ROWERZE/KOLE:** odpięcie czujnika w pracy → moc do 0 + Error 25;
powrót sygnału → wspomaganie wraca po ~5 s; długie mocne deptanie pod górę → brak
fałszywego alarmu.

### 0.0132 — Miękkie odcięcie stopnia mocy (koniec „kliku" na końcu wspomagania)
Objaw: na samym końcu zaniku wspomagania słychać „klik". Analiza kodu: **zanik momentu jest płynny**
(rampa `i_q` w dół, `IQ_RAMP_DOWN_*`), ale **finalne wyłączenie mostka było twarde**. Gdy wirnik stoi
(~1 s po zatrzymaniu, licznik `uint16_half_rotation_counter>4000`), kod naraz zapisywał `_T/2` na trzy
kanały PWM i od razu wołał `timer_primary_output_config(TIMER0, DISABLE)` — skokowe zwolnienie mostka
z aktywnego trzymania do stanu wysokiej impedancji = słyszalny trzask.
- **Nowy `SOFT_CUTOFF_ENABLE=1`**: zamiast skoku, przez `SOFT_CUTOFF_TICKS` (domyślnie 40 ≈ 10 ms)
  napięcia faz są liniowo sprowadzane do wektora neutralnego (`_T/2`), pozwalając prądom/polu zaniknąć,
  i dopiero wtedy mostek jest odcinany. Efekt: brak skoku napięcia → brak kliku.
- Taktowanie okna jest w `reg_ADC_processing` (~4 kHz, obok licznika obrotu), nie w szybkiej pętli głównej,
  więc czas zwolnienia jest przewidywalny niezależnie od obciążenia CPU.
- **Start bez zmian** — był już płynny (rampa `IQ_RAMP_UP_*` + załączenie PWM przy minimalnym `i_q`).
- **Bezpieczeństwo**: ścieżki awaryjne (hamulec, kręcenie wstecz, przegrzanie) tną moment natychmiast
  przez rampę i nie przechodzą przez to okno — odcięcie stopnia mocy dotyczy tylko naturalnego postoju.
  Ruszenie pedałami w trakcie okna od razu przerywa zwolnienie i wraca do FOC.

### (w kodzie, JESZCZE NIE ZBUDOWANE) — Krzywa nacisku expo, osobno dla każdego poziomu
Pytanie wyjściowe: „jak TSDZ2 wygina krzywą nacisku?" → analiza źródeł + symulacja interaktywna
(https://claude.ai/code/artifact/2fd06015-0b0a-40d6-bf53-2dfb3e6df175) → wybrany wariant **expo
w stylu VESC** (`y = x^(1+e)`), ale ulepszony: **jedna gałka NA KAŻDY POZIOM** (TSDZ2/VESC mają globalną).
- Nowy tryb `ASSIST_TORQUE_MODE=2`: naciskowy z krzywą; `ASSIST_CURVE_EXPO_L1..L5` (−100..+100, domyślnie 0).
- Bramki startu (`START_MIN_STEPS`, `TQ_GATE_MIN`), zatrzask-podtrzymanie i rampa czasowa — **nietknięte**
  (krzywa liczy się ZA decyzją „czy silnik działa", PRZED rampą). Tryb kadencyjny (domyślny) bez zmian.
- Wykładnik liczony raz przy zmianie poziomu; w pętli 4 kHz jedno `powf` (jak człon kadencyjny).
- Manual dla użytkowników: **`documentation/MANUAL_KRZYWA_NACISKU.md`** (zalążek przyszłego wiki).

### (w kodzie, JESZCZE NIE ZBUDOWANE) — Licznik prędkości: szybsze zero + płynne opadanie + obwód koła
Objaw: po zatrzymaniu HMI trzymał ostatnią prędkość jeszcze ~5 s. Porównanie z TSDZ2 (timeout ~2,1 s):
- **`SPEED_STOP_TICKS` 20000→10600**: zero po 2,65 s zamiast 5 s (2× szybciej). Minimalna mierzalna
  prędkość rośnie z ~1,6 do **~3 km/h** (kompromis jak w TSDZ2; WA na 6 km/h bezpiecznie powyżej).
  Szybciej reaguje też warunek „postój" w watchdogu CAN i auto-off.
- **Malejący sufit (`SPEED_DECAY_MARGIN_PCT` 25)**: między impulsami wskazanie ograniczone do prędkości
  wynikającej z ciszy od ostatniego impulsu — przy hamowaniu licznik płynnie opada zamiast wisieć.
  Margines +25% gwarantuje, że przy stałej jeździe mechanizm się nie odzywa (brak migotania).
- **`WHEEL_CIRCUMFERENCE` 2200→2218 mm**: domyślny obwód dla koła 27,5″ z oponą 2,4″
  (584 + 2×61 = 706 mm średnicy × π). HMI i tak może nadpisać ramką 0x3203.

### 0.0131 — Walk Assist: dochodzenie do prędkości bez przelotu
Objaw: WA rozpędzał rower i **przelatywał zadane 6 km/h**. Nasza pętla PI reguluje prąd (siłę) —
przy stałym prądzie rower przyspiesza dalej i nic nie gasiło mocy PRZED celem. Wzorzec z TSDZ2
(`apply_walk_assist`): im bliżej celu, tym wolniej dokłada mocy; nad celem tym szybciej zabiera.
- **Fade** (`WA_FADE_BAND`/`WA_NEAR_HOLD_PCT`): sufit mocy maleje liniowo 100%→25% w ostatnich
  1,5 km/h przed celem — siła słabnie wcześniej, rozpęd nie przenosi ponad cel.
- **Twarde zero** (`WA_OVERSPEED_MARGIN`): cel+0,5 km/h → prąd 0 + zerowanie integratora.
- **Martwa strefa** (`WA_DEADBAND`): ±0,2 km/h wokół celu integrator zamrożony (bez „pompowania").
- Sprzątanie: usunięte martwe `PUSHASSIST_CURRENT` (nigdzie nieużywane).
- Bez zmian: kick startowy 180 ms, limity `walk_assist_speed/current` z Canable, cięcie hamulcem.
- Pliki: `src/main.c` (blok WA w `update_setpoint`), `inc/config.h` (4 flagi — tabela w §3).

### 0.0131 — Auto-off po bezczynności + watchdog CAN
Dwa zabezpieczenia „jak fabryka":
- **Auto-off:** licznik bezczynności w pętli 40 ms; zeruje go KAŻDA aktywność (jazda `Speedx100>0`,
  pedałowanie `cadence>0`, praca silnika, hamulec, dowolny przycisk). 10 min ciągłego spokoju
  (`AUTO_OFF_MINUTES`, domyślnie 10) → samo-wyłączenie. HMI wysyła swój ustawiony czas ramką
  `0x6303` (bajt 0 = minuty) — teraz ją czytamy i nadpisuje ona domyślne 10 (wcześniej ramka była
  ignorowana — komunikacja z HMI nie była w pełni odwzorowana).
- **Watchdog CAN (fail-safe):** każda ramka HMI→sterownik (`target=2`) zeruje licznik utraty łączności.
  Brak ramek przez **3 s** (`COMM_CUT_TICKS`) → wspomaganie na 0 (urwany kabel/uszkodzone HMI nie może
  zostawić „ciągnącego" silnika). Brak przez **10 s + postój** (`COMM_OFF_TICKS`) → wyłączenie zasilania.
  Watchdog uzbraja się dopiero po PIERWSZEJ ramce z HMI (grace period na boot — nie zgasi sterownika,
  zanim wyświetlacz wstanie). W jeździe nigdy nie gasi — tylko tnie wspomaganie.
- **Refaktor:** sekwencja gaszenia (zapis SOC → stop PWM → `PIN_4` DC/DC off → `PIN_5` display off)
  wydzielona do `power_off_controller()`; przycisk on/off działa jak dotąd (te same kroki, jedna funkcja
  zamiast trzech kopii).
- Zgodność z fabryką (log fake taxi): HMI śle cyklicznie rodzinę `0x831063xx` (target=2) — na tym
  opiera się watchdog; sterownik śle heartbeaty `0x82FF1200`/`0x82F83000`/`0x82F8320F` (to już mamy),
  dzięki czemu HMI ze swojej strony pokaże błąd 30 (komunikacja), gdy padnie sterownik.
- Pliki: `src/main.c` (licznik bezczynności, watchdog, helper), `src/CAN_Display.c` (reset watchdoga,
  odczyt `0x6303`), `inc/config.h` (3 nowe flagi — patrz tabela w §3).

### 0.0131 — Moc na starcie
Feedback: start trwa za długo / „muszę pokręcić korbą", zanim pojawi się moc.
- **Przyczyna (potwierdzona):** moc = `kadencja^helper × torque_filtered`. Przy starcie `kadencja=0` →
  cały człon ≈ 0 → nacisk dawał grosze; moc rosła dopiero po kilku obrotach.
- **`START_CADENCE_SEED_RPM` 10 → 18:** seed „udaje" kadencję od pierwszego ruchu → człon kadencyjny
  działa od startu. Mocniejszy niż 10.
- **`TQ_FULL_SCALE_MV` 3300 → 2000:** `mapped_torque` to liniowa mapa NACISKU **bez kadencji**, użyta
  jako podłoga (`if(mapped_torque>i_q) i_q=mapped_torque`). Silniejsza podłoga = **moc od razu z nacisku
  na starcie, bez czekania na kadencję** (i bez lagu EMA). W jeździe (wysoka kadencja) człon kadencyjny
  zwykle wygrywa → **charakter „mocniej kręcisz = mocniej" zostaje**. To hybryda, nie zmiana trybu.
- **Guard bez zmian:** `START_MIN_STEPS=4` + `TQ_GATE_MIN=25` → przypadkowy nacisk bez pedałowania nie rusza.

#### Kontekst: tryby wspomagania vs TSDZ2 (moment vs moc)
TSDZ2 ma oddzielne tryby: **Power** = `kadencja×moment` (= nasz tryb kadencyjny; ma problem `kadencja=0`),
**Torque** = `moment×factor` liniowo (bez kadencji; = nasz `ASSIST_TORQUE_MODE=1`), **eMTB** = `moment²`
(progresywny, bez kadencji; nie mamy). Problem startu istnieje **tylko** w trybie Power/kadencyjnym.
Obniżenie `TQ_FULL_SCALE` = dołożenie liniowej podłogi „Torque" do trybu kadencyjnego → fix startu bez
zmiany charakteru. eMTB (moment²) = rezerwa na później dla progresywnego czucia MTB.

### 0.0126 — Rampa czasowa jak TSDZ2 + szybszy pierwszy odczyt kadencji
Cel: wspomaganie ma pojawiać się przewidywalnie, bez losowego opóźnienia po starcie, ale bez samowzbudzenia silnika od kręcenia korbą bez nacisku.

- **Rampa i_q przeszła z kroków na czas** (`IQ_RAMP_TIME_MODE=1`). Stary kod dodawał/odejmował kilka jednostek prądu w każdym tyku 4 kHz, więc realne czasy były bardzo krótkie i nie dało się uzyskać długich ramp TSDZ2. Nowy kod trzyma wewnętrzny licznik ułamkowy (`iq_setpoint_q`) i liczy krok z żądanego czasu.
- **Czasy są podobne do TSDZ2:** narastanie ok. **2,3 s** przy starcie / ok. **0,3 s** przy szybkiej jeździe; opadanie ok. **1,0 s** przy wolnej jeździe / ok. **0,14 s** przy szybkiej jeździe.
- **Adaptacja zostaje:** o tym, czy użyć wolnej czy szybkiej rampy, decyduje prędkość koła (`4–20 km/h`) i kadencja (`20–70 rpm`). Jeśli choć jeden sygnał mówi „jedziemy”, rampa robi się szybsza.
- **Zabezpieczenia startu zostają bez zmian:** `START_MIN_STEPS=4` i `TQ_GATE_MIN=25` nadal decydują, czy wolno włączyć moc. Sama rampa nie generuje celu prądu.
- **Hamulce i bezpieczeństwo są natychmiastowe:** hamulec, `Backwards_counter>=4` i przegrzanie stage 2 omijają rampę i od razu ustawiają cel. Kręcenie wstecz nie czeka na miękkie wygaszanie.
- **Cadence seed:** po `START_CADENCE_SEED_STEPS=2` krokach do przodu i realnym nacisku firmware publikuje tymczasowo `10 rpm`, zanim normalny pomiar kadencji zdąży się ustabilizować. To poprawia pierwsze pojawienie się wspomagania, ale nie omija bramki momentu i kroków do przodu.

### 0.0125 — Fix S+/Boost + spójne załączanie (odporne na jiggle)
Feedback z 0.0124: tryby S+ i B nie działały; załączanie nieregularne (raz od razu, raz po 0,5–1 obrotu, czasem fałszywie na dołku/zjeździe).
- **S+/Boost martwe:** bramka używała `torque_filtered` (zależnego od `TQfilter` per poziom) → na wysokich poziomach zawsze poniżej progu → blokada. **Fix:** bramka na **surowym `torque_on_crank`** (`>750+TQ_GATE_MIN`) — identyczna na każdym poziomie.
- **Spójne załączanie:** nowy licznik `fwd_run` = kroki korby **z rzędu do przodu** (kasowany przy KAŻDYM kroku wstecz i przy postoju). Wspomaganie startuje tylko gdy `fwd_run ≥ START_MIN_STEPS` **i** realny nacisk. Jiggle przód-tył zeruje licznik → brak fałszywego załączenia; nacisk bez obrotu też nie wzbudza; realne pedałowanie uzbraja w ~4 krokach (~15°) — **za każdym razem tak samo**. Zgodne z TSDZ2.
- Nowe flagi: `TQ_GATE_MIN` (mV nad spoczynkiem), `START_MIN_STEPS`. **Commit `286f77d`.**
- **Test:** (a) S+/B wspomagają; (b) naciśnij+ruszaj → załącza szybko i spójnie; (c) na dołku pokręć korbą **przód-tył bez nacisku** → silnik NIE załącza.

### 0.0124 — Strojenie wg feedbacku z jazdy (0.0123) + tryb naciskowy
Feedback: narastanie za wolne, moc odcina zamiast opadać, wzbudzanie „przód-tył" bez nacisku, nieregularne załączanie.
- **A** narastanie za wolne → `IQ_SLEW_UP_SLOW 3→6`, `FAST 7→12`.
- **B** odcięcie zamiast opadania → `IQ_SLEW_DOWN_SLOW 6→4`, `FAST 12→8` (łagodniej).
- **D** wzbudzanie bez nacisku → wtedy **`TQ_GATE_MIN=15`**, aktualnie **`25`** (człon kadencyjny tylko przy realnym momencie).
- **C** nieregularne załączanie → bramka momentu daje spójny próg.
- **KROK 2 opcja:** `ASSIST_TORQUE_MODE` (flaga, off) — tryb czysto naciskowy Bosch. **Commit `7e74c10`.**
- **Test:** jak niżej — sprawdź narastanie (szybsze), opadanie (łagodne, nie cięte), i czy „przód-tył" już NIE wzbudza silnika.

### 0.0123 — Paczka jakości jazdy (3 gałki, każda flagą)
- **#1 Adaptacyjna rampa i_q** (`IQ_RAMP_ADAPTIVE=1`, aktywna): tempo zmiany prądu zależne od
  prędkości i kadencji → miękkie ruszanie, płynne przejścia w jeździe, gładkie schodzenie.
- **#2 Smooth-start** (`SMOOTH_START_ENABLE=0`, uśpiona): miękkie tłumienie startu — włącz jeśli trzeba.
- **#4 `TQ_FULL_SCALE_MV=3300`** (gałka, domyślnie bez zmian): obniż → bardziej naciskowe czucie.
- Domyślnie zmienia odczucie tylko **#1**; #2/#4 to uśpione gałki do strojenia. **Commit `76609bf`.**
- **Test:** patrz sekcja 4a poniżej.

#### 4a. Jak testować 0.0123
1. **Przejścia w jeździe** (główny cel #1): zwalniaj/dodawaj nacisk → moc płynie gładko, bez skoków.
2. **Ruszanie:** miękkie, bez kopa? (jak za miękko/ospale — zmniejsz progi lub włącz #2).
3. **Nacisk (opcjonalnie #4):** obniż `TQ_FULL_SCALE_MV` do ~2000, przebuduj → mocniej czujesz nacisk.
4. Regresja: WA działa, hamulec ucina natychmiast, brak dziwnego zachowania przy prędkości.

### 0.0122 — Koniec „przeciągania" mocy
- **Problem:** po zaprzestaniu pedałowania silnik „dociągał", moc nie schodziła gładko.
- **Zmiana:** flaga `EXTENDED_BOOST_ENABLE` (domyślnie 0) wyłącza blok Override/Extended Boost
  (`main.c` ~2384), który trzymał moc i blokował naturalny zanik.
- **Efekt:** moc podąża za pedałem — płynne opadanie. **Commit `d749f4d`.**
- **Test:** w jeździe przestań pedałować → moc schodzi gładko, bez dociągania i skoku.

### 0.0120 — Wyłączenie dev-telemetrii (czysta magistrala)
- **Problem:** EBICS od pierwszej ms po starcie zalewał CAN ramką `0x81F83100` (co 10 ms),
  podczas gdy fabryka w oknie startu jest cicha.
- **Zmiana:** flaga `SEND_DEV_TELEMETRY` (domyślnie 0) wycisza `0x81F83100` + `0x80010203`.
- **Efekt:** magistrala czysta jak fabryczna; zbędne dane wyłączone. **Commit `13b8098`.**

### 0.0116–0.0121 — Próba naprawy „info na HMI" (patrz sekcja 6)
- Seria poprawek protokołu CAN pod ekran info/ustawień HMI. Zgodne z fabryką, zostają w kodzie
  (czyszczą protokół), ale **problemu nie rozwiązały** — patrz sekcja 6. Commity `36efc35`, `de82001`, `27a83f4`.

---

## 5. Co zbadaliśmy (wiedza zebrana — folder `todo/`)
Podczas pracy powstały szczegółowe analizy (dla dewelopera, do dalszych prac):
- `todo/PLAN_CAN_fake_taxi.md` — jak wygląda cykliczna komunikacja fabrycznego sterownika (wzorzec).
- `todo/PLAN_POWER_PATH_smooth_ride.md` — analiza ścieżki mocy + wnioski z open-source TSDZ2 (płynna jazda).
- `todo/CODE_SKETCH_iq_ramp.md` — szkic adaptacyjnej rampy i_q (następny krok jakości jazdy).
- `todo/PLAN_walk_assist_speed_hold.md` — Walk Assist: siła na start + trzymanie prędkości.
- `todo/COMPARISON_SOC_range.md`, `todo/REVIEW_SOC_and_configurable_ocv.md` — bateria/SOC/zasięg: porównanie i krytyka algorytmu.
- `todo/PLAN_autooff_and_comms_watchdog.md` — auto-wyłączanie po bezczynności + reakcja na zanik CAN.

Pamięć projektu (dla Claude, między sesjami): `~/.claude/projects/.../memory/`.

---

## 6. HMI „info firmware / prędkość max" — dlaczego zaparkowane
- **Objaw:** w menu HMI puste pola: wersja firmware, prędkość max.
- **Co zrobiliśmy:** doprowadziliśmy komunikację CAN EBICS do stanu **identycznego z fabryką**
  (ten sam zestaw ramek, ta sama sekwencja). HMI **odbiera i potwierdza (ACK) wszystkie dane**.
- **Wynik:** mimo to HMI **nie wyświetla** info. Skoro na magistrali wszystko jest poprawne i
  potwierdzone, **blokada jest wewnątrz wyświetlacza** (jego wewnętrzny stan / sprawdzenie typu
  sterownika), czego **z logów CAN nie da się zobaczyć**.
- **Decyzja:** temat **zaparkowany**. Wróci, gdy pojawi się trop spoza logów (np. wariant/wersja
  firmware HMI, albo inny sterownik który NA TYM HMI pokazuje info).
- **Co zyskaliśmy mimo to:** czysty, fabryczny dialekt CAN + wyłączona dev-telemetria.

---

## 7. Stan tematów (skrót) + mapa na przyszłość

### Zrobione / w kodzie
| Temat | Status |
|---|---|
| Silnik + Walk Assist rusza | ✅ działa (baza be40f75) |
| Przeciąganie mocy po puszczeniu pedału | ✅ naprawione (0.0122) |
| Dev-telemetria zalewająca CAN | ✅ wyłączona (0.0120) |
| Płynność jazdy: rampa czasowa, zatrzask, S+/B | ✅ zrobione (0.0126–0.0127), potwierdzone w jeździe |
| Walk Assist — trzymanie prędkości | ✅ pętla PI aktywna; 🔨 fade+anty-przelot+deadband w buildzie 0.0131 — **do testu** |
| Moc na starcie (seed 18 + TQ_FULL_SCALE 2000) | 🔨 build 0.0131 — **do testu w jeździe** |
| Auto-off po bezczynności (10 min, HMI 0x6303) | 🔨 build 0.0131 — **do testu** |
| Watchdog CAN (3 s → moc 0; 10 s + postój → off) | 🔨 build 0.0131 — **do testu** |
| HMI: firmware / prędkość max w menu | ⏸️ zaparkowane (blokada w HMI, nie w CAN) |

### Do zrobienia (folder `todo/` — co pozostało i od czego zacząć)
| # | Temat | Plik w `todo/` | Co trzeba zrobić |
|---|---|---|---|
| 1 | **SOC nie nadąża za rozładowaniem** | `COMPARISON_SOC_range.md` | Zdjąć/poluzować limiter `SOC_DISP_MAX_STEP` (1 %/min — przy 15 A z 15 Ah realny spadek bywa szybszy); wyświetlacz ma podążać za `soc_real`. |
| 2 | **Zasięg — niespójna podstawa napięcia** | `COMPARISON_SOC_range.md` | Ujednolicić: energia do wyliczenia zasięgu liczona z JEDNEJ podstawy (Wh z coulomb+OCV), nie mieszanka chwilowego napięcia i średnich. |
| 3 | **„Uczenie pojemności" tautologiczne** | `REVIEW_SOC_and_configurable_ocv.md` | Uczyć pojemność tylko z pełnych cykli (100 %→próg), nie z własnych estymat; do tego czasu zaufać `battery_capacity_mah` z Canable. |
| 4 | **Krzywa OCV konfigurowalna po CAN** | `REVIEW_SOC_and_configurable_ocv.md` | Dziś na sztywno LG M58T (NMC). Dodać zapis punktów krzywej ramką CAN (jak inne Para), fallback = obecna krzywa. |
| 5 | **Tryb eMTB (moment², progresywny)** | `PLAN_POWER_PATH_smooth_ride.md` (sekcja tryby) | Nowy tryb obok kadencyjnego i naciskowego: `i_q ∝ moment²` — delikatnie przy lekkim nacisku, mocno przy silnym. Prosty kod + flaga wyboru. |
| 6 | **CAN fake taxi — pełne odwzorowanie TX** | `PLAN_CAN_fake_taxi.md` | Częściowo zrobione (heartbeaty, 0x6303 RX). Zostało: odometr/trip `0x83106302` TX, licznik sesji `0x82F83000` (inkrement b0), ew. pełny wariant A. Wróci razem z tematem HMI Info. |

**Sugerowana kolejność:** najpierw build+test tego co w kodzie (start, auto-off, watchdog) → potem
#1/#2 (SOC/zasięg — bo mylą w codziennej jeździe) → #5 eMTB (czucie jazdy) → #3/#4 (bateria zaawansowane) → #6 przy okazji HMI.

---

## 8. Build, wgrywanie, powrót do wersji
**Build:** `build_firmware.ps1 -ArtifactName "<numer>"` → wgrywasz `.build\<numer>_M820_BL820.bin`.
Numer to etykieta (na HMI jako `EBICS <numer>`); kod pochodzi z bieżącej gałęzi.

**Bezpieczeństwo / powrót:**
- Działająca baza: `git checkout be40f75` (build 0.0115).
- Zepsute 0.0114 (do analizy): gałąź `wip-0.0114-broken`.
- Każda zmiana = osobny commit → cofasz pojedynczo: `git revert <hash>`.

**Gałęzie:**
- `feat/incremental-from-be40f75` — bieżąca praca (tu są wszystkie poprawki).
- `experiment/tsdz-experiment` — gałąź z be40f75 (baza).
- `wip-0.0114-broken` — zachowane zepsute 0.0114.
