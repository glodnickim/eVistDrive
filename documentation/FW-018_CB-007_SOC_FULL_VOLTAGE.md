# Karta zmiany FW-018 / CB-007 — Konfigurowalne napięcie „pełnej baterii" (100%)

- **Data:** 2026-07-20
- **Status:** ZAAKCEPTOWANE + WDROŻONE W DRZEWIE (firmware Kroki 1–3, Canable
  Krok 4). Nie zbudowane / nie wgrane / nie zacommitowane — czeka build+flash i
  test sprzętowy. Wariant: **napięcie CAŁEGO pakietu** (bez per-ogniwo/S).
- **Autor:** przygotowane na zlecenie właściciela (non-programmer)
- **Powiązane:** dokument „SOC V2 – wytyczne wdrożeniowe" (rozdz. 4.4, 5, 10.1);
  audyt SOC z 2026-07-20 (kod w `src/main.c`)

---

## 1. Prostym językiem — co i po co

Chcesz w aplikacji **Canable** móc ustawić **napięcie, przy którym bateria jest
uznawana za naładowaną w 100%**.

Dziś tego nie ma. Sterownik **w ogóle nie wykrywa „pełnej baterii"** — stan
naładowania (SOC) wynika ze zliczania prądu, a przy włączeniu jest tylko
zgadywany z krzywej napięcia. Efekt: po pełnym ładowaniu wskazanie nie musi
trafić w równe 100%.

Ta zmiana daje dwie rzeczy:

1. **Pole w Canable** — wpisujesz **napięcie całego pakietu po naładowaniu i
   zbalansowaniu** (np. `45,9 V`). Aplikacja zapamiętuje to w sterowniku.
2. **W firmware** — przy każdym **włączeniu** sterownik sprawdza napięcie
   pakietu. Jeśli jest stabilne i **≥ ustawiony próg**, ustawia **SOC = 100%**
   („zakotwiczenie 100%"). Działa to nawet wtedy, gdy ładowarka omija bocznik
   sterownika (bo patrzymy tylko na napięcie, nie na prąd ładowania).

**Decyzje właściciela (z 2026-07-20):**
- W Canable wpisujesz **napięcie całego pakietu w woltach** (np. `45,9 V`) —
  dokładnie to, co zmierzysz miernikiem na baterii. Bez per-ogniwo, bez liczby
  ogniw, bez przeliczeń.
- 100% oznacza **pełne naładowanie** (nie obniżony sufit użytkowy).

**Zakres dopuszczalny:** 20–90 V (twarde sito bezpieczeństwa). Dla pakietu 11S
pełne to ~`45,9 V` (11 × 4,17). Funkcja jest **domyślnie nieaktywna** — dopóki
nie wpiszesz napięcia w Canable, SOC działa jak dziś.

### Dlaczego napięcie CAŁEGO pakietu, a nie per-ogniwo
Sterownik mierzy tylko napięcie całego pakietu (`MS.Voltage`), nie ogniw, i
**nie zna dokładnej liczby ogniw** — dziś jedynie szacuje ją jako
`system_voltage/3,6` (obcięcie w dół), co potrafi się mylić o jedno ogniwo
(np. nominał 52 V → 14, 36 V → 10). Gdybyśmy operowali per-ogniwo, ten błąd
przenosiłby się na próg. Dlatego **próg przechowujemy i porównujemy wprost jako
napięcie pakietu** — porównanie `MS.Voltage ≥ próg` nie używa liczby ogniw ani
`/3,6` w żadnym miejscu tej funkcji.

---

## 2. Zakres tej karty (świadomie mały)

Ta karta realizuje **tylko** konfigurowalny próg 100% + zakotwiczenie przy
starcie. **NIE** obejmuje pełnej maszyny stanów ładowania, prawdziwej tabeli
OCV ani przeniesienia zapisu SOC poza jazdę — to osobne karty (patrz „Poza
zakresem"). Chodzi o najmniejszą sensowną, testowalną porcję, która daje
dokładnie to, o co prosiłeś.

**Poza zakresem (osobne karty w przyszłości):**
- FW-019: zdjęcie zapisu flash z jazdy (P0 z audytu — najgroźniejszy błąd).
- FW-020: maszyna stanów ładowania przez bocznik (CHARGE_ACTIVE/TAPER/FULL).
- FW-021: prawdziwa tabela OCV + korekcja dryfu.
- Diagnostyczna ramka `0x602A` (podgląd SOC) — opcjonalnie później.

---

## 3. Jak to działa w firmware (technicznie)

### 3.1 Nowy parametr (zapamiętywany)
Dopisany na końcu `MotorParams_t` (po `ride_engine_pad`, offsety wcześniejszych
pól bez zmian — [inc/main.h:188-190](../inc/main.h#L188)):

```c
uint16_t soc_full_magic;         // 0x5F01 = pole poniżej ważne
uint16_t soc_full_pack_10mv;     // próg pełnego pakietu [jednostki 10 mV], np. 4587 = 45,87 V
```

- Przechowujemy **napięcie całego pakietu** w jednostkach 10 mV (żeby zmieściło
  się w `uint16` także dla dużych pakietów: 65535 = 655 V). Firmware **nie
  przechowuje ani nie używa liczby ogniw** dla tej funkcji.
- `InitEEPROM` (`src/parser.c`): magic = 0, wartość = 0.
- Przy starcie, jeśli `soc_full_magic != 0x5F01` lub wartość poza sensownym
  zakresem → **fallback**: `próg = SOC_FULL_CELL_MV_DEFAULT × (system_voltage/3.6)`
  (jedyne miejsce, gdzie pada `/3,6`, i tylko jako domyślny fallback do czasu
  ustawienia w Canable). Jazda zawsze możliwa.

### 3.2 Nowe stałe (`inc/config.h`)
```c
#define SOC_FULL_CELL_MV_DEFAULT      4170   // mV/ogniwo — tylko do policzenia fallbacku
#define SOC_FULL_CELL_MV_MIN          4100   // dolna granica sensownego progu per-ogniwo
#define SOC_FULL_CELL_MV_MAX          4200   // górna granica
#define SOC_FULL_BOOT_SETTLE_S          10   // sekund stabilnego napięcia po starcie
#define SOC_FULL_BOOT_STABLE_MV          200 // maks. wahanie napięcia PAKIETU w oknie [mV]
#define SOC_FULL_RELEASE_FRAC        0.010f  // zejście z 100% dopiero po zużyciu 1,0% pojemności
```

### 3.3 Zakotwiczenie 100% przy starcie (rozdz. 4.4 dokumentu)
Nowa, mała funkcja `soc_boot_full_detect()` wołana **raz** po ustabilizowaniu
filtrów ADC (po istniejącym `soc_init()`). Porównuje napięcie **całego pakietu**
bezpośrednio — bez liczby ogniw:

```
1. Odczekaj SOC_FULL_BOOT_SETTLE_S sekund, obserwując napięcie pakietu MS.Voltage.
2. Jeśli w tym oknie napięcie było stabilne (wahania <= SOC_FULL_BOOT_STABLE_MV [mV])
   ORAZ MS.Voltage >= (uint32_t)MP.soc_full_pack_10mv * 10:
       -> MS.remaining_mah = MP.battery_capacity_estimated_mah;
          MS.soc_real = MS.soc_display = 100; MS.SOC = 100;
          soc_full_anchor = 1;   // pilnuje, by 100% nie spadło od razu
3. W przeciwnym razie nic — zostaje dotychczasowy licznik SOC.
```

Zaleta: żaden `/3,6`, żadna liczba ogniw w tej ścieżce — porównanie surowego
napięcia pakietu z zapisanym progiem pakietu.

### 3.4 Utrzymanie 100% po odłączeniu ładowarki (rozdz. 4.3)
Po zakotwiczeniu napięcie lekko spada — nie wolno od razu pokazać 99%.
`soc_full_anchor` zwalniany dopiero po **zużyciu 1,0% pojemności**
(`SOC_FULL_RELEASE_FRAC`). Do tego czasu `soc_display` trzymane na 100%.

### 3.5 Nowa ramka CAN `0x602B` — zapis progu
Wolny kod (sprawdzone: brak w kodzie i logach). Konwencja jak `0x6027`
(short write, `source=5`, `target=2`). Canable przesyła gotowe napięcie
pakietu w jednostkach 10 mV (przelicza wpisane wolty × 100):

| Bajt | Zawartość |
|---:|---|
| 0 | wersja formatu = `1` |
| 1 | `soc_full_pack_10mv` LSB (jednostki 10 mV) |
| 2 | `soc_full_pack_10mv` MSB |
| 3–6 | rezerwa (0) |
| 7 | CRC-8/SMBUS (wielomian 0x07, init 0x00) bajtów 0–6 |

- Walidacja w firmware: `dlen ≥ 8`, wersja = 1, CRC zgodne, oraz
  `SOC_FULL_PACK_MIN_MV ≤ wartość×10 ≤ SOC_FULL_PACK_MAX_MV` (20–90 V). Inaczej
  ramka odrzucona (poprzednia wartość zostaje). **Zero `/3,6`, zero ogniw.**
- Po walidacji: ustawienie w RAM + `soc_full_magic = 0x5F01`.
- **Utrwalenie do flash** przez istniejący mechanizm zapisu **na postoju**
  (jak banki/silnik — [main.c:1663](../src/main.c#L1663)); flaga `soc_full_persist`.
  Bez zapisu flash w ruchu.
- Odczyt bieżącej wartości: `soc_full_pack_10mv` dołączony do statusu `0x6028`
  (podniesiony do wersji 2, bajty 5–6). Canable pokazuje wprost w woltach.

### 3.6 Czego NIE ruszamy
- FOC (`FOC.c/.h`), algorytm/kalibracja pozycji.
- Monolit Legacy, ramki Bafang (`0x3200/0x3201/0x3202/0x3205`), `0x6101`.
- Dotychczasowe liczenie SOC (coulomb counting, zasięg, tryb kulawy) — działa
  jak dotąd; dokładamy tylko zakotwiczenie 100% i parametr.

---

## 4. Jak to działa w Canable (CB-007) — WDROŻONE

- **Zakładka EBICS Limits → karta „Full charge voltage — 100% anchor (0x602B)"**
  (pod polami „Electrical and battery limits") —
  [ui/index.html](../../../bafang_canable_pro/ui/index.html).
  Decyzja właściciela: nic nowego nie dokładamy do starych (Bafang) zakładek —
  wszystkie nowe funkcje idą do zakładek EBICS (stare karty będą stopniowo
  wycofywane).
  - Pole liczbowe (20–90 V, krok 0,1) + bieżąca wartość ze sterownika + „Save".
  - Opis: zmierz miernikiem na baterii tuż po pełnym naładowaniu i zbalansowaniu
    (np. ~45,9 V dla 11S).
  - Odczyt bieżącej wartości odpala się przy wejściu w zakładkę EBICS Limits
    (`READ_SYSTEM`); ids pól bez zmian, więc obsługa została w `tab-ebics.js`.
- **Bez per-ogniwo, bez liczby ogniw, bez przeliczeń** — wpisane wolty × 100 →
  `soc_full_pack_10mv`.
- **Odczyt:** `0x6028` v2; przy starym firmware (v1) pole pokazuje
  „Unavailable (older firmware)", nie zero.
- **Zapis:** `SET_SOC_FULL:<pack10mv>` → `setSocFull` (`0x602B` + CRC-8), po
  ACK auto-odczyt statusu. Utrwalenie na postoju istniejącym mechanizmem.
- **Walidacja UI:** 20–90 V (poza zakresem → alert, nic nie wysyła).
- Pliki (zrobione): `bafang-parser.js` (`systemStatus` v2), `canbus.js`
  (`setSocFull`), `server.js` (`SET_SOC_FULL`), `ui/js/websocket.js`
  (`soc_full_set_result`), `ui/js/tab-ebics.js` (`updateEngineUI` + zapis),
  `ui/index.html` (karta).

---

## 5. Testy odbioru

**Firmware:**
1. Diff: `FOC.c/.h`, Legacy, ramki Bafang — bez zmian.
2. Świeży firmware (magic=0) → zakotwiczenie nieaktywne, SOC jak dziś, jazda działa.
3. Zapis `0x602B` = 4587 (45,87 V) → odczyt `0x6028` v2 zwraca 4587; po restarcie
   wartość zachowana (utrwalona na postoju).
4. Ramka spoza zakresu 20–90 V (np. 95 V) lub złe CRC → odrzucona, poprzednia
   wartość trzyma.
5. Start z pakietem `≥ próg` i stabilnym napięciem → SOC skacze na 100%.
6. Start z pakietem `< próg` (np. po jeździe) → SOC **nie** skacze na 100%.
7. Po zakotwiczeniu i spadku napięcia po odłączeniu ładowarki → HMI trzyma
   100% aż do zużycia ~1,0% pojemności, potem normalnie schodzi.
8. Zapis progu **nie** wywołuje FMC przy `ui_8_PWM_ON_Flag==1` (tylko postój).

**Canable:**
9. Wpisane 45,9 V przy 11S → wysłane 4173 mV/ogniwo (≈), odczyt spójny.
10. Pole waliduje zakres (46,5 V → błąd, bo >4,20/ogniwo).

**Sprzętowo (właściciel):** naładuj do pełna → włącz → ma pokazać 100%;
przejedź kawałek → 100% schodzi normalnie.

---

## 6. Ryzyka (uczciwie)

- Zakotwiczenie opiera się **wyłącznie na napięciu** — jeśli ustawisz próg za
  nisko, częściowo naładowana bateria może zostać uznana za 100%. Dlatego
  domyślne 4,17 V/ogniwo i walidacja zakresu.
- To **nie** naprawia jeszcze zapisu SOC w trakcie jazdy (osobna karta FW-019,
  wyższy priorytet bezpieczeństwa) — ta karta świadomie go nie dotyka.
- Bez prawdziwej tabeli OCV częściowe doładowanie offline (np. do 80%) nadal nie
  będzie dokładnie wykryte — tylko pełne. To zgodne z Twoim wyborem „100% =
  pełne".

---

## 7. Pliki

**Firmware:** `inc/main.h` (2 pola MP), `inc/config.h` (stałe), `src/parser.c`
(InitEEPROM), `src/main.c` (`soc_boot_full_detect`, `soc_full_anchor`, persist
na postoju), `src/CAN_Display.c` (`0x602B` zapis + `0x6028` odczyt).
**Canable:** `canbus.js`, `bafang-parser.js`, `server.js`, `websocket.js`,
`tab-ebics.js`, `index.html`.
**Nietykalne:** FOC, Legacy, ramki Bafang, `0x6101`, snapshot, `-canfix`.

---

## 8. Kolejność wdrożenia (po akceptacji)

1. Firmware: parametr + stałe + InitEEPROM + walidacja (build, bez logiki startu).
2. Firmware: `soc_boot_full_detect` + `soc_full_anchor` (build, node-mirror progu).
3. Firmware: `0x602B` zapis + `0x6028` odczyt + persist na postoju (build,
   round-trip).
4. Canable: pole + konwersja V↔mV/ogniwo + odczyt/zapis (node --check, restart).
5. Raport + testy sprzętowe u właściciela.

Po każdym kroku: **STOP + raport**. Bez commitów i budowania bez osobnego
polecenia.
```
