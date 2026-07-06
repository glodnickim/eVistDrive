# Plan: cykliczna komunikacja CAN fabrycznego M820 (fake taxi) → wdrożenie w EBICS

## Kontekst (dlaczego ten plan powstaje)

Firmware **EBICS** (`BAFANG_GD32F303RCT6`) ma wysyłać po CAN te same ramki **cyklicznie**
i w tej samej kadencji co sterownik fabryczny Bafang M820 ("fake taxi"), aby HMI
(CR X30P / M820TS) działał jak z oryginałem (pełne Info/Ustawienia, telemetria, auto-off).

### Źródła danych — zweryfikowane (nie zgadywać ponownie)

Prawdziwe fabryczne logi CAN (zawierają string `FAKE TAXI`):
- ✅ `C:\snapshot\bafang_canable_pro\logs\log-2026-05-24-15-38-56-n0.log` (36 KB, najlepszy)
- ✅ `C:\snapshot\bafang_canable_pro\logs\log-2026-06-05-18-30-31-n0.log` (10 KB)

Logi które **NIE są fabryczne** (to EBICS, pole `0x6001` = `EB0.01xx`) — nie mylić:
`log-2026-06-26-23-33-33` (EB0.0104), `-23-18-15` (EB0.0101), `-23-45-16` (EB0.0105),
`log-2026-06-29-21-10-35` (EB0.0108), `log-2026-06-29-22-18-07` — wszystkie 0x80010204-dominant.

Fabryka serial (nie CAN): `Desktop\BAFANG M820\Fake Taxi App\logs\aa55_raw_*.log` —
protokół 55AA/BESST, autorytatywne stringi tożsamości + bloki Para0/1/2.

Uzupełnienie: `documentation/CAN_PROTOCOL_REFERENCE.md` — do **wysokich częstotliwości**
(logi CAN są kompresowane „Repeated N times", więc nie oddają okresów ramek szybkich).

---

## ⚠️ CENTRALNE USTALENIE — EBICS mówi innym dialektem niż fabryka

| | Fabryczny fake taxi (`log-2026-05-24`) | Obecny EBICS (logi 06-26/06-29) |
|---|---|---|
| Główna cyklika statusu | `0x83106300/01/02/03/04`, `0x82F83000`, `0x82F8320F` | `0x80010204`, `0x81F83100`, `0x80010203` |
| Info multiframe | `0x821C/821D/821E` → **target=3** | ta sama rodzina 821x/822x (zależnie od buildu) |
| `0x80010204` w fabryce | **0 wystąpień** | dominuje (dziesiątki tys.) |
| `0x83106300` w EBICS | — | tylko sporadycznie |

**Decyzja architektoniczna do rozstrzygnięcia PRZED kodowaniem:**
- **Wariant A — „bądź fake taxi 1:1":** dodać rodzinę `0x83106300` + `0x82F83000` +
  `0x82F8320F`, wygasić `0x80010204/0x0203/0x3100`. Największa zgodność, przebudowa TX.
- **Wariant B — „dostrój obecny EBICS":** zostawić `0x80010204` (telemetria z HMI działa)
  i uzupełnić braki (odometr `0x83106302`, licznik `0x82F83000`, TX auto-off `0x83106303`).

> Rozstrzygnięcie: zgrać log EBICS↔HMI i sprawdzić, z której rodziny HMI czyta telemetrię
> i wypełnia Info. HMI przyjmuje `0x80010204` → Wariant B. HMI ignoruje → Wariant A.
> Domyślnie (jeśli brak testu): Wariant B (mniejsze ryzyko).

---

## Fabryczna cyklika — zdekodowana z `log-2026-05-24-15-38-56` (wzorzec)

| ID (EFF) | DLC | Przykład Data | Okres | Semantyka | EBICS |
|---|---|---|---|---|---|
| `0x83106300` | 4 | `05 0B 01 01` | ~1 ms* | b0 tryb, b1 bateria(kreski), b2 błąd/stan, b3 aktywny | zweryf. |
| `0x83106301` | 8 | `94 08 00 7A 0D 00 E7 03` | ~5 ms* | b3-4 LE=0x0D7A, b6-7 LE=0x03E7 (napięcie 999) | częściowo |
| `0x83106302` | 5 | `A8 00 7A 0D 00` | ~5 ms* | b0-1 LE trip(0.01 km=1.68), b2-3 LE odometr(0.1 km=345) | ❌ BRAK |
| `0x83106303` | 1 | `0A` | ~250 ms | b0 = auto-off [min] (=10) | RX jest, TX zweryf. |
| `0x83106304` | 4 | `05 03 04 04` | ~5 ms* | stałe (config) | ? |
| `0x82F83000` | 4 | `00/01/02…` | **10 s (potw.)** | b0 inkrement licznika sesji | ❌ BRAK |
| `0x82F8320F` | 8 | `01 00…` | ~30 s | b0=0x01 (NIE mechanizm auto-off) | jest |
| `0x82FF1200` | 1 | `00` | ~3-5 s | watchdog HMI (HMI→sterownik) | RX |
| `0x821C/D/E` | — | stringi | na żądanie | Info/Para multiframe → **target=3** | `send_info_multiframe_821x()` |

\* Okresy ~1-5 ms z `CAN_PROTOCOL_REFERENCE.md §2` (logi CAN kompresowane — nie oddają
okresu ramek szybkich). Okresy wolne (10 s, 30 s) potwierdzone z `log-2026-05-24`.

Info/tożsamość: `CR X30P.250.FC 2.1` (`0x6000`), `FAKE TAXI 20260522w1` (`0x6001`),
`CR X30P.250.FC` (`0x6002`), `MMG532.250.CF3YA120681` (`0x6003`), Para0/1/2
(`0x6010/6011/6012`) — dokładne bajty z `aa55_raw` + `CAN_PROTOCOL_REFERENCE.md §3-4`.

---

## Plan krok po kroku (bez błądzenia)

### KROK 0 — Rozstrzygnij Wariant A vs B (decyzja z użytkownikiem)
Nie pisać kodu przed odpowiedzią. Brak odpowiedzi → domyślnie Wariant B, zaznaczyć w commicie.

### KROK 1 — Zbuduj tabelę cykliki z wzorca (zweryfikowane one-linery)
```bash
L="C:/snapshot/bafang_canable_pro/logs/log-2026-05-24-15-38-56-n0.log"
grep -oE 'ID:[0-9A-F]+' "$L" | sort | uniq -c | sort -rn          # jakie ID
grep 'ID:83106302' "$L" | grep -oE 'Data:[0-9A-F ]+' | sort -u    # payloady
awk -v id="ID:82F83000" '{for(i=1;i<=NF;i++){if($i~/^[0-9]{9,}$/)ts=$i; if($i==id)print ts}}' "$L" \
 | awk 'NR>1{d=$1-p;if(d>0)print d}{p=$1}' | sort -n | awk '{a[n++]=$1}END{print "median",a[int(n/2)]"us"}'
```
Okresy szybkich ramek → `CAN_PROTOCOL_REFERENCE.md §2`. Narzędzie: `documentation/Logaufbereitung.py`.

### KROK 2 — Mapowanie na kod (kotwice zweryfikowane)
- `src/main.c:554–586` — harmonogram TX: `t3100_counter`(10 ms), `t0204_counter`(25 ms),
  slow-loop 40 ms → `hb_tick`(480 ms), `speed_tick`(280 ms,0x3201), `cad_tick`(1480 ms,0x3200),
  `misc_tick`(0x3205), `s202_tick`(120 ms,0x3202).
- `src/CAN_Display.c`: `sendCAN_status_broadcast()` L687 (`hb_efid={0x02FF1200,0x02F8320F,0x02F83000}`),
  `sendCAN_0204()` L540, `sendCAN_3100()` L520, `sendCAN_Poll()` L586, `sendCAN_3202()` L570,
  `send_info_multiframe_821x()` L108 (target=3), `send_multiframe()` L825,
  `send_status_frame_851x()` L96 (zaślepka). RX auto-off `0x6303` L392;
  `0x6400/0x6401` L458–487; `sendCAN_Tx()` L714 (6000/6001/6002 „silent", 6003 L800).
- Auto-off: `MP.auto_off_minutes` (`main.c:451`, `parser.c:157`), idle `s_is_idle`
  (`main.c:576–582`) → byte[6] w `sendCAN_0204`.

### KROK 3 — Implementacja luk (priorytet)
Nowa ramka = funkcja `sendCAN_XXXX()` w `CAN_Display.c` (wzór jak istniejące, ten sam
`can_tx()`), deklaracja w `inc/CAN_Display.h`, wywołanie z licznikiem w `main.c:554–586`.

**Wspólne (oba warianty):**
1. `0x83106302` odometr/trip — `sendCAN_6302()`; b0-1 LE trip(0.01 km), b2-3 LE odometr(0.1 km).
   Źródło: `distance_since_startup` + zmienna odometru.
2. `0x82F83000` licznik sesji — potwierdź/napraw inkrement b0 co 10 s.
3. `0x83106303` auto-off TX — EBICS ma wysyłać `auto_off_minutes` co ~250 ms (RX jest, TX zweryf.).

---

## Wariant A — pełna ścieżka „bądź fake taxi 1:1" (jeśli wybrany)

Cel: ruch CAN EBICS nieodróżnialny od fabrycznego M820. EBICS przestaje nadawać własną
rodzinę (`0x80010204/0x80010203/0x81F83100`) i nadaje rodzinę fabryczną.

**A1. Wygaś rodzinę EBICS** (`main.c:554–556`, `main.c:1926`): za flagą
`HMI_VARIANT_FAKETAXI` (`inc/config.h`) wyłącz `sendCAN_3100()` (0x81F83100),
`sendCAN_0204()` (0x80010204) i ramkę debug `0x0203`.

**A2. Dodaj nadajniki rodziny fabrycznej** (`CAN_Display.c`, wzór jak `sendCAN_*`):
| Funkcja | ID | DLC | Payload |
|---|---|---|---|
| `sendCAN_6300()` | `0x83106300` | 4 | b0 tryb, b1 bateria(kreski z SOC), b2 stan/błąd, b3 aktywny |
| `sendCAN_6301()` | `0x83106301` | 8 | b3-4 LE (0x0D7A), b6-7 LE napięcie (`Battery_Voltage`) |
| `sendCAN_6302()` | `0x83106302` | 5 | b0-1 LE trip 0.01 km, b2-3 LE odometr 0.1 km |
| `sendCAN_6304()` | `0x83106304` | 4 | stałe `05 03 04 04` |
- `0x82F83000` (10 s) i `0x82F8320F` (30 s) — w `hb_efid` (`L693`); potwierdź inkrement/okresy.
- `0x83106303` (auto-off `0A`) — wysyłane co ~250 ms z `MP.auto_off_minutes`.

**A3. Info/tożsamość** przez `0x821C/D/E`→target=3 — już `send_info_multiframe_821x()` (`L108`).
Stringi: `CR X30P.250.FC 2.1` / `FAKE TAXI 20260522w1` (lub wersja EBICS wg decyzji) /
`CR X30P.250.FC` / `MMG532.250.CF3YA120681`; Para0/1/2 z `aa55_raw`.

**A4. Harmonogram** (`main.c:558–586`): wepnij nadajniki z okresami z tabeli.

**A5. Warunek bezpieczeństwa (MUST przed A):** zgrać log EBICS↔HMI i potwierdzić, że HMI
czyta telemetrię z `0x831063xx` (nie z `0x80010204`). Potwierdzić z logu fabrycznego,
**który węzeł** nadaje `0x831063xx` (dekod bitowy daje src=3=HMI → jeśli tak, to NIE
sterownik je nadaje — wtedy Wariant A wymaga rewizji). Bez tego grozi regresją telemetrii.

---

## Rodzina EBICS — co realnie niosą ramki (zdekodowane z kodu)

Zweryfikowane w `CAN_Display.c` / `main.c`. Ważne przy decyzji A vs B: `0x3100` i `0x0204`
to działająca dziś telemetria HMI — nie wygaszać bez przeniesienia danych do `0x831063xx`.

### `0x81F83100` — emulacja czujnika momentu (`sendCAN_3100()` L520, co 10 ms, DLC 4, src=1)
| Bajt | Zawartość |
|---|---|
| 0-1 | `torque_on_crank` (16-bit LE) — moment na korbie |
| 2 | `cadence` — kadencja |
| 3 | licznik rolkowy `ctr++` |

### `0x80010204` — status „M820" (`sendCAN_0204()` L540, co 25 ms, DLC 8, raw 0x0204)
| Bajt | Zawartość |
|---|---|
| 0-1 | `torque_on_crank` signed 16-bit LE |
| 2 | 0 |
| 3 | wolny licznik (`ctr>>1`) |
| 4 | **0x00** ⚠️ komentarz mówi „prąd silnika LE", ale kod wpisuje 0 — **BUG do naprawy** |
| 5 | 0x00 (jw.) |
| 6 | flaga aktywności/auto-off: `is_idle ? 0 : (assist_level>0 lub walk_request ? 1 : 0)` |
| 7 | 0 |

### `0x80010203` — debug FOC (`print_debug_on_CAN()` main.c:1922, wolna pętla, DLC 8, big-endian pary)
| Bajty | Zmienna | Znaczenie |
|---|---|---|
| 0-1 | `MS.Battery_Current` | prąd baterii |
| 2-3 | `MS.u_q` | napięcie osi q |
| 4-5 | `MS.i_q` | prąd osi q |
| 6-7 | `MS.i_q_setpoint` | zadany prąd q |

Uwaga: `0x0203` to czysty debug — dla HMI bezużyteczny, można wygasić bez skutków.

### TODO niezależne od wariantu
- **Napraw `0x80010204` [4-5]:** wpisać `Motor_Current` (signed 16-bit LE, ujemny=napęd)
  zamiast `0x00`, albo poprawić komentarz jeśli celowo puste. (`CAN_Display.c:559-560`)

---

## Weryfikacja (end-to-end)
1. Wgraj build, podłącz CANable jako sniffer na magistrali EBICS↔HMI.
2. Zgraj log: włączenie → Info → Ustawienia → postój (auto-off).
3. Diff z fabrycznym wzorcem (`log-2026-05-24`) tymi samymi one-linerami: każdy cykliczny
   ID docelowego wariantu obecny, okresy wolne (10 s/30 s) zgodne, stringi Info (ASCII) zgodne.
4. Naocznie na HMI: wersja/HW/model/SN wypełnione, prędkość max + obwód koła w Ustawieniach,
   telemetria realtime, auto-off wyłącza po `auto_off_minutes`.
5. Sukces: brak pustych pól + zestaw i okresy ramek zgodne z wybranym wariantem.

## Ryzyka
- Zły dialekt (KROK 0) → HMI ignoruje ramki. Rozstrzygnąć logiem EBICS↔HMI.
- Logi CAN kompresują powtórzenia → nie stroić okresów szybkich na ich podstawie; użyć doc.
- Fabryczne logi krótkie/rzadkie → stringi Info brać z `aa55_raw` (pełne bloki).
- Rozjazd repo vs buildy 0.0101–0.0108 → najpierw ustalić który build jest na HW.
