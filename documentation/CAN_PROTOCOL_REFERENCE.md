# Bafang M820 — CAN Protocol Reference

> Źródło: analiza logu `log-2026-06-24-17-51-19-n0.log` (17:51:19 – 18:00:24, ~9 min).
> **WAŻNE: Log pochodzi z FABRYCZNEGO oprogramowania Bafang** (nie z EBiCS) — cel analizy to
> zrozumienie jak fabryczny FW działa żeby zaimplementować brakujące funkcje w EBiCS.
> Sesja: włączenie → zakładka info firmware → zmiana auto-off 9→8 min → automatyczne wyłączenie.
> Sterownik: fabryczny FW Bafang ("FAKE TAXI 20260522w1"). HMI: CR X30P (oryginalne).
> CAN 250 kbps, extended 29-bit IDs.

---

## 1. Format ID (zweryfikowany — build 0.0091)

Bafang używa standardowego 29-bit extended CAN ID z EFF-bit (bit 31) ustawionym.
W logach: `ID & 0x1FFFFFFF` = faktyczny 29-bit EFID.

### Kodowanie bitowe (zweryfikowane bajt po bajcie)

```
efid = (source << 24) | (target << 19) | (operation << 16) | command

  bit 28..24  = source    (5 bitów, węzeł nadający)
  bit 23..19  = target    (5 bitów, węzeł docelowy)
  bit 18..16  = operation (3 bity, typ operacji)
  bit 15..0   = command   (16 bitów)
```

### Węzły (zweryfikowane z logu)
| Node | Znaczenie |
|------|-----------|
| 2    | Sterownik (EBiCS / fabryczny M820) |
| 3    | HMI — kanał TX (query) i RX dla READ response |
| 4    | HMI — kanał dodatkowy (tgt=4 queries — sterownik odpowiada jak na tgt=2) |
| 5    | HMI — port RX tylko dla WRITE ACK |
| 31 (0x1F) | Broadcast — wszystkie węzły |

### Operacje
| Kod | Stała        | Opis |
|-----|--------------|------|
| 0   | WRITE_CMD    | Zapis parametru |
| 1   | READ_CMD     | Żądanie odczytu (DLC=0 lub z danymi) |
| 2   | NORMAL_ACK   | Potwierdzenie jednoramkowe |
| 4   | LONG_START   | Pierwsza ramka multiframe (DLC=1, data=total_length) |
| 5   | LONG_TRANG   | Środkowe ramki multiframe (DLC=8) |
| 6   | LONG_END     | Ostatnia ramka multiframe (DLC=1..8) |

### Weryfikacja z logiem (przykłady)

```
0x83116001 → 0x03116001
  source = 0x03 & 0x1F = 3       (HMI)
  target = (0x03116001>>19)&0x1F = 98&31 = 2   (sterownik)
  op     = 0x0311 & 0x07 = 1     (READ_CMD)
  cmd    = 0x6001               → HMI pyta sterownik o wersję FW ✓

0x821C6001 → 0x021C6001
  source = 0x02 & 0x1F = 2       (sterownik)
  target = (0x021C6001>>19)&0x1F = 67&31 = 3   (HMI read channel)
  op     = 0x021C & 0x07 = 4     (LONG_START)
  cmd    = 0x6001               → sterownik zaczyna multiframe odpowiedź →3 ✓

0x822A6300 → 0x022A6300
  source = 0x02 & 0x1F = 2       (sterownik)
  target = (0x022A6300>>19)&0x1F = 69&31 = 5   (HMI write port)
  op     = 0x022A & 0x07 = 2     (NORMAL_ACK)
  cmd    = 0x6300               → sterownik ACKuje WRITE 0x6300 od HMI →5 ✓

0x83106300 → 0x03106300
  source = 0x03 & 0x1F = 3       (HMI)
  target = (0x03106300>>19)&0x1F = 98&31 = 2   (sterownik)
  op     = 0x0310 & 0x07 = 0     (WRITE_CMD)
  cmd    = 0x6300               → HMI wysyła dane operacyjne do sterownika ✓
```

### Split-addressing HMI (kluczowe odkrycie — build 0.0091)

HMI używa **dwóch różnych docelowych węzłów** jako odbiornik:
- **target=3**: odbiera READ responses (multiframe ze stringami, danymi info)
- **target=5**: odbiera WRITE ACKs (potwierdzenia zapisów komend)

EBiCS przed 0.0091 wysyłał READ responses do target=5 → HMI filtr CAN odrzucał →
zakładka Controller Info była pusta. Naprawione w 0.0091.

### Znane prefiksy ID (zdekodowane)
| ID (z EFF-bit) | Kierunek        | src→tgt | op | Znaczenie |
|----------------|-----------------|---------|-----|-----------|
| `0x8310XXXX`   | **HMI→Sterownik** | 3→2   | 0  | HMI WRITE: dane operacyjne (0x6300-0x6304) |
| `0x8311XXXX`   | HMI→Sterownik   | 3→2    | 1  | HMI READ query (DLC=0) — żądanie stringa info |
| `0x821CXXXX`   | Sterownik→HMI   | 2→3    | 4  | LONG_START — odpowiedź READ (do target=3) |
| `0x821DXXXX`   | Sterownik→HMI   | 2→3    | 5  | LONG_TRANG — środek multiframe |
| `0x821EXXXX`   | Sterownik→HMI   | 2→3    | 6  | LONG_END — koniec multiframe |
| `0x822AXXXX`   | Sterownik→HMI   | 2→5    | 2  | NORMAL_ACK dla WRITE (do target=5) |
| `0x82F8XXXX`   | Sterownik→Broad | 2→31   | 0  | Broadcast sterownika (0x320F, 0x3000) |
| `0x82FFXXXX`   | Sterownik→Broad | 2→31   | 0  | Broadcast sterownika (0x1200 keepalive?) |
| `0x20000004`   | System          | —      | —  | Marker boot systemu |

> **UWAGA:** Dokumentacja sekcji 2-3 poniżej używa starych oznaczeń kierunków opartych na obserwacji (przed zdekodowaniem ID). Kierunki były mylnie przypisane. Tabela powyżej jest miarodajna.

---

## 2. Ramki wysyłane przez STEROWNIK → HMI (broadcast, ciągłe)

### `0x83106300` — główny status (co ~1 ms)
```
Bajt 0: 0x05 = stan(?) lub poziom wspomagania
Bajt 1: 0x0B = poziom baterii (0–15 kresek? → 11 = ~73%)
Bajt 2: 0x00 = błąd lub tryb (0 = normalny)
Bajt 3: 0x01 = flaga aktywności (1 = aktywny)
```
**Uwaga:** To NAJCZĘSTSZA ramka w logu (>10 000 powtórzeń). Zmiana bajtu 3 na 0x00
powoduje inny stan, widziany jeden raz podczas wstępnego startu (`Data:05 00 00 01`).

**STATUS W EBICS:** Sterownik wysyła podobną ramkę, ale mapowanie pól nie zostało zweryfikowane.

---

### `0x83106301` — status rozszerzony (co ~5 ms)
```
Bajt 0: 0x1D = temperatura sterownika (0x1D = 29°C ✓ — potwierdzone z innymi danymi)
Bajt 1: 0x0C = nieznane (12)
Bajt 2: 0x00
Bajt 3: 0xD2 |
Bajt 4: 0x30 |→ little-endian 0x30D2 = 12498 → PRAWDOPODOBNIE kilometrówka (0.1 km = 1249.8 km)
Bajt 5: 0x00
Bajt 6: 0xE7 |
Bajt 7: 0x03 |→ little-endian 0x03E7 = 999 → PRAWDOPODOBNIE napięcie baterii (skala nieznana)
```
**STATUS W EBICS:** Temperatura jest wysyłana. Odometru i napięcia w tej ramce — brak lub niezweryfikowane.

---

### `0x83106302` — odometer / trip (co ~5 ms)
```
Bajt 0: 0x49 |
Bajt 1: 0x01 |→ little-endian 0x0149 = 329 → PRAWDOPODOBNIE dystans od uruchomienia (0.01 km = 3.29 km)
Bajt 2: 0xD2 |
Bajt 3: 0x30 |→ little-endian 0x30D2 = 12498 → PRAWDOPODOBNIE łączny odometr (0.1 km = 1249.8 km)
Bajt 4: 0x00
```
**STATUS W EBICS:** ❌ NIE ZAIMPLEMENTOWANE. EBiCS liczy `distance_since_startup` (w m), ale
nie wysyła tej ramki z odometrem. HMI prawdopodobnie wyświetla tu dane z oryginalnego FW.

---

### `0x83106303` — timeout auto-off (co ~250 ms)
```
Bajt 0: wartość w minutach (0x09=9, 0x08=8)
```
**KLUCZOWE ODKRYCIE:** To jest **mechanizm auto-off**.
- Sterownik czyta wartość timeout z parametrów HMI (skąd dokładnie w Para1 — nieznane, prawdopodobnie Para1[63] lub Para0)
- Sterownik rozgłasza tę wartość w 0x83106303
- HMI odlicza własny czas bezczynności i wyłącza zasilanie gdy `idle_time ≥ wartość z 0x83106303`
- Brak tej ramki = HMI nie wie o timeoucie = **auto-off nie działa**

Zmiana: 17:51:54 (0x09→0x08) = moment gdy użytkownik zmienił 9→8 min w HMI.
Wyłączenie nastąpiło 18:00:04 = 8 min 10 s po zmianie (dokładność ✓).

**STATUS W EBICS:** ⚠️ SPRAWDZIĆ czy EBiCS wysyła tę ramkę. Jeśli nie — auto-off nie działało z powodu braku tej ramki, a nie z powodu 0x320F bajtu 0.

---

### `0x83106304` — konfiguracja dodatkowa (co ~5 ms)
```
Bajt 0: 0x05
Bajt 1: 0x03
Bajt 2: 0x04
Bajt 3: 0x04
```
Wartości stałe przez cały czas trwania sesji. Znaczenie nieznane.

---

### `0x82F83000` — licznik sesji (co **10 s**)
```
Bajt 0: licznik 0x00→0x33 (0→51 kroków × 10 s = 510 s = 8.5 min ✓)
Bajty 1–3: 0x00
```
Inkrementuje co 10 sekund. Przy starcie: 0x00, 10 s → 0x01, 20 s → 0x02 itd.
Prawdopodobnie licznik czasu bezczynności widoczny przez HMI.

**STATUS W EBICS:** ❌ NIE ZAIMPLEMENTOWANE.

---

### `0x82F8320F` — broadcast aktywności (co ~30 s)
```
Bajt 0: 0x01 = aktywny (PRZEZ CAŁY CZAS SESJI — NIGDY nie zmienił się na 0x00)
Bajty 1–7: 0x00
```
**WAŻNE:** Bajt 0 pozostał `0x01` przez cały czas sesji, aż do wyłączenia.
Auto-off zadziałało BEZ zmiany tego bajtu na 0x00.

**Wniosek:** Hipoteza o bajcie 0x320F (wysyłanie 0x00 przy bezczynności)
jest **PRAWDOPODOBNIE BŁĘDNA** jako mechanizm auto-off. Ten bajt może mieć inne znaczenie.
Implementacja w EBiCS (`is_idle → 0x00`) prawdopodobnie nie ma wpływu na auto-off.

**STATUS W EBICS:** Zaimplementowane błędnie (zmiana na 0x00 przy idle). Nie powoduje szkód,
ale nie jest to mechanizm auto-off. Prawdziwy mechanizm = 0x83106303.

---

## 3. Ramki HMI → Sterownik

### `0x82FF1200` — watchdog HMI (co ~3–5 s)
```
Bajt 0: 0x00
```
Keep-alive / heartbeat od HMI. Jeśli nie pojawia się → HMI odłączone.

---

### `0x821A62D9` — krótka komenda HMI (jednorazowo przy starcie)
```
Bajt 0: 0x07
Bajt 1: 0x00
```
Znaczenie nieznane. Pojawia się raz przy inicjalizacji połączenia.

---

### `0x821B6003` — mini-blok konfiguracyjny (jednorazowo)
```
Bajt 0: 0x01
Bajt 1: 0x00
Bajt 2: 0x02
Bajt 3: 0x06
```
Wysyłany przed blokiem Para1. Możliwy nagłówek/wersja protokołu.

---

### Sekwencja info-query przy starcie (0x8311XXX → 0x821CXXX + 0x821DXXX/0x821EXXX)

Sterownik odpytuje HMI o kolejne bloki informacyjne. Format:
```
1. Sterownik: 0x8311XXXX  DLC=0  (request: "daj mi blok XXXX")
2. HMI:       0x821CXXXX  DLC=1  Data: [rozmiar]  (ACK: "dam N bajtów")
3. HMI:       0x821D0000  DLC=8  Data: [bajty 0–7]
4. HMI:       0x821D0001  DLC=8  Data: [bajty 8–15]
               ...
N. HMI:       0x821EXXXX  DLC=X  Data: [ostatnie bajty]
```

**Kierunek:** HMI PYTA sterownik (0x8311XXXX DLC=0), sterownik ODPOWIADA z danymi.
Sterownik jest źródłem wszystkich stringów poniżej.

#### `0x6000` — identyfikator sprzętu sterownika (18 bajtów)
```
ASCII: "CR X30P.250.FC 2.1"
       ─────── ─── ── ───
       Rodzina Seria FC  ver
```
Fabryczny string modelu sterownika. Widoczny w zakładce "Info" HMI.
**EBiCS:** Wysyłany identycznie z fabrycznym (hardcoded w CAN_Display.c:444).

#### `0x6001` — wersja firmware sterownika (20 bajtów, padding do 20)
```
Fabryczny FW: "FAKE TAXI 20260522w1"   ← nazwa roweru testowego + data buildu
EBiCS:        "EBICS 0.0081           " ← wersja EBiCS padowana spacjami do 20 znaków
```
**TO** jest pole które użytkownik widzi w HMI jako "wersja softu sterownika".
**EBiCS:** `sprintf(tx_data, "EBICS %s", EBICS_BUILD_VERSION)` → CAN_Display.c:451.

#### `0x6002` — skrót modelu sterownika (14 bajtów)
```
ASCII: "CR X30P.250.FC"
```
Skrócona wersja 0x6000.
**EBiCS:** Wysyłany identycznie z fabrycznym (CAN_Display.c:459).

#### `0x6003` — identyfikator seryjny (22 bajty)
```
ASCII: "MMG532.250.CF3YA120681"
       ────── ─── ────── ─────
       typ    seria  numer
```
**EBiCS:** Wysyłany identycznie z fabrycznym (CAN_Display.c:540).

---

## 4. Upload parametrów (HMI → Sterownik)

### Para1 — główne parametry (64 bajty, przez 0x6011)
Dane z logu: `24 0F 2F B8 0B E4 0C 10 | 27 00 19 0F 2F 00 00 30 | 02 1C 01 07 01 01 FC 0D | 00 00 00 00 00 00 00 00 | 00 00 0E 20 01 0A 04 02 | 14 19 1E 23 2A 32 3C 46 | 64 32 32 32 32 32 32 32 | 32 32 01 00 00 00 FF B5`

Dekodowanie znanych pól (na podstawie `src/parser.c`):
```
[0]  = 0x28 = 40    system_voltage (V) ✓
[1]  = 0x0F = 15    battery_current_max (A) ✓
[2]  = 0x2F = 47    max_voltage
[3]  = 0xB8         voltage_min (LSB)
[4]  = 0x0B         voltage_min (MSB) → (0x0BB8)/17 = 176 (w jednostkach ADC×17)
[7]  = 0x00         battery_capacity_mah (LSB)
[8]  = 0x27 = 39    battery_capacity_mah (MSB) → 0x2700 = 9984 mAh
[10] = 0x19 = 25    limp_soc_limit = 25%
[11] = 0x0F = 15    limp_soc_limit_stage2 = 15%
[12] = 0x2F = 47    Cadence_exponent (domyślnie 10, tu 47 — weryfikować!)
[14] = 0x00         legalflag = 0 (tryb off-road) ✓
[20] = 0x01         pulses_per_revolution = 1 ✓
[36] = 0x01         walk_assist_current = 1% (stara wartość, przed zmianą na 25%!)
[37] = 0x0A = 10    Override_Duration = 10×40 = 400 ticks
[38] = 0x04 = 4     PAS_timeout = 4×400 = 1600 ticks
[39] = 0x02 = 2     ramp_end = 11250/2 = 5625 ticks
[57] = 0x32 = 50    assist_settings[5][1] = speed limit level 5 = 50%
[60] = 0x00         walk_assist_speed LSB = 0 (brak konfiguracji → fallback 600 = 6.0 km/h)
[61] = 0x00         walk_assist_speed MSB
[62] = 0xFF         limp_soc_limit = 0xFF = DISABLED ✓
```

**WAŻNE:** Para1[36] = 0x01 (walk_assist_current = 1%) zamiast 25% — bo HMI przechowuje STARE
parametry wysłane zanim zmieniono domyślną wartość. Potrzebny nowy upload z Canable żeby odświeżyć.

### Para2 — profile wspomagania (64 bajty, przez 0x6012)
Zawiera `assist_profile[6][7]` (30 bajtów) + ext_boost_duration/strength.
Dekodowanie szczegółowe pomijane (nie kluczowe dla auto-off).

---

## 5. Sekwencja startu (od włączenia)

```
t=0.0s  [20000004] Data:00 0C 00 00 00 00 00 00  — marker boot systemu
t=0.0s  [82FF1200] Data:00                        — HMI watchdog (1. po boo)
t=0.0s  [82F8320F] Data:01 00 00 00 00 00 00 00   — sterownik aktywny
t=4.0s  [8311F203] DLC=0                          — sterownik pyta HMI (?)
t=4.0s  [83116000] DLC=0                          — request: firmware string 1
t=4.0s  [83106300] Data:05 0B 00 01               — 1. broadcast statusu
t=4.0s  [821A62D9] Data:07 00                     — inicjalizacja HMI
t=4.0s  [831162D9] DLC=0
t=4.0s  [821C6000] Data:12  +  [821D...] "CR X30P.250.FC 2.1"   (18 B)
t=4.0s  [83106302] Data:49 01 D2 30 00            — 1. odometer
t=4.0s  [83106303] Data:09                        — auto-off timeout = 9 min
t=4.0s  [821D0000-821E0002] "CR X30P.250.FC 2.1" (duplikaty, 3 razy)
t=5.0s  [83116001] DLC=0  +  [821C6001] Data:14  +  [821D...]  "FAKE TAXI 20260522w1"  (20 B)
t=6.0s  [83116002] DLC=0  +  [821C6002] Data:0E  +  "CR X30P.250.FC"  (14 B)
t=6.0s  [83116003] DLC=0  +  [821C6003] Data:16  +  "MMG532.250.CF3YA120681"  (22 B)
t=7.0s  [83116010] DLC=0  +  [821B6003] Data:01 00 02 06    — mini config block
t=8.0s  [83116011] DLC=0  +  [821C6011] Data:40  +  8×[821D...] Para1 (64 B)
t=8.0s  [83116012] DLC=0  +  [821C6012] Data:40  +  8×[821D...] Para2 (64 B)
t=9.0s  Normalny broadcast (0x83106300 dominuje)
t=10s   [82F83000] Data:01 00 00 00               — licznik +1
```

---

## 6. Mechanizm auto-off — jak działa NAPRAWDĘ

```
1. HMI ma własny timer (ustawiany przez użytkownika w menu)
2. Przy starcie i co ~250 ms sterownik rozgłasza 0x83106303 z wartością timeoutu (minuty)
3. HMI potwierdza tę wartość i używa jej jako punkt odniesienia dla swojego licznika
4. HMI liczy czas bezczynności (brak prędkości, brak pedałowania, brak przycisków)
5. Gdy idle_time ≥ timeout → HMI odcina zasilanie
6. Brak pożegnalnej ramki CAN — magistrala po prostu milknie
```

**KRYTYCZNE dla EBiCS:**
- Sterownik MUSI wysyłać `0x83106303` z wartością timeout z Para1 (bajt nieznany — do ustalenia)
- Bez tej ramki HMI albo czeka w nieskończoność albo używa sztywnej wartości
- Bajt 0 ramki `0x82F8320F` (aktywny/idle) **NIE jest mechanizmem auto-off** — pozostaje 0x01 przez cały czas trwania sesji aż do wyłączenia

---

## 7. Co EBiCS ma, czego nie ma lub ma źle

| Funkcja | ID | Status (0.0092) | Priorytet |
|---|---|---|---|
| ACK na WRITE 0x6300-0x6304 (HMI→sterownik) | 0x822A630x | ✓ Poprawne — target=5 ✓ | — |
| READ responses (0x6000-0x6003) do target=3 | multiframe 0x821C/D/E | ✅ NAPRAWIONE 0.0091 — wcześniej target=5 | — |
| FW version string 0x6001 jako multiframe | 0x821C6001+ | ✅ NAPRAWIONE 0.0092 — padding do 20 B (LONG_END na frame#2) | — |
| Model string 0x6002 jako multiframe | 0x821C6002+ | ✅ NAPRAWIONE 0.0091 — wcześniej single-frame | — |
| Auto-off: 0x0204 bajt[6] 0x01→0x00 przy idle | 0x80010204 | ✅ NAPRAWIONE 0.0091 — timer is_idle | — |
| Koła (0x62D9) op=NORMAL_ACK | 0x821A62D9 | ✅ NAPRAWIONE 0.0092 — wcześniej op=WRITE(0) | — |
| Speed limit (0xF203) op=NORMAL_ACK | 0x821AF203 | ✅ NAPRAWIONE 0.0092 — wcześniej op=WRITE_CMD | — |
| Temperatura | 0x83106301 bajt 0 | ✓ Wysyłane | — |
| Odpowiedź na tgt=4 queries (0x6000-0x6003) | — | ✓ Zaimplementowane | — |
| **Licznik sesji** | **0x82F83000** | **❌ BRAKUJE** | Średni |
| Odometr / trip distance | 0x83106302 | ❌ BRAKUJE | Średni |
| Broadcast aktywności | 0x82F8320F | ⚠️ Zmiana bajtu[0] na 0x00 nie jest mechanizmem auto-off (wg logu) | Niski |
| Napięcie baterii w 0x83106301 | bajty 6-7 | ❓ Sprawdzić skalę | Niski |

---

## 8. Identyfikacja nieznanego bajtu auto-off w Para1

Wartość 0x09 (9 min) musi być gdzieś w Para1[0..63]. Ze znanych mapowań:
- Para1[37] = Override_Duration = 0x0A (10 × 40 ticks) — nie pasuje
- Para1[38] = PAS_timeout = 0x04 — nie pasuje
- Para1[56] = 0x32 = 50 — nie pasuje
- Para1[57] = 0x32 = 50 (speed limit L5) — nie pasuje
- Para1[63] = 0xB5 = 181 — jeśli to checksum, nie jest auto-off

Możliwe: auto-off jest w **Para0** (inne bajty niż ride mode i TQO_threshold), lub
w jednym z niezmapowanych pól Para1[13], Para1[15], Para1[16..22], Para1[32..33].

**TODO do kolejnej sesji:** Sprawdzić log gdzie użytkownik zmienia TYLKO wartość auto-off
w HMI i przesyła ponownie → zobaczyć który bajt w Para1/Para0 zmienił się (0x09→0x08).

---

## 9. Inne obserwacje

### Ramki przy wyłączeniu (18:00:24)
Przy auto-off pojawiają się po raz drugi ramki z timestampami jak ze startu:
```
[20000004], [83116000], [83116001], ... [83116012], [83216XXX]
```
Są to ZDUPLIKOWANE ramki z loggera (narzędzie zbiera bufor i wypisuje powtórzenia).
Nie jest to nowy boot — to koniec logu.

### Zdublowane ramki w logu
Format logu `(Repeated N times - same data)` oznacza że logger kompresuje powtórzenia
tego samego timestamp/ID/data. Przy końcu logu zbiera wszystkie "unreported" powtórzenia.

### `0x83096000`, `0x83096001` — nieznane
DLC=0, pojawiają się rzadko. Mogą być ack-ami lub rozgłoszeniami z innego urządzenia
(drugi kontroler? BMS?).

### `0x83216XXX` — nieznane, przy starcie/wyłączeniu
Pojawia się 33 razy (x4 różne pod-komendy) tylko przy wyłączeniu logu. Prawdopodobnie
część sekwencji handshake przy ponownym podłączeniu loggera do magistrali.

---

## 10. Zmiany i wnioski — build 0.0092 (2026-06-25)

### Problem: SW version puste, koła puste, speed limit puste (build 0.0091)

Po wgraniu 0.0091 HMI zaczął pokazywać **HW: CR X30** (z 0x6000, 18 B), ale:
- **SW**: puste — `0x6001` wysyłany jako 12 bajtów (2 ramki danych), HMI wymaga ≥3 ramek
- **Koła**: puste — `0x62D9` odpowiedź z `op=WRITE(0)`, HMI wymaga `NORMAL_ACK(2)`
- **Speed limit**: puste — `0xF203` odpowiedź z `op=WRITE_CMD`, HMI wymaga `NORMAL_ACK`

#### Reguła LONG_END frame# (odkryta z logu `18-37-52` po flashu 0.0091)

```
0x821C6000  Data:12 (18B) → frame 0,1 → 0x821E0002 → SHOWS ✓
0x821C6001  Data:0C (12B) → frame 0   → 0x821E0001 → BLANK ✗
0x821C6003  Data:16 (22B) → frame 0,1 → 0x821E0002 → SHOWS ✓
```

HMI ignoruje multiframe gdzie `LONG_END frame# = 1`. Minimalna długość dla `frame#=2` = **17 bajtów**.
Fake taxi wysyłał 0x6001 jako dokładnie **20 bajtów** ("FAKE TAXI 20260522w1").

#### Reguła operation code dla READ responses

```
0x62D9 (koła): EBiCS 0.0091 → 0x821862D9 (op=WRITE=0)   ← BŁĄD
               Fake taxi     → 0x821A62D9 (op=NORMAL_ACK=2) ✓

0xF203 (speed+wheel): EBiCS → 0x8218F203 (op=WRITE_CMD=0) ← BŁĄD
                      Fix    → 0x821AF203 (op=NORMAL_ACK=2) ✓
```

### Naprawy wprowadzone w 0.0092

| Lokalizacja | Zmiana |
|---|---|
| `sendCAN_Tx case 0x6001` | Padding do 20 B spacjami po sprintf → LONG_END trafia na frame#2 |
| `tgt=4 handler, 0x6001` | Identyczny padding |
| `sendCAN_Tx case 0x62D9` | `op=0` → `op=NORMAL_ACK` |
| `0xF203 direct handler` | `op=WRITE_CMD` → `op=NORMAL_ACK` |

### Weryfikacja w logu 0.0092

```
ID:821C6001  DLC:1  Data:14  (= 20 dec)     ← LONG_START 20 bajtów
ID:821D0000  DLC:8  Data:"EBiCS 0."
ID:821D0001  DLC:8  Data:"0092    "          ← 8 bajtów (z paddingiem)
ID:821E0002  DLC:4  Data:"    "              ← LONG_END frame#2 → HMI wyświetli
ID:821A62D9  DLC:2  Data:04 00               ← NORMAL_ACK (było WRITE)
ID:821AF203  DLC:6  Data:80 0C 41 31 02 08   ← NORMAL_ACK (było WRITE_CMD)
```

---

## 11. Zmiany i wnioski — build 0.0091 (2026-06-25)

### Problem główny: Controller Info tab pusty (builds 0.0086–0.0090)

**Objaw:** Zakładka Controller Info w HMI pusta — brak nazwy FW, brak wersji, brak modelu.

**Przyczyna źródłowa:** EBiCS wysyłał READ responses (`send_multiframe()`) do `target=5`
zamiast `target=3`. HMI posiada filtr CAN który przyjmuje READ responses WYŁĄCZNIE
na `target=3`. Ramki do `target=5` były odrzucane przez HMI bez żadnego ACK.

**Skąd błąd:** W 0.0090 dodano globalną zmienną `mf_src=0x02` i ustawiono target=5
("HMI port") we wszystkich wysyłkach multiframe. Logika była błędna — pomylono
kanał WRITE ACK (target=5) z kanałem READ response (target=3).

**Weryfikacja:** Zdekodowanie bit po bicie frame `0x821C6001` z logu fake taxi:
- source=2 (sterownik), target=3 (HMI read channel), op=4 (LONG_START) ✓

### Problem dodatkowy: 0x6001/0x6002 jako single-frame (0.0086–0.0090)

Fake taxi wysyła 0x6001 (20 bajtów) i 0x6002 (14 bajtów) jako multiframe.
EBiCS wysyłał je jako 8-bajtowe single-frame — HMI odrzucał z powodu złego formatu.

### Problem dodatkowy: Auto-off nie działał (0.0086–0.0090)

**Mechanizm auto-off (wg logu fake taxi):** HMI odlicza własny timer bezczynności
i wyłącza się. Ramka `0x0204` (bajt[6]=0x01/0x00) sygnalizuje sterownikowi że
napęd jest aktywny lub w spoczynku. Fabryczny M820 prawdopodobnie wstrzymuje
wysyłanie napięcia napędowego gdy bajt[6]=0x00, co HMI interpretuje jako
"bezpieczne wyłączenie".

**Błąd EBiCS:** bajt[6] był ustawiany na 0x01 gdy `assist_level > 0`, na 0x00 gdy
`assist_level == 0`. Problem: zatrzymanie roweru przy wybranym poziomie wspomagania
utrzymywało bajt[6]=0x01 na stałe → HMI nigdy nie widział sygnału idle.

**Naprawa:** bajt[6] sterowany przez timer `idle_ticks` z `main.c`. Po upływie
`auto_off_thresh` ticków bez ruchu → `is_idle=1` → bajt[6]=0x00.

### Naprawy wprowadzone w 0.0091

#### `src/CAN_Display.c`

| Lokalizacja | Zmiana |
|---|---|
| `send_multiframe()` — 3 miejsca | `target=5,source=mf_src` → `target=3,source=0x02` |
| `sendAcknoledge()` | Bez zmian — target=5 POPRAWNE dla WRITE ACKs |
| `sendCAN_Tx case 0x6001` | Single-frame → `send_multiframe(0x6001, "EBiCS 0.0091", len)` |
| `sendCAN_Tx case 0x6002` | Single-frame → `send_multiframe(0x6002, "CR X30P.250.FC", len)` |
| `sendCAN_Tx case 0x62D9` | target=5→3 |
| `sendCAN_Tx case 0x3203` | target=5→3 |
| `sendCAN_Tx case 0x6200` | target=5→3 |
| `if(cmd==0xF203)` | target=5→3, `can_message_transmit`→`can_tx()` |
| `if(cmd==0x6400)` | target=5→3, source ustalony na 0x02 |
| `if(cmd==0x6401)` | target=5→3, source ustalony na 0x02 |
| `if(tgt==4)` — cały blok | Przepisany: source=0x02, target=3, op=READ_CMD guard |
| `if(tgt==1)` — 2 efid | `5<<19` → `3<<19` |
| `sendCAN_0204()` | Nowy parametr `uint8_t is_idle`; bajt[6] = is_idle ? 0x00 : (assist>0 \|\| walk ? 0x01 : 0x00) |
| `mf_src` — zmienna globalna | Usunięta (była niepotrzebna) |

#### `src/main.c`

| Lokalizacja | Zmiana |
|---|---|
| Scope pliku | Dodano `static uint8_t s_is_idle=0` |
| Fast loop (co 100 ticks ~10 ms) | `sendCAN_0204(&MS, s_is_idle)` zamiast `sendCAN_0204(&MS)` |
| Slow loop (przed hb_tick) | Dodano: `s_is_idle = (idle_ticks >= auto_off_thresh);` |

#### `inc/CAN_Display.h`
Deklaracja: `void sendCAN_0204(MotorState_t* MS)` → `void sendCAN_0204(MotorState_t* MS, uint8_t is_idle)`

#### `inc/build_version.h`
`"0.0090"` → `"0.0091"` (generowane automatycznie przez build_firmware.ps1)

### Wnioski architektoniczne

1. **target=3 = HMI read channel, target=5 = HMI write ACK port** — muszą być rozróżniane.
   Reguła: `send_multiframe()` zawsze target=3; `sendAcknoledge()` zawsze target=5.

2. **0x6300-0x6304 to HMI→sterownik** (source=3, target=2, op=WRITE). Sterownik
   odbiera, przetwarza i ACKuje do target=5. EBiCS nie generuje tych ramek — tylko
   na nie odpowiada. Dokumentacja sekcji 2 myliła kierunek.

3. **Auto-off przez 0x0204** — bajt[6] steruje sygnałem aktywności napędu. Musi być
   oparty o timer bezczynności, nie o bieżący poziom wspomagania.

4. **mf_src global był błędem** — mechanizm "src=4 dla tgt=4" nie był potrzebny;
   sterownik zawsze odpowiada jako source=2 niezależnie od source zapytania.
