# Karta zmiany FW-031 — zatrzask jazdy (run latch) + podłoga prądu

- **Data:** 2026-07-26
- **Status:** WDROŻONE, build `0.0202`. Czeka na test sprzętowy.
- **Build:** `0.0202_M820_BL820.bin`, SHA-256
  `2AA807CCD1346921DACC0A7C1C9E93AC38C2EB2E154686EB63FA1CDA75431DC4`. Bez błędów.
- **Zakres:** firmware (`ride_control.c`, `assist_modes.c`). Bez zmian w Canable (na razie).
- **Powiązane:** [[project-ride-core-runaway-fixed]] (FW-028), FW-030 (jeden tryb ride core).

---

## 1. Problem

W torze ride core wspomaganie było **ściśle proporcjonalne do nacisku**, bez stanu
„załączone". Przy bardzo lekkim pedałowaniu i w **martwych punktach korby** moment
spada prawie do zera → `iq` spada prawie do zera → silnik **gasł i zapalał się**
(ON/OFF/ON/OFF). Filtry `power_fall_filter_ms`/`release_ms` tylko maskowały dziurę.

## 2. Rozwiązanie (Krok 1)

Zatrzask jazdy + podłoga prądu w gałęzi ride core ([ride_control.c], po obliczeniu
`iq_target` z trybu, **przed** manetką i przed limitami/rampą — dokładnie tam, gdzie
wskazał deweloper):

1. **Start** wymaga pedałowania w przód i nacisku ≥ progu → dopiero wtedy `assist_latched=1`.
   Poniżej progu i przed zatrzaśnięciem `iq_target=0` (nie rusza od muśnięcia).
2. **Po starcie** — dopóki kręcisz w przód i nacisk ≥ `RUN_DEADBAND`, zatrzask
   odświeżany. Przy chwilowym braku nacisku (martwy punkt) trzyma przez `HOLD_MS`.
3. **Podłoga:** gdy zatrzaśnięte, `iq_target` nie schodzi poniżej `MIN_IQ_PCT`%
   limitu prądu poziomu → moc nie zapada się między naciskami.
4. **Cięcie natychmiast:** `safety_cut` (hamulec / cofanie / usterka) **lub stop
   korby** → `assist_latched=0`, brak podłogi. Bez zmian w bezpieczeństwie.
5. **Manetka** liczona PO tym bloku → omija próg startu, działa bez pedałowania jak dotąd.

## 3. Parametry

| parametr | znaczenie | wartość (Krok 1) | źródło |
|---|---|---|---|
| próg startu | „nie rusza od muśnięcia" | `without_rotation_threshold_mv` (domyślnie 18 mV ≈ 0,7 kg) | **istniejące „Minimum pedal load (kg)"** w Canable (per poziom) |
| `ASSIST_RUN_DEADBAND_MV` | po starcie wystarcza mniejszy nacisk | 5 mV | stała w `ride_control.c` |
| `ASSIST_HOLD_MS` | trzymanie przy braku nacisku (martwy punkt) | 700 ms | stała |
| `ASSIST_MIN_IQ_PCT` | podłoga prądu (mała, żeby nie wyrywało) | 4 % | stała |

Loop ~4 kHz → `ASSIST_CTRL_TICKS_PER_MS = 4` (`HOLD_MS * 4` ticków).

**„Minimum pedal load" — zmiana znaczenia:** wcześniej działało TYLKO przy
`assist_without_rotation=true` (start z miejsca bez kręcenia) i przy pedałowaniu było
ignorowane. Teraz to samo pole jest też **progiem startu podczas pedałowania**. Jedno
pole = spójny „ile nacisku, żeby wspomaganie się załączyło". Nie dodano nowej zmiennej.

## 4. Dodatkowo w tym build

- Firmware default filtrów wyrównany do wartości sprawdzonych przez właściciela:
  `power_fall_filter_ms = 250`, `release_ms = 400` (było 0; pośrednio 300/500).
  `power_rise_filter_ms = 0` (reakcja na narastanie żwawa).

## 5. Odroczone (Krok 2 — po potwierdzeniu odczucia)

Wystawić `RUN_DEADBAND`, `HOLD_MS`, `MIN_IQ_PCT` jako konfigurowalne w Canable
(Dynamics / Ride feel — globalnie, nie per bank; to charakter całej jazdy). Dodać do
telemetrii flagi diagnostyczne: `assist_latched`, `assist_hold_ticks`,
`raw_iq_request`, `effective_iq_request`.

## 6. Bezpieczeństwo

- Podłoga działa **wyłącznie** przy realnym kręceniu w przód po legalnym starcie.
- `safety_cut` i stop korby zerują zatrzask i podłogę **natychmiast** (tnie od razu).
- Podłoga wchodzi PRZED limitami prędkości/temperatury → na limicie prędkości też cięta.
- Prąd baterii dalej ograniczony `BATTERYCURRENT_MAX` (15 A) przez limiter (bez zmian).

## 7. Test po build

1. Start: bardzo lekki dotyk pedału **NIE** rusza; wyraźny nacisk rusza.
2. Jazda: po starcie bardzo lekkie kręcenie **NIE** gasi wspomagania (brak ON/OFF).
3. Stop korby / cofnięcie / hamulec → wspomaganie **tnie natychmiast**.
4. Manetka (jeśli podłączona): działa bez pedałowania, cięta hamulcem/cofaniem.
5. Walk Assist i kalibracja Halla — bez zmian, działają.
