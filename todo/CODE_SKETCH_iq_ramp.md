# Szkic kodu: adaptacyjna rampa i_q (płynne schodzenie mocy + przejścia w jeździe)

Cel: zastąpić skokowe zmiany `MS.i_q_setpoint` w trakcie jazdy (Błąd #5 z
`PLAN_POWER_PATH_smooth_ride.md`) rampą stosowaną PRZY KAŻDEJ zmianie (góra i dół),
z krokiem adaptacyjnym od prędkości i kadencji (wzorzec TSDZ2 `set_motor_ramp`).

To jest szkic referencyjny dla Opusa — nazwy/skale do dostrojenia na rowerze.
Kontekst kodu: `reg_ADC_processing()` @4 kHz, `map()` przycina do zakresu (`main.c:1864`).

---

## 1. Stałe strojeniowe — `inc/config.h`

```c
// --- Adaptacyjna rampa i_q (zastępuje sztywne IQ_SLEW_UP/IQ_SLEW_DOWN) ---
// Jednostka: przyrost i_q na jeden tick 4 kHz. Większy = szybciej (mniej płynnie).
#define IQ_RAMP_UP_SLOW     1   // start/niska kadencja: miękkie narastanie
#define IQ_RAMP_UP_FAST     6   // w jeździe: żwawa reakcja
#define IQ_RAMP_DOWN_SLOW   2   // niskie prędkości: miękkie schodzenie mocy
#define IQ_RAMP_DOWN_FAST   8   // przy prędkości: szybkie, ale gładkie odcięcie

// Punkty mapy adaptacji (jak w TSDZ2: 4-20 kph, 20-70 rpm)
#define IQ_RAMP_SPEED_LO    400   // MS.Speedx100 = 4.00 km/h
#define IQ_RAMP_SPEED_HI   2000   // 20.00 km/h
#define IQ_RAMP_CAD_LO       20   // rpm
#define IQ_RAMP_CAD_HI       70   // rpm

// Smooth start: obwiednia tłumienia pierwszego momentu z postoju (opcjonalnie, patrz §4)
#define START_RAMP_TICKS   1600   // 400 ms @4kHz: 0->100% obwiedni startu
```

> Uwaga: jeśli `MS.i_q_setpoint` osiąga rzędy setek, dobierz kroki tak, by pełne
> narastanie trwało ~300-500 ms (start) i ~120-200 ms (odcięcie). Przelicz: kroki/tick ×
> 4000 = kroki/s; czas = zakres_i_q / (kroki/s).

## 2. Helpery kroku rampy — `src/main.c` (blisko `map_rezi`, plik-scope static)

```c
// Krok narastania i_q: większy przy większej prędkości LUB kadencji (bierzemy szybszy).
static int32_t iq_ramp_up_step(void){
    int32_t s = map((int32_t)MS.Speedx100, IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI,
                    IQ_RAMP_UP_SLOW, IQ_RAMP_UP_FAST);
    int32_t c = map((int32_t)MS.cadence, IQ_RAMP_CAD_LO, IQ_RAMP_CAD_HI,
                    IQ_RAMP_UP_SLOW, IQ_RAMP_UP_FAST);
    int32_t step = (c > s) ? c : s;           // szybszy z dwóch
    return (step < IQ_RAMP_UP_SLOW) ? IQ_RAMP_UP_SLOW : step;
}

// Krok schodzenia i_q: głównie od prędkości (przy prędkości szybciej, na wolno miękko).
static int32_t iq_ramp_down_step(void){
    int32_t step = map((int32_t)MS.Speedx100, IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI,
                       IQ_RAMP_DOWN_SLOW, IQ_RAMP_DOWN_FAST);
    return (step < IQ_RAMP_DOWN_SLOW) ? IQ_RAMP_DOWN_SLOW : step;
}
```

## 3. Przepisany blok slew — `src/main.c` linie ~1534-1550 (ZAMIANA)

Usuń całą logikę `slewing_up` i skokowe gałęzie `else{ MS.i_q_setpoint=iq_target; }`.

```c
{
    int32_t iq_target = update_setpoint();

    if(MS.brake_active_flag || Backwards_counter>=4 || overtemp_stage>=2){
        MS.i_q_setpoint = iq_target;          // bezpieczeństwo: natychmiast (cel = 0 w tych stanach)
    }
    else if(iq_target > (int32_t)MS.i_q_setpoint){          // WZROST — zawsze rampa w górę
        int32_t step = iq_ramp_up_step();
        int32_t d = iq_target - (int32_t)MS.i_q_setpoint;
        MS.i_q_setpoint += (d > step) ? step : d;
    }
    else if(iq_target < (int32_t)MS.i_q_setpoint){          // SPADEK — zawsze rampa w dół
        int32_t step = iq_ramp_down_step();
        int32_t d = (int32_t)MS.i_q_setpoint - iq_target;
        MS.i_q_setpoint -= (d > step) ? step : d;
    }
    // iq_target == i_q_setpoint -> bez zmian
}
if(s_is_idle) MS.i_q_setpoint = 0;            // twarde zero na idle (bez zmian)
```

Efekt: schodzenie mocy przy zwolnieniu nacisku = gładka rampa-dół (nie skok);
załączanie/wyłączanie w ruchu i przez martwe punkty = `i_q` zawsze goni cel przez rampę.

## 4. (Opcjonalnie) Smooth start — tłumienie pierwszego momentu

Uzupełnia rampę: przy ruszaniu z postoju atenuuje moment, żeby start był miękki nawet
gdy cel od razu jest wysoki. Wstaw w ścieżce liczenia prądu (np. w `assist_legacy_running_current`
przed użyciem `torque_filtered`, albo w nowym `update_setpoint`).

```c
// plik-scope
static uint16_t start_ramp_ticks = 0;

// w reg_ADC_processing, po policzeniu forward_pedaling / stopu:
if(MS.cadence==0 && MS.Speedx100==0){           // pełny postój -> uzbrój obwiednię
    start_ramp_ticks = 0;
} else if(start_ramp_ticks < START_RAMP_TICKS){
    start_ramp_ticks++;
}

// przy liczeniu prądu z momentu:
uint16_t tq = MS.torque_filtered;
if(start_ramp_ticks < START_RAMP_TICKS){
    tq = (uint16_t)(((uint32_t)tq * start_ramp_ticks) / START_RAMP_TICKS);  // 0->100%
}
// ...użyj tq zamiast MS.torque_filtered w mapowaniu na prąd...
```

## 5. Zależność: usuń Override (inaczej rampa-dół nie zadziała)

Rampa-dół zadziała tylko jeśli `iq_target` faktycznie spada, gdy przestajesz pedałować.
Dziś blok Override (`main.c:2462-2470`) trzyma cel i resetuje `torque_counter`.
**Usuń go / za flagę OFF** (KROK 4 planu), żeby po zaniku kadencji cel schodził do 0,
a rampa-dół dała płynne zejście zamiast opóźnionego skoku.

---

## Weryfikacja
1. Log `i_q_setpoint` vs czas (UART `main.c:563` lub CAN `0x80010203`).
2. **Zwolnij nacisk w jeździe** → `i_q` schodzi gładką pochyłą (nie pionową krawędzią).
3. **Odpuść pedały na chwilę i wróć** → brak skoków, `i_q` płynnie w dół i z powrotem.
4. **Wolno pod górę (martwe punkty)** → moc stabilna, bez migotania.
5. Regresje: hamulec/wsteczny/overtemp nadal ucinają natychmiast; limit prędkości OK.
6. Strojenie: jeśli odcięcie za wolne → zwiększ `IQ_RAMP_DOWN_FAST`; jeśli start szarpie →
   zmniejsz `IQ_RAMP_UP_SLOW` lub wydłuż `START_RAMP_TICKS`.

## Przyszła konfiguracja przez CAN
Wystaw `IQ_RAMP_UP_SLOW/FAST`, `IQ_RAMP_DOWN_SLOW/FAST`, `START_RAMP_TICKS` jako pola w
`MotorParams_t` i zmapuj w `parser.c` na wolne indeksy Para1/Para2 (wzorem
`Override_Duration=Para1[37]*40`). Wdrożyć dopiero po strojeniu na rowerze.
