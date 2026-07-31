# FW-054 - czasowe podtrzymanie Walk Assist po puszczeniu przycisku

- Data: 2026-07-28
- Status: wdrozone, build M820 BL820 `0.0242_M820_BL820.bin`,
  test sprzetowy wymagany
- Zakres: firmware M820, banki profili 0x6020/0x6021, zapis 0x6022,
  zakladka eVistDrive Walk w Canable
- Powiazane: FW-029, FW-043, FW-051

## 1. Cel

Walk Assist moze opcjonalnie jechac dalej po puszczeniu przycisku WA.
Funkcja jest konfigurowana niezaleznie dla Banku 1 i Banku 2.

Domyslnie funkcja jest wylaczona. Wtedy pozostaje dotychczasowy tryb
dead-man: silnik pracuje tylko podczas trzymania przycisku.

Po wlaczeniu:

1. Uzytkownik uruchamia WA normalnie i trzyma przycisk do rozpoczecia pracy.
2. Puszczenie przycisku wlacza czasowe podtrzymanie.
3. Podtrzymanie trwa maksymalnie przez ustawiony czas, domyslnie 30 s.
4. Po zatrzymaniu funkcja nie uruchamia sie ponownie bez nowego nacisniecia WA.

## 2. Ustawienia bankowe

| Pole | Format | Zakres | Domyslnie |
|---|---:|---:|---:|
| `wa_latch_after_release` | bool | OFF/ON | OFF |
| `wa_latch_timeout_s` | sekundy | 1-120 | 30 |

Oba pola sa czescia wybranego banku. Zmiana banku moze wiec jednoczesnie
zmienic zachowanie WA, prad, docelowe RPM i odciecie predkosci.

## 3. Warunki natychmiastowego zatrzymania

Podtrzymany WA zatrzymuje sie przed timeoutem po:

- ponownym nacisnieciu przycisku WA,
- nacisnieciu UP albo DOWN,
- zmianie stanu swiatla,
- zmianie poziomu wspomagania,
- nacisnieciu przycisku zasilania,
- nacisnieciu hamulca,
- bledzie sterownika,
- osiagnieciu bankowego odciecia predkosci kola.

Odciecie predkosci kasuje podtrzymanie i nie powoduje samoczynnego ponownego
startu. Przy zwyklym trzymaniu przycisku pozostaje dotychczasowa histereza:
WA moze wznowic prace po spadku predkosci o 0,5 km/h ponizej limitu.

## 4. Format banku

Format banku zostal podniesiony z v2 do v3:

| Wersja | Naglowek | Rekordy | CRC | Dlugosc |
|---|---:|---:|---:|---:|
| v1 | 8 B | 5 x 35 B | 2 B | 185 B |
| v2 | 10 B | 5 x 35 B | 2 B | 187 B |
| v3 | 12 B | 5 x 35 B | 2 B | 189 B |

Nowe bajty naglowka v3:

- `[10]`: `wa_latch_after_release`, 0 = OFF, wartosc niezerowa = ON,
- `[11]`: `wa_latch_timeout_s`, prawidlowy zakres 1-120 s.

Firmware nadal odczytuje v1 i v2. Dla starego banku oba nowe pola dostaja
wartosci OFF i 30 s. Canable odczytuje v1/v2/v3 i zapisuje zawsze v3.
Bufor EEPROM `bank_store[2][192]` nie zmienia rozmiaru, bo 189 B nadal sie
w nim miesci.

## 5. Pliki

Firmware:

- `inc/assist_modes.h`
- `inc/main.h`
- `src/assist_modes.c`
- `src/main.c`

Canable:

- `canbus.js`
- `bafang-parser.js`
- `ui/js/tab-banks.js`
- `ui/js/websocket.js`
- `ui/js/ebics-compat.js`
- `ui/index.html`

## 6. Procedura testu

1. Wgrac nowy firmware i otworzyc `eVistDrive Walk`.
2. Wykonac `Sync`, wybrac Bank 1 i ustawic:
   `Continue after releasing Walk button = On`, `Timed run limit = 30 s`.
3. Uzyc `Apply Walk settings`, potem `Save Walk settings to flash` na postoju.
4. Na kole uniesionym uruchomic WA i puscic przycisk. Potwierdzic ciagla prace.
5. Uruchamiac test ponownie i osobno sprawdzic zatrzymanie przez:
   WA, UP, DOWN, hamulec oraz limit predkosci.
6. Pozostawic WA bez naciskania przyciskow i potwierdzic zatrzymanie po 30 s.
7. W Banku 2 pozostawic opcje OFF i potwierdzic zatrzymanie od razu po
   puszczeniu przycisku.
8. Zrestartowac sterownik, wykonac `Sync` i potwierdzic zachowanie obu bankow.

Pierwszy test nalezy wykonac z kolem w gorze. Dopiero po potwierdzeniu wszystkich
warunkow zatrzymania testowac podczas prowadzenia roweru.

## 7. Weryfikacja

- firmware skompilowany poprawnie,
- artefakt: `.build/0.0242_M820_BL820.bin`,
- `node --check` poprawny dla wszystkich zmienionych plikow JavaScript,
- pozostaly dotychczasowe ostrzezenia kompilatora dotyczace signedness ramek
  CAN, nieuzywanej zmiennej `fw_ver` i segmentu RWX linkera.
