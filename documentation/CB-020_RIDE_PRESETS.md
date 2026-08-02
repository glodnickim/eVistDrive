# CB-020 — presety jazdy: zapis do pliku i wczytanie cudzych ustawień

- **Data:** 2026-08-02
- **Status:** ZAIMPLEMENTOWANE (tylko Canable), **nieprzetestowane w aplikacji**.
- **Zakres:** wyłącznie Canable — nowy `ui/js/evistdrive/presets.js`, drobne eksporty
  w `profiles.js` i `dynamics.js`, `ui/index.html`, `ui/style.css`,
  `tests/cb020_preset_roundtrip.js`.
  **Firmware, protokół i format banków bez zmian.**

---

## 1. Po co, skoro jest zakładka Data Backup

Data Backup zrzuca **wszystkie zdarzenia CAN, jakie aplikacja widziała** — wyświetlacz,
baterię, czujnik, numery seryjne — i przywraca całość. To kopia zapasowa jednego roweru:
dobra, żeby wrócić do stanu sprzed eksperymentu, bezużyteczna, żeby komuś powiedzieć „tak
jeździ mój rower".

Wymaga też wcześniejszego zsynchronizowania każdej zakładki, jest nieczytelna (surowe
zdarzenia) i przywraca wszystko naraz, łącznie z rzeczami związanymi z konkretnym
egzemplarzem.

Preset to co innego: **małý, czytelny plik z samym strojeniem jazdy**, do wysłania mailem
albo wrzucenia na forum.

## 2. Co plik zawiera

- **oba banki, po 5 poziomów** — tryb wspomagania i jego parametry, limity mocy i prądu,
  warunek startu, boost i smooth start, rampy prądu, wygaszanie i filtry mocy,
- **ustawienia bankowe** — kompensacja kadencji, prędkość odcięcia i obroty Walk Assist,
- **blok globalny** — kroki korby do startu, zanik boostu, zatrzask (deadband, sustain,
  podłoga prądu), wygładzanie RUN,
- **metadane** — wersja formatu, data, nazwa i notatka od autora, wersja firmware
  kontrolera i wersja formatu banków.

## 3. Czego NIE zawiera — i dlaczego to jest konstrukcja, a nie filtr

Nie ma tam niczego, co należy do **jednego egzemplarza roweru**:

| Pominięte | Dlaczego wysłanie tego komuś jest szkodliwe |
|---|---|
| kalibracja span czujnika nacisku | zmierzona na jednym czujniku; na innym oznacza inną siłę na pedale |
| napięcie 100% pakietu, pojemność baterii | zależne od pakietu; obce wartości psują wskazanie SOC |
| obwód koła | psuje prędkość, a przez nią limit i licznik |
| numery seryjne, dane wyświetlacza i baterii | nie są ustawieniem |
| `active_bank` | to stan bieżący, nie ustawienie — import nie ma przełączać banku |

**Bezpieczeństwo nie polega na czarnej liście.** Eksport bierze dokładnie dwa obiekty stanu
aplikacji — `state.lastBanks` i `state.lastTuning` — a wymienione wyżej rzeczy po prostu
w nich nie występują. Gdyby ktoś kiedyś dołożył pole do jednego z nich, wejdzie do presetu
świadomie; nie ma listy wykluczeń, o której aktualizacji trzeba pamiętać.

Test sprawdza to wprost: szuka w wygenerowanym pliku ciągów `span_native`,
`soc_full_pack_10mv`, `wheel`, `serial` i **kończy się niepowodzeniem**, gdyby któryś się
pojawił.

## 4. Import

**Nic nie trafia do sterownika automatycznie.** Wczytanie wypełnia tylko edytor; zapis
następuje dopiero po Twoim „Write (RAM)" i „Save to Flash" — ta sama dyscyplina co przy
zwykłej edycji. Plik od obcej osoby nie może wejść do sterownika bez Twojego spojrzenia.

Po wskazaniu pliku pojawia się panel wyboru: **poziom po poziomie dla każdego banku**
i osobno blok globalny, z licznikiem („Loads 7 level(s) + global"). Wzorowane na mechanizmie
„Copy to…" z FW-071, żeby nie uczyć się drugiej logiki wyboru.

Trzy zabezpieczenia:

- **Tryb nieobsługiwany przez firmware** (np. Power Curve na starszym) — poziom wyszarzony
  i odznaczony, z wyjaśnieniem. Bez tego kontroler odrzuciłby przy zapisie **cały bank**,
  a użytkownik zobaczyłby tylko, że nic się nie zmieniło.
- **Wartości spoza zakresu przycinane** do granic edytora, każde przycięcie raportowane
  z wartością przed i po. Ręcznie zepsuty plik nie wprowadzi liczby, której interfejs by
  nie przyjął. Zakresy pochodzą z **tych samych deskryptorów pól**, z których renderuje się
  edytor — nie ma drugiego zestawu, który mógłby się rozjechać.
- **Brak odczytu z roweru blokuje import** — inaczej preset nakładałby się na wartości
  zastępcze i użytkownik zapisałby do sterownika mieszankę cudzych ustawień i placeholderów.

## 5. Format pliku

```json
{
  "format": "evistdrive-preset",
  "version": 1,
  "created": "2026-08-02T…",
  "name": "góry, miękki start",
  "note": "…",
  "source": { "controller_sw_version": "…", "bank_schema_version": 6 },
  "banks": [ { "bank_index": 0, "levels": [ … ] }, { … } ],
  "tuning": { … }
}
```

Wcięcie 2 spacje — plik ma być czytelny i porównywalny w zwykłym edytorze, bo najczęstsze
użycie to „pokaż mi, czym się różnimy".

Import odrzuca: plik o innym `format` (w tym backup całego urządzenia, z wyjaśnieniem, do
czego służy tamta zakładka), `version` nowszy niż aplikacja rozumie, oraz preset bez banków
i bez bloku globalnego.

## 6. Testy w aplikacji

1. Read all → Export preset → plik się pobiera, nazwa zawiera datę i podaną nazwę.
2. Otworzyć plik w edytorze tekstu: mają być banki i blok globalny, **nie ma** kalibracji
   czujnika ani napięcia baterii.
3. Zmienić kilka wartości, Import ten sam plik, wybrać wszystko → wartości wracają.
4. Import z odznaczonym bankiem 2 → bank 2 **nietknięty**.
5. Import bez wcześniejszego „Read all" → odmowa z wyjaśnieniem.
6. Ręcznie ustawić w pliku `max_iq_pct: 250` → import przycina do 100 i **mówi o tym**.
7. Podać plik z Data Backup → odmowa z informacją, że tamten format obsługuje inna zakładka.
8. Po imporcie górny pasek sygnalizuje niezapisane zmiany w RAM; do sterownika nic nie poszło.
