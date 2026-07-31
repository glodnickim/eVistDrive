# FW-047 — wolny "ogon" na końcu wygaszania (klik przy zatrzymaniu)

> **WYCOFANE przez FW-072 (2026-07-31).** Mechanizm opisany w tej karcie **nie usunął kliku**,
> a przy okazji niejawnie wydłużał wyłączanie: `release_ms` doprowadzało prąd do 15 %, po czym
> szło jeszcze 600 ms do zera — ustawione 650 ms zachowywało się jak ~1250 ms. Kod został
> usunięty, wygaszanie to znowu **jedna liniowa rampa**. Karta zostaje jako zapis próby
> i jej wyniku; patrz `FW-072_SINGLE_RELEASE_RAMP.md`.

- **Data:** 2026-07-27
- **Status:** WDROZONE W DRZEWIE — **NIE ZBUDOWANE** (na polecenie wlasciciela; modul Walk
  Assist jest w tym czasie strojony osobno, FW-045/046).
- **Zakres:** wylacznie `src/assist_dynamics.c` (ksztalt wygaszania Iq). Bez zmian w WA, FOC,
  hamulcu, cofaniu, limitach.

---

## Objaw

Wlasciciel po tescie: rampa startowa OK, hamowanie "nawet OK", ale **na samym koncu jest
wyrazne klikniecie, ktore sie NIE ZMNIEJSZA** mimo wydluzania rampy (release_ms 325 -> 650 ms).

## Dlaczego wydluzanie rampy nie pomagalo

Rampa wygaszania jest **liniowa**: moment spada ze STALA predkoscia az do zera. Wydluzenie
release_ms rozciaga caly przebieg proporcjonalnie, wiec **tempo zdejmowania sily w ostatniej
chwili pozostaje takie samo**. A to wlasnie tempo rozprzegania zebow robi klik, nie wartosc
momentu. Stad obserwacja "nie zmniejsza sie".

## Poprawka — dwustopniowe wygaszanie

Ostatni kawalek zjazdu dostaje **wlasna, duzo wolniejsza rampe**:

```
RIDE_FADE_TAIL_PCT        15   // ponizej 15% poziomu, z ktorego zaczela sie rampa
RIDE_FADE_TAIL_MS        600   // normalne zatrzymanie: ten ostatni kawalek trwa ~600 ms
RIDE_FADE_TAIL_SAFETY_MS 250   // hamulec/cofanie/blad: ogon krotszy, reakcja szybka
```

Dla porownania: wczesniej te ostatnie 15% zajmowaly ~0,15 x 650 = **ok. 100 ms**. Teraz
**ok. 600 ms** — czyli sila schodzi do zera ok. 6x lagodniej i zeby rozprzegaja sie stopniowo.

Oba kroki (glowny i ogon) sa **zatrzaskiwane raz, na starcie wygaszania**, wiec przebieg jest
przewidywalny. Krok zawsze >= 1, wiec rampa zawsze dochodzi do zera, takze z bardzo niskiego
poziomu. Rozroznienie normalne/safety po `input->safety_cut` (decyzja wlasciciela: ogon ma
dzialac takze przy hamulcu, ale krotszy).

Uproszczono przy okazji: usuniety akumulator ulamkowy (`profile_release_fraction`) — przy
dwoch zatrzasnietych krokach nie byl juz potrzebny, blad czasu < 1%.

## ⚠️ Drugi podejrzany — NIE ruszony w tej zmianie

W `main.c` (~1803):

```c
if(MS.i_q_setpoint==0){ PI_iq.integral_part=0; PI_id.integral_part=0; }
```

W momencie, gdy komenda pradu osiaga DOKLADNIE zero, calka regulatora jest zerowana **skokowo**.
To nieciaglosc dokladnie w punkcie, w ktorym slychac klik — i rowniez tlumaczy, czemu
wydluzanie rampy nie pomagalo (rampa dochodzi do zera wolniej, ale samo sciecie na koncu jest
identyczne).

**Nie zmieniono tego celowo:** to jest FW-028, poprawka na przeciaganie silnika — najgrozniejszy
naprawiony blad w projekcie. Jesli po FW-047 klik zostanie, kolejnym krokiem jest wygaszanie
calki w kilkadziesiat ms zamiast zerowania skokowego, z ZACHOWANIEM twardego zerowania przy
`safety_cut`. Osobny build, zeby dalo sie jednoznacznie ocenic, ktora zmiana pomogla.

## Test (po zbudowaniu)

1. Zatrzymanie pedałowania w jezdzie: brak wyraznego kliku na koncu; moc schodzi lagodnie,
   ostatnia faza zauwazalnie dluzsza.
2. Hamulec: dalej tnie szybko (ogon 250 ms), ale bez kliku na koncu.
3. Cofniecie korby: tnie natychmiast (bez zmian w logice), ogon jak przy hamulcu.
4. Brak regresji: silnik nie "wisi" po zatrzymaniu, brak przeciagania (FW-028).
5. Jesli klik zostanie -> patrz sekcja o calce PI wyzej (osobny krok).
