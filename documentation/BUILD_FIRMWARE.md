# Budowanie firmware M820/BL820

> **STATUS:** AKTUALNE ŹRÓDŁO PRAWDY DLA BUILDA.
>
> Obsługiwany publiczny target to M820 / GD32F303RCT6 / BL820. Skrypty znajdujące
> się lokalnie w katalogu głównym nie są częścią odtwarzalnego procesu.

Aktualizacja: 2026-08-03.

## 0. Lokalny katalog używany do wgrywania na tym stanowisku

Na stanowisku użytkownika aktualizator pobiera kolejne pliki z dotychczasowego
katalogu głównego:

```text
C:\Projekty\EBICS\BAFANG_GD32F303RCT6\.build\<wersja>_M820_BL820.bin
```

Artefakt przekazywany do testu sprzętowego należy więc zbudować lokalnym
`build_firmware.ps1`, tak jak wcześniejsze `0.0267`, `0.0268`, `0.0271` i
`0.0272`:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command `
  "& '.\build_firmware.ps1' -ArtifactName '<wersja>' -BootloaderMode ''"
```

Lokalny skrypt pobiera następny numer z `.build/version.txt`; przed buildem
trzeba sprawdzić, czy licznik zawiera numer poprzedniego firmware. Skrypt
`scripts/build-firmware.ps1` pozostaje odtwarzalnym buildem kontrolnym i zapisuje
wyniki w katalogach profilowych, ale tej ścieżki nie należy podawać użytkownikowi
jako miejsca wyboru pliku do wgrania. Dla kandydata sprzętowego oba obrazy należy
porównać przez SHA-256.

## 1. Wymagania

- Windows PowerShell 5.1 lub PowerShell 7;
- Git;
- Arm GNU Toolchain `arm-none-eabi` w wersji dokładnie `13.2.1`
  (`Arm GNU Toolchain 13.2 Rel1`);
- repozytorium z kompletnymi katalogami `Firmware/`, `src/`, `inc/`,
  `gcc_startup/` i `ldscripts/`.

Skrypt szuka toolchaina w `PATH`, następnie w:

```text
C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin
```

Inną lokalizację można przekazać przez `-Toolchain`. Inna wersja kompilatora
kończy build błędem, aby nie tworzyć przypadkowo różnych binarek.

## 2. Normalny build do jazdy

Z katalogu głównego projektu:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  .\scripts\build-firmware.ps1 `
  -Target M820_BL820 `
  -Profile debug `
  -Variant normal `
  -Version 0.0264
```

`normal` wymusza `CAN_DIAGNOSTICS_ENABLE=0`. Po linkowaniu skrypt sprawdza, że
ELF nie zawiera `print_debug_on_CAN`. Brak diagnostyki nie wyłącza wymaganej
komunikacji HMI ani konfiguracji Canable.

Jeżeli `-Version` zostanie pominięte, wersja pochodzi z taga lub skrótu commita.
Do wersji zmodyfikowanego worktree dopisywane jest `-dirty`. Numer nie jest już
zależny od zawartości lokalnego katalogu `.build`.

## 3. Build diagnostyczny

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  .\scripts\build-firmware.ps1 `
  -Target M820_BL820 `
  -Profile debug `
  -Variant diagnostic `
  -Version 0.0264-diag
```

`diagnostic` ustawia `CAN_DIAGNOSTICS_ENABLE=1`. Skrypt wymaga obecności symbolu
`print_debug_on_CAN`, dzięki czemu błędne lub wycięte ustawienie diagnostyki nie
zostanie niezauważone.

## 4. Profil release

Profil `release` istnieje wyłącznie do analizy i jest domyślnie zablokowany.
Optymalizacja `-Os` ujawniła, że obecny kod współdzielony z ISR nie ma jeszcze
kompletnych kontraktów `volatile`/snapshot. Obraz jest podejrzanie mały, a
diagnostyczna odmiana nie przechodzi kontroli symbolu.

Do czasu zakończenia AUD-200…AUD-211:

- do sprzętu używamy tylko `-Profile debug`;
- artefaktów release nie wolno flashować ani publikować;
- `-AllowExperimentalRelease` służy wyłącznie do analizy przez developera.

## 5. Wyniki builda

Każdy wariant ma osobny katalog:

```text
.build/M820_BL820/<profil>/<wariant>/
```

Powstają:

- `.elf`, `.bin` i `.hex`;
- `.map`;
- raport rozmiaru `.size.txt`;
- raport segmentów `.program-headers.txt`;
- manifest `.manifest.json`;
- binarka z nagłówkiem i CRC dla BL820: `*_M820_BL820.bin`.

Manifest zapisuje target, profil, wariant, wersję, commit, stan dirty,
toolchain, listę źródeł, adres końca obrazu, wykorzystanie Flash/RAM oraz
SHA-256 binarek.

Pliki generowane trafiają wyłącznie do `.build`. Skrypt nie przepisuje
`config.h`, linker scriptu ani `inc/build_version.h`.

## 6. Kontrakt mapy pamięci

```text
0x08005000–0x0803E7FF  aplikacja, 230 KiB
0x0803E800–0x0803EFFF  Config A, 2 KiB
0x0803F000–0x0803F7FF  Config B / obecny rekord, 2 KiB
0x0803F800–0x0803FFFF  SOC, 2 KiB
```

Linker zawiera symbole granic i `ASSERT`, które zatrzymują build przed wejściem
aplikacji w Config A. Skrypt dodatkowo:

- odczytuje adresy z ELF;
- sprawdza koniec obrazu;
- odrzuca segment `RWE`;
- potwierdza prawidłowy target M820/BL820.

## 7. Jawna lista źródeł

Kompilowane pliki są zapisane w:

```text
scripts/sources-m820.txt
```

Nowy plik `.c` nie trafia automatycznie do firmware. Trzeba go świadomie dopisać
do manifestu źródeł i wykonać oba buildy debug.

## 8. Znane ostrzeżenia debug

Pozostały wcześniejsze ostrzeżenia:

- `char *` kontra `uint8_t *` w `CAN_Display.c`;
- nieużywane `fw_ver` w `main.c`.

Ostrzeżenie segmentu ELF `RWX` zostało usunięte. Ostrzeżenia kodu należy
naprawić w przeznaczonej do tego fazie, bez mieszania ich ze zmianą mapy pamięci.
