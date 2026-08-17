# Shaon · שעון

A custom Jewish halachic smartwatch built on Watchy v3. Displays zmanim,
Hebrew dates, holidays, the parasha, weather and a digital shiviti on a
200×200 e-paper panel, in Hebrew or English.

---

## Hardware

| | |
|---|---|
| Board | Watchy v3 — ESP32-S3FN8, 8 MB flash, no PSRAM |
| Display | GDEY0154D67, 1.54" 200×200 monochrome e-paper |
| Buttons | 4, one per corner |
| Hard reset | hold BACK + UP for >4s, release UP first, then BACK |

---

## Screens

Cycle with the bottom two buttons. Any of the first six can be switched off
in Settings; Settings itself is always reachable.

1. **Halachic Digital** — hero time, Hebrew date, sun/moon
2. **Halachic Analog** — asymmetric halachic dial (Hebrew) or 24-hour
   wall-clock dial (English)
3. **Zmanim List** — dawn through nightfall
4. **Calendar Info** — Omer count, or next holiday and candle lighting
5. **Torah Quotes** — a daily quote keyed to the week's parasha
6. **Weather** — current conditions plus a 6-day forecast
7. **Settings** — language, units, GRA/MGA, screen toggles, WiFi/zip setup,
   debug log

### Shiviti mode

Hold **top-right for 4 seconds** from any screen except Settings. Opens a
digital shiviti, with the 22 Hebrew letters behind it — each showing the
square letter, its name, and its Paleo-Hebrew form. Paging wraps in both
directions. Either bottom button dismisses and returns to the screen you
were on.

While shiviti mode is active the watch skips its once-a-minute timer wake
entirely — the pages are static and e-paper holds its last frame at zero
power, so there is nothing to redraw. The clock does not update until you
dismiss it. There is no idle timeout.

---

## Buttons

Positions are physical corners. This map has been revised three times;
this is the current one.

**Screens 1–6**

| | tap | hold |
|---|---|---|
| top-left | refresh | — |
| top-right | toggle Hebrew/English | 4s → shiviti |
| bottom-left | previous screen | — |
| bottom-right | next screen | — |

**Settings (screen 7)**

| | tap | hold |
|---|---|---|
| top-left | cursor up | activate row |
| top-right | cursor down | activate row |
| bottom-left / bottom-right | leave Settings | — |

**Shiviti mode**

| | |
|---|---|
| top-left | ◀ previous page |
| top-right | next page ▶ |
| either bottom button | dismiss |

The shiviti gesture is unavailable from Settings, where top-right hold
means "activate row".

---

## Building

PlatformIO, `esp32s3` environment.

```bash
pio run                  # build
pio run --target upload  # build and flash
pio device monitor       # serial, 115200
```

Native USB-CDC does **not** survive deep sleep on the ESP32-S3, so serial
is only reliable on the first wake after a reset. For anything beyond that,
use the on-device log below.

---

## Debug log

Every wake appends a tab-separated record to LittleFS. Retrieve it with
Settings → *Serve Debug Log* (hold), then join the `Shaon-Setup` access
point and open `192.168.4.1/log`.

```
1784820841  timer  rollover=n/a  weather=n/a  retry=skipped(slow)
            syncBefore=…  syncAfter=…  syncFresh=no  clockOk=yes
            utcOffset=-25200  battPct=69  moonFrac=0.261
```

`clockOk` reports whether the RTC actually holds real wall-clock time — a
`syncFresh=yes` alongside `clockOk=no` means a sync was recorded against an
unset clock.

---

## Setup

First boot, or after moving: Settings → *WiFi* or *Zip* (hold) opens a
captive portal on the `Shaon-Setup` access point. Enter the network and a
US zip code; everything else follows from those.

---

## Data sources

| | |
|---|---|
| [Hebcal](https://www.hebcal.com/home/195/jewish-calendar-rest-api) | zmanim, Hebrew dates, holidays, parasha, candle lighting |
| [Open-Meteo](https://open-meteo.com/) | weather, no API key |
| [zippopotam.us](https://api.zippopotam.us/) | zip → city name |
| `data/torah_quotes.csv` | quotes, semicolon-delimited, fetched from this repo |

The location's UTC offset is read from Hebcal's own ISO timestamps rather
than hardcoded, so travel and daylight-saving are handled without the
firmware knowing any timezone rules.

---

## Notes for future work

Things that cost real debugging time and are easy to trip over again.

- **`RTC_DATA_ATTR` or it resets.** Deep sleep is a soft reboot of the data
  segment. Plain globals silently revert to their initialisers on every
  timer wake. This has caused at least two real bugs.
- **`display.init(115200, freshBoot)`** — the two-argument form is required
  for true partial refresh on wake cycles.
- **`mktime()` drops DST offsets.** Parse ISO8601 UTC offsets directly.
- **u8g2 cannot stack combining marks.** It advances by glyph width, so
  nikkud renders beside its letter rather than under it. Pointed text has
  to be a pre-rendered bitmap.
- **u8g2's RLE encoder rejects glyphs above ~95px.** Point 68 (94px box) is
  the largest reliable Hebrew size with this subset. The ceiling is not
  monotonic — 69 fails while 70 passes.
- **`drawXBitmap`, not `drawBitmap`,** for XBM data. XBM is LSB-first;
  `drawBitmap` is MSB-first and mirrors every glyph.
- **`printHebrewCentered`/`printHebrewRight` skip RTL reversal in English
  mode.** Correct for UI text, wrong for text that stays Hebrew in both
  modes. Use the `…RTL` variants for those.
- **Verify font metrics, don't estimate them.** Use `getFontAscent()` /
  `getFontDescent()`, or measure per-glyph BDF boxes.

### Font provenance

`shaon_fonts.h` has mixed provenance. The 42/18/12 sizes came from a
520-glyph `FrankRuhlLibre-Regular.ttf` and regenerate byte-for-byte with
`otf2bdf -p N -r 100` + `bdfconv -f 1`. The 10/11/12v2 sizes came from a
later 451-glyph build and do **not** reproduce from that source.
Regenerating any of those three needs the matching TTF version.

---

## Attribution

Hebrew fonts in `include/shaon_fonts.h` are generated from **Frank Ruhl
Libre**, © 2015 The Frank Ruhl Libre Project Authors, licensed under the
[SIL Open Font License 1.1](https://openfontlicense.org/).

Paleo-Hebrew glyphs in `include/shaon_paleo.h` are generated from **Noto
Sans Phoenician** 2.000, © 2017 Google Inc., licensed under the SIL Open
Font License 1.1.