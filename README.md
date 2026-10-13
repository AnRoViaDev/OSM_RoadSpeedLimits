# OSM_RoadSpeedLimits

Monthly-updated **speed-limit data** extracted from **OpenStreetMap** (via [Geofabrik](https://download.geofabrik.de/))
for the SignDetector Android app. One compact offline file per country.

## Download (always the newest)

```
https://github.com/anrobiadev/OSM_RoadSpeedLimits/releases/latest/download/osm_RO.bin
https://github.com/anrobiadev/OSM_RoadSpeedLimits/releases/latest/download/version.txt
```
(`osm_MD.bin`, `osm_HU.bin`, `osm_BG.bin` … for the other countries in `countries.txt`)

## How it works

* `.github/workflows/build.yml` runs **every month** (day 2) and on demand
  (*Actions → Build speed-limit data → Run workflow*).
* For each country in `countries.txt` it downloads the Geofabrik `.osm.pbf`, runs
  `scripts/build_speedlimits.py` and publishes the result in the release **`latest`**.
* Kept: roads with `maxspeed` / `maxspeed:forward` / `maxspeed:backward`
  (incl. `RO:urban`, `RO:rural`, `RO:motorway` …), plus motorways without a tag
  (country default). Direction is respected (`oneway`, forward/backward).

### File format `OSL1` (big-endian)
```
int32 magic 0x4F534C31
repeat: int32 lat1e6, int32 lon1e6, int32 lat2e6, int32 lon2e6, uint8 limit_fwd, uint8 limit_bwd
```
(0 = no limit / not drivable in that direction)

## Adding a country
Add a line to `countries.txt`, e.g. `AT  europe/austria`, then run the workflow.

## License
Data © **OpenStreetMap contributors**, available under the
[Open Database License (ODbL) 1.0](https://opendatacommons.org/licenses/odbl/1-0/).
Code in this repository: MIT.
