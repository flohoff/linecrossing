# Test data

`crossings.osm.xml` is synthetic (lat 52°N, one scenario per ~0.01° of longitude):

| ways | scenario | expected |
| --- | --- | --- |
| 1, 2 | ~10° crossing, way 1 has quotes/unicode in tags and user | reported |
| 3, 4 | 45° crossing | skipped by angle |
| 5, 6 | 0.5° crossing | skipped parallel |
| 7, 8 | 10°, ways share a node | skipped shared |
| 9, 10 | 10°, way 9 `boundary=administrative` | excluded with `-x boundary` |
| 11, 12 | 10°, way 11 untagged, in relation `boundary=nature_reserve` | excluded with `-x boundary` |
| 13 | self-crossing at ~4° | never reported |
| 14, 15 | 10° at an unshared node of way 15 | reported once (dedup) |
| 16, 17 | 10°, way 16 is `landuse` | dropped with `-f highway` |
| 18 | second node missing in file | no segment |
