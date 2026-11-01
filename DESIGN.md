# Design and development notes

Status of this document: October 2026. It records what the program does today, why it
is built the way it is, the measured performance, and the pitfalls we hit on the way.
For usage see README.md.

## 1. Purpose and achievements

`linecrossing` finds places in OpenStreetMap data where two ways (lines or area
outlines) cross each other at a small angle (default window: 1° to 15°). Such
near-parallel crossings are the typical input for QA tasks like "road crosses railway
at a shallow angle".

Done so far:

* Full pipeline: read OSM PBF → segment ways → geometric matching → metadata
  collection → output as streamed GeoJSON or SpatiaLite database.
* Angle window configurable (`-a` upper, `-p` lower bound), measured in true surface
  degrees (longitude compressed by `cos(lat)`).
* Tag filters `-f` (include) and `-x` (exclude) accepting plain keys (`boundary`) and
  key=value pairs (`man_made=pipeline`), including tags inherited through relations.
* Output geometry is the shorter of the two crossing segments, not just the crossing
  point; properties carry the full OSM metadata (tags, user, uid, changeset,
  last modified) of both ways.
* Measured on kreisgt.pbf (Kreis Gütersloh extract, 31 MB): 477,232 ways, 3,376,015
  segments, 2,667 reported crossings after filtering, ~6 s runtime, ~450 MB RSS.
  Also run on nrw.pbf (North Rhine-Westphalia, ~15x larger).

## 2. Pipeline design

```
                 pass 0 (only with -x)          pass 1                          pass 2
 .pbf ──> relations only ──> excluded way ids ──> nodes+ways ──> crossings ──> unchanged reread ──> writers
          (relation tags)     (set of ids)       └─ R-tree,       (way ids)     meta for involved     │
                                                 matching                       ways only             ├── .geojson (streamed)
                                                                                                       └── .sqlite (SpatiaLite)
```

### Pass 0 - relation tags

Some ways carry no tags of their own; the tags live on the multipolygon relation
(e.g. `boundary=nature_reserve`). When `-x` is given, a first pass reads only the
relations of the file (`osmium::osm_entity_bits::relation`) and collects the member
way ids of every relation matching the exclude list. The pass skips nodes and ways
completely and is therefore cheap. Full polygon assembly is not needed for this:
exclusion only depends on the relation's tags and its member list.

### Pass 1 - geometry and matching

Single streaming read. `osmium::handler::NodeLocationsForWays` (FlexMem index)
resolves node coordinates; the `LineIndex` handler does the rest per way:

* store the minimal way record `WayNodes { id, node refs, layer }` - node refs are only
  needed for the shared-node exclusion later;
* for every node pair build a `Segment { x1, y1, x2, y2, way_index }`;
* **query the R-tree for intersecting segments, then insert the new segment**.
  Because every segment queries *before* it is inserted, each unordered pair of
  segments is examined exactly once and no additional flat segment list is kept -
  the tree instance is the only copy.

The tree is `boost::geometry::index::rtree<Segment, bgi::linear<16, 4>>` with the
segment bounding box derived in `SegmentGetter`. Linear insertion keeps build time
low and the node overhead small.

### The filter chain

Applied cheapest first (see `LineIndex::way`):

1. same way (self-intersections are ignored)
2. `seg_intersect` - proper segment intersection, touching endpoints allowed,
   collinear/parallel pairs rejected by the oriented parametrization (denominator 0)
3. `acute_angle` below the parallel tolerance `-p` (default 1°) - this is the
   adjacency class: shared borders of adjacent areas, duplicated digitized borders,
   overlapping outlines. Filtered *before* the expensive shared-node check.
4. `acute_angle` at or above the threshold `-a` (default 15°)
5. ways on different `layer` tag values (absent = 0) - bridge/tunnel, not a crossing
6. ways sharing a common node (`ways_share_node`) - an ordinary junction, not a
   crossing; checked via a small hash set built from the way with fewer nodes
7. deduplication per (way pair, crossing point rounded to a ~0.1 m grid) - several
   segment pairs can report the same physical crossing, e.g. when the crossing point
   coincides with a node of one way

The angle itself is the acute angle between the two segment direction vectors, with
longitude differences compressed by `cos(lat)` taken at the crossing point, so the
value is meaningful at German latitudes (at 52°N raw lon/lat angles would be off by
up to ~40%).

### Pass 2 - metadata

Only the way ids referenced by reported crossings need tags, user, uid, changeset and
timestamp. A second, unchanged read collects `WayMeta` for those ids only. Memory for
metadata therefore stays tiny regardless of input size, and the hot path of pass 1
never copies tag strings.

### Output writers

* **GeoJSON** (default, `-o out.geojson`): FeatureCollection header, one feature per
  line, trailer. Each feature is built as a `nlohmann::ordered_json` object and
  written with a single `dump()` - at most one feature is buffered, so output size is
  irrelevant to memory. Geometry: `LineString` of the shorter crossing segment,
  coordinates rounded to the OSM grid (7 decimals, `1e7`). `angle_deg` rounded to two
  decimals. String escaping is entirely nlohmann's job.
* **SpatiaLite** (`-o out.sqlite`): table `crossings (fid, angle_deg, way1_id,
  way2_id, props TEXT)` plus `geom` LINESTRING SRID 4326 column with spatial index.
  Rows are inserted with one prepared statement (`GeomFromGeoJSON` + `SetSRID`)
  inside a single transaction. way ids get real columns so they can be filtered in
  SQL; `props` holds the same way objects as the GeoJSON properties.

## 3. Performance and memory

Rough numbers from kreisgt.pbf (31 MB, 2.8M nodes):

| stage | time |
| --- | --- |
| pass 1 (read + R-tree + matching) | ~5.5 s |
| pass 2 (metadata reread) | ~0.2 s |
| peak RSS | ~450 MB |

Memory is dominated by the R-tree: ~134 bytes per segment (bbox 32 B + endpoints
32 B + way index + node overhead of the linear tree). Node locations (FlexMem) and
the way node refs add a comparable chunk.

Outlook for a country-wide Germany extract (~400M segments):

* the tree alone will need ~50 GB, FlexMem node locations several GB more; a 64 GB+
  machine or spatial partitioning (tile the country, process per tile) will be
  necessary;
* `-f`/`-x` are the main lever: filtering to highway/railway/waterway cuts segments
  by an order of magnitude;
* if the incremental insert ever becomes the bottleneck, a bulk-loaded STR-packed
  tree (collect segments first, `rtree(std::begin, std::end)`) is the drop-in
  replacement - but then a separate flat segment copy exists again and the
  query-before-insert trick must be replaced by pair filtering during the bulk load.

## 4. Learnings and pitfalls

These cost us time; recorded here so they don't do it twice.

* **Relative find paths break out-of-source builds.** `find_path(... PATHS
  ../libosmium)` resolves relative to the *build* directory, not the source root.
  The CI clones dependencies into the project root and configures from `build/`, so
  every relative hint missed. Fix: anchor all search paths in `cmake/Find*.cmake` to
  `${CMAKE_SOURCE_DIR}/...`. GitLab CI layouts covered: clones in the project root
  (`buildstabletrixie`, `buildlatest*`) and system packages (`buildpackagestrixie`).
* **find modules crash on NOTFOUND.** `FindProtozero.cmake` did `file(STRINGS
  ${PROTOZERO_INCLUDE_DIR}/...)` unconditionally, producing a confusing hard error
  instead of a clean "not found". Version checks must be guarded with
  `if(..._INCLUDE_DIR)`.
* **libgdal-dev masked missing direct dependencies.** zlib and lz4 were found only
  transitively through GDAL in CI. With the unused `gda` component dropped,
  `zlib1g-dev liblz4-dev` must be installed explicitly.
* **SpatiaLite needs three things** before geometry columns work:
  `load_extension('mod_spatialite')`, `SELECT InitSpatialMetaData(1);` (creates the
  metadata tables - without it `AddGeometryColumn` fails with "unexpected metadata
  layout"), and `PRAGMA trusted_schema=ON;` (SQLite >= 3.37 rejects spatialite's
  `GeometryConstraints()` check otherwise). Consumers of the resulting file must also
  load the extension before touching geometry.
* **Near-parallel intersections are an artifact class of their own.** Exactly
  0.00° crossings are rare; the real noise is a smooth tail up to ~2° from
  digitization wobble - area outlines (landuse), duplicated borders, ways that ride
  along roads or rivers. Hence the `-p` parallel tolerance instead of just cutting
  exact zeros.
* **Boundary ways dominate false positives.** On kreisgt.pbf, `-x boundary,power`
  removed ~3,600 crossings (~40%); administrative borders run along nearly every
  river and railway. Relation-tag inheritance (nature reserves etc.) required the
  relations-only pass 0.
* **osmium API details**: `Reader::file()` is not accessible - use
  `osmium::io::ReaderWithProgressBar` (note the constructor puts the *enable flag
  first*: `{true, input_file}`). `osmium::Timestamp::to_iso()` gives the GeoJSON
  timestamp directly. `positive_ref()` on nodes/members avoids negative ids.
* **nlohmann-json prints full double precision by default.** OSM stores coordinates
  as integers scaled by 1e7, so anything beyond 7 decimals is noise - round explicitly
  (`std::llround(x * 1e7) / 1e7`).

## 5. Code map

Everything lives in `linecrossing.cpp` (~650 lines):

| section | content |
| --- | --- |
| structs | `Segment`, `WayNodes`, `Crossing`, `WayMeta`, R-tree type + `SegmentGetter` |
| helpers | `seg_intersect`, `acute_angle`, `ways_share_node`, `crossing_key` (dedup hash), `split_keys`, `tags_match` |
| `LineIndex` handler | pass 1 matching + pass 2 metadata collection (same class, driven by constructor flags) |
| JSON helpers | `way_meta_json`, `geometry_json`, `properties_json` (shared by both writers) |
| writers | `write_geojson`, `SqliteWriter` + `write_spatialite`, format chosen by `-o` extension |
| `main` | CLI parsing, pass 0, the three reader loops, summary statistics |

`cmake/` contains the vendored find modules (`FindOsmium.cmake`, `FindProtozero.cmake`,
`FindLZ4.cmake` from the libosmium project, `FindNlohmannJSON.cmake` own). They anchor
all search paths to `${CMAKE_SOURCE_DIR}` (see learnings above).

## 6. Ideas not yet implemented

* minimum-length filter: skip tiny segments (a crossing between two 20 cm segments is
  rarely interesting);
* explicit parallel-overlap detection as alternative to raising `-p` (near-parallel
  *and* bounding boxes overlapping proportionally);
* value-level relation filters (`boundary=administrative` only);
* spatial partitioning / tiling for country-scale runs;
* report which relation gave a way its inherited tags in the output metadata.
