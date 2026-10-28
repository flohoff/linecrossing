
Purpose of this program is to find ways e.g. lines or areas which cross each other in a small angle.

The tool is 70% vibe coded with the use of GLM 5.3 flash

## Usage

```
./linecrossing --help
./linecrossing -i kreisgt.pbf
./linecrossing -i kreisgt.pbf -o kreisgt-crossings.geojson
```

Options:

| Option | Default | Description |
| ------ | ------- | ----------- |
| `-i, --infile` | (required) | OSM input file (.pbf or any other osmium format) |
| `-o, --outfile` | `crossings.geojson` | GeoJSON output file |
| `-a, --angle` | `15.0` | Maximum angle in degrees to report |
| `-p, --parallel` | `1.0` | Minimum angle in degrees to report; anything below counts as parallel/adjacent |
| `-f, --filter` | (none) | Only process ways carrying one of these tag keys, comma separated |
| `-x, --exclude` | (none) | Skip ways carrying one of these tag keys (also via inherited relation tags), comma separated |

### Examples

Find everything up to 15° except collinear/adjacent lines (default behaviour):

```
./linecrossing -i kreisgt.pbf -o kreisgt-crossings.geojson
```

Exclude the typical noise sources: administrative borders, power lines and ways which
inherit those tags from their parent relation:

```
./linecrossing -i kreisgt.pbf -x boundary,power
```

Only look at roads and railways in the angle window 2-5°:

```
./linecrossing -i kreisgt.pbf -f highway,railway -a 5 -p 2
```

## How it works

The tool loads OpenStreetMap data from a `.pbf` file and reports locations where two
ways (lines or area outlines) cross each other at an angle between the `--parallel`
tolerance and the `--angle` threshold (by default: 1° - 15°).

Processing is done in up to three passes over the input file:

* **Pass 0 - relation tags** (only when `--exclude` is used): some ways carry no tags
  themselves and inherit them from their relation instead, e.g. the outline of a nature
  reserve where `boundary=nature_reserve` sits on the multipolygon relation. This pass
  scans only the relations of the file and collects the member way ids of every relation
  carrying an excluded tag key.
* **Pass 1 - geometry**: all remaining ways are split into individual segments (the line
  between two consecutive nodes). Every segment is looked up in a Boost.Geometry
  in-memory R-tree for intersecting segments and then inserted into the tree, so each
  pair of segments is checked exactly once. For each intersection the following filters
  are applied, cheapest first:
  * segments of the same way (self-intersections are ignored)
  * the segments must not be parallel/collinear (angle >= `--parallel`); overlaps like
    shared borders of adjacent areas fall into this category
  * the acute angle between the two segments must be below `--angle` (90° would be a
    right angle, 0° is parallel)
  * the two ways must not share a common node (that is an ordinary junction, not a
    crossing)
  * crossings are de-duplicated per pair of ways and rounded crossing point
    (~0.1 m grid), so the same crossing found by several segment pairs is reported once
* **Pass 2 - metadata**: the way ids of the reported crossings are typically only a small
  subset of all ways. A second, unchanged read of the input file collects tags, user,
  uid, changeset and last modified timestamp only for those ways, keeping the memory
  footprint independent of the number of tagged ways.

The reported crossings are written as LineString features to a GeoJSON FeatureCollection.
The geometry is the shorter of the two crossing way segments. Each feature contains the
angle in the property `angle_deg` and a `way1`/`way2` object with the OSM way id, user
name, uid, changeset id, last modified timestamp and all tags of the two ways involved.

Memory usage scales mainly with the number of segments (~134 bytes each in the R-tree),
so country wide extracts like Germany need plenty of RAM (rough estimate: 50 GB+ for
the tree plus a few GB for the node location index). The `--filter` and `--exclude`
options reduce the number of segments and therefore both memory and runtime.

## Build

```
cmake .
make
```


