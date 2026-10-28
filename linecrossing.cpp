/*
 * Copyright 2026 - Florian Lohoff <f@zz.de>
 *
 * As this tool has mostly been "created" by an LLM i would consider this all be mostly
 * non copyrightable in most jurisdications. In doubt i put all my contributions under the
 * Creative Commons CC0.
 *
 */

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// For the NodeLocationForWays handler
#include <osmium/handler/node_locations_for_ways.hpp>

// Allow any format of input files (XML, PBF, ...)
#include <osmium/io/any_input.hpp>

// For the location index. There are different types of indexes available.
// This will work for all input files keeping the index in memory.
#include <osmium/index/map/flex_mem.hpp>

#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>
#include <osmium/io/reader_with_progress_bar.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>
#include <sqlite3.h>

// The type of index used. This must match the include file above
using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;

// The location handler always depends on the index type
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

namespace po  = boost::program_options;
namespace bg  = boost::geometry;
namespace bgi = boost::geometry::index;

using Point = bg::model::point<double, 2, bg::cs::cartesian>;
using Box   = bg::model::box<Point>;

struct Segment {
	double x1;
	double y1;
	double x2;
	double y2;
	std::uint32_t way; // index into ways vector
};

struct SegmentGetter {
	using result_type = Box;
	Box operator()(const Segment& s) const {
		const auto minx = std::min(s.x1, s.x2);
		const auto miny = std::min(s.y1, s.y2);
		const auto maxx = std::max(s.x1, s.x2);
		const auto maxy = std::max(s.y1, s.y2);
		return Box{Point{minx, miny}, Point{maxx, maxy}};
	}
};

// Linear insertion: fast bulk build, low memory overhead per node.
using RTree = bgi::rtree<Segment, bgi::linear<16, 4>, SegmentGetter>;

// Minimal per-way data needed while matching: id + node refs for shared-node exclusion
struct WayNodes {
	std::uint64_t id = 0;
	std::vector<std::uint64_t> nodes;
};

struct Crossing {
	std::uint64_t way1;
	std::uint64_t way2;
	double x;      // intersection point, used for dedup only
	double y;
	double sx1;    // shorter of the two crossing segments, written as geometry
	double sy1;
	double sx2;
	double sy2;
	double angle;
};

// Full per-way metadata, only collected for ways with reported crossings (second pass)
struct WayMeta {
	std::string user;
	std::uint64_t uid = 0;
	std::uint64_t changeset = 0;
	std::string timestamp;
	std::vector<std::pair<std::string, std::string>> tags;
};

using WayMetaMap = std::unordered_map<std::uint64_t, WayMeta>;

// SplitMix64 finalizer for the dedup key
inline std::uint64_t mix64(std::uint64_t x) {
	x += 0x9e3779b97f4a7c15ULL;
	x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
	x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
	return x ^ (x >> 31);
}

static inline std::uint64_t crossing_key(const Crossing& c) {
	std::uint64_t h = mix64(std::min(c.way1, c.way2));
	h = mix64(h ^ std::max(c.way1, c.way2));
	// ~0.1m grid to collapse multiple segment pairs reporting the same crossing point
	h = mix64(h ^ static_cast<std::uint64_t>(std::llround(c.x * 1e6)));
	h = mix64(h ^ static_cast<std::uint64_t>(std::llround(c.y * 1e6)));
	return h;
}

// Proper segment intersection (touching endpoints allowed, collinear/parallel rejected).
static inline bool seg_intersect(const Segment& s, const Segment& t, double& ix, double& iy) {
	const double d1x = s.x2 - s.x1;
	const double d1y = s.y2 - s.y1;
	const double d2x = t.x2 - t.x1;
	const double d2y = t.y2 - t.y1;
	const double denom = d1x * d2y - d1y * d2x;
	if (denom == 0.0) {
		return false;
	}
	const double dx = t.x1 - s.x1;
	const double dy = t.y1 - s.y1;
	const double r = dx * d2y - dy * d2x;
	const double u = dx * d1y - dy * d1x;
	if ((denom > 0.0 && (r < 0.0 || r > denom || u < 0.0 || u > denom)) ||
	    (denom < 0.0 && (r > 0.0 || r < denom || u > 0.0 || u < denom))) {
		return false;
	}
	ix = s.x1 + (r / denom) * d1x;
	iy = s.y1 + (r / denom) * d1y;
	return true;
}

// Acute angle between the two segment directions in [0, 90] degrees.
// Longitudes are compressed by cos(latitude) so the angle is areal-correct.
static inline double acute_angle(const Segment& s, const Segment& t, double at_y) {
	const double coslat = std::cos(at_y * M_PI / 180.0);
	const double d1x = (s.x2 - s.x1) * coslat;
	const double d1y = s.y2 - s.y1;
	const double d2x = (t.x2 - t.x1) * coslat;
	const double d2y = t.y2 - t.y1;
	const double dot = d1x * d2x + d1y * d2y;
	const double cr = d1x * d2y - d1y * d2x;
	return std::atan2(std::fabs(cr), std::fabs(dot)) * 180.0 / M_PI;
}

static bool ways_share_node(const WayNodes& a, const WayNodes& b) {
	const WayNodes& small = a.nodes.size() <= b.nodes.size() ? a : b;
	const WayNodes& big   = a.nodes.size() <= b.nodes.size() ? b : a;
	std::unordered_set<std::uint64_t> set{small.nodes.begin(), small.nodes.end()};
	for (const auto n : big.nodes) {
		if (set.count(n)) {
			return true;
		}
	}
	return false;
}

static std::vector<std::string> split_keys(const std::string& keys) {
	std::vector<std::string> out;
	std::size_t pos = 0;
	while (pos < keys.size()) {
		const std::size_t next = keys.find(',', pos);
		out.push_back(keys.substr(pos, next - pos));
		if (next == std::string::npos) {
			break;
		}
		pos = next + 1;
	}
	return out;
}

// filter items are either a plain tag key ("boundary") or a key=value pair ("man_made=pipeline")
static bool tags_match(const osmium::TagList& tags, const std::vector<std::string>& items) {
	for (const auto& tag : tags) {
		for (const auto& item : items) {
			const std::size_t eq = item.find('=');
			if (eq == std::string::npos) {
				if (item == tag.key()) {
					return true;
				}
			} else if (item.compare(0, eq, tag.key()) == 0 && item.compare(eq + 1, std::string::npos, tag.value()) == 0) {
				return true;
			}
		}
	}
	return false;
}

class LineIndex : public osmium::handler::Handler {

	WayMetaMap* m_metas;
	const std::unordered_set<std::uint64_t>* m_involved;
	const std::unordered_set<std::uint64_t>* m_excluded_ways = nullptr;

public:
	RTree rtree;
	std::vector<WayNodes> ways;
	std::vector<Crossing> crossings;

	std::size_t num_ways = 0;
	std::size_t num_segments = 0;
	std::size_t num_touch = 0;     // intersections rejected by the angle threshold (statistic)
	std::size_t num_nodeskip = 0;  // rejected because of shared nodes (statistic)
	std::size_t num_parallel = 0;  // rejected as near-parallel/collinear (statistic)

	std::size_t num_excluded = 0;   // rejected by the exclude tag key filter (statistic)
	std::size_t num_excluded_rel = 0; // rejected as member of a relation with excluded tag keys (statistic)

	// optional tag key filter; empty = accept all ways
	std::vector<std::string> filter_keys;

	// optional tag key exclusion; any way carrying one of these keys is dropped
	std::vector<std::string> exclude_keys;

	double threshold = 15.0;
	double parallel_tol = 1.0; // below this the crossing counts as adjacent/collinear, not a crossing

	explicit LineIndex(WayMetaMap* metas = nullptr, const std::unordered_set<std::uint64_t>* involved = nullptr,
	                   const std::unordered_set<std::uint64_t>* excluded_ways = nullptr)
	: m_metas(metas), m_involved(involved), m_excluded_ways(excluded_ways) {
	}

	void way(const osmium::Way& way) {
		if (m_metas) { // second pass: metadata for involved ways only
			if (m_involved->count(way.id())) {
				WayMeta wm;
				wm.uid = way.uid();
				wm.changeset = way.changeset();
				wm.user = way.user();
				wm.timestamp = way.timestamp().to_iso();
				wm.tags.reserve(way.tags().size());
				for (const auto& t : way.tags()) {
					wm.tags.emplace_back(t.key(), t.value());
				}
				m_metas->emplace(way.id(), std::move(wm));
			}
			return;
		}

		if (!filter_keys.empty()) {
			if (!tags_match(way.tags(), filter_keys)) {
				return;
			}
		}
		if (!exclude_keys.empty() && tags_match(way.tags(), exclude_keys)) {
			++num_excluded;
			return;
		}
		if (m_excluded_ways && m_excluded_ways->count(way.id())) {
			++num_excluded_rel;
			return;
		}

		WayNodes wn;
		wn.id = way.id();
		wn.nodes.reserve(way.nodes().size());
		for (const auto& nr : way.nodes()) {
			wn.nodes.push_back(nr.positive_ref());
		}
		const std::uint32_t way_idx = static_cast<std::uint32_t>(ways.size());
		ways.push_back(std::move(wn));
		++num_ways;

		std::vector<Segment> cand;
		const std::size_t n = way.nodes().size();
		for (std::size_t k = 1; k < n; ++k) {
			const auto& a = way.nodes()[k - 1];
			const auto& b = way.nodes()[k];
			if (!a.location().valid() || !b.location().valid()) {
				continue;
			}
			Segment s{
				a.location().lon_without_check(),
				a.location().lat_without_check(),
				b.location().lon_without_check(),
				b.location().lat_without_check(),
				way_idx
			};
			++num_segments;

			// every intersecting pair of segments is examined exactly once,
			// when the later segment is inserted
			cand.clear();
			const Box qb = SegmentGetter{}(s);
			rtree.query(bgi::intersects(qb), std::back_inserter(cand));

			for (const auto& t : cand) {
				if (t.way == way_idx) { // self-intersections of one way are ignored
					continue;
				}
				double ix = 0.0;
				double iy = 0.0;
				if (!seg_intersect(s, t, ix, iy)) {
					continue;
				}
				const double angle = acute_angle(s, t, iy);
				if (angle < parallel_tol) { // adjacent areas / duplicated digitized borders
					++num_parallel;
					continue;
				}
				if (angle >= threshold) { // also filters near-parallel overlaps
					++num_touch;
					continue;
				}
				const bool shared = ways_share_node(ways[way_idx], ways[t.way]);
				if (shared) { // junction at a common node, not a geometric crossing
					++num_nodeskip;
					continue;
				}
				// report the shorter of the two crossing segments as geometry
				const auto len2 = [](const Segment& seg) {
					const double dx = seg.x2 - seg.x1;
					const double dy = seg.y2 - seg.y1;
					// longitude compression is irrelevant for the length comparison
					return dx * dx + dy * dy;
				};
				const Segment& shorter = len2(s) <= len2(t) ? s : t;
				const Crossing c{ways[way_idx].id, ways[t.way].id, ix, iy,
					shorter.x1, shorter.y1, shorter.x2, shorter.y2, angle};
				if (!crossings_set.insert(crossing_key(c)).second) {
					continue;
				}
				crossings.push_back(c);
			}

			rtree.insert(s);
		}
	}

private:
	std::unordered_set<std::uint64_t> crossings_set;
};

using json = nlohmann::ordered_json;

// Build one feature's way object; all string escaping is done by nlohmann json
static json way_meta_json(std::uint64_t way_id, const WayMeta* wm) {
	json j;
	j["id"] = way_id;
	if (wm) {
		j["user"] = wm->user;
		j["uid"] = wm->uid;
		j["changeset"] = wm->changeset;
		j["last_modified"] = wm->timestamp;
		json tags = json::object();
		for (const auto& kv : wm->tags) {
			tags[kv.first] = kv.second;
		}
		j["tags"] = std::move(tags);
	}
	return j;
}

static const WayMeta* meta_of(const WayMetaMap& metas, std::uint64_t way_id) {
	const auto it = metas.find(way_id);
	return it == metas.end() ? nullptr : &it->second;
}

// LineString geometry of the crossing, coordinates rounded to the OSM grid (1e7)
static json geometry_json(const Crossing& c) {
	return json{
		{"type", "LineString"},
		{"coordinates", {
			{std::llround(c.sx1 * 1e7) / 1e7, std::llround(c.sy1 * 1e7) / 1e7},
			{std::llround(c.sx2 * 1e7) / 1e7, std::llround(c.sy2 * 1e7) / 1e7}
		}}
	};
}

static json properties_json(const Crossing& c, const WayMetaMap& metas) {
	return json{
		{"way1", way_meta_json(c.way1, meta_of(metas, c.way1))},
		{"way2", way_meta_json(c.way2, meta_of(metas, c.way2))}
	};
}

// FeatureCollection header/trailer are streamed, one feature per line,
// so arbitrarily large crossing sets never buffer up in memory
static void write_geojson(const char* outfile, const std::vector<Crossing>& crossings, const WayMetaMap& metas) {
	std::ofstream out(outfile);
	if (!out) {
		std::cerr << "Error: cannot open " << outfile << " for writing\n";
		std::exit(1);
	}
	out << "{\"type\":\"FeatureCollection\",\"features\":[\n";
	for (std::size_t i = 0; i < crossings.size(); ++i) {
		const auto& c = crossings[i];
		json feature;
		feature["type"] = "Feature";
		feature["geometry"] = geometry_json(c);
		feature["properties"] = json{
			{"angle_deg", std::llround(c.angle * 100) / 100.0},
			{"way1", way_meta_json(c.way1, meta_of(metas, c.way1))},
			{"way2", way_meta_json(c.way2, meta_of(metas, c.way2))}
		};
		out << feature.dump() << (i + 1 < crossings.size() ? ",\n" : "\n");
	}
	out << "]}\n";
}

static void sqlite_fail(sqlite3* db, const char* what) {
	std::cerr << "SQLite error (" << what << "): " << (db ? sqlite3_errmsg(db) : "no database") << '\n';
	std::exit(1);
}

class SqliteWriter {

	sqlite3* db = nullptr;
	sqlite3_stmt* stmt = nullptr;

	void exec(const char* sql, const char* what) {
		char* err = nullptr;
		if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
			std::cerr << "SQLite error (" << what << "): " << (err ? err : sqlite3_errmsg(db)) << '\n';
			std::exit(1);
		}
		sqlite3_free(err);
	}

public:
	SqliteWriter(const char* path) {
		std::remove(path); // overwrite like the GeoJSON writer does
		if (sqlite3_open(path, &db) != SQLITE_OK) {
			sqlite_fail(db, "open");
		}
		sqlite3_enable_load_extension(db, 1);
		if (sqlite3_exec(db, "SELECT load_extension('mod_spatialite');", nullptr, nullptr, nullptr) != SQLITE_OK) {
			std::cerr << "Error: cannot load mod_spatialite (" << sqlite3_errmsg(db) << ")\n";
			std::exit(1);
		}
		exec("SELECT InitSpatialMetaData(1);", "init spatial metadata");
		exec("PRAGMA trusted_schema=ON;", "pragma"); // needed for spatialite GeometryConstraints() on SQLite >= 3.37
		exec("PRAGMA synchronous=OFF", "pragma");
		exec("CREATE TABLE crossings ("
		     "fid INTEGER PRIMARY KEY AUTOINCREMENT, "
		     "angle_deg REAL NOT NULL, "
		     "way1_id INTEGER NOT NULL, "
		     "way2_id INTEGER NOT NULL, "
		     "props TEXT NOT NULL)", "create table");
		exec("SELECT AddGeometryColumn('crossings','geom',4326,'LINESTRING','XY');", "add geometry column");
		exec("SELECT CreateSpatialIndex('crossings','geom');", "create spatial index");
		const char* sql = "INSERT INTO crossings (angle_deg,way1_id,way2_id,props,geom) "
		                  "VALUES (?,?,?,?,SetSRID(GeomFromGeoJSON(?),4326));";
		if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
			sqlite_fail(db, "prepare insert");
		}
		exec("BEGIN", "begin transaction");
	}

	void add(const Crossing& c, const WayMetaMap& metas) {
		const json props = properties_json(c, metas);
		const json geom = geometry_json(c);
		if (sqlite3_reset(stmt) != SQLITE_OK) {
			sqlite_fail(db, "reset insert");
		}
		sqlite3_bind_double(stmt, 1, std::llround(c.angle * 100) / 100.0);
		sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(c.way1));
		sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(c.way2));
		sqlite3_bind_text(stmt, 4, props.dump().c_str(), -1, SQLITE_TRANSIENT);
		sqlite3_bind_text(stmt, 5, geom.dump().c_str(), -1, SQLITE_TRANSIENT);
		if (sqlite3_step(stmt) != SQLITE_DONE) {
			sqlite_fail(db, "insert");
		}
	}

	~SqliteWriter() {
		if (stmt) {
			sqlite3_finalize(stmt);
		}
		if (db) {
			exec("COMMIT", "commit transaction");
			sqlite3_close(db);
		}
	}
};

static void write_spatialite(const char* outfile, const std::vector<Crossing>& crossings, const WayMetaMap& metas) {
	SqliteWriter writer{outfile};
	for (const auto& c : crossings) {
		writer.add(c, metas);
	}
}

// output format is selected by the outfile extension
static bool ends_with_ci(const std::string& s, const std::string& suffix) {
	if (s.size() < suffix.size()) {
		return false;
	}
	for (std::size_t i = 0; i < suffix.size(); ++i) {
		if (std::tolower(static_cast<unsigned char>(s[s.size() - suffix.size() + i])) != suffix[i]) {
			return false;
		}
	}
	return true;
}

int main(int argc, char* argv[]) {
	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "produce help message")
		("infile,i", po::value<std::string>()->required(), "Input file")
		("outfile,o", po::value<std::string>()->default_value("crossings.geojson"), "GeoJSON output file")
		("angle,a", po::value<double>()->default_value(15.0), "Angle threshold in degrees, report crossings below it")
		("parallel,p", po::value<double>()->default_value(1.0), "Parallel tolerance in degrees, skip crossings below it")
		("filter,f", po::value<std::string>(), "Only ways having one of these tag keys (comma separated)")
		("exclude,x", po::value<std::string>(), "Exclude ways carrying one of these tag keys (comma separated)")
	;

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);
	} catch (const boost::program_options::error& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << desc << std::endl;
		std::exit(EXIT_FAILURE);
	}

	const auto start = std::chrono::steady_clock::now();

	osmium::io::File input_file{vm["infile"].as<std::string>()};
	const std::string outfile = vm["outfile"].as<std::string>();
	const double angle_threshold = vm["angle"].as<double>();
	const double parallel_tol = vm["parallel"].as<double>();

	const std::vector<std::string> filter_keys = vm.count("filter") ? split_keys(vm["filter"].as<std::string>()) : std::vector<std::string>{};
	const std::vector<std::string> exclude_keys = vm.count("exclude") ? split_keys(vm["exclude"].as<std::string>()) : std::vector<std::string>{};

	if (angle_threshold <= 0.0 || angle_threshold >= 90.0) {
		std::cerr << "Error: angle threshold must be in (0, 90)\n";
		std::exit(EXIT_FAILURE);
	}
	if (parallel_tol < 0.0 || parallel_tol >= angle_threshold) {
		std::cerr << "Error: parallel tolerance must be in [0, angle threshold)\n";
		std::exit(EXIT_FAILURE);
	}

	// relations carrying excluded tag keys already put their tags on the relation
	// (e.g. boundary=nature_reserve on a multipolygon), so every member way is excluded
	std::unordered_set<std::uint64_t> excluded_relation_ways;
	if (!exclude_keys.empty()) {
		class ExcludedRelationCollector : public osmium::handler::Handler {

			const std::vector<std::string>& m_keys;
			std::unordered_set<std::uint64_t>& m_out;

		public:
			explicit ExcludedRelationCollector(const std::vector<std::string>& keys, std::unordered_set<std::uint64_t>& out)
			: m_keys(keys), m_out(out) {
			}

			void relation(const osmium::Relation& rel) {
				if (!tags_match(rel.tags(), m_keys)) {
					return;
				}
				for (const auto& member : rel.members()) {
					if (member.type() == osmium::item_type::way) {
						m_out.insert(member.positive_ref());
					}
				}
			}
		};

		ExcludedRelationCollector collector{exclude_keys, excluded_relation_ways};
		osmium::io::ReaderWithProgressBar reader{true, input_file, osmium::osm_entity_bits::relation};
		while (osmium::memory::Buffer buffer = reader.read()) {
			osmium::apply(buffer, collector);
		}
		reader.close();
		std::cout << "relation-excluded way ids: " << excluded_relation_ways.size() << '\n';
	}

	LineIndex lineindex{nullptr, nullptr, excluded_relation_ways.empty() ? nullptr : &excluded_relation_ways};
	lineindex.threshold = angle_threshold;
	lineindex.parallel_tol = parallel_tol;
	lineindex.filter_keys = filter_keys;
	lineindex.exclude_keys = exclude_keys;

	index_type index;
	location_handler_type location_handler{index};
	location_handler.ignore_errors();

	// pass 1: build rtree, find crossings
	{
		osmium::io::ReaderWithProgressBar reader{true, input_file};
		while (osmium::memory::Buffer buffer = reader.read()) {
			osmium::apply(buffer, location_handler, lineindex);
		}
		reader.close();
	}

	const auto read_done = std::chrono::steady_clock::now();

	// ways referenced by reported crossings
	std::unordered_set<std::uint64_t> involved;
	involved.reserve(lineindex.crossings.size() * 2);
	for (const auto& c : lineindex.crossings) {
		involved.insert(c.way1);
		involved.insert(c.way2);
	}

	// pass 2: collect metadata for the involved ways only
	WayMetaMap metas;
	{
		LineIndex metahandler{&metas, &involved};
		osmium::io::ReaderWithProgressBar reader{true, input_file};
		while (osmium::memory::Buffer buffer = reader.read()) {
			osmium::apply(buffer, metahandler);
		}
		reader.close();
	}
	metas.rehash(metas.size() * 2); // reclaim slack, cheap since tiny

	if (ends_with_ci(outfile, ".geojson")) {
		write_geojson(outfile.c_str(), lineindex.crossings, metas);
	} else if (ends_with_ci(outfile, ".sqlite") || ends_with_ci(outfile, ".spatialite") || ends_with_ci(outfile, ".db")) {
		write_spatialite(outfile.c_str(), lineindex.crossings, metas);
	} else {
		std::cerr << "Error: unknown output format for " << outfile << " (use .geojson or .sqlite)\n";
		std::exit(EXIT_FAILURE);
	}

	const auto write_done = std::chrono::steady_clock::now();
	const auto secs = [](std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
		return std::chrono::duration_cast<std::chrono::duration<double>>(b - a).count();
	};

	std::cout << "ways:            " << lineindex.num_ways << '\n'
	          << "segments:        " << lineindex.num_segments << '\n'
	          << "crossings:       " << lineindex.crossings.size() << " (angle < " << lineindex.threshold << "°, >= " << lineindex.parallel_tol << "°)\n"
	          << "skipped by angle:" << lineindex.num_touch << '\n'
	          << "skipped parallel:" << lineindex.num_parallel << '\n'
	          << "skipped shared:  " << lineindex.num_nodeskip << '\n'
	          << "ways excluded:   " << lineindex.num_excluded << '\n'
	          << "ways excl rel:   " << lineindex.num_excluded_rel << '\n'
	          << "read:            " << secs(start, read_done) << " s\n"
	          << "metadata pass:   " << secs(read_done, write_done) << " s\n"
	          << "output:          " << outfile << '\n';
}
