#include <algorithm>
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
	double x;
	double y;
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
			bool keep = false;
			for (const auto& tag : way.tags()) {
				if (std::find(filter_keys.begin(), filter_keys.end(), tag.key()) != filter_keys.end()) {
					keep = true;
					break;
				}
			}
			if (!keep) {
				return;
			}
		}
		if (!exclude_keys.empty()) {
			for (const auto& tag : way.tags()) {
				if (std::find(exclude_keys.begin(), exclude_keys.end(), tag.key()) != exclude_keys.end()) {
					++num_excluded;
					return;
				}
			}
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
				const Crossing c{ways[way_idx].id, ways[t.way].id, ix, iy, angle};
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
		feature["geometry"] = json{
			{"type", "Point"},
			{"coordinates", {c.x, c.y}}
		};
		feature["properties"] = json{
			{"angle_deg", c.angle},
			{"way1", way_meta_json(c.way1, metas.count(c.way1) ? &metas.at(c.way1) : nullptr)},
			{"way2", way_meta_json(c.way2, metas.count(c.way2) ? &metas.at(c.way2) : nullptr)}
		};
		out << feature.dump() << (i + 1 < crossings.size() ? ",\n" : "\n");
	}
	out << "]}\n";
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
				for (const auto& tag : rel.tags()) {
					if (std::find(m_keys.begin(), m_keys.end(), tag.key()) == m_keys.end()) {
						continue;
					}
					for (const auto& member : rel.members()) {
						if (member.type() == osmium::item_type::way) {
							m_out.insert(member.positive_ref());
						}
					}
					break;
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

	write_geojson(outfile.c_str(), lineindex.crossings, metas);

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
