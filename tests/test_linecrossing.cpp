/*
 * Copyright 2026 - Florian Lohoff <f@zz.de>
 * Unit tests; CC0 like the main program.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <osmium/builder/osm_object_builder.hpp>

#include <filesystem>
#include <set>
#include <sstream>

#include "linecrossing.hpp"

using Catch::Matchers::WithinAbs;

namespace {

const std::string kData = std::string(TEST_DATA_DIR) + "/crossings.osm.xml";

// per-build scratch dir; removed again when the test ends, even if it fails
struct TmpFile {
	std::string path;
	explicit TmpFile(const std::string& name) : path(std::string(TEST_TMP_DIR) + "/" + name) {
		std::filesystem::create_directories(TEST_TMP_DIR);
		std::filesystem::remove(path);
	}
	~TmpFile() {
		std::filesystem::remove(path);
	}
};

json read_json(const std::string& path) {
	std::ifstream in(path);
	return json::parse(in);
}

// owns a buffer with a single TagList
struct Tags {
	osmium::memory::Buffer buf{1024};
	explicit Tags(std::initializer_list<std::pair<const char*, const char*>> kv) {
		{
			osmium::builder::TagListBuilder b{buf};
			for (const auto& p : kv) {
				b.add_tag(p.first, p.second);
			}
		}
		buf.commit();
	}
	const osmium::TagList& list() const {
		return buf.get<osmium::TagList>(0);
	}
};

Segment seg(double x1, double y1, double x2, double y2, std::uint32_t way = 0) {
	return Segment{x1, y1, x2, y2, way};
}

using Pair = std::pair<std::uint64_t, std::uint64_t>;

std::set<Pair> pairs(const LineIndex& li) {
	std::set<Pair> out;
	for (const auto& c : li.crossings) {
		out.emplace(std::min(c.way1, c.way2), std::max(c.way1, c.way2));
	}
	return out;
}

std::unique_ptr<Analysis> run(const Options& opt) {
	auto res = std::make_unique<Analysis>();
	analyze(osmium::io::File{kData}, opt, *res);
	return res;
}

} // namespace

TEST_CASE("mix64 is deterministic and spreads", "[hash]") {
	CHECK(mix64(0) == mix64(0));
	CHECK(mix64(1) != mix64(2));
	CHECK(mix64(0) == 0xe220a8397b1dcdafULL); // known SplitMix64 first output
}

TEST_CASE("crossing_key", "[hash]") {
	const Crossing a{1, 2, 8.5, 52.5, 0, 0, 0, 0, 10};
	SECTION("independent of way order") {
		Crossing b = a;
		std::swap(b.way1, b.way2);
		CHECK(crossing_key(a) == crossing_key(b));
	}
	SECTION("collapses points within the ~0.1m grid") {
		Crossing b = a;
		b.x += 1e-7;
		b.y -= 1e-7;
		CHECK(crossing_key(a) == crossing_key(b));
	}
	SECTION("separates distant points") {
		Crossing bx = a;
		bx.x += 1e-5;
		CHECK(crossing_key(a) != crossing_key(bx));
		Crossing by = a;
		by.y += 1e-5;
		CHECK(crossing_key(a) != crossing_key(by));
	}
	SECTION("separates way pairs") {
		Crossing b = a;
		b.way2 = 3;
		CHECK(crossing_key(a) != crossing_key(b));
		Crossing c = a;
		c.way1 = 3;
		CHECK(crossing_key(a) != crossing_key(c));
	}
	SECTION("(1,2) differs from (2,2) and (1,1)") {
		Crossing b = a;
		b.way1 = 2;
		Crossing c = a;
		c.way2 = 1;
		CHECK(crossing_key(a) != crossing_key(b));
		CHECK(crossing_key(a) != crossing_key(c));
	}
}

TEST_CASE("seg_intersect", "[geometry]") {
	double ix = -1;
	double iy = -1;

	SECTION("proper crossing reports the point") {
		REQUIRE(seg_intersect(seg(0, 0, 2, 2), seg(0, 2, 2, 0), ix, iy));
		CHECK_THAT(ix, WithinAbs(1.0, 1e-12));
		CHECK_THAT(iy, WithinAbs(1.0, 1e-12));
	}
	SECTION("same result with swapped arguments") {
		REQUIRE(seg_intersect(seg(0, 2, 2, 0), seg(0, 0, 2, 2), ix, iy));
		CHECK_THAT(ix, WithinAbs(1.0, 1e-12));
		CHECK_THAT(iy, WithinAbs(1.0, 1e-12));
	}
	SECTION("non-centered point") {
		REQUIRE(seg_intersect(seg(0, 0, 4, 0), seg(1, -1, 1, 3), ix, iy));
		CHECK_THAT(ix, WithinAbs(1.0, 1e-12));
		CHECK_THAT(iy, WithinAbs(0.0, 1e-12));
	}
	SECTION("parallel and collinear are rejected") {
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(0, 1, 2, 1), ix, iy));
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(1, 0, 3, 0), ix, iy));
	}
	SECTION("endpoint touching is allowed") {
		REQUIRE(seg_intersect(seg(0, 0, 2, 0), seg(2, 0, 3, 1), ix, iy));
		CHECK_THAT(ix, WithinAbs(2.0, 1e-12));
		REQUIRE(seg_intersect(seg(0, 0, 2, 0), seg(1, 0, 1, 5), ix, iy));
		CHECK_THAT(iy, WithinAbs(0.0, 1e-12));
	}
	// each of the four range checks, for both orientations (sign of denom)
	SECTION("misses beyond each end") {
		// t is a vertical line; s is the horizontal one
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(3, -1, 3, 1), ix, iy));  // past s end   (r > denom)
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(-1, -1, -1, 1), ix, iy)); // before s     (r < 0)
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(1, 1, 1, 2), ix, iy));   // above t      (u < 0 / > denom)
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(1, -2, 1, -1), ix, iy)); // below t
		// mirrored so that denom has the other sign
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(3, 1, 3, -1), ix, iy));
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(-1, 1, -1, -1), ix, iy));
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(1, 2, 1, 1), ix, iy));
		CHECK_FALSE(seg_intersect(seg(0, 0, 2, 0), seg(1, -1, 1, -2), ix, iy));
		// swapped roles: s vertical
		CHECK_FALSE(seg_intersect(seg(3, -1, 3, 1), seg(0, 0, 2, 0), ix, iy));
		CHECK_FALSE(seg_intersect(seg(-1, -1, -1, 1), seg(0, 0, 2, 0), ix, iy));
		CHECK_FALSE(seg_intersect(seg(1, 1, 1, 2), seg(0, 0, 2, 0), ix, iy));
		CHECK_FALSE(seg_intersect(seg(1, -2, 1, -1), seg(0, 0, 2, 0), ix, iy));
		CHECK_FALSE(seg_intersect(seg(3, 1, 3, -1), seg(0, 0, 2, 0), ix, iy));
		CHECK_FALSE(seg_intersect(seg(-1, 1, -1, -1), seg(0, 0, 2, 0), ix, iy));
		CHECK_FALSE(seg_intersect(seg(1, 2, 1, 1), seg(0, 0, 2, 0), ix, iy));
		CHECK_FALSE(seg_intersect(seg(1, -1, 1, -2), seg(0, 0, 2, 0), ix, iy));
	}
	SECTION("both orientations intersect") {
		CHECK(seg_intersect(seg(0, 0, 2, 0), seg(1, -1, 1, 1), ix, iy));
		CHECK(seg_intersect(seg(0, 0, 2, 0), seg(1, 1, 1, -1), ix, iy));
		CHECK(seg_intersect(seg(1, 1, 1, -1), seg(0, 0, 2, 0), ix, iy));
		CHECK(seg_intersect(seg(1, -1, 1, 1), seg(0, 0, 2, 0), ix, iy));
	}
}

TEST_CASE("acute_angle", "[geometry]") {
	SECTION("right angle") {
		CHECK_THAT(acute_angle(seg(0, 0, 1, 0), seg(0, 0, 0, 1), 0), WithinAbs(90.0, 1e-9));
	}
	SECTION("45 degrees at the equator") {
		CHECK_THAT(acute_angle(seg(0, 0, 1, 0), seg(0, 0, 1, 1), 0), WithinAbs(45.0, 1e-9));
	}
	SECTION("always acute, regardless of direction") {
		CHECK_THAT(acute_angle(seg(0, 0, 1, 0), seg(1, 1, 0, 0), 0), WithinAbs(45.0, 1e-9));
		CHECK_THAT(acute_angle(seg(1, 0, 0, 0), seg(0, 0, 1, 1), 0), WithinAbs(45.0, 1e-9));
		CHECK_THAT(acute_angle(seg(0, 0, 1, 1), seg(0, 0, 1, 0), 0), WithinAbs(45.0, 1e-9));
		CHECK_THAT(acute_angle(seg(0, 0, 1, 0), seg(0, 1, 1, 0), 0), WithinAbs(45.0, 1e-9));
	}
	SECTION("parallel") {
		CHECK_THAT(acute_angle(seg(0, 0, 1, 1), seg(5, 5, 7, 7), 10), WithinAbs(0.0, 1e-9));
	}
	SECTION("longitude is compressed by cos(lat)") {
		// at 60N cos = 0.5: dx 1 -> 0.5, so (1,1) vs (1,0) is atan(1/0.5)
		CHECK_THAT(acute_angle(seg(0, 0, 1, 1), seg(0, 0, 1, 0), 60), WithinAbs(63.4349488, 1e-6));
		CHECK_THAT(acute_angle(seg(0, 0, 1, 0), seg(0, 0, 1, 1), 60), WithinAbs(63.4349488, 1e-6));
		// compression affects both vectors: (1,0) vs (0,1) stays 90
		CHECK_THAT(acute_angle(seg(0, 0, 1, 0), seg(0, 0, 0, 1), 60), WithinAbs(90.0, 1e-9));
	}
	SECTION("negative latitude is symmetric") {
		CHECK_THAT(acute_angle(seg(0, 0, 1, 1), seg(0, 0, 1, 0), -60), WithinAbs(63.4349488, 1e-6));
	}
}

TEST_CASE("ways_share_node", "[geometry]") {
	const WayNodes a{1, {1, 2, 3}};
	const WayNodes b{2, {3, 4}};
	const WayNodes c{3, {7, 8, 9, 10}};
	CHECK(ways_share_node(a, b));
	CHECK(ways_share_node(b, a));
	CHECK_FALSE(ways_share_node(a, c));
	CHECK_FALSE(ways_share_node(c, a));
	CHECK(ways_share_node(WayNodes{4, {10}}, c)); // small side hit in the big side
	CHECK(ways_share_node(c, WayNodes{4, {10}}));
	CHECK(ways_share_node(WayNodes{5, {1, 2, 3}}, WayNodes{6, {1, 2, 3}})); // equal sizes
	SECTION("long ways use the hash set path") {
		WayNodes l1{10, {}};
		WayNodes l2{11, {}};
		for (std::uint64_t i = 0; i < 100; ++i) {
			l1.nodes.push_back(1000 + i);
			l2.nodes.push_back(2000 + i);
		}
		CHECK_FALSE(ways_share_node(l1, l2));
		l2.nodes.push_back(1099);
		CHECK(ways_share_node(l1, l2));
		CHECK(ways_share_node(l2, l1));
	}
	CHECK_FALSE(ways_share_node(WayNodes{}, a));
	CHECK_FALSE(ways_share_node(a, WayNodes{}));
}

TEST_CASE("split_keys", "[tags]") {
	using V = std::vector<std::string>;
	CHECK(split_keys("") == V{});
	CHECK(split_keys("a") == V{"a"});
	CHECK(split_keys("a,b") == V{"a", "b"});
	CHECK(split_keys("a,b,c") == V{"a", "b", "c"});
	CHECK(split_keys("a,,b") == V{"a", "", "b"});
	CHECK(split_keys("a,") == V{"a"});
	CHECK(split_keys("man_made=pipeline,boundary") == V{"man_made=pipeline", "boundary"});
}

TEST_CASE("tags_match", "[tags]") {
	const Tags tags{{"highway", "primary"}, {"man_made", "pipeline"}};
	const auto& tl = tags.list();
	CHECK(tags_match(tl, {"highway"}));
	CHECK(tags_match(tl, {"railway", "highway"}));
	CHECK_FALSE(tags_match(tl, {"railway"}));
	CHECK_FALSE(tags_match(tl, {}));
	CHECK_FALSE(tags_match(tl, {"high"}));          // no prefix match on keys
	CHECK_FALSE(tags_match(tl, {"highway2"}));
	CHECK(tags_match(tl, {"man_made=pipeline"}));
	CHECK(tags_match(tl, {"highway=primary"}));
	CHECK_FALSE(tags_match(tl, {"man_made=power"}));
	CHECK_FALSE(tags_match(tl, {"man_made=pipelines"})); // value must match fully
	CHECK_FALSE(tags_match(tl, {"man_made=pipe"}));
	CHECK_FALSE(tags_match(tl, {"man=pipeline"}));     // key must match fully
	CHECK_FALSE(tags_match(tl, {"man_made_=pipeline"}));
	CHECK_FALSE(tags_match(tl, {"railway=pipeline"}));

	const Tags none{};
	CHECK_FALSE(tags_match(none.list(), {"highway"}));
}

TEST_CASE("ends_with_ci", "[io]") {
	CHECK(ends_with_ci("a.geojson", ".geojson"));
	CHECK(ends_with_ci("a.GeoJSON", ".geojson"));
	CHECK(ends_with_ci(".geojson", ".geojson"));
	CHECK_FALSE(ends_with_ci("geojson", ".geojson"));
	CHECK_FALSE(ends_with_ci("a.geojsoN2", ".geojson"));
	CHECK_FALSE(ends_with_ci("a.xeojson", ".geojson"));
	CHECK_FALSE(ends_with_ci("a.geojsox", ".geojson"));
	CHECK(ends_with_ci("x", ""));
}

TEST_CASE("json helpers", "[json]") {
	SECTION("way_meta_json without meta only has the id") {
		const json j = way_meta_json(5, nullptr);
		CHECK(j.size() == 1);
		CHECK(j["id"] == 5);
	}
	SECTION("way_meta_json with meta") {
		WayMeta wm;
		wm.user = "u\"x";
		wm.uid = 7;
		wm.changeset = 9;
		wm.timestamp = "2024-01-01T00:00:00Z";
		wm.tags = {{"a", "1"}, {"b", "2"}};
		const json j = way_meta_json(5, &wm);
		CHECK(j["id"] == 5);
		CHECK(j["user"] == "u\"x");
		CHECK(j["uid"] == 7);
		CHECK(j["changeset"] == 9);
		CHECK(j["last_modified"] == "2024-01-01T00:00:00Z");
		CHECK(j["tags"] == json{{"a", "1"}, {"b", "2"}});
	}
	SECTION("meta_of") {
		WayMetaMap m;
		m[3] = WayMeta{};
		CHECK(meta_of(m, 3) == &m.at(3));
		CHECK(meta_of(m, 4) == nullptr);
	}
	SECTION("geometry rounds to 1e-7") {
		const Crossing c{1, 2, 0, 0, 8.123456789, 52.987654321, 8.5, 52.25, 3};
		const json g = geometry_json(c);
		CHECK(g["type"] == "LineString");
		CHECK_THAT(g["coordinates"][0][0].get<double>(), WithinAbs(8.1234568, 1e-12));
		CHECK_THAT(g["coordinates"][0][1].get<double>(), WithinAbs(52.9876543, 1e-12));
		CHECK_THAT(g["coordinates"][1][0].get<double>(), WithinAbs(8.5, 1e-12));
		CHECK_THAT(g["coordinates"][1][1].get<double>(), WithinAbs(52.25, 1e-12));
	}
	SECTION("properties_json uses meta of the right way") {
		WayMetaMap m;
		m[1].user = "one";
		m[2].user = "two";
		const Crossing c{1, 2, 0, 0, 0, 0, 0, 0, 3};
		const json p = properties_json(c, m);
		CHECK(p["way1"]["user"] == "one");
		CHECK(p["way2"]["user"] == "two");
		CHECK(p["way1"]["id"] == 1);
		CHECK(p["way2"]["id"] == 2);
	}
}

TEST_CASE("analyze with default options", "[integration]") {
	const auto res = run(Options{});
	const auto& li = res->lineindex;

	const std::set<Pair> expected{{1, 2}, {9, 10}, {11, 12}, {14, 15}, {16, 17}};
	CHECK(pairs(li) == expected);
	CHECK(li.crossings.size() == 5); // (14,15) deduplicated
	CHECK(li.num_touch == 1);        // 3/4 at 45 degrees
	CHECK(li.num_parallel == 1);     // 5/6
	CHECK(li.num_nodeskip == 2);     // 7/8, counted per segment pair
	CHECK(li.num_excluded == 0);
	CHECK(li.num_excluded_rel == 0);
	CHECK(li.num_ways == 18);
	CHECK(res->excluded_relation_ways == 0);

	for (const auto& c : li.crossings) {
		CHECK(c.angle >= 1.0);
		CHECK(c.angle < 15.0);
		CHECK(c.way1 != c.way2);
	}
}

TEST_CASE("analyze counts segments and skips invalid locations", "[integration]") {
	const auto res = run(Options{});
	// 18 ways with one segment each, except 7 (2), 13 (4), 15 (2); way 18 has none (node 99999 missing)
	CHECK(res->lineindex.num_segments == 17 + 1 + 3 + 1);
}

TEST_CASE("analyze reports the expected angle", "[integration]") {
	const auto res = run(Options{});
	bool found = false;
	for (const auto& c : res->lineindex.crossings) {
		if (std::min(c.way1, c.way2) == 1) {
			found = true;
			CHECK_THAT(c.angle, WithinAbs(10.0, 0.05));
			CHECK_THAT(c.y, WithinAbs(52.0, 1e-6));
			CHECK_THAT(c.x, WithinAbs(8.0, 1e-6));
		}
	}
	CHECK(found);
}

TEST_CASE("analyze exclude keys", "[integration]") {
	Options opt;
	opt.exclude_keys = {"boundary"};
	const auto res = run(opt);
	const auto& li = res->lineindex;
	const std::set<Pair> expected{{1, 2}, {14, 15}, {16, 17}};
	CHECK(pairs(li) == expected);
	CHECK(li.num_excluded == 1);          // way 9
	CHECK(li.num_excluded_rel == 1);      // way 11 via relation
	CHECK(res->excluded_relation_ways == 1);
	CHECK(li.num_ways == 16);
}

TEST_CASE("analyze exclude key=value", "[integration]") {
	Options opt;
	opt.exclude_keys = {"boundary=administrative"};
	const auto res = run(opt);
	// the relation has boundary=nature_reserve, so way 11 stays
	const std::set<Pair> expected{{1, 2}, {11, 12}, {14, 15}, {16, 17}};
	CHECK(pairs(res->lineindex) == expected);
	CHECK(res->excluded_relation_ways == 0);
}

TEST_CASE("analyze filter keys", "[integration]") {
	Options opt;
	opt.filter_keys = {"highway"};
	const auto res = run(opt);
	const std::set<Pair> expected{{14, 15}};
	CHECK(pairs(res->lineindex) == expected);
	CHECK(res->lineindex.num_ways == 13);
}

TEST_CASE("analyze filter and exclude combine", "[integration]") {
	Options opt;
	opt.filter_keys = {"highway", "boundary"};
	opt.exclude_keys = {"boundary"};
	const auto res = run(opt);
	CHECK(res->lineindex.num_excluded == 1);
	CHECK(pairs(res->lineindex) == std::set<Pair>{{14, 15}});
}

TEST_CASE("analyze angle window", "[integration]") {
	SECTION("wider upper bound reports the 45 degree crossing") {
		Options opt;
		opt.angle = 50.0;
		const auto res = run(opt);
		CHECK(pairs(res->lineindex).count({3, 4}) == 1);
		CHECK(res->lineindex.num_touch == 0);
	}
	SECTION("upper bound is exclusive") {
		Options opt;
		opt.angle = 10.0;
		const auto res = run(opt);
		CHECK(pairs(res->lineindex).count({1, 2}) == 0);
	}
	SECTION("zero parallel tolerance reports near parallel") {
		Options opt;
		opt.parallel = 0.0;
		const auto res = run(opt);
		CHECK(pairs(res->lineindex).count({5, 6}) == 1);
		CHECK(res->lineindex.num_parallel == 0);
	}
	SECTION("larger parallel tolerance drops 10 degree crossings") {
		Options opt;
		opt.parallel = 11.0;
		const auto res = run(opt);
		CHECK(res->lineindex.crossings.empty());
		CHECK(res->lineindex.num_parallel == 9); // per segment pair, incl. 5/6
	}
}

TEST_CASE("self crossings are ignored", "[integration]") {
	const auto res = run(Options{});
	for (const auto& c : res->lineindex.crossings) {
		CHECK(c.way1 != 13);
		CHECK(c.way2 != 13);
	}
}

TEST_CASE("metadata only for involved ways", "[integration]") {
	const auto res = run(Options{});
	const auto& m = res->metas;
	CHECK(m.size() == 10);
	CHECK(m.count(3) == 0);
	CHECK(m.count(13) == 0);

	const auto& w1 = m.at(1);
	CHECK(w1.user == "Jörg \"J\" <x>");
	CHECK(w1.uid == 4711);
	CHECK(w1.changeset == 1001);
	CHECK(w1.timestamp == "2024-05-01T12:00:00Z");
	REQUIRE(w1.tags.size() == 2);
	CHECK(w1.tags[0] == std::make_pair(std::string("highway"), std::string("primary")));
	CHECK(w1.tags[1].first == "name");

	const auto& w2 = m.at(2);
	CHECK(w2.user == "bahn");
	CHECK(w2.uid == 42);
	CHECK(w2.changeset == 1002);
	CHECK(w2.tags.size() == 1);

	CHECK(m.at(11).tags.empty());
}

TEST_CASE("write_geojson", "[output]") {
	const auto res = run(Options{});
	const TmpFile tmp{"out.geojson"};
	const std::string& path = tmp.path;
	write_geojson(path.c_str(), res->lineindex.crossings, res->metas);
	const json fc = read_json(path);

	CHECK(fc["type"] == "FeatureCollection");
	REQUIRE(fc["features"].size() == 5);
	bool found = false;
	for (const auto& f : fc["features"]) {
		CHECK(f["type"] == "Feature");
		CHECK(f["geometry"]["type"] == "LineString");
		CHECK(f["geometry"]["coordinates"].size() == 2);
		const double a = f["properties"]["angle_deg"].get<double>();
		CHECK_THAT(a * 100, WithinAbs(std::round(a * 100), 1e-6)); // two decimals
		if (f["properties"]["way1"]["id"] == 1 || f["properties"]["way2"]["id"] == 1) {
			found = true;
			const json& w = f["properties"]["way1"]["id"] == 1 ? f["properties"]["way1"] : f["properties"]["way2"];
			CHECK(w["user"] == "Jörg \"J\" <x>");
			CHECK(w["tags"]["name"] == "Quote \"Road\" \\ Ä");
			CHECK_THAT(a, WithinAbs(10.0, 0.05));
		}
	}
	CHECK(found);
}

TEST_CASE("write_geojson with no crossings is valid", "[output]") {
	const TmpFile tmp{"empty.geojson"};
	const std::string& path = tmp.path;
	write_geojson(path.c_str(), {}, {});
	const json fc = read_json(path);
	CHECK(fc["type"] == "FeatureCollection");
	CHECK(fc["features"].empty());
}

TEST_CASE("write_geojson separates features by commas", "[output]") {
	const TmpFile tmp{"lines.geojson"};
	const std::string& path = tmp.path;
	const Crossing c{1, 2, 0, 0, 8, 52, 8.1, 52.1, 5};
	write_geojson(path.c_str(), {c, c, c}, {});
	std::ifstream in(path);
	std::string line;
	std::vector<std::string> lines;
	while (std::getline(in, line)) {
		lines.push_back(line);
	}
	REQUIRE(lines.size() == 5);
	CHECK(lines[1].back() == ',');
	CHECK(lines[2].back() == ',');
	CHECK(lines[3].back() == '}'); // last feature, no trailing comma
}

namespace {
bool spatialite_available() {
	sqlite3* db = nullptr;
	sqlite3_open(":memory:", &db);
	sqlite3_enable_load_extension(db, 1);
	const bool ok = sqlite3_exec(db, "SELECT load_extension('mod_spatialite');", nullptr, nullptr, nullptr) == SQLITE_OK;
	sqlite3_close(db);
	return ok;
}
} // namespace

TEST_CASE("write_spatialite", "[output]") {
	if (!spatialite_available()) {
		SKIP("mod_spatialite not available");
	}
	const auto res = run(Options{});
	const TmpFile tmp{"out.sqlite"};
	const std::string& path = tmp.path;
	write_spatialite(path.c_str(), res->lineindex.crossings, res->metas);

	sqlite3* db = nullptr;
	REQUIRE(sqlite3_open(path.c_str(), &db) == SQLITE_OK);
	sqlite3_enable_load_extension(db, 1);
	REQUIRE(sqlite3_exec(db, "SELECT load_extension('mod_spatialite');", nullptr, nullptr, nullptr) == SQLITE_OK);

	const auto scalar = [&](const char* sql) {
		sqlite3_stmt* st = nullptr;
		REQUIRE(sqlite3_prepare_v2(db, sql, -1, &st, nullptr) == SQLITE_OK);
		REQUIRE(sqlite3_step(st) == SQLITE_ROW);
		const std::string v = reinterpret_cast<const char*>(sqlite3_column_text(st, 0));
		sqlite3_finalize(st);
		return v;
	};
	CHECK(scalar("SELECT count(*) FROM crossings") == "5");
	CHECK(scalar("SELECT count(*) FROM crossings WHERE way1_id=1 OR way2_id=1") == "1");
	CHECK(scalar("SELECT DISTINCT ST_SRID(geom) FROM crossings") == "4326");
	CHECK(scalar("SELECT DISTINCT GeometryType(geom) FROM crossings") == "LINESTRING");
	CHECK(scalar("SELECT count(*) FROM crossings WHERE angle_deg >= 1 AND angle_deg < 15") == "5");
	const auto props = json::parse(scalar("SELECT props FROM crossings WHERE way1_id=1 OR way2_id=1"));
	CHECK(props.contains("way1"));
	CHECK(props.contains("way2"));
	sqlite3_close(db);
}

TEST_CASE("write_spatialite overwrites an existing file", "[output]") {
	if (!spatialite_available()) {
		SKIP("mod_spatialite not available");
	}
	const TmpFile tmp{"overwrite.sqlite"};
	const std::string& path = tmp.path;
	const Crossing c{1, 2, 0, 0, 8, 52, 8.1, 52.1, 5};
	write_spatialite(path.c_str(), {c, c}, {});
	write_spatialite(path.c_str(), {c}, {});

	sqlite3* db = nullptr;
	REQUIRE(sqlite3_open(path.c_str(), &db) == SQLITE_OK);
	sqlite3_stmt* st = nullptr;
	REQUIRE(sqlite3_prepare_v2(db, "SELECT count(*), min(angle_deg) FROM crossings", -1, &st, nullptr) == SQLITE_OK);
	REQUIRE(sqlite3_step(st) == SQLITE_ROW);
	CHECK(sqlite3_column_int(st, 0) == 1);
	CHECK_THAT(sqlite3_column_double(st, 1), WithinAbs(5.0, 1e-9));
	sqlite3_finalize(st);
	sqlite3_close(db);
}
