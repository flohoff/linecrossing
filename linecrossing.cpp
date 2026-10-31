/*
 * Copyright 2026 - Florian Lohoff <f@zz.de>
 *
 * As this tool has mostly been "created" by an LLM i would consider this all be mostly
 * non copyrightable in most jurisdications. In doubt i put all my contributions under the
 * Creative Commons CC0.
 *
 */

#include "linecrossing.hpp"

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

	Options opt;
	opt.angle = angle_threshold;
	opt.parallel = parallel_tol;
	opt.filter_keys = filter_keys;
	opt.exclude_keys = exclude_keys;
	opt.progress = true;

	Analysis res;
	analyze(input_file, opt, res);
	LineIndex& lineindex = res.lineindex;
	const WayMetaMap& metas = res.metas;
	const auto read_done = res.read_done;
	if (!exclude_keys.empty()) {
		std::cout << "relation-excluded way ids: " << res.excluded_relation_ways << '\n';
	}

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
