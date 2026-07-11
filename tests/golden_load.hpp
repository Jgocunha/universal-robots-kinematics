// golden_load.hpp — shared helper: read a golden file from the compiled-in golden dir.
#pragma once

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "json_mini.hpp"
#include "golden_config.hpp"

#ifndef GOLDEN_DIR
#define GOLDEN_DIR "."
#endif

namespace goldenload
{
	inline std::string readFile(const std::string& name)
	{
		const std::string path = std::string(GOLDEN_DIR) + "/" + name;
		std::ifstream is(path, std::ios::binary);
		if (!is)
			throw std::runtime_error("cannot open golden file: " + path);
		std::ostringstream ss;
		ss << is.rdbuf();
		return ss.str();
	}

	inline jsonmini::Value load(const std::string& name)
	{
		return jsonmini::parse(readFile(name));
	}
} // namespace goldenload
