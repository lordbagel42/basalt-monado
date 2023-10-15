#pragma once

#include <cstdint>
#include <set>

static inline std::set<int64_t> debug_ids = {3, 0, 5, 4, 7};

static inline bool is_debug_point(int64_t lmid) { return debug_ids.count(lmid) > 0; }
