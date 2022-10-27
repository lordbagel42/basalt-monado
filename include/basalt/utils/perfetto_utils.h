#pragma once

#include <perfetto.h>

PERFETTO_DEFINE_CATEGORIES(
    perfetto::Category("pipeline").SetDescription("Main pipeline stages"),
    perfetto::Category("other").SetDescription(
        "Other category just to have two"));
