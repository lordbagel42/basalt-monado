#!/usr/bin/env bash
##
## BSD 3-Clause License
##
## This file is part of the Basalt project.
## https://gitlab.com/VladyslavUsenko/basalt.git
##
## Copyright (c) 2019-2021, Vladyslav Usenko and Nikolaus Demmel.
## All rights reserved.
##

# Run clang-tidy on all source files in the project.
# Optionally take folder as argument; default is full inlude and src dirs.

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

FOLDER="${1:-$SCRIPT_DIR/../include $SCRIPT_DIR/../src $SCRIPT_DIR/../test/src}"

CLANG_TIDY_COMMANDS="clang-tidy-15"

# find the first available command:
for CMD in $CLANG_TIDY_COMMANDS; do
    if hash $CMD 2>/dev/null; then
        CLANG_TIDY_CMD=$CMD
        break
    fi
done

if [ -z $CLANG_TIDY_CMD ]; then
    echo "clang-tidy not installed..."
    exit 1
fi

# clang tidy check version
MAJOR_VERSION_NEEDED=8

MAJOR_VERSION_DETECTED=`$CLANG_TIDY_CMD -version | sed -n -E 's/.*version ([0-9]+).*/\1/p'`
if [ -z $MAJOR_VERSION_DETECTED ]; then
    echo "Failed to parse major version (`$CLANG_TIDY_CMD -version`)"
    exit 1
fi

echo "clang-tidy version $MAJOR_VERSION_DETECTED (`$CLANG_TIDY_CMD -version`)"

if [ $MAJOR_VERSION_DETECTED -lt $MAJOR_VERSION_NEEDED ]; then
    echo "Looks like your clang tidy is too old; need at least version $MAJOR_VERSION_NEEDED"
    exit 1
fi

find $FOLDER -iname "*.?pp" -or -iname "*.h" | xargs $CLANG_TIDY_CMD -verbose -i
