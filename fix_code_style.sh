#!/bin/bash

# Change the working directory to the script's directory
SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd $SCRIPTPATH

# Find all files with ".hpp" or ".cpp" extensions in the current directory and subdirectories,
# excluding certain paths (./.git)
find . -iname "*.hpp" -o -iname "*.cpp" | \
grep -Ev "^(./.git)" | \

# Format the files according to the rules specified in .clang-format
xargs clang-format -i
