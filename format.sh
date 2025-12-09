#!/bin/bash
# Formats all src files in the project

INCLUDE_DIR=include/

for file in $(find $INCLUDE_DIR | grep -e ".cpp" -e ".h"); do
    echo "Formatting $file..."
    clang-format -i $file
done
