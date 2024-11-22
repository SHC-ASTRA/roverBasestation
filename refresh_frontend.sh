#!/bin/env bash

last_path=$(pwd)

. scripts/build_yarn.sh
cd $last_path

echo "Frontend refresh complete!"