# !/usr/bin/env bash

rm -rf autoware_map
mkdir -p autoware_map

gdown -O autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
unzip -d autoware_map autoware_map/sample-map-planning.zip

rm autoware_map/sample-map-planning.zip


