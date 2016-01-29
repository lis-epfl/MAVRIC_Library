#!/bin/bash

# save git hash
GITHASH=$(git rev-parse HEAD)

cd ../doc/gh_pages

git init
git config user.name "MavricBot"
git config user.email "bot@mavric.org"
git remote add upstream "https://$GH_TOKEN@github.com/lis-epfl/MAVRIC_Library.git"
git fetch upstream && git reset upstream/gh-pages

touch .

git add -A .

git commit -m "Rebuild doc at ${GITHASH}"
git push -q upstream HEAD:gh-pages