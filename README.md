# Micromouse Algorithms

Year2 source archive by dinhtri-dev. Original workspace files are unchanged.

## Contents and status

Based on mackorone/mms-python. Local additions include Flood Fill, A*, returning to start and an experimental 45-degree path. The 45-degree experiment requires compatible simulator commands and is not hardware-validated. UCI kit files remain references rather than this repository source.

## Run / inspect

Configure the mms simulator with this directory and python "flood fill and A quay 90.py" for the 90-degree variant, or python Main.py for the experimental variant.

Install only this project's listed dependencies in a separate virtual environment. Model binaries, large datasets, private configuration and generated output are excluded. Check DATA_AND_MODELS.md when present.

## Archive validation

See ARCHIVE_STATUS.md and SECURITY_REVIEW.md for the exact publication scope, tests and remaining limitations. A successful source-archive check does not certify production deployment, firmware flashing, model accuracy or CAD geometry.

## Future work

The consolidated Google document records project-specific fixes, missing functions and suggested features. This commit focuses on reproducible archiving, not implementing that roadmap.

## Directory guide

- `__pycache__`

## Tests actually run

Python source compiled without executing model training. virtual 6x6 open maze, explore/return/A*.

## Upstream

Fork of https://github.com/mackorone/mms-python. Original README is retained as README_UPSTREAM.md, and upstream Git history is preserved. Local modifications are separate archive commits.
