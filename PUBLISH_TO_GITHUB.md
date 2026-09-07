# Publish this reproducibility package on GitHub

Suggested repository name: `cores3-edge-ai-visual-inspection`

1. Create a new public GitHub repository with that name.
2. Upload/commit the contents of this directory.
3. Replace `REPLACE-WITH-ACCOUNT` in `CITATION.cff` with the GitHub user or organization name.
4. In `paper/ojcs_cores3_yolo26.tex`, replace the value of `\ProjectRepository` with the same permanent URL.
5. Recompile the paper using `cd paper && sh compile.sh`.
6. Run `python scripts/verify_dataset.py` and `python scripts/summarize_runtime.py` before creating a release.
7. Create a tagged release (for example `v1.0-paper`) and, if desired, archive it with Zenodo to obtain a DOI.

The firmware has been sanitized: Wi-Fi credentials are loaded from `firmware/src/project_config.h`, which is intentionally excluded by `.gitignore`. Copy `project_config.example.h` before local compilation.
