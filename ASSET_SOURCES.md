# Asset provenance

## Website template

- Copied from `siren-bench-site` (commit `f944ba5`): `static/` (Bulma, Font Awesome, `index.js` unchanged), `swl-primary.svg`, `.nojekyll`, `TEMPLATE_ATTRIBUTION.md`.
- `static/css/index.css`: SIREN stylesheet with the accent retuned (red-orange `#c8452b` → teal `#13806d`), `.siren` renamed `.sf3d`, and a SceneFactory-3D block appended (result figures, equation card, delta/range table cells).
- `assets/images/mark.svg`, `favicon.svg`: authored for this page (wheel over a terrain profile).

## Content

All text and numbers are transcribed from the Overleaf manuscript clone
(`astraexplore/manuscript/6ab615f26a16851aaf4fcd22`, commit `c3d5058`): abstract,
`sections/system.tex`, `experiments.tex`, `results.tex`, `appendix.tex`, and the
generated tables `results/final_table.tex` and `results/final_scaling_table.tex`.

## Figures

- `overview.webp` ← `figures/fig1_matched_eval.png` (paper Fig. 1)
- `pipeline.webp` ← `figures/fig2_training_eval_pipeline.png`, resized to 2200 px (paper Fig. 2)
- `fig_friction.svg`, `fig_grade.svg`, `fig_scaling.svg` ← `figures/svg/` (unchanged)

## Media

- `assets/videos/terrain-chase.mp4` ← `astraexplore/artifacts/demo.mp4` (screen recording of the Isaac viewport, 2026-09-11): first 45 s, cropped to remove the viewport tab, black bar, axis glyph and mouse cursor (`crop=1104:904:64:32`), scaled to 880 px, H.264 CRF 26, `+faststart`.
  It is a development render of one vehicle on a procedural course, **not** a benchmark episode, and the page caption says so.
- `cover.jpg` is the t = 30 s frame of that clip.
