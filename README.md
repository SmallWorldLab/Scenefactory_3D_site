# SceneFactory-3D project page

Academic project page for:

> **SceneFactory-3D: Lifting 2D Traffic Scenes into 3D Physical Counterfactuals for Scalable Physically Grounded Safety Evaluation**

Adapted from the Small World Lab [SIREN-Bench project page](https://github.com/SmallWorldLab/siren-bench-site) (itself derived from muSync-GS and the [Nerfies template](https://github.com/nerfies/nerfies.github.io)). Bulma base, lab branding and page JavaScript are carried over; the accent palette (teal, matched to the paper figures), content, tables and media are specific to SceneFactory-3D.

## Local preview

```bash
cd scenefactory3d-site
python3 -m http.server 8000
```

## Publication status

- Paper: [arXiv:2610.02874](https://arxiv.org/abs/2610.02874) (v2, 13 Oct 2026). The page shows the arXiv author list, the arXiv BibTeX and Google Scholar `citation_*` meta tags.
- Code: coming soon (will be SmallWorldLab/SceneFactory_3D, `main`, MIT). The page shows no repository links until release.
- Search indexing is enabled (the `noindex` tag was removed).

See [CONTENT_CHECKLIST.md](CONTENT_CHECKLIST.md) before publication.

## Sections

| Anchor | Content |
|---|---|
| `#abstract` | Abstract + overview figure (paper Fig. 1) |
| `#simulator` | Four mechanism cards, tire-force equation, spec strip |
| `#vehicles` | Spec cards (2 released vehicles + 4 extended-family vehicles) and six demo videos |
| `#benchmark` | Pipeline figure (paper Fig. 2), task, conditions, controller table |
| `#results` | Headline table, friction / grade / scaling figures, findings |
| `#scope` | Release plan and limitations |
| `#BibTeX` | arXiv citation |

## Vehicle demos

`tools/vehicle_demos/` regenerates the eight clips in `#vehicles` (two in `#cargo`):

```bash
# simulate on the GPU (FamilyBatch; env with newton, warp, gymnasium, e.g. astraexplore/.venv-lab)
CUDA_VISIBLE_DEVICES=1 python tools/vehicle_demos/simulate.py --code ~/Github/astraexplore-vehicles   # -> tools/vehicle_demos/logs/*.json
# render (env with playwright, e.g. astraexplore/.venv)
python tools/vehicle_demos/render.py   # -> assets/videos/vehicles, assets/images/vehicles
```

`--code` points at a checkout of the astraexplore branch `vehicle-family`, whose
`astra_drive/vehicle_family` extends the released v1 vehicle to N axles, dual tires, a fifth-wheel
trailer, skid steering and loose cargo crates (same formulas as the released GPU kernel for the Coupe and Crossover; see its
`VEHICLE_FAMILY.md`). `--device cpu` runs the float64 CPU reference instead. `render.py` draws each logged frame with `render.html` (three.js 0.160 from
jsDelivr) in headless Chrome and encodes H.264. Captions quote numbers from the logs; re-check them
after re-simulating.

Vehicle names on the page are display names: SF3D Coupe = release `VEHICLES[0]`, SF3D Crossover =
`VEHICLES[1]`. The release code itself is frozen and keeps its own identifiers.
