# 3D view vehicle models

`models_kenney.json` holds the vehicles of the drive tab's 3D view: Kenney Car Kit 3.1 models
(CC0 1.0, https://kenney.nl, "Created/distributed by Kenney"), converted to real proportions with the drive_viz
converter (`~/Documents/can-reverse-history-20261105/drive_viz/kenney/convert.py`, 2026-11-05) and rounded to millimetres:

| key | Kenney model | L x W x H (m) |
|---|---|---|
| sedan | sedan | 4.85 x 1.84 x 1.45 |
| suv | suv (also the own car) | 4.75 x 1.90 x 1.70 |
| suvl | suv-luxury | 4.90 x 1.92 x 1.74 |
| hatch | hatchback-sports | 4.30 x 1.80 x 1.45 |
| van | van | 5.15 x 1.95 x 1.95 |
| pickup | truck | 5.60 x 2.00 x 1.85 |
| truck | delivery | 8.20 x 2.45 x 3.30 |

The drive tab downloads it only when the 3D view starts.
