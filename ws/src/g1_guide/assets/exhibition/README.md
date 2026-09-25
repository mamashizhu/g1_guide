# Exhibition poster assets

`posters/E01.png` through `posters/E04.png` are deterministic, original
scientific schematic posters for the F01 exhibition scene. Each PNG is RGB,
768×512 pixels, and is generated from Pillow geometry and text only—there are
no downloaded or copyrighted image inputs.

Regenerate the posters from the repository root with:

```bash
python3 g1_ws/src/g1_guide/scripts/generate_exhibit_assets.py \
  --output-dir g1_ws/src/g1_guide/assets/exhibition/posters
```

The generator has no ROS dependency. `generate_assets(output_dir: Path)` is
the programmatic API used by `test/test_exhibit_assets.py`; invoking it again
with the same Pillow/font environment produces byte-for-byte identical files.

| File | Topic | Visual language |
| --- | --- | --- |
| E01.png | Solar system | Orbits, sunlight arrow, Earth and Moon |
| E02.png | Circuits and energy | Battery, closed wires, resistor and lamp |
| E03.png | Plant cell | Cell wall, nucleus, vacuole and chloroplasts |
| E04.png | Information and coding | Binary stream, code modules and signal wave |

E04 keeps its signal wave in the diagram band below the header, gives the
13×13 code-module grid a wide white container, and wraps the small
`pattern → meaning` caption inside the decode panel. The coordinate anchors
are public constants in the generator and are covered by the asset tests.
