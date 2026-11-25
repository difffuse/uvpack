# UVPack

UVPack is Difffuse Studio’s C++17 library and command-line tool for packing UV
islands into texture tiles. It accepts polygons with holes and rigid parts,
supports fixed scale, maximum scale and maximum margin, and reports placement
transforms for the original islands. A Python companion extracts islands from
mesh topology and applies layouts to the original UV corners.

## Build and install

Use CMake 3.19+ and a C++17 compiler. The CPU build uses bundled Clipper and
Boost headers and can build offline.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUVPACK_VULKAN=OFF
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
cmake --install build --config Release --prefix /path/to/uvpack
```

With Vulkan available, omit `-DUVPACK_VULKAN=OFF` to build the optional compute
worker. Shader sources and SPIR-V are included. The build can regenerate the
shaders when `glslangValidator` or `glslc` is available. The worker opens the
Vulkan loader at runtime. Run `uvpack_cli devices` to inspect available devices
and set `options.gpu` to enable GPU trials for a request.

## Pack an island

Save the following as `request.json`.

```json
{
  "target": [0, 0, 1, 1],
  "islands": [{"id": 0, "outer": [[0, 0], [0.4, 0], [0.4, 0.2], [0, 0.2]]}],
  "options": {"scale_mode": "fixed", "scale": 1, "margin": 0.01, "require_complete": true}
}
```

Run `uvpack_cli request.json` from the installed `bin` directory, or put the
directory on your path. The CLI also reads standard input with `uvpack_cli -`.
The response includes `valid`, `complete`, `validation` and an `islands` array
keyed by the supplied IDs. Apply each row-major transform to the input UVs.
The protocol reference covers streaming, snapping, normalization and validation
at storage precision. [CLI and geometry reference](docs/protocol.md)

## C++ and Python

Use `find_package(uvpack CONFIG REQUIRED)` and link `uvpack::core` in a CMake
project. The installation includes the CLI, position-independent static
archives, public headers, a CMake package, the Python companion and licenses.
The installed example shows a consumer embedding the packer in a shared library.
[Example project](examples/installed)

```sh
cmake -S examples/installed -B consumer-build -DCMAKE_PREFIX_PATH=/path/to/uvpack
cmake --build consumer-build --config Release
```

For toolchains that require app-local runtime libraries, pass their absolute
paths in the CMake list `UVPACK_RUNTIME_FILES` and their license notices in
`UVPACK_RUNTIME_LICENSE_FILES`. The install step puts those libraries beside the
CLI and their notices under `share/uvpack/licenses/runtime`.

Install the Python companion with `python -m pip install .` using Python 3.11+
or add the SDK’s `share/uvpack/python` directory to `PYTHONPATH`.
`uvpack_geometry.build_info()` and `uvpack_cli --version` report the matching
version, capabilities and protocol APIs. The source checkout also supplies a
standard-library OBJ command.

```sh
python3 tools/uv_islands.py extract mesh.obj > islands.json
uvpack_cli islands.json > layout.json
python3 tools/uv_islands.py apply mesh.obj layout.json packed.obj
```

## Blender

Copy the `tools/uvpack_addon` folder, `tools/uv_islands.py`,
`tools/uv_similarity.py` and the `python/uvpack_geometry` package together into
Blender’s user scripts `addons` directory. Enable **UVPack Islands** and set the
path to `uvpack_cli` in its preferences. Open the UV Editor sidebar and select
**UVPack** to pack the selected faces, work with tiles and sets, preserve pinned
islands or use the stacking and alignment tools.

## License

UVPack uses the MIT License. Bundled dependencies retain their own licenses.
[MIT License](LICENSE) · [Third-party notices](THIRD_PARTY_NOTICES.md)
