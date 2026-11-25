# Bundled Clipper 6.4.2

`clipper.cpp`, `clipper.hpp` and `License.txt` are unmodified files from Angus
Johnson's [Clipper 6.4.2 release](https://downloads.sourceforge.net/project/polyclipping/clipper_ver6.4.2.zip),
extracted byte for byte from the archive's `cpp/` directory. The library is
distributed under the [Boost Software License 1.0](License.txt).

SHA-256 of the bundled files:

| File | SHA-256 |
|---|---|
| `clipper.cpp` | `5c642a3668311701f72572443aa42c1a981edb037298efc015166d9d90be0755` |
| `clipper.hpp` | `734eba9dc9d399089b2b467017074bd24728a1b9e64c7429e827806ed10e54cc` |
| `License.txt` | `e3cc0380cc5eb9aedf5a7616fe9fde3710891bced21fd6a20a19194ccd4c1c88` |

SHA-256 of the upstream archive:
`a14320d82194807c4480ce59c98aa71cd4175a5156645c4e2b3edd330b930627`.
The same record is available in machine-readable form next to this file.

The build uses only the bundled sources and downloads nothing. Clipper is used
through `src/offset.cpp`, which calls `ClipperOffset` to grow each island
polygon by the packing margin.
