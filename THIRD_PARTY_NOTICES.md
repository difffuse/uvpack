# UVPack bundled dependencies

UVPack's own implementation is MIT licensed; see [LICENSE](LICENSE).

These notices cover bundled third-party dependencies and their redistribution
terms.

- **Clipper 6.4.2**, copyright Angus Johnson 2010–2017, is bundled in
  `third_party/clipper`. Its original headers and source retain its copyright,
  license statement and algorithm attributions. The static SDK includes its
  compiled archive as `uvpack_clipper`, a private link dependency of the core.
- **Boost.Multiprecision and Boost.Config 1.90.0** are bundled as unmodified
  upstream headers in `third_party/boost_multiprecision`. Individual headers
  retain their original copyright notices. The private exact interior predicate
  uses their integer/rational backend; the SDK exposes no Boost types and
  requires no Boost installation or shared runtime library.

Both dependencies use the **Boost Software License 1.0**, included in the
installed package as `BOOST_LICENSE_1_0.txt`. Boost source archive URLs and
SHA-256 hashes are in `boost-provenance.json`; its per-file paths refer to the
source tree, not to installed headers.

Additional compiler runtime files selected through `UVPACK_RUNTIME_FILES` need
their own provenance and redistribution notices in the final platform bundle.
Supply their notices through `UVPACK_RUNTIME_LICENSE_FILES`; these are installed
in the `runtime` subdirectory beside this file.
They are not covered by the Boost license. Optional Vulkan system drivers and
loaders are not included by this install target.
