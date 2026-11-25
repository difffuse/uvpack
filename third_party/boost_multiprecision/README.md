# Bundled Boost.Multiprecision and Boost.Config 1.90.0

The `include/boost` trees are unmodified headers from these pinned upstream tags:

- [Boost.Multiprecision boost-1.90.0](https://github.com/boostorg/multiprecision/tree/boost-1.90.0)
- [Boost.Config boost-1.90.0](https://github.com/boostorg/config/tree/boost-1.90.0)

Archive URLs, SHA-256 hashes and individual file hashes are recorded in
[provenance.json](provenance.json). The headers retain their upstream copyright
notices and use the [Boost Software License 1.0](LICENSE_1_0.txt).

`src/interior_overlap.cpp` privately enables `BOOST_MP_STANDALONE` and uses the
arbitrary-precision integer/rational backend to distinguish zero-gap boundary
contact from interior overlap. Rational coordinates are constructed exactly from
the input binary doubles; no fixed clipping grid or rounded midpoint is used.
The public UVPack headers do not expose Boost types. Builds download nothing and
runtime artifacts have no Boost shared-library dependency.
