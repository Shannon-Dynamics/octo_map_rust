# Safety model

What this project guarantees about memory safety, what it does not, and the
rules that keep the guarantee true as the code changes.

Everything below is a statement about **this repository**. Nothing here is a
claim about an application that links it.

## 1. Philosophy

This library exists in Rust rather than as a binding to the C++ original for one
reason: a binding puts an FFI boundary in the middle of the mapping stack, and
across that boundary the compiler cannot see ownership, lifetimes, or the length
of the buffer it was handed. Occupancy mapping consumes sensor data and files
produced by other software, so that boundary sits exactly where the untrusted
input is.

Removing the boundary is the point. Keeping it removed is a policy, not an
accident, so it is written down here.

## 2. What has been verified

Claims in this document are checked rather than asserted. What that check
consists of:

| Property | How it is verified | Result |
|---|---|---|
| No `unsafe` in the repository | `unsafe_code = "forbid"` at the workspace level — a compile error, not a review step | Enforced by the compiler |
| No undefined behaviour in the tested paths | `cargo +nightly miri test` with `-Zmiri-strict-provenance`, over both crates' unit suites and the `PointCloud2` robustness tests | Clean |
| Malformed files do not panic | `tests/parser_robustness.rs`: every truncation of a valid `.bt`/`.ot`, thousands of single-byte corruptions, and random blobs, through every reader | Clean |
| Malformed messages do not panic | `octomap-ros/tests/pointcloud2_robustness.rs`: randomized field descriptors, dimensions and blob lengths; any accepted cloud must iterate to the end | Clean |
| Length arithmetic cannot wrap | Checked arithmetic on every message-supplied offset and dimension, with a dedicated error variant | Enforced in code, tested |
| Dependency advisories | `cargo audit` in CI; `cargo deny` configuration in `deny.toml` | Clean |

Miri covers what the tests execute, which is not the same as covering every
path. It is evidence, not a proof.

One suite is deliberately outside the Miri run: `tests/parser_robustness.rs`
performs several thousand parses, and Miri's interpreter makes that take hours
rather than seconds. The same parsing code is exercised under Miri by
`octomap-core`'s in-module tests, and the robustness suite itself runs natively
on every `cargo test`.

## 3. Unsafe-code policy

```toml
[workspace.lints.rust]
unsafe_code = "forbid"
```

`forbid`, not `deny`: an `unsafe` block anywhere in the workspace is a compile
error, and — unlike `deny` — it cannot be re-enabled by an `#[allow]` further
down the tree. There is no `unsafe` in this repository today, and adding some
requires editing the workspace manifest, which is a visible change in review.

There is no FFI, no `extern` block, no raw pointer arithmetic, and no
`transmute`. The C++ reference is used to *generate test fixtures* during
development; it is never linked, and `cargo test` needs no C++ toolchain.

**If `unsafe` is ever genuinely required** — a plausible future case is a
`bytemuck`-style reinterpretation in the `.bt` reader, if profiling ever showed
it mattered — the bar is:

1. Only after a safe implementation has been written and measured, and the
   result recorded as an ADR in [`docs/decisions/`](docs/decisions/README.md).
2. Downgrade the lint to `deny` at the workspace level and `allow` it in exactly
   one module, never crate-wide.
3. Every block carries a `// SAFETY:` comment naming the invariant that makes it
   sound and the code responsible for upholding it.
4. The unsafe code is not reachable from the public API without going through a
   safe wrapper that establishes those invariants.
5. Run the affected tests under Miri in CI.

Until all five hold, the answer is no.

## 4. Dependency surface

| Crate | Runtime dependencies |
|---|---|
| `octomap-core` | none — `std` only |
| `octomap-ros` | `octomap-core` |
| `ros2/octomap_server_rs` | `r2r`, `tokio`, `futures` — and everything those pull in |

`octomap-core` and `octomap-ros` inherit no third-party `unsafe` because they
have no third-party dependencies. `criterion` appears as a dev-dependency for
benchmarks and is not propagated to consumers.

`ros2/octomap_server_rs` is the exception and is deliberately a separate cargo
workspace: `r2r` generates bindings against a C++ ROS 2 installation, so that
binary *does* have an FFI boundary. It is excluded from the root workspace, and
the `unsafe_code = "forbid"` claim above does not extend to the dependencies it
links. What that node contributes itself is field-moving between `r2r` structs
and the safe functions in `octomap-ros`; the mapping logic lives on this side of
the boundary and is tested without ROS.

## 5. Ownership model

- **An `OcTree` owns its nodes.** A node holds
  `Option<Box<[Option<Node<T>>; 8]>>` — one lazily allocated array of eight
  children, so a leaf costs no array at all. There is no shared ownership, no
  reference counting, and no interior mutability anywhere in the tree.
- **Queries borrow.** `search`, `get_log_odds`, `is_occupied_at` and the
  iterators take `&self` and hand back borrows tied to the tree's lifetime, so
  the borrow checker prevents mutating a map while walking it.
- **Mutation is explicit in the signature.** Anything that changes the map takes
  `&mut self`. This includes two that look like reads and are not:
  `write_binary` / `write_binary_file` threshold the tree to max likelihood
  before writing, matching the reference. `write_binary_const` is the
  non-mutating variant.
- **`Point3`, `OcTreeKey`, `OccupancyValue` are `Copy`.** They are small
  value types; passing one never transfers ownership of anything.
- **No global or thread-local state.** Two maps in one process cannot interfere,
  and nothing needs synchronizing to construct.

## 6. Safety boundaries — where input comes from outside

Three entry points read bytes this library did not produce. They are the places
a memory-safety claim has to be earned rather than asserted:

| Boundary | Entry point | What it does with malformed input |
|---|---|---|
| OctoMap files | `io::read_binary*`, `io::read_full*` | Returns `IoError`. Header, tree id, resolution and node structure are all validated before use |
| Message payloads | `io::read_binary_data`, `io::read_full_data` | Same, minus the header — the resolution arrives as an argument instead |
| `PointCloud2` | `octomap_ros::pointcloud2::Cloud::new` | Returns `CloudError`. Field offsets, datatypes, row stride and blob length are checked against each other before any point is read |

In all three, the decode is written against slices, so an offset that does not
fit is a bounds check away from a typed error rather than a read past the end.
A truncated file produces "unexpected end of input", not a panic and not a map
with garbage in it.

Two properties of these boundaries are worth stating precisely, because they
are the ones a hostile input would go after:

- **Nothing in a header drives an allocation.** The `size` field of a `.bt`/
  `.ot` header is metadata; a file claiming two billion nodes and carrying none
  fails on the data rather than reserving memory for the claim.
- **Memory use is bounded by the length of the input.** The readers take a
  `Read`, and they will consume what they are given: a header line is
  accumulated until a newline or end of input. For a file that bound is the
  file; for a stream the caller has to impose one. Wrap an untrusted stream in
  `Read::take` if it does not have a natural end.

Arithmetic on message-supplied lengths and offsets is checked rather than
wrapping, so a `PointCloud2` declaring an extreme geometry is rejected with
`CloudError::GeometryOverflow` instead of wrapping a length computation on a
32-bit target and turning a validated bound into an out-of-range index.

## 7. Invariants relied on internally

These are upheld by construction and asserted with `expect` carrying the
reasoning, not by `unsafe`:

- A tree's resolution is finite and strictly positive — enforced in
  `TreeGeometry::new`, which every constructor goes through.
- A child index is in `0..8` — produced only by `compute_child_index`, which
  masks to three bits.
- A depth is in `0..=tree_depth` — every public entry point taking a depth
  either validates it or returns `Result`.
- A key is inside the addressable volume — `coord_to_key_checked` returns
  `Option`; the unchecked variant is only used where the caller has already
  validated the coordinate.

Where an `expect` remains in the library, its message states the invariant
("child was just created", "depth 0 is always valid"). If one of those ever
fires it is a bug in this crate, not in the caller's input.

## 8. Panics

The library does not panic on any input it accepts through its public API.
Fallible operations return `Result` or `Option`:

- Invalid arguments → `OctomapError` (`InvalidResolution`, `InvalidDepth`,
  `CoordinateOutOfBounds`, `InvalidProbability`).
- I/O and malformed data → `IoError`, which wraps `std::io::Error` and adds the
  format-level cases.
- "No answer" → `Option`. An unobserved voxel is not an error, and
  `is_occupied_at` returning `None` is the API's most important distinction.

The single documented panic is `Index`/`IndexMut` on `Point3` with an index
above 2, which behaves like indexing a slice out of range. Examples, tests and
benchmarks use `unwrap` freely; that is appropriate there and nowhere else.

Arithmetic overflow: keys are `u16` and are combined through helpers that
saturate or mask rather than wrap silently. Coordinate conversion of a
non-finite `f32` is rejected before it can produce a nonsense key.

## 9. What is not claimed

- **Not "the whole application is memory safe".** A binary that links this
  library also links its own dependencies, and possibly a ROS 2 client library
  with a C++ core. The claim covers this repository's crates.
- **Not correctness of the mapping result.** A map can be memory safe and still
  be wrong for the robot using it — that is what the differential test suite in
  [`docs/03-verification.md`](docs/03-verification.md) is for, and it is a
  separate property.
- **Not freedom from resource exhaustion.** A pathological scan can allocate a
  great deal of memory, and a very large `max_range` at a fine resolution can
  make a single insertion slow. Neither is undefined behaviour, but both are
  denial-of-service shaped; bound your input.
- **Not thread safety beyond what the types say.** There are no `unsafe impl
  Send`/`Sync` anywhere here, so whatever auto-derived thread-safety the types
  have is the compiler's own conclusion from their fields, and it is sound by
  construction. There is no internal locking because there is no internal
  sharing: concurrent use is the caller's design problem, and the borrow checker
  will hold you to it.

## 10. Reporting a safety problem

Anything that could produce undefined behaviour, a panic from library input, or
a crash from a malformed file or message is a security-relevant bug. Report it
as described in [`SECURITY.md`](SECURITY.md).
