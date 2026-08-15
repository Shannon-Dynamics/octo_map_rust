# Security policy

## Supported versions

The project is pre-1.0 and has not been released yet. Until the first tagged
release, the supported version is the current `main` branch.

## Reporting a vulnerability

Report privately through GitHub's **Report a vulnerability** button under the
[Security tab](https://github.com/Shannon-Dynamics/octo_map_rust/security) of
this repository. Please do not open a public issue for something exploitable.

> **Note for the repository owner:** private vulnerability reporting has to be
> enabled in the repository settings for that button to exist. Until it is,
> this section describes a route that is not yet open. No alternative contact
> is published here, because publishing one is the owner's decision to make.

Include, as far as you can:

- what you fed the library (a file, a message payload, a scan), ideally as a
  reproducer;
- which entry point you called;
- what happened — a panic, a hang, memory growth, an incorrect map;
- the version or commit, and the platform.

Expect an acknowledgement within a few working days. If the report is confirmed
we will agree a disclosure timeline with you before publishing.

## What the project does to reduce risk

Stated so that a reader can judge the claim rather than take it:

- `unsafe` is forbidden by the compiler at the workspace level, and there is
  none in the repository.
- The unit suites and the message-decoding robustness tests run under Miri with
  strict provenance.
- The three parsing boundaries are property-tested against truncation,
  corruption and randomized geometry, asserting a typed error rather than a
  panic.
- Arithmetic on externally supplied lengths and offsets is checked.
- `cargo audit` runs in CI; `deny.toml` pins the licences and sources permitted
  in the dependency tree.
- Workflows request read-only permissions and pin third-party actions to commit
  SHAs.

This reduces risk. It does not make the project free of vulnerabilities, and
nothing here should be read as claiming that.

## What counts as a security issue here

This is a mapping library, not a network service, but it parses input that other
software produced. The following are in scope:

- **Any undefined behaviour.** There is no `unsafe` in this repository
  ([`SAFETY.md`](SAFETY.md)), so this should be impossible through the public
  API. If you find a way, it is the most valuable report you can send.
- **A panic reachable from library input.** A malformed `.bt` or `.ot` file, a
  truncated `octomap_msgs` payload, or a `PointCloud2` blob whose field offsets
  do not match its length must produce a typed error, never a crash.
- **Unbounded resource use from a small input** — a file that claims a node
  count it does not have, or a header that drives an allocation far larger than
  the data behind it.
- **Silent misparsing** — input accepted as a valid map that decodes to
  something other than what it says, which downstream software then trusts.

Out of scope:

- Denial of service from *legitimately* large input. A dense scan at a fine
  resolution is expensive by nature; bound your inputs.
- Vulnerabilities in ROS 2, `r2r`, or anything `ros2/octomap_server_rs` links.
  Report those upstream. Issues in this repository's own code paths inside that
  node are in scope.
- A map that is wrong for reasons unrelated to parsing. That is a correctness
  bug — open a normal issue, and please include the input.
