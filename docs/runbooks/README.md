# Runbooks

Operational procedures. Every runbook ends with a **Verification** section
naming the number or the file that must appear — not "it should work".

| Runbook | When to use it |
|---|---|
| [`regenerate-fixtures.md`](regenerate-fixtures.md) | The C++ reference moved version, or the fixtures need rebuilding |
| [`benchmark.md`](benchmark.md) | Re-measuring the internal timing baselines |
| [`linux-verify.md`](linux-verify.md) | Verifying the suite on Linux — the safety net for the bit-exact comparison |
| [`ros2-node.md`](ros2-node.md) | Building and running the ROS 2 node, plus the smoke test |
| [`troubleshooting.md`](troubleshooting.md) | **Read first** when something fails in a strange way |

None of the first four is needed to **use** this crate. For that,
`cargo test --workspace` and `cargo build` are enough — see
[`../04-running.md`](../04-running.md).

The format for a new runbook is in [`_template.md`](_template.md).
