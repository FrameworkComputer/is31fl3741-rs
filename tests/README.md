# Regression tests

Run from the repository root:

```sh
cargo test --all-features
cargo test --no-default-features
cargo test --all-features --release
```

CI runs these commands. Host tests require no hardware. Release testing also
checks behavior where integer overflow checks differ from debug builds.

- `support/mod.rs`: shared recording I²C mock, configurable reads and transaction failures.
- `core.rs`: PWM page coverage, input bounds, configuration preservation, address targeting, and failure handling.
- `adafruit.rs`: feature-gated RGB bounds and graphics clipping.

Add a regression case with each bug fix or substantive new API. Keep tests of
public behavior here; unit tests for private helpers can live beside those helpers.
Extend the mock with scripted reads when a workflow needs multiple distinct read
values. Keep assertions about register images and required ordering rather than
incidental transaction counts unless transaction count is itself the requirement.

The short-frame test currently records the documented panic-before-I²C contract.
If that API changes to return a validation error, update the test to assert that
error while retaining the no-bus-access check.

Hardware regression firmware lives in
[`hardware_check.rs`](../examples/ledmatrix/examples/hardware_check.rs).
See the [Framework instructions](../examples/ledmatrix/README.md) for running it.
Hardware results are recorded separately from host test results; a passing host
suite does not establish optical output or electrical behavior.
