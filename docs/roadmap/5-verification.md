# Phase 5: Formal Verification

**Status:** Complete
**Depends on:** Phase 1 (algorithm crates to verify), Phase 2 (MRM chain for convergence proofs)
**Goal:** Prove correctness, panic-freedom, and safety properties of all ported algorithms
using bounded model checking (Kani) and deductive proofs (Verus).

## Description

The safety island's value depends on confidence that its algorithms behave correctly under all
inputs, including corrupted sensor data (NaN, Inf), extreme values, and adversarial timing.
Testing covers expected paths; formal verification covers *all* paths.

This phase adds Kani harnesses inline in each algorithm crate and Verus proofs in a dedicated
verification crate, following the same patterns established in nano-ros (157 Kani harnesses
across 14 files, 67+ Verus proof functions across 9 files).

## Tool Selection

| Tool      | Backend                       | Strengths                                                                     | Use For                                                               |
|-----------|-------------------------------|-------------------------------------------------------------------------------|-----------------------------------------------------------------------|
| **Kani**  | CBMC (bounded model checking) | Exhaustive over all IEEE 754 bit patterns; automatic — just assert properties | Per-call correctness, panic-freedom, NaN analysis, f32/f64 edge cases |
| **Verus** | Z3 (SMT solver)               | Unbounded deductive proofs; reasons over arbitrary-length sequences           | State machine invariants, convergence guarantees, ranking functions   |

**Critical constraint:** Verus cannot reason about floating-point arithmetic. Therefore:
- Float-heavy properties → **Kani** (handles all IEEE 754 bit patterns)
- State machine / convergence properties → **Verus** (uses integer ghost models)
- Properties spanning both → **both tools** composed via the ghost type bridge

## Deliverables

### NaN Safety Fix

**StopFilter** `is_stopped` was changed from `x.abs() < threshold` (returns `false` for NaN)
to `!(x.abs() >= threshold)` (returns `true` for NaN → zeroed output = safe). This ensures
corrupted sensor data produces a stopped (safe) output rather than passing NaN through the
control chain.

### Kani Harnesses (13 total)

| Crate | Harnesses | Key Properties |
|-------|-----------|----------------|
| `autoware_stop_filter` | 5 | NaN→zero, output never NaN, panic-freedom, stopped⇒zeros, moving⇒passthrough |
| `autoware_vehicle_velocity_converter` | 4 | Covariance PSD diagonal, f32→f64 sign preservation, panic-freedom, output NaN check |
| `autoware_shift_decider` | 3 | Valid gear output, dead-zone holds previous, NaN velocity holds previous |
| `autoware_mrm_emergency_stop_operator` | 1 | Bounded convergence from 20 m/s to 0 in ≤300 steps |

### Verus Proofs (3 modules, 8 proof functions)

| Module | Proofs | Key Properties |
|--------|--------|----------------|
| `shift_decider` | 2 | Valid gear invariant, no direct DRIVE→REVERSE transition |
| `emergency_stop` | 3 | Ramp phase terminates, decel phase terminates, bounded convergence |
| `mrm_handler` | 3 | Terminal state stability, escalation monotonicity, succeeded requires stopped |

## Verification Crate Structure

```
src/
  autoware_stop_filter/src/lib.rs              # + #[cfg(kani)] mod verification (5 harnesses)
  autoware_vehicle_velocity_converter/src/...   # + #[cfg(kani)] mod verification (4 harnesses)
  autoware_shift_decider/src/lib.rs            # + #[cfg(kani)] mod verification (3 harnesses)
  autoware_mrm_emergency_stop_operator/src/... # + #[cfg(kani)] mod verification (1 harness)
  verification/                                 # Verus crate (standalone, not in workspace)
    Cargo.toml                                  # edition = "2024", [package.metadata.verus]
    src/
      lib.rs                                    # verus! { } macro blocks, module declarations
      ghost_types.rs                            # VelocityZone enum, integer-scaled classification
      shift_decider.rs                          # GhostShiftDecider + valid_gear proofs
      emergency_stop.rs                         # DecelState ghost + convergence ranking functions
      mrm_handler.rs                            # GhostMrmHandler + state machine invariant proofs
```

## Running Verification

```bash
# Kani — all harness crates
just verify-kani

# Kani — specific crate
cd src/autoware_stop_filter && cargo kani

# Kani — specific harness
cd src/autoware_stop_filter && cargo kani --harness output_never_nan

# Verus
just verify-verus

# All verification
just verify

# Existing CI (still passes)
just ci
```

## Property Table

Properties ordered by automotive safety relevance:

| Rank | Property | Algorithm | Tool | Safety Relevance |
|------|----------|-----------|------|------------------|
| 1 | NaN input produces safe output (zero) | StopFilter | Kani | Sensor corruption → vehicle must stop |
| 2 | Emergency stop converges to v=0 | EmergencyStopOp | Kani+Verus | Core MRM guarantee |
| 3 | Output never NaN (all components) | StopFilter, VVC | Kani | NaN propagates through control chain |
| 4 | Gear output is valid constant | ShiftDecider | Kani+Verus | Invalid gear → transmission damage |
| 5 | MRM escalation monotonicity | MRM Handler | Verus | De-escalation during emergency = hazard |
| 6 | MRM Succeeded requires v=0 | MRM Handler | Verus | Premature "all clear" while moving |
| 7 | Anti-chattering dead zone | ShiftDecider | Kani+Verus | Gear hunting → drivetrain wear/jerking |
| 8 | Covariance is valid PSD diagonal | VVC | Kani | Invalid covariance → EKF divergence |
| 9 | f32→f64 sign preservation | VVC | Kani | Sign flip → wrong direction command |
| 10 | Panic freedom (all components) | StopFilter, VVC | Kani | Panic on MCU → system halt |
| 11 | Terminal state stability | MRM Handler | Verus | State machine corruption |

## Ghost Type Bridge Architecture

The ghost type system bridges production code (f32/f64) with Verus proofs (integers only)
through a three-layer design:

1. **Production code** — float-based algorithms in `#![no_std]` crates
2. **Kani harnesses** — exhaustive verification over all IEEE 754 bit patterns inline
3. **Verus proofs** — integer-scaled ghost models in `src/verification/`

### Soundness argument

1. **Kani** verifies concrete properties (NaN handling, panic-freedom, bit-exact passthrough)
   for all possible IEEE 754 inputs
2. **Verus** proves abstract properties (convergence, invariant preservation, monotonicity)
   over integer ghost models
3. **Composition:** The integer scaling (×1000) preserves the essential dynamics while
   enabling SMT reasoning. Kani's companion harness on the actual f32 code validates the
   ghost model's predictions.

## Future Phase Verification

### Phase 3 — Vehicle Command Gate

- **Kani:** Rate limiter output bounds, jerk limiting bounded rate of change, source
  arbitration priority
- **Verus:** Monotonic source priority (unbounded), rate limit envelope invariant

### Phase 4 — Control Validator

- **Kani:** NaN-safe threshold comparisons, diagnostic severity correctness
- **Verus:** Overspeed detection completeness
