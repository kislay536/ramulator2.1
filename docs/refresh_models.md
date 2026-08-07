# Refresh Models in Ramulator2

Ramulator2 models DRAM refresh through a pluggable `IRefreshManager` interface
owned by the memory controller. Each controller instantiates exactly one
refresh-manager implementation, chosen via the `refresh_manager` YAML/Python
config key. This document catalogues all refresh models currently implemented
in this repository.

## Interface

**File:** `src/ramulator/controller/refresh/i_refresh_manager.h`

```cpp
class IRefreshManager {
  RAMULATOR_REGISTER_INTERFACE(IRefreshManager, "refresh_manager")
 public:
  virtual void tick() = 0;   // called every controller clock cycle
};
```

Every implementation is registered under the config group `refresh_manager`
and is `tick()`-driven — it is invoked once per controller cycle, before
request scheduling, so refresh commands are effectively given priority over
regular read/write traffic in the same cycle. Every controller wires this up
the same way in its `tick()`:

```cpp
// GenericDDRController::tick() (src/ramulator/controller/impl/generic_ddr_controller.cpp)
tick_prologue();
m_refresh->tick();   // refresh requests get high priority in the same tick
m_rowpolicy->pre_schedule();
...
```

The same `m_refresh->tick()` call appears in `GenericDDRController`,
`HBMControllerBase`, `LPDDRControllerBase`, `BlockHammerController`, and
`PRACController` (the latter two are RowHammer-mitigation controllers that
otherwise reuse the generic scheduling loop).

A refresh manager issues its commands by constructing a `Request` with
`Request::Cmd` type and calling `m_ctrl->priority_send(req)`, which injects
the command into the controller's priority queue/buffer so it bypasses normal
read/write arbitration.

Refresh managers locate their target DRAM nodes and timing parameters through
the `DRAMSpec`/`DRAMNode` introspection API (`get_command_id`, `get_level_id`,
`get_level_size`, `get_timing_value`, `for_each_at_level`, `has_level`), so
the same implementation works across different DRAM standards without
hard-coding a device topology, as long as the required command/level/timing
names exist in that standard's spec.

There are **four** refresh-manager implementations in the codebase, all under
`src/ramulator/controller/refresh/impl/`, with generated Python bindings
mirrored under `python/ramulator/refresh_manager/`.

| impl name (config key) | Class | Source file | Command issued |
|---|---|---|---|
| `NoRefresh` | `NoRefresh` | `no_refresh.cpp` | none |
| `AllBank` | `AllBankRefresh` | `all_bank.cpp` | `REFab` |
| `PerBank` | `PerBankRefresh` | `per_bank.cpp` | `REFpb` |
| `HBM34PerBankRefresh` | `HBM34PerBankRefresh` | `hbm34_per_bank_refresh.cpp` | `REFpb` |

---

## 1. `NoRefresh`

**Source:** `src/ramulator/controller/refresh/impl/no_refresh.cpp`
**Python:** `python/ramulator/refresh_manager/no_refresh.py` — `ramulator.refresh_manager.NoRefresh`

The null/no-op refresh model. Both `init()` and `tick()` are empty bodies —
no refresh commands are ever issued and DRAM retention/refresh timing is not
modeled at all. Used when refresh behavior is irrelevant to the experiment
(e.g., pure latency/throughput microbenchmarks) or when refresh overhead
should be excluded from the results.

```cpp
class NoRefresh : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, NoRefresh, "NoRefresh")
  void init() override {}
  void tick() override {}
};
```

No parameters.

---

## 2. `AllBank` (`AllBankRefresh`)

**Source:** `src/ramulator/controller/refresh/impl/all_bank.cpp`
**Python:** `python/ramulator/refresh_manager/all_bank.py` — `ramulator.refresh_manager.AllBank`

Implements classic **all-bank refresh (REFab)**: refreshes every bank in a
scope node simultaneously, once every `nREFI` cycles. This is the standard
refresh mode used by most conventional DRAM standards (DDR3/4/5, LPDDR5/6)
and mirrors the JEDEC `REFab` command semantics.

### Refresh scope by DRAM standard

The "scope" is the address level at which one `REFab` command is issued (all
banks *below* that level in the addressing hierarchy are implicitly
refreshed together). This is a hard-coded per-standard lookup table:

| Standard | Refresh scope level |
|---|---|
| DDR3 | Rank |
| DDR4 | Rank |
| DDR5 | Rank |
| LPDDR5 | Rank |
| LPDDR6 | Rank |
| GDDR6 | Channel |
| GDDR7 | Channel |
| HBM1 | Channel |
| HBM2 | PseudoChannel |
| HBM3 | PseudoChannel |
| HBM4 | PseudoChannel |

If the DRAM standard isn't in this table, `init()` throws
`"AllBank refresh: no default scope for DRAM standard '<name>'"`.

At `init()`, the manager walks the DRAM device tree via
`for_each_at_level(m_ref_level, ...)` to collect every node at the scope
level (e.g., every Rank node) into `m_ref_nodes`, and reads the `nREFI`
timing value from the DRAM spec.

### Two scheduling modes

**1. "All-at-once" (default, `scatter_interval` = 0 or unset)** — identical
to the historical/simple `AllBankRefresh` behavior: every `nREFI` cycles, a
`REFab` command is sent to *every* scope node in the same cycle
(`tick_all_at_once()`).

**2. Scattered (`scatter_interval` > 0)** — staggers refreshes to different
scope nodes across time instead of bursting them all in one cycle
(`tick_scattered()`). Each scope node `i` (0-indexed) gets its first refresh
at cycle `(i+1) * scatter_interval`, then repeats every `nREFI` cycles
thereafter. This spreads out the refresh-induced unavailability instead of
stalling all ranks/channels at once. `init()` validates
`scatter_interval * num_ref_nodes <= nREFI`, throwing a `RuntimeError`
(message contains `"scatter_interval"`) if the stagger schedule can't fit
inside one refresh interval.

**3. Postponable (`postponable=True`, mutually exclusive with
`scatter_interval`)** — models the JEDEC refresh-postponement credit budget
(e.g. GDDR7 JESD239D §6.12.1: "a maximum of 8 REFab commands can be
postponed... the resulting maximum interval between the surrounding REFab
commands is limited to 9 × tREFI"). Instead of firing at a rigid
`clk == next_refresh_cycle` boundary, the manager maintains a credit
counter (`tick_postponable()`):

- One credit is already banked at `init()` (matching the other modes'
  first-refresh-due-at-`nREFI` starting point), and one more credit is
  earned every `nREFI` cycles, capped at `max_postponed + 1`.
- A banked credit is spent (issuing one round of `REFab` to every scope
  node, same as `tick_all_at_once()`) as soon as either:
  - **opportunistic**: the controller has no other pending traffic
    (`ControllerBase::has_pending_requests()` — checks the active, read,
    write, and priority buffers — returns false), so spending it now costs
    nothing, or
  - **forced**: credits have saturated at the cap (`credits > max_postponed`),
    so the JEDEC bound would otherwise be violated.

This lets the controller delay refresh to avoid interrupting bursty traffic
and catch up later (during idle periods, or forcibly once the postponement
budget runs out), rather than always paying the refresh stall at a fixed
cadence regardless of what else is happening. Note that `priority_send()`
only *enqueues* a request — actual issuance still waits on normal
`check_timing`/prerequisite resolution (e.g. `REFab` needs all scope banks
precharged first), so under heavy, unrelenting traffic contending for the
same banks, the observed issue clock can lag the credit-consumption clock
by a queueing delay; the credit accounting itself still guarantees the
bound (see `docs/refresh_models.md` test notes below).

### Parameters

| Param | Type | Default | Meaning |
|---|---|---|---|
| `scatter_interval` | int | `0` | Cycles between staggered refreshes to consecutive scope nodes. `0`/unset = classic all-at-once mode. Mutually exclusive with `postponable`. |
| `postponable` | bool | `False` | Enables the credit-based postponement mode described above. Mutually exclusive with `scatter_interval` (`init()` throws `RuntimeError` containing `"mutually exclusive"` if both are set). |
| `max_postponed` | int | `8` | Only used when `postponable=True`. Maximum number of refresh intervals a `REFab` round can be delayed past its nominal due time before being forced out (JEDEC GDDR7 default is 8). Must be non-negative. |
| `debug` | bool | `False` | Prints `[AllBank:init]` and `[AllBank]` trace lines (scope, nREFI, node count, per-refresh clk/addr_vec) to stdout; in postponable mode, also prints `[AllBank:postponable]` lines with the credit/forced/opportunistic decision each tick. |

### Example (Python config)

```python
ramulator.refresh_manager.AllBank()                          # classic, all ranks refreshed together every nREFI
ramulator.refresh_manager.AllBank(scatter_interval=2)         # staggered by 2 cycles per rank/channel
ramulator.refresh_manager.AllBank(postponable=True)           # JEDEC-style postponement, max_postponed=8
ramulator.refresh_manager.AllBank(postponable=True, max_postponed=3)
ramulator.refresh_manager.AllBank(debug=True)
```

### Observed behavior (from `tests/controller_scheduling/test_all_bank_refresh.py`)

- DDR4, 4 ranks, `nREFI=8`, no scatter: `REFab` issued to ranks `[0,1,2,3]`
  simultaneously at clk 8, then again at clk 16, etc. — all `Rank`-scope
  wildcards other than `Rank` are set to "all" (`BankGroup`/`Bank`/`Row`/`Column`
  are wildcarded in the address vector, i.e. it refreshes the whole rank at
  once).
- DDR4, 4 ranks, `nREFI=16`, `scatter_interval=2`: refreshes land at
  clk `2,4,6,8` (ranks 0-3) then `18,20,22,24` (next period).
- `scatter_interval` too large relative to `nREFI` raises a `RuntimeError`.
- HBM3/HBM4: scope is `PseudoChannel` — `REFab` wildcards `Sid`, `BankGroup`,
  `Bank`, `Row`, `Column`.
- HBM1: scope is `Channel`.
- Postponable: with a short burst of traffic that finishes before any
  credit boundary is crossed, `REFab` never fires while requests are in
  flight and fires shortly after the controller goes idle (opportunistic
  path). With a deep, never-ending backlog of traffic to a single bank,
  `REFab` never fires before `max_postponed * nREFI` cycles have elapsed
  (the earliest possible forced firing), and bounded staleness holds even
  though traffic never goes idle — by tick `T`, at least
  `T // nREFI - max_postponed` refresh rounds will have been forced out.
  `postponable=True` combined with `scatter_interval > 0` raises a
  `RuntimeError` containing `"mutually exclusive"`.

---

## 3. `PerBank` (`PerBankRefresh`)

**Source:** `src/ramulator/controller/refresh/impl/per_bank.cpp`
**Python:** `python/ramulator/refresh_manager/per_bank.py` — `ramulator.refresh_manager.PerBank`

Implements **per-bank refresh (REFpb)** as a simple round-robin sweep: at
`init()`, it collects *every* bank node in the entire device (via
`for_each_at_level(bank_level, ...)`, spanning all ranks/channels/bank
groups) into a flat list `m_bank_nodes`. Every `nREFIpb` cycles, it issues a
single `REFpb` command to the next bank in that list (`m_next_bank_idx`),
wrapping around at the end of the list. This spreads refresh overhead into
much smaller, more frequent operations than all-bank refresh, trading lower
per-event stall time for more frequent (but individually cheaper)
interruptions — one bank at a time, cycling through the whole device.

No sub-grouping (no PseudoChannel/Sid pairing logic) — it treats all banks
in the system as one flat round-robin ring. This is a generic/simple model,
distinct from the HBM3/4-specific model below which understands the
PseudoChannel-paired REFpb-set structure of that standard.

### Postponable mode (`postponable=True`)

Same credit/postponement model as `AllBank`'s postponable mode (see above),
applied to `REFpb` instead of `REFab` and keyed off `nREFIpb` instead of
`nREFI` — this also mirrors the JEDEC postponement text extended to REFpb
(e.g. GDDR7 JESD239D §6.12.2: "the maximum interval between refreshes to a
particular bank is limited to 9 × tREFI"). One credit is banked at `init()`
and one more is earned every `nREFIpb` cycles (capped at
`max_postponed + 1`); a banked credit is spent by issuing one `REFpb` to the
next bank in the existing round-robin order, either opportunistically
(`ControllerBase::has_pending_requests()` is false) or forced (credits
saturated at the cap).

**Simulator-fidelity note:** JEDEC's REFpb postponement rule is framed
per-bank ("maximum interval between refreshes to a *particular bank*"), but
`PerBankRefresh` has no per-bank staleness tracking to begin with — it's a
single flat round-robin cursor advancing every `nREFIpb` regardless of
postponable mode. Postponable mode applies the credit/postponement budget to
that same global cadence (one shared credit pool, one `REFpb` consumed per
credit, in round-robin order) rather than tracking a separate postponement
budget per bank. This is a deliberate simplification consistent with the
rest of this refresh-manager catalogue's fidelity level, not a claim of
cycle-exact per-bank JEDEC compliance.

### Parameters

| Param | Type | Default | Meaning |
|---|---|---|---|
| `postponable` | bool | `False` | Enables the credit-based postponement mode described above. |
| `max_postponed` | int | `8` | Only used when `postponable=True`. Maximum number of `nREFIpb` intervals a `REFpb` can be delayed past its nominal due time before being forced out. Must be non-negative. |

### Example (Python config)

```python
ramulator.refresh_manager.PerBank()                              # classic fixed-interval round robin
ramulator.refresh_manager.PerBank(postponable=True)               # JEDEC-style postponement, max_postponed=8
ramulator.refresh_manager.PerBank(postponable=True, max_postponed=2)
```

### Observed behavior (from `tests/controller_scheduling/test_gddr7.py`)

Same qualitative behavior as `AllBank`'s postponable mode: `REFpb` doesn't
fire while a short burst of traffic is in flight and fires shortly after
going idle; under a never-ending backlog to a single bank, `REFpb` never
fires before `max_postponed * nREFIpb` cycles have elapsed, and still
eventually fires within a bounded window despite traffic never going idle.

---

## 4. `HBM34PerBankRefresh`

**Source:** `src/ramulator/controller/refresh/impl/hbm34_per_bank_refresh.cpp`
**Python:** `python/ramulator/refresh_manager/hbm34_per_bank_refresh.py` — `ramulator.refresh_manager.HBM34PerBankRefresh`

A refresh model specific to the internal bank-refresh structure of **HBM3
and HBM4**, which refresh banks in coordinated "REFpb sets" across
PseudoChannels and stacks (Sid), rather than as one global flat ring like
`PerBank`. This model is considerably more elaborate than the other three.

### Concepts

- **PseudoChannel (`PC`)** and **Sid** (stack/die id, if present in the
  spec — some HBM4 configs are single-Sid) are independent refresh
  "lanes": every PC advances through its own bank sweep in lock-step with
  the same Sid index.
- For each `(PC, Sid)` pair there is a **REFpb set** — the full flat list of
  banks in that pseudo-channel/stack (`BankGroup * Bank`, flattened via
  `flat_bank_in_sid = addr_vec[BankGroup] * banks_per_group + addr_vec[Bank]`).
- **`nREFIpb`**: the refresh interval — how often a new bank-pair refresh is
  initiated.
- **`nRFCpb`**: the per-bank refresh-cycle time — once *every* bank in a
  REFpb set has been refreshed (a full sweep completes), that set must wait
  `nRFCpb` cycles before the sweep can restart (`next_set_allowed_clk`).

### Scheduling logic (`tick()`)

Each `tick()`:

1. First tries to drain any `m_pending_refpbs` (a queue of already-decided
   refreshes waiting on `priority_send` to succeed) — `service_pending_refpb()`.
2. If nothing pending and `m_clk >= m_next_refresh_clk`, it tries to seed a
   new batch: `seed_pending_refpbs()` enqueues **one REFpb per
   PseudoChannel** for the current `(m_next_sid, m_next_flat_bank)`
   position — i.e. all PCs are refreshed at the *same* flat-bank index in
   the *same* Sid together, one flat_bank step at a time. This is why PCs
   are "paired": PC0 and PC1 refresh their bank `i` back-to-back before the
   cursor (`advance_logical_cursor()`) moves to bank `i+1`.
3. If any PC's REFpb set for the current Sid is still cooling down
   (`m_clk < next_set_allowed_clk`, i.e., waiting out `nRFCpb` from
   finishing its last full sweep), seeding is deferred one cycle at a time
   until the set is available again.
4. Once a set finishes a full sweep (all `banks_per_sid` flagged
   `refreshed`), the `refreshed` flags reset and `next_set_allowed_clk` is
   set to `m_clk + nRFCpb`.
5. The logical cursor (`m_next_flat_bank`, `m_next_sid`) advances through
   all banks in a Sid before moving to the next Sid, cycling through all
   `(Sid, flat_bank)` combinations, with a new refresh seeded every
   `nREFIpb` cycles.

### Parameters

None — all cadence values (`nREFIpb`, `nRFCpb`) and topology
(`PseudoChannel`/`Sid`/`BankGroup`/`Bank` level sizes) are read directly from
the DRAM spec at `init()` time, not passed as config parameters.

### Observed behavior (from `tests/controller_scheduling/HBMController/test_hbm34_refresh.py`)

- PseudoChannels are paired before the flat-bank cursor advances: for a
  2-PC part, refreshes come out in order `PC0/bank0, PC1/bank0, PC0/bank1,
  PC1/bank1, ...`, with the two same-bank PC refreshes exactly 2 cycles
  apart.
- Every flat bank in a `(PC, Sid)` set is visited exactly once before any
  bank repeats (`seen[pc] == list(range(banks_per_sid))`).
- After a full sweep of a set completes, the same set cannot start its next
  sweep until at least `nRFCpb` cycles after its last refresh in the
  previous sweep (`pc0_repeat.clk - pc0_flat15.clk >= nRFCpb`).

---

## Selecting a refresh model in config

Refresh manager selection is a normal component/plugin config key
(`refresh_manager: ...`) on the controller, using the generated Python
`Component` classes shown above, or the equivalent YAML:

```yaml
memory_system:
  controller:
    refresh_manager:
      impl: AllBank
      scatter_interval: 2
      debug: false
```

```python
import ramulator
dram = ramulator.dram.DDR4(org_preset="DDR4_8Gb_x8", timing_preset="DDR4_2400R", rank=4, nREFI=8)
controller = ramulator.controller.GenericDDRController(
    ...,
    refresh_manager=ramulator.refresh_manager.AllBank(scatter_interval=2),
)
```

For HBM3/HBM4 device configs, `HBM34PerBankRefresh` is the standard-accurate
choice for per-bank refresh; the generic `PerBank` implementation remains
available for any device topology when a simple flat round-robin
per-bank sweep is sufficient (or for standards where the JEDEC per-bank
refresh grouping isn't modeled in detail). `NoRefresh` is used to disable
refresh modeling entirely.

## Related tests

- `tests/controller_scheduling/test_all_bank_refresh.py` — `AllBank` scope
  selection per standard, all-at-once vs. scattered timing, parameter
  validation, and postponable-mode deferral/forced-firing/mutual-exclusion
  behavior.
- `tests/controller_scheduling/test_gddr7.py` — `PerBank` postponable-mode
  deferral and forced-firing behavior (`test_gddr7_per_bank_refresh_postponable_*`).
- `tests/controller_scheduling/HBMController/test_hbm34_refresh.py` —
  `HBM34PerBankRefresh` PC-pairing, full-sweep coverage, `nRFCpb` cooldown
  enforcement.
