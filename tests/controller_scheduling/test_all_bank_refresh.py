import pytest

import ramulator
import tests.controller_scheduling.harness as cs


pytestmark = pytest.mark.controller_scheduling


def _collect_issued(dut, *, command, count, max_ticks):
    found = []
    for _ in range(max_ticks):
        for item in dut.tick():
            if item.command == command:
                found.append(item)
                if len(found) == count:
                    return found
    raise AssertionError(f"Did not observe {count} {command} commands in {max_ticks} ticks")


def _level_index(dut, name):
    return dut.level_names.index(name)


def _assert_wildcard_levels(dut, item, names):
    for name in names:
        assert item.addr_vec[_level_index(dut, name)] == dut.ALL


def test_all_bank_refresh_without_scatter_keeps_all_at_once_schedule():
    dram = ramulator.dram.DDR4(
        org_preset="DDR4_8Gb_x8",
        timing_preset="DDR4_2400R",
        rank=4,
        nREFI=8,
    )
    dut = cs.ControllerUnderTest.make_generic_ddr(
        dram,
        refresh_manager=ramulator.refresh_manager.AllBank(),
    )

    refs = _collect_issued(dut, command="REFab", count=8, max_ticks=32)
    rank_idx = _level_index(dut, "Rank")

    assert [item.addr_vec[rank_idx] for item in refs] == [0, 1, 2, 3, 0, 1, 2, 3]
    assert refs[0].clk == 8
    assert refs[4].clk == 16


def test_all_bank_refresh_scatter_interval_staggers_scope_nodes():
    dram = ramulator.dram.DDR4(
        org_preset="DDR4_8Gb_x8",
        timing_preset="DDR4_2400R",
        rank=4,
        nREFI=16,
    )
    dut = cs.ControllerUnderTest.make_generic_ddr(
        dram,
        refresh_manager=ramulator.refresh_manager.AllBank(scatter_interval=2),
    )

    refs = _collect_issued(dut, command="REFab", count=8, max_ticks=40)
    rank_idx = _level_index(dut, "Rank")

    assert [(item.clk, item.addr_vec[rank_idx]) for item in refs] == [
        (2, 0),
        (4, 1),
        (6, 2),
        (8, 3),
        (18, 0),
        (20, 1),
        (22, 2),
        (24, 3),
    ]


def test_all_bank_refresh_rejects_scatter_interval_that_exceeds_nrefi():
    dram = ramulator.dram.DDR4(
        org_preset="DDR4_8Gb_x8",
        timing_preset="DDR4_2400R",
        rank=4,
        nREFI=7,
    )

    with pytest.raises(RuntimeError, match="scatter_interval"):
        cs.ControllerUnderTest.make_generic_ddr(
            dram,
            refresh_manager=ramulator.refresh_manager.AllBank(scatter_interval=2),
        )


def test_all_bank_refresh_accepts_debug_flag():
    dram = ramulator.dram.DDR4(
        org_preset="DDR4_8Gb_x8",
        timing_preset="DDR4_2400R",
        nREFI=4,
    )
    dut = cs.ControllerUnderTest.make_generic_ddr(
        dram,
        refresh_manager=ramulator.refresh_manager.AllBank(debug=True),
    )

    refs = _collect_issued(dut, command="REFab", count=1, max_ticks=16)
    assert refs[0].command == "REFab"


@pytest.mark.parametrize(
    "dram",
    [
        ramulator.dram.HBM3(org_preset="HBM3_8Gb_8hi", timing_preset="HBM3_6400Mbps", nREFI=2),
        ramulator.dram.HBM4(org_preset="HBM4_32Gb_8Hi", timing_preset="HBM4_8000Mbps", nREFI=2),
    ],
)
def test_all_bank_refresh_uses_pseudochannel_scope_for_hbm34(dram):
    dut = cs.ControllerUnderTest.make_hbm34(
        dram,
        refresh_manager=ramulator.refresh_manager.AllBank(),
    )

    refs = _collect_issued(dut, command="REFab", count=2, max_ticks=16)
    pc_idx = _level_index(dut, "PseudoChannel")

    assert [item.addr_vec[pc_idx] for item in refs] == [0, 1]
    for item in refs:
        _assert_wildcard_levels(dut, item, ["Sid", "BankGroup", "Bank", "Row", "Column"])


def test_all_bank_refresh_uses_rank_scope_for_ddr4():
    dram = ramulator.dram.DDR4(org_preset="DDR4_8Gb_x8", timing_preset="DDR4_2400R", nREFI=4)
    dut = cs.ControllerUnderTest.make_generic_ddr(
        dram,
        refresh_manager=ramulator.refresh_manager.AllBank(),
    )

    ref = _collect_issued(dut, command="REFab", count=1, max_ticks=16)[0]

    assert ref.addr_vec[_level_index(dut, "Rank")] == 0
    _assert_wildcard_levels(dut, ref, ["BankGroup", "Bank", "Row", "Column"])


def test_all_bank_refresh_uses_channel_scope_for_hbm1():
    dram = ramulator.dram.HBM1(org_preset="HBM1_2Gb", timing_preset="HBM1_2Gbps", nREFI=4)
    dut = cs.ControllerUnderTest.make_hbm12(
        dram,
        refresh_manager=ramulator.refresh_manager.AllBank(),
    )

    ref = _collect_issued(dut, command="REFab", count=1, max_ticks=16)[0]

    assert ref.addr_vec[_level_index(dut, "Channel")] == 0
    _assert_wildcard_levels(dut, ref, ["BankGroup", "Bank", "Row", "Column"])


def _bank0_addr(dut, row):
    return dut.addr_vec(Rank=0, BankGroup=0, Bank=0, Row=row, Column=0)


def test_all_bank_refresh_postponable_defers_during_traffic_and_fires_when_idle():
    # nREFI is large enough that the credit boundary is never crossed while
    # the two reads below are in flight, so any REFab observed can only be
    # the initial banked credit firing opportunistically once idle.
    dram = ramulator.dram.DDR4(
        org_preset="DDR4_8Gb_x8",
        timing_preset="DDR4_2400R",
        rank=1,
        nREFI=200,
    )
    dut = cs.ControllerUnderTest.make_generic_ddr(
        dram,
        refresh_manager=ramulator.refresh_manager.AllBank(postponable=True, max_postponed=3),
    )

    # Row-conflicting reads to the same bank keep the controller busy
    # (has_pending_requests() true) until the second one retires.
    dut.send_request("Read", _bank0_addr(dut, 0))
    dut.send_request("Read", _bank0_addr(dut, 1))

    history = []
    for _ in range(150):
        history += dut.tick()

    last_rd_clk = max(item.clk for item in history if item.command == "RD")
    refs = [item for item in history if item.command == "REFab"]

    assert len(refs) == 1
    assert refs[0].clk > last_rd_clk


def test_all_bank_refresh_postponable_forces_fire_at_max_postponed_cap():
    # A deep backlog of row-conflicting reads to the same bank keeps the
    # controller permanently busy (has_pending_requests() stays true), and
    # once the first postponed REFab is enqueued it also blocks further read
    # scheduling (the priority buffer must be empty for pick_rw_if to run),
    # so the exact clock a queued REFab is *issued* is subject to real
    # queueing/timing contention, not just the credit math. What must still
    # hold is the JEDEC bound this feature exists to guarantee: refresh
    # cannot be forced out before max_postponed extra intervals have
    # elapsed, and it cannot fall behind by more than that bound either.
    nrefi = 20
    max_postponed = 3
    total_ticks = 140
    dram = ramulator.dram.DDR4(
        org_preset="DDR4_8Gb_x8",
        timing_preset="DDR4_2400R",
        rank=1,
        nREFI=nrefi,
    )
    dut = cs.ControllerUnderTest.make_generic_ddr(
        dram,
        refresh_manager=ramulator.refresh_manager.AllBank(postponable=True, max_postponed=max_postponed),
    )

    for row in range(20):
        dut.send_request("Read", _bank0_addr(dut, row))

    history = []
    for _ in range(total_ticks):
        history += dut.tick()

    refs = [item for item in history if item.command == "REFab"]

    # The manager starts with 1 banked credit and earns 1 more per nREFI, so
    # it cannot be forced to fire before max_postponed additional intervals
    # have elapsed.
    earliest_possible_force = max_postponed * nrefi
    assert all(item.clk >= earliest_possible_force for item in refs)

    # Bounded staleness: by `total_ticks`, refresh cannot have fallen behind
    # by more than max_postponed periods' worth.
    min_expected = total_ticks // nrefi - max_postponed
    assert len(refs) >= min_expected


def test_all_bank_refresh_rejects_postponable_combined_with_scatter_interval():
    dram = ramulator.dram.DDR4(org_preset="DDR4_8Gb_x8", timing_preset="DDR4_2400R", rank=1, nREFI=20)

    with pytest.raises(RuntimeError, match="mutually exclusive"):
        cs.ControllerUnderTest.make_generic_ddr(
            dram,
            refresh_manager=ramulator.refresh_manager.AllBank(postponable=True, scatter_interval=2),
        )
