#include <algorithm>
#include <stdexcept>
#include <vector>

#include "ramulator/base/param.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/refresh/i_refresh_manager.h"
#include "ramulator/dram/node.h"

namespace Ramulator {

// Per-bank refresh — issues REFpb to one bank at a time in round-robin order.
//
// Default behavior:
//   postponable == false:
//      every nREFIpb cycles, issue one REFpb to the next bank in round-robin
//      order.
//
// Postponable behavior:
//   postponable == true:
//      models the JEDEC refresh-postponement credit budget also extended to
//      REFpb (e.g. GDDR7 JESD239D 6.12.2: the maximum interval between
//      refreshes to a particular bank is limited to (max_postponed + 1) *
//      nREFIpb). A credit is earned every nREFIpb cycles (capped at
//      max_postponed + 1) and consumed by issuing one REFpb to the next bank
//      in round-robin order. A banked credit is spent as soon as the
//      controller has no other pending traffic (opportunistic), or
//      unconditionally once credits saturate at the cap (forced).
//
//      Note: this applies the credit budget to the single global
//      round-robin cursor rather than tracking per-bank staleness
//      individually, which is a simplification relative to JEDEC's
//      per-bank framing — see docs/refresh_models.md.
class PerBankRefresh : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, PerBankRefresh, "PerBank")

 private:
  ControllerBase* m_ctrl;
  Clk_t m_next_refresh_cycle = -1;
  int m_cmd_refpb = -1;
  int m_bank_level = -1;
  int m_nrefipb = -1;  // Cached nREFIpb timing value (cycles)

  // optional: postponable/credit-based refresh
  bool m_postponable = false;
  int m_max_postponed = 8;
  int m_credits = 0;
  Clk_t m_next_credit_clk = -1;

  std::vector<DRAMNode*> m_bank_nodes;
  size_t m_next_bank_idx = 0;

  AddrVec_t build_addr_vec(DRAMNode* node);
  void init() override;
  void tick() override;

  void tick_fixed_interval();
  void tick_postponable();
  void send_refresh();
};

AddrVec_t PerBankRefresh::build_addr_vec(DRAMNode* node) {
  AddrVec_t addr_vec(m_ctrl->m_device.m_spec->level_count, -1);
  for (auto* n = node; n != nullptr; n = n->m_parent_node) {
    addr_vec[n->m_level] = n->m_node_id;
  }
  return addr_vec;
}

void PerBankRefresh::init() {
  m_ctrl = cast_parent<ControllerBase>();
  RAMULATOR_PARSE_PARAM(m_postponable, bool, "postponable").default_val(false);
  RAMULATOR_PARSE_PARAM(m_max_postponed, int, "max_postponed").default_val(8);

  const auto& info = *m_ctrl->m_device.m_spec;
  m_cmd_refpb = info.get_command_id("REFpb");
  m_bank_level = info.get_level_id("Bank");
  m_nrefipb = info.get_timing_value("nREFIpb");

  // Collect all bank-level nodes
  m_ctrl->m_device.m_root->for_each_at_level(m_bank_level, [&](DRAMNode* node) { m_bank_nodes.push_back(node); });

  if (m_postponable && m_max_postponed < 0) {
    throw std::runtime_error("PerBank refresh: max_postponed must be non-negative");
  }

  if (m_postponable) {
    // One refresh is already due at the first nREFIpb boundary, matching
    // the fixed-interval behaviour's starting point.
    m_credits = 1;
    m_next_credit_clk = m_nrefipb;
    return;
  }

  m_next_refresh_cycle = m_nrefipb;
}

void PerBankRefresh::send_refresh() {
  auto* bank_node = m_bank_nodes[m_next_bank_idx];
  AddrVec_t addr_vec = build_addr_vec(bank_node);
  Request req(addr_vec, Request::Cmd, m_cmd_refpb);

  bool is_success = m_ctrl->priority_send(req);
  if (!is_success) {
    throw std::runtime_error("Failed to send per-bank refresh!");
  }

  m_next_bank_idx = (m_next_bank_idx + 1) % m_bank_nodes.size();
}

void PerBankRefresh::tick() {
  if (m_postponable) {
    tick_postponable();
  } else {
    tick_fixed_interval();
  }
}

void PerBankRefresh::tick_fixed_interval() {
  if (m_ctrl->m_clk != m_next_refresh_cycle) {
    return;
  }
  m_next_refresh_cycle += m_nrefipb;
  send_refresh();
}

void PerBankRefresh::tick_postponable() {
  if (m_ctrl->m_clk == m_next_credit_clk) {
    m_credits = std::min(m_credits + 1, m_max_postponed + 1);
    m_next_credit_clk += m_nrefipb;
  }
  if (m_credits <= 0) {
    return;
  }

  bool forced = m_credits > m_max_postponed;
  bool opportunistic = !m_ctrl->has_pending_requests();
  if (!forced && !opportunistic) {
    return;
  }

  send_refresh();
  m_credits--;
}

}  // namespace Ramulator
