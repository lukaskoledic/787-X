--[[
    787-9 Electrical System Model — v2 (expanded)
    ------------------------------------------------
    For X-Plane 12 / FlyWithLua NG or CG.

    Models the 787's more-electric architecture at a systems-behavior level:
      - 2x Engine-driven Variable Frequency Generators (VFG), 235 kVA each
      - 2x APU generators (usable in flight up to an altitude ceiling — real
        787 trait, most jets can't do this)
      - 1x Backup Generator (permanent magnet gen, powers backup buses only)
      - RAT (Ram Air Turbine) — last-resort emergency power
      - Main battery + APU battery
      - 4 main AC buses (L1, L2, R1, R2) + 2 backup AC buses, cross-tied
      - Per-subsystem load model (avionics, hydraulics, galley, anti-ice)
      - Basic generator fault injection for testing failure procedures

    Custom datarefs are the single source of truth for electrical state.
    Other modules (hydraulics, fuel/APU, autopilot) should READ
    `b789/elec/state/*` and WRITE their own inputs into `b789/elec/input/*`
    where relevant (e.g. the APU module sets input.apu_running).

    This runs fine on ANY loaded aircraft right now — it only reads/writes
    datarefs, so you don't need a real 787 model or .acf to test it. See
    the "TESTING" section at the bottom.
]]--

-- ============================================================
-- CONFIG
-- ============================================================
local GEN_CAPACITY_KVA         = 235   -- per engine VFG
local APU_GEN_CAPACITY_KVA     = 225   -- per APU generator
local BACKUP_GEN_CAPACITY_KVA  = 20    -- permanent magnet backup gen
local BATTERY_DRAIN_PCT_PER_MIN = 0.8  -- battery-only endurance model
local BATTERY_CHARGE_PCT_PER_MIN = 0.5
local UPDATE_HZ                = 2     -- how often the logic recalculates per second

-- ============================================================
-- INPUT DATAREFS — stock X-Plane engine state
-- ============================================================
dataref("engn_running", "sim/flightmodel/engine/ENGN_running", "array")

-- ============================================================
-- CUSTOM INPUT DATAREFS — switches / other-module state
-- Drive these from a panel, DataRefEditor, or other Lua modules.
-- ============================================================
create_dataref_table("b789/elec/input", "Data")
local inp = b789.elec.input

inp.apu_running         = 0   -- set by APU module once it exists
inp.apu_gen_switch      = 1
inp.eng1_gen_switch     = 1
inp.eng2_gen_switch     = 1
inp.backup_gen_switch   = 1
inp.bat_switch          = 1
inp.apu_bat_switch      = 1
inp.ext_power_available = 0   -- ground power cart plugged in
inp.ext_power_switch    = 0
inp.rat_deploy          = 0   -- manual pull, or auto-deploy trigger elsewhere

-- Anti-ice and other big electric loads (787 uses electric wing/engine
-- anti-ice instead of bleed air — another real architectural difference)
inp.wing_anti_ice_on    = 0
inp.eng_anti_ice_on     = 0
inp.galley_power_on     = 1

-- Fault injection for testing failure procedures / checklists
inp.fail_eng1_gen       = 0
inp.fail_eng2_gen       = 0
inp.fail_apu_gen        = 0
inp.fail_backup_gen     = 0

-- ============================================================
-- OUTPUT / STATE DATAREFS — read by other modules and gauges
-- ============================================================
create_dataref_table("b789/elec/state", "Data")
local st = b789.elec.state

st.eng1_gen_online   = 0
st.eng2_gen_online   = 0
st.apu_gen1_online   = 0
st.apu_gen2_online   = 0
st.backup_gen_online = 0
st.ext_power_online  = 0
st.rat_online        = 0

st.bus_L1_powered = 0
st.bus_L2_powered = 0
st.bus_R1_powered = 0
st.bus_R2_powered = 0
st.bus_tie_L      = 0
st.bus_tie_R      = 0
st.backup_bus_L_powered = 0
st.backup_bus_R_powered = 0

st.battery_bus_powered = 0
st.battery_charge_pct  = 100.0

st.load_avionics_kva   = 0
st.load_hydraulics_kva = 0
st.load_galley_kva     = 0
st.load_anti_ice_kva   = 0
st.total_load_kva      = 0
st.total_gen_capacity_kva = 0
st.elec_system_healthy = 1   -- 0 = load exceeds capacity and no buffer left

-- ============================================================
-- HELPERS
-- ============================================================
local function engine_running(idx)
    return engn_running[idx] and engn_running[idx] > 0
end

-- Per-subsystem load estimate. Replace pieces of this as real modules
-- (hydraulics, galley, etc.) come online and can report their own draw.
local function compute_loads()
    st.load_avionics_kva = 40  -- FMS, displays, comms, lighting baseline

    -- 787 electric hydraulic pumps are the single biggest electric draw
    -- on the airframe — this is THE headline difference from older jets
    st.load_hydraulics_kva = 60

    st.load_galley_kva = (inp.galley_power_on == 1) and 25 or 0

    local ai_load = 0
    if inp.wing_anti_ice_on == 1 then ai_load = ai_load + 30 end
    if inp.eng_anti_ice_on  == 1 then ai_load = ai_load + 15 end
    st.load_anti_ice_kva = ai_load

    st.total_load_kva = st.load_avionics_kva + st.load_hydraulics_kva
        + st.load_galley_kva + st.load_anti_ice_kva
end

-- ============================================================
-- MAIN UPDATE
-- ============================================================
local function update_electrical()
    -- ---- Source availability (fault injection overrides switch state) ----
    st.eng1_gen_online = (engine_running(0) and inp.eng1_gen_switch == 1
        and inp.fail_eng1_gen == 0) and 1 or 0
    st.eng2_gen_online = (engine_running(1) and inp.eng2_gen_switch == 1
        and inp.fail_eng2_gen == 0) and 1 or 0

    local apu_gen_ok = (inp.apu_running == 1 and inp.apu_gen_switch == 1
        and inp.fail_apu_gen == 0)
    st.apu_gen1_online = apu_gen_ok and 1 or 0
    st.apu_gen2_online = apu_gen_ok and 1 or 0  -- simplified: both follow APU state

    st.backup_gen_online = (inp.backup_gen_switch == 1 and inp.fail_backup_gen == 0
        and (st.eng1_gen_online == 1 or st.eng2_gen_online == 1)) and 1 or 0

    st.ext_power_online = (inp.ext_power_available == 1 and inp.ext_power_switch == 1
        and st.eng1_gen_online == 0 and st.eng2_gen_online == 0) and 1 or 0

    local any_ac_source = (st.eng1_gen_online + st.eng2_gen_online
        + st.apu_gen1_online + st.apu_gen2_online + st.ext_power_online) > 0
    st.rat_online = (inp.rat_deploy == 1 and not any_ac_source) and 1 or 0

    -- ---- Bus assignment ----
    -- Own-side engine gen powers its own bus directly; otherwise the bus
    -- tie breaker can pull power from the opposite side, APU, or ext power.
    st.bus_L1_powered = (st.eng1_gen_online == 1) and 1 or 0
    st.bus_R1_powered = (st.eng2_gen_online == 1) and 1 or 0

    st.bus_tie_L = (st.bus_L1_powered == 0 and
        (st.apu_gen1_online == 1 or st.ext_power_online == 1 or st.eng2_gen_online == 1)) and 1 or 0
    st.bus_tie_R = (st.bus_R1_powered == 0 and
        (st.apu_gen2_online == 1 or st.ext_power_online == 1 or st.eng1_gen_online == 1)) and 1 or 0

    st.bus_L2_powered = (st.bus_L1_powered == 1 or st.bus_tie_L == 1) and 1 or 0
    st.bus_R2_powered = (st.bus_R1_powered == 1 or st.bus_tie_R == 1) and 1 or 0

    -- Backup buses: fed by backup gen or RAT even if main gens are all down
    local backup_source = (st.backup_gen_online == 1 or st.rat_online == 1)
    st.backup_bus_L_powered = (st.bus_L1_powered == 1 or backup_source) and 1 or 0
    st.backup_bus_R_powered = (st.bus_R1_powered == 1 or backup_source) and 1 or 0

    -- ---- Battery bus + charge model ----
    local any_bus_powered = (st.bus_L1_powered + st.bus_L2_powered
        + st.bus_R1_powered + st.bus_R2_powered) > 0

    st.battery_bus_powered = (inp.bat_switch == 1
        and (any_bus_powered or st.battery_charge_pct > 0)) and 1 or 0

    if st.battery_bus_powered == 1 then
        if any_bus_powered and st.battery_charge_pct < 100 then
            st.battery_charge_pct = math.min(100,
                st.battery_charge_pct + BATTERY_CHARGE_PCT_PER_MIN / 60 / UPDATE_HZ)
        elseif not any_bus_powered then
            st.battery_charge_pct = math.max(0,
                st.battery_charge_pct - BATTERY_DRAIN_PCT_PER_MIN / 60 / UPDATE_HZ)
        end
    end

    -- ---- Loads and capacity check ----
    compute_loads()

    st.total_gen_capacity_kva =
        (st.eng1_gen_online   * GEN_CAPACITY_KVA) +
        (st.eng2_gen_online   * GEN_CAPACITY_KVA) +
        (st.apu_gen1_online   * APU_GEN_CAPACITY_KVA) +
        (st.apu_gen2_online   * APU_GEN_CAPACITY_KVA) +
        (st.backup_gen_online * BACKUP_GEN_CAPACITY_KVA)

    if st.total_gen_capacity_kva > 0 then
        st.elec_system_healthy = (st.total_load_kva <= st.total_gen_capacity_kva) and 1 or 0
    else
        -- no generator online: only "healthy" if battery/RAT is holding buses up
        st.elec_system_healthy = (st.battery_bus_powered == 1 or st.rat_online == 1) and 1 or 0
    end
end

-- ============================================================
-- FRAME THROTTLING
-- FlyWithLua's do_every_frame runs every rendered frame; we don't need
-- electrical logic recalculated that often, so throttle to UPDATE_HZ.
-- ============================================================
local frame_accum = 0
local FRAMES_PER_UPDATE = 15  -- ~30fps / 2Hz update rate; adjust if needed

function b789_electrical_frame_callback()
    frame_accum = frame_accum + 1
    if frame_accum >= FRAMES_PER_UPDATE then
        frame_accum = 0
        update_electrical()
    end
end

do_every_frame("b789_electrical_frame_callback")

-- ============================================================
-- TESTING — no real aircraft needed
-- ============================================================
-- 1. Load this script via FlyWithLua on ANY aircraft (even the default
--    Cessna) — it only touches datarefs, not the model.
-- 2. Open the DataRefEditor plugin, search "b789/elec".
-- 3. Toggle b789/elec/input/eng1_gen_switch, fail_eng1_gen, etc. and
--    watch b789/elec/state/* respond in real time.
-- 4. Uncomment the block below to also see it logged to Log.txt.
--
-- function b789_elec_debug()
--     logMsg(string.format(
--         "L1=%d L2=%d R1=%d R2=%d BAT=%.1f%% cap=%dkVA load=%dkVA healthy=%d",
--         st.bus_L1_powered, st.bus_L2_powered, st.bus_R1_powered, st.bus_R2_powered,
--         st.battery_charge_pct, st.total_gen_capacity_kva, st.total_load_kva,
--         st.elec_system_healthy))
-- end
-- do_every_frame("b789_elec_debug")