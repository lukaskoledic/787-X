--[[
    787-9 Hydraulics System Model
    ------------------------------
    For X-Plane 12 / FlyWithLua NG or CG.
    Requires 787_electrical.lua to be loaded (reads b789/elec/state/*).

    Real 787 architecture — genuinely different from older jets:
      - 3 independent systems: LEFT, CENTER, RIGHT
      - NO engine-driven pumps. Instead: electric motor-driven pumps (EDPs)
        fed straight off the AC buses, plus power transfer units and a
        center-system backup pump pair.
      - LEFT system: 2x EDP, powered off bus L1/L2
      - RIGHT system: 2x EDP, powered off bus R1/R2
      - CENTER system: 2x EDP normally, PLUS a Ram Air Turbine (RAT) driven
        pump as a mechanical backup that feeds Center directly if all
        electrical power is lost (bypasses the electrical system entirely,
        which is the point of it).
      - Each system nominally 3000 psi.

    This module only reads b789/elec/state/* — it never writes to it.
    Other modules (flight controls, gear, brakes) should read
    b789/hyd/state/* the same way.
]]--

-- ============================================================
-- CONFIG
-- ============================================================
local NOMINAL_PSI          = 3000
local SPINUP_PSI_PER_SEC   = 1500   -- how fast pressure builds once a pump is powered
local SPINDOWN_PSI_PER_SEC = 2000   -- how fast pressure bleeds off once unpowered
local UPDATE_HZ             = 2

-- ============================================================
-- DEPENDENCY CHECK
-- ============================================================
if b789 == nil or b789.elec == nil or b789.elec.state == nil then
    logMsg("787_hydraulics.lua: WARNING — b789/elec/state not found. " ..
        "Make sure 787_electrical.lua is loaded BEFORE this script. " ..
        "FlyWithLua loads scripts alphabetically, and \"787_electrical.lua\" " ..
        "already sorts before \"787_hydraulics.lua\" (e < h), so this works " ..
        "as long as both keep their current names in the same Scripts folder.")
end

-- ============================================================
-- CUSTOM INPUT DATAREFS
-- ============================================================
create_dataref_table("b789/hyd/input", "Data")
local inp = b789.hyd.input

-- Pump switches — pilot controls these; power availability (from electrical)
-- gates whether they actually spin up.
inp.left_edp1_switch    = 1
inp.left_edp2_switch    = 1
inp.right_edp1_switch   = 1
inp.right_edp2_switch   = 1
inp.center_edp1_switch  = 1
inp.center_edp2_switch  = 1

-- Power Transfer Unit: lets Center system borrow pressure from Left/Right
-- and vice versa without transferring fluid, if one system is weak
inp.ptu_switch          = 1

-- RAT: mechanical backup for Center system only, deploys automatically in
-- the real aircraft when all AC power is lost; here it's driven by the
-- electrical module's rat_online dataref, no separate input needed

-- Fault injection for testing failure procedures
inp.fail_left_sys       = 0
inp.fail_right_sys      = 0
inp.fail_center_sys     = 0

-- ============================================================
-- OUTPUT / STATE DATAREFS
-- ============================================================
create_dataref_table("b789/hyd/state", "Data")
local st = b789.hyd.state

st.left_edp1_online   = 0
st.left_edp2_online   = 0
st.right_edp1_online  = 0
st.right_edp2_online  = 0
st.center_edp1_online = 0
st.center_edp2_online = 0
st.center_rat_online  = 0

st.left_psi   = 0
st.right_psi  = 0
st.center_psi = 0

st.ptu_active = 0

st.left_sys_healthy   = 1
st.right_sys_healthy  = 1
st.center_sys_healthy = 1

-- ============================================================
-- HELPERS
-- ============================================================
local function elec_ok()
    return b789 ~= nil and b789.elec ~= nil and b789.elec.state ~= nil
end

-- Which AC bus powers which pump, matching the electrical module's bus layout
local function bus_L_powered()
    if not elec_ok() then return false end
    return b789.elec.state.bus_L1_powered == 1 or b789.elec.state.bus_L2_powered == 1
end
local function bus_R_powered()
    if not elec_ok() then return false end
    return b789.elec.state.bus_R1_powered == 1 or b789.elec.state.bus_R2_powered == 1
end
-- Center EDPs draw off whichever bus is available (both sides feed center)
local function bus_any_powered()
    return bus_L_powered() or bus_R_powered()
end

local function approach(current, target, max_step)
    if current < target then
        return math.min(target, current + max_step)
    elseif current > target then
        return math.max(target, current - max_step)
    end
    return current
end

-- ============================================================
-- MAIN UPDATE
-- ============================================================
local function update_hydraulics()
    local spin_up   = SPINUP_PSI_PER_SEC / UPDATE_HZ
    local spin_down = SPINDOWN_PSI_PER_SEC / UPDATE_HZ

    -- ---- LEFT system ----
    st.left_edp1_online = (inp.left_edp1_switch == 1 and bus_L_powered()
        and inp.fail_left_sys == 0) and 1 or 0
    st.left_edp2_online = (inp.left_edp2_switch == 1 and bus_L_powered()
        and inp.fail_left_sys == 0) and 1 or 0
    local left_target = (st.left_edp1_online == 1 or st.left_edp2_online == 1)
        and NOMINAL_PSI or 0

    -- ---- RIGHT system ----
    st.right_edp1_online = (inp.right_edp1_switch == 1 and bus_R_powered()
        and inp.fail_right_sys == 0) and 1 or 0
    st.right_edp2_online = (inp.right_edp2_switch == 1 and bus_R_powered()
        and inp.fail_right_sys == 0) and 1 or 0
    local right_target = (st.right_edp1_online == 1 or st.right_edp2_online == 1)
        and NOMINAL_PSI or 0

    -- ---- CENTER system ----
    st.center_edp1_online = (inp.center_edp1_switch == 1 and bus_any_powered()
        and inp.fail_center_sys == 0) and 1 or 0
    st.center_edp2_online = (inp.center_edp2_switch == 1 and bus_any_powered()
        and inp.fail_center_sys == 0) and 1 or 0

    local center_elec_pumped = (st.center_edp1_online == 1 or st.center_edp2_online == 1)

    -- RAT covers Center only, and only if no electrical source is powering it
    st.center_rat_online = (elec_ok() and b789.elec.state.rat_online == 1
        and not center_elec_pumped and inp.fail_center_sys == 0) and 1 or 0

    local center_target = (center_elec_pumped or st.center_rat_online == 1)
        and NOMINAL_PSI or 0

    -- ---- Power Transfer Unit ----
    -- If PTU is on and one system is unpressurized while an adjacent one
    -- isn't, PTU partially props up the weak side (simplified: +60% of
    -- nominal, no fluid transfer, just pressure equivalence for gameplay)
    st.ptu_active = 0
    if inp.ptu_switch == 1 then
        if left_target == 0 and (right_target > 0 or center_target > 0) then
            left_target = NOMINAL_PSI * 0.6
            st.ptu_active = 1
        end
        if right_target == 0 and (left_target > 0 or center_target > 0) then
            right_target = NOMINAL_PSI * 0.6
            st.ptu_active = 1
        end
    end

    -- ---- Pressure ramping (avoids instant snap to target) ----
    st.left_psi   = approach(st.left_psi, left_target,
        left_target > st.left_psi and spin_up or spin_down)
    st.right_psi  = approach(st.right_psi, right_target,
        right_target > st.right_psi and spin_up or spin_down)
    st.center_psi = approach(st.center_psi, center_target,
        center_target > st.center_psi and spin_up or spin_down)

    -- ---- Health flags (below ~80% nominal = degraded) ----
    st.left_sys_healthy   = (st.left_psi   >= NOMINAL_PSI * 0.8) and 1 or 0
    st.right_sys_healthy  = (st.right_psi  >= NOMINAL_PSI * 0.8) and 1 or 0
    st.center_sys_healthy = (st.center_psi >= NOMINAL_PSI * 0.8) and 1 or 0
end

-- ============================================================
-- FRAME THROTTLING
-- ============================================================
local frame_accum = 0
local FRAMES_PER_UPDATE = 15  -- ~30fps / 2Hz

function b789_hydraulics_frame_callback()
    frame_accum = frame_accum + 1
    if frame_accum >= FRAMES_PER_UPDATE then
        frame_accum = 0
        update_hydraulics()
    end
end

do_every_frame("b789_hydraulics_frame_callback")

-- ============================================================
-- TESTING — no real aircraft needed
-- ============================================================
-- 1. Load BOTH 787_electrical.lua and this script (electrical must load
--    first — see the dependency check warning above for the naming trick).
-- 2. In DataRefEditor, search "b789/hyd".
-- 3. Toggle engine gens off in b789/elec/input and watch left_psi /
--    right_psi decay in b789/hyd/state as their bus power drops.
-- 4. Toggle b789/hyd/input/fail_left_sys to 1 and watch ptu_active kick
--    in to partially prop up left_psi if right or center still has power.
--
-- function b789_hyd_debug()
--     logMsg(string.format(
--         "L=%.0f R=%.0f C=%.0f PTU=%d RAT=%d",
--         st.left_psi, st.right_psi, st.center_psi, st.ptu_active, st.center_rat_online))
-- end
-- do_every_frame("b789_hyd_debug")