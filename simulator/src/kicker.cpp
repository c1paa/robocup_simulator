#include "kicker.h"
#include "config.h"
#include "robot.h"
#include "ball.h"
#include <algorithm>
#include <cmath>

static const float MM = 1000.0f;

void Kicker::init(Config& cfg)
{
    m_workingVoltage = cfg.getFloat("/robot/battery/working_voltage", 16.0f);
    m_battInternalR  = cfg.getFloat("/robot/battery/internal_resistance", 0.05f);
    if (m_battInternalR < 0.0f) m_battInternalR = 0.0f;

    m_capacitance     = cfg.getFloat("/robot/capacitor/capacitance_uf", 20000.0f) * 1.0e-6f;
    m_chargeVoltage    = cfg.getFloat("/robot/capacitor/charge_voltage", 48.0f);
    m_chargeResistance = cfg.getFloat("/robot/capacitor/charge_resistance", 2.0f);
    m_boostEfficiency  = cfg.getFloat("/robot/capacitor/boost_efficiency", 0.85f);
    m_boostMaxCurrent  = cfg.getFloat("/robot/capacitor/boost_max_current", 5.0f);
    m_capEsr           = cfg.getFloat("/robot/capacitor/esr", 0.05f);
    m_leakageR         = cfg.getFloat("/robot/capacitor/leakage_resistance", 1.0e6f);
    if (m_capacitance <= 0.0f) m_capacitance = 1.0e-6f;
    if (m_chargeResistance <= 0.0f) m_chargeResistance = 0.01f;
    if (m_leakageR <= 0.0f) m_leakageR = 1.0e6f;

    m_coilResistance = cfg.getFloat("/robot/kicker/coil_resistance", 0.6f);
    m_coilInductance = cfg.getFloat("/robot/kicker/coil_inductance", 0.0008f);
    m_forceConstant  = cfg.getFloat("/robot/kicker/force_constant", 900.0f);
    m_armatureMass   = cfg.getFloat("/robot/kicker/armature_mass", 0.015f);
    m_stroke         = cfg.getFloat("/robot/kicker/stroke", 8.0f) / MM;
    m_springConstant = cfg.getFloat("/robot/kicker/spring_constant", 300.0f);
    m_dampingCoeff   = cfg.getFloat("/robot/kicker/damping_coefficient", 5.0f);
    m_restitution    = cfg.getFloat("/robot/kicker/restitution", 0.3f);
    m_shortCircuitR  = cfg.getFloat("/robot/kicker/short_circuit_resistance", 0.05f);
    m_debugImpulse   = cfg.getFloat("/robot/kicker/debug_impulse", 6.0f);
    if (m_coilResistance <= 0.0f) m_coilResistance = 0.01f;
    if (m_coilInductance <= 0.0f) m_coilInductance = 1.0e-6f;
    if (m_armatureMass <= 0.0f) m_armatureMass = 1.0e-3f;
    if (m_shortCircuitR <= 0.0f) m_shortCircuitR = 0.01f;

    m_heightOffset = cfg.getFloat("/robot/kicker/height_offset", 0.0f) / MM;
    m_range = cfg.getFloat("/robot/kicker/range", 40.0f) / MM;
    // The plunger sits behind/below the dribbler; reuse the dribbler's forward
    // offset as the kick origin (the kicker config has no forward key of its
    // own — see docs/tasks/dribbler-kicker.md).
    m_forwardOffset = cfg.getFloat("/robot/dribbler/forward_offset", 95.0f) / MM;
    float chipMaxAngleDeg = cfg.getFloat("/robot/kicker/chip_max_angle", 30.0f);
    m_chipMaxAngle = chipMaxAngleDeg * 3.14159265f / 180.0f;

    m_capVoltage = 0.0f; // starts empty, like real hardware at power-on
    m_busVoltage = m_workingVoltage;
    m_coilCurrent = 0.0f;
    m_armaturePos = 0.0f;
    m_armatureVel = 0.0f;
    m_hasFiredThisStroke = false;
}

// Charging path: boost converter (regulated to m_chargeVoltage, but current-
// limited to m_boostMaxCurrent at its input) -> m_chargeResistance -> cap.
// Solved with the *exact* exponential RC solution rather than an explicit
// Euler step, so it's stable regardless of how large dt gets relative to the
// tau = R*C time constant (this codebase has been bitten before by per-step
// deadbeat math that only behaves at small dt -- see the "posErr / dt was a
// bug" note in docs/tasks/dribbler-kicker.md; same class of mistake, avoided
// here from the start).
void Kicker::stepCharging(float dt)
{
    // Passive leakage always applies, charging or not.
    m_capVoltage -= dt * m_capVoltage / (m_leakageR * m_capacitance);
    if (m_capVoltage < 0.0f) m_capVoltage = 0.0f;

    if (!m_capacitorSwitchOpen) return;

    float tau = m_chargeResistance * m_capacitance;
    float vBefore = m_capVoltage;
    float vTarget = m_chargeVoltage;
    float vAfter = vTarget - (vTarget - vBefore) * std::exp(-dt / tau);

    // Boost converter's own input current limit: if the ideal RC curve above
    // implies more average charging current than the converter can actually
    // draw from the bus, cap the voltage step instead (charging takes longer
    // under a current-limited converter than the unconstrained RC curve
    // would suggest -- a real, not simplified-away, effect).
    float avgChargeCurrent = m_capacitance * (vAfter - vBefore) / dt;
    float maxChargeCurrentAtOutput = m_boostMaxCurrent * m_busVoltage * m_boostEfficiency / vTarget;
    if (avgChargeCurrent > maxChargeCurrentAtOutput && maxChargeCurrentAtOutput > 0.0f) {
        avgChargeCurrent = maxChargeCurrentAtOutput;
        vAfter = vBefore + avgChargeCurrent * dt / m_capacitance;
    }
    if (vAfter > vTarget) vAfter = vTarget;
    m_capVoltage = vAfter;

    // Bus sag from this charging current, referred back through the boost
    // converter (P_bus = P_cap-side / efficiency).
    float outputPower = avgChargeCurrent * vTarget;
    float busCurrent = m_boostEfficiency > 0.0f ? outputPower / (m_boostEfficiency * m_workingVoltage) : 0.0f;
    m_busVoltage = std::max(0.0f, m_workingVoltage - busCurrent * m_battInternalR);
}

// One electrical/mechanical substep of the discharge (capacitor -> coil ->
// armature). Symplectic (semi-implicit) Euler: current is advanced from the
// *old* capacitor voltage, then the capacitor voltage is advanced from the
// *new* current. This is the standard trick for keeping an oscillatory
// (underdamped RLC / spring-mass) system numerically stable at a fixed step
// size, unlike fully-explicit Euler which visibly gains energy over many
// steps -- same reasoning as a symplectic integrator for a spring, applied
// here to the coil/capacitor pair.
void Kicker::stepDischarge(float subDt)
{
    float rTotal = m_coilResistance + m_capEsr;
    m_coilCurrent += subDt * (m_capVoltage - m_coilCurrent * rTotal) / m_coilInductance;
    if (m_coilCurrent < 0.0f) m_coilCurrent = 0.0f; // flyback-diode assumption:
    // real solenoid drivers clamp reverse current, so once the capacitor is
    // spent the coil current free-wheels down through rTotal instead of
    // driving the capacitor negative -- see the doc comment in kicker.h.

    m_capVoltage -= subDt * m_coilCurrent / m_capacitance;
    if (m_capVoltage < 0.0f) m_capVoltage = 0.0f;

    float solenoidForce = m_forceConstant * m_coilCurrent * m_coilCurrent;
    float springForce = m_springConstant * m_armaturePos;
    float dampingForce = m_dampingCoeff * m_armatureVel;
    float accel = (solenoidForce - springForce - dampingForce) / m_armatureMass;

    m_armatureVel += subDt * accel;
    m_armaturePos += subDt * m_armatureVel;

    if (m_armaturePos >= m_stroke) {
        m_armaturePos = m_stroke;
        if (m_armatureVel > 0.0f && !m_hasFiredThisStroke) {
            deliverImpulse(m_armatureVel);
            m_hasFiredThisStroke = true;
        }
        if (m_armatureVel > 0.0f) m_armatureVel = 0.0f; // mechanical hard stop
    } else if (m_armaturePos <= 0.0f) {
        m_armaturePos = 0.0f;
        if (m_armatureVel < 0.0f) m_armatureVel = 0.0f;
        m_hasFiredThisStroke = false; // fully retracted: ready to fire again
    }
}

// 1D collision between the armature (mass m_armatureMass, speed
// armatureSpeed at the moment it reaches full stroke) and the ball (assumed
// at rest along the strike axis, a fair approximation on the kick's ~ms
// timescale), with a restitution coefficient -- the standard elastic-with-
// restitution formula for a much-lighter striker against a much-heavier
// resting body reduces to v_ball = (1+e) * (m_armature / (m_armature +
// m_ball)) * v_armature. Only fires if the ball is actually in range, exactly
// like the pre-rewrite point-distance check.
void Kicker::deliverImpulse(float armatureSpeed)
{
    if (!m_robot || !m_ball) return;

    float ballSpeed = (1.0f + m_restitution) *
        (m_armatureMass / (m_armatureMass + m_ball->mass())) * armatureSpeed;
    float impulseMag = ballSpeed * m_ball->mass();
    applyKickImpulse(*m_robot, *m_ball, impulseMag);
}

// Same chip-angle reasoning as the pre-electrical-rewrite model: a flat
// plunger contacting the ball has a horizontal contact normal regardless of
// contact height (applyImpulse's linear-velocity change is independent of
// the offset point), so a negative height_offset also tilts the impulse
// vector itself, approximating a hinged/angled plate -- see the long-form
// derivation in this file's git history (pre-rewrite version) if the
// mechanics need re-deriving.
bool Kicker::applyKickImpulse(const Robot& robot, Ball& ball, float impulseMag)
{
    if (!ball.body() || impulseMag <= 0.0f) return false;

    glm::vec3 robotPos = robot.position();
    float yaw = robot.orientation();
    float cosYaw = std::cos(yaw), sinYaw = std::sin(yaw);
    glm::vec3 forwardDir(cosYaw, 0.0f, -sinYaw);

    glm::vec3 kickOrigin = robotPos + forwardDir * m_forwardOffset;
    kickOrigin.y = ball.radius();
    float dist = glm::length(ball.position() - kickOrigin);
    if (dist > m_range) return false; // armature strikes air, no ball there

    float chipAngle = 0.0f;
    if (m_heightOffset < 0.0f) {
        float t = std::min(1.0f, -m_heightOffset / ball.radius());
        chipAngle = t * m_chipMaxAngle;
    }
    float cosA = std::cos(chipAngle), sinA = std::sin(chipAngle);

    btVector3 contactOffset(0.0f, m_heightOffset, 0.0f);
    ball.body()->activate(true);
    ball.body()->applyImpulse(
        btVector3(forwardDir.x * impulseMag * cosA, impulseMag * sinA, forwardDir.z * impulseMag * cosA),
        contactOffset);
    return true;
}

void Kicker::update(float dt)
{
    bool shortCircuit = m_capacitorSwitchOpen && m_kickerSwitchOpen;

    if (shortCircuit) {
        // Both gates open at once: the discharge path (coil, ~m_coilResistance)
        // is a much lower-impedance route than the charge path's current-
        // limiting m_chargeResistance, so current preferentially dumps
        // through the short instead of into the capacitor -- charging stops.
        // Deliberately does NOT apply m_boostMaxCurrent here (unlike the
        // normal charging path above): that limit models the boost
        // converter's own regulated current limit, which a real cheap boost
        // module's protection can't be trusted to enforce against a genuine
        // downstream short (the whole reason this is a *fault* state and not
        // just "charging slower"). Modeled as a direct resistive load across
        // the battery (m_battInternalR + m_shortCircuitR, Ohm's law,
        // bypassing the boost stage's voltage conversion entirely) rather
        // than re-deriving the exact parallel-path circuit -- the real
        // topology isn't known precisely; this reproduces the requested
        // symptom (severe, real bus sag) without pretending to more
        // topological precision than is actually known.
        float faultCurrent = m_workingVoltage / (m_battInternalR + m_shortCircuitR);
        m_busVoltage = std::max(0.0f, m_workingVoltage - faultCurrent * m_battInternalR);

        // Only a share of the fault current reaches the coil branch (current
        // divider against the coil's own resistance) -- driving a weak,
        // erratic armature response instead of a clean capacitor-fed strike.
        // No capacitor state change: it neither charges nor discharges
        // coherently while the fault persists.
        m_coilCurrent = faultCurrent * m_shortCircuitR / (m_shortCircuitR + m_coilResistance + m_capEsr);
        float solenoidForce = m_forceConstant * m_coilCurrent * m_coilCurrent;
        float accel = (solenoidForce - m_springConstant * m_armaturePos - m_dampingCoeff * m_armatureVel) / m_armatureMass;
        m_armatureVel += dt * accel;
        m_armaturePos = std::clamp(m_armaturePos + dt * m_armatureVel, 0.0f, m_stroke);
        return;
    }

    stepCharging(dt);

    if (m_kickerSwitchOpen) {
        int substeps = std::min(m_maxSubsteps, std::max(1, (int)std::ceil(dt / m_electricalSubstepDt)));
        float subDt = dt / (float)substeps;
        for (int i = 0; i < substeps; ++i) {
            stepDischarge(subDt);
        }
    } else {
        // Switch closed -> open again: let the spring return the armature to
        // rest over subsequent frames using the same mechanics (no coil
        // drive current), same substep loop but with m_coilCurrent forced
        // toward 0 via its own RL decay (there's no source voltage without
        // the switch closed, so the coil current naturally free-wheels down
        // through rTotal -- stepDischarge handles this correctly since
        // m_capVoltage isn't what's driving retraction here, the coil's own
        // stored energy plus the spring are).
        int substeps = std::min(m_maxSubsteps, std::max(1, (int)std::ceil(dt / m_electricalSubstepDt)));
        float subDt = dt / (float)substeps;
        float savedCapVoltage = m_capVoltage;
        m_capVoltage = 0.0f; // coil sees no source once the switch is open
        for (int i = 0; i < substeps; ++i) {
            stepDischarge(subDt);
        }
        m_capVoltage = savedCapVoltage; // restore: this branch must not drain
                                          // the capacitor, only relax the coil
    }

    // Bus relaxes to nominal once neither switch draws current from it (the
    // charging branch already reflects its own draw into m_busVoltage above;
    // discharge draws from the capacitor, not the bus, so it doesn't sag it).
    if (!m_capacitorSwitchOpen) {
        m_busVoltage = m_workingVoltage;
    }
}

void Kicker::debugFire(const Robot& robot, Ball& ball)
{
    applyKickImpulse(robot, ball, m_debugImpulse);
}
