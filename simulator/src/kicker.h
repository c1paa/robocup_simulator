#pragma once

#include "common.h"

class Config;
class Robot;
class Ball;

// Kicker + capacitor + battery bus, simulated as a real electrical/mechanical
// system rather than trusted from the client. Two independent switches, each
// held open/closed for exactly as long as the client wants (mirrors the real
// hardware's two MOSFET gates):
//   - openCapacitor()/closeCapacitor(): charging path (boost converter ->
//     charge_resistance -> capacitor).
//   - openKicker()/closeKicker(): discharge path (capacitor -> solenoid coil
//     -> armature -> ball).
// debugFire() is the F-key debug shortcut: a fixed impulse that bypasses the
// capacitor/electrical model entirely (doesn't touch m_capVoltage), matching
// "F always hits regardless of charge, no capacitor involved" from the
// design discussion -- see docs/tasks/dribbler-kicker.md's "Electrical
// rewrite" section for the full derivation and the reasoning behind every
// simplification below.
class Kicker
{
public:
    void init(Config& cfg);

    // Advance the electrical/mechanical simulation by dt (one sim tick).
    // Internally sub-steps the coil RL / armature ODEs at a much finer
    // resolution than dt (see m_electricalSubstepDt) since their real time
    // constants are ~1ms, far faster than a physics frame.
    void update(float dt);

    // Persistent switch state -- call every time the client wants the state
    // changed; stays in effect until the corresponding close*() call, exactly
    // like Dribbler::setTargetSpeed.
    void openCapacitor()  { m_capacitorSwitchOpen = true; }
    void closeCapacitor() { m_capacitorSwitchOpen = false; }
    void openKicker()     { m_kickerSwitchOpen = true; }
    void closeKicker()    { m_kickerSwitchOpen = false; }
    void setCapacitorOpen(bool open) { m_capacitorSwitchOpen = open; }
    void setKickerOpen(bool open)    { m_kickerSwitchOpen = open; }

    // Needs the ball/robot every tick (not just at kick time) because impact
    // detection happens mid-update, whenever the armature's simulated stroke
    // reaches the ball-contact position -- update() must be given both.
    void setActors(Robot* robot, Ball* ball) { m_robot = robot; m_ball = ball; }

    // Debug-only: fire a fixed impulse (SDL 'F' key), completely bypassing
    // the capacitor -- see the class comment.
    void debugFire(const Robot& robot, Ball& ball);

    // ---- Telemetry (published over gRPC) ----
    float capacitorVoltage() const { return m_capVoltage; }
    float chargeVoltage() const { return m_chargeVoltage; }
    float charge() const { return m_chargeVoltage > 0.0f ? m_capVoltage / m_chargeVoltage : 0.0f; }
    float busVoltage() const { return m_busVoltage; }

private:
    // ---- Config: battery/bus ----
    float m_workingVoltage = 16.0f;  // V, nominal bus voltage at zero load
    float m_battInternalR = 0.05f;   // Ohm, battery pack ESR

    // ---- Config: capacitor + charging path ----
    float m_capacitance = 0.02f;      // F (config gives uF)
    float m_chargeVoltage = 48.0f;    // V, boost converter regulated target
    float m_chargeResistance = 2.0f;  // Ohm, series charge-current limiter
    float m_boostEfficiency = 0.85f;  // 0..1
    float m_boostMaxCurrent = 5.0f;   // A, boost converter input current limit
    float m_capEsr = 0.05f;           // Ohm, capacitor's own series resistance
    float m_leakageR = 1.0e6f;        // Ohm, slow self-discharge

    // ---- Config: kicker coil + armature ----
    float m_coilResistance = 0.6f;     // Ohm
    float m_coilInductance = 0.0008f;  // H
    float m_forceConstant = 900.0f;    // N/A^2, solenoid force = k * I^2
    float m_armatureMass = 0.015f;     // kg
    float m_stroke = 0.008f;           // m, plunger travel to ball contact
    float m_springConstant = 300.0f;   // N/m, return spring
    float m_dampingCoeff = 5.0f;       // N*s/m, mechanical damping
    float m_restitution = 0.3f;        // 0..1, plunger-ball impact
    float m_shortCircuitR = 0.1f;      // Ohm, fault-path resistance when both
                                        // switches are open at once
    float m_debugImpulse = 6.0f;       // N*s, fixed F-key debug kick

    float m_heightOffset = 0.0f;    // m, plunger height relative to ball center
    float m_range = 0.04f;          // m, max ball-to-kicker distance
    float m_forwardOffset = 0.095f; // m, kick origin forward of chassis center
    float m_chipMaxAngle = 0.0f;    // rad, see Kicker::deliverImpulse

    // ---- Electrical/mechanical state ----
    float m_capVoltage = 0.0f;   // V, starts empty (real hardware powers on
                                  // with the cap discharged -- client must
                                  // charge it before it can fire, same as a
                                  // real robot's bring-up sequence)
    float m_busVoltage = 16.0f;  // V, current sagged bus voltage
    float m_coilCurrent = 0.0f;  // A
    float m_armaturePos = 0.0f;  // m, 0 = retracted, m_stroke = fully extended
    float m_armatureVel = 0.0f;  // m/s
    bool m_hasFiredThisStroke = false; // armature already delivered its
                                        // impact impulse this extension

    bool m_capacitorSwitchOpen = false;
    bool m_kickerSwitchOpen = false;

    // Electrical/mechanical sub-step target (see update()'s doc comment).
    static constexpr float m_electricalSubstepDt = 0.00005f; // 50 us
    static constexpr int m_maxSubsteps = 400; // caps worst-case cost/frame

    Robot* m_robot = nullptr;
    Ball* m_ball = nullptr;

    void stepCharging(float dt);
    void stepDischarge(float subDt); // one electrical/mechanical substep
    void deliverImpulse(float armatureSpeed);

    // Shared by deliverImpulse and debugFire: range-checks the ball against
    // the kick origin and, if in range, applies impulseMag (N*s) along the
    // robot's forward direction, tilted upward by the chip-angle model for
    // negative height_offset. Returns false (no-op) if out of range.
    bool applyKickImpulse(const Robot& robot, Ball& ball, float impulseMag);
};
