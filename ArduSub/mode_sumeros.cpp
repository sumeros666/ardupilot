#include "Sub.h"

bool ModeSumeros::init(bool ignore_checks)
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_command[i] = 0.0f;
    }

    last_command_ms = 0;
    command_received = false;
    fault_latched = false;

    return true;
}

bool ModeSumeros::set_actuator_target(
		const mavlink_set_actuator_control_target_t &packet)
{
    if (fault_latched) return false;

    // packet validation
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (!isfinite(packet.controls[i])) {
		return false;
	}

	if (packet.controls[i] < -1.0f ||
	    packet.controls[i] > 1.0f) {
		return false;
	}
    }

    // if validation passed, copy values
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_command[i] = packet.controls[i];
    }

    last_command_ms = AP_HAL::millis();
    command_received = true;

    return true;
}

void ModeSumeros::run()
{
    // arm check.
    if (!motors.armed()) {
        motors.set_desired_spool_state(
            AP_Motors::DesiredSpoolState::GROUND_IDLE);
        motors.clear_direct_motor_thrust();
        return;
    }

    // dont allow thrust until valid command exists
    if (!command_received || fault_latched) {
        motors.set_desired_spool_state(
            AP_Motors::DesiredSpoolState::GROUND_IDLE);
        motors.clear_direct_motor_thrust();
        return;
    }

    const uint32_t now = AP_HAL::millis();

    // watchdog
    if ((now - last_command_ms) > COMMAND_TIMEOUT_MS) {
        trigger_failsafe();
	return;
    }

    // if everything is alright, let's go
    motors.set_desired_spool_state(
        AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motors.set_direct_motor_thrust(i, motor_command[i]);
    }
}

void ModeSumeros::trigger_failsafe()
{
    fault_latched = true;
    command_received = false;

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_command[i] = 0.0f;
    }

    motors.clear_direct_motor_thrust();

    gcs().send_text(
        MAV_SEVERITY_CRITICAL,
        "SUMEROS: command timeout");

    if (sub.motors.armed()) {
        sub.arming.disarm(AP_Arming::Method::PILOT_INPUT_FAILSAFE);
    }
}


