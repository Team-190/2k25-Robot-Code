package frc.robot.subsystems.v3_Epsilon.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class V3_EpsilonClimberConstants {
    public static final int DEPLOYMENT_CAN_ID;

    public static final int ROLLER_CAN_ID;

    public static final MotorParameters MOTOR_PARAMETERS;

    public static final double CLIMBER_CLIMBED_RADIANS;

    static {
        DEPLOYMENT_CAN_ID = 50;
        ROLLER_CAN_ID = 51;

        MOTOR_PARAMETERS =
                new MotorParameters(DCMotor.getKrakenX60Foc(1), 16.0, 0.81, Units.inchesToMeters(1.78));

        CLIMBER_CLIMBED_RADIANS = -400;
    }

    public static record MotorParameters(
            DCMotor MOTOR_CONFIG, double GEAR_RATIO, double GEARBOX_EFFICIENCY, double SPOOL_DIAMETER) {
    }

    public static final double CLIMBER_CLIMBED_ZERO_RADIANS = 41;
    public static final double CLIMBER_CLIMBED_DEPLOYED_RADIANS = -10;
    public static final ClimberTimingConfig CLIMBER_TIMING_CONFIG =
            new ClimberTimingConfig(1.1, 0.25, 0.5);
    public static final CurrentLimits CURRENT_LIMITS = new CurrentLimits(40.0, 80.0);

    public static record CurrentLimits(double SUPPLY_CURRENT_LIMIT, double STATOR_CURRENT_LIMIT) {
    }

    public static record ClimberTimingConfig(
            double WAIT_AFTER_RELEASE_SECONDS,
            double REDUNDANCY_DELAY_SECONDS,
            double REDUNDANCY_TRUSTING_TIMEOUT_SECONDS) {
    }
}
