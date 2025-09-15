package frc.robot.subsystems.v3_Epsilon.climber;

public class V3_EpsilonClimberConstants {
    public static final int DEPLOYMENT_CAN_ID;

    public static final int ROLLER_CAN_ID;

    static {
        DEPLOYMENT_CAN_ID = 42;
        ROLLER_CAN_ID = 42; // This used to be 61, but there are two motors, so I replace this with 42 until
    }

    public static final double CLIMBER_CLIMBED_ZERO_RADIANS = 41; //TODO: None of these four lines are valid numbers i just made up stuff
    public static final double CLIMBER_CLIMBED_DEPLOYED_RADIANS = 67; 
    public static final ClimberTimingConfig CLIMBER_TIMING_CONFIG = new ClimberTimingConfig(1.1, 0.25, 0.5);
    public static final CurrentLimits CURRENT_LIMITS = new CurrentLimits(40.0, 80.0);

    public static record CurrentLimits(double SUPPLY_CURRENT_LIMIT, double STATOR_CURRENT_LIMIT) {
    }

    public static record ClimberTimingConfig(
            double WAIT_AFTER_RELEASE_SECONDS,
            double REDUNDANCY_DELAY_SECONDS,
            double REDUNDANCY_TRUSTING_TIMEOUT_SECONDS) {
    }

}
