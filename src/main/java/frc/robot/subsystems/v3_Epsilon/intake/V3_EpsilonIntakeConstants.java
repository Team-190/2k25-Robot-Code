package frc.robot.subsystems.v3_Epsilon.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import lombok.RequiredArgsConstructor;
import lombok.Getter;

public class V3_EpsilonIntakeConstants {
    public static final int PIVOT_CAN_ID;
    
    public static final int ROLLER_CAN_ID;

    public static final IntakeCurrentLimits CURRENT_LIMITS = new IntakeCurrentLimits(
        40.0, 
        40.0, 
        40.0, 
        40.0  
    );

    public static final Gains PIVOT_GAINS = new Gains(
        100.0, 
        0.0,   
        0.5,   
        0.0,   
        0.0,
        0.0    
    );
    public static final Constraints PIVOT_CONSTRAINTS = new Constraints(
        500.0, 
        500.0, 
        Rotation2d.fromDegrees(0.01) 
    );

    public static final IntakeParems PIVOT_PARAMS = new IntakeParems(
        3.0, 
        DCMotor.getKrakenX60Foc(1), 
        0.0042, 
        Rotation2d.fromDegrees(-90.0), 
        Rotation2d.fromDegrees(90.0) 
    );
    public static final IntakeParems ROLLER_PARAMS = new IntakeParems(
        1, 
        DCMotor.getKrakenX60Foc(1), 
        0,
        new Rotation2d(), 
        Rotation2d.fromDegrees(90.0)
    );


    static {
        PIVOT_CAN_ID = 60;
        ROLLER_CAN_ID = 61;
    }

  @RequiredArgsConstructor
  public enum IntakeState {
    STOW(new Rotation2d()),
    INTAKE_CORAL(new Rotation2d()),
    HANDOFF(new Rotation2d(Units.degreesToRadians(90))),
    L1(new Rotation2d()),
    INTAKE_ALGAE(new Rotation2d());

    @Getter private final Rotation2d angle;
  }

  public static record IntakeCurrentLimits(
    double PIVOT_SUPPLY_CURRENT_LIMIT,
    double PIVOT_STATOR_CURRENT_LIMIT,
    double ROLLER_SUPPLY_CURRENT_LIMIT,
    double ROLLER_STATOR_CURRENT_LIMIT
  ) {
  }

  public static record Gains(
    double kP,
    double kD,
    double kS,
    double kV,
    double kA,
    double kG
  ) {
  }

  public static record Constraints(
    double MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED,
    double CRUISING_VELOCITY_RADIANS_PER_SECOND,
    Rotation2d GOAL_TOLERANCE
  ) {
    public edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints getTrapezoidConstraints() {
      return new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
        CRUISING_VELOCITY_RADIANS_PER_SECOND,
        MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED
      );
    }
  }

  public static record IntakeParems(
    double PIVOT_GEAR_RATIO,
    DCMotor MOTOR,
    double MASS_KG,
    Rotation2d MIN_ANGLE,
    Rotation2d MAX_ANGLE
  ) {
  }
}
