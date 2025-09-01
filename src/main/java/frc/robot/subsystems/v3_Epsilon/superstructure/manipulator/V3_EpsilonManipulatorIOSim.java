package frc.robot.subsystems.v3_Epsilon.superstructure.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class V3_EpsilonManipulatorIOSim implements V3_EpsilonManipulatorIO {
  private final SingleJointedArmSim armMotorSim;
  private final DCMotorSim rollerMotorSim;

  private final ProfiledPIDController armFeedbackController;
  private final ArmFeedforward armFeedforwardController;

  private double[] slot0;
  private double[] slot1;
  private double[] slot2;

  private double armAppliedVolts;
  private double rollerAppliedVolts;

  private boolean isClosedLoop;

  public V3_EpsilonManipulatorIOSim() {
    armMotorSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                V3_EpsilonManipulatorConstants.ARM_PARAMETERS.MOTOR_CONFIG(),
                0.004,
                V3_EpsilonManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO()),
            V3_EpsilonManipulatorConstants.ARM_PARAMETERS.MOTOR_CONFIG(),
            V3_EpsilonManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO(),
            V3_EpsilonManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS(),
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            true,
            0.0);
    rollerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 3.0),
            DCMotor.getKrakenX60Foc(1));

    slot0 =
        new double[] {
          V3_EpsilonManipulatorConstants.EMPTY_GAINS.kP().get(),
          V3_EpsilonManipulatorConstants.EMPTY_GAINS.kD().get(),
          V3_EpsilonManipulatorConstants.EMPTY_GAINS.kS().get(),
          V3_EpsilonManipulatorConstants.EMPTY_GAINS.kV().get(),
          V3_EpsilonManipulatorConstants.EMPTY_GAINS.kA().get(),
          V3_EpsilonManipulatorConstants.EMPTY_GAINS.kG().get()
        };
    slot1 =
        new double[] {
          V3_EpsilonManipulatorConstants.CORAL_GAINS.kP().get(),
          V3_EpsilonManipulatorConstants.CORAL_GAINS.kD().get(),
          V3_EpsilonManipulatorConstants.CORAL_GAINS.kS().get(),
          V3_EpsilonManipulatorConstants.CORAL_GAINS.kV().get(),
          V3_EpsilonManipulatorConstants.CORAL_GAINS.kA().get(),
          V3_EpsilonManipulatorConstants.CORAL_GAINS.kG().get()
        };
    slot2 =
        new double[] {
          V3_EpsilonManipulatorConstants.ALGAE_GAINS.kP().get(),
          V3_EpsilonManipulatorConstants.ALGAE_GAINS.kD().get(),
          V3_EpsilonManipulatorConstants.ALGAE_GAINS.kS().get(),
          V3_EpsilonManipulatorConstants.ALGAE_GAINS.kV().get(),
          V3_EpsilonManipulatorConstants.ALGAE_GAINS.kA().get(),
          V3_EpsilonManipulatorConstants.ALGAE_GAINS.kG().get()
        };
    armFeedbackController =
        new ProfiledPIDController(
            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kP().get(),
            0.0,
            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kD().get(),
            new Constraints(
                V3_EpsilonManipulatorConstants.CONSTRAINTS
                    .cruisingVelocityRotationsPerSecond()
                    .get(),
                V3_EpsilonManipulatorConstants.CONSTRAINTS
                    .maxAccelerationRotationsPerSecondSquared()
                    .get()));
    armFeedforwardController =
        new ArmFeedforward(
            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kS().get(),
            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kG().get(),
            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kV().get(),
            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kA().get());
    armFeedbackController.enableContinuousInput(
        -Math.PI, Math.PI); // Wrap around at -180 and 180 degrees

    armAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
    isClosedLoop = true;
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    if (isClosedLoop) {
      armAppliedVolts =
          armFeedbackController.calculate(armMotorSim.getAngleRads())
              + armFeedforwardController.calculate(
                  armMotorSim.getAngleRads(), armMotorSim.getVelocityRadPerSec());
    }

    armAppliedVolts = MathUtil.clamp(armAppliedVolts, -12.0, 12.0);
    rollerAppliedVolts = MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0);

    armMotorSim.setInputVoltage(armAppliedVolts);
    armMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    rollerMotorSim.setInputVoltage(rollerAppliedVolts);
    rollerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.armPosition = Rotation2d.fromRadians(armMotorSim.getAngleRads());
    inputs.armVelocityRadiansPerSecond = armMotorSim.getVelocityRadPerSec();
    inputs.armAppliedVolts = armAppliedVolts;
    inputs.armSupplyCurrentAmps = armMotorSim.getCurrentDrawAmps();

    inputs.armPositionGoal = Rotation2d.fromRadians(armFeedbackController.getGoal().position);
    inputs.armPositionSetpoint =
        Rotation2d.fromRadians(armFeedbackController.getSetpoint().position);
    inputs.armPositionError = Rotation2d.fromRadians(armFeedbackController.getPositionError());

    inputs.rollerPosition = new Rotation2d(rollerMotorSim.getAngularPositionRad());
    inputs.rollerVelocityRadiansPerSecond = rollerMotorSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps = rollerMotorSim.getCurrentDrawAmps();
  }

  public void setArmVoltage(double volts) {
    isClosedLoop = false;
    armAppliedVolts = volts;
  }

  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }

  public void setArmGoal(Rotation2d rotation) {
    isClosedLoop = true;
    armFeedbackController.setGoal(rotation.getRadians());
  }

  public void updateSlot0ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    slot0[0] = kP;
    slot0[1] = kD;
    slot0[2] = kS;
    slot0[3] = kV;
    slot0[4] = kA;
    slot0[5] = kG;
  }

  public void updateSlot1ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    slot1[0] = kP;
    slot1[1] = kD;
    slot1[2] = kS;
    slot1[3] = kV;
    slot1[4] = kA;
    slot1[5] = kG;
  }

  public void updateSlot2ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    slot2[0] = kP;
    slot2[1] = kD;
    slot2[2] = kS;
    slot2[3] = kV;
    slot2[4] = kA;
    slot2[5] = kG;
  }

  public void setSlot(int slot) {
    if (slot >= 0 && slot <= 2) {
      double[] gains;
      if (slot == 0) {
        gains = slot0;
      } else if (slot == 1) {
        gains = slot1;
      } else {
        gains = slot2;
      }
      armFeedbackController.setPID(gains[0], 0.0, gains[1]);
      armFeedforwardController.setKa(gains[4]);
      armFeedforwardController.setKs(gains[2]);
      armFeedforwardController.setKv(gains[3]);
      armFeedforwardController.setKg(gains[5]);
    } else {
      throw new IllegalArgumentException("Slot must be between 0 and 2");
    }
  }

  public void updateArmConstraints(
      double cruisingVelocityRotationsPerSecond, double maxAccelerationRotationsPerSecondSquared) {
    armFeedbackController.setConstraints(
        new Constraints(
            cruisingVelocityRotationsPerSecond, maxAccelerationRotationsPerSecondSquared));
  }
}
