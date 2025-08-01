package frc.robot.subsystems.v3_Epsilon.manipulator;

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
            V3_EpsilonManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians(),
            V3_EpsilonManipulatorConstants.ARM_PARAMETERS.MAX_ANGLE().getRadians(),
            true,
            V3_EpsilonManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians());
    rollerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 3.0),
            DCMotor.getKrakenX60Foc(1));

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
        V3_EpsilonManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians(),
        V3_EpsilonManipulatorConstants.ARM_PARAMETERS.MAX_ANGLE().getRadians());

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

  public void setPivotVoltage(double volts) {
    isClosedLoop = false;
    armAppliedVolts = volts;
  }

  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }

  public void setPivotGoal(Rotation2d rotation) {
    isClosedLoop = true;
    armFeedbackController.setGoal(rotation.getRadians());
  }

  public void updateSlot0ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    armFeedbackController.setPID(kP, 0.0, kD);
    armFeedforwardController.setKa(kA);
    armFeedforwardController.setKs(kS);
    armFeedforwardController.setKv(kV);
    armFeedforwardController.setKg(kG);
  }

  public void updateArmConstraints(
      double cruisingVelocityRotationsPerSecond, double maxAccelerationRotationsPerSecondSquared) {
    armFeedbackController.setConstraints(
        new Constraints(
            cruisingVelocityRotationsPerSecond, maxAccelerationRotationsPerSecondSquared));
  }
}
