package frc.robot.subsystems.v2_Redundancy.superstructure.manipulator;

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

public class ManipulatorIOSim implements ManipulatorIO {
  private final SingleJointedArmSim armSim;
  private final DCMotorSim rollerSim;

  private double armAppliedVolts;
  private double rollerAppliedVolts;

  private final ProfiledPIDController feedback;
  private ArmFeedforward feedforward;

  public ManipulatorIOSim() {
    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                ManipulatorConstants.ARM_PARAMETERS.MOTOR_CONFIG(),
                0.004,
                ManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO()),
            ManipulatorConstants.ARM_PARAMETERS.MOTOR_CONFIG(),
            ManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO(),
            ManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS(),
            ManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians(),
            ManipulatorConstants.ARM_PARAMETERS.MAX_ANGLE().getRadians(),
            true,
            ManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians());
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 3.0),
            DCMotor.getKrakenX60Foc(1));

    armAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;

    feedback =
        new ProfiledPIDController(
            ManipulatorConstants.WITHOUT_ALGAE_GAINS.kP().get(),
            0.0,
            ManipulatorConstants.WITHOUT_ALGAE_GAINS.kD().get(),
            new Constraints(
                ManipulatorConstants.CONSTRAINTS.cruisingVelocityRotationsPerSecond().get(),
                ManipulatorConstants.CONSTRAINTS.maxAccelerationRotationsPerSecondSquared().get()));
    feedforward =
        new ArmFeedforward(
            ManipulatorConstants.WITHOUT_ALGAE_GAINS.kS().get(),
            ManipulatorConstants.WITHOUT_ALGAE_GAINS.kV().get(),
            ManipulatorConstants.WITHOUT_ALGAE_GAINS.kA().get());
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    armAppliedVolts =
        feedback.calculate(armSim.getAngleRads())
            + feedforward.calculate(
                feedback.getSetpoint().position, feedback.getSetpoint().velocity);

    armAppliedVolts = MathUtil.clamp(armAppliedVolts, -12.0, 12.0);
    rollerAppliedVolts = MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0);

    armSim.setInputVoltage(armAppliedVolts);
    armSim.update(Constants.LOOP_PERIOD_SECONDS);
    rollerSim.setInputVoltage(rollerAppliedVolts);
    rollerSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.rollerPosition = Rotation2d.fromRadians(rollerSim.getAngularPositionRad());
    inputs.rollerVelocityRadiansPerSecond = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAccelerationRadiansPerSecondSquared =
        rollerSim.getAngularAccelerationRadPerSecSq();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps = rollerSim.getCurrentDrawAmps();

    inputs.armPosition = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.armVelocityRadiansPerSecond = armSim.getVelocityRadPerSec();
    inputs.armAppliedVolts = armAppliedVolts;
    inputs.armSupplyCurrentAmps = armSim.getCurrentDrawAmps();
    inputs.armPositionGoal = Rotation2d.fromRadians(feedback.getGoal().position);
    inputs.armPositionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.armPositionError = Rotation2d.fromRadians(feedback.getPositionError());
  }

  @Override
  public void setArmPositionGoal(Rotation2d position) {
    feedback.setGoal(position.getRadians());
  }

  @Override
  public void setRollerVoltage(double rollerAppliedVolts) {
    this.rollerAppliedVolts = rollerAppliedVolts;
  }

  @Override
  public void updateSlot0ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    feedback.setPID(kP, 0, kD);
    feedforward = new ArmFeedforward(kS, kG, kV);
  }

  @Override
  public void updateArmConstraints(
      double maxAccelerationRadiansPerSecondSquared, double cruisingVelocityRadiansPerSecond) {
    feedback.setConstraints(
        new Constraints(cruisingVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
  }
}
