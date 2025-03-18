package frc.robot.subsystems.v2_Redundancy.manipulator;

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

public class V2_RedundancyManipulatorIOSim implements V2_RedundancyManipulatorIO {
  private final SingleJointedArmSim armSim;
  private final DCMotorSim rollerSim;

  private double armAppliedVolts;
  private double rollerAppliedVolts;

  private final ProfiledPIDController feedback;
  private ArmFeedforward feedforward;

  private boolean isClosedLoop;

  public V2_RedundancyManipulatorIOSim() {
    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MOTOR_CONFIG(),
                0.004,
                V2_RedundancyManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO()),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MOTOR_CONFIG(),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO(),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS(),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians(),
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MAX_ANGLE().getRadians(),
            true,
            V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians());
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 3.0),
            DCMotor.getKrakenX60Foc(1));

    armAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;

    feedback =
        new ProfiledPIDController(
            V2_RedundancyManipulatorConstants.GAINS.kP().get(),
            0.0,
            V2_RedundancyManipulatorConstants.GAINS.kD().get(),
            new Constraints(
                V2_RedundancyManipulatorConstants.CONSTRAINTS
                    .cruisingVelocityRadiansPerSecond()
                    .get(),
                V2_RedundancyManipulatorConstants.CONSTRAINTS
                    .maxAccelerationRadiansPerSecondSquared()
                    .get()));
    feedforward =
        new ArmFeedforward(
            V2_RedundancyManipulatorConstants.GAINS.kS().get(),
            V2_RedundancyManipulatorConstants.GAINS.kV().get(),
            V2_RedundancyManipulatorConstants.GAINS.kA().get());

    isClosedLoop = false;

    armSim.setState(V2_RedundancyManipulatorConstants.ARM_PARAMETERS.MIN_ANGLE().getRadians(), 0.0);
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    if (isClosedLoop) {
      armAppliedVolts =
          feedback.calculate(armSim.getAngleRads())
              + feedforward.calculate(feedback.getSetpoint().position, feedback.getSetpoint().velocity);
    }

    armAppliedVolts = MathUtil.clamp(armAppliedVolts, -12.0, 12.0);
    rollerAppliedVolts = MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0);

    armSim.setInputVoltage(armAppliedVolts);
    armSim.update(Constants.LOOP_PERIOD_SECONDS);
    rollerSim.setInputVoltage(rollerAppliedVolts);
    rollerSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.rollerPosition = Rotation2d.fromRadians(rollerSim.getAngularPositionRad());
    inputs.rollerVelocityRadiansPerSecond = rollerSim.getAngularVelocityRadPerSec();
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
    isClosedLoop = true;
    feedback.setGoal(position.getRadians());
  }

  @Override
  public void setRollerVoltage(double rollerAppliedVolts) {
    this.rollerAppliedVolts = rollerAppliedVolts;
  }

  @Override
  public void updateArmGains(double kP, double kD, double kS, double kV, double kA, double kG) {
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
