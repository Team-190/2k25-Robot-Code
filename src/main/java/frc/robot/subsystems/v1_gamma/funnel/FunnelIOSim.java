package frc.robot.subsystems.v1_gamma.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FunnelIOSim implements FunnelIO {
  public final DCMotorSim clapperMotorSim;
  public final DCMotorSim rollerMotorSim;

  private final ProfiledPIDController clapperController;
  private final ProfiledPIDController rollerController;
  private final SimpleMotorFeedforward clapperFeedforward;
  private final SimpleMotorFeedforward rollerFeedforward;

  private double clapperAppliedVolts = 0.0;
  private double clapperPositionGoal = 0.0;

  private double rollerAppliedVolts = 0.0;
  private double rollerVelocityGoal = 0.0;

  public FunnelIOSim() {
    clapperMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FunnelConstants.CLAPPER_PARAMS.motor(),
                FunnelConstants.CLAPPER_PARAMS.momentOfInertia(),
                FunnelConstants.CLAPPER_MOTOR_GEAR_RATIO),
            FunnelConstants.CLAPPER_PARAMS.motor());

    rollerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FunnelConstants.ROLLER_PARAMS.motor(),
                FunnelConstants.ROLLER_PARAMS.momentOfInertia(),
                FunnelConstants.ROLLER_MOTOR_GEAR_RATIO),
            FunnelConstants.ROLLER_PARAMS.motor());

    clapperController =
        new ProfiledPIDController(
            FunnelConstants.CLAPPER_MOTOR_GAINS.kP().get(),
            0.0,
            FunnelConstants.CLAPPER_MOTOR_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                FunnelConstants.CLAPPER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get(),
                FunnelConstants.CLAPPER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get()));

    rollerController =
        new ProfiledPIDController(
            FunnelConstants.ROLLER_MOTOR_GAINS.kP().get(),
            0.0,
            FunnelConstants.ROLLER_MOTOR_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                FunnelConstants.ROLLER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get(),
                FunnelConstants.ROLLER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get()));

    clapperFeedforward =
        new SimpleMotorFeedforward(
            FunnelConstants.CLAPPER_MOTOR_GAINS.kS().get(),
            FunnelConstants.CLAPPER_MOTOR_GAINS.kV().get(),
            FunnelConstants.CLAPPER_MOTOR_GAINS.kA().get());

    rollerFeedforward =
        new SimpleMotorFeedforward(
            FunnelConstants.ROLLER_MOTOR_GAINS.kS().get(),
            FunnelConstants.ROLLER_MOTOR_GAINS.kV().get(),
            FunnelConstants.ROLLER_MOTOR_GAINS.kA().get());
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    clapperMotorSim.setInputVoltage(clapperAppliedVolts);
    rollerMotorSim.setInputVoltage(rollerAppliedVolts);
    clapperMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    rollerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.clapperPositionRadians = clapperMotorSim.getAngularPositionRad();
    inputs.clapperVelocityRadiansPerSecond = clapperMotorSim.getAngularVelocityRadPerSec();
    inputs.clapperGoalRadians = clapperPositionGoal;
    inputs.clapperAppliedVolts = clapperAppliedVolts;
    inputs.clapperGoalRadians =
        inputs.clapperSupplyCurrentAmps = clapperMotorSim.getCurrentDrawAmps();
    inputs.clapperTorqueCurrentAmps = clapperMotorSim.getCurrentDrawAmps();
    inputs.clapperPositionSetpointRadians = clapperController.getSetpoint().position;
    inputs.clapperPositionErrorRadians = clapperController.getPositionError();

    inputs.rollerPositionRadians = rollerMotorSim.getAngularPositionRad();
    inputs.rollerVelocityRadiansPerSecond = rollerMotorSim.getAngularVelocityRadPerSec();
    inputs.rollerGoalRadiansPerSecond = rollerVelocityGoal;
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps = rollerMotorSim.getCurrentDrawAmps();
    inputs.rollerTorqueCurrentAmps = rollerMotorSim.getCurrentDrawAmps();
    inputs.rollerVelocitySetpointRadiansPerSecond = rollerController.getSetpoint().velocity;
    inputs.rollerVelocityErrorRadiansPerSecond = rollerController.getVelocityError();
  }

  @Override
  public void setClapperVoltage(double volts) {
    clapperAppliedVolts = volts;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }

  @Override
  public void setClapperPosition(double radians) {
    clapperPositionGoal = radians;
    clapperAppliedVolts =
        MathUtil.clamp(
            clapperController.calculate(radians) + clapperFeedforward.calculate(radians), -12, 12);
  }

  @Override
  public void setRollerVelocity(double radiansPerSecond) {
    rollerVelocityGoal = radiansPerSecond;
    rollerAppliedVolts =
        MathUtil.clamp(
            rollerController.calculate(radiansPerSecond)
                + rollerFeedforward.calculate(rollerController.getSetpoint().position),
            -12,
            12);
  }

  @Override
  public void stopRoller() {
    rollerVelocityGoal = 0.0;
    rollerAppliedVolts = 0.0;
  }
}
