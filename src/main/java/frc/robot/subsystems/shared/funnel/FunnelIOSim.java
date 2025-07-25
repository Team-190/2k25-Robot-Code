package frc.robot.subsystems.shared.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class FunnelIOSim implements FunnelIO {
  public final SingleJointedArmSim clapDaddySim;
  public final DCMotorSim rollerSim;

  private final ProfiledPIDController clapDaddyController;
  private SimpleMotorFeedforward clapDaddyFeedforward;

  private double clapDaddyAppliedVolts;
  private double rollerAppliedVolts;
  private boolean clapDaddyClosedLoop;

  public FunnelIOSim() {
    clapDaddySim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                FunnelConstants.CLAP_DADDY_PARAMS.MOTOR(),
                FunnelConstants.CLAP_DADDY_PARAMS.MOMENT_OF_INERTIA(),
                FunnelConstants.CLAP_DADDY_MOTOR_GEAR_RATIO),
            FunnelConstants.CLAP_DADDY_PARAMS.MOTOR(),
            FunnelConstants.CLAP_DADDY_MOTOR_GEAR_RATIO,
            1.0,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            false,
            FunnelConstants.FunnelState.OPENED.getAngle().getRadians());
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FunnelConstants.ROLLER_PARAMS.MOTOR(),
                FunnelConstants.ROLLER_PARAMS.MOMENT_OF_INERTIA(),
                FunnelConstants.ROLLER_MOTOR_GEAR_RATIO),
            FunnelConstants.ROLLER_PARAMS.MOTOR());

    clapDaddyController =
        new ProfiledPIDController(
            FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kP().get(),
            0.0,
            FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                FunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_VELOCITY().get(),
                FunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get()));
    clapDaddyFeedforward =
        new SimpleMotorFeedforward(
            FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kS().get(),
            FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kV().get(),
            FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kA().get());

    clapDaddyAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
    clapDaddyClosedLoop = true;

    clapDaddyController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    if (clapDaddyClosedLoop) {
      clapDaddyAppliedVolts =
          clapDaddyController.calculate(clapDaddySim.getAngleRads())
              + clapDaddyFeedforward.calculate(clapDaddyController.getSetpoint().position);
    }

    clapDaddyAppliedVolts = MathUtil.clamp(clapDaddyAppliedVolts, -12.0, 12.0);
    rollerAppliedVolts = MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0);

    clapDaddySim.setInputVoltage(clapDaddyAppliedVolts);
    rollerSim.setInputVoltage(rollerAppliedVolts);
    clapDaddySim.update(Constants.LOOP_PERIOD_SECONDS);
    rollerSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.clapDaddyAbsolutePosition = Rotation2d.fromRadians(clapDaddySim.getAngleRads());
    inputs.clapDaddyPosition = Rotation2d.fromRadians(clapDaddySim.getAngleRads());
    inputs.clapDaddyVelocityRadiansPerSecond = clapDaddySim.getVelocityRadPerSec();
    inputs.clapDaddyAppliedVolts = clapDaddyAppliedVolts;
    inputs.clapDaddySupplyCurrentAmps = clapDaddySim.getCurrentDrawAmps();
    inputs.clapDaddyGoal = Rotation2d.fromRadians(clapDaddyController.getGoal().position);
    inputs.clapDaddyPositionSetpoint =
        Rotation2d.fromRadians(clapDaddyController.getSetpoint().position);
    inputs.clapDaddyPositionError = Rotation2d.fromRadians(clapDaddyController.getPositionError());

    inputs.rollerPosition = Rotation2d.fromRadians(rollerSim.getAngularPositionRad());
    inputs.rollerVelocityRadiansPerSecond = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps = rollerSim.getCurrentDrawAmps();
  }

  @Override
  public void setClapDaddyVoltage(double volts) {
    clapDaddyClosedLoop = false;
    clapDaddyAppliedVolts = volts;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }

  @Override
  public void setClapDaddyGoal(Rotation2d position) {
    clapDaddyClosedLoop = true;
    clapDaddyController.setGoal(position.getRadians());
  }

  @Override
  public void stopRoller() {
    rollerAppliedVolts = 0.0;
  }

  @Override
  public boolean atClapDaddyPositionGoal() {
    return clapDaddyController.atGoal();
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    clapDaddyController.setP(kP);
    clapDaddyController.setD(kD);
    clapDaddyFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    clapDaddyController.setConstraints(
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  }
}
