package frc.robot.subsystems.v2_Redundancy.superstructure.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class V2_RedundancyFunnelIOSim implements V2_RedundancyFunnelIO {
  public final SingleJointedArmSim clapDaddySim;
  public final DCMotorSim rollerSim;

  private final ProfiledPIDController clapDaddyController;
  private SimpleMotorFeedforward clapDaddyFeedforward;

  private double clapDaddyAppliedVolts;
  private double rollerAppliedVolts;
  private boolean clapDaddyClosedLoop;

  public V2_RedundancyFunnelIOSim() {
    clapDaddySim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                V2_RedundancyFunnelConstants.CLAP_DADDY_PARAMS.motor(),
                V2_RedundancyFunnelConstants.CLAP_DADDY_PARAMS.momentOfInertia(),
                V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GEAR_RATIO),
            V2_RedundancyFunnelConstants.CLAP_DADDY_PARAMS.motor(),
            V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GEAR_RATIO,
            1.0,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            false,
            V2_RedundancyFunnelConstants.FunnelState.OPENED.getAngle().getRadians());
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V2_RedundancyFunnelConstants.ROLLER_PARAMS.motor(),
                V2_RedundancyFunnelConstants.ROLLER_PARAMS.momentOfInertia(),
                V2_RedundancyFunnelConstants.ROLLER_MOTOR_GEAR_RATIO),
            V2_RedundancyFunnelConstants.ROLLER_PARAMS.motor());

    clapDaddyController =
        new ProfiledPIDController(
            V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kP().get(),
            0.0,
            V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_VELOCITY().get(),
                V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get()));
    clapDaddyFeedforward =
        new SimpleMotorFeedforward(
            V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kS().get(),
            V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kV().get(),
            V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kA().get());

    clapDaddyAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
    clapDaddyClosedLoop = true;

    clapDaddyController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(V2_RedundancyFunnelIOInputs inputs) {
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
