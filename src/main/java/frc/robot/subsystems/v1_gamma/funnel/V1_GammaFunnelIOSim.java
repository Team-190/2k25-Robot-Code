package frc.robot.subsystems.v1_gamma.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class V1_GammaFunnelIOSim implements V1_GammaFunnelIO {
  public final SingleJointedArmSim serializerSim;
  public final DCMotorSim rollerSim;

  private final ProfiledPIDController serializerController;
  private SimpleMotorFeedforward serializerFeedforward;

  private double serializerAppliedVolts;
  private double rollerAppliedVolts;
  private boolean serializerClosedLoop;

  public V1_GammaFunnelIOSim() {
    serializerSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                V1_GammaFunnelConstants.SERIALIZER_PARAMS.motor(),
                V1_GammaFunnelConstants.SERIALIZER_PARAMS.momentOfInertia(),
                V1_GammaFunnelConstants.SERIALIZER_MOTOR_GEAR_RATIO),
            V1_GammaFunnelConstants.SERIALIZER_PARAMS.motor(),
            V1_GammaFunnelConstants.SERIALIZER_MOTOR_GEAR_RATIO,
            1.0,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            false,
            V1_GammaFunnelConstants.FunnelState.OPENED.getAngle().getRadians());
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_GammaFunnelConstants.ROLLER_PARAMS.motor(),
                V1_GammaFunnelConstants.ROLLER_PARAMS.momentOfInertia(),
                V1_GammaFunnelConstants.ROLLER_MOTOR_GEAR_RATIO),
            V1_GammaFunnelConstants.ROLLER_PARAMS.motor());

    serializerController =
        new ProfiledPIDController(
            V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kP().get(),
            0.0,
            V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                V1_GammaFunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get(),
                V1_GammaFunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get()));
    serializerFeedforward =
        new SimpleMotorFeedforward(
            V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kS().get(),
            V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kV().get(),
            V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kA().get());

    serializerAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
    serializerClosedLoop = true;

    serializerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    if (serializerClosedLoop) {
      serializerAppliedVolts =
          serializerController.calculate(serializerSim.getAngleRads())
              + serializerFeedforward.calculate(serializerController.getSetpoint().position);
    }

    serializerAppliedVolts = MathUtil.clamp(serializerAppliedVolts, -12.0, 12.0);

    serializerSim.setInputVoltage(serializerAppliedVolts);
    rollerSim.setInputVoltage(rollerAppliedVolts);
    serializerSim.update(Constants.LOOP_PERIOD_SECONDS);
    rollerSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.serializerAbsolutePosition = Rotation2d.fromRadians(serializerSim.getAngleRads());
    inputs.serializerPosition = Rotation2d.fromRadians(serializerSim.getAngleRads());
    inputs.serializerVelocityRadiansPerSecond = serializerSim.getVelocityRadPerSec();
    inputs.serializerAppliedVolts = serializerAppliedVolts;
    inputs.serializerSupplyCurrentAmps = serializerSim.getCurrentDrawAmps();
    inputs.serializerGoal = Rotation2d.fromRadians(serializerController.getGoal().position);
    inputs.serializerPositionSetpoint =
        Rotation2d.fromRadians(serializerController.getSetpoint().position);
    inputs.serializerPositionError =
        Rotation2d.fromRadians(serializerController.getPositionError());

    inputs.rollerPosition = Rotation2d.fromRadians(rollerSim.getAngularPositionRad());
    inputs.rollerVelocityRadiansPerSecond = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps = rollerSim.getCurrentDrawAmps();
  }

  @Override
  public void setSerializerVoltage(double volts) {
    serializerClosedLoop = false;
    serializerAppliedVolts = volts;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }

  @Override
  public void setSerializerPosition(Rotation2d position) {
    serializerClosedLoop = true;
    serializerController.setGoal(position.getRadians());
  }

  @Override
  public void stopRoller() {
    rollerAppliedVolts = 0.0;
  }

  @Override
  public boolean atSerializerGoal() {
    return serializerController.atGoal();
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    serializerController.setP(kP);
    serializerController.setD(kD);
    serializerFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    serializerController.setConstraints(
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  }
}
