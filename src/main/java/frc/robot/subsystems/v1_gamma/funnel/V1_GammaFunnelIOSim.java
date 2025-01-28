package frc.robot.subsystems.v1_gamma.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class V1_GammaFunnelIOSim implements V1_GammaFunnelIO {
  public final DCMotorSim serializerMotorSim;
  public final DCMotorSim rollerMotorSim;

  private final ProfiledPIDController serializerController;
  private SimpleMotorFeedforward serializerFeedforward;

  private Rotation2d serializerPositionGoal = new Rotation2d();
  private double serializerAppliedVolts = 0.0;

  private double rollerAppliedVolts = 0.0;

  public V1_GammaFunnelIOSim() {
    serializerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_GammaFunnelConstants.SERIALIZER_PARAMS.motor(),
                V1_GammaFunnelConstants.SERIALIZER_PARAMS.momentOfInertia(),
                V1_GammaFunnelConstants.SERIALIZER_MOTOR_GEAR_RATIO),
            V1_GammaFunnelConstants.SERIALIZER_PARAMS.motor());

    rollerMotorSim =
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
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    serializerMotorSim.setInputVoltage(serializerAppliedVolts);
    rollerMotorSim.setInputVoltage(rollerAppliedVolts);
    serializerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    rollerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.serializerPosition =
        Rotation2d.fromRotations(serializerMotorSim.getAngularPositionRotations());
    inputs.serializerVelocityRadiansPerSecond = serializerMotorSim.getAngularVelocityRadPerSec();
    inputs.serializerAppliedVolts = serializerAppliedVolts;
    inputs.serializerSupplyCurrentAmps = serializerMotorSim.getCurrentDrawAmps();
    inputs.serializerGoal = (serializerPositionGoal);
    inputs.serializerPositionSetpoint =
        Rotation2d.fromRadians(serializerController.getSetpoint().position);
    inputs.serializerPositionError =
        Rotation2d.fromRadians(serializerController.getPositionError());

    inputs.rollerPosition = Rotation2d.fromRadians(rollerMotorSim.getAngularPositionRad());
    inputs.rollerVelocityRadiansPerSecond = rollerMotorSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps = rollerMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setSerializerVoltage(double volts) {
    serializerAppliedVolts = volts;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }

  @Override
  public void setSerializerPosition(Rotation2d position) {
    serializerPositionGoal = position;
    serializerAppliedVolts =
        MathUtil.clamp(
            serializerController.calculate(position.getRadians())
                + serializerFeedforward.calculate(position.getRadians()),
            -12,
            12);
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
  public void updateThresholds(double maxAngle, double minAngle) {
    serializerController.enableContinuousInput(minAngle, maxAngle);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    serializerController.setConstraints(
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  }
}
