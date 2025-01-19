package frc.robot.subsystems.v1_gamma.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FunnelIOSim implements FunnelIO {
  public final DCMotorSim serializerMotorSim;
  public final DCMotorSim rollerMotorSim;

  private final ProfiledPIDController serializerController;
  private final SimpleMotorFeedforward serializerFeedforward;

  private double serializerAppliedVolts = 0.0;
  private double serializerPositionGoal = 0.0;

  private double rollerAppliedVolts = 0.0;

  public FunnelIOSim() {
    serializerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FunnelConstants.SERIALIZER_PARAMS.motor(),
                FunnelConstants.SERIALIZER_PARAMS.momentOfInertia(),
                FunnelConstants.SERIALIZER_MOTOR_GEAR_RATIO),
            FunnelConstants.SERIALIZER_PARAMS.motor());

    rollerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FunnelConstants.ROLLER_PARAMS.motor(),
                FunnelConstants.ROLLER_PARAMS.momentOfInertia(),
                FunnelConstants.ROLLER_MOTOR_GEAR_RATIO),
            FunnelConstants.ROLLER_PARAMS.motor());

    serializerController =
        new ProfiledPIDController(
            FunnelConstants.SERIALIZER_MOTOR_GAINS.kP().get(),
            0.0,
            FunnelConstants.SERIALIZER_MOTOR_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                FunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get(),
                FunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get()));

    serializerFeedforward =
        new SimpleMotorFeedforward(
            FunnelConstants.SERIALIZER_MOTOR_GAINS.kS().get(),
            FunnelConstants.SERIALIZER_MOTOR_GAINS.kV().get(),
            FunnelConstants.SERIALIZER_MOTOR_GAINS.kA().get());
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
    inputs.serializerTorqueCurrentAmps = serializerMotorSim.getCurrentDrawAmps();
    inputs.serializerGoal = Rotation2d.fromRadians(serializerPositionGoal);
    inputs.serializerPositionSetpoint = Rotation2d.fromRadians(serializerController.getSetpoint().position);
    inputs.serializerPositionError = Rotation2d.fromRadians(serializerController.getPositionError());

    inputs.rollerPosition = Rotation2d.fromRadians(rollerMotorSim.getAngularPositionRad());
    inputs.rollerVelocityRadiansPerSecond = rollerMotorSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps = rollerMotorSim.getCurrentDrawAmps();
    inputs.rollerTorqueCurrentAmps = rollerMotorSim.getCurrentDrawAmps();
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
  public void setSerializerPosition(double radians) {
    serializerPositionGoal = radians;
    serializerAppliedVolts =
        MathUtil.clamp(
            serializerController.calculate(radians) + serializerFeedforward.calculate(radians),
            -12,
            12);
  }

  @Override
  public void stopRoller() {
    rollerAppliedVolts = 0.0;
  }
}
