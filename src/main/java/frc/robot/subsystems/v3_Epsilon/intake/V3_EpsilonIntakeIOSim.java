package frc.robot.subsystems.v3_Epsilon.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class V3_EpsilonIntakeIOSim implements V3_EpsilonIntakeIO {
  private SingleJointedArmSim pivotMotorSim;
  private DCMotorSim rollerMotorSim;

  private final ProfiledPIDController armFeedbackController;
  private final ArmFeedforward armFeedforwardController;

  private double pivotAppliedVoltage;
  private double rollerAppliedVoltage;

  private boolean isClosedLoop;

  public V3_EpsilonIntakeIOSim() {
    pivotMotorSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                V3_EpsilonIntakeConstants.PIVOT_PARAMS.MOTOR(),
                V3_EpsilonIntakeConstants.PIVOT_PARAMS.PIVOT_GEAR_RATIO(),
                V3_EpsilonIntakeConstants.PIVOT_PARAMS.PIVOT_GEAR_RATIO()),
            V3_EpsilonIntakeConstants.PIVOT_PARAMS.MOTOR(),
            V3_EpsilonIntakeConstants.PIVOT_PARAMS.PIVOT_GEAR_RATIO(),
            0.1,
            V3_EpsilonIntakeConstants.PIVOT_PARAMS.MIN_ANGLE().getRadians(),
            V3_EpsilonIntakeConstants.PIVOT_PARAMS.MAX_ANGLE().getRadians(),
            true,
            V3_EpsilonIntakeConstants.PIVOT_PARAMS.MIN_ANGLE().getRadians());

    rollerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V3_EpsilonIntakeConstants.ROLLER_PARAMS.MOTOR(),
                0.04,
                V3_EpsilonIntakeConstants.ROLLER_PARAMS.PIVOT_GEAR_RATIO()),
            V3_EpsilonIntakeConstants.ROLLER_PARAMS.MOTOR());

    armFeedbackController =
        new ProfiledPIDController(
            V3_EpsilonIntakeConstants.PIVOT_GAINS.kP(),
            0.0,
            V3_EpsilonIntakeConstants.PIVOT_GAINS.kD(),
            V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.getTrapezoidConstraints());
    armFeedbackController.disableContinuousInput();

    armFeedforwardController =
        new ArmFeedforward(
            V3_EpsilonIntakeConstants.PIVOT_GAINS.kS(),
            V3_EpsilonIntakeConstants.PIVOT_GAINS.kA(),
            V3_EpsilonIntakeConstants.PIVOT_GAINS.kV(),
            V3_EpsilonIntakeConstants.PIVOT_GAINS.kA());

    pivotAppliedVoltage = 0.0;
    rollerAppliedVoltage = 0.0;
    isClosedLoop = false;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (isClosedLoop) {
      pivotAppliedVoltage = armFeedbackController.calculate(pivotMotorSim.getAngleRads())+
          armFeedforwardController.calculate(pivotMotorSim.getAngleRads(), pivotMotorSim.getVelocityRadPerSec());
    } 

    pivotAppliedVoltage = MathUtil.clamp(pivotAppliedVoltage, -12.0, 12.0);
    rollerAppliedVoltage = MathUtil.clamp(rollerAppliedVoltage, -12.0, 12.0);

    pivotMotorSim.setInputVoltage(pivotAppliedVoltage);
    pivotMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    rollerMotorSim.setInputVoltage(rollerAppliedVoltage);
    rollerMotorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.pivotPosition = new Rotation2d(pivotMotorSim.getAngleRads());
    inputs.pivotVelocityRadiansPerSecond = pivotMotorSim.getVelocityRadPerSec();
    inputs.pivotAppliedVolts = pivotAppliedVoltage;
    inputs.pivotSupplyCurrentAmps = pivotMotorSim.getCurrentDrawAmps();

    inputs.pivotPositionGoal = new Rotation2d(armFeedbackController.getGoal().position);
    inputs.pivotPositionSetpoint = new Rotation2d(armFeedbackController.getSetpoint().position);
    inputs.pivotPositionError = new Rotation2d(armFeedbackController.getPositionError());

    inputs.rollerPosition = new Rotation2d(rollerMotorSim.getAngularPosition());
    inputs.rollerVelocityRadiansPerSecond = rollerMotorSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVoltage;
    inputs.rollerSupplyCurrentAmps = rollerMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    isClosedLoop = false;
    pivotAppliedVoltage = voltage;
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerAppliedVoltage = voltage;
  }

  @Override
  public void setPivotGoal(Rotation2d rotation) {
    isClosedLoop = true;
    armFeedbackController.setGoal(rotation.getRadians());
  }

  @Override
  public void updateIntakeGains(
      double kP,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG) {
    armFeedbackController.setPID(kP, 0.0, kD);
    armFeedforwardController.setKa(kA);
    armFeedforwardController.setKs(kS);
    armFeedforwardController.setKv(kV);
    armFeedforwardController.setKg(kG);
  }

  @Override
  public void updateIntakeConstraints(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared) {
    armFeedbackController.setConstraints(new Constraints(maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
}
}
