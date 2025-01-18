package frc.robot.subsystems.v1_gamma.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FunnelIOSim implements FunnelIO {
  public final DCMotorSim crabMotorSim;
  public final DCMotorSim intakeMotorSim;

  private final ProfiledPIDController crabController;
  private final ProfiledPIDController intakeController;
  private final SimpleMotorFeedforward crabFeedforward;
  private final SimpleMotorFeedforward intakeFeedforward;

  private double crabPositionRadians = 0.0;
  private double crabAppliedVolts = 0.0;
  private double crabPositionGoal = 0.0;

  private double intakeVelocityRadiansPerSecond = 0.0;
  private double intakeAppliedVolts = 0.0;
  private double intakeVelocityGoal = 0.0;

  public FunnelIOSim() {
    crabMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FunnelConstants.CRAB_PARAMS.motor(),
                FunnelConstants.CRAB_PARAMS.momentOfInertia(),
                FunnelConstants.CRAB_MOTOR_GEAR_RATIO),
            FunnelConstants.CRAB_PARAMS.motor());

    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FunnelConstants.INTAKE_PARAMS.motor(),
                FunnelConstants.INTAKE_PARAMS.momentOfInertia(),
                FunnelConstants.INTAKE_MOTOR_GEAR_RATIO),
            FunnelConstants.INTAKE_PARAMS.motor());

    crabController =
        new ProfiledPIDController(
            FunnelConstants.CRAB_MOTOR_GAINS.kP().get(),
            0.0,
            FunnelConstants.CRAB_MOTOR_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                FunnelConstants.CRAB_MOTOR_CONSTRAINTS.MAX_VELOCITY().get(),
                FunnelConstants.CRAB_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get()));

    intakeController =
        new ProfiledPIDController(
            FunnelConstants.INTAKE_MOTOR_GAINS.kP().get(),
            0.0,
            FunnelConstants.INTAKE_MOTOR_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                FunnelConstants.INTAKE_MOTOR_CONSTRAINTS.MAX_VELOCITY().get(),
                FunnelConstants.INTAKE_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get()));

    crabFeedforward =
        new SimpleMotorFeedforward(
            FunnelConstants.CRAB_MOTOR_GAINS.kS().get(),
            FunnelConstants.CRAB_MOTOR_GAINS.kV().get(),
            FunnelConstants.CRAB_MOTOR_GAINS.kA().get());

    intakeFeedforward =
        new SimpleMotorFeedforward(
            FunnelConstants.INTAKE_MOTOR_GAINS.kS().get(),
            FunnelConstants.INTAKE_MOTOR_GAINS.kV().get(),
            FunnelConstants.INTAKE_MOTOR_GAINS.kA().get());
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    crabMotorSim.setInputVoltage(crabAppliedVolts);
    intakeMotorSim.setInputVoltage(intakeAppliedVolts);
    crabMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    intakeMotorSim.update(Constants.LOOP_PERIOD_SECONDS);
    
    inputs.crabPositionRadians = crabMotorSim.getAngularPositionRad();
    inputs.crabVelocityRadiansPerSecond = crabMotorSim.getAngularVelocityRadPerSec();
    inputs.crabGoalRadians = crabPositionGoal;
    inputs.crabAppliedVolts = crabAppliedVolts;
    inputs.crabGoalRadians = inputs.crabSupplyCurrentAmps = crabMotorSim.getCurrentDrawAmps();
    inputs.crabTorqueCurrentAmps = crabMotorSim.getCurrentDrawAmps();
    inputs.crabPositionSetpointRadians = crabController.getSetpoint().position;
    inputs.crabPositionErrorRadians = crabController.getPositionError();

    inputs.intakePositionRadians = intakeMotorSim.getAngularPositionRad();
    inputs.intakeVelocityRadiansPerSecond = intakeMotorSim.getAngularVelocityRadPerSec();
    inputs.intakeGoalRadiansPerSecond = intakeVelocityGoal;
    inputs.intakeAppliedVolts = intakeAppliedVolts;
    inputs.intakeSupplyCurrentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.intakeTorqueCurrentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.intakeVelocitySetpointRadiansPerSecond = intakeController.getSetpoint().velocity;
    inputs.intakeVelocityErrorRadiansPerSecond = intakeController.getVelocityError();
  }

    @Override
    public void setCrabVoltage(double volts) {
        crabAppliedVolts = volts;
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeAppliedVolts = volts;
    }

    @Override
    public void setCrabPosition(double radians) {
        crabPositionGoal = radians;
        crabAppliedVolts = MathUtil.clamp(crabController.calculate(radians) + crabFeedforward.calculate(radians), -12, 12);
    }

    @Override
    public void setIntakeVelocity(double radiansPerSecond) {
        intakeVelocityGoal = radiansPerSecond;
        intakeAppliedVolts = MathUtil.clamp(intakeController.calculate(radiansPerSecond) + intakeFeedforward.calculate(radiansPerSecond), -12, 12);
    }

    @Override
    public void stopIntake() {
        intakeVelocityGoal = 0.0;
        intakeAppliedVolts = 0.0;
    }

}

