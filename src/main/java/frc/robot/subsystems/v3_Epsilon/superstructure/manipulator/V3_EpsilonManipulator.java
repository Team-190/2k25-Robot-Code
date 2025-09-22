package frc.robot.subsystems.v3_Epsilon.superstructure.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorArmState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorRollerState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.Side;
import java.util.Set;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonManipulator {
  private final V3_EpsilonManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;

  @AutoLogOutput(key = "Manipulator/Arm Goal")
  @Getter
  private ManipulatorArmState armGoal;

  @Setter
  @Getter
  @AutoLogOutput(key = "Manipulator/Arm Side")
  private Side armSide;

  @AutoLogOutput(key = "Manipulator/Roller Goal")
  @Getter
  private ManipulatorRollerState rollerGoal;

  private boolean isClosedLoop;

  @Setter @Getter private boolean clearsElevator;

  public V3_EpsilonManipulator(V3_EpsilonManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();

    isClosedLoop = true;
    armGoal = ManipulatorArmState.VERTICAL_UP;
    armSide = Side.POSITIVE;
    rollerGoal = ManipulatorRollerState.STOP;

    clearsElevator = false;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);

    if (isClosedLoop) {
      Rotation2d goal = armGoal.getAngle(armSide);

      if (!isSafePosition() || clearsElevator) {
        if (armSide == Side.POSITIVE) {
          goal = Rotation2d.fromRotations(goal.getRotations() - 1.0);
        } else {
          goal = Rotation2d.fromRotations(goal.getRotations() + 1.0);
        }
      }

      io.setArmGoal(goal);
    }

    if (hasAlgae()
        && Set.of(ManipulatorRollerState.CORAL_INTAKE, ManipulatorRollerState.STOP)
            .contains(rollerGoal)) {
      io.setRollerVoltage(holdVoltage());
    } else {
      io.setRollerVoltage(rollerGoal.getVoltage());
    }
  }

  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return inputs.canRangeDistanceMeters < V3_EpsilonManipulatorConstants.CORAL_CAN_RANGE_THRESHOLD
        && inputs.canRangeDistanceMeters > 0;
  }

  @AutoLogOutput(key = "Manipulator/Has Algae")
  public boolean hasAlgae() {
    return inputs.canRangeDistanceMeters < V3_EpsilonManipulatorConstants.ALGAE_CAN_RANGE_THRESHOLD
        && inputs.canRangeDistanceMeters > 0;
  }

  public Command runArm(double volts) {
    return Commands.runEnd(
        () -> {
          isClosedLoop = false;
          io.setArmVoltage(volts);
        },
        () -> io.setArmVoltage(0));
  }

  public void setArmGoal(ManipulatorArmState goal) {
    isClosedLoop = true;
    armGoal = goal;
  }

  public void updateArmGains(
      double kP0,
      double kD0,
      double kS0,
      double kV0,
      double kA0,
      double kG0,
      double kP1,
      double kD1,
      double kS1,
      double kV1,
      double kA1,
      double kG1,
      double kP2,
      double kD2,
      double kS2,
      double kV2,
      double kA2,
      double kG2) {
    io.updateSlot0ArmGains(kP0, kD0, kS0, kV0, kA0, kG0);
    io.updateSlot1ArmGains(kP1, kD1, kS1, kV1, kA1, kG1);
    io.updateSlot2ArmGains(kP2, kD2, kS2, kV2, kA2, kG2);
  }

  public void updateArmConstraints(double maxAcceleration, double maxVelocity) {
    io.updateArmConstraints(maxAcceleration, maxVelocity);
  }

  @AutoLogOutput(key = "Manipulator/Arm At Goal")
  public boolean armAtGoal() {
    return armAtGoal(armGoal);
  }

  public boolean armAtGoal(ManipulatorArmState state) {
    return Math.abs(inputs.armPosition.minus(state.getAngle(armSide)).getRadians())
        <= V3_EpsilonManipulatorConstants.CONSTRAINTS.goalToleranceRadians().get();
  }

  public Command waitUntilArmAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::armAtGoal));
  }

  private double holdVoltage() {
    double y;
    double x = Math.abs(inputs.rollerTorqueCurrentAmps);
    if (x <= 20) {
      y = -0.0003 * Math.pow(x, 3) + 0.0124286 * Math.pow(x, 2) - 0.241071 * x + 4.00643;
    } else {
      y = 0.0005 * Math.pow(x, 2) - 0.1015 * x + 3.7425;
    }
    return MathUtil.clamp(
        1.25 * y,
        0.10,
        V3_EpsilonManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().getAsDouble() / 1.5);
  }

  public void setSlot() {
    if (hasAlgae()) {
      io.setSlot(2);
    } else if (hasCoral()) {
      io.setSlot(1);
    } else {
      io.setSlot(0);
    }
  }

  public void setManipulatorState(V3_EpsilonManipulatorConstants.ManipulatorArmState state) {
    io.setManipulatorState(state);
  }

  public void setRollerGoal(V3_EpsilonManipulatorConstants.ManipulatorRollerState rollerGoal) {
    this.rollerGoal = rollerGoal;
    if (hasAlgae()
        && Set.of(
                V3_EpsilonManipulatorConstants.ManipulatorRollerState.ALGAE_INTAKE,
                V3_EpsilonManipulatorConstants.ManipulatorRollerState.CORAL_INTAKE,
                V3_EpsilonManipulatorConstants.ManipulatorRollerState.STOP)
            .contains(rollerGoal)) {

      io.setRollerVoltage(holdVoltage());
    } else {
      io.setRollerVoltage(rollerGoal.getVoltage());
    }
  }

  public Rotation2d getArmAngle() {
    return inputs.armPosition;
  }

  @AutoLogOutput(key = "Manipulator/Safe Position")
  public boolean isSafePosition() {
    double cosThresh =
        Math.cos(Math.PI - ManipulatorArmState.SAFE_ANGLE.getAngle(armSide).getRadians());
    // unsafe if -cos(theta) >= cosThresh
    return (-inputs.armPosition.getCos()) < cosThresh;
  }

  public double getArmVelocity() {
    return inputs.armVelocityRadiansPerSecond;
  }

  public double getRollerVelocity() {
    return inputs.rollerVelocityRadiansPerSecond;
  }
}
