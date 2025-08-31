package frc.robot.subsystems.v3_Epsilon.superstructure.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.manipulator.ManipulatorIOInputsAutoLogged;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructure;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorRollerStates;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.PivotState;
import lombok.Getter;

import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonManipulator {
  private final V3_EpsilonManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;

  @Getter
  @AutoLogOutput(key = "Manipulator/Pivot State")
  private PivotState pivotGoal;
  @Getter
  @AutoLogOutput(key = "Manipulator/Roller State")
  private ManipulatorRollerStates rollerGoal;
  private boolean isClosedLoop;

  public V3_EpsilonManipulator(V3_EpsilonManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();

    isClosedLoop = true;
    pivotGoal = PivotState.STOW_DOWN;
    rollerGoal = ManipulatorRollerStates.STOP;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);

    if (isClosedLoop) {
      setSlot();
      io.setPivotGoal(pivotGoal.getAngle());
    }

    if (hasAlgae() && Set.of(
        ManipulatorRollerStates.CORAL_INTAKE,
        ManipulatorRollerStates.STOP)
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

  public Command runPivot(double volts) {
    return Commands.runEnd(
        () -> {
          isClosedLoop = false;
          io.setPivotVoltage(volts);
        },
        () -> io.setPivotVoltage(0));
  }

  public Command sysIdRoutine(V3_EpsilonSuperstructure superstructure) {
    SysIdRoutine algaeCharacterizationRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.2).per(Second),
            Volts.of(3.5),
            Seconds.of(5),
            (state) -> Logger.recordOutput("Manipulator/SysID State", state.toString())),
        new SysIdRoutine.Mechanism((volts) -> io.setPivotVoltage(volts.in(Volts)), null, superstructure));

    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        algaeCharacterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(.25),
        algaeCharacterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(.25),
        algaeCharacterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(.25),
        algaeCharacterizationRoutine.dynamic(Direction.kReverse));
  }

  public Command setPivotGoal(PivotState goal) {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          pivotGoal = goal;
        });
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
  public boolean pivotAtGoal() {
    return inputs.armPosition.getRadians()
        - pivotGoal.getAngle().getRadians() <= V2_RedundancyManipulatorConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS()
            .get();
  }

  public Command waitUntilPivotAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::pivotAtGoal));
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

  public void setManipulatorState(V3_EpsilonManipulatorConstants.PivotState state) {
    io.setManipulatorState(state);
  }

  public void setRollerGoal(V3_EpsilonManipulatorConstants.ManipulatorRollerStates rollerGoal) {
    if (hasAlgae()
        && Set.of(
            V3_EpsilonManipulatorConstants.ManipulatorRollerStates.ALGAE_INTAKE,
            V3_EpsilonManipulatorConstants.ManipulatorRollerStates.CORAL_INTAKE,
            V3_EpsilonManipulatorConstants.ManipulatorRollerStates.STOP)
            .contains(rollerGoal)) {

      io.setRollerVoltage(holdVoltage());
    } else {
      io.setRollerVoltage(rollerGoal.getVoltage());
    }
  }

  public Rotation2d getArmAngle() {
    return inputs.armPosition;
  }
}
