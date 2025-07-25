package frc.robot.subsystems.v2_Redundancy.superstructure.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotStateLL;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorArmState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyManipulator {
  private final V2_RedundancyManipulatorIO io;
  private final V2_RedundancyManipulatorIOInputsAutoLogged inputs;
  private boolean isClosedLoop;

  @Getter
  @AutoLogOutput(key = "Manipulator/Arm Goal")
  private ManipulatorArmState armGoal;

  @Getter
  @AutoLogOutput(key = "Manipulator/Roller Goal")
  private ManipulatorRollerState rollerGoal;

  public V2_RedundancyManipulator(V2_RedundancyManipulatorIO io) {
    this.io = io;
    inputs = new V2_RedundancyManipulatorIOInputsAutoLogged();
    isClosedLoop = true;

    armGoal = ManipulatorArmState.STOW_DOWN;
    rollerGoal = ManipulatorRollerState.STOP;
  }

  public void periodic() {
    ExternalLoggedTracer.reset();
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Manipulator/Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Manipulator", inputs);
    InternalLoggedTracer.record("Process Inputs", "Manipulator/Periodic");

    InternalLoggedTracer.reset();
    if (isClosedLoop) io.setArmPositionGoal(armGoal.getAngle());

    if (RobotStateLL.isHasAlgae()
        && Set.of(
                ManipulatorRollerState.STOP,
                ManipulatorRollerState.ALGAE_INTAKE,
                ManipulatorRollerState.CORAL_INTAKE)
            .contains(rollerGoal)) {
      io.setRollerVoltage(holdVoltage());
    } else {
      io.setRollerVoltage(rollerGoal.getVoltage());
    }

    if (hasAlgae() && RobotStateLL.isIntakingAlgae()) {
      RobotStateLL.setHasAlgae(true);
    }
    InternalLoggedTracer.record("Manipulator Logic", "Manipulator/Periodic");
    ExternalLoggedTracer.record("Manipulator Total", "Manipulator/Periodic");
  }

  public Rotation2d getArmAngle() {
    return inputs.armPosition;
  }

  public void setRollerGoal(ManipulatorRollerState goal) {
    rollerGoal = goal;
    if (RobotStateLL.isHasAlgae()
        && Set.of(
                ManipulatorRollerState.STOP,
                ManipulatorRollerState.ALGAE_INTAKE,
                ManipulatorRollerState.CORAL_INTAKE)
            .contains(rollerGoal)) {
      io.setRollerVoltage(holdVoltage());
    } else {
      io.setRollerVoltage(rollerGoal.getVoltage());
    }
  }

  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return Math.abs(inputs.rollerTorqueCurrentAmps)
        > V2_RedundancyManipulatorConstants.ROLLER_CURRENT_THRESHOLD;
  }

  @AutoLogOutput(key = "Manipulator/Has Algae")
  public boolean hasAlgae() {
    return RobotStateLL.isIntakingAlgae()
        && Math.abs(inputs.rollerAccelerationRadiansPerSecondSquared) < 1000
        && Math.abs(inputs.rollerVelocityRadiansPerSecond) <= 70;
  }

  @AutoLogOutput(key = "Manipulator/Intaking Algae")
  public boolean isIntakingAlgae() {
    return Math.abs(inputs.rollerVelocityRadiansPerSecond) >= 100.0;
  }

  public Command sysIdRoutine(V2_RedundancySuperstructure superstructure) {
    SysIdRoutine algaeCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(5),
                (state) -> Logger.recordOutput("Manipulator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setArmVoltage(volts.in(Volts)), null, superstructure));
    return Commands.sequence(
        superstructure.runGoal(V2_RedundancySuperstructureStates.OVERRIDE),
        Commands.runOnce(() -> isClosedLoop = false),
        algaeCharacterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(.25),
        algaeCharacterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(.25),
        algaeCharacterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(.25),
        algaeCharacterizationRoutine.dynamic(Direction.kReverse));
  }

  public void setAlgaeArmGoal(ManipulatorArmState goal) {
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
      double kG1) {
    io.updateSlot0ArmGains(kP0, kD0, kS0, kV0, kA0, kG0);
    io.updateSlot1ArmGains(kP1, kD1, kS1, kV1, kA1, kG1);
  }

  public void updateArmConstraints(double maxAcceleration, double maxVelocity) {
    io.updateArmConstraints(maxAcceleration, maxVelocity);
  }

  @AutoLogOutput(key = "Manipulator/Arm At Goal")
  public boolean algaeArmAtGoal() {
    return inputs.armPosition.getRadians() - armGoal.getAngle().getRadians()
        <= V2_RedundancyManipulatorConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS().get();
  }

  public Command waitUntilAlgaeArmAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::algaeArmAtGoal));
  }

  public Command homingSequence() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              isClosedLoop = false;
              io.armMax();
            }),
        Commands.runEnd(() -> io.setArmVoltage(-6), () -> io.setArmVoltage(0))
            .until(() -> Math.abs(inputs.armTorqueCurrentAmps) > 40),
        Commands.runOnce(() -> io.zeroArmPosition()));
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
        ManipulatorRollerState.ALGAE_INTAKE.getVoltage() / 1.5);
  }
}
