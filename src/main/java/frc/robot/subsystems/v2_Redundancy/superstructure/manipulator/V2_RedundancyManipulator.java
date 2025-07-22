package frc.robot.subsystems.v2_Redundancy.superstructure.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import java.util.Set;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyManipulator extends SubsystemBase {
  private final V2_RedundancyManipulatorIO io;
  private final V2_RedundancyManipulatorIOInputsAutoLogged inputs;
  private boolean isClosedLoop;
  private final SysIdRoutine algaeCharacterizationRoutine;

  @Getter
  @AutoLogOutput(key = "Manipulator/Arm Goal")
  private ArmState armGoal;

  @Getter
  @Setter
  @AutoLogOutput(key = "Manipulator/Roller Goal")
  private ManipulatorRollerState rollerGoal;

  public V2_RedundancyManipulator(V2_RedundancyManipulatorIO io) {
    this.io = io;
    inputs = new V2_RedundancyManipulatorIOInputsAutoLogged();
    isClosedLoop = true;
    algaeCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(5),
                (state) -> Logger.recordOutput("Manipulator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setArmVoltage(volts.in(Volts)), null, this));

    armGoal = ArmState.STOW_DOWN;
    rollerGoal = ManipulatorRollerState.STOP;
  }

  @Override
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

    if (RobotState.isHasAlgae()
        && Set.of(
                ManipulatorRollerState.STOP,
                ManipulatorRollerState.ALGAE_INTAKE,
                // ManipulatorRollerState.REMOVE_ALGAE,
                ManipulatorRollerState.CORAL_INTAKE)
            .contains(rollerGoal)) {
      io.setRollerVoltage(holdVoltage());
    } else {
      io.setRollerVoltage(rollerGoal.getVoltage());
    }

    if (hasAlgae() && RobotState.isIntakingAlgae()) {
      RobotState.setHasAlgae(true);
    }
    InternalLoggedTracer.record("Manipulator Logic", "Manipulator/Periodic");
    ExternalLoggedTracer.record("Manipulator Total", "Manipulator/Periodic");
  }

  public Rotation2d getArmAngle() {
    return inputs.armPosition;
  }

  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return Math.abs(inputs.rollerTorqueCurrentAmps)
        > V2_RedundancyManipulatorConstants.ROLLER_CURRENT_THRESHOLD;
  }

  @AutoLogOutput(key = "Manipulator/Has Algae")
  public boolean hasAlgae() {
    return RobotState.isIntakingAlgae()
        && Math.abs(inputs.rollerAccelerationRadiansPerSecondSquared) < 1000
        && Math.abs(inputs.rollerVelocityRadiansPerSecond) <= 70;
  }

  @AutoLogOutput(key = "Manipulator/Intaking Algae")
  public boolean isIntakingAlgae() {
    return Math.abs(inputs.rollerVelocityRadiansPerSecond) >= 100.0;
  }

  public Command sysIdRoutine() {
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

  public void setAlgaeArmGoal(ArmState goal) {
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
        <= V2_RedundancyManipulatorConstants.CONSTRAINTS.goalToleranceRadians().get();
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
        V2_RedundancyManipulatorConstants.ManipulatorRollerState.ALGAE_INTAKE.getVoltage() / 1.5);
  }
}
