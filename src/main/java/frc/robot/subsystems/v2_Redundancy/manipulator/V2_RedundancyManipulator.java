package frc.robot.subsystems.v2_Redundancy.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorConstants;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyManipulator extends SubsystemBase {
  private final V2_RedundancyManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;
  private boolean isClosedLoop;
  private final SysIdRoutine algaeCharacterizationRoutine;
  @Getter private ArmState state;
  private double hasAlgaeTimestamp;

  public V2_RedundancyManipulator(V2_RedundancyManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();
    isClosedLoop = true;
    algaeCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(5),
                (state) -> Logger.recordOutput("Manipulator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setArmVoltage(volts.in(Volts)), null, this));

    state = ArmState.DOWN;
    hasAlgaeTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);

    if (isClosedLoop) io.setArmPositionGoal(state.getAngle());

    if (RobotState.isHasAlgae()) {
      io.setRollerVoltage(holdVoltage());
    }

    if (RobotState.isIntakingAlgae()) {
      if (hasAlgae()) {
        RobotState.setHasAlgae(true);
      }
    }
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
    if (Math.abs(inputs.rollerTorqueCurrentAmps) > 35
        && Math.abs(inputs.rollerVelocityRadiansPerSecond) <= 50.0)
      hasAlgaeTimestamp = Timer.getFPGATimestamp();

    return Timer.getFPGATimestamp() < hasAlgaeTimestamp + 0.5;
  }

  @AutoLogOutput(key = "Manipulator/Has Algae")
  public boolean isIntakingAlgae() {
    return Math.abs(inputs.rollerVelocityRadiansPerSecond) >= 100.0;
  }

  public Command runManipulator(double volts) {
    return Commands.runEnd(() -> io.setRollerVoltage(volts), () -> io.setRollerVoltage(0));
  }

  public Command intakeCoral() {
    return Commands.sequence(
        runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.CORAL_INTAKE_VOLTS().get())
            .until(() -> hasCoral()));
  }

  public Command intakeReefAlgae() {
    return Commands.sequence(
            Commands.parallel(
                Commands.sequence(
                    Commands.waitUntil(() -> isIntakingAlgae()),
                    Commands.runOnce(() -> RobotState.setIntakingAlgae(true))),
                runManipulator(
                    V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().get())))
        .finallyDo(() -> RobotState.setIntakingAlgae(false));
  }

  public Command intakeFloorAlgae() {
    return Commands.sequence(
            Commands.parallel(
                Commands.sequence(
                    Commands.waitUntil(() -> isIntakingAlgae()),
                    Commands.runOnce(() -> RobotState.setIntakingAlgae(true))),
                runManipulator(
                    V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().get())))
        .finallyDo(() -> RobotState.setIntakingAlgae(false));
  }

  public Command scoreCoral() {
    return runManipulator(
        V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_CORAL_VOLTS().get());
  }

  public Command scoreAlgae() {
    return Commands.sequence(
        runManipulator(-V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_ALGAE_VOLTS().get())
            .withTimeout(0.5),
        Commands.runOnce(() -> RobotState.setHasAlgae(false)));
  }

  public Command scoreL1Coral() {
    return runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.L1_VOLTS().get());
  }

  public Command scoreL4Coral() {
    return runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.L4_VOLTS().get());
  }

  public Command halfScoreCoral() {
    return runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.HALF_VOLTS().get());
  }

  public Command unHalfScoreCoral() {
    return runManipulator(-V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.HALF_VOLTS().get());
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

  public Command setAlgaeArmGoal(ArmState goal) {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          state = goal;
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
      double kG1) {
    io.updateSlot0ArmGains(kP0, kD0, kS0, kV0, kA0, kG0);
    io.updateSlot1ArmGains(kP1, kD1, kS1, kV1, kA1, kG1);
  }

  public void updateArmConstraints(double maxAcceleration, double maxVelocity) {
    io.updateArmConstraints(maxAcceleration, maxVelocity);
  }

  public boolean algaeArmAtGoal() {
    return inputs.armPosition.getRadians() - this.state.getAngle().getRadians()
        <= V2_RedundancyManipulatorConstants.CONSTRAINTS.goalToleranceRadians().get();
  }

  public Command waitUntilAlgaeArmAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::algaeArmAtGoal));
  }

  private double holdVoltage() {
    return MathUtil.clamp(
        Math.abs(inputs.rollerTorqueCurrentAmps) > 25
            ? 3
            : 17 / Math.abs(inputs.rollerTorqueCurrentAmps),
        0.5,
        V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().get());
  }
}
