package frc.robot.subsystems.v2_Redundancy.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
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
  private boolean isGoingUp;

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
    isGoingUp = false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);

    isGoingUp = state.equals(ArmState.UP);

    if (isClosedLoop) io.setArmPositionGoal(state.getAngle(), isGoingUp
    );

    if (RobotState.isHasAlgae()) {
      io.setRollerVoltage(.75);
    }

    if (RobotState.isIntakingAlgae()) {
      if (inputs.rollerVelocityRadiansPerSecond <= 50.0) {
        RobotState.setHasAlgae(true);
      }
    }

    if (RobotState.isHasAlgae() && !RobotState.isIntakingAlgae()) {
      if (inputs.rollerVelocityRadiansPerSecond >= 50.0) {
        RobotState.setHasAlgae(false);
      } else {
        io.setRollerVoltage(0.75);
      }
    }
  }

  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return Math.abs(inputs.rollerTorqueCurrentAmps)
        > V2_RedundancyManipulatorConstants.ROLLER_CURRENT_THRESHOLD;
  }

  @AutoLogOutput(key = "Manipulator/Has Algae")
  public boolean hasAlgae() {
    return Math.abs(inputs.rollerTorqueCurrentAmps)
        > V2_RedundancyManipulatorConstants.ROLLER_CURRENT_THRESHOLD;
  }

  public Command runManipulator(double volts) {
    return Commands.runEnd(() -> io.setRollerVoltage(volts), () -> io.setRollerVoltage(0));
  }

  public Command intakeCoral() {
    return Commands.sequence(
        runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.CORAL_INTAKE_VOLTS().get())
            .until(() -> hasCoral()));
  }

  public Command intakeAlgae() {
    return Commands.sequence(
            Commands.runOnce(() -> RobotState.setIntakingAlgae(true)),
            Commands.parallel(
                runManipulator(
                    V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.CORAL_INTAKE_VOLTS().get())))
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
}
