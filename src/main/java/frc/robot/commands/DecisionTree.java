package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.Elevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.Intake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.ManipulatorConstants.ArmState;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DecisionTree {

  private static final BooleanSupplier getIntakeStartIn(Intake intake) {
    return () -> intake.atGoal() && intake.getGoal().equals(IntakeState.STOW);
  }

  private static final BooleanSupplier getIntakeEndIn(IntakeState intakeState) {
    return () -> intakeState.equals(IntakeState.STOW);
  }

  private static final BooleanSupplier getElevatorStartBelow(Elevator elevator) {
    return () -> elevator.getPositionMeters() < ElevatorPositions.ALGAE_MID.getPosition();
  }

  private static final BooleanSupplier getElevatorEndBelow(
      Elevator elevator, Supplier<ReefState> elevatorState) {
    return () ->
        elevator.getPosition(elevatorState.get()).getPosition()
            < ElevatorPositions.ALGAE_MID.getPosition();
  }

  private static final BooleanSupplier getArmCrossStowLine(
      Manipulator manipulator, ArmState armEnd) {
    return () ->
        (manipulator.getArmAngle().getRadians() < ArmState.STOW_LINE.getAngle().getRadians()
                && armEnd.getAngle().getRadians() > ArmState.STOW_LINE.getAngle().getRadians())
            || (manipulator.getArmAngle().getRadians() > ArmState.STOW_LINE.getAngle().getRadians()
                && armEnd.getAngle().getRadians() < ArmState.STOW_LINE.getAngle().getRadians());
  }

  private static final BooleanSupplier getArmCrossIntakeOutLine(
      Manipulator manipulator, ArmState armEnd) {
    return () ->
        (manipulator.getArmAngle().getRadians() < ArmState.INTAKE_OUT_LINE.getAngle().getRadians()
                && armEnd.getAngle().getRadians()
                    > ArmState.INTAKE_OUT_LINE.getAngle().getRadians())
            || (manipulator.getArmAngle().getRadians()
                    > ArmState.INTAKE_OUT_LINE.getAngle().getRadians()
                && armEnd.getAngle().getRadians()
                    < ArmState.INTAKE_OUT_LINE.getAngle().getRadians());
  }

  private static final Command ES_A_EE(
      Elevator elevator,
      Manipulator manipulator,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        elevator.setPosition(() -> ReefState.ALGAE_MID),
        elevator.waitUntilAtGoal(),
        manipulator.setAlgaeArmGoal(armState),
        manipulator.waitUntilAlgaeArmAtGoal(),
        elevator.setPosition(elevatorState),
        elevator.waitUntilAtGoal());
  }

  private static final Command ES_AEE(
      Elevator elevator,
      Manipulator manipulator,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        elevator.setPosition(() -> ReefState.ALGAE_MID),
        elevator.waitUntilAtGoal(),
        Commands.parallel(
            elevator.setPosition(elevatorState),
            manipulator.setAlgaeArmGoal(armState),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal()));
  }

  private static final Command ESA_EE(
      Elevator elevator,
      Manipulator manipulator,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(() -> ReefState.ALGAE_MID),
            manipulator.setAlgaeArmGoal(armState),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal()),
        elevator.setPosition(elevatorState),
        elevator.waitUntilAtGoal());
  }

  private static final Command AEE(
      Elevator elevator,
      Manipulator manipulator,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(elevatorState),
            manipulator.setAlgaeArmGoal(armState),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal()));
  }

  private static final Command IOEAI_A_EE(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(() -> ReefState.ALGAE_FLOOR_INTAKE),
            intake.setExtensionGoal(IntakeState.INTAKE),
            elevator.waitUntilAtGoal(),
            intake.waitUntilExtensionAtGoal()),
        manipulator.setAlgaeArmGoal(armState),
        manipulator.waitUntilAlgaeArmAtGoal(),
        elevator.setPosition(elevatorState),
        elevator.waitUntilAtGoal());
  }

  private static final Command IOEAI_AEE(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(() -> ReefState.ALGAE_FLOOR_INTAKE),
            intake.setExtensionGoal(IntakeState.INTAKE),
            elevator.waitUntilAtGoal(),
            intake.waitUntilExtensionAtGoal()),
        Commands.parallel(
            manipulator.setAlgaeArmGoal(armState),
            manipulator.waitUntilAlgaeArmAtGoal(),
            elevator.setPosition(elevatorState),
            elevator.waitUntilAtGoal()));
  }

  private static final Command IOEAIA_EE(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(() -> ReefState.ALGAE_FLOOR_INTAKE),
            manipulator.setAlgaeArmGoal(armState),
            intake.setExtensionGoal(IntakeState.INTAKE),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal(),
            intake.waitUntilExtensionAtGoal()),
        elevator.setPosition(elevatorState),
        elevator.waitUntilAtGoal());
  }

  private static final Command IOAEE(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(elevatorState),
            manipulator.setAlgaeArmGoal(armState),
            intake.setExtensionGoal(IntakeState.INTAKE),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal(),
            intake.waitUntilExtensionAtGoal()));
  }

  private static final Command IOES_A_EE(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(() -> ReefState.ALGAE_MID),
            intake.setExtensionGoal(IntakeState.INTAKE),
            elevator.waitUntilAtGoal(),
            intake.waitUntilExtensionAtGoal()),
        manipulator.setAlgaeArmGoal(armState),
        manipulator.waitUntilAlgaeArmAtGoal(),
        elevator.setPosition(elevatorState),
        elevator.waitUntilAtGoal());
  }

  private static final Command IOES_AEE(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(() -> ReefState.ALGAE_MID),
            intake.setExtensionGoal(IntakeState.INTAKE),
            elevator.waitUntilAtGoal(),
            intake.waitUntilExtensionAtGoal()),
        Commands.parallel(
            manipulator.setAlgaeArmGoal(armState),
            manipulator.waitUntilAlgaeArmAtGoal(),
            elevator.setPosition(elevatorState),
            elevator.waitUntilAtGoal()));
  }

  private static final Command IOESA_EE(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(() -> ReefState.ALGAE_MID),
            manipulator.setAlgaeArmGoal(armState),
            intake.setExtensionGoal(IntakeState.INTAKE),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal(),
            intake.waitUntilExtensionAtGoal()),
        elevator.setPosition(elevatorState),
        elevator.waitUntilAtGoal());
  }

  private static final Command EAI_A_EEII(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        elevator.setPosition(() -> ReefState.ALGAE_FLOOR_INTAKE),
        elevator.waitUntilAtGoal(),
        manipulator.setAlgaeArmGoal(armState),
        manipulator.waitUntilAlgaeArmAtGoal(),
        Commands.parallel(
            elevator.setPosition(elevatorState),
            intake.setExtensionGoal(IntakeState.STOW),
            elevator.waitUntilAtGoal(),
            intake.waitUntilExtensionAtGoal()));
  }

  private static final Command EAI_AEEII(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        elevator.setPosition(() -> ReefState.ALGAE_FLOOR_INTAKE),
        elevator.waitUntilAtGoal(),
        Commands.parallel(
            elevator.setPosition(elevatorState),
            manipulator.setAlgaeArmGoal(armState),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal()),
        intake.setExtensionGoal(IntakeState.STOW),
        intake.waitUntilExtensionAtGoal());
  }

  private static final Command EAIA_EEII(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(() -> ReefState.ALGAE_FLOOR_INTAKE),
            manipulator.setAlgaeArmGoal(armState),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal()),
        Commands.parallel(
            elevator.setPosition(elevatorState),
            intake.setExtensionGoal(IntakeState.STOW),
            elevator.waitUntilAtGoal(),
            intake.waitUntilExtensionAtGoal()));
  }

  private static final Command AEEII(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(elevatorState),
            manipulator.setAlgaeArmGoal(armState),
            intake.setExtensionGoal(IntakeState.STOW),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal(),
            intake.waitUntilExtensionAtGoal()));
  }

  private static final Command ES_A_EEII(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        elevator.setPosition(() -> ReefState.ALGAE_MID),
        elevator.waitUntilAtGoal(),
        manipulator.setAlgaeArmGoal(armState),
        manipulator.waitUntilAlgaeArmAtGoal(),
        Commands.parallel(
            elevator.setPosition(elevatorState),
            intake.setExtensionGoal(IntakeState.STOW),
            elevator.waitUntilAtGoal(),
            intake.waitUntilExtensionAtGoal()));
  }

  private static final Command ES_AEEII(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        elevator.setPosition(() -> ReefState.ALGAE_MID),
        elevator.waitUntilAtGoal(),
        Commands.parallel(
            elevator.setPosition(elevatorState),
            manipulator.setAlgaeArmGoal(armState),
            intake.setExtensionGoal(IntakeState.STOW),
            elevator.waitUntilAtGoal(),
            intake.waitUntilExtensionAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal()));
  }

  private static final Command ESA_EEII(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setPosition(() -> ReefState.ALGAE_MID),
            manipulator.setAlgaeArmGoal(armState),
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilAlgaeArmAtGoal()),
        Commands.parallel(
            elevator.setPosition(elevatorState),
            intake.setExtensionGoal(IntakeState.STOW),
            elevator.waitUntilAtGoal(),
            intake.waitUntilExtensionAtGoal()));
  }

  private static final Command elevatorSafeHeightSubtree(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState,
      IntakeState intakeState) {
    return Commands.either(
        Commands.either(
            ES_A_EE(elevator, manipulator, elevatorState, armState),
            ES_AEE(elevator, manipulator, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        Commands.either(
            ESA_EE(elevator, manipulator, elevatorState, armState),
            AEE(elevator, manipulator, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        getElevatorStartBelow(elevator));
  }

  private static final Command elevatorSafeHeightSubtreeIO(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState,
      IntakeState intakeState) {
    return Commands.either(
        Commands.either(
            IOES_A_EE(elevator, manipulator, intake, elevatorState, armState),
            IOES_AEE(elevator, manipulator, intake, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        Commands.either(
            IOESA_EE(elevator, manipulator, intake, elevatorState, armState),
            IOAEE(elevator, manipulator, intake, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        getElevatorStartBelow(elevator));
  }

  private static final Command elevatorSafeHeightSubtreeII(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState,
      IntakeState intakeState) {
    return Commands.either(
        Commands.either(
            ES_A_EEII(elevator, manipulator, intake, elevatorState, armState),
            ES_AEEII(elevator, manipulator, intake, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        Commands.either(
            ESA_EEII(elevator, manipulator, intake, elevatorState, armState),
            AEEII(elevator, manipulator, intake, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        getElevatorStartBelow(elevator));
  }

  private static final Command elevatorAlgaeIntakeHeightSubtreeIO(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState,
      IntakeState intakeState) {
    return Commands.either(
        Commands.either(
            IOEAI_A_EE(elevator, manipulator, intake, elevatorState, armState),
            IOEAI_AEE(elevator, manipulator, intake, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        Commands.either(
            IOEAIA_EE(elevator, manipulator, intake, elevatorState, armState),
            IOAEE(elevator, manipulator, intake, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        getElevatorStartBelow(elevator));
  }

  private static final Command elevatorAlgaeIntakeHeightSubtreeII(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState,
      IntakeState intakeState) {
    return Commands.either(
        Commands.either(
            EAI_A_EEII(elevator, manipulator, intake, elevatorState, armState),
            EAI_AEEII(elevator, manipulator, intake, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        Commands.either(
            EAIA_EEII(elevator, manipulator, intake, elevatorState, armState),
            AEEII(elevator, manipulator, intake, elevatorState, armState),
            getElevatorEndBelow(elevator, elevatorState)),
        getElevatorStartBelow(elevator));
  }

  public static final Command moveSequence(
      Elevator elevator,
      Manipulator manipulator,
      Intake intake,
      Supplier<ReefState> elevatorState,
      ArmState armState,
      IntakeState intakeState) {
    return Commands.either(
        Commands.either(
            Commands.either(
                elevatorSafeHeightSubtree(
                    elevator, manipulator, intake, elevatorState, armState, intakeState),
                AEE(elevator, manipulator, elevatorState, armState),
                getArmCrossStowLine(manipulator, armState)),
            Commands.either(
                Commands.either(
                    elevatorSafeHeightSubtreeIO(
                        elevator, manipulator, intake, elevatorState, armState, intakeState),
                    elevatorAlgaeIntakeHeightSubtreeIO(
                        elevator, manipulator, intake, elevatorState, armState, intakeState),
                    getArmCrossIntakeOutLine(manipulator, armState)),
                Commands.either(
                    elevatorSafeHeightSubtreeIO(
                        elevator, manipulator, intake, elevatorState, armState, intakeState),
                    IOEAI_A_EE(elevator, manipulator, intake, elevatorState, armState),
                    getArmCrossIntakeOutLine(manipulator, armState)),
                getArmCrossStowLine(manipulator, armState)),
            getIntakeEndIn(intakeState)),
        Commands.either(
            Commands.either(
                Commands.either(
                    elevatorSafeHeightSubtree(
                        elevator, manipulator, intake, elevatorState, armState, intakeState),
                    elevatorAlgaeIntakeHeightSubtreeII(
                        elevator, manipulator, intake, elevatorState, armState, intakeState),
                    getArmCrossIntakeOutLine(manipulator, armState)),
                Commands.either(
                    elevatorSafeHeightSubtreeII(
                        elevator, manipulator, intake, elevatorState, armState, intakeState),
                    EAI_A_EEII(elevator, manipulator, intake, elevatorState, armState),
                    getArmCrossIntakeOutLine(manipulator, armState)),
                getArmCrossStowLine(manipulator, armState)),
            Commands.either(
                Commands.either(
                    elevatorSafeHeightSubtreeIO(
                        elevator, manipulator, intake, elevatorState, armState, intakeState),
                    elevatorAlgaeIntakeHeightSubtreeIO(
                        elevator, manipulator, intake, elevatorState, armState, intakeState),
                    getArmCrossIntakeOutLine(manipulator, armState)),
                Commands.either(
                    elevatorSafeHeightSubtreeIO(
                        elevator, manipulator, intake, elevatorState, armState, intakeState),
                    IOEAI_A_EE(elevator, manipulator, intake, elevatorState, armState),
                    getArmCrossIntakeOutLine(manipulator, armState)),
                getArmCrossStowLine(manipulator, armState)),
            getIntakeEndIn(intakeState)),
        getIntakeStartIn(intake));
  }
}
