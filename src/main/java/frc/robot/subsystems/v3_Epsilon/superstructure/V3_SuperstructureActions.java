package frc.robot.subsystems.v3_Epsilon.superstructure;

import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulatorConstants;
import lombok.Getter;

public class V3_SuperstructureActions{
    
    @Getter private final ReefState elevatorHeight;
    @Getter private final V3_EpsilonManipulatorConstants.PivotState armState;
    @Getter private final V3_EpsilonIntakeConstants.IntakeState intakeState;

    
    
    
    
    public record SubsystemActions(
        ReefState elevatorHeight,
        V3_EpsilonManipulatorConstants.PivotState manipulatorArmState,
        V3_EpsilonIntakeConstants.IntakeState intakeState) {
    }
}
