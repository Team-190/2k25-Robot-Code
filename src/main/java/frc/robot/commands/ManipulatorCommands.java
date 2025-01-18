package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.v1_gamma.manipulator.Manipulator;
import frc.robot.util.LoggedTunableNumber;

public class ManipulatorCommands {
    
    public static final Command setManipulatorVoltage(Manipulator manipulator, LoggedTunableNumber volts) {
        return manipulator.runManipulator(volts.get());
    }

}
