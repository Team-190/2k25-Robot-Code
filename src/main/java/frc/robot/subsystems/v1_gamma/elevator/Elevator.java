package frc.robot.subsystems.v1_gamma.elevator;

import frc.robot.subsystems.v1_gamma.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputs inputs;
    public Elevator(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorIOInputs();
    }
}
