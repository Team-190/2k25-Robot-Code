package frc.robot.subsystems.v1_gamma.climber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class V1_GammaClimber extends SubsystemBase {
  private V1_GammaClimberIO io;
  private ClimberIOInputsAutoLogged inputs;

  @AutoLogOutput(key="Climber/isClimbed")
  private boolean isClimbed;

  public V1_GammaClimber(V1_GammaClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
    
    isClimbed = false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    
    isClimbed = io.isClimbed();
  }

  public Command setCurrent(double amps) {
    return this.run(()->io.setCurrent(amps));
  }

  public Command releaseClimber() {
    return this.run(()->io.setVoltage(4)).withTimeout(0.1);
  }

  public Command wintchClimber() {
    return this.run(()->io.setCurrent(-60)).until(() -> isClimbed);
  }
}
