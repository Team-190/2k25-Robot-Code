package frc.robot.subsystems.v0_Funky.kitbot_roller;

import edu.wpi.first.math.system.plant.DCMotor;

public class V0_FunkyRollerConstants {
  public static final int ROLLER_CAN_ID = 9;
  public static final double SUPPLY_CURRENT_LIMIT = 40;
  public static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final double ROLLER_MOTOR_GEAR_RATIO = 3.0;
}
