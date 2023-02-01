// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helper.PID;
import edu.wpi.first.wpilibj.SPI;


public class Balance extends SubsystemBase {
  /** Creates a new Balance. */
  public PID controller = new PID(0, 0, 0);
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  public double zero = 0;
  public double out = 0;
  public void reset (){
    zero = gyro.getRoll();
  }
  public Balance() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    out = controller.step(gyro.getRoll() - zero);
  }
}
