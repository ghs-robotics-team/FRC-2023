// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivot extends SubsystemBase {
  private CANSparkMax armSpark = new CANSparkMax(4, MotorType.kBrushless);
  /** Creates a new ArmPivot. */
  public ArmPivot() {
    armSpark.setIdleMode(IdleMode.kBrake);
  }
  public void turning (double speed){
    armSpark.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
