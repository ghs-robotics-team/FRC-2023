// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmShoulder extends SubsystemBase {
  private CANSparkMax armSpark = new CANSparkMax(4, MotorType.kBrushless);
  double TWO_PI = 2*Math.PI;
  /** Creates a new ArmPivot. */
  public ArmShoulder() {
    armSpark.setIdleMode(IdleMode.kBrake);
  }

  public void setPosition(double rad){
    armSpark.getPIDController().setReference(rad/TWO_PI, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}