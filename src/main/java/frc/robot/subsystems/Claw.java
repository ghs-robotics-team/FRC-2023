// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private CANSparkMax claw = new CANSparkMax(5,MotorType.kBrushless);

  /** Creates a new Claw. */
  public Claw() {
    claw.setIdleMode(IdleMode.kBrake);
  }

  public void moveClaw(double speed){
    claw.set(speed);
  }   
  
  public double getEncoderValue(){
    return claw.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  }
