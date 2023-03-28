// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private DoubleSolenoid claw = new DoubleSolenoid(6, PneumaticsModuleType.REVPH, 0, 1);
  private CANSparkMax leftMotor = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(8, MotorType.kBrushless);
  public double runSpeed = 0;

  /** Creates a new Claw. */
  public Claw() {
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    //Flip motors here if they are opposite
    leftMotor.setInverted(true);
  }

  public void openClaw(){
    claw.set(Value.kReverse);
  }
  public void run(double speed){
    runSpeed = speed;
  }
  public void closeClaw(){
    claw.set(Value.kForward);
  }   
  public void killClaw(){
    claw.set(Value.kOff);
  }   
  
  public void toggleClaw(){
    claw.toggle();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftMotor.set(runSpeed);
    rightMotor.set(runSpeed);
    if(runSpeed == 0){
      SmartDashboard.putBoolean("Claw Running", false);
    }else{
      SmartDashboard.putBoolean("Claw Running", true);
    }
  }

  }
