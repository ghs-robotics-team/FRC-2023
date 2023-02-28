// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmElbow;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmBrake;


public class RotateArmSimple extends CommandBase {
  /** Creates a new MoveArmCommand. */
  private Joystick secondary;
  private ArmElbow elbowSubsystem;
  private ArmShoulder shoulderSubsystem;
  private ArmBrake brakeSubsystem;

  public RotateArmSimple(ArmElbow elbowSubsystem, ArmShoulder shoulderSubsystem, ArmBrake brakeSubsystem, Joystick secondary) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.brakeSubsystem = brakeSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    this.secondary = secondary;

    addRequirements(this.elbowSubsystem, this.shoulderSubsystem, this.brakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(secondary.getRawAxis(0))<0.1){
      brakeSubsystem.brakeElbow();
    }else{
      brakeSubsystem.releaseElbow();
    }
    if(Math.abs(secondary.getRawAxis(4))<0.1){
      brakeSubsystem.brakeShoulder();
    }else{
      brakeSubsystem.releaseShoulder();
    }
    elbowSubsystem.setSpeed(secondary.getRawAxis(0)*.05);
    shoulderSubsystem.setSpeed(secondary.getRawAxis(4)*0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
