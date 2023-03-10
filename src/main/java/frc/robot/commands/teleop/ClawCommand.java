// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawCommand extends CommandBase {
  private Joystick controller;
  private Claw subsystem;
  private double runTimer = 0;

  /** Creates a new Claw. */
  public ClawCommand(Claw subsystem, Joystick controller) {
    this.subsystem = subsystem;
    this.controller = controller;
    addRequirements(this.subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.closeClaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getRawAxis(2)>0.1){
      subsystem.openClaw();
    }
    if(controller.getRawAxis(3)>0.1){
      subsystem.closeClaw();
    }
    if(controller.getRawButton(5)){
      subsystem.openClaw();
      runTimer = 50;
      subsystem.run(-1);
    }
    if(runTimer > 0){
      runTimer--;
    }else{
      subsystem.run(0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
