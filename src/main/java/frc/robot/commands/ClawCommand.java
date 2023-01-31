// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawCommand extends CommandBase {
  private Joystick controller;
  private Claw subsystem;
  Boolean clawClosed = true;
  boolean aPressed = false;

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
    subsystem.moveClaw(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.moveClaw(0.155*(controller.getRawAxis(3)-controller.getRawAxis(2)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.moveClaw(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
