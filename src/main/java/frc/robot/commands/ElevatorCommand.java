// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase {
  private XboxController controller;
  private Elevator subsystem;


  /** Creates a new Elevator. */
  public ElevatorCommand(Elevator subsystem, XboxController controller) {
    this.subsystem = subsystem;
    this.controller = controller;
    addRequirements(this.subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   subsystem.elevation (0.0);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.elevation(controller.getRawAxis(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.elevation (0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
