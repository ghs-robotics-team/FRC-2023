// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivot;

public class ArmPivotCommand extends CommandBase {
  /** Creates a new ArmPivotCommand. */
  private ArmPivot subsystem;
  private Joystick secondarycontroller;
  public ArmPivotCommand(ArmPivot subsystem, Joystick secondarycontroller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.secondarycontroller = secondarycontroller;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.turning(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.turning(secondarycontroller.getRawAxis(5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.turning(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
