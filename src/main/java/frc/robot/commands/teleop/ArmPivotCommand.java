// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivot;

public class ArmPivotCommand extends CommandBase {
  /** Creates a new ArmPivotCommand. */
  private ArmPivot subsystem;
  private Joystick secondarycontroller;
  private Joystick rightJoystick;
  private SlewRateLimiter limiter = new SlewRateLimiter(0.5);
  private double speedMult = 1;
  public ArmPivotCommand(ArmPivot subsystem, Joystick secondarycontroller, Joystick rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.secondarycontroller = secondarycontroller;
    this.rightJoystick = rightJoystick;
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
    speedMult = (-rightJoystick.getRawAxis(3) +1)/2;
    subsystem.turning(limiter.calculate(Math.pow(secondarycontroller.getRawAxis(1),2)*speedMult));
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
