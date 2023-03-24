// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  private Joystick leftJoystick;
    private Joystick rightJoystick;
    private DriveTrain subsystem;
    private double speedMult = .8;
  /** Creates a new TankDrive. */
  public TankDrive(DriveTrain subsystem, Joystick leftJoystick, Joystick rightJoystick) {
    this.subsystem = subsystem;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    addRequirements(this.subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.tankdrive(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speedMult = (-leftJoystick.getRawAxis(3) +1)/2;
    subsystem.tankdrive(-leftJoystick.getRawAxis(1)*speedMult , -rightJoystick.getRawAxis(1)*speedMult);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.tankdrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
