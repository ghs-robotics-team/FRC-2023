// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.AutoBalance;
import frc.robot.subsystems.DriveTrain;

public class AutoBalanceDrive extends CommandBase {
  /** Creates a new AutoBalanceDrive. */
  private DriveTrain drive;
  private AutoBalance balance = new AutoBalance();

  public AutoBalanceDrive(DriveTrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = balance.autoBalanceRoutine(drive.gyro.getRoll());
    drive.tankdrive(-speed, -speed);
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
