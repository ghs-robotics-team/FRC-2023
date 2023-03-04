// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class RunClawCommand extends CommandBase {
  /** Creates a new RunClawCommand. */
  private Claw claw;
  private double timer = 0;
  /**
   * I suggest using this on a parallel command, either race or normal
   * @param speed The speed of the claw motors
   * @param claw The claw subsystem we are using
   * @param timer The time in seconds the claw should run
   */
  public RunClawCommand(double speed, Claw claw, double timer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.timer = timer;
    addRequirements(claw);
    claw.run(speed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer-=1/50;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.run(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer<=0;
  }
}
