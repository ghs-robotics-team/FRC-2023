// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtension;

public class ArmExtensionCommand extends CommandBase {
  /** Creates a new ArmExtension. */
  private ArmExtension subsystem;
  private Joystick secondarycontroller;
  private double speedMult = 0.2;

  public ArmExtensionCommand(ArmExtension subsystem, Joystick secondarycontroller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.secondarycontroller = secondarycontroller;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (secondarycontroller.getRawButton(7)){
      speedMult -= 0.001;
    } 
    else if (secondarycontroller.getRawButton(8)){
      speedMult += 0.001;
    }
    this.subsystem.runSpark(secondarycontroller.getRawAxis(5)*.1);
    this.subsystem.extendArm(speedMult*secondarycontroller.getRawAxis(4));
    SmartDashboard.putNumber("extenderPosition",subsystem.getEncoderValue());
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
