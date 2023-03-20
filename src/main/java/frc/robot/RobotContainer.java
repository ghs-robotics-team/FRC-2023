// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.misc.ArmElbowCommand;
import frc.robot.commands.misc.ArmShoulderCommand;
import frc.robot.commands.teleop.MoveArmCommand;
import frc.robot.commands.teleop.RotateArmSimple;
import frc.robot.commands.teleop.TankDrive;
import frc.robot.helper.AutoType;
import frc.robot.subsystems.ArmElbow;
import frc.robot.subsystems.ArmShoulder;
import frc.robot.subsystems.ArmBrake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Subsystems
  private ArmShoulder armPivot = new ArmShoulder();
  private Claw claw = new Claw();
  private DriveTrain driveTrain = new DriveTrain();
  private ArmElbow armElbow = new ArmElbow();
  private ArmBrake armBrake = new ArmBrake();
  private SendableChooser<Command> chooser = new SendableChooser<>();

  //Joysticks
  private Joystick joystick_left = new Joystick(0);
  private Joystick joystick_right = new Joystick(1);
  private Joystick secondarycontroller = new Joystick(2);

  JoystickButton rightTrigger = new JoystickButton(joystick_right, 1);

  //private InverseKinematics IK = new InverseKinematics();

  //Commands
  private MoveArmCommand moveArmCommand = new MoveArmCommand(secondarycontroller, claw);
  private ArmShoulderCommand armPivotCommand = new ArmShoulderCommand(armPivot);
  private ArmElbowCommand armElbowCommand = new ArmElbowCommand(armElbow);
  private RotateArmSimple rotateArmSimple = new RotateArmSimple(armBrake, secondarycontroller);
  private TankDrive tankDrive = new TankDrive(driveTrain, joystick_left, joystick_right);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // chooser.setDefaultOption("Top Auto", topAuto);
    // chooser.addOption("Mid Auto", midAuto);
    // chooser.addOption("Bottom Auto", bottomAuto);
    // chooser.addOption("No Auto", new WaitCommand(15));
    // chooser.addOption("Basic Auto", basicAuto);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  public void setup(){
    armElbow.setDefaultCommand(armElbowCommand);
    armPivot.setDefaultCommand(armPivotCommand);
    moveArmCommand.schedule();
    driveTrain.setDefaultCommand(tankDrive);
    rotateArmSimple.schedule();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(AutoType type) {
    // An example command will be run in autonomous
    return new WaitCommand(15);
  }

  public DriveTrain getDrivetrain(){
    return driveTrain;
  }
}
