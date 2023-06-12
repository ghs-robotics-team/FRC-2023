// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private TalonFX frontLeft = new TalonFX(0);
  private TalonFX frontRight = new TalonFX(2);
  private TalonFX backLeft = new TalonFX(1);
  private TalonFX backRight = new TalonFX(3);
  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20));//Might also be 10
  private DifferentialDriveOdometry odometry;
  private final Field2d field = new Field2d();
  private double gearRatio = 1 / 10.71;

  private double ksVolts = 0.34861;
  private double kvVoltSecondsPerMeter = 2.4841;
  private double kaVoltSecondsSquaredPerMeter = 0.81206;
  private double kpDriveLevel = 3.6728;
  private PIDController lpid = new PIDController(kpDriveLevel, 0, 0);
  private SimpleMotorFeedforward lfeedforward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  private PIDController rpid = new PIDController(kpDriveLevel, 0, 0);
  private SimpleMotorFeedforward rfeedforward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    frontRight.setInverted(true);
    backRight.setInverted(true);
    frontLeft.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);
    SlotConfiguration pid = new SlotConfiguration();
    pid.kP = 0.10988;
    pid.kF = 0.1079;
    frontRight.configureSlot(pid);
    frontLeft.configureSlot(pid);
    backRight.configureSlot(pid);
    backLeft.configureSlot(pid);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), ticksToMeters(frontLeft), ticksToMeters(frontRight));
    gyro.calibrate();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(velocityToMeters(frontLeft.getSelectedSensorVelocity()), velocityToMeters(frontRight.getSelectedSensorVelocity()));
  }
  public void tankdrive(double left, double right){
    frontLeft.set(TalonFXControlMode.PercentOutput,left);
    frontRight.set(TalonFXControlMode.PercentOutput,right);
  }
  public void tankdriveVelocity(double rightVelocity, double leftVelocity){
    double l = lpid.calculate(velocityToMeters(leftVelocity)) + lfeedforward.calculate(velocityToMeters(leftVelocity));
    double r = rpid.calculate(velocityToMeters(rightVelocity)) + rfeedforward.calculate(velocityToMeters(rightVelocity));
    frontLeft.set(TalonFXControlMode.PercentOutput,l/12);
    frontRight.set(TalonFXControlMode.PercentOutput,r/12);
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  public void resetPose(Pose2d pose){
    odometry.resetPosition(gyro.getRotation2d(), ticksToMeters(frontLeft), ticksToMeters(frontRight), pose);
  }
  public void enableBrake(){
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
  }
  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }
  public double ticksToMeters(TalonFX talon){
    double r = Units.inchesToMeters(3);
    //arc length = rad*r;
    double rad = talon.getSelectedSensorPosition()/2048.0 * 2 * Math.PI * gearRatio;
    return rad*r;
  }
  public double velocityToMeters(double velocity){
    double r = Units.inchesToMeters(3);
    //arc length = rad*r;
    double rad = velocity/2048.0 * 2 * Math.PI * gearRatio;
    return rad*r*10;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), ticksToMeters(frontLeft), ticksToMeters(frontRight));
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("Field",field);
    SmartDashboard.putNumber("Roll",gyro.getRoll());
    SmartDashboard.putNumber("Pitch",gyro.getPitch());
    SmartDashboard.putNumber("Yaw",gyro.getYaw());
  }
}
