// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax m_frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearRight = new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

  private final MecanumDrive m_drive =
      new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);


  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees( m_gyro.getAngle() ),
          new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    m_frontLeft.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rearLeft.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_frontRight.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rearRight.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);


    m_frontLeft.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse/60);
    m_frontRight.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse/60);
    m_rearLeft.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse/60);
    m_rearRight.getEncoder().setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse/60);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontLeft.setInverted(false);
    m_rearLeft.setInverted(false);
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees( m_gyro.getAngle() ), getCurrentWheelDistances());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(Rotation2d.fromDegrees( m_gyro.getAngle() ), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, Rotation2d.fromDegrees( m_gyro.getAngle() ));
    } else {
      m_drive.driveCartesian(xSpeed, ySpeed, rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    getFrontLeftEncoder().setPosition(0);
    getFrontRightEncoder().setPosition(0);
    getRearLeftEncoder().setPosition(0);
    getRearRightEncoder().setPosition(0);
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public RelativeEncoder getFrontLeftEncoder() {
    return m_frontLeft.getEncoder();
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public RelativeEncoder getRearLeftEncoder() {
    return m_rearLeft.getEncoder();
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public RelativeEncoder getFrontRightEncoder() {
    return m_frontRight.getEncoder();
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public RelativeEncoder getRearRightEncoder() {
    return m_rearRight.getEncoder();
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        getFrontLeftEncoder().getVelocity(),
        getFrontRightEncoder().getVelocity(),
        getRearLeftEncoder().getVelocity(),
        getRearRightEncoder().getVelocity());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        getFrontLeftEncoder().getPosition(),
        getFrontRightEncoder().getPosition(),
        getRearLeftEncoder().getPosition(),
        getRearRightEncoder().getPosition());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
