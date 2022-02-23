// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.drivers.TurningEncoder;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  // private final Encoder m_driveEncoder;
  private final TurningEncoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderPort,
      boolean driveMotorReversed,
      boolean turningEncoderReversed) {

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_turningMotor = new TalonFX(turningMotorChannel);

    // Set whether drive motor and sensor should be reversed or not
    if (driveMotorReversed) {
      m_driveMotor.setInverted(TalonFXInvertType.CounterClockwise);
    } else {
      m_driveMotor.setInverted(TalonFXInvertType.Clockwise);
    }
    // Set the distance (in this case, angle) per rotation for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi)
    this.m_turningEncoder = new TurningEncoder(turningEncoderPort);

    // Set whether turning encoder should be reversed or not
    m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getRate(), new Rotation2d(m_turningEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetTurningEncoder() {
    m_turningEncoder.reset();
  }

  public void resetDrivingEncoder() {
    m_driveMotor.setSelectedSensorPosition(0.0);
  }

  private double getRate() {
    return (m_driveMotor.getSelectedSensorVelocity() * 10.0 * Constants.ModuleConstants.kDriveEncoderDistancePerPulse);
  }
}
