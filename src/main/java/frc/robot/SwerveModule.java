// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SwerveModule {

    CANSparkLowLevel spark = new CANSparkLowLevel(); 
    
    spark.someMethod();

    private CANSparkFlex Motor;

    private static final double kWheelRadius = 2;
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final CANSparkFlex m_driveMotor;
    private final CANSparkFlex m_turningMotor;

    private final Encoder m_driveEncoder;
    private final Encoder m_turningEncoder;

// Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

// Gains are for example purposes only - must be determined for your own robot!
private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController( 1, 0, 0, new TrapezoidProfile.Constraints( kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

 // Gains are for example purposes only - must be determined for your own robot!
 private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
 private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

 /*public void SparkFlexSwerve(CANSparkFlex motor, MotorType motorType) {
    this.motor = motor;
    motor.setType(); // Set the motor type
}
*/
    





 public SwerveModule(
    int driveMotorChannel,
    int turningMotorChannel,
    int driveEncoderChannelA,
    int driveEncoderChannelB,
    int turningEncoderChannelA,
    int turningEncoderChannelB) {
        m_driveMotor = new CANSparkFlex(turningEncoderChannelB, CANSparkFlex.MotorType.kBrushless);
        m_turningMotor = new CANSparkFlex(turningEncoderChannelB, CANSparkFlex.MotorType.kBrushless);
    
        m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
        m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
    
        m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

        m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
      }


      public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
      }
/**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  } 
}
