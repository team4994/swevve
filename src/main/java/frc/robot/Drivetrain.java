package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;



public class Drivetrain {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);

  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);

  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);

  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);


  private final SwerveModule frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);

  private final SwerveModule frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);

  private final SwerveModule backLeft = new SwerveModule(6, 7, 8, 9, 10, 11);

  private final SwerveModule backRight = new SwerveModule(8, 9, 12, 13, 14, 15);


  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics( m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

private final SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry( m_kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()});

public Drivetrain() {

    m_gyro.reset();
}

public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_Odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }
    





}