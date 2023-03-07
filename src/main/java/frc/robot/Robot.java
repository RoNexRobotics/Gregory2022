package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private AHRS m_navX;
  private Joystick m_joystick;
  private DifferentialDrive m_driveTrain;
  private MotorControllerGroup m_leftDrive;
  private MotorControllerGroup m_rightDrive;
  private Timer m_autoTimer;
  private double m_startTime;
  final double kAutoPower = 0.8;
  final double kPowerPercent = 0.9; // 0.7
  final int path = 1;
  boolean TMOnOff = false;
  double runTime = 2;
  double cornerTime;
  double driveSpeed;
  double cornerSpeed;
  double blocks;
  WPI_VictorSPX m_leftFrontMotor;
  WPI_VictorSPX m_leftBackMotor;
  WPI_VictorSPX m_rightFrontMotor;
  WPI_VictorSPX m_rightBackMotor;
  Spark m_throwMotor;
  Spark m_climbMotor;
  DigitalInput m_throwStopLimit;
  Servo m_ballServo;
  int leverPosition;
  int testNumber = 1;
  final int RIGHTDIRECTION = -1;
  final int LEFTDIRECTION = 1;
  final int TELEOPCORRECTION = -1;
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  boolean ifDetected = false;
  boolean initialHeadingCalibrated = false;
  float initialHeading;

  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be.
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // PID constants should be tuned per robot.
  final double P_GAIN = 0.1;
  final double D_GAIN = 0.0;
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_navX = new AHRS(SPI.Port.kMXP);
    m_joystick = new Joystick(0);
    m_leftFrontMotor = new WPI_VictorSPX(22);
    m_leftBackMotor = new WPI_VictorSPX(23);
    m_leftDrive = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
    m_rightFrontMotor = new WPI_VictorSPX(20);
    m_rightBackMotor = new WPI_VictorSPX(21);
    m_rightDrive = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);
    m_driveTrain = new DifferentialDrive(m_leftDrive, m_rightDrive);
    m_driveTrain.setSafetyEnabled(false);
    m_autoTimer = new Timer();
    m_leftDrive.setInverted(true);
    m_throwMotor = new Spark(0);
    m_climbMotor = new Spark(3);
    m_throwStopLimit = new DigitalInput(9);
    m_ballServo = new Servo(1);
    
    m_driveTrain.setDeadband(0.1);
    m_navX.calibrate();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_throwMotor.set(0);
    m_autoTimer.reset();
    m_autoTimer.start();
    m_startTime = Timer.getFPGATimestamp();
    System.out.println("Auto intialized");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    System.out.println("Starting auto loop");
    double autoTime = Timer.getFPGATimestamp();

    var deltaTime = autoTime - m_startTime;

    if (deltaTime < 1.5) {
      // System.out.println("auto is working");
      m_driveTrain.arcadeDrive(.6, 0.0);
    } else {
      // System.out.println("auto is done");
      m_driveTrain.arcadeDrive(0, 0);
    }
    if (deltaTime > 1.5 && deltaTime < 2.5) {
      // m_throwMotor.set(1);
    } else {
      // m_throwMotor.set(.1);
    }

    if (m_throwStopLimit.get()) {
      // m_throwMotor.set(0);
    }

    // if (deltaTime > 3 && deltaTime < 4.5)
    // {
    // throwMotor.set(1);
    // } else
    // {
    // throwMotor.set (0);
    // }
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    m_throwMotor.set(0);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Joystick drive.
    m_driveTrain.arcadeDrive(m_joystick.getY() * kPowerPercent, m_joystick.getZ() * kPowerPercent * -0.9);

    
    // Automated 360 button :D
    // if (m_joystick.getRawButton(7) && !initialHeadingCalibrated) {
    //   m_navX.reset();
    //   initialHeading = m_navX.getYaw();
    //   initialHeadingCalibrated = true;
    // }

    // System.out.println("Initial Heading: " + initialHeading);
    System.out.println("Current Heading: " + m_navX.getYaw());
    
    if (initialHeadingCalibrated) {
      if (initialHeading - m_navX.getYaw() < 350) {
        m_driveTrain.arcadeDrive(0, 0.5);
      } else {
        m_driveTrain.stopMotor();
        initialHeadingCalibrated = false;
      }
    }

    // Throw motor.
    if (m_joystick.getTrigger()) {
    m_throwMotor.set(1);
    ifDetected = false;
    } else {
    if (!m_throwStopLimit.get()) {
    if (!ifDetected) {
    m_throwMotor.set(0.2);
    }
    } else {
    m_throwMotor.set(-0.1);
    try {
    Thread.sleep(50);
    } catch (InterruptedException e) {
    e.printStackTrace();
    }
    m_throwMotor.set(0);
    ifDetected = true;
    }
    }

    // Ball release with button 2
    if (m_joystick.getRawButton(2)) {
    m_ballServo.set(0.5);
    } else {
    m_ballServo.set(0.2);
    }

    // Climb motor
    if (m_joystick.getRawButton(5)) {
    System.out.println("Button 5 pressed.");
    m_climbMotor.set(1);

    } else if (m_joystick.getRawButton(3)) {
    m_climbMotor.set(-2);

    } else {
    m_climbMotor.set(0);
    }
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}