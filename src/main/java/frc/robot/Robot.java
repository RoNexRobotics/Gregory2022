package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Joystick joystick;
  private DifferentialDrive driveTrain;
  private MotorControllerGroup leftDrive;
  private MotorControllerGroup rightDrive;
  private Timer autoTimer;
  private Timer m_timer;
  private double startTime;
  private ADXRS450_Gyro gyro;
  private AHRS coolGyro;
  boolean TMOnOff = false;
  double autoPower;
  double runTime;
  double cornerTime;
  double driveSpeed;
  double cornerSpeed;
  int path;
  double blocks;
  double powerPercent=.7;
  WPI_VictorSPX leftFrontMotor; 
  WPI_VictorSPX leftBackMotor; 
  WPI_VictorSPX rightFrontMotor; 
  WPI_VictorSPX rightBackMotor; 
  Spark throwMotor;
  Spark climbMotor;
  DigitalInput stopLimit;
  Servo ballServo;
  int leverPosition;
  int testNumber;
  final int RIGHTDIRECTION = -1;
  final int LEFTDIRECTION = 1;
  final int TELEOPCORRECTION = -1;
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  boolean ifDetected;

  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be.
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // PID constants should be tuned per robot.
  final double P_GAIN = 0.1;
  final double D_GAIN = 0.0;
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

  Thread m_visionThread;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

      m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
      m_chooser.addOption("My Auto", kCustomAuto);
      SmartDashboard.putData("Auto choices", m_chooser);
      joystick = new Joystick(0);
      leftFrontMotor = new WPI_VictorSPX(22);
      leftBackMotor = new WPI_VictorSPX(23);
      leftDrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
      rightFrontMotor = new WPI_VictorSPX(20);
      rightBackMotor = new WPI_VictorSPX(21);
      rightDrive = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
      driveTrain = new DifferentialDrive(leftDrive, rightDrive);
      driveTrain.setSafetyEnabled(false);
      autoTimer = new Timer();
      m_timer = new Timer();
      gyro = new ADXRS450_Gyro();
      coolGyro = new AHRS(SerialPort.Port.kUSB);
      leftDrive.setInverted(true);
      throwMotor = new Spark(0); 
      climbMotor = new Spark(9);
      stopLimit = new DigitalInput(9);
      ballServo = new Servo(1);
      ifDetected = false;

      // Could get these values use the Config text if you incorporate a text-reader.
      autoPower = .8;
      runTime = 2;
      testNumber = 1;
      path = 1;

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
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
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
      //m_autoSelected = m_chooser.getSelected();
      // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
      //System.out.println("Auto selected: " + m_autoSelected);
      throwMotor.set(0);
      autoTimer.reset();
      autoTimer.start();
      startTime = Timer.getFPGATimestamp();
      System.out.println("Auto intialized");
      
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double autoTime = Timer.getFPGATimestamp();
    System.out.println ("begining of auto");

    var deltaTime = autoTime - startTime;

    if (deltaTime < 1.5)
    {
      // System.out.println("auto is working");
      driveTrain.arcadeDrive(.6, 0.0);
    } else 
    {
      // System.out.println("auto is done");
      driveTrain.arcadeDrive(0, 0);
    } 
    if (deltaTime > 1.5 && deltaTime < 2.5)
    {
      throwMotor.set(1);
    } else
    {
      throwMotor.set(.1);
    }

    if (stopLimit.get())
    {
      throwMotor.set(0);
    }

    // if (deltaTime > 3 && deltaTime < 4.5)
    // {
    //   throwMotor.set(1);
    // } else
    // {
    //   throwMotor.set (0);
    // }


    
    // //***********X botton ball hit & Y button hammer stop */
    
    
        
  }
  
      
  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    throwMotor.set(0);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Joystick drive.
    driveTrain.arcadeDrive(joystick.getY()*powerPercent, joystick.getZ()*powerPercent*-1);

    if (joystick.getTrigger()) {
      throwMotor.set(1);
      ifDetected = false;
    } else {
      if (!stopLimit.get()) {
        if (!ifDetected) {
          throwMotor.set(-0.2);
        }
      } else {
        throwMotor.set(0.1);
        try {
          Thread.sleep(50);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        throwMotor.set(0);
        ifDetected = true;
      }
    }
    
    // if (joystick.getRawButtonPressed(2))
    // {
    //   throwMotor.set(-.15);
    //   TMOnOff = !TMOnOff;
    //   System.out.println ("Y button is " + TMOnOff);
    // } else {
    //   throwMotor.set(0);
    // }
    // if (TMOnOff == true) {%
    //    throwMotor.set(-.15);
    //  } else
    //  {
    //     throwMotor.set(0);
    //  }
    
    // }
    
    // ball release with aarow pad 
    if (joystick.getRawButton(2)) {
      ballServo.set(0.5);
    } else {
      ballServo.set(0.2);
    }
    //*****climb motor out with A button and in with B button */
    if (joystick.getRawButton(5))
    {
      climbMotor.set(1);
      
    }else if (joystick.getRawButton(3))
    {
      climbMotor.set(-2);
      
    }else{
      climbMotor.set(0);
    }
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    // stopLimit.close();
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
      rightFrontMotor.set(0);
      leftFrontMotor.set(0);
      rightBackMotor.set(0);
      leftBackMotor.set(0);

      gyro.reset();

      coolGyro.calibrate();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    // if (joystick.getTrigger()) {
    //   if (gyro.getRotation2d().getDegrees() < 90) {
    //     driveTrain.arcadeDrive(0, -0.7*powerPercent*-1);
    //   } else {
    //     driveTrain.arcadeDrive(-0.6*powerPercent, 0);
    //   }
    // } else {
    //   driveTrain.arcadeDrive(0, 0);
    // }

  // System.out.println(coolGyro.getVelocityX());
  }
}