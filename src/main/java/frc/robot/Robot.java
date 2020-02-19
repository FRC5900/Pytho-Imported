/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  VictorSPX FrontRight = new VictorSPX(4);
  VictorSPX FrontLeft = new VictorSPX(5);
  VictorSPX RearLeft = new VictorSPX(6);
  VictorSPX RearRight = new VictorSPX(7);
  private Joystick m_joystick;
  private static double ROBOTMAX = 0.50;
  private static double ROBOTMAXSTRAFE = 0.60;
  private static double ROBOTTEST = 0.20;
  private static double STRAFETEST = 0.40;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.190918, 0.461182, 0.3479);
  private final Color kRedTarget = ColorMatch.makeColor(0.468018, 0.383301, 0.148926); 
  private final Color kYellowTarget = ColorMatch.makeColor(0.350586, 0.520508, 0.128906);
  private final Color kGreenTarget = ColorMatch.makeColor(0.229004, 0.540527, 0.230469);
  public static boolean LimelightHasValidTarget = false;
	public static double LimelightDriveCommand = 0.0;
	public static double LimelightSteerCommand = 0.0;
  final double STEER_K = 0.025;
  final double DRIVE_K = 0.03;
  final double DESIRED_TARGET_AREA = 13.0;
  final double MAX_DRIVE = 0.7;
  double tv;
  double tx;
  double ty;
  double ta;
  double steer_cmd;
  private static final int ShooterLeftID = 2;  //M1deviceID
  private static final int ShooterRightID = 3; //M2deviceID
  private CANSparkMax ShooterLeft;  //m_motor
  private CANSparkMax ShooterRight; //m_motor2
  private CANEncoder EncoderLeft;  //m_encoder
  private CANEncoder EncoderRight; //m_encoder2


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
    m_joystick = new Joystick(0);
    RearLeft.setInverted(true);
    FrontLeft.setInverted(true);
    FrontLeft.configOpenloopRamp(1);
    FrontRight.configOpenloopRamp(1);
    RearLeft.configOpenloopRamp(1);
    RearRight.configOpenloopRamp(1);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    ShooterLeft = new CANSparkMax(ShooterLeftID, MotorType.kBrushless);
    ShooterLeft.restoreFactoryDefaults();
    ShooterRight = new CANSparkMax(ShooterRightID, MotorType.kBrushless);
    ShooterRight.restoreFactoryDefaults();
    EncoderLeft = ShooterLeft.getEncoder();
    EncoderLeft.setPosition(0);

    EncoderRight = ShooterRight.getEncoder();
    EncoderRight.setPosition(0);
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
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    SmartDashboard.putNumber("Encoder Position", EncoderLeft.getPosition());
    SmartDashboard.putNumber("Encoder Velocity", EncoderLeft.getVelocity());
    SmartDashboard.putNumber("Encoder Position", EncoderRight.getPosition());
    SmartDashboard.putNumber("Encoder Velocity", EncoderRight.getVelocity());

    /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double forwardSpeed;
    double turnSpeed;
    forwardSpeed = -m_joystick.getY();
    turnSpeed = m_joystick.getX();
    
    if (m_joystick.getRawButton(2))
      Strafe();

    else{
      forwardSpeed = forwardSpeed*ROBOTMAX;
      turnSpeed = turnSpeed*ROBOTMAX;
      FrontRight.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, -turnSpeed);
      FrontLeft.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, turnSpeed);
      RearRight.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, -turnSpeed);
      RearLeft.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, turnSpeed);
    }
  }

  public void Strafe() {
    double forwardSpeed;
    forwardSpeed = -m_joystick.getX()*ROBOTMAXSTRAFE;
    FrontRight.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    FrontLeft.set(ControlMode.PercentOutput, -forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    RearRight.set(ControlMode.PercentOutput, -forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    RearLeft.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, 0);
  }

  public void StrafeTest() {
    double forwardSpeed;
    forwardSpeed = -m_joystick.getX()*STRAFETEST;
    FrontRight.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    FrontLeft.set(ControlMode.PercentOutput, -forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    RearRight.set(ControlMode.PercentOutput, -forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    RearLeft.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, 0);
  }

  public void TurnaroundTest(){
    double forwardSpeed;
    forwardSpeed = -m_joystick.getX()*ROBOTTEST;
    FrontRight.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    FrontLeft.set(ControlMode.PercentOutput, -forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    RearRight.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    RearLeft.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
  }
  public void Diagonal() {
    double forwardSpeed;
    forwardSpeed = -m_joystick.getX()*ROBOTMAXSTRAFE;
    FrontRight.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    FrontLeft.set(ControlMode.PercentOutput, -forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    RearRight.set(ControlMode.PercentOutput, -forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    RearLeft.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
  }


  public void MotorsOff() {
    FrontRight.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    FrontLeft.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    RearRight.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    RearLeft.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
  }

  public void Shooter(double speed){
    ShooterLeft.set(speed);
    ShooterRight.set(-speed);
  }

  public void Update_Limelight_Tracking() {  
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  
    if (tv < 1.0){
      LimelightHasValidTarget = false;
      LimelightDriveCommand = 0.0;
      LimelightSteerCommand = 0.0;
      return;
    }
    else {
      LimelightHasValidTarget = true;
      LimelightSteerCommand = tx * STEER_K;
      LimelightDriveCommand = ty * DRIVE_K;     
    }
  }

  @Override
  public void testPeriodic() {
    double forwardSpeed;
    double turnSpeed;
    forwardSpeed = -m_joystick.getY()*ROBOTTEST;
    turnSpeed = m_joystick.getX()*ROBOTTEST;

    if (m_joystick.getRawButton(11))
      FrontRight.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    else if (m_joystick.getRawButton(10))
      RearRight.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    else if (m_joystick.getRawButton(6))
       FrontLeft.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    else if (m_joystick.getRawButton(7))
      RearLeft.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, 0);
    else if (m_joystick.getRawButton(3))
      TurnaroundTest();
    else if (m_joystick.getRawButton(1))
      Diagonal();
    else if (m_joystick.getRawButton(2))
      StrafeTest();
    else{
      FrontRight.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, -turnSpeed);
      FrontLeft.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, turnSpeed);
      RearRight.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, -turnSpeed);
      RearLeft.set(ControlMode.PercentOutput, forwardSpeed, DemandType.ArbitraryFeedForward, turnSpeed);
    }
  }
}
