package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CAN;
import frc.robot.constants.Constants;
import frc.robot.mechanisms.Elbow;
import frc.robot.mechanisms.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lift;

public class Robot extends TimedRobot {

  private Xbox tyler;
  private PS4 jarm;

  private Compressor compressor;
  private DriveTrain drive;
  private Climber climber;
  private Elbow elbow;
  private Arm arm;
  private Lift lift;
  private Intake intake;
  private VictorSPX liftSlave, climberSlave;

  private byte position = 0;
  private double feedFwd = 0;

  private double turn = 0;

  private boolean climbStateFront = false;
  private boolean climbStateBack = false;

  private NetworkTableEntry cam;

  private boolean back = false;
  private boolean ground = false;
  private Mode mode = Mode.HATCH_PANEL;

  @Override
  public void robotInit() {

    jarm = new PS4(0);
    jarm.setThreshold(Constants.kJoystickThreshold);

    tyler = new Xbox(1);
    tyler.setThreshold(Constants.kJoystickThreshold);

    drive = new DriveTrain(CAN.kFrontLeft, CAN.kMidLeft, CAN.kBackLeft, CAN.kFrontRight, CAN.kMidRight, CAN.kBackRight);

    elbow = new Elbow(CAN.kElbow);
    arm = new Arm(elbow);
    intake = new Intake(CAN.kIntakeLeft, CAN.kIntakeRight);


    lift = new Lift(CAN.kLiftMaster);
    liftSlave = new VictorSPX(CAN.kLiftSlave);
    liftSlave.setNeutralMode(NeutralMode.Brake);
    liftSlave.setInverted(false);
    liftSlave.follow(lift);

    climber = new Climber(CAN.kClimberMaster);
    climberSlave = new VictorSPX(CAN.kClimberSlave);
    climberSlave.setNeutralMode(NeutralMode.Brake);
    climberSlave.follow(climber);

    compressor = new Compressor(CAN.kPCM_24);
    compressor.setClosedLoopControl(true);
    compressor.start();

    cam = NetworkTableInstance.getDefault().getTable("SlickVision").getEntry("Cam");

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {
    compressor.start();
  }

  // ctrl + shift + p
  @Override
  public void autonomousPeriodic() {
    display();

    if (jarm.getBumper(Hand.kLeft)) {
      if (jarm.yAxisExceedsThreshold(Hand.kLeft)) {
        elbow.set(ControlMode.PercentOutput, jarm.getY(Hand.kLeft) * .6);
      } else {
        elbow.stop();
      }
    } else {
      elbow.stop();
    }
  }

  @Override
  public void teleopInit() {
   // drive.resetGyro();
    compressor.start();
  }

  @Override
  public void teleopPeriodic() {

    display();

    /*
     * 
     * JARMS CONTROLS
     * 
     */

    // use vision if button is pressed
    
    if (jarm.getXButton()) {
      turn = drive.getAssistedTurnOnce();
    } else {
      drive.resetAssistance();
      turn = -jarm.getRawAxis(2);
    }

    // drive train code
    if (jarm.joysticksExceedThreshold()) {
      drive.arcadeDrive(jarm.getY(Hand.kLeft), turn, jarm.getThreshold());
    } else {
      drive.stopDriving();
    }

    // shift drive train
    if (jarm.getStickButton(Hand.kLeft)) {
      drive.lowGear();
    } else if (jarm.getStickButton(Hand.kRight)) {
      drive.highGear();
    }
    
    // game piece mode
    if (jarm.getBumper(Hand.kLeft)) {
      mode = Mode.HATCH_PANEL;
    } else if (jarm.getBumper(Hand.kRight)) {
      mode = Mode.CARGO;
    }

    // intake
    if (jarm.getRawButton(8)) {
      if (mode == Mode.HATCH_PANEL) {
        intake.autoHatchIntake();
      } else {
        intake.autoCargoIntake(0.8);
      }
    } else if (jarm.getRawButton(7)) {
      if (mode == Mode.HATCH_PANEL) {
        intake.close();
      } else {
        intake.shoot(0.8);
        intake.close();
      }
    } else {
      intake.stop();
    }
/*
    if (jarm.getSharePressed()) {
      climbStateBack = !climbStateBack;
    } 
    if (jarm.getOptionsPressed()) {
      climbStateFront = !climbStateFront;
    }
    if (climbStateBack) {
      climber.setBack(DoubleSolenoid.Value.kForward);
    } else {
      climber.setBack(DoubleSolenoid.Value.kReverse);
    }

    if (climbStateFront) {
      climber.setFront(DoubleSolenoid.Value.kForward);
    } else {
      climber.setFront(DoubleSolenoid.Value.kReverse);
    }

*/
    if (tyler.getAButtonPressed()) {
      back = !back;
    }

    if (back) {
      cam.setValue(0);
    } else {
      cam.setValue(1);
    }

    if (tyler.getBumper(Hand.kRight)) {
      position = 3;
    } else if (tyler.getBumper(Hand.kLeft)) {
      position = 4;
    } else if (tyler.triggerExceedsThreshold(Hand.kRight)) {
      position = 1;
    } else if (tyler.triggerExceedsThreshold(Hand.kLeft)) {
      position = 2;
    } else if (tyler.getBButton()) {
      position = 0;
    }

    if (position == 0) {
      move(Position.BOTTOM);
    } else if (position == 1) {
      move(Position.PICKUP);
    } else if (position == 2) {
      move(Position.LEVEL_1);
    } else if (position == 3) {
      move(Position.LEVEL_2);
    } else if (position == 4) {
      move(Position.LEVEL_3);
    }

  }

  private void move(Position position) {
    lift.moveTo(mode, position);
    arm.moveTo(mode, back, position);
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
    display();
    if (jarm.yAxisExceedsThreshold(Hand.kLeft)) {
      elbow.set(ControlMode.PercentOutput, jarm.getY(Hand.kLeft) * .3);
    } else {
      elbow.stop();
    }

    if (jarm.yAxisExceedsThreshold(Hand.kRight)) {
      // wrist.set(ControlMode.PercentOutput, controller.getY(Hand.kRight) * .3);
    } else {
      // rist.stop();
    }

    if (jarm.triggerExceedsThreshold(Hand.kLeft)) {
      climber.climb(-.7);
    } else if (jarm.triggerExceedsThreshold(Hand.kRight)) {
      climber.climb(.7);
    } else {
      climber.stop();
    }

  }

  public void display() {
    SmartDashboard.putBoolean("Back", back);
    SmartDashboard.putNumber("Lift Slave Motor Voltage", liftSlave.getMotorOutputVoltage());
   // drive.display();
   elbow.display();
   intake.display();
  }

}