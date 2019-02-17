
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CAN;
import frc.robot.constants.Constants;
import frc.robot.mechanisms.Elbow;
import frc.robot.mechanisms.Wrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lift;

public class Robot extends TimedRobot {

  private Xbox controller;

  private Compressor compressor;
  private DriveTrain drive;
  private Climber climber;
  private Elbow elbow;
  private Wrist wrist;
  private Arm arm;
  private Lift lift;

  private VictorSPX liftSlave, climberSlave;

  private byte position = 0;
  private double feedFwd = 0;

  private boolean xPressed = false;
  private boolean yPressed = false;
  private boolean bPressed = false;
  private boolean bumperPressed = false;
  private boolean back = true;

  private boolean hatchPanel = true;
  private boolean ground = false;
  private Mode mode = Mode.CARGO;

  @Override
  public void robotInit() {

    controller = new Xbox(0);
    controller.setThreshold(Constants.kJoystickThreshold);

    drive = new DriveTrain(CAN.kFrontLeft, CAN.kMidLeft, CAN.kBackLeft, CAN.kFrontRight, CAN.kMidRight, CAN.kBackRight);

    elbow = new Elbow(CAN.kElbow);
    wrist = new Wrist(CAN.kWrist);
    arm = new Arm(elbow, wrist);

    lift = new Lift(CAN.kLiftMaster);
    liftSlave = new VictorSPX(CAN.kLiftSlave);
    liftSlave.setNeutralMode(NeutralMode.Brake);
    liftSlave.setInverted(true);
    liftSlave.follow(lift);

    climber = new Climber(CAN.kClimberMaster);
    climberSlave = new VictorSPX(CAN.kClimberSlave);
    climberSlave.setNeutralMode(NeutralMode.Brake);
    climberSlave.follow(climber);

    compressor = new Compressor();
    compressor.setClosedLoopControl(true);
    compressor.start();

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    display();

    /*
    Using this method as a secondary teleop for mechanism testing
      code in here is irrelevant
    */

    if (controller.getBumper(Hand.kLeft)) {
      if (controller.getBButton()) {
        wrist.moveTo(Constants.kWristParallel);
        elbow.moveTo(Constants.kElbowParallelForward);
      } else if (controller.getXButton()) {
        elbow.moveTo(Constants.kElbowVertical);
        wrist.moveTo(Constants.kWristParallel);
      } else {
        elbow.stop();
        wrist.stop();
      }
      System.out
          .println("Wrist out: " + wrist.getMotorOutputPercent() + "- Elbow out: " + elbow.getMotorOutputPercent());
    } else if (controller.getBumper(Hand.kRight)) {
      if (controller.yAxisExceedsThreshold(Hand.kRight)) {
        elbow.set(ControlMode.PercentOutput, controller.getY(Hand.kRight) * .3);
      } else {
        elbow.stop();
      }
    } else {
      wrist.stop();
      elbow.stop();
    }

    if (controller.getYButton()) {
      arm.extend();
    } else if (controller.getXButton()) {
      arm.retract();
    }

  }

  @Override
  public void teleopInit() {
    drive.resetGyro();
  }

  @Override
  public void teleopPeriodic() {
    display();
    if (controller.joysticksExceedThreshold()) {
      drive.arcadeDrive(controller.getY(Hand.kLeft), -controller.getX(Hand.kRight), controller.getThreshold());
    } else {
      drive.stopDriving();
    }


    // toggle hatch panel mode; true = hatch panel, false = cargo
    if (controller.getXButton()) {
      if (!xPressed) {
        hatchPanel = !hatchPanel;
        xPressed = true;
      }
    } else {
      xPressed = false;
    }
    if (hatchPanel) {
      mode = Mode.HATCH_PANEL;
    } else {
      mode = mode.CARGO;
    }


    // toggle floor pickup or load station pickup; applies only to hatch panels
    if (controller.getYButton()) {
      if (!yPressed) {
        ground = !ground;
        xPressed = true;
      }
    } else {
      yPressed = false;
    }


    /* controls the positions of the subsystems
    
    Bottom = Climb / Defense
    Pickup = Cargo / Hatch Panel Floor / Hatch Panel Loading Station
    Level 1 = Level 1 Placement
    Level 2 = Level 2 Placement
    Level 3 = Level 3 Placement

    */
    if (controller.getBumper(Hand.kRight)) {
      if (!bumperPressed && position < 4) {
        position++;
        bumperPressed = true;
      }
    } else if (controller.getBumper(Hand.kLeft)) {
      if (!bumperPressed && position > 1) {
        position--;
        bumperPressed = true;
      }
    } else if (controller.getBumper(Hand.kLeft) && controller.getPOV() == 90) {
      if (position == 1) {
        position = 0;
      }
    } else {
      bumperPressed = false;
    }

    if (controller.getBButton()) {
      if (bPressed == false) {
        back = !back;
        bPressed = true;
      }
    } else {
      bPressed = false;
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
    lift.moveTo(mode, ground, position);
    arm.moveTo(mode, back, ground, lift.getSelectedSensorPosition(), position);
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
    display();
    if (controller.yAxisExceedsThreshold(Hand.kLeft)) {
      elbow.set(ControlMode.PercentOutput, controller.getY(Hand.kLeft) * .3);
    } else {
      elbow.stop();
    }

    if (controller.yAxisExceedsThreshold(Hand.kRight)) {
      wrist.set(ControlMode.PercentOutput, controller.getY(Hand.kRight) * .3);
    } else {
      wrist.stop();
    }

    if (controller.triggerExceedsThreshold(Hand.kLeft)) {
      climber.climb(-.7);
    } else if (controller.triggerExceedsThreshold(Hand.kRight)) {
      climber.climb(.7);
    } else {
      climber.stop();
    }

  }

  public void display() {
    SmartDashboard.putNumber("Position", position);
    SmartDashboard.putBoolean("Back", back);

    elbow.display();
    wrist.display();
    drive.display();
    lift.display();
  }

}
