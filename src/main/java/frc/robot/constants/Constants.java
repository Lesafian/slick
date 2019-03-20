package frc.robot.constants;

public class Constants {

    public static double kJoystickThreshold = 0.07;

    public static final double kElbowParallelForward = -1750;
    public static final double kElbowCargoPickup = -2200;
    public static final double kElbowParallelReverse = 1750;
    
    public static final double kElbowMaxAngleForward = -1200;
    public static final double kElbowMaxAngleReverse = 400;
    public static final double kElbowVertical = 0;

    public static final double kWristParallel = 2850;
    public static final double kWristMaxAngleForward = 2280;
    public static final double kWristMaxAngleReverse = 3200;
    public static final double kWristHatchPanelPickup = 2280;

    public static final double kLiftBottom = 0;
    public static final double kLiftLevel1HatchPanel = 10000;
    public static final double kLiftLevel2HatchPanel = 43500;
    public static final double kLiftLevel3HatchPanel= 71000;
    public static final double kLiftLevel1Cargo = 15000;
    public static final double kLiftLevel2Cargo = 40000;
    public static final double kLiftLevel3Cargo = 65000;
    public static final double kLiftBottomThreshold = 250;

    public static final double kIntakeSpeed = 0.5;
    public static final double kShootSpeed = -0.5;
    public static final double kHoldBallSpeed = 0.1;

    public int kTicksPerWheelRevHighGear = 4096;
    public int kTicksPerWheelRevLowGear = 4096;

    public static final double kWheelWidth = 6;
    public static final double kMaxVelocityHighGear = 2000;
    public static final double kMaxVelocityLowGear = 1000;
}