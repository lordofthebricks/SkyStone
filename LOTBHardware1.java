package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class LOTBHardware1 {

    public DcMotor Right_Bottom;
    public DcMotor Right_Top;
    public DcMotor Left_Bottom;
    public DcMotor Left_Top;
    public DcMotor Lift;
    public DcMotor Shoulder;

    public Servo Dumper;
    public Servo LEDServo;
    public RevBlinkinLedDriver Blinky;
    public ColorSensor RightColorSensor;
    public ColorSensor LeftColorSensor;
    public DistanceSensor RightDistanceSensor;
    public DistanceSensor LeftDistanceSensor;
    public BNO055IMU IMU;

    public CRServo Conveyor;
    public Servo Elbow;
    public CRServo Intake;


    public static final double Move_Forward = -.3;
    public static final double Move_Backward = .3;

    public VuforiaLocalizer vuforia;


    public TFObjectDetector tfod;


    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    //public ColorSensor colorSensor;

    //public RevBlinkinLedDriver.BlinkinPattern InitPattern;

    /* Constructor */
    public LOTBHardware1() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        Right_Bottom = hwMap.get(DcMotor.class, "Right_Bottom");
        Right_Top = hwMap.get(DcMotor.class, "Right_Top");
        Left_Bottom = hwMap.get(DcMotor.class, "Left_Bottom");
        Left_Top = hwMap.get(DcMotor.class, "Left_Top");
        Lift = hwMap.get(DcMotor.class, "Lift");
        IMU = hwMap.get(BNO055IMU.class, "IMU");
        Shoulder = hwMap.get(DcMotor.class, "Shoulder");
        Elbow = hwMap.get(Servo.class, "Elbow");
        Intake = hwMap.get(CRServo.class, "Intake");
        LEDServo = hwMap.get(Servo.class,"LEDServo");

        Dumper = hwMap.get(Servo.class, "Dumpee");
        Blinky = hwMap.get(RevBlinkinLedDriver.class, "Blinky");

        //InitPattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
        //Blinky.setPattern(InitPattern);

        RightColorSensor = hwMap.get(ColorSensor.class, "RightColorSensor");
        LeftColorSensor = hwMap.get(ColorSensor.class,  "LeftColorSensor");
        RightDistanceSensor = hwMap.get(DistanceSensor.class, "RightColorSensor");
        LeftDistanceSensor = hwMap.get(DistanceSensor.class, "LeftColorSensor");

        Intake.setPower(0);

        Left_Top.setDirection(DcMotor.Direction.REVERSE);
        Left_Bottom.setDirection(DcMotor.Direction.REVERSE);

        Right_Bottom.setPower(0);
        Right_Top.setPower(0);
        Left_Bottom.setPower(0);
        Left_Top.setPower(0);
    }

    /*public void GoToCoordinates(int Coordinates){
           //String CRD = Double.toString(Coordinates);
           Right_Bottom.setTargetPosition(Coordinates);
           Left_Top.setTargetPosition(Coordinates);
           Left_Bottom.setTargetPosition(Coordinates);
           Right_Top.setTargetPosition(Coordinates);
    }*/

}
