package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GimliTeleopDriverControl")

public class GimliteleopDriverControl extends LinearOpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1425.2;//356.3 ;    // eg: DC Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            //first hundred digits of pi fr more accuracy
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679);
    Gimli_hardware robot   = new Gimli_hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    double posWrist = 0.02;
    //Add double posShoulder Variable for the shoulder movement



    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        int Timesused = 0;
        int FirstTime = 0;
        waitForStart();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //


        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */


        while (opModeIsActive()) {


            robot.Right_Top.setPower(gamepad1.right_stick_y);
            robot.Right_Bottom.setPower(gamepad1.right_stick_y);
            robot.Left_Top.setPower(gamepad1.left_stick_y);
            robot.Left_Bottom.setPower(gamepad1.left_stick_y);
            // robot.Conveyor.setPower(.5);
            /*while (gamepad1.right_stick_x == 1){
                robot.Right_Top.setPower(.3);
                robot.Right_Bottom.setPower(.3);
            }

            while (gamepad1.left_stick_x == 1){
                robot.Left_Top.setPower(.3);
                robot.Left_Bottom.setPower(.3);
            }

            while (gamepad1.left_stick_x == -1){
                robot.Left_Bottom.setPower(.3);
                robot.Left_Top.setPower(.3);
            }

            while (gamepad1.right_stick_x == -1){
                robot.Right_Top.setPower(.3);
                robot.Right_Bottom.setPower(.3);
            }

            while (gamepad1.left_stick_button){
                robot.Left_Bottom.setPower(-.3);
                robot.Left_Top.setPower(.3);
                robot.Right_Top.setPower(-.3);
                robot.Right_Bottom.setPower(.3);
            }

            while (gamepad1.right_stick_button){
                robot.Left_Bottom.setPower(.3);
                robot.Left_Top.setPower(-.3);
                robot.Right_Bottom.setPower(-.3);
                robot.Right_Top.setPower(.3);
            }*/

            //
            while (gamepad1.right_stick_x == -1 && gamepad1.right_stick_y == 1) {
                robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Top.setPower(.6);
                robot.Right_Bottom.setPower(.6);
            }
            while (gamepad1.right_stick_y == 1 && gamepad1.right_stick_x == -1) {
                robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Top.setPower(-.6);
                robot.Right_Bottom.setPower(-.6);
            }
            while (gamepad1.left_stick_x == -1 && gamepad1.right_stick_y == 1) {
                robot.Right_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Bottom.setPower(.6);
                robot.Right_Top.setPower(.6);
            }
            while (gamepad1.left_stick_x == 1 && gamepad1.left_stick_y == 1) {
                robot.Right_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Bottom.setPower(-.6);
                robot.Right_Top.setPower(-.6);
            }
            //This is the Strafe
            while (gamepad1.right_stick_x == 1) {
                robot.Right_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Right_Top.setPower(.6);
                robot.Right_Bottom.setPower(-.6);
                robot.Left_Bottom.setPower(.6);
                robot.Left_Top.setPower(-.6);
            }
            //This is the Strafe
            // --Speeds changed by Coach 12/13/19
            while (gamepad1.left_stick_x == -1) {
                robot.Right_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Right_Top.setPower(-.6);
                robot.Right_Bottom.setPower(.6);
                robot.Left_Bottom.setPower(-.6);
                robot.Left_Top.setPower(.6);


            }
            //Fix Auto Stop!!!

            //Trigger/bumper actions changed by Coach 12/13/19
            if (gamepad1.right_bumper) {
                telemetry.addData("Right_Bumper should be working", "");
                telemetry.update();
                //sleep(1000);
                robot.Shoulder.setPosition(0);
            } else if (gamepad1.left_bumper) {
                telemetry.addData("Left_Bumper should be working", "");
                telemetry.update();
                //sleep(1000);
                robot.Wrist.setPosition(1);
            }
            if (gamepad1.left_trigger == 1) {
                telemetry.addData("Left_Trigger should be working", "");
                telemetry.update();
                //sleep(1000);
                robot.Wrist.setPosition(0);

            } else if (gamepad1.right_trigger == 1) {
                telemetry.addData("Right_Trigger should be working", "");
                telemetry.update();
                //sleep(1000);
                robot.Shoulder.setPosition(1);
            }
            if (gamepad1.y) {
                telemetry.addData("Y should be working", "");
                telemetry.update();
                //sleep(1000);
                if (FirstTime == 0){
                    encoderSlider(.3,1,4);
                    FirstTime ++;
                    Timesused ++;
                }
               else {
                   encoderSlider(.3,4,10);
                   Timesused += 7;
                }

            } else if (gamepad1.a) {
                telemetry.addData("A should be working", "");
                telemetry.update();
                //sleep(1000);

                if (Timesused > 1) {
                    Timesused -= 7;
                    encoderSlider(.3, -4, 10);
                }
                else
                {
                    Timesused -= 1;
                    encoderSlider(.3 , -1, 2);
                }

            }

            if (gamepad1.x) {
                telemetry.addData("X should be working", "");
                telemetry.update();
                //sleep(1000);
                encoderSlider(1,-Timesused,12);
                Timesused = 0;
            }

            if (gamepad1.b) {
                telemetry.addData("B should be working", "");
                telemetry.update();
                //sleep(1000);
                FirstTime = 0;
                Timesused = 0;
            }
            if (gamepad1.dpad_down) {
                telemetry.addData("Dumpee should be working", "");
                telemetry.update();
                //sleep(1000);
                robot.Dumpee.setPosition(1);
            }
            if (gamepad1.dpad_up) {
                telemetry.addData("Dumpee should be working", "");
                telemetry.update();
                //sleep(1000);
                robot.Dumpee.setPosition(.3);
            }

        }
    }
    public void encoderSlider ( double speed,
                               double SliderInches,

                               double timeoutS){
        int newLeftBottomTarget;
        int newRightBottomTarget;
        int newRightTopTarget;
        int newLeftTopTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLeftBottomTarget = robot.Slider.getCurrentPosition() + (int) (SliderInches * COUNTS_PER_INCH);
            //newRightBottomTarget = robot.Right_Bottom.getCurrentPosition() + (int) (Right_Bottom_Inches * COUNTS_PER_INCH);
            //newRightTopTarget = robot.Right_Top.getCurrentPosition() + (int) (Right_Top_Inches * COUNTS_PER_INCH);
            //newLeftTopTarget = robot.Left_Top.getCurrentPosition() + (int) (Left_Top_Inches * COUNTS_PER_INCH);

            robot.Slider.setTargetPosition(newLeftBottomTarget);
            //robot.Right_Bottom.setTargetPosition(newRightBottomTarget);
            //robot.Right_Top.setTargetPosition(newRightTopTarget);
            //robot.Left_Top.setTargetPosition(newLeftTopTarget);

            // Turn On RUN_TO_POSITION
            robot.Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.Left_Top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.Right_Top.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.Slider.setPower(Math.abs(speed));
            //robot.Right_Bottom.setPower(Math.abs(speed));
            //robot.Left_Top.setPower(Math.abs(speed));
            //robot.Right_Top.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.Slider.isBusy() ))//&& robot.Right_Bottom.isBusy() && robot.Left_Top.isBusy() && robot.Right_Top.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d :%7d", newLeftBottomTarget, newRightBottomTarget, newLeftTopTarget, newRightTopTarget);
                //telemetry.addData("Path2", "Running at %7d :%7d", robot.Left_Bottom.getCurrentPosition(), robot.Right_Bottom.getCurrentPosition());
                robot.Slider.getCurrentPosition();
                //robot.Right_Top.getCurrentPosition();
                //telemetry.update();
            }

            // Stop all motion;
            robot.Slider.setPower(0);
            //robot.Right_Bottom.setPower(0);
            //robot.Left_Top.setPower(0);
            //robot.Right_Top.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.Left_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.Right_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }
