package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Gimli Driver Control")

public class GimliteleopDriverControl extends LinearOpMode {

    double posWrist = 0.02;
    //Add double posShoulder Variable for the shoulder movement

    Gimli_hardware robot = new Gimli_hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
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
            while (gamepad1.left_stick_x == -1) {
                robot.Right_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Left_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Right_Top.setPower(-.9);
                robot.Right_Bottom.setPower(.3);
                robot.Left_Bottom.setPower(-.3);
                robot.Left_Top.setPower(.9);


            }
            //Fix Auto Stop!!!
            if (gamepad1.y) {
                telemetry.addData("P0tat3r", "robot.Wrist.getPosition()");

            }
            if (gamepad1.right_bumper) {
                robot.Shoulder.setPosition(0);
            } else if (gamepad1.left_bumper) {
                robot.Wrist.setPosition(1);
            }
            if (gamepad1.left_trigger == 1) {
                robot.Wrist.setPosition(0);
            } else if (gamepad1.right_trigger == 1) {
                robot.Shoulder.setPosition(1);
            }
            if (gamepad1.y) {
                robot.Slider.setPower(.4);
            } else if (gamepad1.a) {
                robot.Slider.setPower(-.4);

            }

            if (gamepad1.x) {
                robot.Slider.setPower(0);
            }
        }
    }
}