package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//public class TestGrabber {
    @Disabled
    @TeleOp(name = "Red_Alliance_SkyStone_Foundation", group="Gimli test")
    public class TestGrabber extends LinearOpMode {


        @Override
        public void runOpMode() throws InterruptedException {
            Gimli_hardware robot = new Gimli_hardware();
            robot.init(hardwareMap);

            waitForStart();



            while (opModeIsActive()) {
                if (gamepad1.right_trigger == 1) {
                    robot.Grabee.setPosition(0.1);
                    telemetry.addData("position:", "0.1");
                    telemetry.update();
                }

                if (gamepad1.left_trigger == 1) {
                    robot.Grabee.setPosition(0.2);
                    telemetry.addData("position:", "0.2");
                    telemetry.update();
                }
                if (gamepad1.left_bumper) {
                    robot.Grabee.setPosition(0.3);
                    telemetry.addData("position:", "0.3");
                    telemetry.update();
                }
                if (gamepad1.right_bumper) {
                    robot.Grabee.setPosition(0.4);
                    telemetry.addData("position:", "0.4");
                    telemetry.update();
                }
                if (gamepad1.b) {
                    robot.Grabee.setPosition(0.5);
                                   telemetry.addData("position:", "0.5");
                    telemetry.update();
                }
                if (gamepad1.a) {
                    robot.Grabee.setPosition(0.6);
                    telemetry.addData("position:", "0.6");
                    telemetry.update();
                }
                if (gamepad1.x) {
                    robot.Grabee.setPosition(0.7);
                    telemetry.addData("position:", "0.7");
                    telemetry.update();
                }
                if (gamepad1.y) {
                    robot.Grabee.setPosition(0.8);
                    telemetry.addData("position:", "0.8");
                    telemetry.update();
                }
                if (gamepad1.left_stick_button) {
                    robot.Grabee.setPosition(0.9);
                    telemetry.addData("position:", "0.9");
                    telemetry.update();
                }
            }
        }

        }
