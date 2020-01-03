package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

//import org.firstinspires.ftc.teamcode.ServoTest;


@Autonomous(name="B2 - Blue Alliance - Auto - W/Sensors", group="Gimli test")
public class GimliAutoSkyStonesBlueAllianceWithSensor extends LinearOpMode {

    private static final long SLEEP_10 = 10;
    private static final long SLEEP_25 = 25;
    private static final long SLEEP_50 = 50;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AS34pyX/////AAAAGaIrZJw2gU9xsxqfbnnb+NRMmLab5C2kQ5nc5YQr0V2hS3svZx7pBKzTz+ivN1giF42Wv8bBcm9gKE69/IPfrHT/nmBsKSyBmg5x0lkmlzYZ16vcd8R8hR6+q97ki1Sn/tjGlKalYvYSL+326CcR1EiJ3C7dWYujBqTJwsqySEXcqrn4ieiQJ4lY8/+U6dBTx/OkBvXxAMgJHl+Qjz5o6TUtQX4WolbO9mOD0bZFdTwSwyzycdKDNXLUjABOcdnx2foEvJqcVPOCfHEh8FEZRHpDB5RLgIqF1kwxCfFXx7MVflrtoLet/e6l9PdmC8nIk5Oo9cC9C6hF8L79A52YouscEKTWVx9pmqPgRYDhXUux";

    static final double     COUNTS_PER_MOTOR_REV    = 1425.2;//356.3 ;    // eg: DC Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            //first hundred digits of pi fr more accuracy
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679);
    Gimli_hardware robot   = new Gimli_hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED_1 = 0.5;
    static final double     FORWARD_SPEED_2 = 1;
    static final double     TURN_SPEED    = 0.5;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override public void runOpMode() {

        robot.init(hardwareMap);

        robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Left_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Right_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's \\ "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.\\
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /*  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:


        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.


        boolean firstTime = true;

        targetsSkyStone.activate();
        waitForStart();
        //Setting the wrist and the shoulder all the way up so it doesn't mess up the program
        //robot.Shoulder.setPosition(0);
        robot.Shoulder.setPosition(1);
        sleep(SLEEP_25);
        robot.Wrist.setPosition(1);


        int strafeCount = 0;
        double firstTimeDist = 20;


        while (!isStopRequested()) {
            if(firstTime) {
                firstTimeMove(strafeCount, firstTimeDist);

            }
            moveTowardsStone(strafeCount, firstTime);

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            VuforiaTrackable trackable = allTrackables.get(0);
            //for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                //break;
            }
            else
            {
                try {
                    Thread.sleep(SLEEP_50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            //}

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.update();
                //sleep(SLEEP_25);
                //stop();

                boolean b = navigateToStone(translation);
                if (b)
                    telemetry.addData("Reached the SkyStone", "");
                else
                    telemetry.addData("Did not reach the SkyStone", "");
                telemetry.update();

                //sleep(10000);
                //stop();


                adjustRobotPosition(strafeCount);


                pickStone(strafeCount);

                transferStone();

                dropStone(strafeCount);

                //If we are picking only one stone then parkUnderBridge or continue to pick SecondStone
                parkUnderBridge();
                pickSecondStone(strafeCount);

                stop();

            }
            else {
                telemetry.addData("Visible Target", "none");
                telemetry.update();
                //Strafe to the right
                //encoderDrive(.75,.75,-.75,.75,-.75,0.75);
                encoderDriveWithoutTime(FORWARD_SPEED_2,4,-4,4,-4   );
                sleep(SLEEP_50);
                strafeCount++;
            }
            //telemetry.update();
            firstTime = false;
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }

    private boolean moveTowardsStone(int strafeCount, boolean firstTime) {

        //Sometimes the shoulder is falling so we have to reset it. Wrist up=0. Wrist down=1. Shoulder up=0. Shoulder down=1.
        //robot.Shoulder.setPosition(0);
        //telemetry.addData("The shoulder is up", "");
        //telemetry.update();
        //double finalDist = 13;
        //double currentDist = robot.FrontLookie.getDistance(DistanceUnit.INCH);
        //double deltaDist = finalDist - currentDist;
        while (robot.FrontLookie.getDistance(DistanceUnit.INCH) > 11 && firstTime) {
            telemetry.addData("Distance to Stone ", robot.FrontLookie.getDistance(DistanceUnit.INCH));
            telemetry.update();
            //sleep(2000);
            //encoderDrive(-.75,-.75,-.75,-.75,-.75,0.75);
            encoderDriveWithoutTime(-FORWARD_SPEED_1,-.75,-0.75,-0.75,-0.75);
            sleep(SLEEP_25);
        }
        //encoderDriveWithoutTime(-FORWARD_SPEED_2,deltaDist,deltaDist,deltaDist,deltaDist);
        //sleep(SLEEP_25);
        telemetry.addData("Distance to Stone Less than 11 inches ", robot.FrontLookie.getDistance(DistanceUnit.INCH));
        telemetry.addData("FirstTime Flag ", firstTime);
        telemetry.update();

        return true;
    }

    private boolean firstTimeMove(int strafeCount, double firstTimeDist) {

        //Go forward slightly so the robot doesn't touch the wall
        encoderDriveWithoutTime(FORWARD_SPEED_2, -.25, -.25, -.25, -.25);
        sleep(SLEEP_25);
        //Strafe to the left so that the right wheel comes right over the first and second tile intersection

        /*double rightDist = robot.RightLookie.getDistance(DistanceUnit.INCH);
        double finalRightDist = 14.5;
        double deltaRightDist = rightDist - finalRightDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, deltaRightDist, -deltaRightDist, deltaRightDist, -deltaRightDist);
        sleep(SLEEP_10);*/


        //double p = 18;
        //encoderDriveWithoutTime( FORWARD_SPEED_1, -p, p, -p, p );
        //sleep(SLEEP_25);
        //go forward for 24 inches with variable
        encoderDriveWithoutTime(-FORWARD_SPEED_2,-firstTimeDist,-firstTimeDist,-firstTimeDist,-firstTimeDist);
        sleep(SLEEP_25);

        return true;
    }

    private boolean dropStone(int strafeCount) {

        //The robot needs to lift the arm to put the SkyStone on the foundation
        telemetry.addData("Setting the block down and lifting the wrist", "");
        telemetry.update();
        //sleep(25);

        robot.Wrist.setPosition(1);
        sleep(100);

        /*robot.Shoulder.setPosition(0);
        sleep(200);

        robot.Wrist.setPosition(1);
        sleep(200);*/

        encoderDriveWithoutTime(FORWARD_SPEED_2, 8, -8, 8, -8);
        sleep(SLEEP_25);

        return true;
    }

    private boolean pickStone(int strafeCount) {

        //Lower the shoulder and wrist to grab the skystone
        //robot.Shoulder.setPosition(1);
        //sleep(200);
        robot.Wrist.setPosition(0);
        sleep(200);
        telemetry.addData("Found Target ", "true");
        telemetry.update();
        telemetry.addData("Going Backward","");
        telemetry.update();
        //sleep(50);

        return true;
    }


    private boolean adjustRobotPosition(int strafeCount) {

        double alignRobotDistRight = 2.5;
        double alignRobotDistLeft = 2.25;
        double alignRobotDistRightLess  = 2.5;
        double alignRobotDistLeftLess = 2.25;
        /*if(strafeCount > 3) {
            //Since the robot is not strafing correctly, we put this so the right_top wheel moves so the robot becomes straight
            //telemetry.addData("The Robot is going forward (right_top and right_bottom)", "");
            //telemetry.update();
            //sleep(10);
            encoderDriveWithoutTime(-FORWARD_SPEED_2, alignRobotDistLeft, -alignRobotDistRight, -alignRobotDistRight, alignRobotDistLeft);
            //telemetry.addData("The Robot finished going forward", "");
            //telemetry.update();
            //sleep(2000);
        }*/
        /*if (strafeCount == 3 ) {
            sleep(SLEEP_25);
            encoderDriveWithoutTime(-FORWARD_SPEED_2, alignRobotDistLeftLess, -alignRobotDistRightLess, -alignRobotDistRightLess, alignRobotDistLeftLess);
        }*/
        //stop();
        //double x = translation.get(0) / mmPerInch;
        telemetry.addData("Going Forward toward stone","");
        telemetry.update();
        //sleep(SLEEP_50);
        //go forward slightly
        double x = -8;

        if (strafeCount != 3 && strafeCount <= 4)
            x=-4.75;
        if (strafeCount == 3)
            x=-5.75;

        //encoderDrive(-0.25, x, x, x, x, 0.55);
        encoderDriveWithoutTime(-FORWARD_SPEED_1, x, x, x, x);
        sleep(SLEEP_25);

        return true;
    }

    public void encoderDrive ( double speed,
                               double Left_Bottom_Inches,
                               double Right_Bottom_Inches,
                               double Right_Top_Inches,
                               double Left_Top_Inches,
                               double timeoutS){
        int newLeftBottomTarget;
        int newRightBottomTarget;
        int newRightTopTarget;
        int newLeftTopTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLeftBottomTarget = robot.Left_Bottom.getCurrentPosition() + (int) (Left_Bottom_Inches * COUNTS_PER_INCH);
            newRightBottomTarget = robot.Right_Bottom.getCurrentPosition() + (int) (Right_Bottom_Inches * COUNTS_PER_INCH);
            newRightTopTarget = robot.Right_Top.getCurrentPosition() + (int) (Right_Top_Inches * COUNTS_PER_INCH);
            newLeftTopTarget = robot.Left_Top.getCurrentPosition() + (int) (Left_Top_Inches * COUNTS_PER_INCH);

            robot.Left_Bottom.setTargetPosition(newLeftBottomTarget);
            robot.Right_Bottom.setTargetPosition(newRightBottomTarget);
            robot.Right_Top.setTargetPosition(newRightTopTarget);
            robot.Left_Top.setTargetPosition(newLeftTopTarget);

            // Turn On RUN_TO_POSITION
            robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Left_Top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right_Top.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.Left_Bottom.setPower(Math.abs(speed));
            robot.Right_Bottom.setPower(Math.abs(speed));
            robot.Left_Top.setPower(Math.abs(speed));
            robot.Right_Top.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.Left_Bottom.isBusy() && robot.Right_Bottom.isBusy() && robot.Left_Top.isBusy() && robot.Right_Top.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d :%7d", newLeftBottomTarget, newRightBottomTarget, newLeftTopTarget, newRightTopTarget);
                //telemetry.addData("Path2", "Running at %7d :%7d", robot.Left_Bottom.getCurrentPosition(), robot.Right_Bottom.getCurrentPosition());
                robot.Left_Top.getCurrentPosition();
                robot.Right_Top.getCurrentPosition();
                //telemetry.update();
            }

            // Stop all motion;
            robot.Left_Bottom.setPower(0);
            robot.Right_Bottom.setPower(0);
            robot.Left_Top.setPower(0);
            robot.Right_Top.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Left_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }

    public void encoderDriveWithoutTime ( double speed,
                                          double Left_Bottom_Inches,
                                          double Right_Bottom_Inches,
                                          double Right_Top_Inches,
                                          double Left_Top_Inches
    ){
        int newLeftBottomTarget;
        int newRightBottomTarget;
        int newRightTopTarget;
        int newLeftTopTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLeftBottomTarget = robot.Left_Bottom.getCurrentPosition() + (int) (Left_Bottom_Inches * COUNTS_PER_INCH);
            newRightBottomTarget = robot.Right_Bottom.getCurrentPosition() + (int) (Right_Bottom_Inches * COUNTS_PER_INCH);
            newRightTopTarget = robot.Right_Top.getCurrentPosition() + (int) (Right_Top_Inches * COUNTS_PER_INCH);
            newLeftTopTarget = robot.Left_Top.getCurrentPosition() + (int) (Left_Top_Inches * COUNTS_PER_INCH);

            robot.Left_Bottom.setTargetPosition(newLeftBottomTarget);
            robot.Right_Bottom.setTargetPosition(newRightBottomTarget);
            robot.Right_Top.setTargetPosition(newRightTopTarget);
            robot.Left_Top.setTargetPosition(newLeftTopTarget);

            // Turn On RUN_TO_POSITION
            robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Left_Top.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right_Top.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.Left_Bottom.setPower(Math.abs(speed));
            robot.Right_Bottom.setPower(Math.abs(speed));
            robot.Left_Top.setPower(Math.abs(speed));
            robot.Right_Top.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (robot.Left_Bottom.isBusy() && robot.Right_Bottom.isBusy() && robot.Left_Top.isBusy() && robot.Right_Top.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d :%7d", newLeftBottomTarget, newRightBottomTarget, newLeftTopTarget, newRightTopTarget);
                //telemetry.addData("Path2", "Running at %7d :%7d", robot.Left_Bottom.getCurrentPosition(), robot.Right_Bottom.getCurrentPosition());
                robot.Left_Top.getCurrentPosition();
                robot.Right_Top.getCurrentPosition();
                //telemetry.update();
            }

            // Stop all motion;
            robot.Left_Bottom.setPower(0);
            robot.Right_Bottom.setPower(0);
            robot.Left_Top.setPower(0);
            robot.Right_Top.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Left_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right_Bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Left_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right_Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }

    public boolean navigateToStone(VectorF translation)
    {
        boolean success = true;

        double x = translation.get(0) / mmPerInch;
        double y = translation.get(1) / mmPerInch;
        double targetx = -9.7;
        double maxAllowedVariance = 0.3;
        double targety = -5.5;
        double deltax = targetx - x;
        double deltay = targety - y;
        boolean findStoneX = false;
        boolean findStoneY = false;
        int counter = 0;
        if(!findStoneX || !findStoneY) {
            if (deltax > (targetx + maxAllowedVariance) ||  deltax < (targetx - maxAllowedVariance)) {
                //move forward
                encoderDriveWithoutTime(FORWARD_SPEED_2, -deltax, -deltax, -deltax, -deltax);
            }
            else
            {
                findStoneX = true;
            }
            sleep(SLEEP_25);
            if (deltay > (targety + maxAllowedVariance) ||  deltay < (targety - maxAllowedVariance)) {
                //move sideways
                encoderDriveWithoutTime(FORWARD_SPEED_2, -deltay, deltay, -deltay, deltay);
            }
            else
                findStoneY = true;

            translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            telemetry.update();

            x = translation.get(0) / mmPerInch;
            y = translation.get(1) / mmPerInch;
            deltax = targetx - x;
            deltay = targety - y;
            counter++;
            if (counter > 5)
                return false;
        }


        return success;

    }

    /**
     * First we get the distance from the back sensor to the wall and the left
     * sensor to the wall
     * We make the robot go back so the final distance is 20 inches from the wall
     * Then we make the robot go right so the final distance is 96 inches
     *
     * @return
     */
    public boolean transferStone() {
        double backDist = robot.BackLookie.getDistance(DistanceUnit.INCH);
        double rightDist = robot.RightLookie.getDistance(DistanceUnit.INCH);

        double finalBackDist = 20;
        double deltaBackDist = backDist - finalBackDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, deltaBackDist, deltaBackDist, deltaBackDist, deltaBackDist);
        sleep(SLEEP_25);
        double finalRightDist = 86;
        double deltaRightDist = finalRightDist - rightDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, -deltaRightDist, deltaRightDist, -deltaRightDist, deltaRightDist);
        sleep(SLEEP_25);

        return true;
    }

    public boolean transferSecondStone() {
        double backDist = robot.BackLookie.getDistance(DistanceUnit.INCH);
        double rightDist = robot.RightLookie.getDistance(DistanceUnit.INCH);

        double finalBackDist = 20;
        double deltaBackDist = backDist - finalBackDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, deltaBackDist, deltaBackDist, deltaBackDist, deltaBackDist);
        sleep(SLEEP_25);
        double finalRightDist = 83;
        double deltaRightDist = rightDist - finalRightDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, deltaRightDist, -deltaRightDist, deltaRightDist, -deltaRightDist);
        sleep(SLEEP_25);

        return true;
    }

    /**
     * First we get the distance from the back sensor to the wall and the left
     * sensor to the wall
     * We make the robot go forward so the final distance is 24 inches from the wall
     * Then we make the robot go left so the final distance is 63 inches
     * @return
     */
    public boolean parkUnderBridge() {


        double backDist = robot.BackLookie.getDistance(DistanceUnit.INCH);
        double rightDist = 86;

        double finalBackDist = 16;
        double deltaBackDist = backDist - finalBackDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, deltaBackDist, deltaBackDist, deltaBackDist, deltaBackDist);
        sleep(SLEEP_10);
        robot.Wrist.setPosition(0);
        sleep(SLEEP_25);
        double finalRightDist = 53;
        telemetry.addData("This is how much the left lookie reads: ", rightDist);
        telemetry.update();
        //sleep(10000);
        //stop();
        double deltaRightDist = rightDist - finalRightDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, deltaRightDist, -deltaRightDist, deltaRightDist, -deltaRightDist);
        sleep(SLEEP_25);
        /*leftDist = robot.LeftLookie.getDistance(DistanceUnit.INCH);
        telemetry.addData("This is what the Lookie can see: %d", leftDist);
        telemetry.update();
        sleep(200);*/


        return true;
    }
    public boolean parkSecondStoneUnderBridge() {


        double backDist = robot.BackLookie.getDistance(DistanceUnit.INCH);
        double rightDist = 80;

        double finalBackDist = 24;
        double deltaBackDist = backDist - finalBackDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, deltaBackDist, deltaBackDist, deltaBackDist, deltaBackDist);
        sleep(SLEEP_10);
        robot.Wrist.setPosition(0);
        sleep(SLEEP_25);
        double finalRightDist = 63;
        telemetry.addData("This is how much the left lookie reads: ", rightDist);
        telemetry.update();
        //sleep(10000);
        //stop();
        double deltaRightDist = rightDist - finalRightDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, deltaRightDist, -deltaRightDist, deltaRightDist, -deltaRightDist);
        sleep(SLEEP_25);
        /*leftDist = robot.LeftLookie.getDistance(DistanceUnit.INCH);
        telemetry.addData("This is what the Lookie can see: %d", leftDist);
        telemetry.update();
        sleep(200);*/


        return true;
    }
    public boolean pickSecondStone(int strafeCount) {

        //encoderDriveWithoutTime(FORWARD_SPEED_2, 8, 8, 8, 8);
        //sleep(SLEEP_25);
        //encoderDriveWithoutTime(FORWARD_SPEED_2, -10, 10, -10, 10);
        //sleep(SLEEP_25);
        //adjustRobotPosition(strafeCount);

        double backDist = robot.BackLookie.getDistance(DistanceUnit.INCH);
        double rightDist = robot.RightLookie.getDistance(DistanceUnit.INCH);
        telemetry.addData("This is what the Lookie can see: %d", rightDist);
        telemetry.update();
        sleep(200);
        double finalRightDist = 28;

        double finalBackDist = 24;
        double deltaBackDist = backDist - finalBackDist;
        /*encoderDriveWithoutTime(FORWARD_SPEED_2, deltaBackDist, deltaBackDist, deltaBackDist, deltaBackDist);
        sleep(SLEEP_50);
        robot.Wrist.setPosition(0);
        sleep(100);
        */
        // Blue Alliance: 5th stone is 26 in.
        //Blue Alliance: 6th stone is 33 in.
        //Second SkyStone is in 5th position
        if(strafeCount > 3 && strafeCount <= 5)
        {
            //Basically the second skystone is in the first position
            finalRightDist = 26;
            //stop();
        }
        //Second SkyStone is in 1st so we cannot pick, instead pick stone 5
        else if(strafeCount == 0)
        {
            finalRightDist = 26;
        }
        //Second SkyStone is in 6th position
        else
        {
            finalRightDist = 33;
        }
        double deltaRightDist = rightDist - finalRightDist;
        encoderDriveWithoutTime(FORWARD_SPEED_2, deltaRightDist, -deltaRightDist, deltaRightDist, -deltaRightDist);
        robot.Wrist.setPosition(1);
        sleep(200);
        //Drive forward 4 inches
        encoderDriveWithoutTime(-FORWARD_SPEED_1, -14, -14, -14, -14);
        sleep(SLEEP_25);
        //moveTowardsStone(strafeCount,true);

        pickStone(strafeCount);

        transferSecondStone();

        dropStone(strafeCount);

        parkSecondStoneUnderBridge();

        return true;
    }

}

