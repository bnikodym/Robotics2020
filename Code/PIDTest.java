/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="PIDTest", group="A")
@Disabled
public class PIDTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightRear = null;
    private DcMotor rightFront = null;
    private DcMotor launcherMotor = null;
    private Servo wobbleGrabber = null;
    private DcMotor rampDrive1 = null;
    private DcMotor rampDrive2 = null;
    private DcMotor collectionMotor = null;
    private Servo grabberArm = null;
    private CRServo knockydowny =null;


    //Ring determination program
    OpenCvWebcam webcam;
    RingDeterminationPipelinev2 pipeline = new RingDeterminationPipelinev2();


    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public void stopRampDrive()
    {
        rampDrive1.setPower(0);
        rampDrive2.setPower(0);
    }

    public void stopDriveMotors()
    {
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }

    public void stopAndResetEncodersAll()
    {
        //reset all encoders
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //all wheels run using encoders
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void runToPositionAll()
    {
        leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    static final double FORWARD_SPEED = 1;
    static final double SIDEWAYS_SPEED = 0.8;



    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.update();
        // Don't burn CPU cycles busy-looping in this sample
        sleep(50);

        telemetry.addData("Status: ", "Initializing Hardware");
        telemetry.update();

        // Initialize the hardware variables.
        leftFront = hardwareMap.get(DcMotorEx.class, "left_rear");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_rear");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_front");
        launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor");
        wobbleGrabber = hardwareMap.get(Servo.class, "wobble_grabber");
        grabberArm = hardwareMap.get(Servo.class, "grabber_Arm");
        rampDrive1 = hardwareMap.get(DcMotor.class, "ramp_drive1");
        rampDrive2 = hardwareMap.get(DcMotor.class, "ramp_drive2");
        collectionMotor = hardwareMap.get(DcMotor.class, "collection_motor");
        knockydowny = hardwareMap.get(CRServo.class, "knocky_downy");


        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);


        telemetry.addData("Status: ", "Initializing IMU");
        telemetry.update();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Get heading while on starting line
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        telemetry.addData("First Angle: ", startHeading);
        telemetry.addData("Average:", pipeline.getAnalysis());
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //set zero power to brake
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //reset all encoders
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //all wheels run using encoders
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Init:", "Complete");
        telemetry.addData("Path0", "Starting at %7d :%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
        telemetry.addData("First Angle: ", startHeading);
        telemetry.addData("Average:", pipeline.getAnalysis());
        telemetry.update();

        waitForStart();
        runtime.reset();


        //beginning of Autonomous Program
        sleep(600);
        int avg1 = pipeline.getAnalysis();
        telemetry.addData("Average:", pipeline.getAnalysis());
        telemetry.update();
        sleep(450);
        telemetry.addData("Average:", pipeline.getAnalysis());
        telemetry.update();
        sleep(400);

        knockydowny.setPower(-.6);

        //________________________________________________________________________________________________________________________________________________________________________________________
        if (avg1 > 141) // go to far zone
        {
            knockydowny.setPower(-.6);
            grabberArm.setPosition(1);
            wobbleGrabber.setPosition(0);

            //drive right
            leftFront.setTargetPosition(1120);
            rightFront.setTargetPosition(-1120);
            leftRear.setTargetPosition(-1120);
            rightRear.setTargetPosition(1120);

            runToPositionAll();

            leftFront.setPower(SIDEWAYS_SPEED);
            rightFront.setPower(SIDEWAYS_SPEED);
            leftRear.setPower(SIDEWAYS_SPEED);
            rightRear.setPower(SIDEWAYS_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();

            sleep(400);

            //drive forward to far goal
            leftFront.setTargetPosition(9200);
            rightFront.setTargetPosition(9200);
            leftRear.setTargetPosition(9200);
            rightRear.setTargetPosition(9200);

            runToPositionAll();

            leftFront.setPower(FORWARD_SPEED);
            rightFront.setPower(FORWARD_SPEED);
            leftRear.setPower(FORWARD_SPEED);
            rightRear.setPower(FORWARD_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path", "Complete, Turning");
            telemetry.update();

            //turn right to drop wobble
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;
            double realHeading = currentHeading;
            if (currentHeading < 0 || currentHeading == -0) {
                realHeading = 360 + currentHeading;
            } else {
                realHeading = currentHeading;
            }
            realHeading = Range.clip(realHeading, 0, 360);
            telemetry.addData("before turn Angle: ", realHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (realHeading > 230 || realHeading < 20) {
                leftFront.setPower(.4);
                rightFront.setPower(-.4);
                leftRear.setPower(.4);
                rightRear.setPower(-.4);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                if (currentHeading < 0 || currentHeading == -0) {
                    realHeading = 360 + currentHeading;
                } else {
                    realHeading = currentHeading;
                }
                realHeading = Range.clip(realHeading, 0, 360);

                telemetry.addData("angle turn ", realHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();

            sleep(300);

            if (realHeading < 270 && realHeading > 200) {
                grabberArm.setPosition(0);
                sleep(450);
                wobbleGrabber.setPosition(1);
            }
            sleep(800);

            grabberArm.setPosition(1);


            //back away from dropped wobble
            leftFront.setTargetPosition(300);
            rightFront.setTargetPosition(-300);
            leftRear.setTargetPosition(-300);
            rightRear.setTargetPosition(300);


            runToPositionAll();

            leftFront.setPower(SIDEWAYS_SPEED);
            rightFront.setPower(SIDEWAYS_SPEED);
            leftRear.setPower(SIDEWAYS_SPEED);
            rightRear.setPower(SIDEWAYS_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();

            //realign to face the north wall
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            realHeading = currentHeading;
            if (currentHeading < 0 || currentHeading == -0) {
                realHeading = 360 + currentHeading;
            } else {
                realHeading = currentHeading;
            }
            telemetry.addData("before turn Angle: ", realHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (realHeading < 360 && realHeading > 5) {
                leftFront.setPower(-.35);
                rightFront.setPower(.35);
                leftRear.setPower(-.35);
                rightRear.setPower(.35);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                if (currentHeading < 0 || currentHeading == -0) {
                    realHeading = 360 + currentHeading;
                } else {
                    realHeading = currentHeading;
                }
                realHeading = Range.clip(realHeading, 0, 360);

                telemetry.addData("angle turn ", realHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();

            //backup across the shooting line
            leftFront.setTargetPosition(-4200);
            rightFront.setTargetPosition(-4200);
            leftRear.setTargetPosition(-4200);
            rightRear.setTargetPosition(-4200);

            runToPositionAll();

            leftFront.setPower(FORWARD_SPEED);
            rightFront.setPower(FORWARD_SPEED);
            leftRear.setPower(FORWARD_SPEED);
            rightRear.setPower(FORWARD_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();


            sleep(400);

            //drive left to shoot
            leftFront.setTargetPosition(-2800);
            rightFront.setTargetPosition(2800);
            leftRear.setTargetPosition(2800);
            rightRear.setTargetPosition(-2800);

            runToPositionAll();

            leftFront.setPower(SIDEWAYS_SPEED);
            rightFront.setPower(SIDEWAYS_SPEED);
            leftRear.setPower(SIDEWAYS_SPEED);
            rightRear.setPower(SIDEWAYS_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();

            //turn to first power shot
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            telemetry.addData("before turn Angle: ", currentHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (currentHeading > -20 && currentHeading < 16)  {
                leftFront.setPower(-.15);
                rightFront.setPower(.15);
                leftRear.setPower(-.15);
                rightRear.setPower(.15);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                telemetry.addData("angle turn ", currentHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();


            //fire 1st ring at power shot
            rampDrive1.setPower(-1);
            rampDrive2.setPower(-1);
            sleep(100);
            stopRampDrive();
            launcherMotor.setPower(.8);
            sleep(1300);
            rampDrive1.setPower(1);
            rampDrive2.setPower(1);
            sleep(600);
            stopRampDrive();
            sleep(200);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            telemetry.addData("before turn Angle: ", currentHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (currentHeading > 0 && currentHeading < 24)  {
                leftFront.setPower(-.15);
                rightFront.setPower(.15);
                leftRear.setPower(-.15);
                rightRear.setPower(.15);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                telemetry.addData("angle turn ", currentHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();


            //fire 2nd ring at power shot
            rampDrive1.setPower(1);
            rampDrive2.setPower(1);
            sleep(750);
            stopRampDrive();
            sleep(400);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            telemetry.addData("before turn Angle: ", currentHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (currentHeading > 0 && currentHeading < 30)  {
                leftFront.setPower(-.15);
                rightFront.setPower(.15);
                leftRear.setPower(-.15);
                rightRear.setPower(.15);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                realHeading = Range.clip(realHeading, 0, 360);
                telemetry.addData("angle turn ", currentHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();


            //fire 3rd ring at power shot
            rampDrive1.setPower(1);
            rampDrive2.setPower(1);
            sleep(1200);
            stopRampDrive();
            launcherMotor.setPower(0);
            sleep(400);

            leftFront.setTargetPosition(2000);
            rightFront.setTargetPosition(2000);
            leftRear.setTargetPosition(2000);
            rightRear.setTargetPosition(2000);

            runToPositionAll();

            leftFront.setPower(FORWARD_SPEED);
            rightFront.setPower(FORWARD_SPEED);
            leftRear.setPower(FORWARD_SPEED);
            rightRear.setPower(FORWARD_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();


            sleep(30000);
        }
        //_________________________________________________________________________________________________________________________________________________________________________________________
        else if (avg1 >= 131) // go to middle zone
        {
            knockydowny.setPower(-.6);
            grabberArm.setPosition(1);
            wobbleGrabber.setPosition(0);

            //drive right
            leftFront.setTargetPosition(900);
            rightFront.setTargetPosition(-900);
            leftRear.setTargetPosition(-900);
            rightRear.setTargetPosition(900);

            runToPositionAll();

            leftFront.setPower(SIDEWAYS_SPEED);
            rightFront.setPower(SIDEWAYS_SPEED);
            leftRear.setPower(SIDEWAYS_SPEED);
            rightRear.setPower(SIDEWAYS_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();

            sleep(400);

            //drive forward to far goal
            leftFront.setTargetPosition(9500);
            rightFront.setTargetPosition(9500);
            leftRear.setTargetPosition(9500);
            rightRear.setTargetPosition(9500);

            runToPositionAll();

            leftFront.setPower(FORWARD_SPEED);
            rightFront.setPower(FORWARD_SPEED);
            leftRear.setPower(FORWARD_SPEED);
            rightRear.setPower(FORWARD_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path", "Complete, Turning");
            telemetry.update();

            grabberArm.setPosition(0);
            sleep(650);
            wobbleGrabber.setPosition(1);
            sleep(800);
            grabberArm.setPosition(1);
            sleep(400);


            //back away from dropped wobble
            leftFront.setTargetPosition(300);
            rightFront.setTargetPosition(-300);
            leftRear.setTargetPosition(-300);
            rightRear.setTargetPosition(300);


            runToPositionAll();

            leftFront.setPower(SIDEWAYS_SPEED);
            rightFront.setPower(SIDEWAYS_SPEED);
            leftRear.setPower(SIDEWAYS_SPEED);
            rightRear.setPower(SIDEWAYS_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();


            //reverse across shooting line
            leftFront.setTargetPosition(-5000);
            rightFront.setTargetPosition(-5000);
            leftRear.setTargetPosition(-5000);
            rightRear.setTargetPosition(-5000);

            runToPositionAll();

            leftFront.setPower(FORWARD_SPEED);
            rightFront.setPower(FORWARD_SPEED);
            leftRear.setPower(FORWARD_SPEED);
            rightRear.setPower(FORWARD_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path", "Complete, Turning");
            telemetry.update();

            sleep(300);

            //drive left to shoot
            leftFront.setTargetPosition(-3300);
            rightFront.setTargetPosition(3300);
            leftRear.setTargetPosition(3300);
            rightRear.setTargetPosition(-3300);

            runToPositionAll();

            leftFront.setPower(SIDEWAYS_SPEED);
            rightFront.setPower(SIDEWAYS_SPEED);
            leftRear.setPower(SIDEWAYS_SPEED);
            rightRear.setPower(SIDEWAYS_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();

            //turn to first power shot
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;
            telemetry.addData("before turn Angle: ", currentHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (currentHeading > -20 && currentHeading < 16.5)  {
                leftFront.setPower(-.1);
                rightFront.setPower(.1);
                leftRear.setPower(-.1);
                rightRear.setPower(.1);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                telemetry.addData("angle turn ", currentHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();


            //fire 1st ring at power shot
            rampDrive1.setPower(-1);
            rampDrive2.setPower(-1);
            sleep(100);
            stopRampDrive();
            launcherMotor.setPower(.8);
            sleep(1300);
            rampDrive1.setPower(1);
            rampDrive2.setPower(1);
            sleep(600);
            stopRampDrive();
            sleep(200);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            telemetry.addData("before turn Angle: ", currentHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (currentHeading > 0 && currentHeading < 20)  {
                leftFront.setPower(-.1);
                rightFront.setPower(.1);
                leftRear.setPower(-.1);
                rightRear.setPower(.1);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                telemetry.addData("angle turn ", currentHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();


            //fire 2nd ring at power shot
            rampDrive1.setPower(1);
            rampDrive2.setPower(1);
            sleep(750);
            stopRampDrive();
            sleep(400);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            telemetry.addData("before turn Angle: ", currentHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (currentHeading > 0 && currentHeading < 26.4)  {
                leftFront.setPower(-.1);
                rightFront.setPower(.1);
                leftRear.setPower(-.1);
                rightRear.setPower(.1);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                telemetry.addData("angle turn ", currentHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();


            //fire 3rd ring at power shot
            launcherMotor.setPower(.7);
            sleep(300);
            rampDrive1.setPower(1);
            rampDrive2.setPower(1);
            sleep(1200);
            stopRampDrive();
            launcherMotor.setPower(0);
            sleep(400);

            leftFront.setTargetPosition(2000);
            rightFront.setTargetPosition(2000);
            leftRear.setTargetPosition(2000);
            rightRear.setTargetPosition(2000);

            runToPositionAll();

            leftFront.setPower(FORWARD_SPEED);
            rightFront.setPower(FORWARD_SPEED);
            leftRear.setPower(FORWARD_SPEED);
            rightRear.setPower(FORWARD_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();


            sleep(30000);
        }
        //__________________________________________________________________________________________________________________________________________________________________________________________________
        else // go to close zone
        {
            knockydowny.setPower(-.6);
            grabberArm.setPosition(1);
            wobbleGrabber.setPosition(0);

            //drive right
            leftFront.setTargetPosition(1120);
            rightFront.setTargetPosition(-1120);
            leftRear.setTargetPosition(-1120);
            rightRear.setTargetPosition(1120);

            runToPositionAll();

            leftFront.setPower(SIDEWAYS_SPEED);
            rightFront.setPower(SIDEWAYS_SPEED);
            leftRear.setPower(SIDEWAYS_SPEED);
            rightRear.setPower(SIDEWAYS_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();

            sleep(400);

            //drive forward to far goal
            leftFront.setTargetPosition(5000);
            rightFront.setTargetPosition(5000);
            leftRear.setTargetPosition(5000);
            rightRear.setTargetPosition(5000);

            runToPositionAll();

            leftFront.setPower(FORWARD_SPEED);
            rightFront.setPower(FORWARD_SPEED);
            leftRear.setPower(FORWARD_SPEED);
            rightRear.setPower(FORWARD_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path", "Complete, Turning");
            telemetry.update();

            //turn right to drop wobble
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;
            double realHeading = currentHeading;
            if (currentHeading < 0 || currentHeading == -0) {
                realHeading = 360 + currentHeading;
            } else {
                realHeading = currentHeading;
            }
            realHeading = Range.clip(realHeading, 0, 360);
            telemetry.addData("before turn Angle: ", realHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (realHeading > 230 || realHeading < 20) {
                leftFront.setPower(.6);
                rightFront.setPower(-.6);
                leftRear.setPower(.6);
                rightRear.setPower(-.6);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                if (currentHeading < 0 || currentHeading == -0) {
                    realHeading = 360 + currentHeading;
                } else {
                    realHeading = currentHeading;
                }
                realHeading = Range.clip(realHeading, 0, 360);

                telemetry.addData("angle turn ", realHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();

            sleep(250);

            if (realHeading < 270 && realHeading > 200) {
                grabberArm.setPosition(0);
                sleep(450);
                wobbleGrabber.setPosition(1);
            }
            sleep(800);

            grabberArm.setPosition(1);

            //back away from dropped wobble
            leftFront.setTargetPosition(300);
            rightFront.setTargetPosition(-300);
            leftRear.setTargetPosition(-300);
            rightRear.setTargetPosition(300);

            runToPositionAll();

            leftFront.setPower(SIDEWAYS_SPEED);
            rightFront.setPower(SIDEWAYS_SPEED);
            leftRear.setPower(SIDEWAYS_SPEED);
            rightRear.setPower(SIDEWAYS_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();

            //realign to face the north wall
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            realHeading = currentHeading;
            if (currentHeading < 0 || currentHeading == -0) {
                realHeading = 360 + currentHeading;
            } else {
                realHeading = currentHeading;
            }
            telemetry.addData("before turn Angle: ", realHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (realHeading < 360 && realHeading > 5) {
                leftFront.setPower(-.6);
                rightFront.setPower(.6);
                leftRear.setPower(-.6);
                rightRear.setPower(.6);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                if (currentHeading < 0 || currentHeading == -0) {
                    realHeading = 360 + currentHeading;
                } else {
                    realHeading = currentHeading;
                }
                realHeading = Range.clip(realHeading, 0, 360);

                telemetry.addData("angle turn ", realHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();

            sleep(250);


            //drive left to shoot
            leftFront.setTargetPosition(-3000);
            rightFront.setTargetPosition(3000);
            leftRear.setTargetPosition(3000);
            rightRear.setTargetPosition(-3000);

            runToPositionAll();

            leftFront.setPower(SIDEWAYS_SPEED);
            rightFront.setPower(SIDEWAYS_SPEED);
            leftRear.setPower(SIDEWAYS_SPEED);
            rightRear.setPower(SIDEWAYS_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();

            sleep(250);

            //turn to first power shot
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            telemetry.addData("before turn Angle: ", currentHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (currentHeading > -20 && currentHeading < 14)  {
                leftFront.setPower(-.2);
                rightFront.setPower(.2);
                leftRear.setPower(-.2);
                rightRear.setPower(.2);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                telemetry.addData("angle turn ", currentHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();


            //fire 1st ring at power shot
            rampDrive1.setPower(-1);
            rampDrive2.setPower(-1);
            sleep(100);
            stopRampDrive();
            launcherMotor.setPower(.8);
            sleep(1300);
            rampDrive1.setPower(1);
            rampDrive2.setPower(1);
            sleep(600);
            stopRampDrive();
            sleep(200);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            telemetry.addData("before turn Angle: ", currentHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (currentHeading > 0 && currentHeading < 20)  {
                leftFront.setPower(-.2);
                rightFront.setPower(.2);
                leftRear.setPower(-.2);
                rightRear.setPower(.2);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                telemetry.addData("angle turn ", currentHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();


            //fire 2nd ring at power shot
            rampDrive1.setPower(1);
            rampDrive2.setPower(1);
            sleep(750);
            stopRampDrive();
            sleep(400);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            telemetry.addData("before turn Angle: ", currentHeading);
            telemetry.addData("Status: ", "Init");
            telemetry.update();

            while (currentHeading > 0 && currentHeading < 26)  {
                leftFront.setPower(-.2);
                rightFront.setPower(.2);
                leftRear.setPower(-.2);
                rightRear.setPower(.2);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;

                realHeading = Range.clip(realHeading, 0, 360);
                telemetry.addData("angle turn ", currentHeading);
                //telemetry.addData("Status: ", "Init");
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();


            //fire 3rd ring at power shot
            rampDrive1.setPower(1);
            rampDrive2.setPower(1);
            sleep(1200);
            stopRampDrive();
            launcherMotor.setPower(0);
            sleep(400);

            leftFront.setTargetPosition(2000);
            rightFront.setTargetPosition(2000);
            leftRear.setTargetPosition(2000);
            rightRear.setTargetPosition(2000);

            runToPositionAll();

            leftFront.setPower(FORWARD_SPEED);
            rightFront.setPower(FORWARD_SPEED);
            leftRear.setPower(FORWARD_SPEED);
            rightRear.setPower(FORWARD_SPEED);

            while (leftFront.isBusy() && opModeIsActive()) {
                telemetry.addData("LF Target:", leftFront.getTargetPosition());
                telemetry.addData("LF Current:", leftFront.getCurrentPosition());

                telemetry.addData("RF Target:", rightFront.getTargetPosition());
                telemetry.addData("RF Current:", rightFront.getCurrentPosition());

                telemetry.addData("LR Target:", leftRear.getTargetPosition());
                telemetry.addData("LR Current:", leftRear.getCurrentPosition());

                telemetry.addData("RR Target:", rightRear.getTargetPosition());
                telemetry.addData("RR Current:", rightRear.getCurrentPosition());
                telemetry.update();
            }
            stopDriveMotors();
            stopAndResetEncodersAll();
            telemetry.addData("Path:", "Complete, Pausing");
            telemetry.update();


            sleep(30000);
        }
    }
}


