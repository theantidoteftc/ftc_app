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

package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.NewarkHardware;

import java.util.Locale;

//NOT NEEDED - motors are identified within actual program

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="autoRightWorlds", group="worldsautos")
//@Disabled
public class autoRightWorlds extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AVF7OF7/////AAABmaKBSYRMHkclubr6nFb2TLcr3QzadzX163OzDe2NS0p2hQlEvibYh8W2xO78LrAUPInfApVZ1qzOxq7fnHZ9KQ0QiJM0E5WbwxdY7U+Gbrk8NuDgceoPw7eD8j2Sk7NuvuTcXYAAoA4wKwgDlw+iA19frB/9/WuUonCWiMAi+sxSoAGkudWAx8f1AO0AXBNyf6d0QHRGVeGRyMYtvkvsez3kU6U7LnMUwpDkX5RfQi+AMKq+BTLYtOo90waG5G84TV9LU1OSlDHtPh7sSG6YuVdn0Pmm/+k9nEtedozzDeDmwKfT1A5uL1m+RGmgCe4gA45H7qH6p9ymyKDbhvfDbTo/fVI0Y9g+Z8FEMByMnI6X";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;


    /* Declare OpMode members. */
    NewarkHardware robot   = new NewarkHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    int key = 4;
    int gMineralX = -1;
    int sMineral1X = -1;
    int sMineral2X = -1;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public String pitch;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.hexFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.hexFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.hexFrontLeft.getCurrentPosition(),
                robot.hexFrontRight.getCurrentPosition(), robot.hexRearLeft.getCurrentPosition(), robot.hexRearRight.getCurrentPosition());
        telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //initVuforia(0);

        // Set up our telemetry dashboard
        composeTelemetry();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking - U R GO, GOOD LUCK!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Activate Tensor Flow Object Detection.
        if (tfod != null) {
            tfod.activate();
        }

        runtime.reset(); //TensorFlow Timer Wait
        while (opModeIsActive() && runtime.seconds() < .75) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    if (updatedRecognitions.size() == 3) { //if 3 minerals detected, use TF mandated algorithm
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                key = 0;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                key = 2;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                key = 1;
                            }
                        }
                        gMineralX = goldMineralX;
                        sMineral1X = silverMineral1X;
                        sMineral2X = silverMineral2X;
                    }

                    //if 2 minerals detected, split into (not) see gold ifs
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        //populates coordinate values
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                        //combination of 1 gold and 1 silver

                        if (goldMineralX != -1 && (silverMineral1X != -1 || silverMineral2X != -1)) {
                            if (goldMineralX < silverMineral1X && goldMineralX < 150) {
                                telemetry.addData("Gold Mineral Position", "Left (GS)");
                                key = 0;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > 350) {
                                telemetry.addData("Gold Mineral Position", "Right (GS)");
                                key = 2;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center (GS)");
                                key = 1;
                            }
                            // combination of 2 silvers
                        } else if (silverMineral1X != -1 && silverMineral2X != -1) {
                            if ((200 < silverMineral1X) && (silverMineral1X < 300) && (350 < silverMineral2X)) {
                                telemetry.addData("Gold Mineral Position", "Left (SS1)");
                                key = 0;
                            } else if ((200 < silverMineral2X) && (silverMineral2X < 300) && (350 < silverMineral1X)) {
                                telemetry.addData("Gold Mineral Position", "Left (SS2)");
                                key = 0;
                            } else if ((150 > silverMineral1X) && ((200 < silverMineral2X) && (silverMineral2X < 300))) {
                                telemetry.addData("Gold Mineral Position", "Right (SS3)");
                                key = 2;
                            } else if ((150 > silverMineral2X) && ((200 < silverMineral1X) && (silverMineral1X < 300))) {
                                telemetry.addData("Gold Mineral Position", "Right (SS4)");
                                key = 2;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center (SS5)");
                                key = 1;
                            }
                        }

                        gMineralX = goldMineralX;
                        sMineral1X = silverMineral1X;
                        sMineral2X = silverMineral2X;
                    }
                    telemetry.addData("Key", key);
                    telemetry.addData("goldMineralX", gMineralX);
                    telemetry.addData("silverMineral1X", sMineral1X);
                    telemetry.addData("silverMineral2X", sMineral2X);
                    telemetry.update();
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
            tfod.deactivate();

        }

        if (key == 0) {
            telemetry.addData("gold mineral", "left");
        } else if (key == 1) {
            telemetry.addData("gold mineral", "center");
        } else if (key == 2) {
            telemetry.addData("gold mineral", "right");
        }
        telemetry.update();*/

        /*robot.leftBlock.setPosition(1); //unlatching procedure
        robot.rightBlock.setPosition(0);
        runtime.reset();
        while (runtime.milliseconds() < 1200 && opModeIsActive()) {
        }
        robot.leftHook.setPosition(0);
        robot.rightHook.setPosition(1);
        runtime.reset();
        while (runtime.milliseconds() < 325 && opModeIsActive()) {
        }
        robot.leftHook.setPosition(0.5);
        robot.rightHook.setPosition(0.5);
        encoderAccessory(0.75,725,1);
        encoderAccessoryTimeout(0.8, 525, 0,1);
        encoderDrive(0.15,100,100,2);
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        encoderAccessoryTimeout(0.5,925,0,2.5);
        encoderDrive(0.1,-40,-40,1);
        encoderAccessory(0.4,-275,1);
        encoderAccessoryTimeout(0.95,-1725,0,2);
        encoderAccessoryTimeout(0.3,-1000,1,1);
        encoderAccessoryTimeout(0.4,200,0,0.6);
        sleep(100);*/

        key = 2;

        if (key == 0) { //left
            telemetry.addData("Left", true);
            telemetry.update();
            /*experimentalDrive(0.75,400,0.5,2);
            experimentalTurn(0.8,0.01,40,2);
            experimentalDrive(0.75,900,0.5,2);
            experimentalTurn(0.8,0.02,35,1);
            experimentalDrive(0.875,1400,0.5,2);
            experimentalTurn(0.8,0.02,25,1);
            experimentalDrive(0.75,1000,0.5,2);
            experimentalTurn(0.8,0.02,12,1);*/
            encoderDrive(.175,230, 230,2);
            encoderDrive(.22,-195,195,1.8);
            sleep(250);
            encoderDrive(.175,850,850,2);
            encoderDrive(.1,-430,-430,2);
            experimentalTurn(0.9,0.01,43,3);
            experimentalDrive(.9,1675,0.75,2.75);
            encoderDrive(0.2,-225,225,1.5);/*
            experimentalDrive(0.6,600,0.9,2.5);
            encoderAccessoryTimeout(0.99,2600,0,2);
            encoderAccessory(0.5,800,1);
            robot.intakeMotor.setPower(-0.5);
            runtime.reset();
            while (runtime.seconds() < .5) {
                telemetry.addData("Dropping Marker!", true);
                telemetry.update();
            }
            robot.intakeMotor.setPower(0);
            encoderAccessoryTimeout(0.99,-2500,0,2);
            encoderDrive(.15,-175,175,1.5);
            encoderDrive(.15,-175,175,1.5);
            encoderDrive(0.23,1200,1200,1.8);
            encoderAccessory(0.5,800,1);*/
        } else if (key == 1) { //center
            //OVER 30 DEGREES
            /*experimentalTurn(.8,0.006,90,3);*/

            //UNDER 30 DEGREES
            //experimentalTurn(5,0.02,-15,2);
            telemetry.addData("Center", true);
            telemetry.update();
            experimentalDrive(0.7,1100,0.7,3);
            experimentalDrive(0.5,-250,0.6,3);
            experimentalTurn(0.9,0.006,86,3);
            experimentalDrive(0.95,1800,0.6,3);
            encoderDrive(0.2,-160,160,1.2);
            experimentalDrive(0.7,1000,0.5,3);
            encoderDrive(0.25,-90,90,1);
            encoderAccessoryTimeout(1,2500,0,2);
            encoderAccessoryTimeout(0.75,400,1,2);
            robot.intakeMotor.setPower(-0.45);
            runtime.reset();
            while (runtime.seconds() < .5) {
                telemetry.addData("Dropping Marker!", true);
                telemetry.update();
            }
            robot.intakeMotor.setPower(0);
            encoderAccessoryTimeout(0.99,-2500,0,3);
            encoderDrive(1,-1700,-1700,3);
        } else if (key == 2) { //right
            telemetry.addData("Right", true);
            telemetry.update();
            experimentalTurn(5,0.03,-30,2);
            sleep(500);
            experimentalDrive(0.7,1150,0.7,3);
            experimentalDrive(0.5,-380,0.6,3);
            experimentalTurn(0.9,0.006,115,3);
            experimentalDrive(0.7,2375,0.7,3);
            encoderDrive(0.1,-165,165,1.5);
            experimentalDrive(0.8,750,0.4,3);
            encoderDrive(0.3,-90,90,1);
            encoderAccessoryTimeout(1,2500,0,1.7);
            encoderAccessoryTimeout(0.75,400,1,1);
            robot.intakeMotor.setPower(-0.75);
            runtime.reset();
            while (runtime.seconds() < .35) {
                telemetry.addData("Dropping Marker!", true);
                telemetry.update();
            }
            robot.intakeMotor.setPower(0);
            encoderAccessoryTimeout(0.99,-2500,0,2);
            robot.hexFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.hexFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.hexRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.hexRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            encoderDrive(1,-1700,-1700,3);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void experimentalTurn(double speed, double minSpeed, double bearing,int acceptRange/*, double timeoutS*/) {
        robot.hexFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double existingHeading = angles.firstAngle;
        double percentBearing = bearing * 0.7; //5 //425
        double currentHeading = 0;
        double trueDeltaHeading = 1;
        double deltaHeading = 1;
        double gain = 4.4112e-4 + (1.8732e-4 * (Math.log(Math.abs(bearing))));//(0.000002*bearing) + 0.001106; //0.001375; //0.0013 //75
        double driveSpeed;
        boolean turnDone = false;
        boolean timerStart = false;
        while (!turnDone && opModeIsActive()) {
            if (bearing > 0) {
                currentHeading = angles.firstAngle - existingHeading;
                trueDeltaHeading = bearing - currentHeading;
                deltaHeading = percentBearing - currentHeading;
                if (bearing > 30) {
                    if (trueDeltaHeading < 0) {
                        break;
                    } else if (deltaHeading < 0) {
                        driveSpeed = minSpeed;
                        if (Math.abs(trueDeltaHeading) <= acceptRange) {
                            if (!timerStart) {
                                runtime.reset();
                                timerStart = true;
                            } else if (runtime.seconds() > .2) {
                                turnDone = true;
                            }
                            driveSpeed = 0;
                        }
                    } else {
                        timerStart = false;
                        driveSpeed = gain * deltaHeading * speed;
                        if (driveSpeed > 1) {
                            driveSpeed = 1;
                        } else if (driveSpeed < -1) {
                            driveSpeed = -1;
                        }
                    }
                    robot.hexRearLeft.setPower(-driveSpeed * 2);
                    robot.hexRearRight.setPower(driveSpeed * 2);
                } else {
                    if (trueDeltaHeading < 0) {
                        break;
                    } else if (deltaHeading < 0) {
                        driveSpeed = minSpeed;
                        if (Math.abs(trueDeltaHeading) <= acceptRange) {
                            if (!timerStart) {
                                runtime.reset();
                                timerStart = true;
                            } else if (runtime.seconds() > .1) {
                                turnDone = true;
                            }
                            driveSpeed = 0;
                        }
                    } else {
                        timerStart = false;
                        driveSpeed = gain * (Math.pow((deltaHeading/bearing),4) * bearing) * speed * 3;
                        if (driveSpeed < minSpeed) {
                            driveSpeed = minSpeed;
                        }
                        if (driveSpeed > 1) {
                            driveSpeed = 1;
                        } else if (driveSpeed < -1) {
                            driveSpeed = -1;
                        }
                    }
                    robot.hexFrontLeft.setPower(-driveSpeed);
                    robot.hexRearRight.setPower(driveSpeed);
                    robot.hexRearLeft.setPower(-driveSpeed);
                    robot.hexRearRight.setPower(driveSpeed);
                }
            } else if (bearing < 0){
                currentHeading = angles.firstAngle - existingHeading;
                trueDeltaHeading = currentHeading - bearing;
                deltaHeading = currentHeading - percentBearing;
                if (bearing < -30) {
                    if (trueDeltaHeading < 0) {
                        break;
                    } else if (deltaHeading < 0) {
                        driveSpeed = minSpeed;
                        if (Math.abs(trueDeltaHeading) <= acceptRange) {
                            if (!timerStart) {
                                runtime.reset();
                                timerStart = true;
                            } else if (runtime.seconds() > .2) {
                                turnDone = true;
                            }
                            driveSpeed = 0;
                        }
                    } else {
                        timerStart = false;
                        driveSpeed = gain * deltaHeading * speed;
                        if (driveSpeed > 1) {
                            driveSpeed = 1;
                        } else if (driveSpeed < -1) {
                            driveSpeed = -1;
                        }
                    }
                    robot.hexRearLeft.setPower(driveSpeed * 2);
                    robot.hexRearRight.setPower(-driveSpeed * 2);
                } else {
                    if (trueDeltaHeading < 0) {
                        break;
                    } else if (deltaHeading < 0) {
                        driveSpeed = minSpeed;
                        if (Math.abs(trueDeltaHeading) <= acceptRange) {
                            if (!timerStart) {
                                runtime.reset();
                                timerStart = true;
                            } else if (runtime.seconds() > .1) {
                                turnDone = true;
                            }
                            driveSpeed = 0;
                        }
                    } else {
                        timerStart = false;
                        driveSpeed = Math.abs(gain * (Math.pow((deltaHeading/bearing),4) * bearing) * speed * 3);
                        if (driveSpeed < minSpeed) {
                            driveSpeed = minSpeed;
                        }
                        if (driveSpeed > 1) {
                            driveSpeed = 1;
                        } else if (driveSpeed < -1) {
                            driveSpeed = -1;
                        }
                    }
                    robot.hexFrontLeft.setPower(driveSpeed);
                    robot.hexRearRight.setPower(-driveSpeed);
                    robot.hexRearLeft.setPower(driveSpeed);
                    robot.hexRearRight.setPower(-driveSpeed);
                }
            } else {
                driveSpeed = 0;
            }
            telemetry.addData("current", currentHeading);
            telemetry.addData("delta", deltaHeading);
            telemetry.addData("truedelta", trueDeltaHeading);
            telemetry.addData("multipliedDelta", (Math.pow((deltaHeading/bearing),2) * bearing));
            telemetry.addData("gain", gain);
            telemetry.addData("Left Power", -driveSpeed);
            telemetry.addData("Right Power", driveSpeed);
            telemetry.addData("BACK Left Power", -driveSpeed * 1.5);
            telemetry.addData("BACK Right Power", driveSpeed * 1.5);
            telemetry.update();
        }
        robot.hexFrontLeft.setPower(0);
        robot.hexFrontRight.setPower(0);
        robot.hexRearLeft.setPower(0);
        robot.hexRearRight.setPower(0);
    }

    public void experimentalDrive(double speed, double encoderAmount, double percentage, double severity) {
        robot.hexFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double startHeading = angles.firstAngle;
        double currentHeading;
        double deltaHeading;
        double averageEncoder;
        double deltaEncoder = 1;
        double trueDeltaEncoder = 1;
        double percentEncoder = encoderAmount * percentage;
        double gain = 0.0075;
        double leftSpeed;
        double rightSpeed;
        boolean turnDone = false;
        boolean timerStart = false;
        while (!turnDone && opModeIsActive()) {
            currentHeading = angles.firstAngle;
            deltaHeading = currentHeading - startHeading;
            averageEncoder = ((robot.hexFrontLeft.getCurrentPosition() + robot.hexFrontRight.getCurrentPosition() + robot.hexRearLeft.getCurrentPosition() + robot.hexRearRight.getCurrentPosition()) / 4);

            if (encoderAmount > 0) {
                trueDeltaEncoder = encoderAmount - averageEncoder;
                deltaEncoder = percentEncoder - averageEncoder;
                if (trueDeltaEncoder < 0) {
                    leftSpeed = -0.14;
                    rightSpeed = -0.14;
                } else if (deltaEncoder < 0) {
                    leftSpeed = 0.17;
                    rightSpeed = 0.17;
                    if (Math.abs(trueDeltaEncoder) <= 40) {
                        if (!timerStart) {
                            runtime.reset();
                            timerStart = true;
                        } else if (runtime.seconds() > .3) {
                            turnDone = true;
                        }
                        leftSpeed = 0;
                        rightSpeed = 0;
                    }
                } else {
                    leftSpeed = gain * deltaEncoder * speed;
                    rightSpeed = gain * deltaEncoder * speed;
                    if (leftSpeed > speed) {
                        leftSpeed = speed;
                    } else if (leftSpeed <= 0.15) {
                        leftSpeed = 0.17;
                    }
                    if (rightSpeed > speed) {
                        rightSpeed = speed;
                    } else if (rightSpeed <= 0.15) {
                        rightSpeed = 0.17;
                    }

                    if (deltaHeading > 0) {
                        rightSpeed -= (deltaHeading/100 * severity);
                    } else if (deltaHeading < 0) {
                        leftSpeed += (deltaHeading/100 * severity);
                    }
                }
                robot.hexFrontLeft.setPower(leftSpeed);
                robot.hexFrontRight.setPower(rightSpeed);
                robot.hexRearLeft.setPower(leftSpeed);
                robot.hexRearRight.setPower(rightSpeed);
            } else if (encoderAmount < 0) {
                trueDeltaEncoder = averageEncoder - encoderAmount;
                deltaEncoder = averageEncoder - percentEncoder;
                if (deltaEncoder < 0) {
                    leftSpeed = 0.15;
                    rightSpeed = 0.15;
                    if (Math.abs(trueDeltaEncoder) <= 40) {
                        if (!timerStart) {
                            runtime.reset();
                            timerStart = true;
                        } else if (runtime.seconds() > .3) {
                            turnDone = true;
                        }
                        leftSpeed = 0;
                        rightSpeed = 0;
                    }
                } else {
                    leftSpeed = gain * deltaEncoder * speed;
                    rightSpeed = gain * deltaEncoder * speed;
                    if (leftSpeed > speed) {
                        leftSpeed = speed;
                    }
                    if (rightSpeed > speed) {
                        rightSpeed = speed;
                    }

                    if (deltaHeading > 0) {
                        rightSpeed += (deltaHeading/100 * severity);
                    } else if (deltaHeading < 0) {
                        leftSpeed -= (deltaHeading/100 * severity);
                    }
                }
                robot.hexFrontLeft.setPower(-leftSpeed);
                robot.hexFrontRight.setPower(-rightSpeed);
                robot.hexRearLeft.setPower(-leftSpeed);
                robot.hexRearRight.setPower(-rightSpeed);
            } else {
                leftSpeed = 0;
                rightSpeed = 0;
            }

            telemetry.addData("delta", deltaHeading);
            telemetry.addData("live", averageEncoder);
            telemetry.addData("left", leftSpeed);
            telemetry.addData("right", rightSpeed);
            telemetry.addData("delta encoder", deltaEncoder);
            telemetry.addData("true delta encoder", trueDeltaEncoder);
            telemetry.update();
        }
    }


    public void encoderDrive(double speed, double leftEncoder, double rightEncoder, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        robot.hexFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.hexFrontLeft.getCurrentPosition() + (int)(leftEncoder);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.hexFrontRight.getCurrentPosition() + (int)(rightEncoder);// * COUNTS_PER_INCH);
            newRearLeftTarget = robot.hexFrontLeft.getCurrentPosition() + (int)(leftEncoder);// * COUNTS_PER_INCH);
            newRearRightTarget = robot.hexFrontRight.getCurrentPosition() + (int)(rightEncoder);// * COUNTS_PER_INCH);
            robot.hexFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.hexFrontRight.setTargetPosition(newFrontRightTarget);
            robot.hexRearLeft.setTargetPosition(newRearLeftTarget);
            robot.hexRearRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            robot.hexFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hexFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hexRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hexRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.hexFrontLeft.setPower(Math.abs(speed));
            robot.hexFrontRight.setPower(Math.abs(speed));
            robot.hexRearLeft.setPower(Math.abs(speed));
            robot.hexRearRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.hexFrontLeft.isBusy() || robot.hexRearLeft.isBusy() || robot.hexFrontRight.isBusy() || robot.hexRearRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.hexFrontLeft.getCurrentPosition(),
                        robot.hexFrontRight.getCurrentPosition());
                telemetry.update();
            }

        }
    }

    public void encoderAccessory(double speed, double encoderAmount, int port) {
        int newSlideTarget;
        int newPivotTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if (port == 0) {
                newSlideTarget = robot.hexSlide.getCurrentPosition() + (int)(encoderAmount);// * COUNTS_PER_INCH);
                robot.hexSlide.setTargetPosition(newSlideTarget);
            } else if (port == 1) {
                newPivotTarget = robot.pivotMotor.getCurrentPosition() + (int)(encoderAmount);// * COUNTS_PER_INCH);
                robot.pivotMotor.setTargetPosition(newPivotTarget);
            }

            // Turn On RUN_TO_POSITION
            if (port == 0) {
                robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (port == 1) {
                robot.pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // reset the timeout time and start motion.
            runtime.reset();
            if (port == 0) {
                robot.hexSlide.setPower(Math.abs(speed));
            } else if (port == 1) {
                robot.pivotMotor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (robot.hexSlide.isBusy() || robot.pivotMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running at %7d :%7d", robot.hexSlide.getCurrentPosition(), robot.pivotMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.hexSlide.setPower(0);
            robot.pivotMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderAccessoryTimeout(double speed, double encoderAmount, int port, double timeoutS) {
        int newSlideTarget;
        int newPivotTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if (port == 0) {
                newSlideTarget = robot.hexSlide.getCurrentPosition() + (int)(encoderAmount);// * COUNTS_PER_INCH);
                robot.hexSlide.setTargetPosition(newSlideTarget);
            } else if (port == 1) {
                newPivotTarget = robot.pivotMotor.getCurrentPosition() + (int)(encoderAmount);// * COUNTS_PER_INCH);
                robot.pivotMotor.setTargetPosition(newPivotTarget);
            }

            // Turn On RUN_TO_POSITION
            if (port == 0) {
                robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (port == 1) {
                robot.pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // reset the timeout time and start motion.
            runtime.reset();
            if (port == 0) {
                robot.hexSlide.setPower(Math.abs(speed));
            } else if (port == 1) {
                robot.pivotMotor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.hexSlide.isBusy() || robot.pivotMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running at %7d :%7d", robot.hexSlide.getCurrentPosition(), robot.pivotMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.hexSlide.setPower(0);
            robot.pivotMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(double io) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        if (io == 0) {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */

            parameters.vuforiaLicenseKey = "AVF7OF7/////AAABmaKBSYRMHkclubr6nFb2TLcr3QzadzX163OzDe2NS0p2hQlEvibYh8W2xO78LrAUPInfApVZ1qzOxq7fnHZ9KQ0QiJM0E5WbwxdY7U+Gbrk8NuDgceoPw7eD8j2Sk7NuvuTcXYAAoA4wKwgDlw+iA19frB/9/WuUonCWiMAi+sxSoAGkudWAx8f1AO0AXBNyf6d0QHRGVeGRyMYtvkvsez3kU6U7LnMUwpDkX5RfQi+AMKq+BTLYtOo90waG5G84TV9LU1OSlDHtPh7sSG6YuVdn0Pmm/+k9nEtedozzDeDmwKfT1A5uL1m+RGmgCe4gA45H7qH6p9ymyKDbhvfDbTo/fVI0Y9g+Z8FEMByMnI6X";
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

        } else if (io == 1) {
            parameters.camera.close();
        }

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        /*telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        */

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        pitch = formatAngle(angles.angleUnit, angles.thirdAngle);
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        /*telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });

        telemetry.addLine()
                .addData("Debugging: ", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        */
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
