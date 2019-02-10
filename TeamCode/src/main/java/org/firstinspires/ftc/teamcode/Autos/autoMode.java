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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.NewarkHardware;

import java.util.List;

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

@Autonomous(name="autoMode", group="newarkautos")
//@Disabled
public class autoMode extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AVF7OF7/////AAABmaKBSYRMHkclubr6nFb2TLcr3QzadzX163OzDe2NS0p2hQlEvibYh8W2xO78LrAUPInfApVZ1qzOxq7fnHZ9KQ0QiJM0E5WbwxdY7U+Gbrk8NuDgceoPw7eD8j2Sk7NuvuTcXYAAoA4wKwgDlw+iA19frB/9/WuUonCWiMAi+sxSoAGkudWAx8f1AO0AXBNyf6d0QHRGVeGRyMYtvkvsez3kU6U7LnMUwpDkX5RfQi+AMKq+BTLYtOo90waG5G84TV9LU1OSlDHtPh7sSG6YuVdn0Pmm/+k9nEtedozzDeDmwKfT1A5uL1m+RGmgCe4gA45H7qH6p9ymyKDbhvfDbTo/fVI0Y9g+Z8FEMByMnI6X";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;


    /* Declare OpMode members. */
    NewarkHardware robot   = new NewarkHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX MotorC Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265);
    static final double     DRIVE_SPEED             = 0.45;
    static final double     TURN_SPEED              = 0.25;

    int inte = 0;
    int rot = 0;
    int key = 0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking - U R GO, GOOD LUCK!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //drive forward
        encoderDrive(0.125,4000,4000,3);

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        while (rot == 0 && opModeIsActive()) {
            if (opModeIsActive()) {
                runtime.reset();

                }
            }

        }




    public void encoderTurn(double baseSpeed, double leftAmount, double rightAmount) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int greater = 0;
        double speedRatio = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if (rightAmount > leftAmount) {
                greater = 1;
            } else if (rightAmount < leftAmount) {
                greater = -1;
            }

            if (greater == 1) {
                speedRatio = rightAmount/leftAmount;
            } else if (greater == -1) {
                speedRatio = leftAmount/rightAmount;
            }

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.hexFrontLeft.getCurrentPosition() + (int)(leftAmount);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.hexFrontRight.getCurrentPosition() + (int)(rightAmount);// * COUNTS_PER_INCH);
            newRearLeftTarget = robot.hexFrontLeft.getCurrentPosition() + (int)(leftAmount);// * COUNTS_PER_INCH);
            newRearRightTarget = robot.hexFrontRight.getCurrentPosition() + (int)(rightAmount);// * COUNTS_PER_INCH);
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
            if (greater == 1) {
                robot.hexFrontLeft.setPower(Math.abs(baseSpeed));
                robot.hexFrontRight.setPower(Math.abs(baseSpeed) * speedRatio);
                robot.hexRearLeft.setPower(Math.abs(baseSpeed));
                robot.hexRearRight.setPower(Math.abs(baseSpeed) * speedRatio);
            } else if (greater == -1) {
                robot.hexFrontLeft.setPower(Math.abs(baseSpeed) * speedRatio);
                robot.hexFrontRight.setPower(Math.abs(baseSpeed));
                robot.hexRearLeft.setPower(Math.abs(baseSpeed) * speedRatio);
                robot.hexRearRight.setPower(Math.abs(baseSpeed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.hexFrontLeft.isBusy() || robot.hexFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("yeet", robot.hexFrontLeft.getCurrentPosition());
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.hexFrontLeft.getCurrentPosition(),
                        robot.hexFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.hexFrontLeft.setPower(0);
            robot.hexFrontRight.setPower(0);
            robot.hexRearLeft.setPower(0);
            robot.hexRearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.hexFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hexFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hexRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hexRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftEncoder, double rightEncoder, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

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
                    (robot.hexFrontLeft.isBusy() || robot.hexFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.hexFrontLeft.getCurrentPosition(),
                        robot.hexFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.hexFrontLeft.setPower(0);
            robot.hexFrontRight.setPower(0);
            robot.hexRearLeft.setPower(0);
            robot.hexRearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.hexFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hexFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hexRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hexRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
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
}
