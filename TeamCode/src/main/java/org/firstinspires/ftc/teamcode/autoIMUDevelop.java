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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
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

@Autonomous(name="autoIMUDevelop", group="conceptautos")
//@Disabled
public class autoIMUDevelop extends LinearOpMode {


    /* Declare OpMode members. */
    NewarkHardware robot   = new NewarkHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public String pitch;

    @Override
    public void runOpMode() {

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

        // Set up our telemetry dashboard
        composeTelemetry();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        /*robot.hexFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking - U R GO, GOOD LUCK!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //experimentalDrive(0.3,2500, 2);
        experimentalTurn(0.4,-90,3);
        experimentalTurn(0.4,50,3);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void experimentalDrive(double speed, double encoderAmount, double severity) {
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
        double deltaEncoder;
        double leftSpeed;
        double rightSpeed;
        while (opModeIsActive()) {
            leftSpeed = speed;
            rightSpeed = speed;
            currentHeading = angles.firstAngle;
            deltaHeading = currentHeading - startHeading;
            averageEncoder = ((robot.hexFrontLeft.getCurrentPosition() + robot.hexFrontRight.getCurrentPosition() + robot.hexRearLeft.getCurrentPosition() + robot.hexRearRight.getCurrentPosition()) / 4);
            deltaEncoder = encoderAmount - averageEncoder;

            if (deltaHeading > 0) {
                rightSpeed -= (deltaHeading/100 * severity);
            } else if (deltaHeading < 0) {
                leftSpeed += (deltaHeading/100 * severity);
            }

            robot.hexFrontLeft.setPower(leftSpeed);
            robot.hexFrontRight.setPower(rightSpeed);
            robot.hexRearLeft.setPower(leftSpeed);
            robot.hexRearRight.setPower(rightSpeed);
            telemetry.addData("delta", deltaHeading);
            telemetry.addData("live", averageEncoder);
            telemetry.addData("left", leftSpeed);
            telemetry.addData("right", rightSpeed);
            telemetry.addData("delta encoder", deltaEncoder);
            telemetry.update();
        }
    }

    public void experimentalTurn(double speed, double bearing,int acceptRange/*, double timeoutS*/) {
        double existingHeading = angles.firstAngle;
        double percentBearing = bearing * 0.45;
        double currentHeading = 0;
        double trueDeltaHeading = 1;
        double deltaHeading = 1;
        double gain = 0.0075;
        double driveSpeed;
        boolean turnDone = false;
        boolean timerStart = false;
        while (!turnDone && opModeIsActive()) {
            if (bearing > 0) {
                currentHeading = angles.firstAngle - existingHeading;
                trueDeltaHeading = bearing - currentHeading;
                deltaHeading = percentBearing - currentHeading;
                if (deltaHeading < 0) {
                    driveSpeed = .015;
                    if (Math.abs(trueDeltaHeading) <= acceptRange) {
                        if (!timerStart) {
                            runtime.reset();
                            timerStart = true;
                        } else if (runtime.seconds() > .3) {
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
                robot.hexFrontLeft.setPower(-driveSpeed);
                robot.hexFrontRight.setPower(driveSpeed);
                robot.hexRearLeft.setPower(-driveSpeed);
                robot.hexRearRight.setPower(driveSpeed);
            } else if (bearing < 0){
                currentHeading = angles.firstAngle - existingHeading;
                trueDeltaHeading = currentHeading - bearing;
                deltaHeading = currentHeading - percentBearing;
                if (deltaHeading < 0) {
                    driveSpeed = .015;
                    if (Math.abs(trueDeltaHeading) <= acceptRange) {
                        if (!timerStart) {
                            runtime.reset();
                            timerStart = true;
                        } else if (runtime.seconds() > .3) {
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
                robot.hexFrontLeft.setPower(driveSpeed);
                robot.hexFrontRight.setPower(-driveSpeed);
                robot.hexRearLeft.setPower(driveSpeed);
                robot.hexRearRight.setPower(-driveSpeed);
            } else {
                driveSpeed = 0;
            }
            telemetry.addData("current", currentHeading);
            telemetry.addData("delta", deltaHeading);
            telemetry.addData("gain", gain);
            telemetry.addData("Left Power", -driveSpeed);
            telemetry.addData("Right Power", driveSpeed);
            telemetry.update();
        }
    }

    public void deprecated(double speed, double minSpeed, double bearing,int acceptRange/*, double timeoutS*/) {
        double currentHeading = 0;
        double deltaHeading = 1;
        double gain = 0.0075;
        double driveSpeed;
        int count = 0;
        boolean turnDone = false;
        boolean timerStart = false;
        while (!turnDone && opModeIsActive()) {
            currentHeading = angles.firstAngle;
            deltaHeading = bearing - currentHeading;
            if (Math.abs(deltaHeading) <= acceptRange) {
                if (!timerStart) {
                    runtime.reset();
                    timerStart = true;
                } else if (runtime.seconds() > .5) {
                    turnDone = true;
                }
                driveSpeed = 0;
            } else {
                timerStart = false;
                driveSpeed = gain * deltaHeading * speed;
                if (driveSpeed > 1) {
                    driveSpeed = 1;
                } else if (driveSpeed < -1) {
                    driveSpeed = -1;
                } else if (Math.abs(driveSpeed) < minSpeed) {
                    if (driveSpeed < 0) {
                        driveSpeed = -minSpeed;
                    } else {
                        driveSpeed = minSpeed;
                    }
                }
            }
            robot.hexFrontLeft.setPower(-driveSpeed);
            robot.hexFrontRight.setPower(driveSpeed);
            robot.hexRearLeft.setPower(-driveSpeed);
            robot.hexRearRight.setPower(driveSpeed);
            telemetry.addData("current", currentHeading);
            telemetry.addData("delta", deltaHeading);
            telemetry.addData("gain", gain);
            telemetry.addData("count", count);
            telemetry.addData("Left Power", -driveSpeed);
            telemetry.addData("Right Power", driveSpeed);
            telemetry.update();
        }
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
