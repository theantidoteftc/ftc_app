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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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

@Autonomous(name="autoIMUDevelop", group="worldsautos")
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

        //OVER 30 DEGREES
        /*experimentalTurn(.8,0.006,90,3);*/

        //UNDER 30 DEGREES
        //experimentalTurn(5,0.02,-15,2);
        /*while (opModeIsActive()) {
            telemetry.addData("heading", angles.firstAngle);
            telemetry.update();
        }*/


        //experimentalTurn(0.9, 0.03,-90,3);

        /*experimentalTurn(0.5,135,3);
        sleep(2000);
        experimentalTurn(0.5,-90,3);
        sleep(2000);
        experimentalTurn(0.5,-135,3);
        sleep(2000);
        experimentalTurn(0.5,90,3);*/

        while (opModeIsActive()) {
            telemetry.addData("distance travelled", ((robot.hexFrontLeft.getCurrentPosition() + robot.hexFrontRight.getCurrentPosition() + robot.hexRearLeft.getCurrentPosition() + robot.hexRearRight.getCurrentPosition()) / 4));
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
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
                    leftSpeed = -0.22;
                    rightSpeed = -0.22;
                } else if (deltaEncoder < 0) {
                    leftSpeed = 0.1; //0.17
                    rightSpeed = 0.1; //0.17
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
