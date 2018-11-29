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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="AutoServoTest", group="RRAutos")
//@Disabled
public class AutoServoTest extends LinearOpMode {

    /* Declare OpMode members. */
    RoverRuckusHardware robot   = new RoverRuckusHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    boolean alwaysTrue = true;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.hexFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.hexFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //robot.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /* Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d :%7d :%7d ",
                robot.hexFrontLeft.getCurrentPosition(),
                robot.hexFrontRight.getCurrentPosition(),
                robot.hexRearLeft.getCurrentPosition(),
                robot.hexRearRight.getCurrentPosition(),
                robot.pivotMotor.getCurrentPosition(),
                robot.hexSlide.getCurrentPosition());
        telemetry.update();*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        robot.latchLeft.setPosition(0.3);
        robot.latchRight.setPosition(0.7);
        while (alwaysTrue) {
            if (runtime.seconds() > 0.5) {
                encoderMove(0.0625,1100,0, 10);
            }            }

        if (runtime.seconds() > 1) {
                robot.latchLeft.setPosition(0.5);
                robot.latchRight.setPosition(0.5);
                alwaysTrue = false;
        }

        /*while (runtime.seconds() < 1) {
            telemetry.addData("time", runtime.seconds());
            telemetry.update();
            robot.latchLeft.setPosition(0.3);
            robot.latchRight.setPosition(0.7);
        }
        robot.latchLeft.setPosition(0.5);
        robot.latchRight.setPosition(0.5);

        encoderMove(0.0625,1100,0, 10);
        sleep(750);
        encoderMove(0.25, 0, 6000,10);
        /*telemetry.addData("meow", "meow");
        telemetry.update();
        robot.latchLeft.setPosition(0.7);
        robot.latchRight.setPosition(0.3);
        sleep(100);
        encoderDrive(0.25,1000, 1000, 3);*/


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderMove(double speed, double pivotAmount, double slideAmount, double timeoutS) {
        int newPivotMotorTarget;
        int newHexSlideTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newPivotMotorTarget = robot.pivotMotor.getCurrentPosition() + (int)(pivotAmount);// * COUNTS_PER_INCH);
            newHexSlideTarget = robot.hexSlide.getCurrentPosition() + (int)(slideAmount);// * COUNTS_PER_INCH);
            robot.pivotMotor.setTargetPosition(newPivotMotorTarget);
            robot.hexSlide.setTargetPosition(newHexSlideTarget);

            // Turn On RUN_TO_POSITION
            robot.pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.pivotMotor.setPower(Math.abs(speed));
            robot.hexSlide.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.pivotMotor.isBusy() || robot.hexSlide.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newPivotMotorTarget,  newHexSlideTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.pivotMotor.getCurrentPosition(),
                        robot.hexSlide.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.pivotMotor.setPower(0);
            robot.hexSlide.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
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
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.hexFrontLeft.getCurrentPosition() + (int)(leftInches);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.hexFrontRight.getCurrentPosition() + (int)(rightInches);// * COUNTS_PER_INCH);
            newRearLeftTarget = robot.hexFrontLeft.getCurrentPosition() + (int)(leftInches);// * COUNTS_PER_INCH);
            newRearRightTarget = robot.hexFrontRight.getCurrentPosition() + (int)(rightInches);// * COUNTS_PER_INCH);
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
                    (robot.hexFrontLeft.isBusy() && robot.hexFrontRight.isBusy())) {

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


}
