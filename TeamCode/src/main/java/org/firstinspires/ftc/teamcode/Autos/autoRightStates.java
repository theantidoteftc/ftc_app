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

    @Autonomous(name="autoRightStates", group="statesautos")
    //@Disabled
    public class autoRightStates extends LinearOpMode {
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

            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia(0);

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start tracking - U R GO, GOOD LUCK!");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            runtime.reset(); //TensorFlow Timer Wait
            while (opModeIsActive() && runtime.seconds() < 2) {
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
            telemetry.update();

            robot.leftBlock.setPosition(1); //unlatching procedure
            robot.rightBlock.setPosition(0);
            sleep(1500);
            runtime.reset();
            while (runtime.seconds() < 0.325 && opModeIsActive()) {
                robot.leftHook.setPosition(0);
                robot.rightHook.setPosition(1);
            }
            robot.leftHook.setPosition(0.5);
            robot.rightHook.setPosition(0.5);
            encoderAccessory(0.75,1200,1);
            encoderAccessoryTimeout(0.7, 400, 0,1);
            encoderDrive(0.15,100,100,2);
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            encoderAccessoryTimeout(0.5,925,0,2.5);
            encoderAccessory(0.2,-300,1);
            encoderAccessoryTimeout(0.5,-1800,0,3);
            encoderAccessory(0.3,-800,1);
            sleep(100);

            if (key == 0) { //left
                telemetry.addData("Left", true);
                telemetry.update();
                encoderDrive(.225,180, 180,2);
                encoderDrive(.2,-220,220,1.5);
                encoderDrive(.15,1725,1725,1.5);
                encoderDrive(.1,-325,-325,1);
                encoderDrive(.25,270,-270,2);
                encoderDrive(.225,1200,1200,1.5);
                runtime.reset();
                robot.intakeServo.setPosition(0.06);
                while (opModeIsActive() && runtime.seconds() < 2) {
                    telemetry.addData("Dropping Marker!", true);
                    telemetry.update();
                }
                robot.intakeServo.setPosition(0.5);
                encoderDrive(.2,-1350,-1350,2);
                encoderAccessory(.45,300,1);
                encoderAccessoryTimeout(.8,-400,0,1);
                encoderDrive(.15,300,-300,1.5);
                encoderDrive(.95,2125,2125,2.75);
                encoderAccessory(0.5,650,1);
                encoderAccessory(0.9,2300,0);
            } else if (key == 1) { //center
                telemetry.addData("Center", true);
                telemetry.update();
                encoderDrive(0.2,30,-30,2);
                encoderDrive(.2,1750,1750,2);
                runtime.reset();
                robot.intakeServo.setPosition(0.06);
                while (opModeIsActive() && runtime.seconds() < 1.5) {
                    telemetry.addData("Dropping Marker!", true);
                    telemetry.update();
                }
                robot.intakeServo.setPosition(0.5);
                encoderDrive(.2,-1375,-1375,2);
                sleep(100);
                encoderAccessory(.2,400,1);
                encoderAccessoryTimeout(.5,-200,0,1.5);
                encoderDrive(.15,355,-355,2);
                encoderDrive(.2,1650,1650,3.5);
                encoderDrive(.15,120,-120,2);
                encoderAccessory(0.3,650,1);
                encoderAccessory(0.6,2000,0);
            } else if (key == 2) { //right
                telemetry.addData("Right", true);
                telemetry.update();
                encoderDrive(0.25,-50,-50,2);
                encoderAccessory(0.2,600,1);
                encoderDrive(0.25,170,-170,2);
                encoderAccessory(0.2,-500,1);
                encoderDrive(0.17,1450,1450,2);
                encoderAccessory(0.3,800,1);
                encoderDrive(0.2,-380,380,1.5);
                encoderDrive(0.25,975,975,1.5);
                encoderDrive(0.15,65,-65,1.5);
                runtime.reset();
                robot.intakeServo.setPosition(0.06);
                while (opModeIsActive() && runtime.seconds() < 1) {
                    telemetry.addData("Dropping Marker!", true);
                    telemetry.update();
                }
                robot.intakeServo.setPosition(0.5);
                encoderDrive(0.275,-1100,-1100,3);
                encoderDrive(0.15,-900,900,2);
                encoderDrive(0.5,825,825,2.5);
                encoderAccessory(0.355,650,1);
                encoderAccessory(0.625,2000,0);
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
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
