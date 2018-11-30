package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp
public class DeltaOp extends LinearOpMode {

    RoverRuckusHardware robot = new RoverRuckusHardware();
    private Gyroscope imu;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        imu = hardwareMap.get(Gyroscope.class, "imu");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double powerRT = 0;
        double powerLT = 0;
        double tgtPowerStick = 0;
        double leftPower = 0;
        double rightPower = 0;
        double wheel = 0;
        int trueFalse = 0;
        double setting;

        boolean slowMode = false;
        while (opModeIsActive()) {
            /*powerRT = this.gamepad1.right_trigger;
            powerLT =  -this.gamepad1.left_trigger;
            tgtPowerStick = this.gamepad1.left_stick_x;
            leftPower = (powerLT + powerRT);
            rightPower = (powerLT + powerRT);
            if (powerRT != 0) {
                if (tgtPowerStick > 0 && tgtPowerStick <= 0.75) {
                    rightPower -= (tgtPowerStick * 3);
                } else if (tgtPowerStick > 0.75) {
                    leftPower *= tgtPowerStick;
                    rightPower *= -tgtPowerStick;
                } else if (-0.75 <= tgtPowerStick && tgtPowerStick < 0) {
                    leftPower += (tgtPowerStick * 3);
                } else if (tgtPowerStick < -0.75) {
                    leftPower *= tgtPowerStick;
                    rightPower *= -tgtPowerStick;
                }
            }
            if (powerLT != 0) {
                if (tgtPowerStick > 0 && tgtPowerStick <= 0.75) {
                    rightPower -= (tgtPowerStick * 2);
                } else if (tgtPowerStick > 0.75) {
                    leftPower *= -tgtPowerStick;
                    rightPower *= tgtPowerStick;
                } else if (-0.75 <= tgtPowerStick && tgtPowerStick < 0) {
                    leftPower -= (tgtPowerStick * 2);
                } else if (tgtPowerStick < -0.75) {
                    leftPower *= -tgtPowerStick;
                    rightPower *= tgtPowerStick;
                }
            }*/

            //gamepad 1 (xbox)
            double throttle = ((gamepad1.right_trigger) - (gamepad1.left_trigger));
            double steering = gamepad1.left_stick_x;
            if (gamepad1.a) {
                slowMode = true;
            }
            if (gamepad1.b) {
                slowMode = false;
            }

            //gamepad 2 (logitech)
            double intake = ((gamepad2.right_trigger) - (gamepad2.left_trigger)) + 0.5;
            double slide = -gamepad2.left_stick_y;
            double pivot = -gamepad2.right_stick_y;
            double rPower = throttle - steering;
            double lPower = throttle + steering;

            if (gamepad2.a) {
                robot.latchLeft.setPosition(0.7);
                robot.latchRight.setPosition(0.3);
                sleep(1000);
                robot.latchLeft.setPosition(0.5);
                robot.latchRight.setPosition(0.5);
                sleep(100);
                robot.latchLeft.setPosition(0.3);
                robot.latchRight.setPosition(0.7);
                sleep(200);
                robot.latchLeft.setPosition(0.5);
                robot.latchRight.setPosition(0.5);

            }
            if (gamepad2.b) {
            }

            if (slowMode == true) {
                lPower /= 3;
                rPower /= 3;
            }

            //gamepad 1 (xbox) setPower
            robot.hexFrontLeft.setPower(lPower);
            robot.hexFrontRight.setPower(rPower);
            robot.hexRearLeft.setPower(lPower);
            robot.hexRearRight.setPower(rPower);

            //gamepad 2 (logitech) setPower
            robot.intake.setPosition(intake);
            robot.hexSlide.setPower(slide);
            robot.pivotMotor.setPower(pivot/5);

            telemetry.addData("right", gamepad2.right_trigger);
            telemetry.addData("left", gamepad2.left_trigger);
            telemetry.addData("Live Intake", robot.intake.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.addData("slowmode", slowMode);
            telemetry.update();
        }
    }
}
