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

            double throttle = ((gamepad1.right_trigger) - (gamepad1.left_trigger));
            double pivot = gamepad1.left_stick_x;
            double slider = gamepad1.right_stick_y;
            double rPower = throttle - pivot;
            double lPower = throttle + pivot;

            if (gamepad1.a == true) {
                //while (opModeIsActive() && )
            }


            robot.hexFrontLeft.setPower(lPower);
            robot.hexFrontRight.setPower(rPower);
            robot.hexRearLeft.setPower(lPower);
            robot.hexRearRight.setPower(rPower);
            robot.pivotMotor.setPower(slider/10);
            telemetry.addData("Stick", pivot);
            telemetry.addData("slider", slider);
            telemetry.addData("Live Left", robot.hexFrontLeft.getPower());
            telemetry.addData("Live Right", robot.hexFrontRight.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
