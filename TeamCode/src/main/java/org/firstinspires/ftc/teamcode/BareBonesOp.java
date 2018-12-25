package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp
public class BareBonesOp extends LinearOpMode {

    BareBonesHardware robot = new BareBonesHardware();
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

        boolean slowMode = false;
        while (opModeIsActive()) {
            //gamepad 1 (xbox)
            double throttle = ((gamepad1.right_trigger) - (gamepad1.left_trigger));
            double steering = gamepad1.left_stick_x;

            //gamepad 2 (logitech)
            double slide = gamepad2.left_stick_y;
            double pivot = gamepad2.right_stick_y;

            if (gamepad1.a) {
                slowMode = true;
            }
            if (gamepad1.b) {
                slowMode = false;
            }
            double rPower = throttle - steering;
            double lPower = throttle + steering;

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
            robot.hexSlide.setPower(slide/15);
            robot.pivotMotor.setPower(pivot/15);

            telemetry.addData("slide", gamepad2.left_stick_y);
            telemetry.addData("pivot", gamepad2.right_stick_y);
            telemetry.addData("Status", "Running");
            telemetry.addData("slowmode", slowMode);
            telemetry.update();
        }
    }
}
