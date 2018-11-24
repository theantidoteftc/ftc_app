package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp
public class RecordEncoders extends LinearOpMode {

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

        /*robot.hexFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hexRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        robot.hexSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {
            double throttle = gamepad1.left_stick_x;

            robot.hexSlide.setPower(throttle);
            /*telemetry.addData("Live Left", robot.hexRearLeft.getCurrentPosition());
            telemetry.addData("Live Right", robot.hexRearRight.getCurrentPosition());*/
            telemetry.addData("raw", throttle);
            telemetry.addData("Live Left", (robot.hexFrontLeft.getCurrentPosition() + (robot.hexRearLeft.getCurrentPosition()))/2);
            telemetry.addData("Live Right", (robot.hexFrontRight.getCurrentPosition() + (robot.hexRearRight.getCurrentPosition()))/2);
            telemetry.addData("Live Pivot", robot.pivotMotor.getCurrentPosition());
            telemetry.addData("Live Slide", robot.hexSlide.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
