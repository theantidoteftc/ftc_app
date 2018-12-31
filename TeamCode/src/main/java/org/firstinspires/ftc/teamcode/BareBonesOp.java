package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class BareBonesOp extends LinearOpMode {

    BareBonesHardware robot = new BareBonesHardware();
    private ElapsedTime runtime = new ElapsedTime();
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

            if (gamepad1.a) {
                slowMode = true;
            }
            if (gamepad1.b) {
                slowMode = false;
            }
            double rPower = throttle - steering;
            double lPower = throttle + steering;

            if (slowMode == true) {
                if (steering != 0) {
                    lPower /= 3;
                    rPower /= 3;
                } else {
                    lPower /= 2;
                    rPower /= 2;
                }
            }

            //gamepad 2 (logitech)
            double slide = -gamepad2.left_stick_y;
            double pivot = gamepad2.right_stick_y;
            double intake = ((gamepad2.right_trigger) - (gamepad2.left_trigger)) + 0.5;

            if (gamepad2.dpad_up) {
                robot.lonk.setPosition(1); //close
            } else if (gamepad2.dpad_down) {
                robot.lonk.setPosition(0); //open
            }

            /*if (gamepad2.right_bumper) {
                robot.hexSlide.setTargetPosition(6000);
                robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hexSlide.setPower(0.25);
                while (opModeIsActive() && robot.hexSlide.isBusy()) {
                    telemetry.addData(">", runtime.seconds());
                    telemetry.update();
                }
                sleep(1000);
            } else if (gamepad2.left_bumper) {
                robot.hexSlide.setTargetPosition(1500);
                robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hexSlide.setPower(0.25);
                while (opModeIsActive() && robot.hexSlide.isBusy()) {
                    telemetry.addData(">", runtime.seconds());
                    telemetry.update();
                }
                sleep(1000);
            }*/

            //gamepad 1 (xbox) setPower
            robot.hexFrontLeft.setPower(lPower);
            robot.hexFrontRight.setPower(rPower);
            robot.hexRearLeft.setPower(lPower);
            robot.hexRearRight.setPower(rPower);

            //gamepad 2 (logitech) setPower
            robot.hexSlide.setPower(slide);
            robot.pivotMotor.setPower(pivot/4);
            robot.take.setPosition(intake);

            telemetry.addData("slide input", gamepad2.left_stick_y);
            telemetry.addData("pivot input", gamepad2.right_stick_y);
            telemetry.addData("slide LIVE", robot.hexSlide.getCurrentPosition());
            telemetry.addData("pivot LIVE", robot.pivotMotor.getCurrentPosition());
            telemetry.addData("intake", robot.take.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.addData("slowmode", slowMode);
            telemetry.update();
        }
    }
}
