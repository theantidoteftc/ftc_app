package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp
//@Disabled //DO NOT ENABLE UNLESS YOU HAVE ADI'S PERMISSION
public class WorldsOp extends LinearOpMode {

    NewarkHardware robot = new NewarkHardware();
    private ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public String pitch;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        boolean slowMode = false;
        boolean slowSlide = false;
        boolean sorterOpen = false;
        boolean ledSorter = false;

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
                    lPower /= 2.5;
                    rPower /= 2.5;
                } else {
                    lPower /= 2;
                    rPower /= 2;
                }
            }

            //gamepad 2 (logitech)
            double slide = -gamepad2.left_stick_y;
            double pivot = gamepad2.right_stick_y;
            double intake = ((-gamepad2.right_trigger) + (gamepad2.left_trigger));

            if (gamepad2.x) {
                intake = 0.4;
            }

            if (gamepad2.left_bumper) {
                robot.hexSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.dpad_up) {
                robot.mineralGate.setPosition(1); //open
                ledSorter = true;
            } else if (gamepad2.dpad_down) {
                robot.mineralGate.setPosition(0.5); //close
                ledSorter = false;
            }

            if (!sorterOpen && robot.hexSlide.getCurrentPosition() >= 1700 && robot.pivotMotor.getCurrentPosition() >= 1100) {
                sorterOpen = true;
                robot.mineralGate.setPosition(.99); //open
            } else if (sorterOpen && robot.hexSlide.getCurrentPosition() <= 800 && robot.pivotMotor.getCurrentPosition() <= 950) {
                sorterOpen = false;
                robot.mineralGate.setPosition(0.5); //close
            }

            if (runtime.seconds() < 1) {
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            } else if (runtime.seconds() > 105) {
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            } else if (runtime.seconds() > 90) {
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            } else if (runtime.seconds() > 75) {
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            } else if (runtime.seconds() > 0) {
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }

            if (ledSorter || sorterOpen) {
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            }



            if (gamepad2.a) {
                slowSlide = true;
            }
            if (gamepad2.b) {
                slowSlide = false;
            }

            if (slowSlide == true) {
                slide /= 5;
            }

            if (gamepad1.right_bumper) {
                lPower *= 5;
                rPower *= 5;
            }


            //gamepad 1 (xbox) setPower
            if (steering != 0) {
                if (throttle != 0) {
                    robot.hexFrontLeft.setPower(lPower);
                    robot.hexFrontRight.setPower(rPower);
                    robot.hexRearLeft.setPower(lPower);
                    robot.hexRearRight.setPower(rPower);
                } else {
                    robot.hexFrontLeft.setPower(lPower);
                    robot.hexFrontRight.setPower(rPower);
                    robot.hexRearLeft.setPower(-lPower);
                    robot.hexRearRight.setPower(-rPower);
                }
            } else {
                robot.hexFrontLeft.setPower(lPower);
                robot.hexFrontRight.setPower(rPower);
                robot.hexRearLeft.setPower(lPower);
                robot.hexRearRight.setPower(rPower);
            }

            //gamepad 2 (logitech) setPower
            robot.hexSlide.setPower(slide);
            robot.hSLeft.setPower(slide);
            robot.pivotMotor.setPower(pivot/3);
            robot.intakeMotor.setPower(intake);

            telemetry.addData("slowmode", slowMode);
            telemetry.addData("front", robot.hexFrontRight.getCurrentPosition());
            telemetry.addData("rear", robot.hexRearRight.getCurrentPosition());
            telemetry.addData("gate position", robot.mineralGate.getPosition());
            telemetry.addData("slide input", gamepad2.left_stick_y);
            telemetry.addData("slide LIVE", robot.hexSlide.getCurrentPosition());
            telemetry.addData("pivot LIVE", robot.pivotMotor.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.addData("runtime", runtime.seconds());
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

        telemetry.addLine()
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

        telemetry.addLine()
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
