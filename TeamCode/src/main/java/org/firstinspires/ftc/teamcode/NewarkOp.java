package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
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
@Disabled //DO NOT ENABLE UNLESS YOU HAVE ADI'S PERMISSION
public class NewarkOp extends LinearOpMode {

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
        // run until the end of the match (driver presses STOP)

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        boolean slowMode = false;
        boolean slowSlide = false;
        double startPosPivot = robot.pivotMotor.getCurrentPosition();
        boolean fallDetected = false;

        robot.mineralBlock.setPosition(0); //close

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
                    lPower /= 4;
                    rPower /= 4;
                } else {
                    lPower /= 3;
                    rPower /= 3;
                }
            }

            //gamepad 2 (logitech)
            double slide = -gamepad2.left_stick_y;
            double pivot = gamepad2.right_stick_y;
            double intake = ((gamepad2.right_trigger) - (gamepad2.left_trigger)) + 0.5;

            if (gamepad2.dpad_up) {
                robot.mineralBlock.setPosition(1); //close
            } else if (gamepad2.dpad_down) {
                robot.mineralBlock.setPosition(0); //open
            }

            /*if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                robot.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }*/

            if (gamepad2.right_bumper) { //preset to make slide extend to full position
                runtime.reset();
                robot.hexSlide.setTargetPosition(5600); //full position
                robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hexSlide.setPower(0.375);
                while (opModeIsActive() && robot.hexSlide.isBusy()) {
                    if ((gamepad2.right_bumper && runtime.seconds() > 1) || gamepad2.left_stick_y != 0) { //failsafe protection
                        robot.hexSlide.setPower(0);
                        break;
                    }
                    if (startPosPivot - robot.pivotMotor.getCurrentPosition() > 50) {
                        fallDetected = true;
                        robot.hexSlide.setTargetPosition(500); //full position
                        robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.hexSlide.setPower(.875);
                        while (opModeIsActive() && robot.hexSlide.isBusy()) {
                        }
                        robot.hexSlide.setPower(0);
                        robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    } else {
                        fallDetected = false;
                    }
                    throttle = ((gamepad1.right_trigger) - (gamepad1.left_trigger)); //continuation of motor power commands
                    steering = gamepad1.left_stick_x;
                    rPower = throttle - steering;
                    lPower = throttle + steering;
                    if (slowMode == true) {
                        if (steering != 0) {
                            lPower /= 3;
                            rPower /= 3;
                        } else {
                            lPower /= 2;
                            rPower /= 2;
                        }
                    }
                    pivot = gamepad2.right_stick_y;
                    intake = ((gamepad2.right_trigger) - (gamepad2.left_trigger)) + 0.5;

                    if (gamepad2.dpad_up) {
                        robot.mineralBlock.setPosition(1); //open
                    } else if (gamepad2.dpad_down) {
                        robot.mineralBlock.setPosition(0); //close
                    }
                    
                    //gamepad 1 & 2 (xbox) setPower
                    robot.hexFrontLeft.setPower(lPower/4);
                    robot.hexFrontRight.setPower(rPower/4);
                    robot.hexRearLeft.setPower(lPower/4);
                    robot.hexRearRight.setPower(rPower/4);
                    robot.pivotMotor.setPower(pivot/4);
                    robot.intakeServo.setPosition(intake);

                    telemetry.addData("Current Position", robot.hexSlide.getCurrentPosition()); //debugging
                    telemetry.addData("RunTime", runtime.seconds());
                    telemetry.update();
                }
                robot.hexSlide.setPower(0);
                robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(250);
            }
            if (gamepad2.left_bumper) { //WIP
                robot.hexSlide.setTargetPosition(1500);
                robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hexSlide.setPower(0.25);
                while (opModeIsActive() && robot.hexSlide.isBusy()) {
                    telemetry.addData(">", runtime.seconds());
                    telemetry.update();
                }
                sleep(1000);
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

            //gamepad 1 (xbox) setPower
            robot.hexFrontLeft.setPower(lPower);
            robot.hexFrontRight.setPower(rPower);
            robot.hexRearLeft.setPower(lPower);
            robot.hexRearRight.setPower(rPower);

            //gamepad 2 (logitech) setPower
            robot.hexSlide.setPower(slide);
            robot.pivotMotor.setPower(pivot/4);
            robot.intakeServo.setPosition(intake);

            telemetry.addData("slide input", gamepad2.left_stick_y);
            telemetry.addData("slide LIVE", robot.hexSlide.getCurrentPosition());
            telemetry.addData("intake", robot.intakeServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.addData("slowmode", slowMode);
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
