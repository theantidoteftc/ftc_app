package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
public class EncodersOp extends LinearOpMode {

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

        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        double fLStart = robot.hexFrontLeft.getCurrentPosition();
        double fRStart = robot.hexFrontRight.getCurrentPosition();
        double rLStart = robot.hexRearLeft.getCurrentPosition();
        double rRStart = robot.hexRearRight.getCurrentPosition();

        double sStart = robot.hexSlide.getCurrentPosition();
        double pStart = robot.pivotMotor.getCurrentPosition();

        robot.mineralGate.setPosition(0); //close

        while (opModeIsActive()) {

            //gamepad 1 (xbox)
            double throttle = ((gamepad1.right_trigger) - (gamepad1.left_trigger));
            double steering = gamepad1.left_stick_x;

            double rPower = throttle - steering;
            double lPower = throttle + steering;

            //gamepad 1 (xbox) setPower
            robot.hexFrontLeft.setPower(lPower/20);
            robot.hexFrontRight.setPower(rPower/20);
            robot.hexRearLeft.setPower(lPower/20);
            robot.hexRearRight.setPower(rPower/20);

            //gamepad 2 (logitech)
            double slide = -gamepad2.left_stick_y;
            double pivot = gamepad2.right_stick_y;

            //gamepad 2 (logitech) setPower
            robot.hexSlide.setPower(slide);
            robot.pivotMotor.setPower(pivot/4);

            telemetry.addData("Delta Front Left", robot.hexFrontLeft.getCurrentPosition() - fLStart);
            telemetry.addData("Delta Front Right", robot.hexFrontRight.getCurrentPosition() - fRStart);
            telemetry.addData("Delta Rear Left", robot.hexRearLeft.getCurrentPosition() - rLStart);
            telemetry.addData("Delta Rear Right", robot.hexRearRight.getCurrentPosition() - rRStart);
            telemetry.addData("Delta Hex Slide", robot.hexSlide.getCurrentPosition() - sStart);
            telemetry.addData("Delta Pivot Motor", robot.pivotMotor.getCurrentPosition() - pStart);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
