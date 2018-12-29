package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp(name = "TensorFlowTest", group = "Concept")
//@Disabled
public class TensorFlowTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //Show Vuforia our Vuforia license key
    private static final String VUFORIA_KEY = "AVF7OF7/////AAABmaKBSYRMHkclubr6nFb2TLcr3QzadzX163OzDe2NS0p2hQlEvibYh8W2xO78LrAUPInfApVZ1qzOxq7fnHZ9KQ0QiJM0E5WbwxdY7U+Gbrk8NuDgceoPw7eD8j2Sk7NuvuTcXYAAoA4wKwgDlw+iA19frB/9/WuUonCWiMAi+sxSoAGkudWAx8f1AO0AXBNyf6d0QHRGVeGRyMYtvkvsez3kU6U7LnMUwpDkX5RfQi+AMKq+BTLYtOo90waG5G84TV9LU1OSlDHtPh7sSG6YuVdn0Pmm/+k9nEtedozzDeDmwKfT1A5uL1m+RGmgCe4gA45H7qH6p9ymyKDbhvfDbTo/fVI0Y9g+Z8FEMByMnI6X";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // Initiate the Vuforia engine
        initVuforia();

        // Checks if phone is compatible with TensorFlow
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            //Initiate TensorFLow
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Activate Tensor Flow Object Detection.
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {

                if (tfod != null) {

                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {

                        telemetry.addData("Objects detected", "none");

                        // if the webcam sees one or more minerals
                        if (updatedRecognitions.size() >= 1) {

                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addData("Gold: ", "found");
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AVF7OF7/////AAABmaKBSYRMHkclubr6nFb2TLcr3QzadzX163OzDe2NS0p2hQlEvibYh8W2xO78LrAUPInfApVZ1qzOxq7fnHZ9KQ0QiJM0E5WbwxdY7U+Gbrk8NuDgceoPw7eD8j2Sk7NuvuTcXYAAoA4wKwgDlw+iA19frB/9/WuUonCWiMAi+sxSoAGkudWAx8f1AO0AXBNyf6d0QHRGVeGRyMYtvkvsez3kU6U7LnMUwpDkX5RfQi+AMKq+BTLYtOo90waG5G84TV9LU1OSlDHtPh7sSG6YuVdn0Pmm/+k9nEtedozzDeDmwKfT1A5uL1m+RGmgCe4gA45H7qH6p9ymyKDbhvfDbTo/fVI0Y9g+Z8FEMByMnI6X";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

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

