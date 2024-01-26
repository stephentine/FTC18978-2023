package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class TensorFlowAutonomous extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Red_X_Prop_V2.tflite";
    private static final String[] LABELS = {
       "Red X Prop",
    };
    private static final int MINIMUM_CONFIDENCE = 70;   // Minimum confidence for a Team Prop to be detected
    int teamPropLocation = 0;   // 0: Left, 1: Center, 2: Right


    // The variable to store our instance of the TensorFlow Object Detection processor.
    private TfodProcessor tfod;

    // The variable to store our instance of the vision portal.
    private VisionPortal visionPortal;
    

    @Override
    public void runOpMode() {

        initTfod();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetryTfod();
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        
        if (opModeIsActive()) {
            telemetry.addData("Spike Mark:", teamPropLocation);
            telemetry.update();
        }


    }   // end runOpMode()

    // Initialize the TensorFlow Object Detection processor.
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
            .setModelFileName(TFOD_MODEL_FILE)
            .setModelLabels(LABELS)
            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "BackWebcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }   // end method initTfod()

    // Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
        private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            
            // Determine where the Team Prop is located
            if (recognition.getConfidence()*100 > MINIMUM_CONFIDENCE) {
                if (x < 100) {
                    teamPropLocation = 0;                
                } else if (x > 500) {
                    teamPropLocation = 2;
                } else {
                    teamPropLocation = 1;
                }
            }

            telemetry.addData("Target Spike Mark:", teamPropLocation);


        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class
