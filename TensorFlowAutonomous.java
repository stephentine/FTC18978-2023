package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

    DcMotorEx m1,m2,m3,m4,intake,hanger;
    Servo auto_claw;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Red_X_Prop_V2.tflite";
    private static final String[] LABELS = {
            "Red X Prop",
    };
    private static final int MINIMUM_CONFIDENCE = 70;   // Minimum confidence for a Team Prop to be detected
    int teamPropLocation = 0;   // 0: Left, 1: Center, 2: Right

    //Math to turn encoder ticks into distance
    static final double TICKS_PER_REV_MOTOR = 384.5;
    static final double GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_MM = 96.0;
    //25.4 is the conversion factor between mm and in
    static final double WHEEL_CIR_IN = WHEEL_DIAMETER_MM / 25.4 * Math.PI;

    static final double TICKS_PER_REV_WHEEL = TICKS_PER_REV_MOTOR * GEAR_REDUCTION;
    static final double TICKS_PER_IN = TICKS_PER_REV_WHEEL / WHEEL_CIR_IN;

    //Set turning constants to allow for rough degree calculations
    static final double TICKS_PER_QUARTER_TURN = 648.0;
    static final double TICKS_PER_DEGREE = TICKS_PER_QUARTER_TURN / 90;

    //Field Constants
    static final double INCHES_PER_FIELD_TILE = 24.0;

    //Set wheel velocity (RPM/60 = RPS)
    double VELOCITY_RPM = 100;
    double TICKS_PER_SEC = (VELOCITY_RPM / 60) * TICKS_PER_REV_WHEEL;

    //Set variable to determine route
    //0: Blue far, 1: Blue close, 2: Red far, 3: Red close
    int driveRoute = 2;


    // The variable to store our instance of the TensorFlow Object Detection processor.
    private TfodProcessor tfod;

    // The variable to store our instance of the vision portal.
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {

        //Set up motors
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        hanger = (DcMotorEx) hardwareMap.dcMotor.get("hanger");
        auto_claw = (Servo) hardwareMap.servo.get("auto_claw");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m4.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        initTfod();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetryTfod();
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }


        if (opModeIsActive()) {
            telemetry.addData("Spike Mark:", teamPropLocation);
            telemetry.update();

            // DRIVE CODE

            driveTiles(-1.0,TICKS_PER_SEC);
            if (teamPropLocation == 0) {
                // Left
                turnEnc(90,TICKS_PER_SEC);
            } else if (teamPropLocation == 1) {
                // Center
                turnEnc(180,TICKS_PER_SEC);
            } else {
                // Right
                turnEnc(-90,TICKS_PER_SEC);
            }

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

        }


    }   // end runOpMode()

    // Initialize the TensorFlow Object Detection processor
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


    // Autonomous Functions
    //Turn a number of given degrees based on a rough 90-degree turn constant
    public void turnEnc(int degrees, double vel) {
        m1.setTargetPosition((int)(-degrees * TICKS_PER_DEGREE));
        m2.setTargetPosition((int)(degrees * TICKS_PER_DEGREE));
        m3.setTargetPosition((int)(-degrees * TICKS_PER_DEGREE));
        m4.setTargetPosition((int)(degrees * TICKS_PER_DEGREE));

        runAllToPosition();
        m1.setVelocity(vel);
        m2.setVelocity(vel);
        m3.setVelocity(vel);
        m4.setVelocity(vel);

        while(m1.isBusy() || m2.isBusy() || m3.isBusy() || m4.isBusy());
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Drive a given distance in inches
    public void driveEnc(int inches, double vel) {
        //motor.setTargetPosititon: set the desired encoder value that you want the motor to hit
        m1.setTargetPosition((int)(inches * TICKS_PER_IN));
        m2.setTargetPosition((int)(inches * TICKS_PER_IN));
        m3.setTargetPosition((int)(inches * TICKS_PER_IN));
        m4.setTargetPosition((int)(inches * TICKS_PER_IN));

        runAllToPosition();
        m1.setVelocity(vel);
        m2.setVelocity(vel);
        m3.setVelocity(vel);
        m4.setVelocity(vel);

        while(m1.isBusy() || m2.isBusy() || m3.isBusy() || m4.isBusy());
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Drive a given number of field tiles
    public void driveTiles(double tiles, double vel) {
        //motor.setTargetPosititon: set the desired encoder value that you want the motor to hit
        m1.setTargetPosition((int)(tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));
        m2.setTargetPosition((int)(tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));
        m3.setTargetPosition((int)(tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));
        m4.setTargetPosition((int)(tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));

        runAllToPosition();
        m1.setVelocity(vel);
        m2.setVelocity(vel);
        m3.setVelocity(vel);
        m4.setVelocity(vel);

        while(m1.isBusy() || m2.isBusy() || m3.isBusy() || m4.isBusy());
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //Strafe a given distance in inches
    public void strafeEnc(int inches, double vel) {
        //motor.setTargetPosititon: set the desired encoder value that you want the motor to hit
        m1.setTargetPosition((int)(-inches * TICKS_PER_IN));
        m2.setTargetPosition((int)(inches * TICKS_PER_IN));
        m3.setTargetPosition((int)(inches * TICKS_PER_IN));
        m4.setTargetPosition((int)(-inches * TICKS_PER_IN));

        runAllToPosition();
        m1.setVelocity(vel);
        m2.setVelocity(vel);
        m3.setVelocity(vel);
        m4.setVelocity(vel);

        while(m1.isBusy() || m2.isBusy() || m3.isBusy() || m4.isBusy());
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Strafe a given number of field tiles
    public void strafeTiles(double tiles, double vel) {
        //motor.setTargetPosititon: set the desired encoder value that you want the motor to hit
        m1.setTargetPosition((int)(-tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));
        m2.setTargetPosition((int)(tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));
        m3.setTargetPosition((int)(tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));
        m4.setTargetPosition((int)(-tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));

        runAllToPosition();
        m1.setVelocity(vel);
        m2.setVelocity(vel);
        m3.setVelocity(vel);
        m4.setVelocity(vel);

        while(m1.isBusy() || m2.isBusy() || m3.isBusy() || m4.isBusy());
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Makes other functions more compact
    public void runAllToPosition() {
        //RUN_TO_POSITION: tells the motor to run the the target position
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}   // end class