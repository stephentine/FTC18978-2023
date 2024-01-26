package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp
public class BasicTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx m1,m2,m3,m4,intake,slide,plane,hanger;
    //DistanceSensor distance;
    Servo arm,claw;

    public void runOpMode() {

        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        slide = (DcMotorEx) hardwareMap.dcMotor.get("viper_slide");
        plane = (DcMotorEx) hardwareMap.dcMotor.get("plane_launcher");
        hanger = (DcMotorEx) hardwareMap.dcMotor.get("hanger");
        arm = (Servo) hardwareMap.servo.get("drop_arm");
        claw = (Servo) hardwareMap.servo.get("claw");
        
        //Set them to the correct modes
        //This reverses the motor direction
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m4.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse the viper slide
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        //This resets the encoder values when the code is initialized
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get the distance sensor and motor from hardwareMap
        //distance = hardwareMap.get(DistanceSensor.class, "distance");

        // Set default intake toggle state
        double intakeState = 0.0;
        int bumperToggleState = 0;
        
        // Set time requirement for plane launcher
        int planeButtonTimer = 0;

        waitForStart();

        while(opModeIsActive()) {
            // Mecanum drive code
            double px = 0.0;
            double py = 0.0;
            double pa = 0.0;
            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
                // This allows for driving via dpad as well
                // Uses ternaries to make the code more compact (condition ? if true : if false)
                px = gamepad1.dpad_left ? -1.0 : 0.0;
                px = gamepad1.dpad_right ? 1.0 : px;
                py = gamepad1.dpad_down ? -1.0 : 0.0;
                py = gamepad1.dpad_up ? 1.0 : py;
            } else {
                // If the dpad is not in use, drive via sticks
                px = gamepad1.left_stick_x;
                py = -gamepad1.left_stick_y;
                pa = gamepad1.right_stick_x;
            }
            
            if (gamepad1.x) {
                px *= 0.5;
                py *= 0.5;
                pa *= 0.5;
            }
            double p1 = px + py - pa;
            double p2 = -px + py + pa;
            double p3 = -px + py - pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);

            // Distance sensor cutoff test
            // If the robot is travelling backwards and is too close to a wall, it will stop
            // The backwards specification should allow the robot to drive forward away from the wall
            /*
            if ((p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) && (distance.getDistance(DistanceUnit.CM) < 25)) {
                m1.setPower(0.0);
                m2.setPower(0.0);
                m3.setPower(0.0);
                m4.setPower(0.0);
            } else {
                m1.setPower(p1);
                m2.setPower(p2);
                m3.setPower(p3);
                m4.setPower(p4);
            }
            */

            // Intake toggle
            if (gamepad1.right_bumper) {
                if (bumperToggleState == 0) {
                    bumperToggleState = 1;
                } else if (bumperToggleState == 1) {
                    // Toggle intake on edge
                    if (intakeState == 0.0) {
                        intakeState = 1.0;
                    } else {
                        intakeState = 0.0;
                    }
                    bumperToggleState = 2;
                }
            } else {
                bumperToggleState = 0;
            }
            
            if (gamepad1.left_bumper) {
                // Run the intake backwards
                intake.setPower(1.0);
            } else {
                // Follow the toggle variable
                intake.setPower(-intakeState);
            }
            
            // Sync the claw with the intake
            if (intakeState == 0.0) {
                claw.setPosition(0.01);
            } else {
                claw.setPosition(0.15);
            }
    
            // Viper Slide Manual Controls
            slide.setPower(0.0);
            if (gamepad2.y) {
                slide.setPower(1.0);
            } else if (gamepad2.a) {
                slide.setPower(-1.0);
            }

            // Drop arm positioning
            // 0.8 is 90 degree angle
            // 0.44 is default position
            if (gamepad2.right_trigger > 0.8) {
                arm.setPosition(0.8);
            } else if (gamepad2.right_trigger < 0.44) {
                arm.setPosition(0.44);
            } else {
                arm.setPosition(gamepad2.right_trigger);
            }
            
            // Plane launcher
            if (gamepad2.left_trigger > 0.0) {
                planeButtonTimer += 1;
            } else {
                planeButtonTimer = 0;
            }
            if (planeButtonTimer > 100) {
                plane.setPower (1.0);
            } else {
                plane.setPower(0.0);
            }
            
            
            // Hanging arm
            if (gamepad1.left_trigger > 0.8) { 
                hanger.setPower(1.0);
            } else if (gamepad1.right_trigger > 0.8) {
                hanger.setPower(-1.0);
            } else {
                hanger.setPower(0.0);
            }
            
            /*
            if (gamepad1.left_trigger > 0.0) { 
                hanger.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.0) {
                hanger.setPower(-gamepad1.right_trigger);
            } else {
                hanger.setPower(0.0);
            }
            */
        }
    }
}
