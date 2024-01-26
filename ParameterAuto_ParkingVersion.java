/*
This code assumes that you have a pixel preloaded in the intake
and another hooked on the hanging arm (the arm lifts, the pixel falls).

We guess at the spike mark (because no Vision yet) to spit out the first pixel
and drive to the proper parking spot to deposit the second pixel


POSITIVE IS CLOCKWISE
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ParameterAuto_ParkingVersion extends LinearOpMode {
//public class PenguinsAutoBase extends LinearOpMode {

    DcMotorEx m1,m2,m3,m4,intake,hanger;
    Servo servo;

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

    public void runOpMode() {
        //Set up motors
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        hanger = (DcMotorEx) hardwareMap.dcMotor.get("hanger");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m4.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set wheel velocity (RPM/60 = RPS)
        double VELOCITY_RPM = 100;
        double TICKS_PER_SEC = (VELOCITY_RPM / 60) * TICKS_PER_REV_WHEEL;

        //Set variable to determine route
        //0: Blue far, 1: Blue close, 2: Red far, 3: Red close
        int driveRoute = 2;

        //Set variable to determine spike mark (will be set via vision)
        //0: Left, 1: Center, 2: Right
        int targetSpike = 0;

        //Telemetry: information that can print onto the DriverHub
        //telemetry.addData adds the data, so you need to update it
        telemetry.addData("Press start when ready","");
        telemetry.update();
        waitForStart();

        if(opModeIsActive()) {
            if (targetSpike == 0) {
                //LEFT spike mark
                driveTiles(-1.2,TICKS_PER_SEC);
                turnEnc(90,TICKS_PER_SEC);
                    //Deposit pixel
                    intake.setPower(0.2);
                    sleep(1000);
                    intake.setPower(0.0);
                driveEnc(-2,TICKS_PER_SEC);
                turnEnc(-90,TICKS_PER_SEC);
                strafeEnc(2,TICKS_PER_SEC);
                driveTiles(-1.1,TICKS_PER_SEC);
            } else if (targetSpike == 1) {
                //CENTER spike mark
                driveTiles(-1.9,TICKS_PER_SEC);
                    //Deposit pixel
                    intake.setPower(0.2);
                    sleep(1000);
                    intake.setPower(0.0);
                driveTiles(-0.4,TICKS_PER_SEC);
            } else {
                //RIGHT spike mark
                driveTiles(-1.2,TICKS_PER_SEC);
                turnEnc(-90,TICKS_PER_SEC);
                    //Deposit pixel
                    intake.setPower(0.2);
                    sleep(1000);
                    intake.setPower(0.0);
                driveEnc(-2,TICKS_PER_SEC);
                turnEnc(90,TICKS_PER_SEC);
                strafeEnc(-2,TICKS_PER_SEC);
                driveTiles(-1.1,TICKS_PER_SEC);
            }

            if (driveRoute == 0) {
                //Drive from FAR to RED parking zone
                turnEnc(90,TICKS_PER_SEC);
                driveTiles(4,TICKS_PER_SEC);
                    //Deposit pixel -->
            } else if (driveRoute == 1) {
                //Drive from CLOSE to RED parking zone
                turnEnc(90,TICKS_PER_SEC);
                driveTiles(2,TICKS_PER_SEC);
                    //Deposit pixel -->
            } else if (driveRoute == 2) {
                //Drive from FAR to BLUE parking zone
                turnEnc(-90,TICKS_PER_SEC);
                driveTiles(4,TICKS_PER_SEC);
                    //Deposit pixel -->
            } else {
                //Drive from CLOSE to BLUE parking zone
                turnEnc(-90,TICKS_PER_SEC);
                driveTiles(2,TICKS_PER_SEC);
                    //Deposit pixel -->
            }
            //Deposit pixel
            // Lift hanger to drop pixel
            hanger.setPower(1.0);
            sleep(100);
            hanger.setPower(0.0);
            // Wait for the pixel to fall
            sleep(700);
            // Drop hanger to starting position
            hanger.setPower(-1.0);
            sleep(100);
            hanger.setPower(0.0);

        }
    }

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
}
