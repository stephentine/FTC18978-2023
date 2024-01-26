package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// POSITIVE IS COUNTER-CLOCKWISE

@Autonomous
//public class BasicAutoTest extends LinearOpMode {
public class PenguinsAutoBase extends LinearOpMode {

    DcMotorEx m1,m2,m3,m4;
    Servo servo;

    //Math to turn encoder ticks into distance
    static final double TICKS_PER_REV_MOTOR = 384.5;
    static final double GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_MM = 96.0;
    static final double WHEEL_CIR_IN = WHEEL_DIAMETER_MM / 25.4 * Math.PI;

    static final double TICKS_PER_REV_WHEEL = TICKS_PER_REV_MOTOR * GEAR_REDUCTION;
    static final double TICKS_PER_IN = TICKS_PER_REV_WHEEL / WHEEL_CIR_IN;

    //Set turning constants to allow for rough degree calculations
    static final double TICKS_PER_QUARTER_TURN = 648.0;
    static final double TICKS_PER_DEGREE = TICKS_PER_QUARTER_TURN / 90;

    //Set drive constants for field tiles
    static final double INCHES_PER_FIELD_TILE = 24.0;

    public void runOpMode() {
        //Set up motors
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set wheel velocity (RPM/60 = RPS)
        double VELOCITY_RPM = 100;
        double TICKS_PER_SEC = (VELOCITY_RPM / 60) * TICKS_PER_REV_WHEEL;

        //Telemetry: information that can print onto the DriverHub
        //telemetry.addData adds the data, so you need to update it
        telemetry.addData("Press start when ready","");
        telemetry.update();
        waitForStart();

        if(opModeIsActive()) {
            for(int i=0;i<4;i++) {
                driveEnc(22,TICKS_PER_SEC);
                turnEnc(90,TICKS_PER_SEC);
            }
        }
    }

    //Turn a number of given degrees based on a rough 90-degree turn constant
    public void turnEnc(int degrees, double vel) {
        m1.setTargetPosition((int)(degrees * TICKS_PER_DEGREE));
        m2.setTargetPosition((int)(-degrees * TICKS_PER_DEGREE));
        m3.setTargetPosition((int)(degrees * TICKS_PER_DEGREE));
        m4.setTargetPosition((int)(-degrees * TICKS_PER_DEGREE));

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
        m1.setTargetPosition((int)(inches * TICKS_PER_IN));
        m2.setTargetPosition((int)(-inches * TICKS_PER_IN));
        m3.setTargetPosition((int)(-inches * TICKS_PER_IN));
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

    //Strafe a given number of field tiles
    public void strafeTiles(double tiles, double vel) {
        //motor.setTargetPosititon: set the desired encoder value that you want the motor to hit
        m1.setTargetPosition((int)(tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));
        m2.setTargetPosition((int)(-tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));
        m3.setTargetPosition((int)(-tiles * INCHES_PER_FIELD_TILE * TICKS_PER_IN));
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

    //Makes other functions more compact
    public void runAllToPosition() {
        //RUN_TO_POSITION: tells the motor to run the the target position
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
