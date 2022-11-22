package org.firstinspires.ftc.teamcode.drive.opmode;

import android.text.method.Touch;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(group = "drive")

public class TeleOp_Test_S7 extends LinearOpMode {
    double mls(double x) //modul de liniaritate si sens
    {
        return x*x*x;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        double sensivity=0.75, position=0.5, movementSensitivity=0.6, grip=0;
        boolean u1=true, u2=true, s1=true, s2=true;
        DcMotor umard = hardwareMap.dcMotor.get("umarDreapta");
        DcMotor umars= hardwareMap.dcMotor.get("umarStanga");
        DcMotor holder = hardwareMap.dcMotor.get("motorCentral");
        Servo cots = hardwareMap.get(Servo.class, "cotStanga");
        Servo cotd = hardwareMap.get(Servo.class, "cotDreapta");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        umars.setDirection(DcMotorSimple.Direction.REVERSE);
        cots.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (!isStopRequested()) {
            //movement
            drive.setWeightedDrivePower(
                    new Pose2d(
                            mls(gamepad1.left_stick_y) * movementSensitivity,
                            mls(gamepad1.left_stick_x) * movementSensitivity,
                            gamepad1.right_stick_x * movementSensitivity
                    )
            );

            //one push
            //pozitie antebrat
            if (gamepad2.dpad_up && s1 && position<0.9) {
                position += 0.1;
                s1 = false;
            }
            else
                s1 = true;
            if (gamepad2.dpad_down && s2 && position>0.1) {
                position -= 0.1;
                s2 = false;
            }
            else
                s2 = true;
            telemetry.addData("Pozitie servo cot:", position);
            telemetry.addData("Unghi servo cot:", position*270);

            //sensivitate brat
            if (gamepad2.right_bumper && u1 && sensivity<1) {
                sensivity += 0.05;
                u1 = false;
            }
            else
                u1 = true;
            if (gamepad2.left_bumper && u2 && sensivity>0.4) {
                sensivity -= 0.05;
                u2 = false;
            }
            else
                u2 = true;
            telemetry.addData("Sensivitate brat:", sensivity);

            //control gripper
            if (gamepad2.y) // pick
                grip=0;
            if (gamepad2.x) // drop
                grip=0.2;
            //control si atribuire
            cotd.setPosition(position);
            cots.setPosition(position);
            gripper.setPosition(grip);
            umard.setPower(gamepad2.left_stick_y*sensivity);
            umars.setPower(gamepad2.left_stick_y*sensivity);
            holder.setPower(gamepad2.left_stick_y*sensivity);

            //indicatii
//            telemetry.addData("contorl sensivitate brat: BUMPER");
//            telemetry.addData("pozitie servo: Y/A");
//            telemetry.addData("range servo [0.1, 0.9]");
            telemetry.update();
        }
    }
}
