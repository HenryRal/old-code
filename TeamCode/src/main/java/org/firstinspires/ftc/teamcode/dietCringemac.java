package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="dietCringemac", group="Arcade")

public class dietCringemac extends LinearOpMode{

    static hardwareMap mDrive = new hardwareMap();
    static double frontLeftP, frontRightP, backLeftP, backRightP;

    public void runOpMode(){
        mDrive.init(hardwareMap);
        mDrive.resetEncoders();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            lift(gamepad1.dpad_up, gamepad1.dpad_down);
            intakeOuttake(gamepad1.left_trigger, gamepad1.right_trigger); //switch the inputs if Henry doesn't like the new controls
            duck(gamepad1.b, gamepad1.x);
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            drive.update();
        }
    }

    public static void drive(double x, double y, double t)
    {
        double magnitude = Math.sqrt(x * x + y * y);
        double distance = Math.atan2(y, x);
        double turn =  2 * t / 3.0;

        double backLeft = magnitude * Math.sin(distance - Math.PI / 4) + turn;
        double backRight = magnitude * Math.sin(distance + Math.PI / 4) - turn;
        double frontLeft = magnitude * Math.sin(distance + Math.PI / 4) + turn;
        double frontRight = magnitude * Math.sin(distance - Math.PI / 4) - turn;

        if (magnitude != 0) {
            double divisor = 0;
            divisor = Math.max(Math.abs(backLeft), Math.abs(backRight));
            divisor = Math.max(divisor, Math.abs(frontLeft));
            divisor = Math.max(divisor, Math.abs(frontRight));

            backLeft = magnitude * (backLeft / divisor);
            backRight = magnitude * (backRight / divisor);
            frontLeft = magnitude * (frontLeft / divisor);
            frontRight = magnitude * (frontRight / divisor);
        }

        mDrive.BL.setPower(backRight * 0.025);
        mDrive.BR.setPower(-backLeft * 0.025);
        mDrive.FL.setPower(frontRight * 0.025);
        mDrive.FR.setPower(-frontLeft * 0.025);

        frontLeftP = frontLeft * 0.025;
        frontRightP = frontRight * 0.025;
        backLeftP = backLeft * 0.025;
        backRightP = backRight * 0.025;
    }

    public static void lift(boolean up, boolean down){
        double liftPower;
        if (up){
            liftPower = -1.0;
        }
        else if (down){
            liftPower = 1.0;
        }
        else{
            liftPower = 0;
        }
        mDrive.Laft.setPower(liftPower);
    }

    public static void intakeOuttake(double lt, double rt){
        double power = lt - rt;

        mDrive.InOut.setPower(power);
    }

    public static void duck(boolean br, boolean bl){
        double power;
        if (br == true){
            power = 0.5;
        }
        else if (bl == true){
            power = -0.5;
        }
        else{
            power = 0;
        }

        mDrive.Duckie.setPower(power);
    }
}