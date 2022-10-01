package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor; 

@TeleOp(name="Vikrtamimo", group="Arcade")

public class Cringemac extends LinearOpMode{

    static hardwareMap mDrive = new hardwareMap();
    static double frontLeftP, frontRightP, backLeftP, backRightP;

    public void runOpMode(){
        mDrive.init(hardwareMap);
        mDrive.resetEncoders();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            lift(gamepad2.left_stick_y);
            intakeOuttake(gamepad2.left_trigger, gamepad2.right_trigger); //switch the inputs if Henry doesn't like the new controls
            duck(gamepad2.right_stick_x);
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            drive.update();

            telemetry.addData("Encoder: ", mDrive.Laft.getCurrentPosition());
            telemetry.addData("Encoder: ", mDrive.Duckie.getCurrentPosition());
            telemetry.addData("Encoder: ", mDrive.Filler2.getCurrentPosition());
            telemetry.addData("Encoder: ", mDrive.Filler1.getCurrentPosition());
            telemetry.update();
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

    public static void lift(double lp){
        double liftPower = lp;

        mDrive.Laft.setPower(-liftPower);
    }

    public static void intakeOuttake(double lt, double rt){
        double power = lt - rt;

        mDrive.InOut.setPower(power);
    }

    public static void duck(double ls){
        double power = ls;
        power /= 3;

        mDrive.Duckie.setPower(power);
    }
}