package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="CringemacAutoBlue", group="auto")
public class CringemacAutoBlue extends LinearOpMode {

    BNO055IMU imu;
    double globalAngle;
    Orientation lastAngles = new Orientation();

    hardwareMap mDrive = new hardwareMap();

    public final double WHEEL_DIAMETER = 4.0; //Wheel diameter in inches
    public final int MOTOR_GEAR_TEETH = 1; //# of teeth on the motor gear
    public final int WHEEL_GEAR_TEETH = 15; //# of teeth on the wheel gear
    public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI; //For every full turn of both motors, the wheel moves forward this many inches
    public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 28; //For every turn of the wheel


    int level;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        mDrive.init(hardwareMap);
        imu.initialize(parameters);

        Vision vision = new Vision(this, 'r');

        while (!isStarted()) {
            level = vision.levelIdent('r');
            telemetry.addData("Score Level: ", level);
            telemetry.update();
        }

        waitForStart();
        if (!isStopRequested()){
            switch (level) {
                case 1:
                    linearMovement(18, 1.5, 0.0004, 0.00007, 0.000068);
                    mDrive.Laft.setPower(1);
                    sleep(250);
                    mDrive.Laft.setPower(0);
                    mDrive.FL.setPower(-0.5); //right strafe
                    mDrive.BL.setPower(0.5); //right strafe
                    mDrive.FR.setPower(-0.5); //right strafe
                    mDrive.BR.setPower(0.5); //right strafe
                    sleep(800);
                    mDrive.BR.setPower(0);
                    mDrive.BL.setPower(0);
                    mDrive.FR.setPower(0);
                    mDrive.FL.setPower(0);
                    turnDegree(-12, 1);
                    mDrive.InOut.setPower(-1);
                    sleep(3000);
                    mDrive.InOut.setPower(1);
                    sleep(2000);
                    mDrive.InOut.setPower(-1);
                    sleep(2000);
                    mDrive.InOut.setPower(0);
                    sleep(1000);
                    mDrive.FL.setPower(0.25);
                    mDrive.FR.setPower(-0.25);
                    mDrive.BL.setPower(0.25);
                    mDrive.BR.setPower(-0.25);
                    sleep(150);
                    turnDegree(-70,0.5);
                    mDrive.FL.setPower(-0.5); //right strafe
                    mDrive.BL.setPower(0.5); //right strafe
                    mDrive.FR.setPower(-0.5); //right strafe
                    mDrive.BR.setPower(0.5); //right strafe
                    sleep(500);
                    mDrive.FL.setPower(1);
                    mDrive.FR.setPower(-1);
                    mDrive.BL.setPower(1);
                    mDrive.BR.setPower(-1);
                    sleep(850);
                    mDrive.Laft.setPower(-1);
                    sleep(250);
                    mDrive.Laft.setPower(0);
                    turnDegree(-35, 0.5);
                    linearMovement(-10,1,0.0004,0.00007,0.000068);
                    break;
                case 2:
                    linearMovement(18, 1.5, 0.0004, 0.00007, 0.000068);
                    mDrive.Laft.setPower(1);
                    sleep(550);
                    mDrive.Laft.setPower(0);
                    mDrive.FL.setPower(-0.5); //right strafe
                    mDrive.BL.setPower(0.5); //right strafe
                    mDrive.FR.setPower(-0.5); //right strafe
                    mDrive.BR.setPower(0.5); //right strafe
                    sleep(800);
                    mDrive.BR.setPower(0);
                    mDrive.BL.setPower(0);
                    mDrive.FR.setPower(0);
                    mDrive.FL.setPower(0);
                    turnDegree(-12, 1);
                    mDrive.InOut.setPower(-1);
                    sleep(3000);
                    mDrive.InOut.setPower(1);
                    sleep(2000);
                    mDrive.InOut.setPower(-1);
                    sleep(2000);
                    mDrive.InOut.setPower(0);
                    sleep(1000);
                    mDrive.FL.setPower(0.25);
                    mDrive.FR.setPower(-0.25);
                    mDrive.BL.setPower(0.25);
                    mDrive.BR.setPower(-0.25);
                    sleep(150);
                    turnDegree(-70,0.5);
                    mDrive.FL.setPower(-0.5); //right strafe
                    mDrive.BL.setPower(0.5); //right strafe
                    mDrive.FR.setPower(-0.5); //right strafe
                    mDrive.BR.setPower(0.5); //right strafe
                    sleep(500);
                    mDrive.FL.setPower(1);
                    mDrive.FR.setPower(-1);
                    mDrive.BL.setPower(1);
                    mDrive.BR.setPower(-1);
                    sleep(600);
                    mDrive.FL.setPower(0.5);
                    mDrive.FR.setPower(-0.5);
                    mDrive.BL.setPower(0.5);
                    mDrive.BR.setPower(-0.5);
                    sleep(350);
                    mDrive.Laft.setPower(-1);
                    sleep(500);
                    mDrive.Laft.setPower(0);
                    turnDegree(-35, 0.5);
                    linearMovement(-10,1,0.0004,0.00007,0.000068);
                    break;
                default:
                    linearMovement(20,1.5, 0.0004, 0.00007, 0.000068);
                    mDrive.Laft.setPower(1);
                    sleep(900);
                    mDrive.Laft.setPower(0);
                    mDrive.FL.setPower(-0.5); //right strafe
                    mDrive.BL.setPower(0.5); //right strafe
                    mDrive.FR.setPower(-0.5); //right strafe
                    mDrive.BR.setPower(0.5); //right strafe
                    sleep(825);
                    mDrive.BR.setPower(0);
                    mDrive.BL.setPower(0);
                    mDrive.FR.setPower(0);
                    mDrive.FL.setPower(0);
                    turnDegree(-12, 1);
                    mDrive.InOut.setPower(-1);
                    sleep(3000);
                    mDrive.InOut.setPower(1);
                    sleep(2000);
                    mDrive.InOut.setPower(-1);
                    sleep(2000);
                    mDrive.InOut.setPower(0);
                    sleep(1000);
                    mDrive.FL.setPower(0.25);
                    mDrive.FR.setPower(-0.25);
                    mDrive.BL.setPower(0.25);
                    mDrive.BR.setPower(-0.25);
                    sleep(150);
                    turnDegree(-70,0.5);
                    mDrive.FL.setPower(-0.5); //right strafe
                    mDrive.BL.setPower(0.5); //right strafe
                    mDrive.FR.setPower(-0.5); //right strafe
                    mDrive.BR.setPower(0.5); //right strafe
                    sleep(500);
                    mDrive.FL.setPower(1);
                    mDrive.FR.setPower(-1);
                    mDrive.BL.setPower(1);
                    mDrive.BR.setPower(-1);
                    sleep(650);
                    mDrive.FL.setPower(0.5);
                    mDrive.FR.setPower(-0.5);
                    mDrive.BL.setPower(0.5);
                    mDrive.BR.setPower(-0.5);
                    sleep(250);
                    mDrive.Laft.setPower(-1);
                    sleep(800);
                    mDrive.Laft.setPower(0);
                    turnDegree(-35, 0.5);
                    linearMovement(-10,1,0.0004,0.00007,0.000068);
                    break;
            }
        }
    }


    public void linearMovement(double distance, double timeframe, double kP, double kI, double kD) {
        double conversionIndex = 1104.04; // ticks per inch
        double timeFrame = timeframe; //distance * distanceTimeIndex;
        double errorMargin = 5;
        double powerFloor = 0;
        double powerCeiling = 1;

        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        mDrive.resetEncoders();

        double targetTick = -1 * distance * conversionIndex;
        telemetry.addData("target tick", targetTick);
        telemetry.update();


        double error = targetTick;
        double errorPrev = error;
        double time = clock.seconds();
        double timePrev = time;

        double  p, d, output;
        double i = 0;

        while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opModeIsActive()) {
            //output = linearPID.PIDOutput(targetTick,averageEncoderTick(),clock.seconds());

            errorPrev = error;
            timePrev = time;

            double tempAvg = targetTick > 0 ? mDrive.getEncoderAvg() : -mDrive.getEncoderAvg();

            error = targetTick - tempAvg;
            time = clock.seconds();
            //telemetry.addData("error", error);
            //telemetry.addData("time", time);

            p = Math.abs(error) / 33.0 * kP;
            i += (time - timePrev) * Math.abs(error) / 33.0 * kI;
            d = Math.abs((error - errorPrev) / (time - timePrev) / 33.0 * kD);
/*
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
*/

            output = p + i - d;
            telemetry.addData("output", output);
            output = Math.max(output, powerFloor);
            output = Math.min(output, powerCeiling);
            if (error < 0) output *= -1;

            double currentAngle = imu.getAngularOrientation().firstAngle;
            double raw = globalAngle - currentAngle;
            if (raw > 180)
                raw -= 360;
            if (raw < -180)
                raw += 360;
            double fudgeFactor = 1.0 - raw / 40.0;

            if (distance > 0)
            {
                mDrive.FL.setPower(output);
                mDrive.FR.setPower(-output);
                mDrive.BL.setPower(output);
                mDrive.BR.setPower(-output);

            }
            else
            {
                mDrive.FL.setPower(-output);
                mDrive.FR.setPower(output);
                mDrive.BL.setPower(-output);
                mDrive.BR.setPower(output);

            }


            telemetry.addData("error", error);
            telemetry.update();
        }
        mDrive.freeze();
        telemetry.speak("rip bozo plus ratio plus you fell off");
        telemetry.update();
    }

    public void turnDegree(double degree, double timeframe) {
        telemetry.addLine("made it");
        telemetry.update();
        lastAngles = imu.getAngularOrientation();
        double currentAngle = lastAngles.firstAngle;
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        globalAngle += degree;
        if (globalAngle > 180)
            globalAngle -= 360;
        if (globalAngle < -180)
            globalAngle += 360;
        double leftPower, rightPower;

        // restart imu movement tracking.
        //resetAngle();
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // set power to rotate.

        // rotate until turn is completed.

        double error = globalAngle - currentAngle;
        double errorPrev = error;

        double time = clock.seconds();
        double timePrev = time;

        double kP = 0.0118;
        double kI = 0.005;
        double kD = 0.002;

        double p, d, output;
        double i = 0;

        while (clock.seconds() < timeframe && Math.abs(error) > 1 && opModeIsActive()) {
            lastAngles = imu.getAngularOrientation();
            currentAngle = lastAngles.firstAngle;

            timePrev = time;
            errorPrev = error;

            time = clock.seconds();
            error = globalAngle - currentAngle;



            if (error > 180)
                error -= 360;
            if (error < -180)
                error += 360;

            p = Math.abs(error) * kP;
            i += (time - timePrev) * Math.abs(error) * kI;
            d = ((Math.abs(error) - Math.abs(errorPrev)) / (time - timePrev)) * kD;

            output = p + i + d;



            //telemetry.addData("output ", output);
            telemetry.addData("globalAngle", globalAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("error ", error);
            //telemetry.addData("p", p);
            //telemetry.addData("i", i);
            //telemetry.addData("d", d);*/
            telemetry.update();


            if (error > 0)
            {
                mDrive.FL.setPower(output);
                mDrive.BL.setPower(output);
                mDrive.FR.setPower(output);
                mDrive.BR.setPower(output * 0.35);
            }
            else
            {
                mDrive.FL.setPower(-output); //backwards
                mDrive.BL.setPower(-output); //backwards
                mDrive.FR.setPower(-output); //forwards
                mDrive.BR.setPower(-output * 0.35); //forwards
            }
        }
        mDrive.freeze();
    }

    public void floorit(double distance, double timeframe) { //literally why the fuck do you even exist
        double conversionIndex = NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION / MOTOR_TO_INCHES; // ticks per inch
        double timeFrame = timeframe; //distance * distanceTimeIndex;
        double errorMargin = 5;
        double powerFloor = 0;
        double powerCeiling = 1;

        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        mDrive.resetEncoders();

        double targetTick = -1 * distance * conversionIndex;
        telemetry.addData("target tick", targetTick);
        telemetry.update();

        double kP = 0.0004;
        double kI = 0.00007;
        double kD = 0.000068;

        double error = targetTick;
        double errorPrev = error;
        double time = clock.seconds();
        double timePrev = time;

        double  p, d, output;
        double i = 0;

        while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opModeIsActive()) {
            //output = linearPID.PIDOutput(targetTick,averageEncoderTick(),clock.seconds());

            errorPrev = error;
            timePrev = time;

            double tempAvg = targetTick > 0 ? mDrive.getEncoderAvg() : -mDrive.getEncoderAvg();

            error = targetTick - tempAvg;
            time = clock.seconds();
            //telemetry.addData("error", error);
            //telemetry.addData("time", time);

            p = Math.abs(error)  * kP;
            i += (time - timePrev) * Math.abs(error) * kI;
            d = Math.abs((error - errorPrev) / (time - timePrev) * kD);

            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);


            output = p + i + d;
            telemetry.addData("output", output);
            output = Math.max(output, powerFloor);
            output = Math.min(output, powerCeiling);
            if (error < 0) output *= -1;

            double currentAngle = imu.getAngularOrientation().firstAngle;
            double raw = globalAngle - currentAngle;
            if (raw > 180)
                raw -= 360;
            if (raw < -180)
                raw += 360;
            double fudgeFactor = 1 - raw / 30;

            mDrive.FL.setPower(1);
            mDrive.FR.setPower(-1);
            mDrive.BL.setPower(1);
            mDrive.BR.setPower(-1);

            telemetry.addData("error", error);
            telemetry.update();
        }
        mDrive.freeze();
        telemetry.addData("movement", " done.");
        telemetry.update();

    }
}
