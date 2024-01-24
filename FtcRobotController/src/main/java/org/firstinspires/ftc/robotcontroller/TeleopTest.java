package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleopTest extends RobotNew LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor rightElevator = hardwareMap.dcMotor.get("rightElevator");
        DcMotor leftElevator = hardwareMap.dcMotor.get("leftElevator");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        // Servo intakeAngle = hardwareMap.Servo.get("intakeAngle");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {



            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;
            double i = gamepad1.right_trigger;
            //double ia gamepad1.right_bumper;

            elapsedTime.reset();
            while (opModeIsActive() && !isStopRequested()) {


                double botHeading = -imu.getAngularOrientation().firstAngle /*- Math.PI*/;

                double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);



                double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
                double lfPower = (rotY + rotX + rx) /denominator;
                double lbPower = (rotY - rotX + rx) /denominator;
                double rfPower = (rotY - rotX - rx) /denominator;
                double rbPower = (rotY + rotX - rx) /denominator;


//            drive.lf.setPower(/*y + x + rx*/lfPower);
//            drive.lb.setPower(/*y - x + rx*/lbPower);
//            drive.rf.setPower(/*y - x - rx*/rfPower);
//            drive.rb.setPower(/*y + x - rx*/rbPower);

                if (gamepad1.circle){
                    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                    imu.initialize(parameters);

                }



                // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double rightElevatorPower =ry;
            double leftElevatorPower=ry;
            double intakePower=i;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            intake.setPower(intakePower);
            if(ry>0.5){
                rightElevator.setPower(0.5);
                leftElevator.setPower(0.5);

            }
            else{
                rightElevator.setPower(rightElevatorPower);
                leftElevator.setPower(leftElevatorPower);
            }

        }
    }
}