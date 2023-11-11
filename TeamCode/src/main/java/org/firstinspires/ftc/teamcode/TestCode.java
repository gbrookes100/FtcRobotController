//Importing packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class TestCode extends LinearOpMode {

    //defining motor variables
    private DcMotor lf_motor = null;
    private DcMotor rf_motor = null;
    private DcMotor lb_motor = null;
    private DcMotor rb_motor = null;
    private DcMotor manipulator1 = null;
    private Servo fred;
    // fred is wrist, girlypop is manipulator
    private Servo GirlyPop;



    //defining imu variable
    IMU imu = null;

    //defining variables which are used later in the code
    double botHeading = 0;
    double cosineMove = 0;
    double sineMove = 0;
    double cosinePivot = 0;
    double sinePivot = 0;
    double x = 0;
    double y = 0;
    double rx = 0;
    double mag = 0;
    double startAngle = 0;

    @Override
    public void runOpMode() {

        //attaching motors to variables
        lf_motor = hardwareMap.get(DcMotor.class, "lf_motor");
        rf_motor = hardwareMap.get(DcMotor.class, "rf_motor");
        lb_motor = hardwareMap.get(DcMotor.class, "lb_motor");
        rb_motor = hardwareMap.get(DcMotor.class, "rb_motor");
        manipulator1 = hardwareMap.get(DcMotor.class, "manipulator1");

        GirlyPop = hardwareMap.get(Servo.class, "girlypop");
        fred = hardwareMap.get(Servo.class, "fred");

        //configuring motors so they move in the right direction
        lf_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        rf_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lb_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        rb_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        manipulator1.setDirection(DcMotorSimple.Direction.FORWARD);

        //attaching imu to variable and getting the gyroscope set up
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        //wait until the start button is clicked
        waitForStart();

        //calculating start angle (this is subtracted from the gyroscope so "forward" is always the direction the robot faces upon initialization
        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //loop that runs while the program is active
        while (opModeIsActive()) {

            //resets the gyroscope when button is clicked
            if (gamepad1.start) {
                imu.resetYaw();
            }

            //calculating bot heading using the gyroscope subtracted by the initial orientation. pi/4 is added to this, as mecanum wheels drive as if they are on a pi/4 radian angle
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + (Math.PI/4) - startAngle;

            //defining stick variables. Not sure why x has to be negated, I'll run some tests to figure it out
            x = -gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            //revolving the point (x,y) around the origin by the bot heading to get its linear speeds
            cosineMove = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            sineMove = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            //calculating the magnitude of the new points
            mag = Math.sqrt(cosineMove*cosineMove + sineMove*sineMove);

            //because the function divides by the magnitude, we have to run two functions if mag = 0 to avoid dividing by zero. Just so you know, the limit of the second function as x approaches 0 is 1, so they are essentially identical
            if (mag == 0) {

                //setting the rotation speeds to the right stick x variable
                cosinePivot = rx;
                sinePivot = rx;

            } else {

                //setting the rotation speeds to the right stick x variable multiplied by a chaning function which ensures that the motor speed can't exceed 0, resulting in smoother movement
                cosinePivot = rx * ( (1 - mag) + (sineMove*sineMove)/(2*mag) );
                sinePivot = rx * ( (1 - mag) + (cosineMove*cosineMove)/(2*mag) );

            }

            //Setting the motor speeds to their various linear + rotational speeds.
            lf_motor.setPower(cosineMove - cosinePivot);
            rf_motor.setPower(sineMove + sinePivot);
            rb_motor.setPower(cosineMove + cosinePivot);
            lb_motor.setPower(sineMove - sinePivot);

            if (gamepad1.a) {
                manipulator1.setPower(0.25);
            } else if (gamepad1.b) {
                manipulator1.setPower(-0.25);
            } else {
                manipulator1.setPower(0);
            }
            if (gamepad1.x) {
                GirlyPop.setPosition(0.15);
            }
            if (gamepad1.y) {
                GirlyPop.setPosition(0.1);
            }
            if (gamepad2.x) {
                fred.setPosition(0.3);
            }
            if (gamepad2.y) {
                fred.setPosition(-0.2);
            }
            telemetry.addData("girlypop pos", GirlyPop.getPosition());
            telemetry.update();

        }
    }
}