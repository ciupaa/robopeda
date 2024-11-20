package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "TeleOpCiupa9", group = "Robot")
//@Disabled
public class TeleOpCiupa9 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  fata_stanga   = null; //the left drivetrain motor
    public DcMotor  fata_dreapta  = null; //the right drivetrain motor
    public DcMotor  spate_stanga    = null;
    public DcMotor  spate_dreapta   = null;
    public DcMotor  motor_stanga         = null; //the arm motor
    public DcMotor  motor_glisiere        = null; //
    public CRServo  servoRotire           = null; //the active servoRotire servo
    public Servo    cleste            = null; //the cleste servo


    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 99.5 // This is the exact gear ratio of the gobilda 60rpm motor
                    * 10 // This is the external gear reduction
                    * 1/360.0; // we want ticks per degree, not per rotation


    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double cosSusBrat = 100 * ARM_TICKS_PER_DEGREE;
    final double cosJosBrat = 90 * ARM_TICKS_PER_DEGREE;
    final double SpecimenBrat = 90 * ARM_TICKS_PER_DEGREE;
    final double Hang = 110 * ARM_TICKS_PER_DEGREE;
    final double test = 10 * ARM_TICKS_PER_DEGREE;

    final double servoRetras = 0;
    final double servoTras = 0;

    /* Variables to store the positions that the cleste should be set to when folding in, or folding out. */
    final double cleste_deschis   = 0.7;
    final double cleste_inchis  = 1;

    /* A number in degrees that the triggers can adjust the arm position by */


    /* Variables that are used to set the arm to a specific position */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;


    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;


    double rotirePosition = (int)servoTras;


    final double LIFT_TICKS_PER_MM = (7363.2 / 72.0);


    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;
    double liftPosition = LIFT_COLLAPSED;


    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;


    @Override
    public void runOpMode() {

        /* Define and Initialize Motors */
        fata_stanga    = hardwareMap.dcMotor.get("fata_stanga");
        spate_stanga   = hardwareMap.dcMotor.get("spate_stanga");
        fata_dreapta   = hardwareMap.dcMotor.get("fata_dreapta");
        spate_dreapta  = hardwareMap.dcMotor.get("spate_dreapta");
        motor_glisiere = hardwareMap.dcMotor.get("motor_glisiere");
        motor_stanga   = hardwareMap.get(DcMotor.class, "motor_stanga"); //the arm motor


       /*
       we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
       drive motors to go forward.
        */

        fata_stanga.setDirection(DcMotor.Direction.REVERSE);
        spate_stanga.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_glisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) motor_stanga).setCurrentAlert(5,CurrentUnit.AMPS);

        /* Before starting the motor_stanga. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        motor_stanga.setTargetPosition(0);
        motor_stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_stanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_glisiere.setTargetPosition(0);
        motor_glisiere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_glisiere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        servoRotire = hardwareMap.get(CRServo.class, "servoRotire");
        cleste  = hardwareMap.get(Servo.class, "cleste");

        /* Make sure that the servoRotire is off, and the cleste is folded in. */
        servoRotire.setPower(rotirePosition);


        cleste.setPosition(cleste_inchis);


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive())

        {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.15;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 2.2);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            fata_stanga.setPower(frontLeftPower);
            spate_stanga.setPower(backLeftPower);
            fata_dreapta.setPower(frontRightPower);
            spate_dreapta.setPower(backRightPower);

            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            servoRotire and cleste) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the cleste to make sure it is in the correct orientation to servoRotire, and it
            turns the servoRotire on to the COLLECT mode.*/

            if(gamepad2.a) {
                cleste.setPosition(cleste_deschis);
            }
            else if (gamepad2.x) {
                cleste.setPosition(cleste_inchis);
            }

            if(gamepad2.y)
                servoRotire.setPower(servoTras);
            else if (gamepad2.b)
                servoRotire.setPower(servoRetras);

            if(gamepad2.dpad_left)
                motor_stanga.setPower(test);



            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));




            if(gamepad2.dpad_up) {
                motor_stanga.setPower(cosSusBrat);
                motor_glisiere.setPower(LIFT_SCORING_IN_HIGH_BASKET);
            }
            if(gamepad2.dpad_down) {
                motor_glisiere.setPower(LIFT_SCORING_IN_LOW_BASKET);
                motor_stanga.setPower(cosJosBrat);
            }
            if(gamepad2.dpad_right) {
                motor_glisiere.setPower(LIFT_SCORING_IN_LOW_BASKET);
                motor_stanga.setPower(SpecimenBrat);
            }




            if (armPosition < 45 * ARM_TICKS_PER_DEGREE){
                armLiftComp = (0.25568 * liftPosition);
            }
            else{
                armLiftComp = 0;
            }



            motor_stanga.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));


            ((DcMotorEx) motor_stanga).setVelocity(3500);// Velocity inseamna viteaza maxima
            motor_stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) motor_stanga).isOverCurrent()){
                telemetry.addLine("BRAT EXCEEDED CURRENT LIMIT!");
            }



            if (gamepad2.right_bumper) {
                liftPosition += 50 * cycletime;
            }
            else if (gamepad2.left_bumper) {
                liftPosition -= 50 * cycletime;
            }


            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET) {
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
            else if (liftPosition < 0) {
                liftPosition = 0;
            }


            motor_glisiere.setTargetPosition((int) (liftPosition));

            ((DcMotorEx) motor_glisiere).setVelocity(2100);
            motor_glisiere.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            servoRotire.setPower(-gamepad2.left_stick_y);


            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("arm Target Position: ", motor_stanga.getTargetPosition());
            telemetry.addData("arm Encoder: ", motor_stanga.getCurrentPosition());
            telemetry.addData("lift variable", liftPosition);
            telemetry.addData("Lift Target Position",motor_glisiere.getTargetPosition());
            telemetry.addData("lift current position", motor_glisiere.getCurrentPosition());
            telemetry.addData("motor_glisiere Current:",((DcMotorEx) motor_glisiere).getCurrent(CurrentUnit.AMPS));
            telemetry.update();



        }
    }
}
