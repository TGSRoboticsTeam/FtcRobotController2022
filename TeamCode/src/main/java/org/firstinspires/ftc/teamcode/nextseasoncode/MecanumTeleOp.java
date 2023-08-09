package org.firstinspires.ftc.teamcode.nextseasoncode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2022 Steering Tourney Drive", group="Linear Opmode")

public class MecanumTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    double lAdjust  =   0;
    double lbAdjust =   0;
    double rAdjust  =   0;
    double rbAdjust =   0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted() && !isStopRequested()){
            telemetry.addData("FL Encoder:", leftFrontDrive.getCurrentPosition());
            telemetry.addData("BL Encoder:", leftBackDrive.getCurrentPosition());
            telemetry.addData("FR Encoder:", rightFrontDrive.getCurrentPosition());
            telemetry.addData("BR Encoder:", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            boolean adjustLeftStrafe = gamepad1.dpad_left;
            boolean adjustRightStrafe = gamepad1.dpad_right;
            boolean adjustBackward = gamepad1.dpad_down;
            boolean adjustForward = gamepad1.dpad_up;
            boolean adjustLeftTurn = gamepad1.left_bumper;
            boolean adjustRightTurn = gamepad1.right_bumper;

            if(adjustForward){
                lAdjust = .25;
                lbAdjust = .25;
                rAdjust = .25;
                rbAdjust = .25;
            }else if(adjustBackward){
                lAdjust = -.25;
                lbAdjust = -.25;
                rAdjust = -.25;
                rbAdjust = -.25;
            }else if(adjustRightStrafe){
                lAdjust = .25;
                lbAdjust = -.25;
                rAdjust = -.25;
                rbAdjust = .25;
            }else if(adjustLeftStrafe){
                lAdjust = -.25;
                lbAdjust = .25;
                rAdjust = .25;
                rbAdjust = -.25;
            }else if(adjustLeftTurn){
                lAdjust = -.25;
                lbAdjust = -.25;
                rAdjust = .25;
                rbAdjust = .25;
            }else if(adjustRightTurn){
                lAdjust = .25;
                lbAdjust = .25;
                rAdjust = -.25;
                rbAdjust = -.25;
            }else{
                lAdjust = 0;
                lbAdjust = 0;
                rAdjust = 0;
                rbAdjust = 0;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw + lAdjust;
            double rightFrontPower = axial - lateral - yaw + rAdjust;
            double leftBackPower   = axial - lateral + yaw + lbAdjust;
            double rightBackPower  = axial + lateral - yaw + rbAdjust;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}