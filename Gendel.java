package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.reflect.Field;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;



/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="Mecanum Drive Example", group="Iterative Opmode")
public class Gendel extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor Arm = null;
    private Servo ground_claw = null;
    //private Servo Arm_rotation = null;
    private Servo Arm_claw = null;
    

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        
        front_left  = hardwareMap.dcMotor.get("left-front");
        front_right = hardwareMap.dcMotor.get("right-front");
        back_left   = hardwareMap.dcMotor.get("left-back");
        back_right  = hardwareMap.dcMotor.get("right-back");
        Arm = hardwareMap.dcMotor.get("elavate");
        ground_claw = hardwareMap.servo.get("ground-claw");
    //private Servo Arm_rotation = null;
        Arm_claw = hardwareMap.servo.get("arm-claw");
        
        //front_left   = hardwareMap.get(DcMotor.class, "front_left");
        //front_right  = hardwareMap.get(DcMotor.class, "front_right");
        //back_left    = hardwareMap.get(DcMotor.class, "back_left");
        //back_right   = hardwareMap.get(DcMotor.class, "back_right");
    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;

        double Elavate = gamepad1.right_stick_y;

        boolean ground_open = gamepad1.a;
        boolean Arm_up = gamepad1.x;
        boolean Arm_claw_open = gamepad1.y;
        
        

        //seting the power states of the servo
        //checking for arm control
        boolean Ground_press = false;
        boolean Arm_press = false;
        boolean Arm_claw_press = false;
        if (ground_open == true){
            Ground_press = !Ground_press;
        }
        if(Arm_up == true){
            Arm_press = !Arm_press;
        }
        if(Arm_claw_open == true){
            Arm_claw_press = !Arm_claw_press;
        }
        if (Ground_press == true){
            ground_claw.setPosition(1);
        }
        else {
            ground_claw.setPosition(0);
        }

        if (Arm_press == true){
           // Arm_rotation.setPosition(.30);
        }
        else {
            //Arm_rotation.setPosition(0);
        }

        if (Arm_claw_press == true){
            Arm_claw.setPosition(1);
        }
        else {
            Arm_claw.setPosition(0);
        }


        // lift controls
        Arm.setPower(Elavate);
        
        
        //   If we had a gyro and wanted to do field-oriented control, here
        //   is where we would implement it.
         
        //   The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
        //   coordinate (strafe, drive), and we just rotate it by the gyro
        //   reading minus the offset that we read in the init() method.
        //   Some rough pseudocode demonstrating:
         
        //   if Field Oriented Control;
        //       new strafe = strafe * cos(heading) - drive * sin(heading);
        //       new drive  = strafe * sin(heading) + drive * cos(heading);
         
        //  If you want more understanding on where these rotation formulas come
        //  from, refer to
        //  https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        
        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }
}
