package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// Utils
import java.util.HashMap;

@Autonomous
public class W_AUTO extends LinearOpMode {

  // Motor and Servo Initialization
  private HashMap<String, DcMotor> motors;
  private DcMotor[] motors_arm = new DcMotor[2];
  private DcMotor[] motors_movement = new DcMotor[4];
  private DcMotor motors_aRotate;

  private HashMap<String, Servo> servos;
  private Servo servo_lift_1;
  private Servo servo_lift_2;

  // Gamepad Initialization
  @Override
  public void runOpMode() {
    // Initialize motor and servo arrays

    // Initialize Servo HashMap
    servos = new HashMap<>();
    
    // Initialize Motor HashMap
    motors = new HashMap<>();
    
    // Add Intake Servos to map
    servos.put("iRotate1", hardwareMap.get(Servo.class, "s1"));
    servos.put("iRotate2", hardwareMap.get(Servo.class, "s2"));
    
    // Add Movement Motors to map
    motors.put("lBack", hardwareMap.get(DcMotor.class, "0"));
    motors.put("rBack", hardwareMap.get(DcMotor.class, "1"));

    // Arm Control Motors
    motors.put("aRotate", hardwareMap.get(DcMotor.class, "2"));

    // motors.get("lBack").setDirection(DcMotorSimple.Direction.REVERSE); // Modify or add lines if motor setup changes

    motors_movement[0] = motors.get("lBack");
    motors_movement[1] = motors.get("rBack");
    
    motors_movement[0].setDirection(DcMotorSimple.Direction.REVERSE); // Reverse Rear Left motor (untest - change if incorrect)

    motors_aRotate = motors.get("aRotate");
  
    servo_lift_1 = servos.get("iRotate1");
    servo_lift_2 = servos.get("iRotate2");
    
    waitForStart();

    while (opModeIsActive()) {
      run_auto();
      break;
    }
  }

  private void mode_linear(double power, double seconds) {
    for (DcMotor m : motors_movement) {
      m.setPower(power);
    }
    
    sleep(time * 1000);

    for (DcMotor m : motors_movement) {
      m.setPower(0.0);
    }
  }

  private void move_turn(double time, char dir) {
    switch (dir) {
      case 'l':
      case 'L':
        {
          motors_movement[0].setPower(1); // L back
          motors_movement[1].setPower(-1); // R back
          break;
        }
      
      case 'r':
      case 'R':
        {
          motors_movement[0].setPower(-1);
          motors_movement[1].setPower(0.0);
          break;
        }
      default:
        break;
    }

    sleep(time);

    for (DcMotor m : motors_movement) {
      m.setPower(0.0);
    }
  }


  private void arm_rotate(double power, double time) {
     motors_aRotate.setPower(power);
     sleep(time);
     motors_aRotate.setPower(0.0);
  }

  private void intake_move() {
    servo_lift_1.setPosition(1);
    servo_lift_2.setPosition(1);

    sleep(0.2);

    servo_lift_1.setPosition(0);
    servo_lift_2.setPosition(0);
    
  }

    
  public void run_auto() {
    move_linear(1, 0.5);
    move_turn(0.25, 'L');

    move_linear(1, 4);
    move_linear(-1, 0.25);

    move_turn(0.15, 'L');
    move_linear(1, 0.3);

    arm_rotate(1, 0.25);
    intake_move();
    arm_rotate(-1, 0.25);
    
    move_linear(-1, 0.25);
    move_turn(0.25, 'R');
    move_linear(-1, 4);
    move_linear(1, 0.2);
    move_turn(0.5, 'R');
    move_linear(-1, 1);
  }

}