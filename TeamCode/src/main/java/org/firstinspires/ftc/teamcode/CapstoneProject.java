package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//-21000 = 90° rotate 90° clockwise

@TeleOp(name = "Capstone")

public class CapstoneProject extends LinearOpMode{
    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;

    private long Y1,Y2,X;
    private int y1,y2,x;
    private PID pid1,pid2,pid3,pid1_0,pid2_0,pid3_0;
    private int target_x,target_y,target_angle;


    public void runOpMode() {
        //initialize 4 motors
        lf = hardwareMap.get(DcMotor.class, "1");
        lb = hardwareMap.get(DcMotor.class, "0");
        rf = hardwareMap.get(DcMotor.class, "3");
        rb = hardwareMap.get(DcMotor.class, "2");

        rb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);

        //Get the values of additional encoder through motor
        Y1=rf.getCurrentPosition();
        Y2=lf.getCurrentPosition();
        X=rb.getCurrentPosition();

        //create pid. Parameters for pid-location are same as the pid-speed.
        pid1_0 = new PID(0.005,0,0.015,25); //y-pid-location
        pid1 = new PID(0.005,0,0.015,25); //y-pid-speed

        pid2_0 = new PID(0.005,0,0.002,50);  //angle-pid-location
        pid2 = new PID(0.005,0,0.002,50);  //angle-pid-speed

        pid3_0 = new PID(0.005,0,0.015,25);  //x-pid-location
        pid3 = new PID(0.005,0,0.015,25);  //x-pid-speed

        //set pid output limitation to 0.5
        pid1.limit(0.5);
        pid2.limit(0.5);
        pid3.limit(0.5);

        waitForStart();

        /// initialization ends here.
        if (opModeIsActive()) {
            if (opModeIsActive()) {  //calculate once, can be replaced by while for multiple times

                //calculation relative change
                y1=(int)(rf.getCurrentPosition()-Y1);
                y2=(int)(lf.getCurrentPosition()-Y2);
                x=-(int)(rb.getCurrentPosition()-X);

                //output tuning info
                telemetry.addData("y1",y1);
                telemetry.addData("y2",y2);
                telemetry.addData("x",x);

                //translate around the field for one revolution
                moveTo(0,160000,0,true);
                moveTo(0,0,0,true);
                moveTo(140000,0,0,true);
                moveTo(0,-160000,0,true);
                moveTo(-140000,0,0,true);

                telemetry.update();


            }
        }

    }

    /// wrapped control method, facilitating chassis control
    public void move(double x,double y,double angle,double power){
        angle = -angle;

        lf.setPower(power * (y + (x + angle)));
        lb.setPower(power * (y - (x - angle)));
        rf.setPower(power * (y - (x + angle)));
        rb.setPower(power * (y + (x - angle)));
    }

    /*move method: calling PID to keep calculating until the current location is satisfied
      while keep-moving being set on, the iteration continues and the robot would self-adjust.
      self-adjustment would avoid the location deviation caused by external force.
    */
    ///@param target_x the target value of the x encoder (int)
    ///@param target_y the target value of the y encoders (int)
    ///@param target_angle the target angle of the robot (int)
    ///@param Flag a boolean indicates status of self-adjustment function: true means activated.
    public void moveTo(int target_x,int target_y,int target_angle,boolean Flag){
        Y1=rf.getCurrentPosition();
        Y2=lf.getCurrentPosition();
        X=rb.getCurrentPosition();
        while(true){
            y1=(int)(rf.getCurrentPosition()-Y1);
            y2=(int)(lf.getCurrentPosition()-Y2);
            x=-(int)(rb.getCurrentPosition()-X);

            //all the upper here refers to the pid-location
            int upper1 = (int)(pid1_0.run(y2,target_y)*20);                 //pid-location
            int upper2 = (int)(pid2_0.run(y2-y1,target_angle)*20);  //pid-speed
            int upper3 = (int)(pid3_0.run(x,target_x)*20);

            //PID calculation
            double move_y = pid1.run(-upper1,0);
            double move_angle = pid2.run(-upper2,0);
            double move_x = pid3.run(-upper3,0);
            move(move_x,move_y,move_angle,1);
            telemetry.addData("t",System.currentTimeMillis());
            telemetry.addData("upper",upper1);

            double E = 0.00001; //output lower bound

            //decide whether the iteration breaks
            if(Flag && Math.abs(move_y)<E && Math.abs(move_x)<E && Math.abs(move_angle)<E){
                break;
            }

            telemetry.update(); //refresh tuning info

        }
    }



}
