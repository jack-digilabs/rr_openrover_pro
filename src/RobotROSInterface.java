
// interface between a python ROS node and AVOS Robot SDK
// to activate, uncomment the avos code

// Oracle compiled version has capital A in Avcontrol throughout
//import avos.programs.Avcontrol;
import avos.programs.Avcontrol;
import java.io.IOException;
//import java.util.Timer;
import javax.swing.Timer;
import javax.sound.sampled.*;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
/*
import java.io.ByteArrayInputStream;
import javax.imageio.ImageIO;
import javax.sound.sampled.*;
import javax.swing.*;
import java.awt.image.BufferedImage;
*/
import java.lang.*;
import java.io.*;
import java.net.*;

public class RobotROSInterface {
    public static ServerSocket ROSServer;
    public static Socket ROSSocket;
    public static int ChargeStatus;
    public static BufferedReader ROSInput;
    public static PrintWriter ROSOutput;
    public static byte LedState;
    // two types of feedback, slow and fast (slow for battery, temperature, fast for encoders, velocities)
    public static String ROSOutputDataSlow;
    public static String ROSOutputDataFast;
    
    public static void InitROSServer() 
    {
        LedState=0;
        // open server socket
    	try {
            ROSServer = new ServerSocket(8123);
    	} catch (IOException e) {
    	    System.out.println(e);
	    }
    }

    // BufferedReader
    public static void PollROSServer() {
        // open socket object on the server socket to listen/accept new clients
    	while (true) {
            System.out.println("Waiting for connection...");
            try {
                ROSSocket = ROSServer.accept();
            }
            catch (IOException e) {
                System.out.println(e);
            }
            System.out.println("Connected");
            // open input stream from the socket
            try {
                ROSInput = new BufferedReader(new InputStreamReader(ROSSocket.getInputStream()));
                // for sending information back over socket
                ROSOutput = new PrintWriter(ROSSocket.getOutputStream(), true);
                String line;
    	        while (true) {
                    if (ROSOutputDataSlow.length()>0) {
                        ROSOutput.println(ROSOutputDataSlow);
                        ROSOutputDataSlow="";
                    }
                    if (ROSOutputDataFast.length()>0) {
                        ROSOutput.println(ROSOutputDataFast);
                        ROSOutputDataFast="";
                    }
                    line = ROSInput.readLine();
                    if (line == null) {
                        break;
                    }
                    // parse it
                    ParseROSCommand(line);
                }
                System.out.println("connection broken...");
                ROSInput.close();
                ROSSocket.close();
            }
            catch (IOException e) {
                System.out.println(e);
            }
        }
    }

    public static void ParseROSCommand(String line) {
        // echo back to socket (or print to stdout)
        //ROSOutput.println(line);
        // switch on string after SET=<drive>_<turn>_<Toggle>
        // where drive=-1000 to 1000, turn=-1000 to 1000
        String[] tokens = line.split("[=_]");
        if (tokens.length != 4) {
            System.out.println(line);
            return;
        }
        int left = Integer.parseInt(tokens[1]);
        int right = Integer.parseInt(tokens[2]);
        byte ledtoggle = Byte.parseByte(tokens[3]);
        if (ledtoggle!=LedState) {
            LedState=ledtoggle;
            System.out.println("Left="+left+", Right="+right+", LED="+LedState);
            SetLED(LedState);
        }
        //System.out.println("Left="+left+", Right="+right+", LED="+LedState);
        // ramp on motor velocities here
/*
    # limit acceleration in any input commands, so the velocity is ramped upwards to the requested drive/turn cmds at this max accel
    # NOTE: if we spend too much time ramping up, we can cause problems - only one callback can be active at a time
    # drive_cmd==1000, what do we actually do? We send 100, then 200, 300, 400, etc, ramp by 10% first three 200 msecs, then by 20% next three 100 msecs then finally 100%
    # start at either 20% or 10% of drive_cmd, depending on if we're over 200
    if (abs(drive_cmd)>100):
        drive_cmd=req_drive_cmd/5.0
    if (abs(drive_cmd)>100):
        drive_cmd=req_drive_cmd/2.0
    if (abs(turn_cmd)>100):
        turn_cmd=req_turn_cmd/5.0
    if (abs(turn_cmd)>100):
        turn_cmd=req_turn_cmd/2.0
    while (abs(drive_cmd)<abs(req_drive_cmd) or abs(turn_cmd)<abs(req_turn_cmd)):
        drive_cmd *= 1.1
        turn_cmd *= 1.1
        if (abs(drive_cmd)>abs(req_drive_cmd)):
            drive_cmd=req_drive_cmd
        if (abs(turn_cmd)>abs(req_turn_cmd)):
            turn_cmd=req_turn_cmd
        # send move commands down sockets interface
        left=(drive_cmd-turn_cmd)
        right=(drive_cmd+turn_cmd)
*/
        SetMotorVelocities(left,right);
    }

    public static void CloseROSServer() {
        try {
            ROSInput.close();
            ROSOutput.close();
            ROSSocket.close();
            ROSServer.close();
        } catch (IOException e) {
            System.out.println(e);
        }
    }

    public static void InitAvos() {
        System.out.println("Initializing AVOS SDK interface");
        Avcontrol.startConnection();
    }

    public static void SetMotorVelocities(int left, int right) {
        Avcontrol.setMotorVelocities(left, right, 0);
    }

    public static void SetLED(byte level) {
        // IR LED is first, White LED is second
        Avcontrol.setLEDVals(level,level);
    }
        public static void StoreChargeStatus(int state) {
            ChargeStatus=state;
        };


    public static void main(String[] args) {
        System.out.println("Initializing ROS interface, starting server for input from ROS");
        ROSOutputDataSlow="";
        ROSOutputDataFast="";
        ChargeStatus=0;
    	InitAvos();
    	InitROSServer();
        byte ledoff=0;
        SetLED(ledoff);
        SetMotorVelocities(0,0);
    	// once per 30 seconds, get the motor temperature, battery charge and current
        ActionListener SlowTaskPerformer = new ActionListener() {
            public void actionPerformed(ActionEvent evt) {
                short[] MotorTemp = Avcontrol.getMotorTemps();
                short[] BatteryPCT = Avcontrol.getBatteryCharge();
	        int Curr = Avcontrol.getCurrent();
        	StoreChargeStatus(Curr);
                // send back via socket
                ROSOutputDataSlow="S:"+MotorTemp[0]+","+MotorTemp[1]+":"+BatteryPCT[0]+","+BatteryPCT[1]+":"+Curr+":E";
            }
        };
	Timer TimerSlow = new Timer(30000, SlowTaskPerformer);
        TimerSlow.start();
	// continuously during motion (10Hz), get the encoders and velocities
        ActionListener FastTaskPerformer = new ActionListener() {
            public void actionPerformed(ActionEvent evt) {
                int[] Motor_Enc = Avcontrol.getMotorEncCounts();
                float[] FB_RPM = Avcontrol.getFbRPMs();
                short[] Velocity = Avcontrol.getMotorVelocities();
	        int Curr = Avcontrol.getCurrent();
                // check the charging status more rapidly, if it changes, send a new status message
                /*
                if (ChargeStatus != Curr) {
                    short[] MotorTemp = Avcontrol.getMotorTemps();
                    short[] BatteryPCT = Avcontrol.getBatteryCharge();
	            Curr = Avcontrol.getCurrent();
        	    StoreChargeStatus(Curr);
                    ROSOutputDataSlow="S:"+MotorTemp[0]+","+MotorTemp[1]+":"+BatteryPCT[0]+","+BatteryPCT[1]+":"+Curr+":E";
                }
                */
                ROSOutputDataFast="M:"+Motor_Enc[0]+","+Motor_Enc[1]+":"+FB_RPM[0]+","+FB_RPM[1]+":"+Velocity[0]+","+Velocity[1]+":E";
            }
        };
        // send motor encoders at 5Hz (10Hz saw some dropped UDP packets when concurrently driving)
	Timer TimerFast = new Timer(200, FastTaskPerformer);
        TimerFast.start();

        PollROSServer();
        TimerSlow.stop();
        TimerFast.stop();
    	CloseROSServer();
        System.out.println("*************************************************************************");
        System.out.println("*************************  DONE    **************************************");
        System.exit(0);
    }
}

