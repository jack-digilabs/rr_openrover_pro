
// interface between a python ROS node and AVOS Robot SDK
// to activate, uncomment the avos code

import avos.programs.avcontrol;
import java.io.IOException;
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
    public static BufferedReader ROSInput;
    public static int LedState;
    
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
                String line;
                // for sending information back over socket
                //ROSOutput = new PrintStream(ROSSocket.getOutputStream());
    	        while (true) {
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
        int ledtoggle = Integer.parseInt(tokens[3]);
        if (ledtoggle==1) {
            LedState=1-LedState;
            System.out.println("Left="+left+", Right="+right+", LED="+LedState);
            SetLED(LedState);
        }
        System.out.println("Left="+left+", Right="+right+", LED="+LedState);
        SetMotorVelocities(left,right);
    }

    public static void CloseROSServer() {
        try {
            ROSInput.close();
            ROSSocket.close();
            ROSServer.close();
        } catch (IOException e) {
            System.out.println(e);
        }
    }

    public static void InitAvos() {
        System.out.println("Initializing AVOS SDK interface");
        avcontrol.startConnection();
    }

    public static void SetMotorVelocities(int left, int right) {
        avcontrol.setMotorVelocities(left, right, 0);
    }
    public static void SetLED(int on) {
	byte ledon=50;
	byte ledoff=0;
        if (on==1) {
            avcontrol.setLEDVals(ledon,ledon);
        } else {
            avcontrol.setLEDVals(ledoff,ledoff);
        }
    }

    public static void main(String[] args) {
        System.out.println("Initializing ROS interface, starting server for input from ROS");
    	InitAvos();
    	InitROSServer();
        PollROSServer();
    	CloseROSServer();
        SetLED(0);
        SetMotorVelocities(0,0);
        System.out.println("*************************************************************************");
        System.out.println("*************************  DONE    **************************************");
        System.exit(0);
    }
}

