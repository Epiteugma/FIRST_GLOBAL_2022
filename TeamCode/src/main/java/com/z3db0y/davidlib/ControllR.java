//package com.z3db0y.davidlib;
//
//import android.content.Context;
//import android.util.Log;
//
//import com.qualcomm.ftccommon.FtcEventLoop;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.ftccommon.external.OnCreate;
//import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
//import org.firstinspires.ftc.ftccommon.external.OnDestroy;
//import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
//
//import java.io.BufferedReader;
//import java.io.IOException;
//import java.io.InputStream;
//import java.io.InputStreamReader;
//import java.net.ServerSocket;
//import java.net.Socket;
//import java.nio.ByteBuffer;
//import java.nio.charset.StandardCharsets;
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.HashMap;
//import java.util.List;
//import java.util.Map;
//
//@Disabled
//public class ControllR {
//    static ControllR instance;
//    FtcEventLoop eventLoop;
//    OpModeManagerImpl manager;
//    ServerSocket server;
//    Thread serverThread;
//    Thread gamepadThread;
//    List<Gamepad> gamepads = Arrays.asList(new Gamepad(), new Gamepad(), new Gamepad(), new Gamepad());
//    String TAG = "ControllR";
//
//    @OnCreate
//    public static void onCreate(Context context) {
//        if(instance == null && !ControllR.class.isAnnotationPresent(Disabled.class)) instance = new ControllR();
//    }
//
//    @OnDestroy
//    public static void onDestroy(Context context) {
//        if(instance != null) instance.stop();
//    }
//
//    @OnCreateEventLoop
//    public static void onEventLoop(Context context, FtcEventLoop eventLoop) {
//        if(instance != null) instance.eventLoop = eventLoop;
//    }
//
//    void stop() {
//        if(this.serverThread != null && this.serverThread.isAlive()) {
//            this.serverThread.interrupt();
//            try {
//                if (this.server != null && !this.server.isClosed()) this.server.close();
//            } catch (IOException ignored) {}
//            if(this.gamepadThread != null && this.gamepadThread.isAlive()) this.gamepadThread.interrupt();
//        }
//    }
//
//    private ControllR() {
//        this.serverThread = new Thread(() -> {
//            while (this.eventLoop == null) {
//                try {
//                    Thread.sleep(250);
//                } catch(InterruptedException ignored) {}
//            }
//            Log.w(TAG, "Obtained event loop");
//            this.manager = eventLoop.getOpModeManager();
//
//            this.gamepadThread = new Thread(() -> {
//                while(true) {
//
//                    this.gamepads.forEach((gamepad) -> {
//                        try {
//                            if(this.manager.getActiveOpMode() != null) {
//                                if (gamepad.getGamepadId() == 1) {
//                                    this.manager.getActiveOpMode().gamepad1.copy(gamepad);
//                                } else if (gamepad.getGamepadId() == 2) {
//                                    this.manager.getActiveOpMode().gamepad2.copy(gamepad);
//                                }
//                            }
//                        } catch(Exception ignored) {}
//                    });
//                }
//            });
//            this.gamepadThread.start();
//
//            try {
//                this.server = new ServerSocket(5051);
//                while(true) {
//                    Socket client = this.server.accept();
//                    Log.i(TAG, "Client connected");
//                    try {
//                        int b;
//                        int i = 0;
//                        byte[] buffer = new byte[11];
//                        String msg;
//                        try {
//                            while ((b = client.getInputStream().read()) != -1) {
//                                buffer[i] = (byte) b;
//                                i++;
//                                if(i == buffer.length) {
//                                    i = 0;
//
//                                    ByteBuffer bb = ByteBuffer.wrap(buffer);
//                                    int cmd = bb.get(0);
//                                    int gamepadID = bb.get(1);
//                                    int button = bb.get(2);
//                                    float value = (float) bb.getDouble(3);
//
//                                    Log.v(TAG, "CMD: " + cmd + " | Gamepad: " + gamepadID + " | Button: " + button + " | Value: " + value);
//
//                                    if(cmd == 0xA) {
//                                        // Button event
//                                        switch (button) {
//                                            case 0x0:
//                                                this.gamepads.get(gamepadID).a = value == 1;
//                                                break;
//                                            case 0x1:
//                                                this.gamepads.get(gamepadID).b = value == 1;
//                                                break;
//                                            case 0x2:
//                                                this.gamepads.get(gamepadID).x = value == 1;
//                                                break;
//                                            case 0x3:
//                                                this.gamepads.get(gamepadID).y = value == 1;
//                                                break;
//                                            case 0x4:
//                                                this.gamepads.get(gamepadID).left_bumper = value == 1;
//                                                break;
//                                            case 0x5:
//                                                this.gamepads.get(gamepadID).right_bumper = value == 1;
//                                                break;
//                                            case 0x6:
//                                                this.gamepads.get(gamepadID).left_trigger = value;
//                                                break;
//                                            case 0x7:
//                                                this.gamepads.get(gamepadID).right_trigger = value;
//                                                break;
//                                            case 0x8:
//                                                this.gamepads.get(gamepadID).back = value == 1;
//                                                break;
//                                            case 0x9:
//                                                this.gamepads.get(gamepadID).start = value == 1;
//                                                break;
//                                            case 0xA:
//                                                this.gamepads.get(gamepadID).left_stick_button = value == 1;
//                                                break;
//                                            case 0xB:
//                                                this.gamepads.get(gamepadID).right_stick_button = value == 1;
//                                                break;
//                                            case 0xC:
//                                                this.gamepads.get(gamepadID).dpad_up = value == 1;
//                                                break;
//                                            case 0xD:
//                                                this.gamepads.get(gamepadID).dpad_down = value == 1;
//                                                break;
//                                            case 0xE:
//                                                this.gamepads.get(gamepadID).dpad_left = value == 1;
//                                                break;
//                                            case 0xF:
//                                                this.gamepads.get(gamepadID).dpad_right = value == 1;
//                                                break;
//                                        }
//                                    } else if(cmd == 0xB) {
//                                        // Axis event
//                                        switch (button) {
//                                            case 0x0:
//                                                this.gamepads.get(gamepadID).left_stick_x = value;
//                                                break;
//                                            case 0x1:
//                                                this.gamepads.get(gamepadID).left_stick_y = value;
//                                                break;
//                                            case 0x2:
//                                                this.gamepads.get(gamepadID).right_stick_x = value;
//                                                break;
//                                            case 0x3:
//                                                this.gamepads.get(gamepadID).right_stick_y = value;
//                                                break;
//                                        }
//                                    }
//
//                                    if (gamepads.get(gamepadID).start) {
//                                        if (gamepads.get(gamepadID).a) {
//                                            gamepads.get(gamepadID).setGamepadId(1);
//                                            Log.w(TAG, "Gamepad 1 assigned");
//                                        }
//                                        if (gamepads.get(gamepadID).b) {
//                                            gamepads.get(gamepadID).setGamepadId(2);
//                                            Log.w(TAG, "Gamepad 2 assigned");
//                                        }
//                                    }
//                                }
//                            }
//                            if (!client.isClosed()) client.close();
//                        } catch (IOException e) {
//                            Log.e(TAG, e.getMessage());
//                        }
//                    } catch (Exception ignored) {}
//                }
//
//            } catch (Exception ignored) {}
//        });
//        this.serverThread.start();
//    }
//}
