package com.z3db0y.davidlib;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.ArrayList;
import java.lang.Thread;

public class Logger {

    static private Telemetry telemetry;
    static private ArrayList<String> lines = new ArrayList<>();

    static public void setTelemetry(Telemetry tele) {
        telemetry = tele;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    static private String stripHTML(Object str) {
        return str.toString()
                .replaceAll("<", "&lt;")
                .replaceAll(">", "&gt;")
                .replaceAll(" ", "&nbsp;")
                .replaceAll("\"", "&quot;")
                .replaceAll("'", "&apos;");
    }

    static private void addDataInternal(Object message, String color, String caller) {
        if(color == null) color = "#ffffff";
        if(telemetry != null) {
            lines.add(caller + "() " + message);
            telemetry.addData("<span style='color: #ffaa00'>" + caller + "()</span>", "<span style='color: " + color + "'>" + stripHTML(message) + "</span>");
        }
    }

    static public void addDataDashboard(String key, Object message) {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        dashboardTelemetry.addData(key, message);
        Object combined_message = key + ": " + message;
        StackTraceElement stackTraceElement = Thread.currentThread().getStackTrace()[3];
        addDataInternal(combined_message, null, stackTraceElement.getClassName().split("\\.")[stackTraceElement.getClassName().split("\\.").length-1] + "." + stackTraceElement.getMethodName());
    }

    static public void addData(Object message, String color) {
        StackTraceElement stackTraceElement = Thread.currentThread().getStackTrace()[3];
        addDataInternal(message, color, stackTraceElement.getClassName().split("\\.")[stackTraceElement.getClassName().split("\\.").length-1] + "." + stackTraceElement.getMethodName());
    }

    static public void addData(Object message) {
        StackTraceElement stackTraceElement = Thread.currentThread().getStackTrace()[3];
        addDataInternal(message, null, stackTraceElement.getClassName().split("\\.")[stackTraceElement.getClassName().split("\\.").length-1] + "." + stackTraceElement.getMethodName());
    }

    static public void speak(String message){
        StackTraceElement stackTraceElement = Thread.currentThread().getStackTrace()[3];
        telemetry.speak(message);
        addDataInternal(message, null, stackTraceElement.getClassName().split("\\.")[stackTraceElement.getClassName().split("\\.").length-1] + "." + stackTraceElement.getMethodName());
    }

    static public void update() {
        if(telemetry != null) {
            for(String line : lines) {
                Log.i("davidlib", line);
            }
            lines = new ArrayList<>();
            telemetry.update();
        }
        if(FtcDashboard.getInstance() != null) {
            Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
            dashboardTelemetry.update();
        }
    }

}
