package org.firstinspires.ftc.teamcode.util;

public class MenuOptionExport {
    public final String name;
    public String selection = null;
    public double exportedNumber;

    MenuOptionExport(String iname, String iselection){
        name = iname;
        selection = iselection;
    }

    MenuOptionExport(String iname, double inumber){
        name = iname;
        exportedNumber = inumber;
    }
}
