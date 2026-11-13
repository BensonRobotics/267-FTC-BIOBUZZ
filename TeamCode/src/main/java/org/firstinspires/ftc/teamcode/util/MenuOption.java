package org.firstinspires.ftc.teamcode.util;

public class MenuOption {
    public final String name;
    public final String[] options;
    public int optionIdx = 0;

    MenuOption(String iname, String[] ioptions){
        name = iname;
        options = ioptions;
    }
}
