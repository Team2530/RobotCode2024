package frc.robot.util;

public class ConstantData {
    private final String fullName;
    private final String type;
    private final String value;

    public ConstantData(String category, String name, String type, String value) {
        this.fullName = String.format("%s.%s", category, name);
        this.type = type;
        this.value = value;
    }

    public String getFullName() {
        return fullName;
    }

    public String getType() {
        return type;
    }

    public String getValue() {
        return value;
    }
}