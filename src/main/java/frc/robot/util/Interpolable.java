package frc.robot.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
