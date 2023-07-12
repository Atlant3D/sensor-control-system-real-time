#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include <mutex>
#include <iostream>
#include <condition_variable>

class Sensor
{
private:
    double data;
    std::mutex mtx;

public:
    Sensor() : data(0) {}

    void update(double value)
    {
        std::lock_guard<std::mutex> lock(mtx);
        data = value;
    }

    double get_data()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return data;
    }
};

class TemperatureSensor : public Sensor
{
public:
    TemperatureSensor() : Sensor() {}
};

class PressureSensor : public Sensor
{
public:
    PressureSensor() : Sensor() {}
};

class PIDController
{
private:
    double kp, ki, kd; // PID gains
    double prev_error; // Error from previous iteration
    double integral;   // Integral of error over time

public:
    // Constructor that initializes the PID gains
    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), prev_error(0), integral(0) {}

    // Calculates the output of the PID controller for a given setpoint and process variable
    double control(double setpoint, double pv)
    {
        double error = setpoint - pv;                                 // Calculate the error
        double derivative = error - prev_error;                       // Calculate the derivative of the error
        integral += error;                                            // Calculate the integral of the error
        double output = kp * error + ki * integral + kd * derivative; // Calculate the output
        prev_error = error;                                           // Store the error for the next iteration
        return output;                                                // Return the output
    }
};

class ControlSystem
{
private:
    TemperatureSensor temperatureSensor;
    PressureSensor pressureSensor;
    PIDController temperatureController;
    PIDController pressureController;

    std::mutex mtx;

    double temperature_fluctuation = 0.0;
    double pressure_fluctuation = 0.0;
    double control_input_temperature = 0.0;
    double control_input_pressure = 0.0;
    double temperature = 20.0; // Initial temperature in degrees Celsius
    double pressure = 1.0;     // Initial pressure in atmospheres

public:
    ControlSystem() : temperatureSensor(), pressureSensor(), temperatureController(1.0, 0.1, 0.05), pressureController(1.0, 0.1, 0.05) {}

    void updateSensors(unsigned int numIterations, std::default_random_engine &generator,
                       std::normal_distribution<double> &temperature_distribution,
                       std::normal_distribution<double> &pressure_distribution)
    {

        for (unsigned int i = 0; i < numIterations; ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::lock_guard<std::mutex> lock(mtx);
            temperature_fluctuation = temperature_distribution(generator);
            pressure_fluctuation = pressure_distribution(generator);
        }
    }

    void updateController(unsigned int numIterations, double setpoint_temperature, double setpoint_pressure)
    {

        for (unsigned int i = 0; i < numIterations; ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::lock_guard<std::mutex> lock(mtx);
            control_input_temperature = temperatureController.control(setpoint_temperature, temperatureSensor.get_data());
            control_input_pressure = pressureController.control(setpoint_pressure, pressureSensor.get_data());
        }
    }

    void adjustParameters(unsigned int numIterations)
    {
        for (unsigned int i = 0; i < numIterations; ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            {
                std::lock_guard<std::mutex> lock(mtx);
                temperature += temperature_fluctuation + control_input_temperature;
                pressure += pressure_fluctuation + control_input_pressure;
            }
            temperatureSensor.update(temperature);
            pressureSensor.update(pressure);
        }
    }

    void run(unsigned int numIterations)
    {
        std::default_random_engine generator;
        std::normal_distribution<double> temperature_distribution(2.0, 2.0);
        std::normal_distribution<double> pressure_distribution(0.5, 0.5);

        double setpoint_temperature = 50.0; // Setpoint temperature in degrees Celsius
        double setpoint_pressure = 2.5;     // Setpoint pressure in atmospheres

        std::thread t1(&ControlSystem::updateSensors, this, numIterations, std::ref(generator), std::ref(temperature_distribution), std::ref(pressure_distribution));
        std::thread t2(&ControlSystem::updateController, this, numIterations, setpoint_temperature, setpoint_pressure);
        std::thread t3(&ControlSystem::adjustParameters, this, numIterations);

        t1.join();
        t2.join();
        t3.join();
    }

    int get_temperature()
    {
        return temperatureSensor.get_data();
    }

    int get_pressure()
    {
        return pressureSensor.get_data();
    }
};

extern "C"
{
    ControlSystem *ControlSystem_new() { return new ControlSystem(); }
    void ControlSystem_run(ControlSystem *cs, int numIterations) { cs->run(numIterations); }
    int ControlSystem_get_temperature(ControlSystem *cs) { return cs->get_temperature(); }
    int ControlSystem_get_pressure(ControlSystem *cs) { return cs->get_pressure(); }
}
