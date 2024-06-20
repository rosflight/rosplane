# ROSplane Tuning

This ROS2 package contains tools useful for tuning the autopilot performance of ROSplane.

We recommend using [PlotJuggler](https://github.com/facontidavide/PlotJuggler) to visualize command input and response.

## Signal Generator

Signal generator is a ROS2 node that will generate step inputs, square waves, sine waves, sawtooth waves, and triangle waves to be used as command input for ROSplane. It has support for roll, pitch, altitude, course, and airspeed command input.

This is useful for tuning autopilots as we can give a clear and repeatable command to any portion of the autopilot and observe its response to that input. We can then tune gains, re-issue the same commands, and observe whether performance improved or worsened.

Signal generator works by publishing autopilot commands on the ROS network on the `controller_commands` topic. Each control output (roll, pitch, altitude, course, airspeed) has a default values that is set by a ROS parameter. The signal generator will then add the generated signal to one of the control outputs with a set magnitude and frequency.

### Signal Types
- Step: This is a non-continuous signal, where the generated signal will jump between the default value and the default+magnitude every time the `toggle_step_signal` service is called.
- Sine: This is a continuous signal that create a standard sine wave.
- Square: This is a continuous signal that creates a square type pattern.
- Triangle: This is a continuous signal that ramps up and down between the minimum and maximum value of the signal, creating a triangle like pattern.
- Sawtooth: This is a continuous signal (sometimes call a ramp signal) that creates a constantly increasing signal that resets to its minimum value once the maximum value is reached.

![Waveforms](Waveforms.svg)

*By Omegatron - Own work, CC BY-SA 3.0, https://commons.wikimedia.org/w/index.php?curid=343520*

### Parameters
- `controller_output`: Specifies what controller to apply the generated signal to. All other controllers will be constant at their default values. Valid values are `roll`, `pitch`, `altitude`, `course`, and `airspeed`.
- `signal_type`: Specified what kind of signal to generate. Valid values are `step`, `square`, `sawtooth`, `triangle`, and `sine`.
- `publish_rate_hz`: Specifies the rate to publish control commands. Must be greater than 0.
- `signal_magnitude`: Specifies the magnitude of the signal to generate. The signal will only be added to the default value, rather than subtracted. For example, if the signal has a magnitude of 2 and a default value of 5, the generated signal will range from 5 to 7.
- `frequency_hz`: Specifies the frequency of the generated signal. Must be greater than 0, and does not apply to step signals. For step signals, manually toggle the signal up and down with the `toggle_step_signal` service.
- `default_va_c`: The default value for the commanded airspeed, in meters per second.
- `default_h_c`: The default value for the commanded altitude, in meters above takeoff altitude.
- `default_chi_c`: The default value for the commanded course, in radians clockwise from north.
- `default_theta_c`: The default value for the commanded pitch, in radians pitched up from horizontal.
- `default_phi_c`: The default value for the commanded roll, in radians 'rolled right' from horizontal.

To get a parameter from the command line, use this command, replacing <parameter> with the desired parameter to get.
```
ros2 param get signal_generator <parameter>
```

To set a parameter from the command line, use this command, replacing <parameter> with the name of the parameter to set. Enter number parameters as float values, not integers (i.e. 1.0, not 1).
```
ros2 param set signal_generator <parameter>
```

### ROS Services
- `toggle_step_signal`: Toggles the step signal up and down. Does not apply to any other type of signal.
- `reset_signal`: Stops the generated signal and sets it to its default value.
- `pause_signal`: Pauses the generated signal at its current value. Does not apply to step signal.
- `start_continuous_signal`: Starts the signal generator at its current value, running continously until manually stopped. Does not apply to step signal.
- `start_single_period_signal`: Starts the signal generator at its current value, stopping after one full cycle. Does not apply to step signal.

To call a service from the command line use this command, replacing <service> with the name of the service you wish to call.
```
ros2 service call <service> std_srvs/srv/Trigger
```

