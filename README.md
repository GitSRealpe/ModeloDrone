# DroneSimulation

An OpenModelica model of a small electric quadrotor, built to study how **battery size, payload and wind affect flight time and flight behaviour**.

The default vehicle is a 5-inch, 2S drone weighing about 250 g. The model includes the rigid-body dynamics, a LiPo battery, four ESCs, four DC motors with propellers, aerodynamic drag, wind and a cascaded flight controller.

## Contents

1. [Getting started](#1-getting-started)
2. [Model structure](#2-model-structure)
3. [Experiment parameters](#3-experiment-parameters)
4. [Chassis](#4-chassis)
5. [Battery and payload](#5-battery-and-payload)
6. [Wind](#6-wind)
7. [Flight controller](#7-flight-controller)
8. [ESC](#8-esc)
9. [Propellers](#9-propellers)
10. [Feedback](#10-feedback)
11. [Time of flight: `endOfFlight`](#11-time-of-flight-endofflight)
12. [Running from a script](#12-running-from-a-script)
13. [Useful result variables](#13-useful-result-variables)
14. [Assumptions and limitations](#14-assumptions-and-limitations)

## 1. Getting started

### Requirements

- OpenModelica (tested with 1.27.2)
- Modelica Standard Library 4.1.0
- The local `DroneLibrary` folder, which provides the chassis and blades

### Run the model in OMEdit

1. Open `ModeladoTeam/DroneLibrary/package.mo`.
2. Open `ModeladoTeam/ModeloDrone/DroneSimulation/package.mo`.
3. Open `DroneSimulation.DroneBattery`.
4. Click **Simulate**.

Load `DroneLibrary` first, because `DroneSimulation/package.mo` doesn't declare it as a dependency. Don't open the parent `ModeloDrone/package.mo`: `omc` rejects it because `DroneSimulation/package.mo` has no `within ModeloDrone;` line.

A default run ends with the message `End of flight` after about 17 minutes of simulated time. In the plot window, start with `feedback.r[3]` (altitude), `feedback.v_bat` (battery voltage) and `plant.battery.SOC` (state of charge).

## 2. Model structure

`DroneBattery` is the top-level model. It contains these blocks:

| Block | Type | Purpose |
|---|---|---|
| `testx`, `testy`, `testz`, `multiplex3` | MSL sources and routing | Build the position setpoint `{x_ref, y_ref, z_ref}` |
| `controller` | `FlightController` | Computes a duty cycle for each motor |
| `plant` | `DronePlant` | Battery, ESCs, propellers, chassis and battery mass |
| `feedback` | `Sensors` | Measures position, velocity, angles and battery voltage |
| `disturbances` | `Disturbances` | Wind, aerodynamic drag and payload |
| `endOfFlight` | `TerminateSimulation` | Stops the simulation when the flight is over |
| `world` | MSL `World` | Gravity and the inertial frame (z points up) |

The signals form a closed loop:

```text
             r_ref [3]                 duty [4]
 reference ------------> controller ------------> plant <----- disturbances
                             ^                     |  |       (wind, drag, payload)
                             |                     |  |
                             |   r, v, angles      |  | airframe (mechanical frame)
                             |   v_bat             |  | bat_p, bat_n (electrical pins)
                             +----- feedback <-----+--+
```

## 3. Experiment parameters

These parameters are declared at the top of `DroneBattery.mo`:

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `x_ref`, `y_ref`, `z_ref` | 2, 3, 10 | m | Position setpoint |
| `Q_bat_Ah` | 1.0 | Ah | Battery capacity |
| `N_cells` | 2 | - | Battery cells in series |
| `m_payload` | 0 | kg | Payload mass |
| `V_wind` | 3 | m/s | Mean wind speed |
| `wind_dir_deg` | 0 | deg | Wind direction |
| `V_gust` | 3 | m/s | Gust speed added to the mean wind |
| `t_gust` | 60 | s | Gust start time |
| `sigma_turb` | 0.5 | m/s | Turbulence standard deviation |
| `seed` | 1 | - | Turbulence random seed |
| `CdA_h` | 0.015 | m² | Horizontal drag area |
| `maxTilt_deg` | 25 | deg | Maximum roll and pitch command |

### How to change them

**Option 1: Text View (recommended).** Edit the default value at the top of `DroneBattery.mo`, then save. Every link in the model stays intact.

```modelica
parameter Real Q_bat_Ah = 2.0 annotation(Dialog(group = "Experiment"));
```

**Option 2: Diagram View.** Double-click the block that uses the value and type a number:

| Value | Block | Field |
|---|---|---|
| `Q_bat_Ah`, `N_cells` | `plant` | `Q_bat_Ah`, `N_cells` |
| `m_payload`, wind values, `CdA_h` | `disturbances` | Same name |
| `x_ref`, `y_ref`, `z_ref` | `testx`, `testy`, `testz` | `k` |
| `maxTilt_deg` | `controller` | `maxTilt_deg` |

> **Note:** these fields contain a parameter name, such as `Q_bat_Ah`. Typing a number replaces that link, so the top-level value and any script override of it are then ignored for that field. Type the name back to restore the link.

**Option 3: Scripts.** Pass values with `-override`, as shown in [section 12](#12-running-from-a-script).

## 4. Chassis

The airframe is `DroneLibrary.Mechanical.Chassis.Variants.droneChassis`, configured inside `DronePlant` as a **`+` layout**. Four rigid arms join at a central hub, and each arm tip carries one propeller.

### Geometry parameters

Change these by double-clicking `plant`. They have no links to break.

| Parameter | Default | Meaning |
|---|---|---|
| `l` | 0.2 m | Arm length, from the hub to the motor |
| `m_frame` | 0.2 kg | Airframe mass, including motors and blades |
| `cog_offset` | 0.02 m | Distance of the central mass below the arm plane |

The controller reads `l` and the total mass from the plant, so it updates automatically.

### Mass distribution

`m_frame` is divided so the total always stays equal to `m_frame`. The eight blades take 2 g each, and the remainder is split equally between the four arms and the central mass:

```text
chassis.m = (m_frame - 4*m_blades) / 5 = (200 g - 16 g) / 5 = 36.8 g per piece
```

| Part | Mass | Height relative to the arm plane |
|---|---|---|
| Four arms | 147.2 g | 0 |
| Eight blades | 16 g | 0 |
| Central point mass | 36.8 g | -20 mm (`cog_offset`) |
| Battery | 49.3 g (default) | -10 mm |
| Payload | `m_payload` | -50 mm (`hook_length`) |

### Center of mass

The overall center of mass sits on the drone's vertical axis, slightly **below the rotor plane**:

| Configuration | Center of mass height |
|---|---|
| Default, no payload | about -4.9 mm |
| Default, 100 g payload | about -18 mm |

These values are hand calculations from the parameters above. A low center of mass is typical of a drone with its battery underneath, but it **doesn't make the drone self-levelling**. Thrust acts along the body z axis through the center of mass, so neither thrust nor gravity produces a restoring torque. The controller keeps the drone level.

The offset does matter for sideways forces. Drag acts at the hub, above the center of mass, which creates a small torque that tends to tilt the drone against its motion.

### Motor layout

| Propeller | Chassis port | Position | Spin parameter `dir` |
|---|---|---|---|
| `prop1` | `frame_a3` | +x | +1 |
| `prop2` | `frame_a2` | +y | -1 |
| `prop3` | `frame_a` | -x | +1 |
| `prop4` | `frame_a1` | -y | -1 |

Opposite propellers spin the same way, so their reaction torques cancel at hover.

## 5. Battery and payload

### Battery weight changes automatically

**Change only `Q_bat_Ah` and `N_cells`.** The battery mass, and therefore the total mass and hover power, follow automatically:

```text
E_bat_Wh  = N_cells * Q_bat_Ah * V_cell_nominal
m_battery = E_bat_Wh / specificEnergy
```

With `V_cell_nominal = 3.7 V` and `specificEnergy = 150 Wh/kg`, a 1 Ah, 2S battery stores 7.4 Wh and weighs 49.3 g.

The same two parameters also set the electrical behaviour:

| Parameter | Mass | Electrical effect |
|---|---|---|
| `Q_bat_Ah` | Proportional | Capacity rises and resistance falls (`R_bat = R_times_Ah / Q_bat_Ah`) |
| `N_cells` | Proportional | Pack voltage rises, at 3.7 V nominal per cell |

`cellData` inside `plant` holds the single-cell data: capacity, internal resistance, RC pair and LiPo voltage curve. It doesn't set the mass, and you only need to edit it to model a different cell.

### Payload

Set `m_payload` in kilograms. The payload is a point mass rigidly attached 5 cm below the hub, and it's defined in `Disturbances`, outside the plant. It can't swing. `hook_length` in `Disturbances` changes the distance.

The controller's mass value, `plant.mass`, deliberately **excludes the payload**. The altitude integrator supplies the extra thrust.

### Reference results

Calm air, hovering at 2 m:

| Battery | Battery mass | Payload | Flight time |
|---|---|---|---|
| 2S 500 mAh | 24.7 g | 0 g | 9.8 min |
| 2S 1000 mAh | 49.3 g | 0 g | 17.1 min |
| 2S 2000 mAh | 98.7 g | 0 g | 26.3 min |
| 3S 1000 mAh | 74.0 g | 0 g | 22.4 min |
| 3S 2000 mAh | 148.0 g | 0 g | 31.5 min |
| 2S 1000 mAh | 49.3 g | 100 g | 10.2 min |
| 2S 1000 mAh | 49.3 g | 300 g | 5.0 min |

Doubling the capacity doesn't double the flight time, because the extra battery mass must also be lifted.

## 6. Wind

Wind comes from `WindProfile` and acts through `AeroDrag`, both inside `disturbances`. The wind velocity is the sum of three parts:

```text
wind speed = mean (V_wind) + gust (V_gust, trapezoid starting at t_gust)
v_wind[x]  = speed * cos(wind_dir_deg) + turbulence x
v_wind[y]  = speed * sin(wind_dir_deg) + turbulence y
v_wind[z]  = turbulence z
```

`wind_dir_deg` is the direction the wind blows **towards**: 0° pushes the drone towards +x, and 90° pushes it towards +y.

### Top-level parameters

Change these as described in [section 3](#how-to-change-them).

| To change | Set |
|---|---|
| Mean wind | `V_wind`, `wind_dir_deg` |
| Gust | `V_gust`, `t_gust` |
| Turbulence | `sigma_turb`, `seed` |
| Horizontal drag | `CdA_h` |
| **All wind off** | `V_wind = 0`, `V_gust = 0`, `sigma_turb = 0` |

### Other wind parameters

These have fixed defaults in the subsystem files and aren't exposed at the top level:

| Parameter | Default | Meaning |
|---|---|---|
| `disturbances.wind.rise_gust` | 2 s | Gust rise time |
| `disturbances.wind.hold_gust` | 5 s | Time at full gust |
| `disturbances.wind.fall_gust` | 3 s | Gust fall time |
| `disturbances.wind.T_turb` | 5 s | Turbulence filter time constant |
| `disturbances.wind.Ts_noise` | 0.5 s | Noise sample period |
| `disturbances.CdA_v` | 0.04 m² | Vertical drag area |

Set them in `Disturbances.mo` by adding a value to the `wind(...)` line, or from a script:

```text
-override disturbances.wind.hold_gust=20
```

### Reference results

Steady wind with no gust or turbulence, 2S 1000 mAh, hovering at 2 m:

| Wind | Steady tilt | Holds position? | Flight time |
|---|---|---|---|
| 3 m/s | 1.9° | Yes | 17.0 min |
| 8 m/s | 13.5° | Yes | 16.3 min |
| 10 m/s | 20.6° | Yes | 15.4 min |
| 12 m/s | 23.6° (at t = 200 s, still drifting) | **No**, blown over 1 km away | 14.9 min |

The limit comes from `maxTilt_deg` and the drag area. At 12 m/s, the 25° tilt limit doesn't provide enough horizontal force to balance the drag. Turbulence makes runs slower, because every noise sample is a simulation event.

## 7. Flight controller

`FlightController` compares the setpoint with the measurements and produces one duty cycle per motor.

### Inputs and outputs

| Port | Direction | Size | Unit | Connected to |
|---|---|---|---|---|
| `r_ref` | Input | 3 | m | Position setpoint |
| `r` | Input | 3 | m | `feedback.r` |
| `v` | Input | 3 | m/s | `feedback.v` |
| `angles` | Input | 3 | rad | `feedback.angles` |
| `v_bat` | Input | 1 | V | `feedback.v_bat` |
| `duty` | Output | 4 | 0 to 1 | `plant.duty` |

### Structure

```text
              +--> z --> ForceCompute: F = mass*(a + g) --------------> F_z -----+
              |                                                                  |
r_ref -> position P -> velocity PI                                               v
              |                                                                mixer -> duty[4]
              +--> x, y -> /g -> tilt limit -> attitude PD -> tau_x, y, z ------> ^
                                      yaw ref = 0 --^                             |
                                                                  v_bat ----------+
```

Each `Controller3D` block holds three `LimPID` controllers, one per axis.

| Stage | Block | Input | Output | Type |
|---|---|---|---|---|
| Position | `pos_controller3D` | Position error | Velocity setpoint | P |
| Velocity | `vel_controller3D` | Velocity error | Acceleration command | PI |
| Attitude | `att_controller3D` | Angle error | Torque command | PD |
| Mixer | `mixer` | Force, three torques, `v_bat` | Four duty cycles | Motor equations |

### Altitude control vs attitude control

| | Altitude control | Attitude control |
|---|---|---|
| **Controls** | Height, `z` | Orientation: roll, pitch and yaw |
| **Measured signals** | `r[3]`, `v[3]` | `angles` |
| **Controller** | Position P, then velocity PI | PD |
| **Command produced** | Total thrust, `F_z` | Torques, `tau_x`, `tau_y`, `tau_z` |
| **Effect on the motors** | All four change together | Motors change in opposite pairs |
| **Physical result** | The drone climbs or descends | The drone rotates |
| **Setpoint comes from** | `z_ref` | The x and y position loops (yaw is fixed at 0) |

**Horizontal position has no direct actuator.** A quadrotor moves sideways by tilting, so the x and y loops request an acceleration and convert it into a small-angle tilt setpoint:

```text
pitch_ref =  a_x / g
roll_ref  = -a_y / g     (both limited to +/- maxTilt_deg)
```

The attitude controller is the inner loop that follows these setpoints. Altitude runs separately, and the integrator in its velocity loop makes up for extra thrust needs, such as the payload or the thrust lost to tilting.

### Default gains

Change these by double-clicking `controller`.

| Parameter | x / roll | y / pitch | z / yaw | Meaning |
|---|---|---|---|---|
| `k_pos` | 1 | 1 | 1 | Position gain |
| `k_vel` | 1 | 1 | 3 | Velocity gain |
| `Ti_vel` | 1 s | 1 s | 2 s | Velocity integral time |
| `k_att` | 0.2 | 0.2 | 0.05 | Attitude gain |
| `Td_att` | 0.15 s | 0.15 s | 0.3 s | Attitude derivative time |

The attitude torque commands are limited to 0.05, 0.05 and 0.01 N·m for roll, pitch and yaw. The velocity setpoint is limited to 10 m/s per axis.

### Mixer

`MotorMixerVoltage` converts the commands into duty cycles in three steps. With `d = l` and `c = km/kf`:

```text
1. Thrust per motor:
   T1 = F_z/4 - tau_y/(2d) - tau_z/(4c)
   T2 = F_z/4 + tau_x/(2d) + tau_z/(4c)
   T3 = F_z/4 + tau_y/(2d) - tau_z/(4c)
   T4 = F_z/4 - tau_x/(2d) + tau_z/(4c)

2. Motor speed and voltage:
   w = sqrt(T / kf)
   V = psi*w + Ra*km*w^2/psi

3. Duty cycle:
   duty = V / v_bat
```

Dividing by the measured battery voltage keeps thrust steady as the battery drains. The mixer ignores motor friction, and the velocity integrator supplies the difference.

## 8. ESC

Each motor has one `ESC`. It's an **averaged, lossless DC-DC converter**: it represents the mean effect of PWM switching, with no individual switching events.

### Inputs and outputs

| Port | Direction | Type | Connected to |
|---|---|---|---|
| `duty` | Input | Signal, clamped to 0 to 1 | `controller.duty[i]` |
| `bat_p`, `bat_n` | Electrical input side | Pins | Battery |
| `mot_p`, `mot_n` | Electrical output side | Pins | `prop[i].p`, `prop[i].n` |

### Equations

```text
v_motor   = duty * v_battery
i_battery = duty * i_motor
```

Input power equals output power, so all ESC efficiency losses are ignored. At hover with the default battery in calm air, the duty is about 0.51 and each motor draws about 1.65 A.

## 9. Propellers

Each `ElectricPropeller` combines an MSL permanent-magnet DC motor, a revolute joint and two blades.

### Inputs and outputs

| Port | Direction | Type | Connected to |
|---|---|---|---|
| `p`, `n` | Electrical input | Pins | `esc[i].mot_p`, `esc[i].mot_n` |
| `Airframe` | Mechanical | Frame carrying force and torque to the chassis | Chassis arm tip |

The block has no signal outputs. To see its internal values, plot `plant.prop1.relSpeed.w_rel` for speed, for example.

### Equations

With `w` the rotor speed relative to the arm:

```text
Thrust = kf * w^2                  along the rotor z axis (body up)
Drag   = -km * dir * w * |w|       opposing the rotor's spin
```

The motor pushes against the airframe through the joint's support, so spinning the rotors produces a real reaction torque on the chassis. At hover in calm air, the default drone's rotors spin at about 870 rad/s.

### Parameters

Set these in `DronePlant`:

| Parameter | Default | Meaning |
|---|---|---|
| `D_prop` | 0.127 m | Propeller diameter (5 inch) |
| `C_T`, `C_P` | 0.10, 0.05 | Thrust and power coefficients |
| `rho_air` | 1.225 kg/m³ | Air density |
| `m_blade` | 2 g | Mass per blade |
| `KV` | 2300 rpm/V | Motor speed constant |
| `Ra` | 0.15 Ω | Motor armature resistance |
| `P_fric`, `w_fric` | 0.5 W, 800 rad/s | Motor friction reference point |

These give `kf = 8.07e-7 N·s²/rad²` and `km = 8.16e-9 N·m·s²/rad²`, calculated in `DronePlant`.

## 10. Feedback

`Sensors`, instantiated as `feedback`, closes the control loop. All of its measurements are **ideal**, with no noise, delay or bias.

### Inputs and outputs

| Port | Direction | Measured from |
|---|---|---|
| `frame_a` | Input (mechanical) | `plant.airframe`, the hub |
| `bat_p`, `bat_n` | Input (electrical) | Battery terminals |
| `r[3]` | Output | Hub position in the world frame, m |
| `v[3]` | Output | Hub velocity in the world frame, m/s |
| `angles[3]` | Output | Body angles from `AbsoluteAngles`, rad |
| `v_bat` | Output | Battery voltage, V |

### How feedback enters the controller

| Measurement | Goes to | Used for |
|---|---|---|
| `r` | `pos_controller3D.u_m` | Position error, including altitude |
| `v` | `vel_controller3D.u_m` | Velocity error |
| `angles` | `att_controller3D.u_m` | Attitude error |
| `v_bat` | `mixer.v_bat` | Converting motor voltage to duty |

`endOfFlight` also reads `feedback.r[3]` and `feedback.v_bat`.

## 11. Time of flight: `endOfFlight`

`endOfFlight` is an MSL `TerminateSimulation` block. It has no wires: it reads variables directly from its condition and **stops the simulation** when any of these becomes true:

| Condition | Meaning |
|---|---|
| `time > 20` and `feedback.r[3] < 0.75*z_ref` | The drone can no longer hold altitude |
| `plant.battery.SOC < 0.02` | The battery is essentially empty |
| `time > 5` and `feedback.v_bat < N_cells*V_cell_cutoff` | Voltage is below the cutoff (3.3 V per cell, so 6.6 V for 2S) |

The time delays allow for the climb and startup.

**The flight time is the final simulation time.** The block doesn't output a value, so read the last value of `time` or use a script ([section 12](#12-running-from-a-script)). In the runs in this document, the voltage cutoff ended the flight, at about 4% state of charge.

> **Check the message.** The log shows `End of flight` when the block fired. If that message is missing, the run hit `StopTime` (7200 s by default) instead. In that case, the final time isn't the flight time, so increase `StopTime`.

Keep this block for flight-time studies. Without it, the simulation continues past the point where a real flight would end.

## 12. Running from a script

You can also change values and read the flight time without opening OMEdit. Save this as a `.mos` file in the `ModeloDrone` folder and run it with `omc`, or run the lines in OMShell from that folder:

```modelica
loadModel(Modelica, {"4.1.0"});
loadFile("../DroneLibrary/package.mo");
loadFile("DroneSimulation/package.mo");
simulate(DroneSimulation.DroneBattery, stopTime = 7200, numberOfIntervals = 4000,
         fileNamePrefix = "test",
         simflags = "-override Q_bat_Ah=2,N_cells=3,m_payload=0.1,V_wind=0,V_gust=0,sigma_turb=0");
n := readSimulationResultSize("test_res.mat");
d := readSimulationResult("test_res.mat", {time}, n);
print("Flight time: " + String(d[1, n]/60) + " min\n");
```

Set the stop time with `stopTime` in `simulate`. `-override` doesn't change it. Result files are written to the folder you run from.

## 13. Useful result variables

| Variable | Meaning |
|---|---|
| `time` | Simulation time; its final value is the flight time |
| `feedback.r[1]`, `[2]`, `[3]` | Position x, y and z |
| `feedback.v[1]`, `[2]`, `[3]` | Velocity x, y and z |
| `feedback.angles[1]`, `[2]`, `[3]` | Body angles |
| `feedback.v_bat` | Battery voltage |
| `plant.battery.SOC` | Battery state of charge |
| `plant.m_battery` | Battery mass |
| `plant.mass` | Airframe plus battery, without payload |
| `controller.duty[1]` to `[4]` | Motor duty commands |
| `controller.forceCompute.F` | Total thrust command |
| `controller.mixer.tau_x`, `tau_y`, `tau_z` | Torque commands |
| `plant.prop1.relSpeed.w_rel` | Rotor 1 speed |
| `plant.esc1.iMot.i` | Motor 1 current |
| `disturbances.wind.v_wind[1]`, `[2]`, `[3]` | Wind velocity |

## 14. Assumptions and limitations

Keep these in mind when interpreting results:

- **Arm inertia:** the arms use the default `I_11 = I_22 = I_33 = 0.001 kg·m²`, because they aren't set in `DronePlant`. These defaults dominate the roll and pitch inertia (about 0.005 kg·m² in total) and weren't derived from a real airframe.
- **Center of mass position:** `cog_offset` is an assumed value, and the battery height is fixed at -10 mm in `DronePlant`.
- **Mass and energy:** battery mass assumes a constant 150 Wh/kg at every size.
- **Propellers:** `C_T` and `C_P` are constant. Blade flapping and rotor-induced drag aren't modelled, and neither are ground effect or changes in air density.
- **Drag:** drag acts at the hub, and the payload adds no drag of its own.
- **ESC:** the ESC is averaged and lossless, with no switching or thermal effects.
- **Sensors:** measurements are ideal.
- **Ground:** there is no ground contact. The drone starts at z = 0 and could pass below it if it descended.
- **Payload:** the payload is rigid, so a slung, swinging load would need a joint.
- **Altitude check:** the first `endOfFlight` condition assumes the drone reaches 75% of `z_ref` within 20 s, which matters only for very high setpoints.

The results in this document are simulations of the default parameters above. They haven't been validated against a physical drone.
