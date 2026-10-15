In your current navigation launch, the controller does this: turn each map waypoint into a desired velocity, use DLQR to produce the required translational force, add proportional yaw control, and distribute the resulting force and torque across eight thrusters. Near the waypoint, it switches to position control to brake and hold.

Nothing was changed during this explanation.

Your launch currently selects:

'waypoint_coordinates': 'world',
'control_mode': 'state',
'waypoint_cruise_speed': 1.0,

Despite the name state, the nonzero cruise speed activates velocity feedback during travel. State feedback takes over near arrival.

1. What information enters the controller

| Input | How the node uses it |
|---|---|
| /trajectory/waypoint | Absolute target position from the navigation planner |
| estimation/odom | Current position |
| holocean/dvl | Measured velocity in vehicle coordinates |
| holocean/mag | Heading used to rotate between vehicle and world coordinates |
| holocean/depth/scalar | Current vertical position, overriding odometry’s Z |
| /movement/finished_execution | Commands all thrusters to zero |

The navigation planner currently publishes targets with z = -28 m.

The controller constructs this state:

s=[vₓ,v_{y},v_{z},pₓ,p_{y},p_{z}]^{T}

All six values must refer to world coordinates. DVL measures velocity relative to the vehicle, so the node rotates it using heading:

v_{world}=R(ψ)v_{body}

For example, when the vehicle faces north, its forward velocity must become northward world velocity.

The node deliberately does not use the estimator’s velocity output. It uses DVL velocity directly and the estimator’s position.

This Mermaid diagram doesn't fit the current terminal width.
flowchart LR
    Planner["Navigation waypoint"] --> Feedback["DLQR feedback"]
    Estimator["Estimated position"] --> Feedback
    DVL["DVL velocity"] --> Rotation["Rotate using heading"]
    Mag["Magnetometer heading"] --> Rotation
    Rotation --> Feedback
    Depth["Depth"] --> Feedback
    Feedback --> Force["World force"]
    Force --> Body["Rotate force into vehicle coordinates"]
    Yaw["Proportional yaw torque"] --> Allocation["Thruster allocation"]
    Body --> Allocation
    Allocation --> Limit["Limit all eight commands"]
    Limit --> Sim["/holo/cmd"]

2. What physical model DLQR assumes

For each translational axis, the nominal model is:

ṗ=v,  mv̇=F-cv+b

Here:

- m=31.02 kg is vehicle mass.
- c=3.0 N per m/s is nominal linear drag.
- F is commanded force.
- b is net buoyancy, relevant on the vertical axis.

The controller assumes three independent translational axes. It does not model detailed nonlinear hydrodynamics.

The header converts that model into a discrete model for a nominal 30 Hz update rate:

sₖ₊₁=Asₖ+B(Fₖ+b)

This conversion is exact for the assumed linear model with force held constant between updates.

The node does not predict its actual position using this equation. It uses incoming position and velocity measurements. The model is used to calculate feedback gains.

3. What makes the force calculation “DLQR”

DLQR chooses feedback gains that balance state error against force effort:

   ∞
J= ∑  (eₖ^{T}Qeₖ+Fₖ^{T}RFₖ)
  k=0

The header solves the discrete Riccati equation at startup to obtain those gains. They stay fixed afterward.

The weights express tuning preferences. They are not physical limits:

- Larger position-error weights generally produce stronger position correction.
- Larger velocity-error weights penalize motion more strongly.
- Larger force weights discourage force effort.

The optimum applies to the nominal, unconstrained linear model. Adding yaw control and saturating thrusters means the complete controller is not globally “optimal.”

4. How cruising works with your settings

The node calculates the displacement to the waypoint:

Δ=p_{target}-p,  d=‖Δ‖

It then requests a velocity pointing toward that waypoint:

         Δ
v_{ref}= ─ min( 1.0,√(2(0.8)max(0,d-0.25)) )
         d

The three constants are:

| Parameter | Current value | Meaning |
|---|---:|---|
| Cruise speed | 1.0 m/s | Maximum requested speed |
| Braking acceleration | 0.8 m/s² | Shapes the approach-speed reference |
| Stopping distance | 0.25 m | Distance where the cruise reference reaches zero |

That braking acceleration is a guidance setting, not a guaranteed achieved acceleration.

Horizontal reference velocity is also multiplied by:

max(0.1,cos(heading error))

Consequently, the vehicle requests less horizontal motion when facing away from the target. It can still move while turning.

Velocity feedback then calculates:

F=Kᵥ(v_{ref}-v)+c v_{ref}-b

The terms mean:

- Correct the difference between requested and measured velocity.
- Supply the force needed to overcome nominal drag.
- Compensate nominal buoyancy.

With the default weights, Kᵥ≈7.40 on each axis.

For example, with a forward reference of 1 m/s and measured velocity zero:

Fₓ≈7.40(1-0)+3(1)=10.40 N

As measured speed increases, the feedback term decreases. If the vehicle moves faster than requested, that term becomes a braking force.

1 m/s is a reference limit; it does not guarantee that actual speed never exceeds 1 m/s.

5. How arrival and holding work

When the cruise reference reaches zero, your state configuration switches to:

F=Kₚ(p_{target}-p)-Kₛ v-b

For the default model and weights, each axis approximately uses:

F=12.40(p_{target}-p)-36.35v-b

Position error pulls the vehicle toward the target. Measured velocity provides braking.

For example, if the vehicle is 0.1 m short of the target but still moving toward it at 0.3 m/s:

F≈12.40(0.1)-36.35(0.3)=-9.66 N

That negative force brakes the approach.

Within 0.25 m in three dimensions, the node publishes an arrival report. It continues holding; arrival alone does not zero the motors. The mission-stop topic zeros them.

Because cruising uses a separate velocity gain, changing state_weights mainly changes arrival and holding behavior in your current configuration. velocity_weights affects travel.

6. How heading and thrusters are handled

Heading control is proportional:

τ_{z}= clamp (0.6 heading error,-10,10)

Heading error is wrapped to [-π,π].

Yaw is not controlled by DLQR. DLQR handles translation; proportional feedback handles yaw. There is no roll or pitch feedback loop.

The world force is rotated back into vehicle coordinates:

F_{body}=R(ψ)^{T}F_{world}

The requested body wrench is:

w=[Fₓ,F_{y},F_{z},0,0,τ_{z}]^{T}

Using NBV’s HoveringAUV geometry, the allocator finds eight thruster forces that produce this wrench. It uses the minimum-norm allocation.

If any command exceeds the node’s default 40 N limit, every command is scaled by the same factor. This preserves the requested wrench direction, but reduces its magnitude—including yaw torque.

7. What assumptions and limitations you should accept knowingly

The implementation assumes:

- Planner targets and estimated position use the same world origin and axes. It performs no TF conversion between map and odom.
- DVL velocity follows the expected vehicle-axis convention.
- Heading alone is sufficient for coordinate rotation; substantial roll or pitch would invalidate that approximation.
- Depth uses the same sign as target Z. The default leaves the sensor value unchanged.
- The nominal mass, drag, buoyancy, and thruster geometry sufficiently represent your simulator.

It also has no integral term. Unmodelled constant forces can cause a holding offset.

Freshness is checked using message reception time, not sensor acquisition timestamps. By default, missing sensor updates for 0.5 s or waypoint updates for 1 s cause zero commands. Zero thrust does not mean the moving vehicle instantly stops.

Finally, there is a concrete issue upstream: point_extract_navi.cpp (pipe_track_ros2/src/pipe_track_planning/src/point_extract_navi.cpp:84) compares candidate map-cell indices with vehicle position in metres when selecting the nearest target. It converts the selected target into metres afterward. That comparison can select the wrong waypoint even when the controller follows it correctly.

The numerical and ROS tests establish that the port implements the intended equations and interfaces. They do not establish successful navigation through a complete HoloOcean mission, especially with that planner issue still present.