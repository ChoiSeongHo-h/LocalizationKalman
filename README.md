# sensor_fusion_GPS_ODOM_IMU_kalman_ROS

The algorithm of the above two files is similar.

localization_node_gps+odom+imu uses encoder-based odometry that includes speed and rotation with respect to the starting point and direction, and imu, where there is no integration error.

imu already has a gyroscope and magnetometer fusion.

localization_node_gps+oodom knows the speed information of the two wheels in the robot's coordinate system.

I will explain the first one first.

The odometry is not positioned correctly due to slip.

However, if there is no slip for rotation, the speed is relatively accurate.

The direction through odometry and imu has the same coordinate system when the robot is first run.

If imu is used, it is possible to obtain a direction relative to the initial direction of the robot that does not accumulate errors.

This is used to compensate the rotation of the robot.

Receive the robot's velocity vector and rotation through odometry. 

After that, it is corrected by rotating the velocity vector by the difference between the rotation direction of imu and the rotation direction of odometry.

Continue to integrate this to obtain the displacement.

The corrected odometry and gps are fused through a Kalman filter.

When GPS is received, it transforms the coordinates according to the current coordinate system.


for ‘x [x y vx vy]’

A =

[1 0 dt 0

0 1 0 dt

0 0 1 0

0 0 0 1]

But the speed changes.

It maintains the dimension of the elements in this matrix and transforms it into a similar form.

A =

[1 0 (x_o-x)/vx 0

0 1 0 (y_o-y)/vy

0 0 vx_o/vx 0

0 0 0 vy_o/vy]

The subscript o means that it is a value by odometry.


H =

[1 0 0 0

0 1 0 0] because only x and y will be used.


Q = 

[dt*unitVarX 0 0 0
0 dt*unitVarY 0 0
0 0 0.01 0
0 0 0 0.01]

Since gps are received at irregular intervals, the variance per unit time is multiplied by the incoming interval.

Since the variance for velocity is small, a very small value is entered.


R =

[1 0

0 1]. Assume that the distribution of gps is 1m horizontally and vertically.

P_0 and x_0 give appropriate variance and values ​​according to the initial position.

P_k is saved and updated whenever gps comes in.

x and y are extracted separately from the updated x_k, and the speed of odometry is added and used normally.

If the GPS does not come on for a long time, the error is accumulated because it is not corrected.

But once every second we get the gps value, we move slowly so that's ok.

Now figure out the direction.

It is assumed that the moving direction of the robot is the direction of the robot.

If you receive the gps value more than once and determine the absolute position, you can know the absolute direction.

Of course, we get the relative orientation via imu (since it's 0 when on), but that's not enough.

Determine the direction of movement of the robot.

The current position from the previous position will reflect the direction of the robot.

To this, the direction change due to the pure rotation of the robot is added.

This can be done by using imu to find the difference between the previous and current directions.

imu is relative, but difference in direction can be used.

Each time the GPS value is received, the position and direction of the robot change greatly, which is a clear error factor.

Design a filter to reduce this.

The first filter is the filter for the odometry integral.

LPF is applied to this. Although the integration itself removes the high frequency component, it goes through the filter once because it uses a pure value.

The second filter stores several positions of the robot and calculates the current direction with respect to more past positions.

Since past values ​​are inaccurate, they are weighted and averaged. → It is robust against errors and reflects the current value well.

The third filter is to apply the LPF again to the direction obtained through the above.

The second and third methods are non-overlapping, different methods.



localization_node_gps+oodom knows the speed information of the two wheels in the robot's coordinate system.

Here, the odometry does not change the coordinate system because the wheel speed belongs to the coordinate system of the robot.

A method similar to the above is applied by finding out the speed, displacement, and angular change of the robot with the wheel speed.
