# Homework 2: Kinematics

## \# Car Kinematics

In this part of the assignment, you will work with the MuSHR platform. The goal is to implement the kinematics model of the MuSHR car in 2D.

**Q1**: Kinematics car model
<details>
<summary>
Kinematics model derivation
</summary>
Let’s first review the kinematic model of the car and annotate the relevant lengths and angles (add figure).
First, let’s assume that there is no control and that the velocity was stable and the steering angle is <em>zero</em>. We can then write out the change in states:

$$
\begin{align*}
&\dot{x} = v \cos \theta \\
&\dot{y} = v \sin \theta \\
&\dot{\theta} = \omega
\end{align*}
$$
where $\omega$ is the angular velocity from the center of rotation to the center of the rear axle.
By the definition of angular velocity:
$$
\omega = \frac{v}{R} = \frac{v}{L} \tan \delta
$$
Formally, the changes in states are:
$$
\begin{align*}
&\frac{\partial x}{\partial t} = v \cos \theta \\
&\frac{\partial y}{\partial t} = v \sin \theta \\
&\frac{\partial \theta}{\partial t} = \frac{v}{L} \tan \delta
\end{align*}
$$
After apply control $\mathbf{u}_t$, integrate over the time step:
$$
\begin{align*}
&\int_{\theta_{t}}^{\theta_{t+1}}d\theta =
\int_{t}^{t+\Delta t} \frac{v}{L} \tan \delta dt \\
&\theta_{t+1}-\theta_{t} =\frac{v}{L} \tan \delta \Delta t
\end{align*}
$$

Changes in positions: 
$$
\int_{x_{t}}^{x_{t+1}} d x
= \int_{t}^{t+\Delta t} v \cos \theta d t
= \int_{t}^{t+\Delta t} v \cos \theta \frac{d \theta}{\frac{v}{L} \tan \delta}
= \frac{L}{\tan \delta} \int_{\theta_{t}}^{\theta_{t+1}} \cos \theta d \theta
$$

$$
x_{t+1}-x_{t} = \frac{L}{\tan \delta}\left[ \sin \theta_{t+1} - \sin \theta_{t} \right]
$$
$$
\int1_{y_{t}}^{y_{t+1}} dy
= \int_{t}^{t+\Delta t} v \sin \theta d t
= \int_{t}^{t+\Delta t} v \sin \theta \frac{d \theta}{\frac{v}{L} \tan \delta}
=\frac{L}{\tan \delta} \int_{\theta_{t}}^{\theta_{t+1}} \sin \theta d \theta
$$
$$
y_{t+1}-y_{t} =\frac{L}{\tan \delta} \left[ -\cos \theta_{t+1} + \cos \theta_{t} \right]
$$
Putting it all together:
$$
\begin{align*}
&\theta_{t+1} = \theta_{t} + \frac{v}{L} \tan \delta \Delta t \\
&x_{t+1}
= x_{t} + \frac{L}{\tan \delta} \left[ \sin \theta_{t+1} - \sin \theta_{t} \right] \\
&y_{t+1}
= y_{t} + \frac{L}{\tan \delta} \left[ -\cos \theta_{t+1} + \cos \theta_{t} \right] \\
\end{align*}
$$
</details>

**Q1.1 (25 points)** Implement the kinematic car equations in the `KinematicCarMotionModel.compute_changes` method (`src/kinematics/kinematic_model.py`). Note that this method is deterministic: given initial state $\mathbf{x}_{t-1}$ and control $\mathbf{u}_t$, it integrates the kinematic car equations and returns a new state $\mathbf{x}_t$.

<details>
<summary>
What if the steering angle is 0?
</summary>

The update rule we derived divides by $\tan \delta$, which means we’re dividing by 0 if the steering angle is 0. (Mathematically, the center of rotation is now infinitely far away.) Fortunately, the update rule becomes even simpler for this case: if there’s no steering angle, the car continues driving at angle $\theta$ with velocity $v$.

$$
\begin{align*}
&\theta_{t+1} = \theta_{t} \\
&x_{t+1} = x_{t} + v \cos \theta \Delta t\\
&y_{t+1} = y_{t} + v \sin \theta \Delta t
\end{align*}
$$

In your implementation, you should treat any steering angle where $| \delta |$ is less than `delta_threshold` as a zero steering angle.
</details>

You can verify your implementation on the provided test suite by running 
```
roscd car_kinematics/
python3 test/kinematic_model.py
``` 

Your code should pass all test cases starting with `test_compute_changes`.

After successfully implementing the kinematic car equations, we would want to propogate the changes (obtained using `KinematicCarMotionModel.compute_changes`) across states to generate the motion of our car.

**Q1.2 (20 points)**: Implement the simple deterministic motion model in the `KinematicCarMotionModel.apply_deterministic_motion_model` method (`src/kinematics/kinematic_model.py`). Your implementation should also **wrap**<a href="#fn3" class="footnote-ref" id="fnref3" role="doc-noteref"><sup>3</sup></a> the resulting $\theta$ component of the state to the interval $(-\pi, \pi]$. Angles that differ by a multiple of $2\pi$ radians are equivalent. For example, if the resulting $\theta$ component was $\theta = \frac{3\pi}{2}$, your implementation should return the equivalent angle $\theta = -\frac{\pi}{2}$.

After completing **Q1.2**, you should pass all test cases starting with `test_apply_deterministic`.

<blockquote>
Remember the plot you generated for Project 1 comparing the `norm_python` and `norm_numpy` computation times? Your implementations for Q1.1 and Q1.2 should be <em>vectorized</em> using <a href="https://numpy.org/doc/stable/reference/arrays.indexing.html">NumPy indexing</a>, rather than iterating over each particle (representing the state) with Python for loops. (Both the “Basic Slicing and Indexing” and “Advanced Indexing” sections of that page are useful, but <a href="https://numpy.org/doc/stable/reference/arrays.indexing.html#boolean-array-indexing">Boolean array indexing</a> will be particularly useful when thresholding $\delta$.)
</blockquote>
<br>
We've provided a script to visualize the motion of a car using the deterministic model.

```
rosrun car_kinematics make_rollout_plot
```

The staff solution produces the following plot after running the command above. Try to match these plots by correctly implementing the `apply_deterministic_motion_model` method.
<figure class="figure mw-100 w-500px">
    <a id="fig-motion-model-cluster">
    <img src="make_rollout_plot.png"
         alt="Figure 2: Deterministic rollouts"
         class="figure-img img-fluid"
    /> </a>
    <figcaption class="figure-caption">
            Figure 2: Rollouts generated using the deterministic motion model. The initial state (green arrow), final state (red arrow) and the intermediate states from integrating the deterministic model (red dots).
        </figcaption>
</figure>

Next, to make this simple kinematic model robust to various sources of modeling error, you’ll add noise in three steps. Noise is parameterized by $\sigma_v, \sigma_\delta$ (action noise) and $\sigma_x, \sigma_y, \sigma_\theta$ (model noise).

<ol type="1">
<li>

Given <em>nominal</em> controls $\mathbf{u}_t = (v_t, \delta_t)$, sample <em>noisy</em> controls $\hat{\mathbf{u}}_t = (\hat{v}_t, \hat{\delta}_t)$ where $\hat{v}_t \sim \mathcal{N}(v_t, \sigma_v^2)$ and $\hat{\delta}_t \sim \mathcal{N}(\delta_t, \sigma_\delta^2)$.</li>
<li>

Integrate kinematic car equations with noisy controls $\Delta \hat{\mathbf{x}}_t = \int_{t-1}^t f(\mathbf{x}_{t-1}, \hat{\mathbf{u}}_t) dt$ (by calling the `compute_changes` method).</li>
<li>

Add model noise to the output $\Delta \mathbf{x}_t \sim \mathcal{N}(\Delta \hat{\mathbf{x}}_t, \mathrm{diag}(\sigma_x^2, \sigma_y^2, \sigma_\theta^2))$.<a href="#fn2" class="footnote-ref" id="fnref2" role="doc-noteref"><sup>2</sup></a></li>
</ol>

**Q1.3 (15 points)**: Reuse the code written in `KinematicCarMotionModel.apply_deterministic_motion_model` method and implement a noisy motion model in the `KinematicCarMotionModel.apply_motion_model` method (`src/car_kinematics/kinematic_model.py`). There is an instance attribute corresponding to each noise parameter: `vel_std` corresponds to $\sigma_v$, `delta_std` corresponds to $\sigma_\delta$, etc. These instance attributes can be accessed with dot notation, e.g., `self.vel_std`.
After completing Q1.1, Q1.2 and Q1.3, expect your code to pass all the test cases in `test/kinematic_model.py`.

<br>

## [Optional] Exploring the Motion Model Parameters

**Bonus Points**: The noise in this motion model is controlled by the parameters $\sigma_v, \sigma_\delta$ (action noise) and $\sigma_x, \sigma_y, \sigma_\theta$ (model noise). We’ve provided some initial values in `config/parameters.yaml`, but it’ll be up to you to tune them and make sure they’re reasonable. We’ve provided a script to visualize samples from your probabilistic motion model, under the current noise parameters in `config/parameters.yaml`.

```
rosrun car_kinematics make_motion_model_plot
```

The staff solution produces the following plots with our motion model that is tuned to match the physical MuSHR car. Try to match these plots by tuning your parameters.
<figure class="figure mw-100 w-500px">
    <a id="fig-motion-model-cluster">
    <img src="motion_model_cluster.png"
         alt="Figure 3: The initial state (green) and state from integrating the deterministic model (red). Particles (blue) after moving forward and slightly to the left."
         class="figure-img img-fluid"
    /> </a><figcaption class="figure-caption">
            Figure 3: The initial state (green) and state from integrating the deterministic model (red). Particles (blue) after moving forward and slightly to the left.
        </figcaption>
</figure>

<figure class="figure mw-100 w-500px">
    <a id="fig-motion-model-banana">
    <img src="motion_model_banana.png"
         alt="Figure 4: Particles (blue) after moving forward and to the left. This banana-shaped curve is expected when noise is added to the system."
         class="figure-img img-fluid"
    /> </a><figcaption class="figure-caption">
            Figure 4: Particles (blue) after moving forward and to the left. This banana-shaped curve is expected when noise is added to the system.
        </figcaption>
</figure>

> In `scripts/make_motion_model_plot`, look at the `example_cases` dictionary that generates these two plots. Each key of this dictionary is a tuple of nominal velocity, nominal steering angle, and duration to apply those nominal commands. Why does the motion model produce more particles within 10cm of the deterministic model prediction in Figure 2 than Figure 3?

The key to tuning is first understanding how each parameter affects the model.

>Tune your motion model parameters to match the staff parameters. To explain your tuning process, please save three versions of the figure with (3.0, 0.4, 0.5) that were generated by different parameters (`mm1.png`, `mm2.png`, `mm3.png`). In your writeup, explain what’s different between your plot and Figure 3 above, and how you’re changing the parameters in response to those differences. (Your last figure doesn’t need to match ours exactly.)

<details>
<summary>Rubric</summary>

+ For <b>Q1.1</b>, 25 points if all test cases pass. Deduct two points for every failed test case.
+ For <b>Q1.2</b>
    + 10 points if all test cases pass. Deduct 4 points for every failed test case.
    + 10 points for matching the output in Figure 2.
+ For <b>Q1.3</b>, 15 points if all test cases pass. Deduct 2 point for every failed test case.
</details>
<br>

## \# Arm Kinematics
### Overview
In this assignment, you will work with an 6 dof arm robot called [WidowX 250](https://www.trossenrobotics.com/widowx-250-robot-arm.aspx). 

<figure>
  <img src="widowx250.jpg" alt="alt text" width="400"/>
  <figcaption> picture src: https://www.trossenrobotics.com/widowx-250-robot-arm.aspx </figcaption>
</figure> 


Feel free to run the launch file and have a look at your workspace.
```
# in terminal 1
$ roslaunch arm_kinematics fk.launch
```
You should see both `rviz` and `joint_state_publisher_gui` (might be behind rviz window) being launched. `joint_state_publisher_gui` allows you to use sliders to control all joint values. Feel free to play around with it.


To simplify computation, we will treat this arm as a 4 link robot. Each link is assigned a different color. 

<!-- + Base link: black
+ Shoulder link: blue
+ forearm link: green
+ gripper link: dark blue -->

You will also see a frame called `fk_frame` that is alone in the wild. `fk_frame`'s location is computed based on your implementation of forward kinematics, and your task is to implement forward kinematics from `wx250s/base_link` to `wx250s/ee_gripper_link` so that `fk_frame` is perfectly aligned with `wx250s/ee_gripper_link`.

On the left panel of Rviz, you will find a dropdown menu for [`TF`](http://wiki.ros.org/tf) package (shown below). This package allows you to keep track of multiple coordiante frames, and you will encounter this many times during the course. Take a look at the [website](http://wiki.ros.org/tf), and you might find many tools useful for debugging.

As you can see in the image below, we have checked the frames that we care about in this assignment, feel free to visualize other frames, but they are treated as fixed joints for Q2.

<figure>
  <img src="tf.png" alt="alt text" width="200"/>
  <figcaption> tf interface </figcaption>
</figure> 

**Q2**:

**Important**: First, click the `Center` button on `joint_state_publisher_gui` to zero all joint angles. 

Since we are treating WidowX250 as a 4-link robot, we will only concern ourselves with waist angle, should angle, and elbow angle. Please leave other joint angles at 0.00 at all time.

Your task is to complete function `compute_fk` in `src/arm_kinematics/fk_broadcaster`. This function should compute forward kinematics from `wx250s/base_link` to `wx250s/ee_gripper_link`. Specifically, your implementation should contain transformation matrices from `wx250s/base_link` to `wx250s/shoulder_link`, from `wx250s/shoulder_link` to `wx250s/upper_arm_link`, from `wx250s/upper_arm_link` to `wx250s/upper_forearm_link`, and from `wx250s/upper_forearm_link` to `wx250s/ee_gripper_link`. Once you finish this function, the broadcaster will publish a frame `fk_frame` using your implementation, and you should expect to see `fk_frame` is aligned with `wx250s/ee_gripper_link`. 


Once the two frames are aligned, feel free to change the waist, shoulder, and elbow joint values using the `joint_state_publisher_gui` to test your implementation with different angles.

**Hint**: tf is an awesome ros package, and you will work with it almost every time you use ROS. To obtain transformation between frames, use `tf_echo [source_frame] [target_frame]`.

For example, at startup configuration, 
```
$ rosrun tf tf_echo wx250s/base_link wx250s/ee_gripper_link
At time 1626900134.757
- Translation: [0.458, 0.000, 0.361]
- Rotation: in Quaternion [0.000, -0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.001, 0.000]
            in RPY (degree) [0.000, -0.046, 0.000]
```

Your implementation should pass all four tests. 
```
roscd arm_kinematics
python3 test/forward_kinematics.py
```

<details>
<summary>Rubric</summary>

Tolerance is 0.01 for every element in position and quaternion.
+ if failed position test: -50%
+ if failed orientation test: -50%
</details>