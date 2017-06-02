# Control-Systems-Rotary-Control
Code used to model and analyze a rotary control experiment for my control systems lab.

![Alt text](/FlexibleArmApparatus.jpg?raw=true "Flexible Arm Apparatus")
![Alt text](/experimental_setup.jpg?raw=true "Experimental Setup")


More In-Depth
------

In order to move a flexible arm optimally with a DC motor using PD control, we had to model its movement and tip deflection using by deriving closed-loop transfer functions for each of these and analyzing the step response with MATLAB. We then found optimal gain values that, within the constraints, moved the arm in a critically damped fashion to the desired position.

![Alt text](/Control_diagram.jpg?raw=true "Control Block Diagram")
![Alt text](/rigidKd.jpg?raw=true "Model Dampening Comparison")

This code:
* Defines parameters of the physical experimental system.
* Defines the transfer function for a rigid arm's movement -- then models this to find optimal gain values.
* Defines transfer functions for flexible arm movement and tip deflection -- then models this to find optimal gain values.
* Compares the flexible arm model to our experimental data.
![Alt text](/comparison.jpg?raw=true "Model-Experiment Comparison")
