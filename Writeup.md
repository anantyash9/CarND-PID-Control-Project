**PID Controller**

The PID controller or proportional–integral–derivative controller uses 3 terms to device a control stratigy for a process.
In this project, we modulate the steering and throttle to keep the car as close to the center of the road as possible.

* **P** is proportional to the error (required value - current value). So, if the error is large and positive, the control output will be proportionately large and positive, taking into account the gain factor "K".In the PID used for controlling the steering value, this error term is cross track error (CTE) which is the perpendicular distance between the current position of the car and the center of the track. If this value is large the P term will also be large.  
* **I** is the integration of the error over time. If there is a bias (environmental or systematic) which causes a relatively small offset in the output it may not be sufficient to cause a response in the loop. By integrating the error over a period of time we cause the output to react as though the error was increasing with time so eventually causing a reaction in the loop.
* **D** is the rate of change of the error. This term damps the controller so that the control value won't keep oscillating indefinitely. It anticipates the future state of the control value and counters the P term when the rate of change is high.This term is essentially what makes the PID controller smooth. The more rapid the change, the greater the dampening effect.

I tuned the Gain values(Kp, Ki, Kd) manually. The values that worked best for me are 

For Steering controll 
* Kp - 0.18
* Ki - 0.0002
* Kd - 1.6

For  throttle controll
* Kp - 0.1
* Ki - 0.0002
* Kd - 2.0

While tuning the Gane values I observed that:

* For low values of **Kp** the car takes a long time to come to the center of the road and doesn't turn hard enough in curves to stay on road. On the other hand for higher value, the car starts to wobble with increasing intensity until it goes completely off track.
* For high values of **Ki** the car starts to behave unpredictably often making sharp turns on straight stretches. 
* For low values of **Kd** the car takes a long time to stop wobbling and gets out of controll quite soon. For higher values it takes a long time to come to the center of the road and doesn't perform well in curves.
