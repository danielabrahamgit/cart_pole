# Cart Pole Simulator

### Dependencies
To run this, you will need to have the `arcade` library installed. You can do this using pip:
```
pip3 install arcade
```


### How to use
The following command will run the visualization.

```
python3 cart_poly.py [kp] [ki] [kd]
```  

 in your terminal window. You can play around with different
and see the resulting simulation. kp, ki, and kd are the proprtional, integral, and derivative gains that you are expected to fill in with real numbers. An example is shown below:

```
python3 cart_poly.py 100 0 20
```  

Another important parameter is `u_max` which is defined in the code. This is the maximum input the the system 
can provide. Having a large calue of `u_max` is less realistic, as most real world systems cannot generate 
very large forces on the cart.
