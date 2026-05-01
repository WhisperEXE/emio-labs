:::::: collapse Automated Formulation Benchmarking

To objectively compare the performance of the three inverse kinematics formulations discussed in the theoretical sections, you will run an automated benchmarking script. This script overrides manual control, moving the target in a "Figure-8" trajectory to test multi-axis tracking and boundaries. 

It will automatically log the **Tracking Error**, **Computation Time**, and **Motor Angles** into a CSV file.

* **Option 1:** Integrated Formulation (Native SOFA)
* **Option 2:** Reduced Actuator-Space Formulation (Standard QP)
* **Option 3:** OIM-Inspired Formulation (Light Regularization)

Select a formulation below to run the benchmark. Once the simulation finishes (or after letting it run for a few intervals), check the generated `benchmark_formX.csv` files in your directory. Plotting `Motor0_rad` against the `Step` will allow you to visualize the oscillatory "chattering" behavior of the reduced formulations!

:::: select exoformulation
::: option 1
::: option 2
::: option 3
::::

#runsofa-button("assets/labs/lab_inversekinematics/compare_ik_formulations.py" "--formulation" "exoformulation")

::::::