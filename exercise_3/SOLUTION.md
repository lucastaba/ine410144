# State Estimation and Positioning

## Exercises

1. **Linear State Observers**
   1. Considering the model from equation (4.1) write a simulation and implement a state observer to estimate the system’s velocity from position measurements and compare the results, qualitatively and quantitatively, with the position difference estimator and the acceleration integration estimator for different values for the measurement noise variance and the input uncertainty variance, but disregarding (setting to zero) the input uncertainty bias.

    ```math
    \begin{cases}
    p_{k+1} = p_k + \Delta t \nu_k \\
    \nu_{k+1} = \nu_k + \Delta t a_k \\
    \end{cases}
    ```