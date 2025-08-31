# World and Object Representation

## Exercises

1. **Bounding Box Definition**
- Given a 2D bounding box defined by $\displaystyle (x_{min},y_{min},x_{max},y_{max}) = (2,3,6,8)$, list all four corner coordinates.

    The bounding box corner will be given by $\displaystyle p_1=(2,3),p_2=(2,8),p_3=(6,8) {\textstyle and} p_4=(6,3)$.

    ![Bounding box](./img/bounding_box.png)

- Compute the area of the bounding box.

    The area of the bounding box is,

    $$box_{area}=(x_{max}-x_{min})\times(y_{max}-y_{min})$$

    Then,

    $$box_{area}=(6-2)\times(8-3) = 20$$

    The $\displaystyle box_{area}=20$.

2. **Bounding Boxes and Occupied Space**
- (a) Given a 3D bounding box with parameters $\displaystyle (x,y,z,l,w,h,\Psi)=(5,3,0,4,2,2,45^\circ)$, compute the volume occupied by the object.

    The volume $\displaystyle box_{volume}$ occupied by the object is given by:

    $$box_{volume}=l \times w \times h = 4 \times 2 \times 2 = 16$$

- (b) If the bounding box in (a) is rotated by $\displaystyle \Psi=45^\circ$, sketch (or describe) how the occupied space differs compared to $\displaystyle \Psi=0^\circ$.

    The heading $\displaystyle \Psi$ is the measured angle of object local reference relative to the global origin coordinate $\textstyle X-axis$, using the right hand rule. Which means that the rotation will be along the $\textstyle Z-axis$. The Z rotation matrix is given by:

    $$R_z(\theta)=\begin{bmatrix}
        \cos\theta & -\sin\theta & 0 \\\\ \sin\theta & \cos\theta & 0 \\\\ 0 & 0 & 1\end{bmatrix}$$

    Assuming the initial object direction is in $\textstyle X-axis$, we can write,

    $$P=\left[\begin{array}{c} x \\\\ y \\\\ z \end{array}\right]$$

    The rotation is,

    $$P_r=R_z(\theta) \times P=\begin{bmatrix}
        \cos\theta & -\sin\theta & 0 \\\\ \sin\theta & \cos\theta & 0 \\\\ 0 & 0 & 1 \end{bmatrix} \times \left[\begin{array}{c} x \\\\ y \\\\ z \end{array}\right]$$
    
    $$P_r=\begin{bmatrix} x\cos\theta-y\sin\theta \\\\
        x\sin\theta+y\cos\theta \\\\
        z \end{bmatrix}$$

    The translation matrix is,

    $$T=\begin{bmatrix} 1 & 0 & 0 & x \\\\ 0 & 1 & 0 & y \\\\ 0 & 0 & 1 & z \\\\ 0 & 0 & 0 & 1 \end{bmatrix}$$
    
    Applying the rotation and translation for each corner, $\displaystyle P'=T\times R_z \times P$,

    |<img src="./img/3d_bounding_box_psi_45.png" alt="Psi 45 degree" height=300 width=300>|<img src="./img/3d_bounding_box_psi_0.png" alt="Psi 0 degree" height=300 width=300>|
    |-|-|
    |*Show bounding box when $\Psi=45^\circ$*|*Show bounding box when $\Psi=0^\circ$*|

- (c) Two 2D bounding boxes $\textstyle B_1 and B_2$ are defined as:

    $$B_1:(x_{min},y_{min},x_{max},y_{max})=(0,0,4,3), B_2:(2,1,6,5).$$

    Compute their intersection-over-union (IoU).

    Given the two bounding boxes depicted in the image bellow, we will calculate the $\displaystyle A(B1\cap B2)$ first,

    ![bounding box plot](./img/iou.png)

    The intersection are, denoted by $\displaystyle A(B1\cap B2)$ can be calculated as,

    $$A(B1\cap B2)=(b1_{x_{max}}-b2_{x_{min}}) \times (b1_{y_{max}}-b2_{y_{min}})$$

    $$A(B1\cap B2)=(4-2) \times (3-1)=4$$

    The area for each bounding box is,

    $$A(B1)=4 \times 3 = 12,\\ A(B2)=(5-1) \times (6-2)=4 \times 4 = 16$$

    The area of the union,

    $$A(B1 \cup B2)=A(B1)+A(B2)-A(B1\cap B2) \\\\
    A(B1 \cup B2)=12+16-4=24$$

    Finally, we can calculate the $IoU$,

    $$IoU=\frac{A(B1\cap B2)}{A(B1 \cup B2)} = \frac{4}{24}= \frac{1}{6} \approx 0.1666$$

3. **Confidence and Uncertainty**
- A detector outputs a bicycle at 0.65 confidence. Discuss whether this uncertainty is more likely to be aleatoric or epistemic in the following scenarios:

  - (a) The bicycle is partially hidden behind a bus.

    This is more likely to be *aleatoric*. The bicycle is only partially observable by the detector.

  - (b) The detector was trained mostly on car and pedestrian classes, with few examples of bicycles.

    This is more likely to be *epistemic*. The detector need a better dataset with more classes for bicycles.

- A detector identifies a pedestrian at 0.85 confidence under foggy conditions. Which type of uncertainty (aleatoric or epistemic) is dominant, and why?

    The dominant uncertainty is *aleatoric*. The foggy condition is a environmental effect. Note that in such adverse condition, this high 0.85 confidence could be a false positive.

- An object is detected at 0.40 confidence. Suggest two possible ways to reduce epistemic uncertainty for this case.

    Increase the dataset for this specific scenario by collecting more data under similar condition. And create more scenarios, to increase detection confidence in new-but-similar scenarios where the detector will have good confidence even in cases it never saw, but was trained in a dataset with similar scenario.

- Suppose the detector assigns high confidence (0.95) to a phantom detection caused by sensor reflection. Discuss why confidence alone may be misleading in safety-critical contexts.

    This is a false positive case where the detector sees something that is not there. Confidence is the indicator on how confident the detector is about its detection. Factors such as bad bounding box selection, lack of training data, faulty sensors, etc, can trick the detector into wrong classifications.

4. **Coordinate Transformation**
- An object is observed in the local frame of vehicle $O_1$ at position $(px, py) = (10,5)$ m.
- The origin of $O_2$ is 4m east and 3m north of $O_1$.
- Compute the position of the object relative to $O_2$ assuming both vehicles use ENU coordinates.

    Given the $Object$ and origins $O_1$ and $O_2$, depicted bellow,

    ![o1_o2_object](./img/o1_o2.png)

    The position $P'$ of the object relative to $O_2$ is,

    $$P'=(x_{obj}-x_{O_2},y_{obj}-y_{O_2})=(10-4,5-3)$$
    $$P'=(6,2)$$

- An observer $O_3$ detects and object at $(p_x,p_y)=(15,-3)$ m. Observer $O_4$ is located 10 m east and 2 m north of $O_3$. Express the object's coordinates in $O_4$'s frame.

    Given the $Object$ and origins $O_3$ and $O_4$, depicted bellow,

    ![o3_o4_object](./img/o3_o4.png)

    The transformation matrix $T_{O_3 \to O_4}$ is,

    $$
    T_{O_3 \to O_4}= \begin{bmatrix}
        1 & 0 & -10\\\\
        0 & 1 & -2\\\\
        0 & 0 & 1
    \end{bmatrix}
    $$

    The position $P'$ of the object relative to $O_4$' frame is,

    $$P'=T_{O_3 \to O_4}(P)=T_{O_3 \to O_4} \times P$$
    $$P'= \begin{bmatrix}
        1 & 0 & -10\\\\
        0 & 1 & -2\\\\
        0 & 0 & 1
    \end{bmatrix} \times
    \begin{bmatrix}
    15\\\\
    -3\\\\
    1
    \end{bmatrix}=(5,-5)$$

    Note: The position should be augmented by 1 row.

- Generalize the previous problem by deriving the transformation matrix $T_{O_3 \to O_4}$ for arbitrary displacements $(\Delta x, \Delta y)$ between observers.

    The transformation matrix $T_{O_3 \to O_4}$ can be expressed as,

    $$T'_{O_3 \to O_4}=\begin{bmatrix} 1&0&(-10+\Delta x)\\\\0&1&(-2+\Delta y)\\\\0&0&1 \end{bmatrix}$$

- If observer $O_4$ is additionally rotated by $30^\circ$ with respect to $O_3$, update the transformation and compute the new object coordinates.

    To better visualize the rotation, the can imagine the points rotating around a imaginary $Z-axis$. Given the 3D rotation transformation matrix for the $Z-axis$,

    $$R_z(\theta)=R(\theta)=\begin{bmatrix}
        \cos\theta & -\sin\theta & 0\\\\
        \sin\theta & \cos\theta & 0\\\\
        0 & 0 & 1
    \end{bmatrix}$$

    The updated transformation to transform a object in $O_3$'s coordinate to $O_4$ coordinate, must first rotate the $O_4$ then apply the transformation,

    $$
    O'_4=R(\theta)\times O_4=
    \begin{bmatrix}
        \cos\theta & -\sin\theta & 0\\\\
        \sin\theta & \cos\theta & 0\\\\
        0 & 0 & 1
    \end{bmatrix} \times
    \begin{bmatrix}
    10\\\\
    2\\\\
    1
    \end{bmatrix}=
    \begin{bmatrix}
        10\cos\theta-2\sin\theta\\\\
        10\sin\theta+2\cos\theta\\\\
        1
    \end{bmatrix}
    $$

    $$
    T_{O_3 \to O'_4}=
    \begin{bmatrix}
        1 & 0 & -(10\cos\theta-2\sin\theta)\\\\
        0 & 1 & -(10\sin\theta+2\cos\theta)\\\\
        0 & 0 & 1
    \end{bmatrix}
    $$

    Applying the transformation $T_{O_3 \to O'_4}$ to $(px,py)=(15,-3)$ and making $\theta=30^\circ$,

    $$
    P''=T_{O_3 \to O'_4}(15,-3)=
    \begin{bmatrix}
        1 & 0 & -(10\cos30^\circ-2\sin30^\circ)\\\\
        0 & 1 & -(10\sin30^\circ+2\cos30^\circ)\\\\
        0 & 0 & 1
    \end{bmatrix} \times
    \begin{bmatrix}
        15\\\\
        -3\\\\
        1
    \end{bmatrix}=
    \begin{bmatrix}
        15-(10\cos30^\circ-2\sin30^\circ)\\\\
        -3-(10\sin30^\circ+2\cos30^\circ)\\\\
        1
    \end{bmatrix}
    $$

    Thus, the position of the object relative to $O'_4$, depicted on the image, is,

    $$
    P''\approx(7.34,-9.73)
    $$

    ![o3_o'_4](./img/o3_04_30.png)

5. **Motion Vector Propagation**
- A car modeled with the CTRV state vector:
    
    $$
    x_k=\left[p_x,p_y,v,\Psi,\dot{\Psi}\right]=[0,0,20,30^\circ,0]
    $$

    where speed is 20m/s, heading $30^\circ$ and yaw rate 0.
- Compute is position after $\Delta t=2$s.

    Given a $\dot{\Psi}=0$, the CTRV model fallback to a CV model, thus, we cal calculate the car position using,

    $$
    x_{k+1}=x_k+v\cos\Psi\Delta t
    $$

    $$
    y_{k+1}=y_k+v\sin\Psi\Delta t
    $$

    Substituting the values into the equations,

    $$
    P=
    \begin{bmatrix}
        0+20\cos30^\circ\times 2\\\\
        0+20\sin30^\circ\times 2
    \end{bmatrix}^T=
    (34.64,20)m
    $$

6. **Comparing Motion Models**
- Using the same initial conditions as Exercise 5, describe qualitatively how the predicted position would differ if the CTRA model were used with a longitudinal acceleration of $a=2m/s²$.

    The CTRA model takes a motion vector $x_k=\left[p_x,p_y,v,\Psi,\dot{\Psi},a\right]^T$, substituting the values, $x_k=\left[0,0,20,30^\circ,0,2\right]^T$.

    Same as the Exercise 5, there is a discontinuity when $\dot{\Psi}=0$, thus, the CTRA model becomes,

    $$
    x_{k+1}=x_k+\left[\left(v\Delta t+\frac{at²}{2}\right)\cos\Psi\right]
    $$

    $$
    y_{k+1}=y_k+\left[\left(v\Delta t+\frac{at²}{2}\right)\sin\Psi\right]
    $$

    Substituting the values,

    $$
    P=
    \begin{bmatrix}
    0+\left[\left(20\times 2+\frac{2\times 2²}{2}\right)\cos30^\circ\right]\\\\
    0+\left[\left(20\times 2+\frac{2\times 2²}{2}\right)\sin30^\circ\right]
    \end{bmatrix}^T=
    (38.01,22)m
    $$

    As the CTRV model does not take in consideration the longitudinal acceleration, the final position can be considerably different after some time.

- Likewise, compare with the CV model.

    As noticed on Exercise 5, for this particular case where the $\dot{\Psi}=0$, the observations made on the CTRA are the same for the CV. For a value of $\dot{\Psi}\neq 0$, the difference between CTRA and CV final positions would be much higher.

## Reference
[[1]](https://geogebra.org). Images generated using the online tool available in https://geogebra.org.

<script type="text/javascript" src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>
<script type="text/x-mathjax-config">
    MathJax.Hub.Config({ tex2jax: {inlineMath: [['$', '$']]}, messageStyle: "none" });
</script>