# World and Object Representation

## Exercises

1. **Bounding Box Definition**
- Given a 2D bounding box defined by $\displaystyle (x_{min},y_{min},x_{max},y_{max}) = (2,3,6,8)$, list all four corner coordinates.

The bounding box corner will be given by $\displaystyle p_1=(2,3),\;p_2=(2,8),\;p_3=(6,8)\; {\textstyle and}\; p_4=(6,3)$.

![Bounding box](./img/bounding_box.png)

- Compute the area of the bounding box.

The area of the bounding box is,

```math
box_{area}=(x_{max}-x_{min})\times(y_{max}-y_{min})
```

Then,

```math
box_{area}=(6-2)\times(8-3) = 20
```

The $\displaystyle box_{area}=20$.

2. **Bounding Boxes and Occupied Space**
- (a) Given a 3D bounding box with parameters $\displaystyle (x,y,z,l,w,h,\Psi)=(5,3,0,4,2,2,45^\circ)$, compute the volume occupied by the object.



- (b) If the bounding box in (a) is rotated by $\displaystyle \Psi=45^\circ$, sketch (or describe) how the occupied space differs compared to $\displaystyle \Psi=0^\circ$.



- (c) Two 2D bounding boxes $\textstyle B_1\; and\; B_2$ are defined as:
    ```math
    B_1:(x_{min},y_{min},x_{max},y_{max})=(0,0,4,3),\; B_2:(2,1,6,5).
    ```
    Compute their intersection-over-union (IoU).