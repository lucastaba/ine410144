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