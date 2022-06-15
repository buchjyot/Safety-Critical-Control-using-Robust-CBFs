
# Safety-Critical-Control-using-Robust-CBFs

This repository is the reference implementation of our paper:

### Robust Control Barrier Functions with Sector-Bounded Uncertainties
by _Jyot Buch, Shih-Chi Liao, Peter Seiler_

This paper focus on safety critical control with sector-bounded uncertainties at the plant input. The uncertain-ties can represent nonlinear and/or time-varying  components. We propose a new robust control barrier function (RCBF) approach to enforce safety requirements in the presence of these uncertainties. The primary objective is to minimally alterthe given baseline control command to guarantee safety in the presence of modeled uncertainty. The resulting min-norm optimization problem can be recast as a  Second-Order Cone Program (SOCP) to enable online implementation. Properties of this controller are studied and a numerical example is provided to illustrate the effectiveness of  this approach.

Paper: [IEEE Control Systems Letters](https://ieeexplore.ieee.org/document/9656566) or [arXiv](https://arxiv.org/abs/2109.02537)        

Presentation: [slides at American Control Conference 2022](https://github.com/buchjyot/Safety-Critical-Control-using-Robust-CBFs/blob/main/doc/Presentation_ACC20220608.pdf)

#### Citing
If this project is helpful, please consider citing the following work:
```
@ARTICLE{9656566,
        author={Buch, Jyot and Liao, Shih-Chi and Seiler, Peter},
        journal={IEEE Control Systems Letters}, 
        title={Robust Control Barrier Functions With Sector-Bounded Uncertainties}, 
        year={2022},
        volume={6},
        number={},
        pages={1994-1999},
        doi={10.1109/LCSYS.2021.3136653}
}
```
