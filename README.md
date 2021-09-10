
# Safety-Critical-Control-using-Robust-CBFs

This repository is the reference implementation of our paper:

### Robust Control Barrier Functions with Sector-Bounded Uncertainties
[PDF](https://arxiv.org/abs/2109.02537)

_Jyot Buch, Shih-Chi Liao, Peter Seiler_

This paper focus on safety critical control with sector-bounded uncertainties at the plant input. The uncertain-ties can represent nonlinear and/or time-varying  components. We propose a new robust control barrier function (RCBF) approach to enforce safety requirements in the presence of these uncertainties. The primary objective is to minimally alterthe given baseline control command to guarantee safety in the presence of modeled uncertainty. The resulting min-norm optimization problem can be recast as a  Second-Order Cone Program (SOCP) to enable online implementation. Properties of this controller are studied and a numerical example is provided to illustrate the effectiveness of  this approach.

#### Citing
If this project is helpful, please consider citing the following work:
```
@misc{buch2021robust,
      title={Robust Control Barrier Functions with Sector-Bounded Uncertainties}, 
      author={Jyot Buch and Shih-Chi Liao and Peter Seiler},
      year={2021},
      eprint={2109.02537},
      archivePrefix={arXiv},
      primaryClass={math.OC}
}
```
