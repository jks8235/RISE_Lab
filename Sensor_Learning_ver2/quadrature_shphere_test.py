#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Kyungsub Jee (RISE)

import numpy as np
import quadpy

scheme = quadpy.u3.get_good_scheme(6)
print(scheme.points)
print(scheme.degree)
scheme.show()


# scheme_2 = quadpy.u3.get_good_scheme(45)
# scheme_2.show()

# val = scheme.integrate(lambda x: np.exp(x[0]), [0.0, 0.0, 0.0], 1.0)
# print(val)