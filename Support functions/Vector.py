import numpy


def orthogonal_complements(a: numpy.ndarray, b: numpy.ndarray) -> numpy.ndarray:
    c = a - numpy.dot(b, a.T) / numpy.linalg.norm(b)*b/numpy.linalg.norm(b)
    return c
