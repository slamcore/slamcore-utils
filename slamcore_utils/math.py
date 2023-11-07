import numpy as np


def is_symmetric(arr: np.ndarray, tol=1e-8) -> bool:
    """
    Check if a numpy array is approximately symmetric.

    A matrix is symmetric if its transpose is approximately equal to itself.

    Examples:

        >>> arr1 = np.array([[1, 2, 3],
        ...                  [2, 4, 5],
        ...                  [3, 5, 6]])
        >>> is_symmetric(arr1)
        True

        >>> arr2 = np.array([[1, 2, 3],
        ...                  [4, 5, 6],
        ...                  [7, 8, 9]])
        >>> is_symmetric(arr2)
        False

        >>> arr3 = np.array([[1.0, 2.0],
        ...                  [2.0, 1.0]])
        >>> is_symmetric(arr3)
        True

        >>> arr4 = np.array([[1.0, 2.0, 3.0],
        ...                  [2.0, 1.0, 4.0]])
        >>> is_symmetric(arr4)
        False

    """
    if arr.ndim != 2:
        raise ValueError(f"Input array must be 2D - Current #dims: {arr.ndim}")

    rows, cols = arr.shape
    if rows != cols:
        return False

    return np.allclose(arr, arr.T, atol=tol)
