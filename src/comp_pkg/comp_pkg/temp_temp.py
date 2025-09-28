import numpy as np
H=np.array([[2,-1,0],[-1,2,-1],[0,-1,1]])
eigvals, eigvecs = np.linalg.eig(H)
eigvals_sorted = np.sort(eigvals)


print("\nEigenvalues (sorted):", eigvals_sorted)
print("Smallest (algebraic connectivity, ignoring exact zero):", eigvals_sorted[1] if len(eigvals_sorted)>1 else eigvals_sorted[0])
print("Largest eigenvalue:", eigvals_sorted[-1])