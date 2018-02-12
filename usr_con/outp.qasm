#Auto generated QASM
	 qubit	 q0,0
	 qubit	 q1,0
	 qubit	 q2,0
	 H	 q0
	 cnot 	 q0,q1
	 measure	 q0
	 zero	 q0
	 H	 q0
	 swap 	 q1,q0
	 cnot 	 q0,q2
	 measure	 q0
