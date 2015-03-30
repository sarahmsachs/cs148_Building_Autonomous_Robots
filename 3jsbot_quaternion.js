//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

quaternion_from_axisangle
quaternion_normalize
quaternion_multiply
quaternion_to_rotation_matrix
*/


function quaternion_from_axisangle(origAxis, angle){
	axis = vector_normalize(origAxis);
	var myQuat =  [Math.cos(angle/2), axis[0]*Math.sin(angle/2), axis[1]*Math.sin(angle/2), axis[2]*Math.sin(angle/2)];
	return myQuat
}

function quaternion_normalize(tempQuat){
	tempVec = vector_normalize(tempQuat);
	return tempVec
}

function quaternion_multiply(q1, q2){
	//I set up local variables to keep consistent with lecture slides
	var a,b,c,d,e,f,g,h;
	a = q1[0];
	b = q1[1];
	c = q1[2];
	d = q1[3];
	e = q2[0];
	f = q2[1];
	g = q2[2];
	h = q2[3];
	var product = [(ae-bf-cg-ah),(af+be+ch-dg),(ag-bh+ce+df),(ah+bg-cf+de)];
	return product
}

function quaternion_to_rotation_matrix(quat){
	var q0, q1, q2, q3, q0s, q1s, q2s, q3s;
	q0 = quat[0];
	q1 = quat[1];
	q2 = quat[2];
	q3 = quat[3];
	q0s = q0*q0;
	q1s = q1*q1;
	q2s = q2*q2;
	q3s = q3*q3;

	var rotMat = 
	[[(1-2*(q2s+q3s)), 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3),0],
	 [(2*(q1*q2+q0*q3)), (1-2*(q1s+q3s)), 2*(q2*q3-q0*q1),0],
	 [2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), 1-2*(q1s+q2s),0],
	 [0,0,0,1]];
	 return rotMat;
}