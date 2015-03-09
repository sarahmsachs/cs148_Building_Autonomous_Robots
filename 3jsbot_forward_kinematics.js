//////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:

robot_forward_kinematics
traverse_forward_kinematics_link
traverse_forward_kinematics_joint
compute_and_draw_heading
*/



function robot_forward_kinematics(){
	order = Array();
	xforms = Array();
	traverse_forward_kinematics_link(robot, robot.links[robot.base]);
}
function traverse_forward_kinematics_link(robot, link){
	order.push(link["name"]); //This is in case I want to print an array of my traversal in order. used for debugging purposes
	if (link.parent){ //If it isn't the base, I take the parent joints xform
		link.xform = robot.joints[link.parent].xform;
	}
	else {
		link.xform = getOriginXform();
	}
	for (x in link.children){ //I call on the links children joints to traverse
		traverse_forward_kinematics_joint(robot, robot.joints[link.children[x]]);
	}
	compute_and_draw_heading(link);
}
function traverse_forward_kinematics_joint(robot, joint){
	order.push(joint); //for debugging puroses
	joint.origin.xform = getXform(joint);
	joint.xform = matrix_multiply(joint.origin.xform, generate_identity());
	compute_and_draw_heading(joint);
	traverse_forward_kinematics_link(robot, robot.links[joint.child]);//I traverse on child

}
function compute_and_draw_heading(object){
	var tempmat = matrix_2Darray_to_threejs(object.xform);
    simpleApplyMatrix(object.geom,tempmat);
}

function getXform(obj){
	xforms.push(obj) 
	var tx, ty, tz, rr, rp, ry;
	var parent = robot.links[obj.parent];
	tx = obj.origin.xyz[0];
	ty = obj.origin.xyz[1];
	tz = obj.origin.xyz[2];
	rr = obj.origin.rpy[0];
	rp = obj.origin.rpy[1];
	ry = obj.origin.rpy[2];
	var tempMatrix1 = matrix_multiply(parent.xform, generate_translation_matrix(tx, ty, tz));
	var tempMatrix2 = matrix_multiply(tempMatrix1, generate_rotation_matrix_X(rr)); 
	var tempMatrix3 = matrix_multiply(tempMatrix2, generate_rotation_matrix_Y(rp));
	var tempMatrix4 = matrix_multiply(tempMatrix3, generate_rotation_matrix_Z(ry));
	xforms.push(tempMatrix4); //This is for debugging so that I can view my xforms in order
	return tempMatrix4;
}
function getOriginXform(obj){
	var origin = robot.origin
	var tx, ty, tz, rr, rp, ry
	tx = origin.xyz[0];
	ty = origin.xyz[1];
	tz = origin.xyz[2];
	rr = origin.rpy[0];
	rp = origin.rpy[1];
	ry = origin.rpy[2];
	var tempMatrix = generate_identity();
	tempMatrix = matrix_multiply(tempMatrix, generate_translation_matrix(tx, ty, tz));
	tempMatrix = matrix_multiply(tempMatrix, generate_rotation_matrix_X(rr));
	tempMatrix = matrix_multiply(tempMatrix, generate_rotation_matrix_Y(rp));
	tempMatrix = matrix_multiply(tempMatrix, generate_rotation_matrix_Z(ry));
	xforms.push(tempMatrix);
	return tempMatrix
}
