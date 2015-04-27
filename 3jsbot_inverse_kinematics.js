//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/

function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
    // compute joint angle controls to move location on specified link to Cartesian location
    if (update_ik) {
        iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos);
        endeffector_geom.visible = true;
        target_geom.visible = true;
    }
    else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;

}


function  iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos){
	var targetX = target_pos[0][0];
	var targetY = target_pos[1][0];
	var targetZ = target_pos[2][0];
	var targetMatrix = generate_translation_matrix(targetX, targetY, targetZ);
	simpleApplyMatrix(target_geom, matrix_2Darray_to_threejs(targetMatrix));
	var efX = endeffector_local_pos[0][0];
	var efY = endeffector_local_pos[1][0];
	var efZ = endeffector_local_pos[2][0];
	var efLocalPosMatrix = generate_translation_matrix(efX, efY, efZ);
	var global_ef_pos = matrix_multiply(robot.joints[endeffector_joint].xform, efLocalPosMatrix);
	simpleApplyMatrix(endeffector_geom, matrix_2Darray_to_threejs(global_ef_pos));

	var jointNumber = 0;
	var jacobianFlip = new Array(4);
	var currentJoint = robot.joints[endeffector_joint];
	while (currentJoint){
		var subI = jacobianSubI(global_ef_pos, currentJoint);
		jacobianFlip[jointNumber] = subI
		jointNumber++;
		currentJoint = robot.joints[robot.links[currentJoint.parent].parent];
	}
	jacobian = matrix_transpose(jacobianFlip);
	var jacobianInverse = psuedoInverse(jacobian);
	var dx = caluclateDx(target_pos, global_ef_pos);
	var dq = matrix_multiply(jacobianInverse, dx);
	applyControl(dq, robot.joints[endeffector_joint]);

}

function jointZi(joint){ //note, when call on zi, you have to do actual joint, not string
	var xform = joint.xform
	var rotationOnlyXform = new Array(4);
	for (var i=0; i<xform.length; i++){
		rotationOnlyXform[i] = [];
		for (var j = 0; j<xform[0].length; j++){
			rotationOnlyXform[i][j]= joint.xform[i][j];
		}
	}
	rotationOnlyXform[0][3]=0;//this gets rid of the xyz from xform
	rotationOnlyXform[1][3]=0;
	rotationOnlyXform[2][3]=0;
	var axisX = joint.axis[0];
	var axisY = joint.axis[1];
	var axisZ = joint.axis[2]
	var zi = matrix_multiply(rotationOnlyXform, [[axisX],[axisY],[axisZ],[1]]);
	return [[zi[0][0]], [zi[1][0]], [zi[2][0]]]; //because you don't want last element, which is zero

}

function jacobianSubI(global_ef_pos, joint){

	var efX = global_ef_pos[0][3];
	var efY = global_ef_pos[1][3];
	var efZ = global_ef_pos[2][3];
	var jointX = joint.xform[0][3];
	var jointY = joint.xform[1][3];
	var jointZ = joint.xform[2][3];

	var difVector = [[efX - jointX], [efY-jointY], [efZ-jointZ]];
	var zi = jointZi(joint);
	
	var cross = vector_cross(zi, difVector);
	return [cross[0], cross[1], cross[2], zi[0][0], zi[1][0], zi[2][0]];

}

function psuedoInverse(jacobian){
	transposeJacobian = false; //this is to be set according to how you want to take your inverse

	if (transposeJacobian==false){
		var m = jacobian.length;
		var n = jacobian[0].length;
		var a = jacobian;
		var at = matrix_transpose(jacobian);
		if (m>n){
			var ata = matrix_multiply(at, a);
			var atai = numeric.inv(ata);
			return matrix_multiply(atai, at);
		}
		else if(n>m){
			var aat = matrix_multiply(a,at);
			var aati = numeric.inv(aat);
			return matrix_multiply(at, aati);
		}
		else{
			return numeric.inv(jacobian);
		}
	}
	else if (transposeJacobian==true) {
		return matrix_transpose(jacobian)
	}
	

}

function caluclateDx(target_pos, ef_global_pos){
	return [[target_pos[0]-ef_global_pos[0][3]],[target_pos[1]-ef_global_pos[1][3]],[target_pos[2]-ef_global_pos[2][3]],[0],[0],[0]];
}

function applyControl(dq, currentJoint){
	alpha = .09;
	jointNumber = 0;
	while (currentJoint){
		currentJoint.control = alpha*dq[jointNumber][0];
		jointNumber++;
		currentJoint = robot.joints[robot.links[currentJoint.parent].parent];
	}
}