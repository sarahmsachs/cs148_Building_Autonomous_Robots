//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
function robot_pd_control () {
	var curdate = new Date();
	for (x in robot.joints) {
		// robot.joints[x].servo.p_desired = curdate.getSeconds()/6000*2*Math.PI;
		//should change for regrade?
		robot.joints[x].servo.p_desired = curdate.getSeconds()/60*2*Math.PI;
		// robot.joints[x].control += robot.joints[x].servo.p_desired;
		robot.joints[x].control += (robot.joints[x].servo.p_desired - robot.joints[x].angle);
	}
}

