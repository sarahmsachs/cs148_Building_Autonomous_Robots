//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// CS148: 
// implement RRT-Connect by Kuffner and LaValle (2000)
//    paper link: http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf

// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by robot_collision_test()

/*
CS148: reference code has functions for:

    tree_add_vertex
    tree_add_edge
    random_config
    new_config
    nearest_neighbor
    rrt_extend
    rrt_connect
    find_path
    path_dfs
*/


function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // CS148: add necessary RRT initialization here
    rrt_iterate = true;
    eps = 1; //initialize as epsilon = 1, can play around with later
    x_min = robot_boundary[0][0];
    x_max = robot_boundary[1][0];
    y_min = robot_boundary[0][2];
    y_max = robot_boundary[1][2];
    t1 = tree_init(q_start_config);
    t2 = tree_init(q_goal_config);
    iterationCount = 0;
    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    console.log("planner initialized");
}


function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        // CS148: implement RRT iteration here
        qrand = random_config();
        if (rrt_extend(t1, qrand)!='trapped'){
            if (rrt_connect(t2, qnew)=='reached'){
                return find_path(t1, t2);
            }
        }
        //otherwise swithch trees;
        temp = t1;
        t1= t2;
        t2=temp;
        iterationCount++;
        if (iterationCount>100){//if over 10k iterations
            rrt_iterate = false; //you are done, took too long
        } 
    }

    // return path not currently found
    return false;
}

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    console.log("about to add config origin")
    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;
    tree.root = tree.vertices[0]; //this is the center of the tree

    return tree;
}


function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);

    vertex.geom = temp_mesh;
}

function tree_add_vertex (vertex, tree){
    // a={};
    // a.vertex = vertex;
    // console.log(a.vertex)
    // a.edges=[]
    vertex.visited = false;
    tree.vertices.push(vertex);
    add_config_origin_indicator_geom(vertex);
    tree.newest++;
        // add_config_origin_indicator_geom(vertex);


    // console.log('successful add vertex')
}
function tree_add_edge(vertexA, vertexB, tree){
    // var newEdge = new Object();
    var newEdge = {};
    vertexA.edges.push(newEdge);
    vertexB.edges.push(newEdge);
    newEdge.vertexA = vertexA;
    newEdge.vertexB = vertexB;
    //maybe make some array of things you are connected to, add here
}

function random_config(){
    //we know that a config is eleven long. it's xyzrpy and then the angles for the five joints of the robot
    //and we are given the following instructions
    // the robot base does not move outside the X-Z plane. Specifically, the base should not translate along the Y axis, and should not rotate about the X and Z axes.
    
    randomArray = Array.apply(null, new Array(11)).map(Number.prototype.valueOf,0); //make an array of 11 zeros
    randomArray[0] = randomInRange(x_min,x_max); //x coordinate plane
    randomArray[2] = randomInRange(y_min,y_max); //technically the z coordinate plane. we don't touch y
    randomArray[4] = randomAngle(); //angle;
    for (var joint = 6; joint<11; joint++){ //set all the other joints to have random angles
        var newRandomAngle = randomAngle()
        randomArray[joint] = newRandomAngle;
    }
    //unclear if this should be making a new vertex or have made a new vertex along the way
    // randomArray.vertex = randomArray;
    return randomArray;
}

function new_config(q, qnear, qnew){ //according to paper, this makes a motion toward q with some fixed incremental distance , and tests for collision.
    if (getDistance(q, qnear)<=eps){ //if less than one step away
        // console.log("Arrived, less than one step away");
        return !robot_collision_test(q);
    }
    diff = Array.apply(null, new Array(11)).map(Number.prototype.valueOf,0);
    else { //move towards nearest neighbor
        for (var i = 0; i<11; i++){
            diff[i] = q[i]-qnear[i];
        }
        directionVector = vector_normalize(diff);
        for (var i = 0; i<11; i++){
            directionVector[i] = directionVector[i]*eps;
        }
        qnew = qnear+directionVector
        console.log("The q is: "+q+", the qnear is: "+qnear+", qnew is: "+qnew)
        return !robot_collision_test(qnew);
    }

}
function nearest_neighbor(vertex, tree){//simple argmin calculation
    var closestDist = Number.MAX_VALUE
    var closestV = vertex //this is arbitrary, if nothing, i guess just set neighbor to be yourself
    for (var v=0; v<tree.vertices.length; v++){
        var newVertex = tree.vertices[v].vertex;
        var newDist = getDistance(newVertex, vertex);
        if (newDist<closestDist){
            closestDist = newDist;
            closestV = newVertex;
        }
    }
    return closestV;
}

function rrt_extend(tree, q){ //straight from psuedocode from paper
    qnear = nearest_neighbor(q, tree);
    qnew = Array.apply(null, new Array(11)).map(Number.prototype.valueOf,0);
    if (new_config(q, qnear, qnew)){
        qNewVertex={};
        qNewVertex.vertex = qnew;
        qNewVertex.edges=[]
        // tree_add_vertex(qnew, tree);
        tree_add_vertex(qNewVertex, tree);
        // vA = {};
        // vA.vertex = qnear
        // vA.
        qNearVertex={};
        qNearVertex.vertex = qnew;
        qNearVertex.edges=[]
        tree_add_edge(qNearVertex, qNewVertex, tree); //changed from qnew and qnear
        if (sameVertex(qNearVertex, qNewVertex)){
            return 'reached';
        }   
        else{
            return 'advanced';
        }
    }
    return 'trapped';
}

function rrt_connect(tree, q){
    var state = rrt_extend(tree, q) //start off as a state
    while (state=='advanced'){
        state = rrt_extend(tree, q);
    }
    return state; //basically until trapped for reached
}

function find_path(t1, t2){
    var startV = t1.root;
    startV.visited = true; //this will be helpful for dfs
    var robot_path = [startV] //starts as just an array with start
    pathSuccessfullyFound = path_dfs(startV, t2.root);
    // robot_path = myPath;
    for (var i =0; i<robot_path.length; i++){
        robot_path[i].geom.material.color = {r:1,g:0,b:0};
        console.log('colored path at index '+i);
    }
    console.log(robot_path)
    return pathSuccessfullyFound;
}
function path_dfs(start, end){
    for (var i = 0; i<start.edges.length; i++){
        var edge = start.edges[i];
        if (edge.vertexA==start){
            next = edge.vertexB;
        }
        else{
            next = edge.vertexA;
        }

        if (next.visited==true){
            continue; // we don't care about visited ones in dfs
        }
        robot_path.push(next)
        next.visited = true;
        if (sameVertex(next, end)){
            return true;
        }
        if (path_dfs(next, end)){
            console.log('called more dfs')
            return true;
        }
        else {
            robot_path.pop() //get rid of it if it doesn't have a dfs, its not part of our path
        }
    }
    return false; //if dfs doesn't work
}



function randomAngle(){
    var newAngle = randomInRange(0, 2*Math.PI);
    return newAngle
}
function randomInRange(min, max) { 
    var randomNumUnshifted = (max - min)*Math.random();
    return (min + randomNumUnshifted);
}

function getDistance(vertexA, vertexB){
    var vAx, vAy,vBx, vBy;
    vAx= vertexA[0]
    vAy= vertexA[1]
    vBx=vertexB[0]
    vBy= vertexB[1]
    return Math.sqrt(Math.pow((vBx-vAx),2)+Math.pow((vBy-vAy),2)); //Euclidean distance formula
}

function sameVertex(v1, v2){ //simple check that coordinates of both vertices are the same
    var vertex1 = v1.vertex;
    var vertex2 = v2.vertex;
    for (var i = 0; i<11; i++){
        if (vertex1[i]!=vertex2[i]){
            return false;
        }
    }
    return true;
}

