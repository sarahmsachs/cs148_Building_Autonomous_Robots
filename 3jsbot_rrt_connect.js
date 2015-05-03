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
    // x_min = robot_boundary[0][0];
    // x_max = robot_boundary[1][0];
    // y_min = robot_boundary[0][2];
    // y_max = robot_boundary[1][2];
    x_min = -100;
    x_max = 100;
    y_min = -100;
    y_max = 100;
    t1 = tree_init(q_start_config);
    t2 = tree_init(q_goal_config);
    iterationCount = 0;
    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
    robot_path_traverse_idx = 0;

    // console.log("planner initialized");
}


function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        // CS148: implement RRT iteration here
        qrand = random_config();
        if (rrt_extend(t1, qrand)!='trapped'){
            if (rrt_connect(t2, qnew)=='reached'){
                tree_add_edge(t1.newest, t2.newest, t2)
                // console.log('reached')
                rrt_iterate=false;
                // console.log("about to call find path")
                return find_path(t1, t2);
            }
        }
        //otherwise swithch trees;
        temp = t1;
        t1= t2;
        t2=temp;
        iterationCount++;
        // if (iterationCount>10000){//if over 10k iterations
        //     console.log("took to long")
        //     rrt_iterate = false; //you are done, took too long
        // } 

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
    tree.vertices[0].edges = [];
    tree.vertices[0].vertex = q
    tree_add_vertex(tree.vertices[0], tree) //just added this
    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

   // maintain index of newest vertex added to tree
    tree.newest = tree.vertices[0];
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

function tree_add_vertex (vertex, myTree){
    vertex.visited = false;
    // console.log("about to add vertex")
    // console.log(vertex)
    // console.log("tree.vertices is")
    // console.log(tree.vertices);
    myTree.vertices.push(vertex);
    // console.log("after add vertex")
    // console.log(tree.vertices)
    add_config_origin_indicator_geom(vertex)
    myTree.newest = vertex
        // add_config_origin_indicator_geom(vertex);


    // console.log('successful add vertex')
}
function tree_add_edge(vertexA, vertexB, tree){
    // var newEdge = new Object();

    
        // console.log("adding to root")
    newEdge = {};
    vertexA.edges.push(vertexB);
    vertexB.edges.push(vertexA);
    newEdge.vertexA = vertexA;
    newEdge.vertexB = vertexB;
    // draw_2D_edge_configurations(vertexA.vertex, vertexB.vertex);
    //maybe make some array of things you are connected to, add here
}

function random_config(){
    //we know that a config is eleven long. it's xyzrpy and then the angles for the five joints of the robot
    //and we are given the following instructions
    // the robot base does not move outside the X-Z plane. Specifically, the base should not translate along the Y axis, and should not rotate about the X and Z axes.
    randomArray = [0,0,0,0,0,0,0,0,0,0,0]
    randomArray[0] = randomInRange(x_min,x_max); //x coordinate plane
    randomArray[2] = randomInRange(y_min,y_max); //technically the z coordinate plane. we don't touch y
    randomArray[4] = randomAngle(); //angle;
    for (var joint = 6; joint<11; joint++){ //set all the other joints to have random angles
        var newRandomAngle = randomAngle()
        randomArray[joint] = newRandomAngle;
    }
    //unclear if this should be making a new vertex or have made a new vertex along the way
 
    return randomArray;
}

function new_config(q, qnear, qnew){ //according to paper, this makes a motion toward q with some fixed incremental distance , and tests for collision.

        diff = [0,0,0,0,0,0,0,0,0,0,0]
        for (var i = 0; i<11; i++){
            diff[i] = q[i]-qnear[i];
        }
        directionVector = vector_normalize(diff);
        for (var i = 0; i<11; i++){
            directionVector[i] = directionVector[i]*eps;
        }
        for (var i = 0; i<11; i++){
            qnew[i] = qnear[i]+directionVector[i];
        }
        return !robot_collision_test(qnew);
    

}
function nearest_neighbor(vertexConfig, tree){//simple argmin calculation
   var closestDist = Number.MAX_VALUE;
    var closestV = tree.vertices[0]; //this is arbitrary, if nothing, i guess just set neighbor to be yourself
    // console.log(tree.vertices.length)
    for (var v=0; v<tree.vertices.length; v++){
        var newVertex = tree.vertices[v];
        // console.log(newVertex)
        var newDist = getDistance(newVertex.vertex, vertexConfig);
        if (newDist<closestDist){
            // console.log("returned new vertex")
            closestDist = newDist;
            closestV = newVertex;
        }
    }
    return closestV;
}

function rrt_extend(tree, q){ //straight from psuedocode from paper
    qnearV = nearest_neighbor(q, tree);
    qnew = [0,0,0,0,0,0,0,0,0,0,0]
    if (new_config(q, qnearV.vertex, qnew)){
        // console.log("aboutto add edges")
        qNewVertex={};
        qNewVertex.vertex = qnew;
        // if (getDistance(q, qnew)<eps){
        //     qNewVertex.vertex = q;
        // }
        qNewVertex.edges=[]
        // console.log(qnew)
        tree_add_vertex(qNewVertex, tree);
        tree_add_edge(qnearV, qNewVertex, tree); //changed from qnew and qnear
        if (hasReached(q, qnew)){ //changed to same vertex of q
            return 'reached';
        }   
        else{
            // console.log('advanced')
            return 'advanced';
        }
    }
    return 'trapped';
}


function rrt_connect(tree, q){
    // console.log(q.vertex)
    var state = rrt_extend(tree, q) //start off as a state
    while (state=='advanced'){
        state = rrt_extend(tree, q);
    }
    return state; //basically until trapped for reached
}

function find_path(t1, t2){
    // console.log("finding path")
    // var startV = t1.root;
    // startV.visited = true; //this will be helpful for dfs
    // robot_path = [startV] //starts as just an array with start
    pathSuccessfullyFound = path_dfs(t1.root, t2.root);
    // console.log(pathSuccessfullyFound)
    if (pathSuccessfullyFound==true){
        // console.log("got to inside")
        robot_path=[]
        var tempV = t2.root;
        while (tempV != t1.root){
            robot_path.push(tempV);
            tempV = tempV.prev;
        }
        robot_path.push(t1.root);
        if (!sameVertex(t2.root,q_goal_config)){
            // console.log("swapped");
            robot_path = robot_path.reverse()
        }
        
    // // robot_path = myPath;
    for (var i =0; i<robot_path.length; i++){
        robot_path[i].geom.material.color = {r:1,g:0,b:0};
        // console.log('colored path at index '+i);
    }
    // draw_highlighted_path(robot_path);
}

    // console.log(robot_path)
    return pathSuccessfullyFound;
}

function path_dfs(start, end){
    // console.log("dfs called")
    if (start == end){
        // console.log("same vertex")
        return true;
    }
    else{
        // console.log(start.vertex)
        // console.log(end.vertex)
        start.visited = true;
        for (var i = 0; i<start.edges.length; i++){
            var next = start.edges[i];
            // var next;
            // // if (edge.vertexA.visited== true)
            // //     next  = edge.vertexB;
            // // else{
            // //     next = edge.vertexA;
            // // }
            // if (sameVertex(edge.vertexA.vertex,start.vertex)){
            //     next = edge.vertexB;
            // }
            // else {
            //     next = edge.vertexA;
            // }
            if (next.visited ==false){
                next.prev = start;
                if (path_dfs(next, end)){
                    return true;
                }
            }
        }    
        return false;
    }
}



function randomAngle(){
    var newAngle = randomInRange(0, 2*Math.PI);
    return newAngle
}
function randomInRange(min, max) { 
    var randomNumUnshifted = (max - min)*Math.random();
    return (min + randomNumUnshifted);
}

function hasReached(vertexA, vertexB){
    for (var i=0; i<11; i++){
        if (Math.abs(vertexA[i]-vertexB[i])>eps){
            return false
        }
    }
    return true;
}
function getDistance(vertexA, vertexB){
    var vAx, vAy,vBx, vBy;
    vAx= vertexA[0]
    vAy= vertexA[2]
    vBx=vertexB[0]
    vBy= vertexB[2]
    return Math.sqrt(Math.pow((vBx-vAx),2)+Math.pow((vBy-vAy),2)); //Euclidean distance formula
}
function sameVertex(v1, v2){ //simple check that coordinates of both vertices are the same
    var vertex1 = v1;
    var vertex2 = v2;
    // var vertex1 = v1.vertex;
    // var vertex2 = v2.vertex;
    for (var i = 0; i<11; i++){
        if (vertex1[i]!=vertex2[i]){
            return false;
        }
    }
    return true;
}

