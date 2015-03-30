//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

matrix_multiply
matrix_transpose
vector_normalize
vector_cross
generate_identity
generate_translation_matrix
generate_rotation_matrix_X
generate_rotation_matrix_Y
generate_rotation_matrix_Z
*/

function matrix_multiply(matrix1, matrix2){
	var finalMatrix = []
    for (var i = 0; i < matrix1.length; i++) {
        finalMatrix[i] = [];
        for (var j=0; j<matrix2[0].length; j++){
        	var dotProduct = 0;
        	for (var k = 0; k<matrix2.length; k++){
        		dotProduct+=matrix1[i][k]*matrix2[k][j];
        	}
        finalMatrix[i].push(dotProduct);
        }
    }
    return finalMatrix;
}


function matrix_transpose(matrix){
	var finalMatrix= [];
	for (var col = 0; col < matrix[0].length; col++) {
		finalMatrix[col]=[]
		for (var row = 0; row < matrix.length; row++) {
			finalMatrix[col][row]=matrix[row][col];
		}
	}
	return finalMatrix
}

function vector_normalize(vector){
	var squaredSum = 0;
	for(var i = 0; i<vector.length; i++){
		squaredSum+=(vector[i]*vector[i]);
	}
	var norm = Math.sqrt(squaredSum);
	finalVector = [];
	for(var i = 0; i<vector.length; i++){
		finalVector.push(vector[i]/norm);
	}
	return finalVector;
}

function vector_cross(u, v){
	var finalVector = [];
	finalVector.push((u[1]*v[2])-(u[2]*v[1]));
	finalVector.push((u[2]*v[0])-(u[0]*v[2]));
	finalVector.push((u[0]*v[1])-(u[1]*v[0]));
	return finalVector;
}

function generate_identity(){
	return [[1,0,0,0],[0,1,0,0],[0,0,1,0], [0,0,0,1]];
}

function generate_translation_matrix(x,y,z){
	return [[1,0,0,x],
			[0,1,0,y],
			[0,0,1,z], 
			[0,0,0,1]];
}
function generate_rotation_matrix_X(theta){
	return [[1,0,0,0],
			[0, Math.cos(theta), -Math.sin(theta), 0],
			[0, Math.sin(theta), Math.cos(theta), 0],
			[0,0,0,1]
			]
}
function generate_rotation_matrix_Y(theta){
		return [[Math.cos(theta),0,Math.sin(theta),0],
			[0, 1, 0, 0],
			[-Math.sin(theta), 0, Math.cos(theta), 0],
			[0,0,0,1]
			]
}
function generate_rotation_matrix_Z(theta){
			return [[Math.cos(theta),-Math.sin(theta),0,0],
				[Math.sin(theta), Math.cos(theta),0,0],
				[0,0,1,0],
				[0,0,0,1]
				]
}
