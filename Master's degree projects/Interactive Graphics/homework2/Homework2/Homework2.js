"use strict";

var canvas;
var gl;
var program;

// viewing parameters ---------------------------------------------------------
var near = -50;
var far = 50;
var radius = 1.0;
var theta_view = 0.0;
var phi = 0.0;
var dr = 5.0 * Math.PI/180.0;

var left = -25.0;
var right = 25.0;
var top_view = 25.0;
var bottom = -25.0;

var eye;

const at = vec3(0.0, 0.0, 0.0);
const up = vec3(0.0, 1.0, 0.0);

// light and materials properties----------------------------------------------
var lightPosition1 = vec4(-30.0, 30.0, 70.0, 1.0);
var lightPosition2 = vec4(60.0, 70.0, -40.0, 1.0);
var lightAmbient = vec4(1.0, 1.0, 1.0, 1.0);
var lightDiffuse = vec4(1.0, 1.0, 1.0, 1.0);
var lightSpecular = vec4(1.0, 1.0, 1.0, 1.0);

var materialAmbientSheep = vec4(0.1, 0.1, 0.1, 1.0);
var materialDiffuseSheep = vec4(0.9, 0.9, 0.9, 1.0);
var materialSpecularSheep = vec4(0.2, 0.2, 0.2, 1.0);
var materialShininessSheep = 10.0;

var materialAmbientWood = vec4(0.5, 0.5, 0.5, 1.0);
var materialDiffuseWood = vec4(1.0, 1.0, 1.0, 1.0);
var materialSpecularWood = vec4(0.2, 0.2, 0.2, 1.0);
var materialShininessWood = 10.0;

var materialAmbientGrass = vec4(0.4, 0.4, 0.4, 1.0);
var materialDiffuseGrass = vec4(0.6, 0.6, 0.6, 1.0);
var materialSpecularGrass = vec4(0.1, 0.1, 0.1, 1.0);
var materialShininessGrass = 10.0;

//----------------------------------------------------------------------------

var projectionMatrix, modelViewMatrix, nMatrix;
var instanceMatrix;

var modelViewMatrixLoc, projectionMatrixLoc, nMatrixLoc;
var objectId, objectIdLoc;

var texture2, texture3, texture4;

var vertices = [

    vec4(-0.5, -0.5,  0.5, 1.0),
    vec4(-0.5,  0.5,  0.5, 1.0),
    vec4(0.5,  0.5,  0.5, 1.0),
    vec4(0.5, -0.5,  0.5, 1.0),
    vec4(-0.5, -0.5, -0.5, 1.0),
    vec4(-0.5,  0.5, -0.5, 1.0),
    vec4(0.5,  0.5, -0.5, 1.0),
    vec4(0.5, -0.5, -0.5, 1.0)
];

var texCoord = [
    vec2(0, 0),
    vec2(0, 1),
    vec2(1, 1),
    vec2(1, 0)
];

var grassId = 0;
var fence1Id = 1;
var fence2Id = 2;
var fence3Id = 3;
var torsoId = 4;
var headId  = 5;
var head1Id = 5;
var head2Id = 15;
var tailId = 6;
var leftBackLegId = 7;
var leftLowerBackLegId = 8;
var rightBackLegId = 9;
var rightLowerBackLegId = 10;
var leftFrontLegId = 11;
var leftLowerFrontLegId = 12;
var rightFrontLegId = 13;
var rightLowerFrontLegId = 14;

var torsoHeight = 3.5;
var torsoWidth = 7.0;
var upperLegWidth  = 1.3;
var lowerLegWidth  = 1.0;
var lowerLegHeight = 1.0;
var upperLegHeight = 2.0;
var headHeight = 1.7;
var headWidth = 2.3;
var tailHeight = 1.0;
var tailWidth = 1.0;

var grassHeight = 25.0;
var grassWidth = 50.0;

var fenceHeight = 6.0;
var fenceWidth = 1.2;
var fenceHorizHeight = 4.0;
var fenceHorizWidth = 1.2;

var numNodes = 15;
var numAngles = 16;
var angle = 0;

var theta = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10];
var sheepPosition = [-20.0, grassHeight+lowerLegHeight+upperLegHeight-0.3, 0.0];

var numVertices = 48;

var stack = [];

var figure = [];

for( var i=0; i<numNodes; i++) figure[i] = createNode(null, null, null, null);

var vBuffer, fBuffer;
var modelViewLoc;

var pointsArray = [];
var texCoordsArray = [];
var normalsArray = [];
var tangentsArray = [];

//-----------------------------FACE TEXTURE--------------------------------//
var texSize = 64;

// texture for eyes
var image1 = new Array()
    for (var i =0; i<texSize; i++)  image1[i] = new Array();
    for (var i =0; i<texSize; i++)
        for ( var j = 0; j < texSize; j++)
           image1[i][j] = new Float32Array(4);
    for (var i =0; i<texSize; i++) for (var j=0; j<texSize; j++) 
        image1[i][j] = [0.9, 0.9, 0.9, 1];
    for (var i = 3*texSize/4-5; i<3*texSize/4+5; i++) for (var j=1*texSize/4-5; j<1*texSize/4+5; j++)
        image1[i][j] = [0, 0, 0, 1];

var face_texture = new Uint8Array(4*texSize*texSize);

    for (var i = 0; i < texSize; i++)
        for (var j = 0; j < texSize; j++)
            for(var k =0; k<4; k++)
                face_texture[4*texSize*i+4*j+k] = 255*image1[i][j][k];

// texture for mouth
var image2 = new Array()
    for (var i =0; i<texSize; i++)  image2[i] = new Array();
    for (var i =0; i<texSize; i++)
        for ( var j = 0; j < texSize; j++)
           image2[i][j] = new Float32Array(4);
    for (var i =0; i<texSize; i++) for (var j=0; j<texSize; j++) 
        image2[i][j] = [0.9, 0.9, 0.9, 1];
    for (var i = 1*texSize/4-5; i<1*texSize/4+5; i++) for (var j=1*texSize/2-8; j<1*texSize/2+8; j++)
        image2[i][j] = [0.8, 0.7, 0.7, 1];
    /* To make the eyes and the mouth be on the same face of the cube, uncomment this
    for (var i = 3*texSize/4-5; i<3*texSize/4+5; i++) for (var j=1*texSize/4-5; j<1*texSize/4+5; j++)
        image2[i][j] = [0, 0, 0, 1];
    for (var i = 3*texSize/4-5; i<3*texSize/4+5; i++) for (var j=1*texSize/4-5; j<1*texSize/4+5; j++)
        image2[i][j] = [0, 0, 0, 1];
    for (var i = 3*texSize/4-5; i<3*texSize/4+5; i++) for (var j=3*texSize/4-5; j<3*texSize/4+5; j++)
        image2[i][j] = [0, 0, 0, 1]; */

var face_texture1 = new Uint8Array(4*texSize*texSize);

    for (var i = 0; i < texSize; i++)
        for (var j = 0; j < texSize; j++)
            for(var k =0; k<4; k++)
                face_texture1[4*texSize*i+4*j+k] = 255*image2[i][j][k];

//------------------------------ SHEEP TEXTURE ----------------------------//

// Bump Data

var data = new Array()
    for (var i = 0; i<= texSize; i++)  data[i] = new Array();
    for (var i = 0; i<= texSize; i++) for (var j=0; j<=texSize; j++) {
        var n = Math.floor(Math.random()*2);
        if (n % 2 == 0)
            data[i][j] = 0.7;   // we use 0.7 in place of 0.0 to not make the sheep's body black and white
        else
            data[i][j] = 1.0;
    }

// Bump Map Normals

var normalst = new Array()
    for (var i=0; i<texSize; i++)  normalst[i] = new Array();
    for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++)
        normalst[i][j] = new Array();
    for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++) {
        normalst[i][j][0] = data[i][j]-data[i+1][j];
        normalst[i][j][1] = data[i][j]-data[i][j+1];
        normalst[i][j][2] = 1;
    }

// Scale to Texture Coordinates

    for (var i=0; i<texSize; i++) for (var j=0; j<texSize; j++) {
       var d = 0;
       for(k=0;k<3;k++) d+=normalst[i][j][k]*normalst[i][j][k];
       d = Math.sqrt(d);
       for(k=0;k<3;k++) normalst[i][j][k]= 0.5*normalst[i][j][k]/d + 0.5;
    }

// Normal Texture Array

var bump_texture = new Uint8Array(3*texSize*texSize);

    for ( var i = 0; i < texSize; i++ )
        for ( var j = 0; j < texSize; j++ )
            for(var k =0; k<3; k++)
                bump_texture[3*texSize*i+3*j+k] = 255*normalst[i][j][k];

//-------------------------------------------------------------------------//

//--------------------------- CONFIGURE TEXTURES --------------------------//

function configureTexture(face_texture, grass_texture, wood_texture, bump_texture) {
    // Color texture for the sheep's head
    var texture1 = gl.createTexture();
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, texture1);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, texSize, texSize, 0,
        gl.RGBA, gl.UNSIGNED_BYTE, face_texture);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
        gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);

    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapFace"), 0);

    // Color (image) texture for the grass field
    texture2 = gl.createTexture();
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, texture2);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, grass_texture);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
                      gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapGrass"), 1);

    // color texture to give the appereance of wood to the fence
    texture3 = gl.createTexture();
    gl.activeTexture(gl.TEXTURE2);
    gl.bindTexture(gl.TEXTURE_2D, texture3);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, wood_texture);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
                      gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapWood"), 2);

    // Bump textures for the sheep's body, tail and upper legs
    var texture4 = gl.createTexture();
    gl.activeTexture(gl.TEXTURE3);
    gl.bindTexture(gl.TEXTURE_2D, texture4);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, texSize, texSize, 0, gl.RGB, gl.UNSIGNED_BYTE, bump_texture);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapBump"), 3);

    var texture5 = gl.createTexture();
    gl.activeTexture(gl.TEXTURE4);
    gl.bindTexture(gl.TEXTURE_2D, texture5);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, 16, 32, 0, gl.RGB, gl.UNSIGNED_BYTE, bump_texture);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapBump1"), 4);

    var texture6 = gl.createTexture();
    gl.activeTexture(gl.TEXTURE5);
    gl.bindTexture(gl.TEXTURE_2D, texture6);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, texSize, texSize, 0,
        gl.RGBA, gl.UNSIGNED_BYTE, face_texture1);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
        gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);

    gl.uniform1i(gl.getUniformLocation(program, "uTextureMapFace1"), 5);
}

//---------------------------------------------------------------------------//
    
function scale4(a, b, c) {
   var result = mat4();
   result[0] = a;
   result[5] = b;
   result[10] = c;
   return result;
}

//-------------------------------------------------------------------------//


function createNode(transform, render, sibling, child){
    var node = {
    transform: transform,
    render: render,
    sibling: sibling,
    child: child,
    }
    return node;
}


function initNodes(Id) {

    var m = mat4();

    switch(Id) {

    case grassId:

    m = translate(0.0, -25.0, 0.0);
    m = mult(m, rotate(theta[grassId], vec3(0, 1, 0)));
    figure[grassId] = createNode(m, grass, null, fence1Id);
    break;

    case fence1Id:

    m = translate(3.0, grassHeight+1.0, 0.0);
    m = mult(m, rotate(theta[fence1Id], vec3(0, 1, 0)));
    figure[fence1Id] = createNode(m, fenceHoriz, fence2Id, null);
    break;

    case fence2Id:

    m = translate(3.0, grassHeight, 5.0);
    m = mult(m, rotate(theta[fence2Id], vec3(0, 1, 0)));
    figure[fence2Id] = createNode(m, fence, fence3Id, null);
    break;

    case fence3Id:

    m = translate(3.0, grassHeight, -5.0);
    m = mult(m, rotate(theta[fence3Id], vec3(0, 1, 0)));
    figure[fence3Id] = createNode(m, fence, torsoId, null);
    break;

    case torsoId:

    m = translate(sheepPosition[0], sheepPosition[1], sheepPosition[2]);
    m = mult(m, rotate(theta[torsoId], vec3(0, 0, 1)));
    figure[torsoId] = createNode(m, torso, null, headId);
    break;

    case headId:
    case head1Id:
    case head2Id:


    m = translate(0.5*torsoWidth+0.3*headWidth, 0.75*torsoHeight, 0.0);
	  m = mult(m, rotate(theta[head1Id], vec3(1, 0, 0)));
	  m = mult(m, rotate(theta[head2Id], vec3(0, 0, 1)));
    figure[headId] = createNode(m, head, tailId, null);
    break;

    case tailId:

    m = translate(-(0.5*torsoWidth+0.5*tailWidth), 0.45*torsoHeight, 0.0);
	  m = mult(m, rotate(theta[tailId], vec3(0, 1, 0)));
    figure[tailId] = createNode(m, tail, leftBackLegId, null);
    break;

    case leftBackLegId:

    m = translate(-(0.45*torsoWidth-0.8*upperLegWidth), -upperLegHeight+0.3, -0.3*torsoWidth);
	  m = mult(m, rotate(theta[leftBackLegId], vec3(0, 0, 1)));
    figure[leftBackLegId] = createNode(m, leftUpperBackLeg, rightBackLegId, leftLowerBackLegId);
    break;

    case rightBackLegId:

    m = translate(-(0.45*torsoWidth-0.8*upperLegWidth), -upperLegHeight+0.3, 0.3*torsoWidth);
	  m = mult(m, rotate(theta[rightBackLegId], vec3(0, 0, 1)));
    figure[rightBackLegId] = createNode(m, rightUpperBackLeg, leftFrontLegId, rightLowerBackLegId);
    break;

    case leftFrontLegId:

    m = translate(0.45*torsoWidth-0.8*upperLegWidth, -upperLegHeight+0.3, -0.3*torsoWidth);
	  m = mult(m , rotate(theta[leftFrontLegId], vec3(0, 0, 1)));
    figure[leftFrontLegId] = createNode(m, leftUpperFrontLeg, rightFrontLegId, leftLowerFrontLegId);
    break;

    case rightFrontLegId:

    m = translate(0.45*torsoWidth-0.8*upperLegWidth, -upperLegHeight+0.3, 0.3*torsoWidth);
	  m = mult(m, rotate(theta[rightFrontLegId], vec3(0, 0, 1)));
    figure[rightFrontLegId] = createNode(m, rightUpperFrontLeg, null, rightLowerFrontLegId);
    break;

    case leftLowerBackLegId:

    m = translate(0.0, -lowerLegHeight, 0.0);
    m = mult(m, rotate(theta[leftLowerBackLegId], vec3(0, 0, 1)));
    figure[leftLowerBackLegId] = createNode(m, leftLowerBackLeg, null, null);
    break;

    case rightLowerBackLegId:

    m = translate(0.0, -lowerLegHeight, 0.0);
    m = mult(m, rotate(theta[rightLowerBackLegId], vec3(0, 0, 1)));
    figure[rightLowerBackLegId] = createNode(m, rightLowerBackLeg, null, null);
    break;

    case leftLowerFrontLegId:

    m = translate(0.0, -lowerLegHeight, 0.0);
    m = mult(m, rotate(theta[leftLowerFrontLegId],vec3(0, 0, 1)));
    figure[leftLowerFrontLegId] = createNode(m, leftLowerFrontLeg, null, null);
    break;

    case rightLowerFrontLegId:

    m = translate(0.0, -lowerLegHeight, 0.0);
    m = mult(m, rotate(theta[rightLowerFrontLegId], vec3(0, 0, 1)));
    figure[rightLowerFrontLegId] = createNode(m, rightLowerFrontLeg, null, null);
    break;

    }
}


function traverse(Id) {

   if(Id == null) return;
   stack.push(modelViewMatrix);
   modelViewMatrix = mult(modelViewMatrix, figure[Id].transform);
   figure[Id].render();
   if(figure[Id].child != null) traverse(figure[Id].child);
    modelViewMatrix = stack.pop();
   if(figure[Id].sibling != null) traverse(figure[Id].sibling);
}

function torso() {
    objectId = 4;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5*torsoHeight, 0.0));
    instanceMatrix = mult(instanceMatrix, scale(torsoWidth, torsoHeight, torsoWidth));
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));

    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function head() {
    objectId = 3;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * headHeight, 0.0 ));
	instanceMatrix = mult(instanceMatrix, scale(headWidth, headHeight, headWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));

    for(var i = 0; i < 6; i++) {
        if (i == 0 || i == 4) 
            gl.uniform1i(gl.getUniformLocation(program, "uFaceTexture"), 1);
        else if (i == 1)
            gl.uniform1i(gl.getUniformLocation(program, "uFaceTexture"), 2);
        else
            gl.uniform1i(gl.getUniformLocation(program, "uFaceTexture"), 0);
        
        if (i == 4)                                 // draw flipped back faces to have the eye in the correct position
            gl.drawArrays(gl.TRIANGLE_FAN, 4*6, 4);
        else {
            if (i == 3 || i == 5) {                 // top and left faces are bump
                gl.uniform1i(objectIdLoc, 4);
                gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
                gl.uniform1i(objectIdLoc, objectId);
            } else {
                gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
            }
        }

        /* To make the eyes and the mouth be on the same face of the cube, use the following code
        if (i == 1)
            gl.uniform1i(gl.getUniformLocation(program, "uFaceTexture"), 2);
        else
            gl.uniform1i(gl.getUniformLocation(program, "uFaceTexture"), 0);
        
        if (i == 4)                                 // draw flipped back faces to have the eye in the correct position
            gl.drawArrays(gl.TRIANGLE_FAN, 4*6, 4);
        else {
            if (i == 3 || i == 5 || i == 0 || i == 4) {                 // top and left faces are bump
                gl.uniform1i(objectIdLoc, 4);
                gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
                gl.uniform1i(objectIdLoc, objectId);
            } else {
                gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
            }
        }*/
    }
}

function tail() {
    objectId = 5;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * tailHeight, 0.0 ));
	instanceMatrix = mult(instanceMatrix, scale(tailWidth, tailHeight, tailWidth));
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function leftUpperBackLeg() {
    objectId = 5;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperLegHeight, 0.0));
	instanceMatrix = mult(instanceMatrix, scale(upperLegWidth, upperLegHeight, upperLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function leftLowerBackLeg() {
    objectId = 0;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * lowerLegHeight, 0.0));
	instanceMatrix = mult(instanceMatrix, scale(lowerLegWidth, lowerLegHeight, lowerLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightUpperBackLeg() {
    objectId = 5;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperLegHeight, 0.0));
	instanceMatrix = mult(instanceMatrix, scale(upperLegWidth, upperLegHeight, upperLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightLowerBackLeg() {
    objectId = 0;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * lowerLegHeight, 0.0));
	instanceMatrix = mult(instanceMatrix, scale(lowerLegWidth, lowerLegHeight, lowerLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function  leftUpperFrontLeg() {
    objectId = 5;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperLegHeight, 0.0));
	instanceMatrix = mult(instanceMatrix, scale(upperLegWidth, upperLegHeight, upperLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function leftLowerFrontLeg() {
    objectId = 0;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate( 0.0, 0.5 * lowerLegHeight, 0.0));
	instanceMatrix = mult(instanceMatrix, scale(lowerLegWidth, lowerLegHeight, lowerLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightUpperFrontLeg() {
    objectId = 5;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperLegHeight, 0.0));
	instanceMatrix = mult(instanceMatrix, scale(upperLegWidth, upperLegHeight, upperLegWidth) );
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function rightLowerFrontLeg() {
    objectId = 0;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * lowerLegHeight, 0.0));
    instanceMatrix = mult(instanceMatrix, scale(lowerLegWidth, lowerLegHeight, lowerLegWidth) )
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function grass() {
    objectId = 1;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5*grassHeight, 0.0));
    instanceMatrix = mult(instanceMatrix, scale(grassWidth, grassHeight, grassWidth));
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));

    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function fence() {
    objectId = 2;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5*fenceHeight, 0.0));
    instanceMatrix = mult(instanceMatrix, scale(fenceWidth, fenceHeight, fenceWidth));
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function fenceHoriz() {
    objectId = 2;
    gl.uniform1i(objectIdLoc, objectId);

    instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5*fenceHorizHeight, 0.0));
    instanceMatrix = mult(instanceMatrix, scale(fenceHorizWidth, fenceHorizHeight, 10.0));
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix));
    for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
}

function quad(a, b, c, d) {
     var t1 = subtract(vertices[b], vertices[a]);
     var t2 = subtract(vertices[c], vertices[b]);
     var normal = cross(t1, t2);
     normal = vec3(normal);
     normal = normalize(normal);

     pointsArray.push(vertices[a]);
     pointsArray.push(vertices[b]);
     pointsArray.push(vertices[c]);
     pointsArray.push(vertices[d]);

     texCoordsArray.push(texCoord[0]);
     texCoordsArray.push(texCoord[1]);
     texCoordsArray.push(texCoord[2]);
     texCoordsArray.push(texCoord[3]);

     normalsArray.push(normal);
     normalsArray.push(normal);
     normalsArray.push(normal);
     normalsArray.push(normal);

     tangentsArray.push(t1);
     tangentsArray.push(t1);
     tangentsArray.push(t1);
     tangentsArray.push(t1);
}

// The fifth argument is used to specify which face of the cube it is
function cube() {
    quad(3, 2, 1, 0); 	//front
    quad(7, 6, 2, 3);	//right
    quad(3, 0, 4, 7);	//bottom
    quad(5, 1, 2, 6);	//top
    quad(6, 7, 4, 5);	//back
    quad(5, 4, 0, 1);	//left
    quad(5, 6, 7, 4);   //back flipped
}


window.onload = function init() {

    canvas = document.getElementById("gl-canvas");

    gl = canvas.getContext('webgl2');
    if (!gl) { alert("WebGL 2.0 isn't available"); }

    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.clearColor(0.2, 0.8, 1.0, 1.0);

    gl.enable(gl.DEPTH_TEST);

    //
    //  Load shaders and initialize attribute buffers
    //
    program = initShaders(gl, "vertex-shader", "fragment-shader");

    gl.useProgram(program);

    instanceMatrix = mat4();

    eye = vec3(radius*Math.sin(phi), radius*Math.sin(theta_view),
             radius*Math.cos(phi));
    modelViewMatrix = lookAt(eye, at , up);

    modelViewMatrixLoc = gl.getUniformLocation(program, "modelViewMatrix");
    projectionMatrixLoc = gl.getUniformLocation(program, "projectionMatrix");
    nMatrixLoc = gl.getUniformLocation(program, "uNormalMatrix");
    objectIdLoc = gl.getUniformLocation(program, "objectId");

    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition1"), lightPosition1);
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition2"), lightPosition2);

    cube();

    vBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(pointsArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation(program, "aPosition");
    gl.vertexAttribPointer(positionLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(positionLoc);

    var tBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(texCoordsArray), gl.STATIC_DRAW);

    var texCoordLoc = gl.getAttribLocation(program, "aTexCoord");
    gl.vertexAttribPointer(texCoordLoc, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(texCoordLoc);

    // array buffer for normals
    var nBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, nBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW);

    var normalLoc = gl.getAttribLocation(program, "aNormal");
    gl.vertexAttribPointer(normalLoc, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(normalLoc);

    // array buffer for tangents
    var tangBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tangBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(tangentsArray), gl.STATIC_DRAW);

    var tangentLoc = gl.getAttribLocation(program, "aTangent");
    gl.vertexAttribPointer(tangentLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(tangentLoc);


    // Handling lights
    // Since the two lights have the same properties, we compute the products only once
    // for each object's material and we use them for both lights

    var ambientProduct1 = mult(lightAmbient, materialAmbientSheep);
    var diffuseProduct1 = mult(lightDiffuse, materialDiffuseSheep);
    var specularProduct1 = mult(lightSpecular, materialSpecularSheep); 

    var ambientProduct2 = mult(lightAmbient, materialAmbientGrass);
    var diffuseProduct2 = mult(lightDiffuse, materialDiffuseGrass);
    var specularProduct2 = mult(lightSpecular, materialSpecularGrass);

    var ambientProduct3 = mult(lightAmbient, materialAmbientWood);
    var diffuseProduct3 = mult(lightDiffuse, materialDiffuseWood);
    var specularProduct3 = mult(lightSpecular, materialSpecularWood);

    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct1"),
        ambientProduct1);
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct1"),
        diffuseProduct1);
    gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct1"),
        specularProduct1);
    gl.uniform1f(gl.getUniformLocation(program,
        "uShininess1"), materialShininessSheep);

    
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct2"),
        ambientProduct2);
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct2"),
        diffuseProduct2);
    gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct2"),
        specularProduct2);
    gl.uniform1f(gl.getUniformLocation(program,
        "uShininess2"), materialShininessGrass);

    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct3"),
        ambientProduct3);
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct3"),
        diffuseProduct3);
    gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct3"),
        specularProduct3);
    gl.uniform1f(gl.getUniformLocation(program,
        "uShininess3"), materialShininessWood);

    // Load and configure the textures

    var grass_texture = document.getElementById("texImage1");
    var wood_texture = document.getElementById("texImage2");

    configureTexture(face_texture, grass_texture, wood_texture, bump_texture);

    for(i=0; i<numNodes; i++) initNodes(i);


    // Sliders to move the camera
    document.getElementById("depthSlider").onchange = function(event) {
        far = event.target.value/2;
        near = -event.target.value/2;
    };

    document.getElementById("radiusSlider").onchange = function(event) {
       radius = event.target.value;
    };
    document.getElementById("thetaSlider").onchange = function(event) {
        theta_view = event.target.value* Math.PI/180.0;
    };
    document.getElementById("phiSlider").onchange = function(event) {
        phi = event.target.value* Math.PI/180.0;
    };
    document.getElementById("heightSlider").onchange = function(event) {
        top_view = event.target.value/2;
        bottom = -event.target.value/2;
    };
    document.getElementById("widthSlider").onchange = function(event) {
        right = event.target.value/2;
        left = -event.target.value/2;
    };

    document.getElementById("Start").onclick = function(event) {
        setInterval(function() { start(); }, 150);
    };

    document.getElementById("Reset").onclick = function(event) {
        reset();
    };

    render();
}

// ================================ ANIMATION =============================//
var walk = true;
var stand = false;
var jumping = false;
var landing = false;

function start() {
    // Handling the walking mode: this is used at the beginning and at the end, after the sheep lands
    // on the grass field again. There is an if-else in order to make the legs move back and forth.
    if (sheepPosition[0] < -6.0 || (sheepPosition[0] < 19 && !landing && !jumping && !stand)) {
        
        sheepPosition[0] += 1.0;
        initNodes(torsoId);

        if (walk){

            sheepPosition[1] -= 0.02;
            initNodes(torsoId);
            
            theta[head2Id] += 5;
            initNodes(head2Id);

            theta[leftFrontLegId] -= 5;
            initNodes(leftFrontLegId);

            theta[rightFrontLegId] += 5;
            initNodes(rightFrontLegId);

            theta[leftBackLegId] += 5;
            initNodes(leftBackLegId);

            theta[rightBackLegId] -= 5;
            initNodes(rightBackLegId);

            theta[leftLowerFrontLegId] += 5;
            initNodes(leftLowerFrontLegId);

            theta[rightLowerFrontLegId] -= 5;
            initNodes(rightLowerFrontLegId);

            theta[leftLowerBackLegId] -= 5;
            initNodes(leftLowerBackLegId);

            theta[rightLowerBackLegId] += 5;
            initNodes(rightLowerBackLegId);
                
            if (theta[leftFrontLegId] == -15)
                walk = false;	
            if (sheepPosition[0] >= -6 && sheepPosition[0] < 0)
                stand = true;
        } else{

            theta[head2Id] -= 5;
            initNodes(head2Id);

            theta[leftFrontLegId] += 5;
            initNodes(leftFrontLegId);

            theta[rightFrontLegId] -= 5;
            initNodes(rightFrontLegId);

            theta[leftBackLegId] -= 5;
            initNodes(leftBackLegId);

            theta[rightBackLegId] += 5;
            initNodes(rightBackLegId);

            theta[leftLowerFrontLegId] -= 5;
            initNodes(leftLowerFrontLegId);

            theta[rightLowerFrontLegId] += 5;
            initNodes(rightLowerFrontLegId);

            theta[leftLowerBackLegId] += 5;
            initNodes(leftLowerBackLegId);

            theta[rightLowerBackLegId] -= 5;
            initNodes(rightLowerBackLegId);

            if (theta[leftFrontLegId] == 15)
                walk = true;
            if (sheepPosition[0] >= -6 && sheepPosition[0] < 0)
                stand = true;
        }
    }
    
    // Handling the standing phase before the jump. The sheep's body gets skewed and
    // the the legs move to comply with this movement.
    else if (stand && theta[torsoId] != -50) {

        theta[torsoId] -= 10;
        initNodes(torsoId);

        theta[leftBackLegId] = 4;
        initNodes(leftBackLegId);
        
        theta[rightBackLegId] = theta[leftBackLegId];
        initNodes(rightBackLegId);

        theta[rightFrontLegId] = theta[rightBackLegId];
        initNodes(rightFrontLegId);

        theta[leftFrontLegId] = theta[rightFrontLegId];
        initNodes(leftFrontLegId);

        theta[rightLowerBackLegId] += 5;
        initNodes(rightLowerBackLegId);

        theta[leftLowerBackLegId] = theta[rightLowerBackLegId];
        initNodes(leftLowerBackLegId);

        theta[rightLowerFrontLegId] = theta[rightLowerBackLegId];
        initNodes(rightLowerFrontLegId);

        theta[leftLowerFrontLegId] = theta[rightLowerFrontLegId];
        initNodes(leftLowerFrontLegId);

        sheepPosition[1] += 0.25;
        initNodes(torsoId);

        if (theta[leftLowerBackLegId]== 25 || theta[torsoId] == -50){
            stand = false;
            jumping = true;
        }

    } 
    
    // Handling the sheep's jump over the fence
    else if (jumping) {

        if (sheepPosition[1] < 38.0) {

            theta[torsoId] += 3;
            initNodes(torsoId);

            sheepPosition[0]+=1;
            sheepPosition[1]+=1;
            initNodes(torsoId);

        } else {
            jumping = false;
            landing = true;
        }

    } 
    
    // Handling the landing phase: the sheep's body returns horizontal and the legs go
    // back to their initial position. 
    else if (landing) {
        if (sheepPosition[1] > 29) {

            theta[torsoId] += 2;
            initNodes(torsoId);

            sheepPosition[0]+=1;
            sheepPosition[1]-=1;
            initNodes(torsoId);

            theta[leftLowerBackLegId] -= 3;
            initNodes(leftLowerBackLegId);
    
            theta[rightLowerBackLegId] -= 2.5;
            initNodes(rightLowerBackLegId);
    
            theta[leftLowerFrontLegId] -= 1.5;
            initNodes(leftLowerFrontLegId);
    
            theta[rightLowerFrontLegId] -= 1.5;
            initNodes(rightLowerFrontLegId);
    
            theta[leftBackLegId] -= 1.5;
            initNodes(leftBackLegId);
    
            theta[rightBackLegId] -= 1.0;
            initNodes(rightBackLegId);
    
            theta[leftFrontLegId] -= 1.0;
            initNodes(leftFrontLegId);
    
            theta[rightFrontLegId] -= 1.5;
            initNodes(rightFrontLegId);

        } else if (sheepPosition[1] < 29 && theta[torsoId] != 0) {
            theta[torsoId] -= 5;
            initNodes(torsoId);

            sheepPosition[1] -= 0.15;
            initNodes(torsoId);

            theta[leftBackLegId] = 0;
            initNodes(leftBackLegId);
    
            theta[rightBackLegId] = 0;
            initNodes(rightBackLegId);
    
            theta[leftFrontLegId] = 0;
            initNodes(leftFrontLegId);
    
            theta[rightFrontLegId] = 0;
            initNodes(rightFrontLegId);

            theta[leftLowerBackLegId] = 0;
            initNodes(leftLowerBackLegId);
    
            theta[rightLowerBackLegId] = 0;
            initNodes(rightLowerBackLegId);
    
            theta[leftLowerFrontLegId] = 0;
            initNodes(leftLowerFrontLegId);
    
            theta[rightLowerFrontLegId] = 0;
            initNodes(rightLowerFrontLegId);

            if (theta[torsoId] <= 0) {
            landing = false;
            walk = true;
            }
	    }
    }
}

function reset(){
    window.location.reload();
}


var render = function() {

    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    instanceMatrix = mat4();

    projectionMatrix = ortho(left, right, bottom, top_view, near, far);
     
    eye = vec3(radius*Math.sin(phi), radius*Math.sin(theta_view),
		radius*Math.cos(phi));
    modelViewMatrix = lookAt(eye, at, up);
    nMatrix = normalMatrix(modelViewMatrix, true);

    for(i=0; i<numNodes; i++) initNodes(i);

    gl.uniformMatrix4fv(projectionMatrixLoc, false, flatten(projectionMatrix));
    gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix));

    traverse(grassId);
    requestAnimationFrame(render);
}
