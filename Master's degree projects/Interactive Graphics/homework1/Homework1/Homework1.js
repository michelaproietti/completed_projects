"use strict";

var canvas;
var gl;

// Setting viewing parameters
var near = 0.1;
var far = 6.0;
var radius = 5.0;
var theta_view = 0.0;
var phi = 0.0;
var dr = 5.0 * Math.PI/180.0;

var  fovy = 45.0;
var  aspect = 1.0;


var ctm;
var ambientColor, diffuseColor, specularColor;
var modelViewMatrix, modelViewMatrix1, projectionMatrix;
var modelViewMatrixLoc, projectionMatrixLoc;
var modelViewMatrixLoc2, projectionMatrixLoc2;
var nMatrix, nMatrix1, nMatrixLoc, nMatrixLoc2;

var viewerPos;
const at = vec3(0.0, 0.0, 0.0);		
const up = vec3(0.0, 1.0, 0.0);
var program1, program2;

var xAxis = 0;
var yAxis = 1;
var zAxis = 2;
var axis = 0;
var theta = vec3(0, 0, 0);
var direction = true;
var shadingModel = true;
var neon = false;
var bumpTexture = false;

var thetaLoc, shadingModelLoc, shadingModelLoc2, neonLoc, neonLoc2, bumpTextureLoc;

var flag = false;

var points = [];
var normals = [];
var colors = [];
var texCoord = [];

var nsolid, ncylinder;

//======================================SOLID=================================//
function solid() {

var data = {};

var texSize = 64;

var positionsArray = [];
var normalsArray = [];
var texCoordsArray = [];
var tangentsArray = [];

var vertices = [
        vec4(-0.5, -0.5,  0.5, 1.0),	//vertex 0
        vec4(-0.5,  0.5,  0.5, 1.0),	//vertex 1
        vec4(0.5,  0.5,  0.5, 1.0),	//vertex 2
        vec4(0.5, -0.5,  0.5, 1.0),	//vertex 3
        vec4(-0.5, -0.5, -0.5, 1.0),	//vertex 4
        vec4(-0.5,  0.5, -0.5, 1.0),	//vertex 5
        vec4(0.5,  0.5, -0.5, 1.0),	//vertex 6
        vec4(0.5, -0.5, -0.5, 1.0),	//vertex 7
	vec4(0.3, 0.8, 0.3, 1.0),	//vertex 8
	vec4(0.3, 0.8, -0.3, 1.0),	//vertex 9
	vec4(-0.3, 0.8, 0.3, 1.0),	//vertex 10
	vec4(-0.3, 0.8, -0.3, 1.0),	//vertex 11
	vec4(0.8, -0.3, 0.3, 1.0),	//vertex 12
	vec4(0.8, -0.3, -0.3, 1.0),	//vertex 13
	vec4(0.8, 0.3, -0.3, 1.0),	//vertex 14
	vec4(0.8, 0.3, 0.3, 1.0),	//vertex 15
	vec4(-0.3, -0.8, -0.3, 1.0),	//vertex 16
	vec4(-0.3, -0.8, 0.3, 1.0),	//vertex 17
	vec4(0.3, -0.8, 0.3, 1.0),	//vertex 18
	vec4(0.3, -0.8, -0.3, 1.0),	//vertex 19
	vec4(-0.8, 0.3, -0.3, 1.0),	//vertex 20
	vec4(-0.8, -0.3, -0.3, 1.0),	//vertex 21
	vec4(-0.8, -0.3, 0.3, 1.0),	//vertex 22
	vec4(-0.8, 0.3, 0.3, 1.0),	//vertex 23
	vec4(0.0, 0.0, 1.2, 1.0),	//vertex 24
	vec4(0.0, 0.0, -0.8, 1.0)	//vertex 25
];

var texCoord = [
    vec2(0, 0), 	//0
    vec2(0, 1),		//1
    vec2(1, 1),		//2
    vec2(1, 0),		//3
    vec2(0.5, 0.5),	//4
    vec2(0.2, 0.8),	//5
    vec2(0.8, 0.8)	//6
];

function quad(a, b, c, d, quad) {
     var t1 = subtract(vertices[b], vertices[a]);
     var t2 = subtract(vertices[c], vertices[b]);
     var normal = cross(t1, t2);
     normal = vec3(normal);
     normal = normalize(normal);
     
     // Handling triangles as degenerate quads
     if (c == d) {
	positionsArray.push(vertices[a]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[0]);
     	positionsArray.push(vertices[b]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[1]);
     	positionsArray.push(vertices[c]);
	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[4]);
     }
     else if (quad == true) {
     	positionsArray.push(vertices[a]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[0]);
     	positionsArray.push(vertices[b]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[1]);
     	positionsArray.push(vertices[c]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[2]);
     	positionsArray.push(vertices[a]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[0]);
     	positionsArray.push(vertices[c]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[2]);
     	positionsArray.push(vertices[d]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[3]);
     }
     else {
     	positionsArray.push(vertices[a]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[0]);
     	positionsArray.push(vertices[b]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[5]);
     	positionsArray.push(vertices[c]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[6]);
     	positionsArray.push(vertices[a]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[0]);
     	positionsArray.push(vertices[c]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[6]);
     	positionsArray.push(vertices[d]);
     	normalsArray.push(normal);
     	tangentsArray.push(t1);
	texCoordsArray.push(texCoord[3]);
     }
}

function buildSolid() {
    quad(6, 9, 8, 2, false);		//top
    quad(8, 9, 11, 10, true);
    quad(1, 10, 11, 5, false);
    quad(2, 8, 10, 1, false);
    quad(5, 11, 9, 6, false);

    quad(2, 15, 14, 6, false);		//right
    quad(3, 12, 15, 2, false);
    quad(7, 13, 12, 3, false);
    quad(6, 14, 13, 7, false);
    quad(15, 12, 13, 14, true);

    quad(0, 17, 18, 3, false);		//bottom
    quad(3, 18, 19, 7, false);
    quad(7, 19, 16, 4, false);
    quad(4, 16, 17, 0, false);
    quad(19, 18, 17, 16, true);

    quad(1, 23, 22, 0, false);		//left
    quad(0, 22, 21, 4, false);
    quad(4, 21, 20, 5, false);
    quad(5, 20, 23, 1, false);
    quad(22, 23, 20, 21, true);

    quad(2, 1, 24, 24, false);		//front
    quad(3, 2, 24, 24, false);
    quad(0, 3, 24, 24, false);
    quad(1, 0, 24, 24, false);

    quad(5, 6, 25, 25, false);		//back
    quad(6, 7, 25, 25, false);
    quad(7, 4, 25, 25, false);
    quad(4, 5, 25, 25, false);
}

buildSolid();

data.TriangleVertices = positionsArray;
data.TriangleNormals = normalsArray;
data.TextureCoordinates = texCoordsArray;
data.TriangleTangents = tangentsArray;

return data;
}

//================================CYLINDER====================================//
function cylinder(numSides, numLevels, caps) {

var sides = 36;
if(numSides) sides = numSides;
var levels = 1;
if(numLevels) levels = numLevels;
var capsFlag = true;
if(caps==false) capsFlag = caps;

var data = {};

var top = 0.5;
var bottom = -0.5;
var radius = 0.5;
var topCenter = [0.0, top, 0.0];
var bottomCenter = [0.0, bottom, 0.0];


var color = [0.0, 0.2, 1.0, 0.0];


var cylinderVertexCoordinates = [];
var cylinderNormals = [];
var cylinderVertexColors = [];
var cylinderTextureCoordinates = [];

// side

for(var j=0; j<levels; j++) {
  var stop = bottom + (j+1)*(top-bottom)/levels;
  var sbottom = bottom + j*(top-bottom)/levels;
  var topPoints = [];
  var bottomPoints = [];
  var topST = [];
  var bottomST = [];
  for(var i =0; i<sides; i++) {
    var theta = 2.0*i*Math.PI/sides;
    topPoints.push([radius*Math.sin(theta), stop, radius*Math.cos(theta), 1.0]);
    bottomPoints.push([radius*Math.sin(theta), sbottom, radius*Math.cos(theta), 1.0]);
  };

  topPoints.push([0.0, stop, radius, 1.0]);
  bottomPoints.push([0.0, sbottom, radius, 1.0]);


  for(var i=0; i<sides; i++) {
    var a = topPoints[i];
    var d = topPoints[i+1];
    var b = bottomPoints[i];
    var c = bottomPoints[i+1];
    var u = [b[0]-a[0], b[1]-a[1], b[2]-a[2]];
    var v = [c[0]-b[0], c[1]-b[1], c[2]-b[2]];

    var normal = [
      u[1]*v[2] - u[2]*v[1],
      u[2]*v[0] - u[0]*v[2],
      u[0]*v[1] - u[1]*v[0]
    ];

    var mag = Math.sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2])
    normal = [normal[0]/mag, normal[1]/mag, normal[2]/mag];
    cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
    cylinderVertexColors.push(color);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/sides, j*(top-bottom)/levels]);

    cylinderVertexCoordinates.push([b[0], b[1], b[2], 1.0]);
    cylinderVertexColors.push(color);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([i/sides, (j-1)*(top-bottom)/levels]);

    cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
    cylinderVertexColors.push(color);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/sides, (j-1)*(top-bottom)/levels]);

    cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
    cylinderVertexColors.push(color);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/sides, j*(top-bottom)/levels]);

    cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
    cylinderVertexColors.push(color);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/sides, (j-1)*(top-bottom)/levels]);

    cylinderVertexCoordinates.push([d[0], d[1], d[2], 1.0]);
    cylinderVertexColors.push(color);
    cylinderNormals.push([normal[0], normal[1], normal[2]]);
    cylinderTextureCoordinates.push([(i+1)/sides, j*(top-bottom)/levels]);
  };
};

  var topPoints = [];
  var bottomPoints = [];
  for(var i =0; i<sides; i++) {
    var theta = 2.0*i*Math.PI/sides;
    topPoints.push([radius*Math.sin(theta), top, radius*Math.cos(theta), 1.0]);
    bottomPoints.push([radius*Math.sin(theta), bottom, radius*Math.cos(theta), 1.0]);
  };
  topPoints.push([0.0, top, radius, 1.0]);
  bottomPoints.push([0.0, bottom, radius, 1.0]);

if(capsFlag) {

//top

for(i=0; i<sides; i++) {
  normal = [0.0, 1.0, 0.0];
  var a = [0.0, top, 0.0, 1.0];
  var b = topPoints[i];
  var c = topPoints[i+1];
  cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
  cylinderVertexColors.push(color);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);

  cylinderVertexCoordinates.push([b[0], b[1], b[2], 1.0]);
  cylinderVertexColors.push(color);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);

  cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
  cylinderVertexColors.push(color);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);
};

//bottom

for(i=0; i<sides; i++) {
  normal = [0.0, -1.0, 0.0];
  var a = [0.0, bottom, 0.0, 1.0];
  var b = bottomPoints[i];
  var c = bottomPoints[i+1];
  cylinderVertexCoordinates.push([a[0], a[1], a[2], 1.0]);
  cylinderVertexColors.push(color);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);

  cylinderVertexCoordinates.push([b[0], b[1], b[2], 1.0]);
  cylinderVertexColors.push(color);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);

  cylinderVertexCoordinates.push([c[0], c[1], c[2], 1.0]);
  cylinderVertexColors.push(color);
  cylinderNormals.push(normal);
  cylinderTextureCoordinates.push([0, 1]);
};
};

function translate(x, y, z){
   for(var i=0; i<cylinderVertexCoordinates.length; i++) {
     cylinderVertexCoordinates[i][0] += x;
     cylinderVertexCoordinates[i][1] += y;
     cylinderVertexCoordinates[i][2] += z;
   };
}

function scale(sx, sy, sz){
    for(var i=0; i<cylinderVertexCoordinates.length; i++) {
        cylinderVertexCoordinates[i][0] *= sx;
        cylinderVertexCoordinates[i][1] *= sy;
        cylinderVertexCoordinates[i][2] *= sz;
        cylinderNormals[i][0] /= sx;
        cylinderNormals[i][1] /= sy;
        cylinderNormals[i][2] /= sz;
    };
}

function radians( degrees ) {
    return degrees * Math.PI / 180.0;
}

function rotate( angle, axis) {

    var d = Math.sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);

    var x = axis[0]/d;
    var y = axis[1]/d;
    var z = axis[2]/d;

    var c = Math.cos( radians(angle) );
    var omc = 1.0 - c;
    var s = Math.sin( radians(angle) );

    var mat = [
        [ x*x*omc + c,   x*y*omc - z*s, x*z*omc + y*s ],
        [ x*y*omc + z*s, y*y*omc + c,   y*z*omc - x*s ],
        [ x*z*omc - y*s, y*z*omc + x*s, z*z*omc + c ]
    ];

    for(var i=0; i<cylinderVertexCoordinates.length; i++) {
          var u = [0, 0, 0];
          var v = [0, 0, 0];
          for( var j =0; j<3; j++)
           for( var k =0 ; k<3; k++) {
              u[j] += mat[j][k]*cylinderVertexCoordinates[i][k];
              v[j] += mat[j][k]*cylinderNormals[i][k];
            };
           for( var j =0; j<3; j++) {
             cylinderVertexCoordinates[i][j] = u[j];
             cylinderNormals[i][j] = v[j];
           };
    };
}

data.TriangleVertices = cylinderVertexCoordinates;
data.TriangleNormals = cylinderNormals;
data.TriangleVertexColors = cylinderVertexColors;
data.TextureCoordinates = cylinderTextureCoordinates;
data.rotate = rotate;
data.translate = translate;
data.scale = scale;

return data;
}

//=================================SOLID MATERIAL=============================//

function opaqueMaterial() {
  var data  = {};
  data.materialAmbient = vec4(0.4, 0.2, 0.8, 1.0);
  data.materialDiffuse = vec4(0.4, 0.4, 0.8, 1.0);
  data.materialSpecular = vec4(0.8, 0.8, 0.8, 1.0);
  data.materialShininess = 100.0;
  return data;
}

//===============================CYLINDER MATERIAL============================//

function emissiveMaterial() {
  var data  = {};
  data.materialAmbient = vec4(0.2, 0.2, 0.2, 1.0);
  data.materialDiffuse = vec4(0.5, 0.5, 0.5, 1.0);
  data.materialSpecular = vec4(0.0, 0.0, 0.0, 1.0);
  data.materialEmissive = vec4(0.5, 0.5, 0.5, 1.0);
  data.materialShininess = 100.0;
  return data;
}

//====================================LIGHTS=================================//

function light(x, y, z, generic) {
    var data = {};
    data.lightPosition = vec4(x, y, z, 0.0);

    if (generic) {
	data.lightDiffuse = vec4(1.0, 1.0, 1.0, 1.0);
	data.lightAmbient = vec4(0.2, 0.2, 0.2, 1.0);
	data.lightSpecular = vec4(1.0, 1.0, 1.0, 1.0);
    } else {
	data.lightDiffuse = vec4(0.1, 0.1, 0.1, 1.0);
	data.lightAmbient = vec4(0.1, 0.1, 0.1, 1.0);
	data.lightSpecular = vec4(1.0, 1.0, 1.0, 1.0);
    }

    data.lightShineness = 100.0;
    return data;
}

//===============================INITIAL TEXTURE==============================//

var texSize = 64;
var numChecks = 16;
var c;

// We define the texture to use by defining a checkerboard in floats
var image = new Uint8Array(4*texSize*texSize);

    for (var i = 0; i < texSize; i++) {
        for (var j = 0; j <texSize; j++) {
            var patchx = Math.floor(i/(texSize/numChecks));
            if(patchx%2) c = 255;
            else c = 0;
            image[4*i*texSize+4*j] = c;
            image[4*i*texSize+4*j+1] = c;
            image[4*i*texSize+4*j+2] = c;
            image[4*i*texSize+4*j+3] = 255;
        }
    }

var texture1 = new Uint8Array(4*texSize*texSize);

    // Create a checkerboard pattern
    for ( var i = 0; i < texSize; i++ ) {
        for ( var j = 0; j <texSize; j++ ) {
            var patchy = Math.floor(j/(texSize/numChecks));
            if(patchy%2) c = 255;
            else c = 0;
            texture1[4*i*texSize+4*j] = c;
            texture1[4*i*texSize+4*j+1] = c;
            texture1[4*i*texSize+4*j+2] = c;
            texture1[4*i*texSize+4*j+3] = 255;
           }
    }

/*===================ALTERNATIVE INITIAL TEXTURE==========================

var image = new Array()
    for (var i =0; i<texSize; i++)  image[i] = new Array();
    for (var i =0; i<texSize; i++)
        for ( var j = 0; j < texSize; j++)
           image[i][j] = new Float32Array(4);
    for (var i =0; i<texSize; i++) for (var j=0; j<texSize; j++) {
        var c = (((i & 0x8) == 0) ^ ((j & 0x8) == 0));
        image[i][j] = [c, c, c, 1];
    }

// Convert floats to ubytes for texture

var texture1 = new Uint8Array(4*texSize*texSize);

    for (var i = 0; i < texSize; i++)
        for (var j = 0; j < texSize; j++)
           for(var k =0; k<4; k++)
                texture1[4*texSize*i+4*j+k] = 255*image[i][j][k];

=========================================================================*/

function configureTexture(image) {
    var texture = gl.createTexture();
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, texSize, texSize, 0,
        gl.RGBA, gl.UNSIGNED_BYTE, image);
    gl.generateMipmap(gl.TEXTURE_2D);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER,
        	     gl.NEAREST_MIPMAP_LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
}

//=============================BUMP TEXTURE===================================//

var texSizeBump = 64;

// Bump Data
/*var data = new Array()
    for (var i = 0; i<= texSizeBump; i++)  data[i] = new Array();
    for (var i = 0; i<= texSizeBump; i++) for (var j=0; j<=texSizeBump; j++)
        data[i][j] = 0.0;
    for (var i = texSizeBump/2-28; i<texSizeBump/2+28; i++) for (var j = texSizeBump/2-28; j<texSizeBump/2+28; j++)
        data[i][j] = 1.0;
    for (var i = texSizeBump/2-24; i<texSizeBump/2+24; i++) for (var j = texSizeBump/2-24; j<texSizeBump/2+24; j++)
        data[i][j] = 0.0;
    for (var i = texSizeBump/2-20; i<texSizeBump/2+20; i++) for (var j = texSizeBump/2-20; j<texSizeBump/2+20; j++)
        data[i][j] = 1.0
    for (var i = texSizeBump/2-16; i<texSizeBump/2+16; i++) for (var j = texSizeBump/2-16; j<texSizeBump/2+16; j++)
        data[i][j] = 0.0;
    for (var i = texSizeBump/2-12; i<texSizeBump/2+12; i++) for (var j = texSizeBump/2-12; j<texSizeBump/2+12; j++)
        data[i][j] = 1.0;
    for (var i = texSizeBump/2-8; i<texSizeBump/2+8; i++) for (var j = texSizeBump/2-8; j<texSizeBump/2+8; j++)
        data[i][j] = 0.0;
    for (var i = texSizeBump/2-4; i<texSizeBump/2+4; i++) for (var j = texSizeBump/2-4; j<texSizeBump/2+4; j++)
        data[i][j] = 1.0;*/

// -----------------------------ALTERNATIVE BUMP TEXTURE---------------------------

var data = new Array()
    for (var i = 0; i<= texSizeBump; i++)  data[i] = new Array();
    for (var i = 0; i<= texSizeBump; i++) for (var j=0; j<=texSizeBump; j++)
        data[i][j] = 0.0;
    for (var i = 0; i<= texSizeBump; i+=4) for (var j=0; j<=texSizeBump; j+=4)
        data[i][j] = 1.0;

//----------------------------------------------------------------------------------*/


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

    for (var i=0; i<texSizeBump; i++) for (var j=0; j<texSizeBump; j++) {
       var d = 0;
       for(k=0;k<3;k++) d+=normalst[i][j][k]*normalst[i][j][k];
       d = Math.sqrt(d);
       for(k=0;k<3;k++) normalst[i][j][k]= 0.5*normalst[i][j][k]/d + 0.5;
    }

// Normal Texture Array

var texture2 = new Uint8Array(3*texSizeBump*texSizeBump);

    for ( var i = 0; i < texSizeBump; i++ )
        for ( var j = 0; j < texSizeBump; j++ )
           for(var k =0; k<3; k++)
                texture2[3*texSizeBump*i+3*j+k] = 255*normalst[i][j][k];

function configureTextureBump(image) {
    var texture = gl.createTexture();
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, texSizeBump, texSizeBump, 0, gl.RGB, gl.UNSIGNED_BYTE, image);
    gl.generateMipmap(gl.TEXTURE_2D);
}
//============================================================================//


window.onload = function init() {
    canvas = document.getElementById("gl-canvas");

    gl = canvas.getContext('webgl2');
    if (!gl) alert( "WebGL 2.0 isn't available");

    gl.viewport(0, 0, canvas.width, canvas.height);
    aspect =  canvas.width/canvas.height;
    gl.clearColor(0.9, 0.9, 0.9, 1.0);

    gl.enable(gl.DEPTH_TEST);

    // define objects
    var mySolid = solid();

    var myCylinder = cylinder(72, 3, true);
    myCylinder.scale(0.25, 1.0, 0.25);
    myCylinder.rotate(90.0, [0, 0, 1]);
    myCylinder.translate(0.0, 1.5, 1.0);

    // defining lights and materials
    var solidMaterial = opaqueMaterial();
    var cylinderMaterial = emissiveMaterial();
    var generic_light = light(1.0, 1.0, 2.5, true);
    var light1 = light(0.0, 1.5, 1.0, false);
    var light2 = light(0.4, 1.5, 1.0, false);
    var light3 = light(-0.4, 1.5, 1.0, false);

    // setting points, normals, texture coordinates and colors arrays
    points = mySolid.TriangleVertices;
    normals = mySolid.TriangleNormals;
    texCoord = mySolid.TextureCoordinates;
    colors = myCylinder.TriangleVertexColors;
    points = points.concat(myCylinder.TriangleVertices);
    normals = normals.concat(myCylinder.TriangleNormals);
    texCoord = texCoord.concat(myCylinder.TextureCoordinates);

    // number of vertices in the solid and in the cylinder
    nsolid = mySolid.TriangleVertices.length;
    ncylinder = myCylinder.TriangleVertices.length;

    // load shaders and initialize attribute buffers
    program1 = initShaders(gl, "vertex-shader", "fragment-shader");
    program2 = initShaders(gl, "vertex-shader-light", "fragment-shader-light");

    // array buffer for normals
    var nBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, nBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(normals), gl.STATIC_DRAW);

    var normalLoc = gl.getAttribLocation(program1, "aNormal");
    gl.vertexAttribPointer(normalLoc, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(normalLoc);

    var normalLoc2 = gl.getAttribLocation(program2, "aNormal");
    gl.vertexAttribPointer(normalLoc2, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(normalLoc2);

    // array buffer for vertices
    var vBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(points), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation(program1, "aPosition");
    gl.vertexAttribPointer(positionLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(positionLoc);

    var positionLoc2 = gl.getAttribLocation(program2, "aPosition");
    gl.vertexAttribPointer(positionLoc2, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(positionLoc2);

    // array buffer for textures
    var tBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(texCoord), gl.STATIC_DRAW);

    var texCoordLoc = gl.getAttribLocation(program1, "aTexCoord");
    gl.vertexAttribPointer(texCoordLoc, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(texCoordLoc);

    // array buffer for tangents
    var tangBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tangBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(mySolid.TriangleTangents), gl.STATIC_DRAW);

    var tangentLoc = gl.getAttribLocation(program1, "aTangent");
    gl.vertexAttribPointer(tangentLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(tangentLoc);


    thetaLoc = gl.getUniformLocation(program1, "theta");

    // products between material and light properties
    // generic light
    var ambientProduct1 = mult(generic_light.lightAmbient, solidMaterial.materialAmbient);
    var diffuseProduct1 = mult(generic_light.lightDiffuse, solidMaterial.materialDiffuse);
    var specularProduct1 = mult(generic_light.lightSpecular, solidMaterial.materialSpecular); 

    // generic light
    var ambientProduct2 = mult(generic_light.lightAmbient, cylinderMaterial.materialAmbient);
    var diffuseProduct2 = mult(generic_light.lightDiffuse, cylinderMaterial.materialDiffuse);
    var specularProduct2 = mult(generic_light.lightSpecular, cylinderMaterial.materialSpecular); 

    // light 1
    var ambientProduct11 = mult(light1.lightAmbient, solidMaterial.materialAmbient);
    var diffuseProduct11 = mult(light1.lightDiffuse, solidMaterial.materialDiffuse);
    var specularProduct11 = mult(light1.lightSpecular, solidMaterial.materialSpecular);

    var ambientProduct12 = mult(light1.lightAmbient, cylinderMaterial.materialAmbient);
    var diffuseProduct12 = mult(light1.lightDiffuse, cylinderMaterial.materialDiffuse);
    var specularProduct12 = mult(light1.lightSpecular, cylinderMaterial.materialSpecular);

    //light 2
    var ambientProduct21 = mult(light2.lightAmbient, solidMaterial.materialAmbient);
    var diffuseProduct21 = mult(light2.lightDiffuse, solidMaterial.materialDiffuse);
    var specularProduct21 = mult(light2.lightSpecular, solidMaterial.materialSpecular);

    var ambientProduct22 = mult(light2.lightAmbient, cylinderMaterial.materialAmbient);
    var diffuseProduct22 = mult(light2.lightDiffuse, cylinderMaterial.materialDiffuse);
    var specularProduct22 = mult(light2.lightSpecular, cylinderMaterial.materialSpecular);

    // light 3
    var ambientProduct31 = mult(light3.lightAmbient, solidMaterial.materialAmbient);
    var diffuseProduct31 = mult(light3.lightDiffuse, solidMaterial.materialDiffuse);
    var specularProduct31 = mult(light3.lightSpecular, solidMaterial.materialSpecular);

    var ambientProduct32 = mult(light3.lightAmbient, cylinderMaterial.materialAmbient);
    var diffuseProduct32 = mult(light3.lightDiffuse, cylinderMaterial.materialDiffuse);
    var specularProduct32 = mult(light3.lightSpecular, cylinderMaterial.materialSpecular);


    // Rotation controls
    document.getElementById("ButtonT").onclick = function(){
	flag = !flag;
	if(flag) document.getElementById("ButtonT").innerHTML="Stop rotation";
	else document.getElementById("ButtonT").innerHTML="Start rotation";
    };
    document.getElementById("Direction").onclick = function () {
	direction = !direction;
    };
    document.getElementById("AxisRotation" ).onclick = function(event) {
        switch(event.target.index) {
          case 0:
            axis = xAxis;
            break;
         case 1:
            axis = yAxis;
            break;
         case 2:
            axis = zAxis;
            break;
       }
    };


    // Buttons to control the viewing parameters
    document.getElementById("zFarSlider").onchange = function(event) {
        far = event.target.value;
    };
    document.getElementById("zNearSlider").onchange = function(event) {
        near = event.target.value;
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
    document.getElementById("aspectSlider").onchange = function(event) {
        aspect = event.target.value;
    };
    document.getElementById("fovSlider").onchange = function(event) {
        fovy = event.target.value;
    };

    document.getElementById("Reset").onclick = function () {
	near = 0.1;
	far = 6.0;
	radius = 5.0;
	theta_view = 0.0;
	phi = 0.0;
	fovy = 45.0;
	aspect = canvas.width / canvas.height;
    };

    // Button to change shading model
    document.getElementById("ShadingModel").onclick = function() {
	shadingModel = !shadingModel;
	if (shadingModel) document.getElementById("ShadingModel").innerHTML="Activate per fragment shading";
	else document.getElementById("ShadingModel").innerHTML="Deactivate per fragment shading";
    };

    // Button to control neon light
    document.getElementById("Neon").onclick = function() {
	neon = !neon;
	if (neon) {
	    document.getElementById("Neon").innerHTML="Turn off";
	    if (bumpTexture) {
		bumpTexture = !bumpTexture;
		document.getElementById("BumpTexture").innerHTML="Apply bump texture";
	    }
	}
	else document.getElementById("Neon").innerHTML="Turn on";
    };

    // Button to activate/deactivate bump texture
    document.getElementById("BumpTexture").onclick = function() {
	bumpTexture = !bumpTexture;
	if(bumpTexture) {
	    document.getElementById("BumpTexture").innerHTML="Remove bump texture";
	    if (neon) {
		neon = !neon;
		document.getElementById("Neon").innerHTML="Turn on";
	    }
	}
	else document.getElementById("BumpTexture").innerHTML="Apply bump texture";
    };

    gl.useProgram(program1);

    // send uniform variables for lighting to program 1
    gl.uniform4fv(gl.getUniformLocation(program1, "uAmbientProduct"),
       ambientProduct1);
    gl.uniform4fv(gl.getUniformLocation(program1, "uDiffuseProduct"),
       diffuseProduct1);
    gl.uniform4fv(gl.getUniformLocation(program1, "uSpecularProduct"),
       specularProduct1);
    gl.uniform4fv(gl.getUniformLocation(program1, "uLightPosition"),
       generic_light.lightPosition);

    gl.uniform4fv(gl.getUniformLocation(program1, "uAmbientProduct11"),
       ambientProduct11);
    gl.uniform4fv(gl.getUniformLocation(program1, "uDiffuseProduct11"),
       diffuseProduct11);
    gl.uniform4fv(gl.getUniformLocation(program1, "uSpecularProduct11"),
       specularProduct11);
    gl.uniform4fv(gl.getUniformLocation(program1, "uLightPosition1"),
       light1.lightPosition);

    gl.uniform4fv(gl.getUniformLocation(program1, "uAmbientProduct21"),
       ambientProduct21);
    gl.uniform4fv(gl.getUniformLocation(program1, "uDiffuseProduct21"),
       diffuseProduct21);
    gl.uniform4fv(gl.getUniformLocation(program1, "uSpecularProduct21"),
       specularProduct21);
    gl.uniform4fv(gl.getUniformLocation(program1, "uLightPosition2"),
       light2.lightPosition);

    gl.uniform4fv(gl.getUniformLocation(program1, "uAmbientProduct31"),
       ambientProduct31);
    gl.uniform4fv(gl.getUniformLocation(program1, "uDiffuseProduct31"),
       diffuseProduct31);
    gl.uniform4fv(gl.getUniformLocation(program1, "uSpecularProduct31"),
       specularProduct11);
    gl.uniform4fv(gl.getUniformLocation(program1, "uLightPosition3"),
       light3.lightPosition);

    gl.uniform1f(gl.getUniformLocation(program1,
       "uShininess"), solidMaterial.materialShininess);

    modelViewMatrixLoc = gl.getUniformLocation(program1, "uModelViewMatrix");
    projectionMatrixLoc = gl.getUniformLocation(program1, "uProjectionMatrix");
    nMatrixLoc = gl.getUniformLocation(program1, "uNormalMatrix");

    shadingModelLoc = gl.getUniformLocation(program1, "uShadingModel");
    neonLoc = gl.getUniformLocation(program1, "uNeon");
    bumpTextureLoc = gl.getUniformLocation(program1, "uBumpTexture");
   
    gl.useProgram(program2);

    // send uniform variables for lighting to program 2
    gl.uniform4fv(gl.getUniformLocation(program2, "uAmbientProduct"),
       ambientProduct2);
    gl.uniform4fv(gl.getUniformLocation(program2, "uDiffuseProduct"),
       diffuseProduct2);
    gl.uniform4fv(gl.getUniformLocation(program2, "uSpecularProduct"),
       specularProduct2);
    gl.uniform4fv(gl.getUniformLocation(program2, "uLightPosition"),
       generic_light.lightPosition);

    gl.uniform4fv(gl.getUniformLocation(program2, "uAmbientProduct12"),
       ambientProduct12);
    gl.uniform4fv(gl.getUniformLocation(program2, "uDiffuseProduct12"),
       diffuseProduct12);
    gl.uniform4fv(gl.getUniformLocation(program2, "uSpecularProduct12"),
       specularProduct12);
    gl.uniform4fv(gl.getUniformLocation(program2, "uLightPosition1"),
       light1.lightPosition);

    gl.uniform4fv(gl.getUniformLocation(program2, "uAmbientProduct22"),
       ambientProduct22);
    gl.uniform4fv(gl.getUniformLocation(program2, "uDiffuseProduct22"),
       diffuseProduct22);
    gl.uniform4fv(gl.getUniformLocation(program2, "uSpecularProduct22"),
       specularProduct22);
    gl.uniform4fv(gl.getUniformLocation(program2, "uLightPosition2"),
       light2.lightPosition);

    gl.uniform4fv(gl.getUniformLocation(program2, "uAmbientProduct32"),
       ambientProduct32);
    gl.uniform4fv(gl.getUniformLocation(program2, "uDiffuseProduct32"),
       diffuseProduct32);
    gl.uniform4fv(gl.getUniformLocation(program2, "uSpecularProduct32"),
       specularProduct32);
    gl.uniform4fv(gl.getUniformLocation(program2, "uLightPosition3"),
       light3.lightPosition);

    gl.uniform1f(gl.getUniformLocation(program2,
       "uShininess"), cylinderMaterial.materialShininess);
    gl.uniform4fv(gl.getUniformLocation(program2,
       "uEmissive"), cylinderMaterial.materialEmissive);

    modelViewMatrixLoc2 = gl.getUniformLocation(program2, "uModelViewMatrix");
    projectionMatrixLoc2 = gl.getUniformLocation(program2, "uProjectionMatrix");
    nMatrixLoc2 = gl.getUniformLocation(program2, "uNormalMatrix");

    shadingModelLoc2 = gl.getUniformLocation(program2, "uShadingModel");
    neonLoc2 = gl.getUniformLocation(program2, "uNeon");

    render();
}

var render = function(){

    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    if(flag) theta[axis] += (direction ? 2.0 : -2.0);

    viewerPos = vec3(radius*Math.sin(theta_view)*Math.cos(phi),
	  radius*Math.sin(theta_view)*Math.sin(phi), radius*Math.cos(theta_view));
    modelViewMatrix = lookAt(viewerPos, at , up);
    projectionMatrix = perspective(fovy, aspect, near, far);
    nMatrix = normalMatrix(modelViewMatrix, true);

    var barycenter = vec3(0, 0, 0.1);

    modelViewMatrix1 = mult(modelViewMatrix, translate(barycenter[0], barycenter[1], barycenter[2]));    

    modelViewMatrix1 = mult(modelViewMatrix1, rotate(theta[xAxis], vec3(1, 0, 0)));
    modelViewMatrix1 = mult(modelViewMatrix1, rotate(theta[yAxis], vec3(0, 1, 0)));
    modelViewMatrix1 = mult(modelViewMatrix1, rotate(theta[zAxis], vec3(0, 0, 1)));

    modelViewMatrix1 = mult(modelViewMatrix1, inverse(translate(barycenter[0], barycenter[1], barycenter[2])));

    nMatrix1 = normalMatrix(modelViewMatrix1, true);

    gl.useProgram(program1);
    gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(modelViewMatrix1));
    gl.uniformMatrix4fv(projectionMatrixLoc, false, flatten(projectionMatrix));
    gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix1));
   
    gl.uniform1i(shadingModelLoc, shadingModel);
    gl.uniform1i(neonLoc, neon);
    gl.uniform1i(bumpTextureLoc, bumpTexture);
   
    if(bumpTexture) {
	configureTextureBump(texture2);
	gl.uniform1i(gl.getUniformLocation(program1, "uTextureMap"), 0);
    } else {
	configureTexture(texture1);
	gl.uniform1i(gl.getUniformLocation(program1, "uTextureMap"), 0);
    }

    gl.drawArrays(gl.TRIANGLES, 0, nsolid);

    gl.useProgram(program2);
    gl.uniformMatrix4fv(modelViewMatrixLoc2, false, flatten(modelViewMatrix));
    gl.uniformMatrix4fv(projectionMatrixLoc2, false, flatten(projectionMatrix));
    gl.uniformMatrix3fv(nMatrixLoc2, false, flatten(nMatrix));
   
    gl.uniform1i(shadingModelLoc2, shadingModel);
    gl.uniform1i(neonLoc2, neon);

    gl.drawArrays(gl.TRIANGLES, nsolid, ncylinder);

    requestAnimationFrame(render);
}
