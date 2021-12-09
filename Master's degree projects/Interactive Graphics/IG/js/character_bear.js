import * as THREE from 'https://threejs.org/build/three.module.js';
import {GLTFLoader} from 'https://threejs.org/examples/jsm/loaders/GLTFLoader.js';

import * as MODELS from './models.js';
import * as GAME from './game.js';

export let go_left, go_right, go_up, go_down;
let stand_up=0; // needed in order to make the bear stand up again when the iced lake is finished.
let checkAnimating; // boolean: if true, it means that there is already a movement in progress
                    // so no other movement are allowed to start. Therefore even if you click
                    // another arrow, no movement will start. You have to wait the first movement
                    // to end in order to click again.
let move = true;
let a;
export let hero_position = {x: 0, y: 100, z: 0};
export let bear_score = 0;

export let dead_bear = false;
export let coin_collision = false;

let body, anterior_legL, anterior_legR, posterior_legL, posterior_legR;

// function needed to represent the hierarchical structure of the hero/main character
function dumpObject(obj, lines = [], isLast = true, prefix = '') {
  const localPrefix = isLast ? '└─' : '├─';
  lines.push(`${prefix}${prefix ? localPrefix : ''}${obj.name || '*no-name*'} [${obj.type}]`);
  const newPrefix = prefix + (isLast ? '  ' : '│ ');
  const lastNdx = obj.children.length - 1;
  obj.children.forEach((child, ndx) => {
    const isLast = ndx === lastNdx;
    dumpObject(child, lines, isLast, newPrefix);
  });
  return lines;
}

function create_bear(scene, night) {
    // Allows to decide the influence of light on the bear during the day and at night
    let luceOrso;
    if(night) luceOrso = 0;
    else luceOrso = 0.5;

    const gltfLoader = new GLTFLoader(MODELS.loadingManager);
    //to apply texture
    const textureLoader = new THREE.TextureLoader(MODELS.loadingManager);
    const texture = textureLoader.load('assets/models/level2/polar_bear/textures/MAT_BODY_baseColor.png')
    texture.flipY = false;
    gltfLoader.load('assets/models/level2/polar_bear/scene.gltf', (gltf) => {
      const root = gltf.scene;
      // to apply texture
      root.traverse ( ( o ) => {
        if (o.isMesh ) {
          o.material = new THREE.MeshStandardMaterial({map: texture});
          o.material.skinning = true;
          o.receiveShadow = true
          o.castShadow = true;
          o.material.metalness = luceOrso;
        }
      });

      const r = 100;
      const theta = 0;
      const y = r*Math.cos(theta);
      const z = r*Math.sin(theta);

      root.scale.set(0.0135, 0.0135, 0.0135);
      root.position.set(0, y, z);
      root.rotation.y = Math.PI;
      root.name = "Bear";
      
      // Print the hierarchical model of the main character 
      //console.log(dumpObject(root).join('\n'));

      scene.add(root);

      body = root.getObjectByName('RootNode');

      //anterior left leg
      anterior_legL = root.getObjectByName('front_thighL_012');
      //anterior right leg
      anterior_legR = root.getObjectByName('front_thighR_017');
      anterior_legL.rotation.x += Math.PI/8;  // I do that xk in the 3D model it is not straight allineated as the other legs

      //posterior left leg
      posterior_legL = root.getObjectByName('thighL_025');
      //posterior right leg
      posterior_legR = root.getObjectByName('thighR_029');
      posterior_legR.rotation.x -= Math.PI/8; // I do that xk in the 3D model it is not straight allineated as the other legs
    });
}

let mov, mov_sx, mov_dx; // Needed to clean the interval when ypu want to stop the movement
let mov2, mov3, mov4;

function animate(hModel){

  if(!checkAnimating){
    check_collision_ice();
    if (go_down && !go_left && !go_right) {
      body.position.y = -55;

      posterior_legR.rotation.x = -3.14;
      posterior_legL.rotation.x = -3;
      anterior_legL.rotation.x = 6;
      anterior_legL.rotation.y = -1;
      anterior_legR.rotation.x = 6;
      anterior_legR.rotation.y = 1;

      mov2 = setInterval(function(){stand_down(hModel)}, 100);
    } else if(go_down && go_left){
      checkAnimating = true;
      mov3= setInterval(function(){slide_left(hModel)}, 100);
      hero_position.x -= 2.2;
    } else if(go_down && go_right){
      checkAnimating = true;
      mov4= setInterval(function(){slide_right(hModel)}, 100);
      hero_position.x += 2.2;
    } else if(stand_up==2 && a && !go_down){ //the bear stands up when the ice is finished
      body.position.y = 0;
      posterior_legR.rotation.x = 1.3728129173310428;
      posterior_legL.rotation.x = 1.636908986829248;
      anterior_legL.rotation.x = 0.938198351903096;
      anterior_legL.rotation.y = 0.06744027095270091;
      anterior_legR.rotation.x = 0.7770161045236647;
      anterior_legR.rotation.y = -0.16891183089685902;
      stand_up = 0;
      // otherwise necessary the bear takes a step after it gets up
      go_up = false;
      go_left = false;
      go_right = false;
      a=false;
    }else if(go_up && !go_down) {
      body.rotation.y = 0;
      checkAnimating = true;
      check_collisions_obstacles_front();
      if (move)
        mov = setInterval(function(){move_up(mov, hModel); }, 12);
    } else if(go_left && !go_down){
      checkAnimating = true;
      body.rotation.y  = -80;
      check_collisions_obstacles_around(-1);
      if (move) {
        mov_sx = setInterval(function(){move_left(mov_sx)}, 12);
        hero_position.x -= 2.2;
      }
    } else if(go_right && !go_down){
      checkAnimating = true;
      body.rotation.y  = 80;
      check_collisions_obstacles_around(1);
      if (move) {
        mov_dx = setInterval(function(){move_right(mov_dx)}, 12);
        hero_position.x += 2.2;
      }
    }
  }
}

let lenght_lateral_shift = 0;
let sum_for_score = 0;

function add_score_slider(){
  if (!dead_bear) {
    sum_for_score += 1;
    if(sum_for_score==4){
      sum_for_score=0;
      bear_score += 1;
      document.getElementById("score").innerHTML = "score: " + bear_score;
    }
  }
}

function slide_left(hModel){
  if(lenght_lateral_shift <= 2){
    body.position.x += 40;
    hModel.rotation.x += 0.008865;
    add_score_slider();
    lenght_lateral_shift += 1;
  } else {
    go_left = false;
    lenght_lateral_shift = 0;
    clearInterval(mov3);
    checkAnimating = false;
  }
}

function slide_right(hModel){
  if(lenght_lateral_shift <= 2){
    hModel.rotation.x += 0.008865;
    add_score_slider();
    body.position.x -= 40;
    lenght_lateral_shift += 1;
  } else {
    go_right = false;
    lenght_lateral_shift = 0;
    clearInterval(mov4);
    checkAnimating = false;
  }
}

function stand_down(hModel){
  check_collision_ice();
  if(go_down) {
    hModel.rotation.x += 0.008865;
    add_score_slider();
  } else{
    clearInterval(mov2);
    animate(hModel);
  }
}



//USEFUL VARIABLES FOR THE WALK OF THE BEAR
let switch_legs = false;  //if
let transition = false; //true when I do the intermediate movement,
                        // so I bring all the 4 legs at the starting position
                        //it will be necessary 2 if for the transition,
                        //xk I have 2 different movements for the legs.
let number_steps_per_click_up = 0;



// the bear makes 2 steps. This function is called when the bear has to move straight, left or right.
function make_steps(){
  if (switch_legs==false && transition==false) {
    //move backward
    posterior_legR.rotation.x += Math.PI/40;
    anterior_legL.rotation.x += Math.PI/40;
    //move forward
    posterior_legL.rotation.x -= Math.PI/35;
    anterior_legR.rotation.x -= Math.PI/35;

    if(number_steps_per_click_up==4) transition = true;
    number_steps_per_click_up += 1;

  } else if(switch_legs==false && transition==true){ //transition 1
    posterior_legR.rotation.x -= Math.PI/40;
    anterior_legL.rotation.x -= Math.PI/40;

    posterior_legL.rotation.x += Math.PI/35;
    anterior_legR.rotation.x += Math.PI/35;

    if(number_steps_per_click_up==9){
      switch_legs = true;
      transition = false;
    }
    number_steps_per_click_up += 1;

  } else if(switch_legs==true && transition==false){
    //move forward
    posterior_legR.rotation.x -= Math.PI/40;
    anterior_legL.rotation.x -= Math.PI/40;
    //move backward
    posterior_legL.rotation.x += Math.PI/35;
    anterior_legR.rotation.x += Math.PI/35;

    if(number_steps_per_click_up==14) transition = true;
    number_steps_per_click_up += 1;

  } else if(switch_legs==true && transition==true){ //transition 2
    posterior_legR.rotation.x += Math.PI/40;
    anterior_legL.rotation.x += Math.PI/40;

    posterior_legL.rotation.x -= Math.PI/35;
    anterior_legR.rotation.x -= Math.PI/35;

    if(number_steps_per_click_up==19){
      switch_legs = false;
      transition = false;
    }
    number_steps_per_click_up += 1;

  }
}




function move_up(mov, hModel){

  if(number_steps_per_click_up < 20){ // max 2 steps for each click of the arrow up
    hModel.rotation.x += 0.0018; 
    make_steps();
  } else {
    go_up = false;
    clearInterval(mov); // to stop the recall to the setInterval() fne
    number_steps_per_click_up = 0;  // in order to make possible the next 4 steps when clicking again the arrow up
    checkAnimating = false;
  }
}



function move_left(mov_sx){

  if(number_steps_per_click_up < 20){ // max 2 steps for each click of the arrow up
    body.position.x += 8;
    make_steps();
  } else {
    go_left=false;
    clearInterval(mov_sx); // to stop the recall to the setInterval() function
    number_steps_per_click_up = 0; // in order to make possible the next 4 steps when clicking again the arrow up
    checkAnimating = false;
  }
}


function move_right(mov_dx){
  if(number_steps_per_click_up < 20){ // max 2 steps for each click of the arrow up
    body.position.x -= 8; 
    make_steps();
  } else {
    go_right=false;
    clearInterval(mov_dx); // to stop the recall to the setInterval() fne
    number_steps_per_click_up = 0;  // in order to make possible the next 4 steps when clicking again the arrow up
    checkAnimating = false;
  }

}


export function initialize_bear(scene, hModel, night){

    create_bear(scene, night); //, body, hip, spine, legL, legR, armL, armR

    //// EVENT LISTENER MAIN CHARACTER  /////
    go_left=false;
    go_right=false;
    go_up=false;
    go_down=false;
    checkAnimating=false;
    stand_up = false;
    window.addEventListener("keydown", letsMove, false);

    function letsMove(event) {
      if(GAME.permessoPerMuoversi){
        if((event.keyCode === 37|| event.keyCode === 65) && body.position.x < 1480 && !dead_bear) {  //arrow left or 'a' or 'A'
          check_collisions_obstacles_around(-1);
          if(!checkAnimating && move){
            go_left = true;
            if(!go_down){
              GAME.sound_audio3.play();
            }
          }
        }
        else if((event.keyCode === 38 || event.keyCode === 87) && !dead_bear) { //arrow up or 'w' or 'W'
          check_collisions_obstacles_front();
          if(!checkAnimating && move){
            go_up = true;
            if(!go_down){
              bear_score += 1;
              document.getElementById("score").innerHTML = "score: " + bear_score;
              GAME.sound_audio3.play();
            }          
          }
        }
        else if ((event.keyCode === 39 || event.keyCode === 68) && body.position.x > -1480 && !dead_bear) { //arrow right or 'd' or 'D'
          check_collisions_obstacles_around(1);
          if(!checkAnimating && move){
            go_right = true;
            if(!go_down){
              GAME.sound_audio3.play();
            }
          }
        }
        animate(hModel);  //scene
      }
    }


}

// It checks whethere we are stepping on the ice or we are stepping out of it
// and this is used to handle animations and collisions differently
function check_collision_ice() {
  let pos = new THREE.Vector3();

  MODELS.ice_end.forEach(function (element, index) {
    let ice = MODELS.ice_end[index];
    // last row of iced lake
    pos.setFromMatrixPosition(ice.matrixWorld);
    if (pos.z>12 && pos.z<24 && pos.y > 0){
      go_down = false;
      stand_up = 2;
    }
  });

  MODELS.ice_start.forEach(function (element, index) {
    let ice = MODELS.ice_start[index];
    // first row of the iced lake
    pos.setFromMatrixPosition(ice.matrixWorld);
    if (pos.z > -10 && pos.z < 0 && pos.y > 0){
      go_down = true;
      a=true;
      stand_up = 1;
    }
  });
}

// It checks whether there are obstacles in front of the hero and therefore
// it cannot walk on it
function check_collisions_obstacles_front() {
  move = true;
  coin_collision = false;

  let pos = new THREE.Vector3();
  let distx = 2.5;
  let distz = 4;

  MODELS.bigObstaclesCollision.forEach(function (element, index) {
    let bigObs = MODELS.bigObstaclesCollision[index];

    if (bigObs.name == 'Rock_Snow_1') {distx = 2.0; distz = 6.0;}
    if (bigObs.name == 'treeDecorated') { distx = 3.0; distz = 6;}  
    if (bigObs.name == 'Rock_Snow_6') {distz = 6.5;}
    if (bigObs.name == 'bench') {distx = 4.5; distz = 5.5;}
    if (bigObs.name == 'TreeStump_Snow') {distx = 3.0; distz = 5.5;};
    if (bigObs.name == 'WoodLog_Moss') {distx = 6; distz = 5;}
    if (bigObs.name == 'WoodLog_Snow') {distx = 5.2; distz = 5.3;}

    pos.setFromMatrixPosition(bigObs.matrixWorld);
    if(pos.z >= -14 && pos.z < -1 && pos.y > 0 && hero_position.x <= pos.x + distx && hero_position.x >= pos.x - distx){
        let distance = Math.sqrt(Math.pow(pos.z, 2));
        if(distance <= distz){
          move = false;
        }
    }
  });

  MODELS.smallObstaclesCollision.forEach(function (element, index) {
    let smallObs = MODELS.smallObstaclesCollision[index];
    pos.setFromMatrixPosition(smallObs.matrixWorld);

    if(smallObs.name == 'BirchTree_Dead_Snow_1') {distx = 2; distz = 6;}
    if(smallObs.name == 'BirchTree_Snow_1') {distx = 2.5; distz = 6;}
    if(smallObs.name == 'CommonTree_Snow_4') {distx = 2; distz = 6;}
    if(smallObs.name == 'PineTree_Snow_3') {distx = 1.75; distz = 7;}
    if(smallObs.name == 'candyCane' || smallObs.name == 'candyCaneMint') {distx = 4; distz = 1.5;}
    if(smallObs.name == 'present') {distx = 2; distz = 6;}

    if(pos.z >= -5 && pos.z < 0 && pos.y > 0 && hero_position.x <= pos.x + distx && hero_position.x >= pos.x - distx){
        let distance = Math.sqrt(Math.pow(pos.z, 2));
        if(distance <= distz){
          move = false;
        }
    }
  });

  MODELS.coinsCollisions.forEach(function (element, index) {
    let coin = MODELS.coinsCollisions[index];
    pos.setFromMatrixPosition(coin.matrixWorld);

    let distx = 2;

    if(pos.z >= -14 && pos.z < -2.5 && pos.y > 0 && hero_position.x <= pos.x + distx && hero_position.x >= pos.x - distx){ // check just for cars that are in the next slice, up to 7.5 deg (hero.position.z = 0 always)
        let distance = Math.sqrt(Math.pow(pos.z, 2));
        if(coin.visible && distance <= 6){
          GAME.sound_audio4.play();
          coin.visible = false;
          coin_collision = true;
          bear_score += 10;
          document.getElementById("score").innerHTML = "score: " + bear_score;
        }
    }
  });
}

// It checks whether there are obstacles on the right or left side of the hero.
// We check this only in the direction in which the hero is moving.
function check_collisions_obstacles_around(direction) {
  move = true;
  coin_collision = false;

  let pos = new THREE.Vector3();
  let distx;
  let distz;

  MODELS.bigObstaclesCollision.forEach(function (element, index) {
    let bigObs = MODELS.bigObstaclesCollision[index];
    pos.setFromMatrixPosition(bigObs.matrixWorld);

    if(bigObs.name == 'treeDecorated'){ distx = 4.2; distz = 2;}
    if (bigObs.name == 'Rock_Snow_6') {distx = 4.7; distz = 2;}
    if (bigObs.name == 'bench') {distx = 6.5; distz = 2;}
    if (bigObs.name == 'Rock_Snow_1') {distx = 4.5; distz = 2;}
    if (bigObs.name == 'TreeStump_Snow') {distx = 4.3; distz = 2;};
    if (bigObs.name == 'WoodLog_Moss') {distx = 7; distz = 2;}
    if (bigObs.name == 'WoodLog_Snow') {distx = 6.5; distz = 2;}

    if(pos.z >= -distz && pos.z < distz && pos.y > 0 && ((direction == -1 && pos.x <= hero_position.x) || (direction == 1 && pos.x >= hero_position.x))){
        let distance = Math.sqrt(Math.pow(hero_position.x-pos.x, 2));
        if(distance <= distx){
          move = false;
        }
    }
  });

  MODELS.smallObstaclesCollision.forEach(function (element, index) {
    let smallObs = MODELS.smallObstaclesCollision[index];
    pos.setFromMatrixPosition(smallObs.matrixWorld);


    if(smallObs.name == 'BirchTree_Dead_Snow_1') {distx = 5.5; distz = 2;}
    if(smallObs.name == 'BirchTree_Snow_1') {distx = 5.5; distz = 2.5;}
    if(smallObs.name == 'CommonTree_Snow_4') {distx = 5.5; distz = 2;}
    if(smallObs.name == 'PineTree_Snow_3') {distx = 6.5; distz = 3;}
    if(smallObs.name == 'candyCane' || smallObs.name == 'candyCaneMint') {distx = 4.9; distz = 1.5;}
    if(smallObs.name == 'present') {distx = 5; distz = 2;}

    if(pos.z >= -distz && pos.z < distz && pos.y > 0 && ((direction == -1 && pos.x < hero_position.x) || (direction == 1 && pos.x > hero_position.x))){
      let distance = Math.sqrt(Math.pow(hero_position.x-pos.x, 2));
      if(distance <= distx){
        move = false;
      }
    }
  });

  MODELS.coinsCollisions.forEach(function (element, index) {
    let coin = MODELS.coinsCollisions[index];
    pos.setFromMatrixPosition(coin.matrixWorld);

    let distz = 5;
    let distx = 5.5;

    if(pos.z >= -distz && pos.z < distz && pos.y > 0 && ((direction == -1 && pos.x < hero_position.x) || (direction == 1 && pos.x > hero_position.x))){ // check just for cars that are in the next slice, up to 7.5 deg (hero.position.z = 0 always)
      let distance = Math.sqrt(Math.pow(hero_position.x-pos.x, 2));
      if(coin.visible && distance <= distx){
          GAME.sound_audio4.play();
          coin.visible = false;
          coin_collision = true;
          bear_score += 10;
          document.getElementById("score").innerHTML = "score: " + bear_score;
      }
    }
  });
}

// It checks collisions on ice: if we hit an obstacle the hero dies
export function check_collisions_onice(direction) {
  let pos = new THREE.Vector3();
  let distx;
  let distz;

  MODELS.iceCollisions.forEach(function (element, index) {
    let obs = MODELS.iceCollisions[index];
    pos.setFromMatrixPosition(obs.matrixWorld);

    if (obs.name == 'Rock_Snow_6') {distx = 4; distz = 2;}
    else if (obs.name == 'Rock_Snow_1') {distx = 1.6; distz = 2;}
    else if (obs.name == 'snowFort') {distx = 5; distz = 2;}
    else if (obs.name == 'WoodLog_Snow') {distx = 3.5; distz = 2;}

    // We check collisions while moving to the left/right
    if(pos.z >= -14 && pos.z < 0 && pos.y > 0 && ((direction == -1 && pos.x <= hero_position.x) || (direction == 1 && pos.x >= hero_position.x))){
      let distancex = Math.abs(hero_position.x-pos.x);
      let distancez = Math.abs(pos.z);
      if(distancez <= distz && distancex <= distx && !dead_bear){
        window.cancelAnimationFrame(GAME.id);
        GAME.sound_audio.stop();
        GAME.sound_audio2.play();
        dead_bear = true;
      }
    }

    // We check collisions while we are just moving straight
    if (pos.z >= -14 && pos.z < 0 && pos.y > 0 && direction == 0 && hero_position.x < pos.x + distx && hero_position.x > pos.x - distx) {
      let distancez = Math.abs(pos.z);

      if (obs.name == 'Rock_Snow_6') distx = 3.5;
      else if (obs.name == 'snowFort') distx = 4.5;
      else if (obs.name == 'WoodLog_Snow') distx = 3;

      if(distancez <= 2 && !dead_bear) {
        window.cancelAnimationFrame(GAME.id);
        GAME.sound_audio.stop();
        GAME.sound_audio2.play();
        dead_bear = true;
      }
    } 
  });
}