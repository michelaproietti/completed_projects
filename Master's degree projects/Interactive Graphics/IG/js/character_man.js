import * as THREE from 'https://threejs.org/build/three.module.js';
import {GLTFLoader} from 'https://threejs.org/examples/jsm/loaders/GLTFLoader.js';

import * as MODELS from './models.js';
import * as GAME from './game.js'

export let go_left, go_right, start;
let checkAnimating;  //boolean: if true, it means that there is already a movement in progress
                    // so no other movement are allowed to start. Therefore even if you click 
                    // another arrow, no movement will start. You have to wait the first movement
                    // to end in order to click again.
export let hero_position = {x: 0, y: 100, z: 0};
export let dead_baby = false;
export let coin_collision = false;
export let baby_score = 0;

let body;
let upperArm_L, upperArm_R, lowerArm_L, lowerArm_R;
let upperLeg_L, upperLeg_R;

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

function create_man(scene) {

    const gltfLoader = new GLTFLoader(MODELS.loadingManager);
    gltfLoader.load('assets/models/level3/girl/scene.gltf', (gltf) => {
      const root = gltf.scene;
       // to apply texture
      root.traverse ( ( o ) => {
        if (o.isMesh ) {
          o.material.skinning = true;
          o.receiveShadow = true
          o.castShadow = true;
        }
      });  
      
      const r = 100;
      const theta = 0;
      const y = r*Math.cos(theta+0.02);
      const z = r*Math.sin(theta);
      
      root.scale.set(1.0, 1.0, 1.0);
      root.position.set(0, y, z);  
      root.rotation.y = Math.PI;
      root.name = "Girl";

      // Print the hierarchical model of the main character 
      //console.log(dumpObject(root).join('\n'));

      scene.add(root);

      body = root.getObjectByName('RootNode');

      upperLeg_L = root.getObjectByName('thighL_016');
      upperLeg_R = root.getObjectByName('thighR_021');

      upperArm_L = root.getObjectByName('upper_armL_07');
      lowerArm_L = root.getObjectByName('forearmL_08');      
      upperArm_R = root.getObjectByName('upper_armR_011');
      lowerArm_R = root.getObjectByName('forearmR_012');

      // initialization of the orientations of the arms
      upperArm_L.rotation.y = 1;
      lowerArm_L.rotation.x = 0; // (> 0 if it goes forward)
      upperArm_R.rotation.y = -1;
      lowerArm_R.rotation.x = 0;

      // the girl start already sat down
      body.position.y -= 1;
      upperLeg_L.rotation.x = 1;
      upperLeg_R.rotation.x = 1;
    });
}


let mov, mov_sx, mov_dx; // needed to clean the interval when ypu want to stop the movement

function animate(hModel){
  if(!checkAnimating && GAME.permessoPerMuoversi) {
    if(go_left && start) {
      baby_score += 1;
      document.getElementById("score").innerHTML = "score: " + baby_score;
      checkAnimating = true;
      mov_sx = setInterval(function(){slide_left(hModel, mov_sx)}, 20);
      hero_position.x -= 2.2;
      check_coins_collisions(-1);
    } else if(go_right && start) {
      baby_score += 1;
      document.getElementById("score").innerHTML = "score: " + baby_score;
      checkAnimating = true;
      mov_dx = setInterval(function(){slide_right(hModel, mov_dx)}, 20);
      hero_position.x += 2.2;
      check_coins_collisions(1);
    } else if(start && !go_left && !go_right) {
      baby_score += 1;
      document.getElementById("score").innerHTML = "score: " + baby_score;
      mov = setInterval(function(){go_straight(hModel)}, 75);
      check_coins_collisions(0);
    }
  }

}

function go_straight(hModel){
  check_coins_collisions(0);
  hModel.rotation.x += 0.008865;
}


let lenght_lateral_shift = 0;

function slide_left(hModel, mov_sx){

  hModel.rotation.x += 0.002955; // 0.008865;

  if(lenght_lateral_shift < 8){
    body.rotation.z -= (0.1*4/12);
    body.position.x += 0.4*4/12;
    upperArm_L.rotation.z += 0.2;
    upperArm_R.rotation.z -= 0.1;
    lenght_lateral_shift += 1;
  } else if (lenght_lateral_shift < 12) {
    body.rotation.z += 0.1*2/12;
    body.position.x += 0.4*2/12;
    upperArm_L.rotation.z -= 0.4;
    upperArm_R.rotation.z += 0.2;
    lenght_lateral_shift += 1;
  } else {
    body.rotation.z = 0;
    go_left = false;
    lenght_lateral_shift = 0;
    clearInterval(mov_sx);
    checkAnimating = false;
  }
}


function slide_right(hModel, mov_dx){

  hModel.rotation.x += 0.002955;

  if(lenght_lateral_shift < 8){
    body.rotation.z += (0.1*4/12);
    body.position.x -= 0.4*4/12;
    upperArm_L.rotation.z += 0.1;
    upperArm_R.rotation.z -= 0.2;
    lenght_lateral_shift += 1;
  } else if (lenght_lateral_shift < 12) {
    body.rotation.z -= 0.1*2/12;
    body.position.x -= 0.4*2/12;
    upperArm_L.rotation.z -= 0.2;
    upperArm_R.rotation.z += 0.4;
    lenght_lateral_shift += 1;
  } else {
    body.rotation.z = 0;
    go_right = false;
    lenght_lateral_shift = 0;
    clearInterval(mov_dx);
    checkAnimating = false;
  }
}




export function initialize_man(scene, hModel){
    
    create_man(scene); //, body, hip, spine, legL, legR, armL, armR

    //// EVENT LISTENER MAIN CHARACTER  /////
    go_left=false;
    go_right=false;
    checkAnimating=false;
    window.addEventListener("keydown", letsMove, false);
    start = false;

    function letsMove(event) {
      if(GAME.permessoPerMuoversi){
        if((event.keyCode === 37|| event.keyCode === 65) && body.position.x < 22 && start) {
          check_coins_collisions(1);
          if(!checkAnimating){
            go_left = true;
            animate(hModel); 
          }
        }
        else if ((event.keyCode === 39 || event.keyCode === 68) && body.position.x > -22 && start) {
          check_coins_collisions(-1);
          if(!checkAnimating){
            go_right = true;
            animate(hModel); 
          }
        }
        else if ((event.keyCode === 38 || event.keyCode === 87) && !start){
          check_coins_collisions(0);
          start = true;
          animate(hModel); 
        }
      }
    }
}

export function check_collisions_around(direction) {
  coin_collision = false;
  let pos = new THREE.Vector3();

  MODELS.smallObstaclesCollision.forEach(function (element, index) {
    let smallObs = MODELS.smallObstaclesCollision[index];
    pos.setFromMatrixPosition(smallObs.matrixWorld);

    let distx = 4;
    if (smallObs.name == 'cauldron') distx = 3;
    else if (smallObs.name == 'lampPost' || smallObs.name == 'treeA' || smallObs.name == 'treeB' || smallObs.name == 'treeC' || 
    smallObs.name == 'treeD' || smallObs.name == 'trunk' || smallObs.name == 'trunkLong' || smallObs.name == 'cross') distx = 2;
    else if (smallObs.name == 'lollipopA' || smallObs.name == 'lollipopB') distx = 1;

    let distz = 4;

    if(pos.z >= -14 && pos.z < 0 && pos.y > 0 && ((direction == -1 && pos.x <= hero_position.x) || (direction == 1 && pos.x >= hero_position.x))){
      let distancex = Math.abs(hero_position.x-pos.x);
      let distancez = Math.abs(pos.z);
      if(distancez <= distz && distancex <= distx && !dead_baby){
        window.cancelAnimationFrame(GAME.id);
        GAME.sound_audio.stop();
        GAME.sound_audio2.play();
        dead_baby = true;
      }
    }

    if (pos.z >= -14 && pos.z < 0 && pos.y > 0 && direction == 0 && hero_position.x < pos.x + distx && hero_position.x > pos.x - distx) {
      let distancez = Math.abs(pos.z);
      if(distancez <= 2 && !dead_baby){
        window.cancelAnimationFrame(GAME.id);
        GAME.sound_audio.stop();
        GAME.sound_audio2.play();
        dead_baby = true;
      }
    }
  });

  MODELS.bigObstaclesCollision.forEach(function (element, index) {
    let bigObs = MODELS.bigObstaclesCollision[index];
    pos.setFromMatrixPosition(bigObs.matrixWorld);

    let distx = 2;
    if (bigObs.name == 'gravestone') distx = 3.5;

    let distz = 3;
    if (bigObs.name == 'gravestone') distz = 4;

    if(pos.z >= -14 && pos.z < 0 && pos.y > 0 && ((direction == -1 && pos.x <= hero_position.x) || (direction == 1 && pos.x >= hero_position.x))){
      let distancex = Math.abs(hero_position.x-pos.x);
      let distancez = Math.abs(pos.z);
      if(distancex <= distx && distancez <= distz && !dead_baby){
        window.cancelAnimationFrame(GAME.id);
        GAME.sound_audio.stop();
        GAME.sound_audio2.play();
        dead_baby = true;
      }
    }

    if (bigObs.name == 'gravestone') distx = 2;

    if (pos.z >= -14 && pos.z < 0 && pos.y > 0 && direction == 0 && hero_position.x < pos.x + distx && hero_position.x > pos.x - distx) {
      let distance = Math.abs(hero_position.x-pos.x) + Math.abs(pos.z);
      if(distance <= 1.5 && !dead_baby){
        window.cancelAnimationFrame(GAME.id);
        GAME.sound_audio.stop();
        GAME.sound_audio2.play();
        dead_baby = true;
      }
    }
  });
}

export function check_coins_collisions(direction) {
  coin_collision = false;
  let pos = new THREE.Vector3();


  MODELS.coinsCollisions.forEach(function (element, index) {
    let coin = MODELS.coinsCollisions[index];
    pos.setFromMatrixPosition(coin.matrixWorld);

    let distx = 4;

    if(pos.z >= -20 && pos.z < 3 && pos.y > 0 && ((direction == -1 && pos.x <= hero_position.x) || (direction == 1 && pos.x >= hero_position.x))){
      let distance = Math.sqrt(Math.pow(hero_position.x-pos.x, 2) + Math.pow(pos.z, 2));
      
      if(coin.visible && distance <= 4 && !dead_baby){
        GAME.sound_audio4.play();
        coin.visible = false;
        coin_collision = true;
        baby_score += 10;
        document.getElementById("score").innerHTML = "score: " + baby_score;
      }
    }

    if (pos.z >= -14 && pos.z < 3 && pos.y > 0 && direction == 0 && hero_position.x < pos.x + distx && hero_position.x > pos.x - distx) {
      let distancez = Math.abs(pos.z);
      
      if(coin.visible && distancez <= 5 && !dead_baby){
        GAME.sound_audio4.play();
        coin.visible = false;
        coin_collision = true;
        baby_score += 10;
        document.getElementById("score").innerHTML = "score: " + baby_score;
      }
    }
  });
}